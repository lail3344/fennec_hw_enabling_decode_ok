/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/. */

#include <iostream>
#include "omx_data_dump.h"


#include "webrtc/video_engine/include/vie_external_codec.h"

#include "webrtc/modules/video_coding/codecs/mediacodec/mediacodec_impl.h"

#include "webrtc/modules/video_coding/codecs/mediacodec/include/gui/Surface.h"
#include "webrtc/modules/video_coding/main/source/internal_defines.h"
#include "webrtc/modules/video_coding/codecs/mediacodec/include/media/ICrypto.h"
#include "webrtc/modules/video_coding/codecs/mediacodec/include/media/stagefright/foundation/ABuffer.h"
#include "webrtc/modules/video_coding/codecs/mediacodec/include/media/stagefright/foundation/AMessage.h"
#include "webrtc/modules/video_coding/codecs/mediacodec/include/media/OMX_Component.h"
#include "webrtc/modules/video_coding/codecs/mediacodec/include/media/IOMX.h"
#include "webrtc/modules/video_coding/codecs/mediacodec/include/media/stagefright/MediaCodec.h"
#include "webrtc/modules/video_coding/codecs/mediacodec/include//media/stagefright/MediaDefs.h"
#include "webrtc/modules/video_coding/codecs/mediacodec/include/media/stagefright/MediaErrors.h"

using namespace android;

#define EXT_VIDEO_PLAYLOAD_NAME "VP8"
#define EXT_VIDEO_FRAME_WIDTH 640
#define EXT_VIDEO_FRAME_HEIGHT 480

#define EXT_ENCODE_VIDEO_FRAME_WIDTH 720
#define EXT_ENCODE_VIDEO_FRAME_HEIGHT 480

#define MEDIACODEC_VIDEO_NAME_INTEL_ENCODE "OMX.Intel.VideoEncoder.VP8"
#define MEDIACODEC_VIDEO_NAME_INTEL_DECODE "OMX.Intel.VideoDecoder.VP8" //"OMX.google.vp8.decoder" // <- NG // "OMX.qcom.video.decoder.vp8" <- NG  //"OMX.Intel.VideoDecoder.VP8" <- NG
#define MEDIACODEC_VIDEO_MIME_VP8 "video/x-vnd.on2.vp8"

#define EXT_VIDEO_MAX_FRAMERATE 30
#define DEQUEUE_BUFFER_TIMEOUT_US (100 * 1000ll) // 100ms
#define START_DEQUEUE_BUFFER_TIMEOUT_US (10 * DEQUEUE_BUFFER_TIMEOUT_US) // 1s

namespace webrtc {

MediaCodecEncoder* MediaCodecEncoder::Create() {
	WEBRTC_TRACE(webrtc::kTraceDebug, webrtc::kTraceVideoCoding, 9999,
               "MediaCodecEncoder::%s",
               __FUNCTION__);
  return new MediaCodecEncoderImpl();
}

static const char* logTag ="WebrtcExtVideoCodec";

// Link to Stagefright
class WebrtcOMX {
public:
  WebrtcOMX(const char* mime, bool encoder) {
    WEBRTC_TRACE(webrtc::kTraceDebug, webrtc::kTraceVideoCoding, 9999,
               "WebrtcOMX::%s",
               __FUNCTION__);
	started_ = false;
    looper_ = new ALooper;
    looper_->start();
    // using CreateByType API, mime should be "video/x-vnd.on2.vp8"
//    omx_ = MediaCodec::CreateByType(looper_, mime, encoder);
    
    // added by luqiang
    // using CreateByComponentName API, mime should be name (OMX.xxxxxxxx)
    // FIXME, create different class for CreateByType and CreateByComponentName
    omx_ = MediaCodec::CreateByComponentName(looper_, mime);
  }

  virtual ~WebrtcOMX() {
    looper_.clear();
    omx_.clear();
    omx_ = nullptr;
  }

  status_t Configure(const sp<AMessage>& format,
    const sp<Surface>& nativeWindow,
    const sp<ICrypto>& crypto, uint32_t flags) {
    WEBRTC_TRACE(webrtc::kTraceDebug, webrtc::kTraceVideoCoding, 9999,
               "WebrtcOMX::%s",
               __FUNCTION__);
    status_t err = omx_->configure(format, nativeWindow, crypto, flags);
	WEBRTC_TRACE(webrtc::kTraceDebug, webrtc::kTraceVideoCoding, 9999,
               "WebrtcOMX::%s, err = %d",
               __FUNCTION__, err);
	
    return err;
  }

  status_t Start() {
    WEBRTC_TRACE(webrtc::kTraceDebug, webrtc::kTraceVideoCoding, 9999,
               "WebrtcOMX::%s",
               __FUNCTION__);

    status_t err = omx_->start();
    
    WEBRTC_TRACE(webrtc::kTraceDebug, webrtc::kTraceVideoCoding, 9999,
               "WebrtcOMX::%s, omx_->start() return err = %d",
               __FUNCTION__, err);
    
    started_ = err == OK;
    
    WEBRTC_TRACE(webrtc::kTraceDebug, webrtc::kTraceVideoCoding, 9999,
               "WebrtcOMX::%s, omx_->start() started_ = %d",
               __FUNCTION__, started_);

    omx_->getInputBuffers(&input_);
    omx_->getOutputBuffers(&output_);

    return err;
  }

  status_t Stop() {
    WEBRTC_TRACE(webrtc::kTraceDebug, webrtc::kTraceVideoCoding, 9999,
               "WebrtcOMX::%s",
               __FUNCTION__);

	status_t err = omx_->stop();
    started_ = err == OK;
    return err;
  }

  friend class MediaCodecEncoderImpl;
  friend class MediaCodecDecoderImpl;

  sp<ALooper> looper_;
  sp<MediaCodec> omx_; // OMXCodec
  bool started_;
  Vector<sp<ABuffer> > input_;
  Vector<sp<ABuffer> > output_;
  // To distinguish different OMX configuration:
  // 1. HW
  // 2. SW
  // 3. HW using graphic buffer)
  int type_;

// TODO
//  NS_INLINE_DECL_THREADSAFE_REFCOUNTING(WebrtcOMX)
};

static WebrtcOMX* omxEnc = nullptr;

// Encoder.
MediaCodecEncoderImpl::MediaCodecEncoderImpl()
  : timestamp_(0),
    callback_(nullptr),
    encoder_(nullptr) {
  WEBRTC_TRACE(webrtc::kTraceDebug, webrtc::kTraceVideoCoding, 9999,
               "MediaCodecEncoderImpl::%s",
               __FUNCTION__);
               
  dataEncoderDump_ = new OMXDataDump();
  dataEncoderDump_->Init(MEDIACODEC_VIDEO_MIME_VP8);

  memset(&encoded_image_, 0, sizeof(encoded_image_));
//  EXT_LOGV("MediaCodecEncoderImpl::MediaCodecEncoderImpl %p", this);
}

static AMessage* VideoCodecSettings2AMessage(
    const VideoCodec* codecSettings, const char* mime) {
  AMessage* format = new AMessage;
  
  WEBRTC_TRACE(webrtc::kTraceDebug, webrtc::kTraceVideoCoding, 9999,
               "MediaCodecEncoderImpl::%s",
               __FUNCTION__);

  // added by luqiang, mime must be "video/x-vnd.on2.vp8", it is not same as codec name which set by CreateByType/CreateByComponentName
  format->setString("mime", MEDIACODEC_VIDEO_MIME_VP8);
//  format->setInt32("store-metadata-in-buffers", 0);
//  format->setInt32("prepend-sps-pps-to-idr-frames", 0);
  format->setInt32("bitrate", 200);//codecSettings->minBitrate * 1000); // kbps->bps
  format->setInt32("width", codecSettings->width);
  format->setInt32("height", codecSettings->height);
//  format->setInt32("stride", codecSettings->width);
//  format->setInt32("slice-height", codecSettings->height);
//  format->setFloat("frame-rate", (float)codecSettings->maxFramerate);
  // added by luqiang, for bypass ACodec.cpp framerate check logic on setupVideoEncoder
  format->setInt32("frame-rate", 30);
  // TODO: QCOM encoder only support this format. See
  // <B2G>/hardware/qcom/media/mm-video/vidc/venc/src/video_encoder_device.cpp:
  // venc_dev::venc_set_color_format()
  format->setInt32("color-format", OMX_COLOR_FormatYUV420SemiPlanar);
  // FIXME: get correct parameters from codecSettings?
  format->setInt32("i-frame-interval", 10); // one I-frame per sec
  
//  format->setInt32("profile", OMX_VIDEO_AVCProfileBaseline);
//  format->setInt32("level", OMX_VIDEO_AVCLevel3);
  //format->setInt32("bitrate-mode", OMX_Video_ControlRateConstant);

  return format;
}

int32_t MediaCodecEncoderImpl::InitEncode(
    const VideoCodec* codecSettings,
    int32_t numberOfCores,
    uint32_t maxPayloadSize) {
  max_payload_size_ = maxPayloadSize;
  WEBRTC_TRACE(webrtc::kTraceDebug, webrtc::kTraceVideoCoding, 9999,
               "MediaCodecEncoderImpl::%s",
               __FUNCTION__);
               
  int retVal = Release();
  if (retVal < 0) {
    return retVal;
  }

  if (omxEnc == nullptr) { // FIXME: implement proper lifecycle management
    // FIXME: use input parameters
    WEBRTC_TRACE(webrtc::kTraceDebug, webrtc::kTraceVideoCoding, 9999,
               "MediaCodecEncoderImpl::%s omxEnc == nullptr 1",
               __FUNCTION__);
    VideoCodec codec_inst;
    memset(&codec_inst, 0, sizeof(webrtc::VideoCodec));
    strncpy(codec_inst.plName, EXT_VIDEO_PLAYLOAD_NAME, 31);
    codec_inst.plType = 120;
    codec_inst.width = EXT_ENCODE_VIDEO_FRAME_WIDTH;
    codec_inst.height = EXT_ENCODE_VIDEO_FRAME_HEIGHT;

    codec_inst.maxBitrate = codecSettings->maxBitrate;
    codec_inst.minBitrate = codecSettings->minBitrate;

    codec_inst.maxFramerate = codecSettings->maxFramerate;

    sp<AMessage> conf = VideoCodecSettings2AMessage(&codec_inst, MEDIACODEC_VIDEO_MIME_VP8);
    
    omxEnc = new WebrtcOMX(MEDIACODEC_VIDEO_MIME_VP8, true /* encoder */);
    
    status_t err = omxEnc->Configure(conf, nullptr, nullptr, MediaCodec::CONFIGURE_FLAG_ENCODE);
    
    WEBRTC_TRACE(webrtc::kTraceDebug, webrtc::kTraceVideoCoding, 9999,
               "MediaCodecEncoderImpl::%s omxEnc == nullptr 2, err = %d",
               __FUNCTION__, err);
               
    if (err != OK) {
		omxEnc = nullptr;
		return WEBRTC_VIDEO_CODEC_ERROR;
	}
  }
  omxEnc->started_ = false;
  encoder_ = omxEnc;

  WEBRTC_TRACE(webrtc::kTraceDebug, webrtc::kTraceVideoCoding, 9999,
               "MediaCodecEncoderImpl::%s 2",
               __FUNCTION__);
  // TODO: eliminate extra pixel copy & color conversion
  size_t size = EXT_ENCODE_VIDEO_FRAME_WIDTH * EXT_ENCODE_VIDEO_FRAME_HEIGHT * 3 / 2;
  if (encoded_image_._size < size) {
    if (encoded_image_._buffer) {
      delete [] encoded_image_._buffer;
    }
    encoded_image_._buffer = new uint8_t[size];
    encoded_image_._size = size;
  }
  WEBRTC_TRACE(webrtc::kTraceDebug, webrtc::kTraceVideoCoding, 9999,
               "MediaCodecEncoderImpl::%s 3",
               __FUNCTION__);
  return WEBRTC_VIDEO_CODEC_OK;
}

int32_t MediaCodecEncoderImpl::Encode(
    const webrtc::I420VideoFrame& inputImage,
    const webrtc::CodecSpecificInfo* codecSpecificInfo,
    const std::vector<webrtc::VideoFrameType>* frame_types) {
  WEBRTC_TRACE(webrtc::kTraceDebug, webrtc::kTraceVideoCoding, 9999,
               "MediaCodecEncoderImpl::%s",
               __FUNCTION__);
  if (encoder_ == nullptr) {
    return WEBRTC_VIDEO_CODEC_ERROR;
  }

//  uint32_t time = PR_IntervalNow();
  WebrtcOMX* encoder = (reinterpret_cast<WebrtcOMX*>(encoder_));
  if (!encoder->started_) {
    status_t err = encoder->Start();
    WEBRTC_TRACE(webrtc::kTraceDebug, webrtc::kTraceVideoCoding, 9999,
               "MediaCodecEncoderImpl::%s encoder->started, err = %d",
               __FUNCTION__, err);
//    time = PR_IntervalNow();
  }
  // TODO: eliminate extra pixel copy & color conversion
  size_t sizeY = inputImage.allocated_size(webrtc::kYPlane);
  size_t sizeUV = inputImage.allocated_size(webrtc::kUPlane);
  const uint8_t* u = inputImage.buffer(webrtc::kUPlane);
  const uint8_t* v = inputImage.buffer(webrtc::kVPlane);
  size_t size = sizeY + 2 * sizeUV;
  
  // dump P420 YUV, steal from webrtc/common_video/libyuv/webrtc_libyuv.cc
  dataEncoderDump_->PrintI420VideoFrame(inputImage);

  sp<MediaCodec> omx = encoder->omx_;
  size_t index = 0;
  status_t err = omx->dequeueInputBuffer(&index, DEQUEUE_BUFFER_TIMEOUT_US);
  if (err != OK) {
    WEBRTC_TRACE(webrtc::kTraceDebug, webrtc::kTraceVideoCoding, 9999,
               "MediaCodecEncoderImpl::%s, dequeue err: %d",
               __FUNCTION__, err);
    return WEBRTC_VIDEO_CODEC_ERROR;
  }

//  CSFLogDebug(logTag,  "%s MediaCodecEncoderImpl::Encode() dequeue OMX input buffer took %u ms", __FUNCTION__, PR_IntervalToMilliseconds(PR_IntervalNow()-time));

  //time = PR_IntervalNow();

  const sp<ABuffer>& omxIn = encoder->input_.itemAt(index);
//  MOZ_ASSERT(omxIn->capacity() >= size);
  omxIn->setRange(0, size);
  uint8_t* dstY = omxIn->data();
  uint16_t* dstUV = reinterpret_cast<uint16_t*>(dstY + sizeY);
  memcpy(dstY, inputImage.buffer(webrtc::kYPlane), sizeY);
  for (int i = 0; i < sizeUV;
      i++, dstUV++, u++, v++) {
    *dstUV = (*v << 8) + *u;
  }

  err = omx->queueInputBuffer(index, 0, size,
    inputImage.render_time_ms() * 1000000, // ms to us
    0);
  if (err != OK) {
//    CSFLogError(logTag,  "%s MediaCodecEncoderImpl::Encode() queue OMX input buffer error:%d", __FUNCTION__, err);
	WEBRTC_TRACE(webrtc::kTraceDebug, webrtc::kTraceVideoCoding, 9999,
               "MediaCodecEncoderImpl::%s, enqueue err: %d",
               __FUNCTION__, err);
    return WEBRTC_VIDEO_CODEC_ERROR;
  }
//  CSFLogDebug(logTag,  "%s MediaCodecEncoderImpl::Encode() queue OMX input buffer took %u ms", __FUNCTION__, PR_IntervalToMilliseconds(PR_IntervalNow()-time));

//  time = PR_IntervalNow();

  timestamp_ = inputImage.timestamp();

  EncodedFrame frame;
  frame.width_ = inputImage.width();
  frame.height_ = inputImage.height();
  frame.timestamp_ = timestamp_;

//  CSFLogDebug(logTag,  "%s MediaCodecEncoderImpl::Encode() %dx%d -> %ux%u took %u ms", __FUNCTION__, inputImage.width(), inputImage.height(), frame.width_, frame.height_,
//    PR_IntervalToMilliseconds(PR_IntervalNow()-time));
	WEBRTC_TRACE(webrtc::kTraceDebug, webrtc::kTraceVideoCoding, 9999,
               "MediaCodecEncoderImpl::%s, Encode() %dx%d -> %ux%u ",
               __FUNCTION__, inputImage.width(), inputImage.height(), frame.width_, frame.height_);
//  time = PR_IntervalNow();

  encoded_image_._encodedWidth = frame.width_;
  encoded_image_._encodedHeight = frame.height_;
  encoded_image_._timeStamp = frame.timestamp_;
  encoded_image_.capture_time_ms_ = frame.timestamp_;

  size_t outOffset;
  size_t outSize;
  int64_t outTime;
  uint32_t outFlags;
  err = omx->dequeueOutputBuffer(&index, &outOffset, &outSize, &outTime,
    &outFlags, DEQUEUE_BUFFER_TIMEOUT_US);
  if (err == INFO_FORMAT_CHANGED) {
//    CSFLogDebug(logTag,  "%s MediaCodecEncoderImpl::Encode() dequeue OMX output format change", __FUNCTION__);
	WEBRTC_TRACE(webrtc::kTraceDebug, webrtc::kTraceVideoCoding, 9999,
               "MediaCodecEncoderImpl::%s, encode dequeue OMX output format change",
               __FUNCTION__);
    return WEBRTC_VIDEO_CODEC_OK;
  } else if (err == INFO_OUTPUT_BUFFERS_CHANGED) {
    err = omx->getOutputBuffers(&encoder->output_);
//    CSFLogDebug(logTag,  "%s MediaCodecEncoderImpl::Encode() dequeue OMX output buffer change", __FUNCTION__);
	WEBRTC_TRACE(webrtc::kTraceDebug, webrtc::kTraceVideoCoding, 9999,
               "MediaCodecEncoderImpl::%s, Encode() dequeue OMX output buffer change",
               __FUNCTION__);
//    MOZ_ASSERT(err == OK);
    return WEBRTC_VIDEO_CODEC_OK;
  } else if (err != OK) {
//    CSFLogDebug(logTag,  "%s MediaCodecEncoderImpl::Encode() dequeue OMX output buffer error:%d", __FUNCTION__, err);
	WEBRTC_TRACE(webrtc::kTraceDebug, webrtc::kTraceVideoCoding, 9999,
               "MediaCodecEncoderImpl::%s,Encode() dequeue OMX output buffer error:%d",
               __FUNCTION__, err);
    return WEBRTC_VIDEO_CODEC_ERROR;
  }
//  CSFLogDebug(logTag,  "%s MediaCodecEncoderImpl::Encode() dequeue OMX output buffer err:%d len:%u time:%lld flags:0x%08x took %u ms", __FUNCTION__, err, outSize, outTime, outFlags, PR_IntervalToMilliseconds(PR_IntervalNow()-time));
  WEBRTC_TRACE(webrtc::kTraceDebug, webrtc::kTraceVideoCoding, 9999,
               "MediaCodecEncoderImpl::%s,Encode() dequeue OMX output buffer err:%d len:%u time:%lld flags:0x%08x ",
               __FUNCTION__, err, outSize, outTime, outFlags);

  sp<ABuffer> omxOut = encoder->output_.itemAt(index);
  encoded_image_._length = omxOut->size();
  uint8_t* data = omxOut->data();
  memcpy(encoded_image_._buffer, data, encoded_image_._length);
  omx->releaseOutputBuffer(index);

//  CSFLogDebug(logTag,  "%s MediaCodecEncoderImpl::Encode() outFlags:%d ", __FUNCTION__, outFlags);
  WEBRTC_TRACE(webrtc::kTraceDebug, webrtc::kTraceVideoCoding, 9999,
               "MediaCodecEncoderImpl::%s, MediaCodecEncoderImpl::Encode() outFlags:%d",
               __FUNCTION__, outFlags);
  encoded_image_._completeFrame = true;
  encoded_image_._frameType =
    outFlags & (MediaCodec::BUFFER_FLAG_SYNCFRAME | MediaCodec::BUFFER_FLAG_CODECCONFIG)?
    webrtc::kKeyFrame:webrtc::kDeltaFrame;


//  CSFLogDebug(logTag,  "%s MediaCodecEncoderImpl::Encode() frame type:%d size:%u", __FUNCTION__, encoded_image_._frameType, encoded_image_._length);
  WEBRTC_TRACE(webrtc::kTraceDebug, webrtc::kTraceVideoCoding, 9999,
               "MediaCodecEncoderImpl::%s, Encode() frame type:%d size:%u",
               __FUNCTION__, encoded_image_._frameType, encoded_image_._length);

  // added by luqiang, for dump encoded data
  dataEncoderDump_->DumpInput(encoded_image_._buffer, encoded_image_._length, encoded_image_.capture_time_ms_);

  WEBRTC_TRACE(webrtc::kTraceDebug, webrtc::kTraceVideoCoding, 9999,
               "MediaCodecEncoderImpl::%s, Encode() frame type:%d size:%u time:%lld",
               __FUNCTION__, encoded_image_._frameType, encoded_image_._length, encoded_image_.capture_time_ms_);

  callback_->Encoded(encoded_image_, nullptr, nullptr);

  return WEBRTC_VIDEO_CODEC_OK;
}

int32_t MediaCodecEncoderImpl::RegisterEncodeCompleteCallback(
    webrtc::EncodedImageCallback* callback) {
  WEBRTC_TRACE(webrtc::kTraceDebug, webrtc::kTraceVideoCoding, 9999,
               "MediaCodecEncoderImpl::%s",
               __FUNCTION__);
  callback_ = callback;

  return WEBRTC_VIDEO_CODEC_OK;
}

int32_t MediaCodecEncoderImpl::Release() {
  WEBRTC_TRACE(webrtc::kTraceDebug, webrtc::kTraceVideoCoding, 9999,
               "MediaCodecEncoderImpl::%s",
               __FUNCTION__);
  if (encoder_) {
    // FIXME: Leak! Should implement proper lifecycle management
     WebrtcOMX* omx = static_cast<WebrtcOMX*>(encoder_);
     delete omx;
     encoder_ = nullptr;
  }

  if (encoded_image_._buffer) {
    delete [] encoded_image_._buffer;
    encoded_image_._buffer = nullptr;
    encoded_image_._size = 0;
  }

  return WEBRTC_VIDEO_CODEC_OK;
}

MediaCodecEncoderImpl::~MediaCodecEncoderImpl() {
  WEBRTC_TRACE(webrtc::kTraceDebug, webrtc::kTraceVideoCoding, 9999,
               "MediaCodecEncoderImpl::%s",
               __FUNCTION__);
  dataEncoderDump_->Deinit();
//  EXT_LOGV("MediaCodecEncoderImpl::~MediaCodecEncoderImpl %p", this);
}

// TODO
int32_t MediaCodecEncoderImpl::SetChannelParameters(uint32_t packetLoss,
                                                           int rtt) {
  WEBRTC_TRACE(webrtc::kTraceDebug, webrtc::kTraceVideoCoding, 9999,
               "MediaCodecEncoderImpl::%s",
               __FUNCTION__);
  return WEBRTC_VIDEO_CODEC_OK;
}

// TODO
int32_t MediaCodecEncoderImpl::SetRates(uint32_t newBitRate,
                                               uint32_t frameRate) {
  WEBRTC_TRACE(webrtc::kTraceDebug, webrtc::kTraceVideoCoding, 9999,
               "MediaCodecEncoderImpl::%s",
               __FUNCTION__);
  WebrtcOMX* encoder = static_cast<WebrtcOMX*>(encoder_);
  if (!encoder || !encoder->omx_.get()) {
	WEBRTC_TRACE(webrtc::kTraceDebug, webrtc::kTraceVideoCoding, 9999,
               "MediaCodecEncoderImpl::%s, WEBRTC_VIDEO_CODEC_UNINITIALIZED",
               __FUNCTION__);
    return WEBRTC_VIDEO_CODEC_UNINITIALIZED;
  }

  sp<MediaCodec> omx = encoder->omx_;
  sp<AMessage> msg = new AMessage();
  msg->setInt32("videoBitrate", newBitRate * 1000 /* kbps -> bps */);
  msg->setInt32("frame-rate", frameRate);
  omx->setParameters(msg);

  return WEBRTC_VIDEO_CODEC_OK;
}

MediaCodecDecoder* MediaCodecDecoder::Create() {
  WEBRTC_TRACE(webrtc::kTraceDebug, webrtc::kTraceVideoCoding, 9999,
               "MediaCodecDecoder::%s",
               __FUNCTION__);
  return new MediaCodecDecoderImpl();
}

// TODO: eliminate global variable after implementing prpper lifecycle management code
static WebrtcOMX* omxDec = nullptr;

// Decoder.
MediaCodecDecoderImpl::MediaCodecDecoderImpl()
    : callback_(nullptr) {
  WEBRTC_TRACE(webrtc::kTraceDebug, webrtc::kTraceVideoCoding, 9999,
               "MediaCodecDecoderImpl::%s",
               __FUNCTION__);
//  in_use_[0] = in_use_[1] = false;
//  EXT_LOGV("MediaCodecDecoderImpl::MediaCodecDecoderImpl %p", this);
  dataDecoderDump_ = new OMXDataDump();
  dataDecoderDump_->Init(MEDIACODEC_VIDEO_MIME_VP8);

}

int32_t MediaCodecDecoderImpl::InitDecode(
    const webrtc::VideoCodec* codecSettings,
    int32_t numberOfCores) {
  WEBRTC_TRACE(webrtc::kTraceDebug, webrtc::kTraceVideoCoding, 9999,
               "MediaCodecDecoderImpl::%s",
               __FUNCTION__);
//  EXT_LOGV("MediaCodecDecoderImpl::InitDecode %p", this);
  // FIXME: implement proper lifecycle management
  if (omxDec == nullptr) {
    // FIXME: use input parameters
    webrtc::VideoCodec codec_inst;
    memset(&codec_inst, 0, sizeof(webrtc::VideoCodec));
    strncpy(codec_inst.plName, EXT_VIDEO_PLAYLOAD_NAME, 31);
    codec_inst.plType = 120;
    codec_inst.width = EXT_VIDEO_FRAME_WIDTH;
    codec_inst.height = EXT_VIDEO_FRAME_HEIGHT;

    codec_inst.maxFramerate = codecSettings->maxFramerate;
    codec_inst.maxBitrate = codecSettings->maxBitrate;
    codec_inst.minBitrate = codecSettings->minBitrate;

    sp<AMessage> conf = VideoCodecSettings2AMessage(&codec_inst, MEDIACODEC_VIDEO_NAME_INTEL_DECODE);
    
    omxDec  = new WebrtcOMX(MEDIACODEC_VIDEO_NAME_INTEL_DECODE, false /* encoder */);


    omxDec->type_ = 2;//atoi(type);
    sp<Surface> nativeWindow = nullptr;

    status_t err = omxDec->Configure(conf, nullptr, nullptr, 0);
    WEBRTC_TRACE(webrtc::kTraceDebug, webrtc::kTraceVideoCoding, 9999,
               "MediaCodecDecoderImpl::%s, err = %d",
               __FUNCTION__, err);
    if (err != OK) {
		WEBRTC_TRACE(webrtc::kTraceDebug, webrtc::kTraceVideoCoding, 9999,
               "MediaCodecDecoderImpl::%s, decoder configure return err = %d",
               __FUNCTION__, err);
	}
  }
  omxDec->started_ = false;
  decoder_ = omxDec;

  return WEBRTC_VIDEO_CODEC_OK;
}


void generateVideoFrame_sw(EncodedFrame* frame, const sp<ABuffer>& decoded,
  webrtc::I420VideoFrame* videoFrame) {
  
  WEBRTC_TRACE(webrtc::kTraceDebug, webrtc::kTraceVideoCoding, 9999,
               "MediaCodecDecoderImpl::%s",
               __FUNCTION__);
  
  // FIXME: eliminate extra pixel copy/color conversion
  // QCOM HW only support OMX_QCOM_COLOR_FormatYVU420PackedSemiPlanar32m4ka
  size_t width = frame->width_;
  size_t height = frame->height_;
  size_t widthUV = width / 2;
  if (videoFrame->CreateEmptyFrame(width, height,
                                      width, widthUV, widthUV)) {
    return;
  }
  size_t roundedStride = (width + 31) & ~31;
  size_t roundedSliceHeight = (height + 31) & ~31;
  uint8_t* y = decoded->data();
  uint8_t* uv = y + (roundedStride * roundedSliceHeight);
  uint8_t* dstY = videoFrame->buffer(webrtc::kYPlane);
  uint8_t* dstU = videoFrame->buffer(webrtc::kUPlane);
  uint8_t* dstV = videoFrame->buffer(webrtc::kVPlane);
  size_t heightUV = height / 2;
  size_t padding = roundedStride - width;
  for (int i = 0; i < height; i++) {
    memcpy(dstY, y, width);
    y += roundedStride;
    dstY += width;
    if (i < heightUV) {
      for (int j = 0; j < widthUV; j++) {
		//updated by luqiang, change U,V order from VU to UV !!! 
		*dstU++ = *uv++;
        *dstV++ = *uv++;
      }
      uv += padding;
    }
  }

  videoFrame->set_timestamp(frame->timestamp_);
}

status_t feedOMXInput(WebrtcOMX* decoder, const sp<MediaCodec>& omx,
  const webrtc::EncodedImage& inputImage, int64_t* timeUs) {
  static int64_t firstTime = -1ll;
  size_t index;
  //uint32_t time = PR_IntervalNow();
  WEBRTC_TRACE(webrtc::kTraceDebug, webrtc::kTraceVideoCoding, 9999,
               "MediaCodecDecoderImpl::%s",
               __FUNCTION__);
  status_t err = omx->dequeueInputBuffer(&index, 
    firstTime < 0? START_DEQUEUE_BUFFER_TIMEOUT_US:DEQUEUE_BUFFER_TIMEOUT_US);
  if (err != OK) {
	WEBRTC_TRACE(webrtc::kTraceDebug, webrtc::kTraceVideoCoding, 9999,
               "MediaCodecDecoderImpl::%s, dequeue input buffer error:%d",
               __FUNCTION__, err);
    return err;
  }

  //time = PR_IntervalNow();

  uint32_t flags = 0;
  if (inputImage._frameType == webrtc::kKeyFrame) {
    flags |= (firstTime < 0)? MediaCodec::BUFFER_FLAG_CODECCONFIG:MediaCodec::BUFFER_FLAG_SYNCFRAME;
  }
  size_t size = inputImage._length;
  const sp<ABuffer>& omxIn = decoder->input_.itemAt(index);
//  MOZ_ASSERT(omxIn->capacity() >= size);
  omxIn->setRange(0, size);
  memcpy(omxIn->data(), inputImage._buffer, size);
  if (firstTime < 0) {
    firstTime = inputImage._timeStamp;
  }
  *timeUs = (inputImage._timeStamp - firstTime) * 10; // FIXME
  err = omx->queueInputBuffer(index, 0, size, *timeUs, flags);

	WEBRTC_TRACE(webrtc::kTraceDebug, webrtc::kTraceVideoCoding, 9999,
               "MediaCodecDecoderImpl::%s, queue input buffer len:%u flags:%u time:%lld ",
               __FUNCTION__, size, flags, *timeUs);

//  CSFLogDebug(logTag,  "%s MediaCodecDecoderImpl::Decode() queue input buffer len:%u flags:%u time:%lld took %u ms", __FUNCTION__, size, flags, *timeUs, PR_IntervalToMilliseconds(PR_IntervalNow()-time));
//  EXT_LOGV("MediaCodecDecoderImpl::Decode() queue input buffer len:%u flags:%u time:%lld took %u ms",
//    size, flags, *timeUs, PR_IntervalToMilliseconds(PR_IntervalNow()-time));

  return err;
}

status_t getOMXOutput(WebrtcOMX* decoder, const sp<MediaCodec>& omx,
  const webrtc::EncodedImage& inputImage, const int64_t timeUs,
  webrtc::I420VideoFrame* video_frame,
  webrtc::DecodedImageCallback* callback_, webrtc::OMXDataDump *dataDump_) {
  sp<ABuffer> omxOut;
  
  WEBRTC_TRACE(webrtc::kTraceDebug, webrtc::kTraceVideoCoding, 9999,
               "MediaCodecDecoderImpl::%s",
               __FUNCTION__);

  // return directly for test
//  return OK;

  EncodedFrame* frame = new EncodedFrame();
  // FIXME: inputImage.encoded_(width|height) was not set correctly
  frame->width_ = EXT_VIDEO_FRAME_WIDTH;
  frame->height_ = EXT_VIDEO_FRAME_HEIGHT;
  frame->timestamp_ = inputImage._timeStamp;
  frame->decode_timestamp_ = TickTime::MillisecondTimestamp();

//  uint32_t time = PR_IntervalNow();

  size_t index;
  size_t outOffset;
  size_t outSize;
  int64_t outTime;
  uint32_t outFlags;
  status_t err = omx->dequeueOutputBuffer(&index, &outOffset, &outSize, &outTime, &outFlags);
  if (err == INFO_FORMAT_CHANGED) {
    // TODO: handle format change
    WEBRTC_TRACE(webrtc::kTraceDebug, webrtc::kTraceVideoCoding, 9999,
               "MediaCodecDecoderImpl::%s, INFO_FORMAT_CHANGED",
               __FUNCTION__);
    goto end;
  } else if (err == INFO_OUTPUT_BUFFERS_CHANGED) {
	WEBRTC_TRACE(webrtc::kTraceDebug, webrtc::kTraceVideoCoding, 9999,
               "MediaCodecDecoderImpl::%s, dequeue OMX output buffer change:%d",
               __FUNCTION__, err);
    err = omx->getOutputBuffers(&decoder->output_);
//    MOZ_ASSERT(err == OK);
    err = INFO_OUTPUT_BUFFERS_CHANGED;
    goto end;
  } else if (err != OK) {
	WEBRTC_TRACE(webrtc::kTraceDebug, webrtc::kTraceVideoCoding, 9999,
               "MediaCodecEncoderImpl::%s, dequeue OMX output buffer error:%d",
               __FUNCTION__, err);
//    CSFLogDebug(logTag,  "%s MediaCodecDecoderImpl::Decode() dequeue OMX output buffer error:%d", __FUNCTION__, err);
//    EXT_LOGE("MediaCodecDecoderImpl::Decode() dequeue OMX output buffer error:%d", err);
    goto end;
  }
  omxOut = decoder->output_.itemAt(index);

//  CSFLogDebug(logTag,  "%s MediaCodecDecoderImpl::Decode() dequeue output buffer#%u(%p) err:%d len:%u time:%lld flags:0x%08x took %u ms", __FUNCTION__, index, omxOut.get(), err, outSize, outTime, outFlags, PR_IntervalToMilliseconds(PR_IntervalNow()-time));
  WEBRTC_TRACE(webrtc::kTraceDebug, webrtc::kTraceVideoCoding, 9999,
               "MediaCodecDecoderImpl::%s, dequeue OMX output buffer change:%d",
               __FUNCTION__, err);

//  EXT_LOGV("MediaCodecDecoderImpl::Decode() dequeue output buffer#%u(%p) err:%d len:%u time:%lld flags:0x%08x took %u ms",
//    index, omxOut.get(), err, outSize, outTime, outFlags, PR_IntervalToMilliseconds(PR_IntervalNow()-time));
//  time = PR_IntervalNow();

  if (timeUs < outTime) {
    // invalid buffer
    omx->releaseOutputBuffer(index);
  } else if (decoder->type_ == 3) {
//  CSFLogDebug(logTag,  "%s decoder->type_ == 3", __FUNCTION__);
//    int i = in_use_[0]? 1:0;
//    webrtc::I420VideoFrame* videoFrame = &video_frames_[i];
//    generateVideoFrame_hw(decoder->native_window_.get(), frame,
//      omx, index, omxOut, videoFrame, &in_use_[i]);
//    callback_->Decoded(*videoFrame);
  } else {
//    CSFLogDebug(logTag,  "%s MediaCodecDecoderImpl::Decode() generate video frame", __FUNCTION__);
    WEBRTC_TRACE(webrtc::kTraceDebug, webrtc::kTraceVideoCoding, 9999,
               "MediaCodecDecoderImpl::%s, Decode() generate video frame",
               __FUNCTION__);
//    EXT_LOGV("MediaCodecDecoderImpl::Decode() generate video frame");
    generateVideoFrame_sw(frame, omxOut, video_frame);
    
    // dump decoded i420 frame to file
    dataDump_->PrintI420VideoFrame_2(video_frame);
    
    callback_->Decoded(*video_frame);

    omx->releaseOutputBuffer(index);
  }

end:
  delete frame;

  return err;
}

int32_t MediaCodecDecoderImpl::Decode(
    const webrtc::EncodedImage& inputImage,
    bool missingFrames,
    const webrtc::RTPFragmentationHeader* fragmentation,
    const webrtc::CodecSpecificInfo* codecSpecificInfo,
    int64_t renderTimeMs) {
    
  WEBRTC_TRACE(webrtc::kTraceDebug, webrtc::kTraceVideoCoding, 9999,
               "MediaCodecDecoderImpl::%s, 1",
               __FUNCTION__);
//  EXT_LOGV("MediaCodecDecoderImpl::Decode()");

  if (inputImage._length== 0 || !inputImage._buffer) {
    return WEBRTC_VIDEO_CODEC_ERROR;
  }

//  uint32_t time = PR_IntervalNow();
  WebrtcOMX* decoder = static_cast<WebrtcOMX*>(decoder_);
  if (!decoder->started_) {
    status_t err = decoder->Start();
    WEBRTC_TRACE(webrtc::kTraceDebug, webrtc::kTraceVideoCoding, 9999,
               "MediaCodecDecoderImpl::%s, decoder->Start(), err = %d",
               __FUNCTION__, err);
//    CSFLogDebug(logTag,  "%s MediaCodecDecoderImpl::Decode() start decoder took %u ms", __FUNCTION__, PR_IntervalToMilliseconds(PR_IntervalNow()-time));
//    EXT_LOGV("MediaCodecDecoderImpl::Decode() start decoder took %u ms",
//      PR_IntervalToMilliseconds(PR_IntervalNow()-time));
  }

  sp<MediaCodec> omx = decoder->omx_;
  bool feedFrame = true;

  while (feedFrame) {
    int64_t timeUs;
    status_t err = feedOMXInput(decoder, omx, inputImage, &timeUs);
    feedFrame = (err == -EAGAIN);
    do {
      err = getOMXOutput(decoder, omx, inputImage, timeUs, &video_frame_, callback_, dataDecoderDump_);
    } while (err == INFO_OUTPUT_BUFFERS_CHANGED);
  }

//  EXT_LOGV("MediaCodecDecoderImpl::Decode() end");
//  CSFLogDebug(logTag,  "%s MediaCodecDecoderImpl::Decode() end", __FUNCTION__);
  WEBRTC_TRACE(webrtc::kTraceDebug, webrtc::kTraceVideoCoding, 9999,
               "MediaCodecDecoderImpl::%s, end",
               __FUNCTION__);
  return WEBRTC_VIDEO_CODEC_OK;
}

void MediaCodecDecoderImpl::DecodeFrame(EncodedFrame* frame) {
  WEBRTC_TRACE(webrtc::kTraceDebug, webrtc::kTraceVideoCoding, 9999,
               "MediaCodecDecoderImpl::%s",
               __FUNCTION__);
}

int32_t MediaCodecDecoderImpl::RegisterDecodeCompleteCallback(
    webrtc::DecodedImageCallback* callback) {
   WEBRTC_TRACE(webrtc::kTraceDebug, webrtc::kTraceVideoCoding, 9999,
               "MediaCodecDecoderImpl::%s",
               __FUNCTION__);
    
  callback_ = callback;

  return WEBRTC_VIDEO_CODEC_OK;
}

int32_t MediaCodecDecoderImpl::Release() {
   WEBRTC_TRACE(webrtc::kTraceDebug, webrtc::kTraceVideoCoding, 9999,
               "MediaCodecDecoderImpl::%s",
               __FUNCTION__);
  return WEBRTC_VIDEO_CODEC_OK;
}

MediaCodecDecoderImpl::~MediaCodecDecoderImpl() {
   WEBRTC_TRACE(webrtc::kTraceDebug, webrtc::kTraceVideoCoding, 9999,
               "MediaCodecDecoderImpl::%s",
               __FUNCTION__);
//  EXT_LOGV("MediaCodecDecoderImpl::~MediaCodecDecoderImpl %p", this);
}

int32_t MediaCodecDecoderImpl::Reset() {
   WEBRTC_TRACE(webrtc::kTraceDebug, webrtc::kTraceVideoCoding, 9999,
               "MediaCodecDecoderImpl::%s",
               __FUNCTION__);
  return WEBRTC_VIDEO_CODEC_OK;
}

}
