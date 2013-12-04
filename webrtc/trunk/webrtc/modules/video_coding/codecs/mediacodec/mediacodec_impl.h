/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/. */

// Class templates copied from WebRTC:
/*
 *  Copyright (c) 2012 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef MediaCodec_Impl_h__
#define MediaCodec_Impl_h__

#include <queue>

#include "omx_data_dump.h"
#include "webrtc/modules/video_coding/codecs/mediacodec/include/mediacodec.h"
#include "webrtc/modules/video_coding/main/source/internal_defines.h"
#include "webrtc/system_wrappers/interface/tick_util.h"
#include "webrtc/modules/video_coding/codecs/interface/video_codec_interface.h"
#include "webrtc/system_wrappers/interface/trace.h"

namespace webrtc {

struct EncodedFrame {
  uint32_t width_;
  uint32_t height_;
  uint32_t timestamp_;
  uint64_t decode_timestamp_;
};

class MediaCodecEncoderImpl : public MediaCodecEncoder {
 public:
  MediaCodecEncoderImpl();

  virtual ~MediaCodecEncoderImpl();

  // Implement VideoEncoder interface.
  virtual int32_t InitEncode(const VideoCodec* codecSettings,
                                   int32_t numberOfCores,
                                   uint32_t maxPayloadSize);

  virtual int32_t Encode(const I420VideoFrame& inputImage,
      const CodecSpecificInfo* codecSpecificInfo,
      const std::vector<VideoFrameType>* frame_types);

  virtual int32_t RegisterEncodeCompleteCallback(
      webrtc::EncodedImageCallback* callback);

  virtual int32_t Release();

  virtual int32_t SetChannelParameters(uint32_t packetLoss,
                                             int rtt);

  virtual int32_t SetRates(uint32_t newBitRate,
                                 uint32_t frameRate);

 private:
//  void EmitFrames();
//  void EmitFrame(EncodedFrame *frame);

  std::queue<EncodedFrame> frames_;
  size_t max_payload_size_;
  uint32_t timestamp_;
  webrtc::EncodedImage encoded_image_;
  webrtc::EncodedImageCallback* callback_;
//  mozilla::Mutex mutex_;

  void* encoder_;
  OMXDataDump *dataEncoderDump_;
};


class MediaCodecDecoderImpl : public MediaCodecDecoder {
 public:
  MediaCodecDecoderImpl();

  virtual ~MediaCodecDecoderImpl();

  // Implement VideoDecoder interface.
  virtual int32_t InitDecode(const VideoCodec* codecSettings,
                                   int32_t numberOfCores);
  virtual int32_t Decode(const EncodedImage& inputImage,
                               bool missingFrames,
                               const RTPFragmentationHeader* fragmentation,
                               const CodecSpecificInfo*
                               codecSpecificInfo = NULL,
                               int64_t renderTimeMs = -1);
  virtual int32_t RegisterDecodeCompleteCallback(
      DecodedImageCallback* callback);

  virtual int32_t Release();

  virtual int32_t Reset();

 private:
  void DecodeFrame(EncodedFrame* frame);
  void RunCallback();

  DecodedImageCallback* callback_;
  I420VideoFrame video_frame_;
  bool in_use_[2];
//  mozilla::Mutex mutex_;

//  RefPtr<VideoRenderer> renderer_;

  void* decoder_;
  
  OMXDataDump *dataDecoderDump_;

};

}

#endif // WebrtcExtVideoCodec_h__
