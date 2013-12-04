/*
 *  Copyright (c) 2013 Intel Corporation. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "omx_data_dump.h"

#define __CLASS__ "OMXDataDump"
#define DUMP_OMX_COMP_DATA

#if defined(DUMP_OMX_COMP_DATA)
namespace webrtc {
static const char* logTag ="OMXDataDump";

OMXDataDump::OMXDataDump() : _pIn(NULL), _pOut(NULL), _isEncoder(false), _inCount(0), _outCount(0) {
}

OMXDataDump::~OMXDataDump() {
  Deinit();
}

void OMXDataDump::Init(const char * compName) {
  const char *pInName = NULL;
  const char *pOutName = NULL;

//  if(!strcmp(compName, "OMX.Intel.VideoEncoder.VP8")) {
    if (!strcmp(compName, "video/x-vnd.on2.vp8")) {
//    CSFLogDebug(logTag,  "%s, video/x-vnd.on2.vp8 ", __FUNCTION__);
	  WEBRTC_TRACE(webrtc::kTraceDebug, webrtc::kTraceVideoCoding, 9998,
               "%s, video/x-vnd.on2.vp8 ", __FUNCTION__);
    _codecType = 0;//CODEC_VP8;
    _isEncoder = true;
    pInName = DUMP_ENCODER_INPUT_FILE;
    pOutName = DUMP_VP8_ENCODER_OUTPUT_FILE;
  } else if(!strcmp(compName, "OMX.Intel.VideoEncoder.AVC")) {
    _codecType = 1;//CODEC_AVC;
    _isEncoder = true;
    pInName = DUMP_ENCODER_INPUT_FILE;
    pOutName = DUMP_AVC_ENCODER_OUTPUT_FILE;
  } else if(!strcmp(compName, "video/x-vnd.on2.vp8")) {
    _codecType = 0;//CODEC_VP8;
    _isEncoder = false;
    pInName = DUMP_VP8_DECODER_INPUT_FILE;
    pOutName = DUMP_DECODER_OUTPUT_FILE;
  } else if(!strcmp(compName, "OMX.Intel.VideoDecoder.AVC")) {
    _codecType = 1;//CODEC_AVC;
    _isEncoder = false;
    pInName = DUMP_AVC_DECODER_INPUT_FILE;
    pOutName = DUMP_DECODER_OUTPUT_FILE;
  } else {
//    CSFLogDebug(logTag,  "%s, Invalid component name %s for dumping.", __FUNCTION__, compName);
//    OMX_LOGE("Invalid component name %s for dumping.", compName);
	WEBRTC_TRACE(webrtc::kTraceDebug, webrtc::kTraceVideoCoding, 9998,
               "%s, Invalid component name %s for dumping.", __FUNCTION__, compName);
    return;
  }

  bool dumpIn = false;
  bool dumpOut = false;

  if(_isEncoder) {
#if defined(DUMP_ENCODER_INPUT)
    dumpIn = true;
#endif
#if defined(DUMP_ENCODER_OUTPUT)
    dumpOut = true;
#endif
  } else {
#if defined(DUMP_DECODER_INPUT)
    dumpIn = true;
#endif
#if defined(DUMP_DECODER_OUTPUT)
    dumpOut = true;
#endif
  }

  if(dumpIn) {
    _pIn = fopen(pInName, "wb");
    if(!_pIn) {
		WEBRTC_TRACE(webrtc::kTraceDebug, webrtc::kTraceVideoCoding, 9998,
               "OMXDataDump::%s, Failed to open file %s for input stream dump. ", __FUNCTION__, pInName);
//      CSFLogDebug(logTag,  "%s, Failed to open file %s for input stream dump.", __FUNCTION__, pInName);
      //OMX_LOGE("Failed to open file %s for input stream dump.", pInName);
    }
  }

  if(dumpOut) {
    _pOut = fopen(pOutName, "wb");
    if(!_pOut) {
		WEBRTC_TRACE(webrtc::kTraceDebug, webrtc::kTraceVideoCoding, 9998,
               "OMXDataDump::%s, Failed to open file %s for output stream dump.", __FUNCTION__, pOutName);
//      CSFLogDebug(logTag,  "%s, Failed to open file %s for output stream dump.", __FUNCTION__, pOutName);
      //OMX_LOGE("Failed to open file %s for output stream dump.", pOutName);
    }
  }
}

void OMXDataDump::Deinit() {
  WEBRTC_TRACE(webrtc::kTraceDebug, webrtc::kTraceVideoCoding, 9998,
               "OMXDataDump::%s ", __FUNCTION__);
//  CSFLogDebug(logTag,  "%s ", __FUNCTION__);
  if(_codecType == 0) {
    FILE *pFile = NULL;
    int frameCount = 0;
    if(_isEncoder) {
      pFile = _pOut;
      frameCount = _outCount;
    } else {
      pFile = _pIn;
      frameCount = _inCount;
    }

    if(pFile) {
      // update VP8 file header
      if(ftell(pFile) > 31) {
        fseek(pFile, 24, SEEK_SET);  //24-27: number of frames in file
        fwrite(&frameCount, 4, 1, pFile);
      }
    }
  }

  if(_pIn) {
    fclose(_pIn);
    _pIn = NULL;
  }

  if(_pOut) {
    fclose(_pOut);
    _pOut = NULL;
  }
}

void OMXDataDump::DumpInput(void* pBufferHdr, int len, uint32_t timestamp) {
//  CSFLogDebug(logTag,  "%s ", __FUNCTION__);
  WEBRTC_TRACE(webrtc::kTraceDebug, webrtc::kTraceVideoCoding, 9998,
               "OMXDataDump::%s ", __FUNCTION__);
  if(_pIn && len > 0) {
    if(_isEncoder) {
//      CSFLogDebug(logTag,  "%s, please use PrintI420VideoFrame instead!", __FUNCTION__);
      if (_codecType == 0) {
//        CSFLogDebug(logTag,  "%s, Input write to VP8", __FUNCTION__);
	    WEBRTC_TRACE(webrtc::kTraceDebug, webrtc::kTraceVideoCoding, 9998,
               "OMXDataDump::%s, write to VP8 ", __FUNCTION__);
        WriteVP8(pBufferHdr, len, timestamp, _inCount, _pOut);
      }
//      WriteNV12(pBufferHdr, len, _outCount, _pIn);
//      WriteI420(pBufferHdr, len, _outCount, _pIn);
    } else {
      if(_codecType == 0) {
//        CSFLogDebug(logTag,  "%s, Input write to VP8", __FUNCTION__);
        WriteVP8(pBufferHdr, len, timestamp, _inCount, _pIn);
      } else if(_codecType == 1) {
//        CSFLogDebug(logTag,  "%s, Input write to AVC", __FUNCTION__);
        WriteAVC(pBufferHdr, len, _inCount, _pIn);
      }
    }
    _inCount++;
  }
}

void OMXDataDump::DumpOutput(void* pBufferHdr, int len, uint32_t timestamp) {
//  CSFLogDebug(logTag,  "%s ", __FUNCTION__);
  WEBRTC_TRACE(webrtc::kTraceDebug, webrtc::kTraceVideoCoding, 9998,
               "OMXDataDump::%s ", __FUNCTION__);
  if(_pOut && len > 0) {
    if(!_isEncoder) {
		WEBRTC_TRACE(webrtc::kTraceDebug, webrtc::kTraceVideoCoding, 9998,
               "OMXDataDump::%s, please use PrintI420VideoFrame instead! ", __FUNCTION__);
//      CSFLogDebug(logTag,  "%s, please use PrintI420VideoFrame instead!", __FUNCTION__);
//      WriteNV12(pBufferHdr, len, _outCount, _pOut);
//      WriteI420(pBufferHdr, len, _outCount, _pIn);
    } else {
      if(_codecType == 0) {
//        CSFLogDebug(logTag,  "%s, output write to VP8", __FUNCTION__);
        WriteVP8(pBufferHdr, len, timestamp, _outCount, _pOut);
      } else if(_codecType == 1) {
//        CSFLogDebug(logTag,  "%s, output write to AVC", __FUNCTION__);
        WriteAVC(pBufferHdr, len, _outCount, _pOut);
      }
    }
    _outCount++;
  }
}

void OMXDataDump::WriteVP8(void* pBufferHdr, int len, uint32_t timestamp, int frameIdx, FILE *pFile) {
  if(frameIdx == 0) { // write IVF file header first
//    CSFLogDebug(logTag,  "%s, WriteVP8 Header", __FUNCTION__);
	WEBRTC_TRACE(webrtc::kTraceDebug, webrtc::kTraceVideoCoding, 9998,
               "OMXDataDump::%s ", __FUNCTION__);
    uint8_t ivfFileHdr[32] = {0};
    uint16_t ivfFileHdrLen = 32;
    uint16_t width = 720;
    uint16_t height = 480;
    uint32_t framerate = 15;
    uint32_t timescale = 1000;

    memcpy(&ivfFileHdr[0], "DKIF", 4);          //0-3: signature
                                                //4-5: version(must be 0)
    memcpy(&ivfFileHdr[6], &ivfFileHdrLen, 2);  //6-7: length of header = 32
    memcpy(&ivfFileHdr[8], "VP80", 4);          //8-11: FOURCC = VP80
    memcpy(&ivfFileHdr[12], &width, 2);         //12-13: width
    memcpy(&ivfFileHdr[14], &height, 2);        //14-15: height
    memcpy(&ivfFileHdr[16], &framerate, 4);     //16-19: framerate
    memcpy(&ivfFileHdr[20], &timescale, 4);     //20-23: timescale
                                                //24-27: number of frames in file (to be set at the end)
                                                //28-31: unused
    fwrite(&ivfFileHdr[0], sizeof(ivfFileHdr), 1, pFile);
  }

//  CSFLogDebug(logTag,  "%s, WriteVP8 other data", __FUNCTION__);
  WEBRTC_TRACE(webrtc::kTraceDebug, webrtc::kTraceVideoCoding, 9998,
               "OMXDataDump::%s, WriteVP8 other data", __FUNCTION__);
  uint8_t ivfFrameHdr[12] = {0};
  memcpy(&ivfFrameHdr[0], &len, 4);    //0-3: size of frame
  memcpy(&ivfFrameHdr[4], &timestamp, 8);    //4-11: timestamp
  fwrite(&ivfFrameHdr[0], sizeof(ivfFrameHdr), 1, pFile);

  fwrite(pBufferHdr, len, 1, pFile);
}

void OMXDataDump::WriteAVC(void* pBufferHdr, int len, int frameIdx, FILE *pFile) {
  fwrite(pBufferHdr, len, 1, pFile);
}

void OMXDataDump::WriteNV12(void* pBufferHdr, int len, int frameIdx, FILE *pFile) {
  fwrite(pBufferHdr, len, 1, pFile);
}

int OMXDataDump::PrintI420VideoFrame(const webrtc::I420VideoFrame& frame) {
  WEBRTC_TRACE(webrtc::kTraceDebug, webrtc::kTraceVideoCoding, 9998,
               "OMXDataDump::%s ", __FUNCTION__);
//  CSFLogDebug(logTag,  "%s, frame.width = %d, frame.height = %d", __FUNCTION__, frame.width(), frame.height());
  if (_pIn == NULL) {
//    CSFLogError(logTag,  "%s, _pIn is NULL!!! ", __FUNCTION__);
    return -1;
  }
  if (frame.IsZeroSize()) {
//    CSFLogDebug(logTag,  "%s frame.IsZeroSize", __FUNCTION__);
    return -1;
  }
//  CSFLogDebug(logTag,  "%s frame.width() = %d, frame.height() = %d ", __FUNCTION__, frame.width(), frame.height());
  for (int planeNum = 0; planeNum < 3; ++planeNum) {
    int width = (planeNum ? (frame.width() + 1) / 2 : frame.width());
    int height = (planeNum ? (frame.height() + 1) / 2 : frame.height());
    PlaneType plane_type = static_cast<webrtc::PlaneType>(planeNum);
    const uint8_t* plane_buffer = frame.buffer(plane_type);
    for (int y = 0; y < height; y++) { 
     if (fwrite(plane_buffer, 1, width, _pIn) !=
         static_cast<unsigned int>(width)) {
       return -1;
       }
       plane_buffer += frame.stride(plane_type);
    }
 }
 return 0;
}

int OMXDataDump::PrintI420VideoFrame_2(const webrtc::I420VideoFrame* frame) {
  WEBRTC_TRACE(webrtc::kTraceDebug, webrtc::kTraceVideoCoding, 9998,
               "OMXDataDump::%s ", __FUNCTION__);
//  CSFLogDebug(logTag,  "%s, frame.width = %d, frame.height = %d", __FUNCTION__, frame.width(), frame.height());
	WEBRTC_TRACE(webrtc::kTraceDebug, webrtc::kTraceVideoCoding, 9998,
               "OMXDataDump::%s, frame.width = %d, frame.height = %d", __FUNCTION__, frame->width(), frame->height());
  if (_pIn == NULL) {
//    CSFLogError(logTag,  "%s, _pIn is NULL!!! ", __FUNCTION__);
    return -1;
  }
  if (frame->IsZeroSize()) {
//    CSFLogDebug(logTag,  "%s frame.IsZeroSize", __FUNCTION__);
    return -1;
  }
//  CSFLogDebug(logTag,  "%s frame.width() = %d, frame.height() = %d ", __FUNCTION__, frame.width(), frame.height());
  for (int planeNum = 0; planeNum < 3; ++planeNum) {
    int width = (planeNum ? (frame->width() + 1) / 2 : frame->width());
    int height = (planeNum ? (frame->height() + 1) / 2 : frame->height());
    PlaneType plane_type = static_cast<webrtc::PlaneType>(planeNum);
    const uint8_t* plane_buffer = frame->buffer(plane_type);
    for (int y = 0; y < height; y++) { 
     if (fwrite(plane_buffer, 1, width, _pIn) !=
         static_cast<unsigned int>(width)) {
       return -1;
       }
       plane_buffer += frame->stride(plane_type);
    }
 }
 return 0;
}

} // namespace webrtc

#endif //DUMP_OMX_COMP_DATA
