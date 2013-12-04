/*
 *  Copyright (c) 2013 Intel Corporation. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef WEBRTC_MODULES_VIDEO_CODING_CODECS_OMX_SOURCE_OMX_DATA_DUMP_H_
#define WEBRTC_MODULES_VIDEO_CODING_CODECS_OMX_SOURCE_OMX_DATA_DUMP_H_

#include "webrtc/modules/video_coding/codecs/interface/video_codec_interface.h"
#include "webrtc/common_video/interface/i420_video_frame.h"
#include "webrtc/system_wrappers/interface/trace.h"

#define DUMP_ENCODER_OUTPUT
#define DUMP_DECODER_INPUT
#define DUMP_ENCODER_INPUT  // not supported currently
#define DUMP_DECODER_OUTPUT // not supported currently

#if defined(DUMP_ENCODER_INPUT) || defined(DUMP_DECODER_INPUT) || defined(DUMP_ENCODER_OUTPUT) || defined(DUMP_DECODER_OUTPUT)
#define DUMP_OMX_COMP_DATA
#endif

#if defined(DUMP_OMX_COMP_DATA)

#define DUMP_ENCODER_INPUT_FILE         "/sdcard/omx_enc_in.i420"
#define DUMP_VP8_ENCODER_OUTPUT_FILE    "/sdcard/omx_enc_out.ivf"
#define DUMP_AVC_ENCODER_OUTPUT_FILE    "/sdcard/omx_enc_out.h264"
#define DUMP_VP8_DECODER_INPUT_FILE     "/sdcard/omx_dec_in.ivf"
#define DUMP_AVC_DECODER_INPUT_FILE     "/sdcard/omx_dec_in.h264"
#define DUMP_DECODER_OUTPUT_FILE        "/sdcard/omx_dec_out.i420"

namespace webrtc {

class OMXDataDump {
public:
  OMXDataDump();
  ~OMXDataDump();
  void Init(const char * compName);
  void Deinit();
  void DumpInput(void* pBufferHdr, int len, uint32_t timestamp);
  void DumpOutput(void* pBufferHdr, int len, uint32_t timestamp);
  int PrintI420VideoFrame(const webrtc::I420VideoFrame& frame);
  int PrintI420VideoFrame_2(const webrtc::I420VideoFrame* frame);

private:
  void WriteVP8(void* pBufferHdr, int len, uint32_t timestamp, int frameIdx, FILE *pFile);
  void WriteAVC(void* pBufferHdr, int len, int frameIdx, FILE *pFile);
  void WriteNV12(void* pBufferHdr, int len, int frameIdx, FILE *pFile);
//  void WriteI420(void* pBufferHdr, int len, int frameIdx, FILE *pFile);


  enum OMXCodecType {
    CODEC_VP8,
    CODEC_AVC,
  };

  FILE         *_pIn;
  FILE         *_pOut;
  int  _codecType;
  bool          _isEncoder;
  int           _inCount;
  int           _outCount;
};

} // namespace webrtc

#endif //DUMP_OMX_COMP_DATA

#endif // WEBRTC_MODULES_VIDEO_CODING_CODECS_OMX_SOURCE_OMX_DATA_DUMP_H_
