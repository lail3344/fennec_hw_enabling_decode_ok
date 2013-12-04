/*
 *  Copyright (c) 2012 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 *
 *  WEBRTC VP8 wrapper interface
 */

#ifndef WEBRTC_MODULES_VIDEO_CODING_CODECS_MEDIACODEC_INCLUDE_MEDIACODEC_H_
#define WEBRTC_MODULES_VIDEO_CODING_CODECS_MEDIACODEC_INCLUDE_MEDIACODEC_H_

#include "webrtc/modules/video_coding/codecs/interface/video_codec_interface.h"

namespace webrtc {

class MediaCodecEncoder : public VideoEncoder {
 public:
  static MediaCodecEncoder* Create();

  virtual ~MediaCodecEncoder() {};
};  // end of MediaCodecEncoder class


class MediaCodecDecoder : public VideoDecoder {
 public:
  static MediaCodecDecoder* Create();

  virtual ~MediaCodecDecoder() {};
};  // end of MediaCodecDecoder class
}  // namespace webrtc

#endif // WEBRTC_MODULES_VIDEO_CODING_CODECS_MEDIACODEC_INCLUDE_MEDIACODEC_H_
