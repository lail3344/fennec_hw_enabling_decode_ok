/*
 * Copyright (C) 2009 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * @@DOLBY_BANNER
 *
 * This file was modified by Dolby Laboratories, Inc. The portions of the
 * code that are surrounded by "DOLBY..." are copyrighted and
 * licensed separately, as follows:
 *
 *  (C) 2011-2012 Dolby Laboratories, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * @@DOLBY_FILE_M
 * @@DOLBY_FILE_R
 * @@DOLBY_BANNER_END
 */

#ifndef AUDIO_PLAYER_H_

#define AUDIO_PLAYER_H_

#ifdef DOLBY_UDC
#include "include/TimedEventQueue.h"
#endif // DOLBY_UDC
#include <media/MediaPlayerInterface.h>
#include <media/stagefright/MediaBuffer.h>
#include <media/stagefright/TimeSource.h>
#include <utils/threads.h>

namespace android {

class MediaSource;
class AudioTrack;
class AwesomePlayer;
#ifdef AUDIO_DUMP_ENABLE
class AudioDump;
#endif

class AudioPlayer : public TimeSource {
public:
    enum {
        REACHED_EOS,
        SEEK_COMPLETE
    };

    enum {
        ALLOW_DEEP_BUFFERING = 0x01,
        USE_OFFLOAD = 0x02,
        HAS_VIDEO   = 0x1000,
        IS_STREAMING = 0x2000

    };

    AudioPlayer(const sp<MediaPlayerBase::AudioSink> &audioSink,
                uint32_t flags = 0,
                AwesomePlayer *audioObserver = NULL);

    virtual ~AudioPlayer();

    // Caller retains ownership of "source".
    void setSource(const sp<MediaSource> &source);

    // Return time in us.
    virtual int64_t getRealTimeUs();

    status_t start(bool sourceAlreadyStarted = false);

    void pause(bool playPendingSamples = false);
    status_t resume();

    // Returns the timestamp of the last buffer played (in us).
    int64_t getMediaTimeUs();

    // Returns true iff a mapping is established, i.e. the AudioPlayer
    // has played at least one frame of audio.
    bool getMediaTimeMapping(int64_t *realtime_us, int64_t *mediatime_us);

    status_t seekTo(int64_t time_us);

    bool isSeeking();
    bool reachedEOS(status_t *finalStatus);

    status_t setPlaybackRatePermille(int32_t ratePermille);

    void notifyAudioEOS();

private:
    friend class VideoEditorAudioPlayer;
    sp<MediaSource> mSource;
    sp<AudioTrack> mAudioTrack;

    MediaBuffer *mInputBuffer;

    int mSampleRate;
    int64_t mLatencyUs;
    size_t mFrameSize;

    Mutex mLock;
    int64_t mNumFramesPlayed;
    int64_t mNumFramesPlayedSysTimeUs;

    int64_t mPositionTimeMediaUs;
    int64_t mPositionTimeRealUs;

    bool mSeeking;
    bool mReachedEOS;
    status_t mFinalStatus;
    int64_t mSeekTimeUs;

    bool mStarted;

    bool mIsFirstBuffer;
    status_t mFirstBufferResult;
    MediaBuffer *mFirstBuffer;

    sp<MediaPlayerBase::AudioSink> mAudioSink;
    AwesomePlayer *mObserver;
    int64_t mPinnedTimeUs;

    bool mPlaying;
    int64_t mStartPosUs;
    const uint32_t mCreateFlags;

   //Pointer to AudioDump object.
#ifdef AUDIO_DUMP_ENABLE
    AudioDump *mDecAudioDump;
#endif

    static void AudioCallback(int event, void *user, void *info);
    void AudioCallback(int event, void *info);

    static size_t AudioSinkCallback(
            MediaPlayerBase::AudioSink *audioSink,
            void *data, size_t size, void *me,
            MediaPlayerBase::AudioSink::cb_event_t event);

    size_t fillBuffer(void *data, size_t size);

    int64_t getRealTimeUsLocked() const;

    void reset();

#ifdef DOLBY_UDC
    void onPortSettingsChangedEvent();
    TimedEventQueue mQueue;
    bool mQueueStarted;
    sp<TimedEventQueue::Event> mPortSettingsChangedEvent;
    bool mPortSettingsChangedEventPending;
    int reOpenSink(int numChannels, int channelMask);
#endif // DOLBY_UDC
    uint32_t getNumFramesPendingPlayout() const;
    int64_t getOutputPlayPositionUs_l() const;

    bool allowDeepBuffering() const { return (mCreateFlags & ALLOW_DEEP_BUFFERING) != 0; }
    bool useOffload() const { return (mCreateFlags & USE_OFFLOAD) != 0; }

    AudioPlayer(const AudioPlayer &);
    AudioPlayer &operator=(const AudioPlayer &);
#ifdef BGM_ENABLED
    bool mAllowBackgroundPlayback;
    int mBGMAudioSessionID;
    void updateBGMoutput();
#endif
};

}  // namespace android

#endif  // AUDIO_PLAYER_H_
