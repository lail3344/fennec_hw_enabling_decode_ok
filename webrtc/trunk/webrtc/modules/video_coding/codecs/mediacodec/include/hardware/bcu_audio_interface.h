/*****************************************************************************
 * Copyright (C) 2013 The Android Open Source Project
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
 *****************************************************************************/
#ifndef BCU_AUDIO_INTERFACE_H
#define BCU_AUDIO_INTERFACE_H

#if defined(__cplusplus)
extern "C" {
#endif

/* Macro definition of audio_io_handle_t value for BCU */
#define AUDIO_IO_HANDLE_BCU       0

/* Macros for the various Audio throttling levels */
#define AUDIO_THROTTLE_LEVEL_0    0
#define AUDIO_THROTTLE_LEVEL_1    1
#define AUDIO_THROTTLE_LEVEL_2    2
#define AUDIO_THROTTLE_LEVEL_3    3

/**
 * Interface methods b/w platform specific BCU HAL and Audio HAL interface, to
 * ge/set the audio throttling parameters values.
 */
struct audio_throttle {
    int (*audioSetThrottle)(uint8_t level);
    int (*audioGetThrottle)();
};

/**
 * This is an interface function between the Audio HAL interface and platform
 * specific HAL. Platform specific HAL should use this function during the
 * initialization, to get the audio interface throttling info for get/set audio
 * throttling.
 */
extern void get_bcu_audioif_info(struct audio_throttle **audioif_info);

#if defined(__cplusplus)
} /* extern "C" */
#endif

#endif /* BCU_AUDIO_INTERFACE_H */
