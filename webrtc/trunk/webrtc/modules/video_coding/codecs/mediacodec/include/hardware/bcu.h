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

#ifndef ANDROID_INCLUDE_HARDWARE_BCU_H
#define ANDROID_INCLUDE_HARDWARE_BCU_H

#include <stdint.h>
#include <sys/cdefs.h>
#include <sys/types.h>

#include <hardware/hardware.h>

__BEGIN_DECLS

#define BCU_MODULE_API_VERSION_0_1  HARDWARE_MODULE_API_VERSION(0, 1)
#define BCU_MODULE_API_VERSION_0_2  HARDWARE_MODULE_API_VERSION(0, 2)

/**
 * The id of this module
 */
#define BCU_HARDWARE_MODULE_ID "bcu"

/**
 * BCU subsystem identifiers to get/set throttle level
 */
enum sub_system {
    BCU_SUBSYS_AUDIO = 0x01,
    BCU_SUBSYS_CAMERA = 0x02,
    BCU_SUBSYS_DISPLAY = 0x03,
    BCU_SUBSYS_OTG = 0x04,
    BCU_SUBSYS_VIBRA = 0x05,
};

/**
 * Every hardware module must have a data structure named HAL_MODULE_INFO_SYM
 * and the fields of this data structure must begin with hw_module_t
 * followed by module specific information.
 */
typedef struct bcu_module {
    struct hw_module_t common;

    /**
     * (*init)() performs initial required bcu and other subsystem setup
     * actions which should be called by CMS before get/set throttling
     */
    void (*init)(struct bcu_module *module);

    /**
     * (*setThrottleLevel) performs set throttle for various subsystems,
     * which is the interface between CMS and BCU generic HAL.
     */
    int (*setThrottleLevel)(enum sub_system subsys, uint8_t level);

    /**
     * (*getThrottleLevel) performs get throttle level of various
     * subsystems, which is the interface between CMS and BCU generic HAL.
     */
    int (*getThrottleLevel)(enum sub_system subsys, uint8_t *level);

} bcu_module_t;

/**
 * Common interface call structure between the Generic BCU HAL and Platform
 * specific BCU HAL to set/get the Throttle levels for Audio, Display, Camera
 * Otg and Vibra.
 */
struct bcu_throttle {
    int (*setAudioThrottle)(uint8_t level);
    int (*setCameraThrottle)(uint8_t level);
    int (*setDisplayThrottle)(uint8_t level);
    int (*setOtgThrottle)(uint8_t level);
    int (*setVibraThrottle)(uint8_t level);
    int (*getAudioThrottle)(uint8_t *level);
    int (*getCameraThrottle)(uint8_t *level);
    int (*getDisplayThrottle)(uint8_t *level);
    int (*getOtgThrottle)(uint8_t *level);
    int (*getVibraThrottle)(uint8_t *level);
};

/**
 * This is an interface function between the generic HAL and platform specific
 * HAL. Generic HAL should use this function during the initialization, to get
 * the platform specific layer throttling interfaces/info.
 */
extern void get_throttle_info(struct bcu_throttle **throttle_info);

__END_DECLS

#endif /* ANDROID_INCLUDE_HARDWARE_BCU_H */
