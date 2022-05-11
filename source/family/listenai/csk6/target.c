/**
 * @file    target.c
 * @brief   Target information for the CSK6
 *
 * DAPLink Interface Firmware
 * Copyright (c) 2022 Anhui LISTENAI Co., Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "target_config.h"

// target information
target_cfg_t target_device = {
    .version                        = kTargetConfigVersion,
    .flash_regions[0].start         = 0x18000000,
    .flash_regions[0].end           = 0x18800000,
    .flash_regions[0].flags         = kRegionIsDefault,
    .ram_regions[0].start           = 0x00080000,
    .ram_regions[0].end             = 0x000d0000,
    .target_vendor                  = "LISTENAI",
    .target_part_number             = "CSK6",
};
