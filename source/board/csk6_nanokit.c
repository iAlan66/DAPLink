/**
 * @file    csk6_nanokit.c
 * @brief   board information for LISTENAI CSK-6-NanoKit
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

#include "target_family.h"
#include "target_board.h"

const board_info_t g_board_info = {
    .info_version = kBoardInfoVersion,
    .family_id = kStub_SWVectReset_FamilyID,
    .board_id = "0000",
    .flags = kEnablePageErase,
    .target_cfg = &target_device,
    .daplink_url_name = "LISTENAIHTM",
    .daplink_drive_name = "LISTENAI",
    .daplink_target_url = "https://www.listenai.com/",
    .board_vendor = "LISTENAI",
    .board_name = "CSK-6-NanoKit",
};
