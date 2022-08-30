/**
 * @file    target_reset.c
 * @brief   Target reset for the CSK6
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

#include "swd_host.h"
#include "target_family.h"

const target_family_descriptor_t g_target_family_csk6 = {
    .family_id = kStub_SWVectReset_FamilyID,
    .default_reset_type = kSoftwareReset,
    .soft_reset_type = VECTRESET,
};

const target_family_descriptor_t *g_target_family = &g_target_family_csk6;
