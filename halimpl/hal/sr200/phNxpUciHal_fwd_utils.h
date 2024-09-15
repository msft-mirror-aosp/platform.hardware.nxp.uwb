/*
 * Copyright 2022-2023 NXP
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
 */

#include <stdint.h>
#include <stdio.h>

#define __PACKED__ __attribute__((packed))

#define MWCESFW_COUNT 0x08

#define MWCESFW_A1V1_FW_OFFSET              0x00
#define MWCESFW_A1V1_RECOVERY_FW_OFFSET     0x01
#define MWCESFW_A1V2_FW_OFFSET              0x02
#define MWCESFW_A1V2_RECOVERY_FW_OFFSET     0x03
#define MWCESFW_A1V1_LC_FW_OFFSET           0x04
#define MWCESFW_A1V2_LC_FW_OFFSET           0x05

typedef struct __PACKED__ MWCESFW
{
  uint32_t layout_version;
  uint8_t fw_ver_major;
  uint8_t fw_ver_minor;
  uint8_t fw_ver_dev;
  uint8_t fw_ver_is_to;
  uint8_t fw_ver_git_sha1[32];
  uint32_t fw_artifact_number;
  uint32_t lenCESFW;
  uint8_t *pCESFW;
} MWCESFW_t;

typedef struct __PACKED__ UWBManifest
{
  uint32_t layout_version;
  uint8_t creation_date_yy;
  uint8_t creation_date_month;
  uint8_t creation_date_day;
  uint8_t creation_date_hour;
  uint8_t creation_date_minutes;
  uint8_t creation_date_seconds;
  uint8_t padding;
  uint8_t countMWCESFW;
  MWCESFW_t *mwCESFW[MWCESFW_COUNT];
} UWBManifest_t;
