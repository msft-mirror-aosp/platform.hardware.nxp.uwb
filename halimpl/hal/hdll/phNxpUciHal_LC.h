/*
 * Copyright 2012-2020, 2023 NXP
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

#ifndef _PHNXPUCIHAL_LC_H_
#define _PHNXPUCIHAL_LC_H_

#include <phNxpUciHal.h>

/* Macros for parsing GetDeviceInfo response */
#define UWB_INDEX_TO_RETRIEVE_PARAMS 0x08
#define UWBS_LIFECYCLE 0x06
#define UWBS_LIFECYCLE_LENGTH 0x04

/* LC Rotation data structure */
typedef struct {
  pthread_attr_t attr_thread;
  pthread_t lcfwdl_tread;      /* lcfwdl thread handle */
  uint8_t lcfwdl_thread_running;
  /* Rx data */
  uint8_t rcv_data[100];
  uint16_t rcv_data_len;
  /* To check LC rotation process*/
  uint8_t isLcRotationOngoing;
  /* UWBs device lifecycle */
  uint32_t uwbsDeviceLc;
  /* To check platformId */
  bool isPlatformIdSet;
  /* Platform ID */
  uint8_t uwbsPlatformId[NXP_MAX_CONFIG_STRING_LEN];
}phNxpUciHal_lcfwdl_Control_t;

#define UWBS_LC_DEVELOPMENT_MODE  0xC5C5C5C5
#define UWBS_LC_CUSTOMER_MODE     0xA5A5A5A5
#define UWBS_LC_PROTECTED_MODE    0x55555555

extern phNxpUciHal_lcfwdl_Control_t nxpucihal_lcfwdl_ctrl;

extern int phNxpUciHal_fw_lcrotation();
tHAL_UWB_STATUS phNxpUciHal_start_lcfwdl_thread();
uint32_t phNxpUciHal_parseUWBSLifecycle(uint8_t * p_rx_data , uint16_t rx_data_len);
void phNxpUciHal_parsePlatformId(uint8_t * p_rx_data , uint16_t rx_data_len);
tHAL_UWB_STATUS phNxpUciHal_getPlatformId();
tHAL_UWB_STATUS phNxpUciHal_setPlatformId();
tHAL_UWB_STATUS phNxpUciHal_setSecureConfig();

#endif
