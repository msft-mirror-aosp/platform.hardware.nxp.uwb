/*
 * Copyright 2012-2019, 2022-2023 NXP
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

#if 0

#include <log/log.h>
#include <phNxpLog.h>
#include <cutils/properties.h>
#include <phNxpUciHal.h>
#include <phNxpUciHal_Adaptation.h>
#include <phNxpUciHal_ext.h>
#include <phTmlUwb.h>
#include <phTmlUwb_spi.h>
#include <sys/stat.h>
#include <string.h>
#include <array>
#include "hal_nxpuwb.h"
#include "phNxpConfig.h"
#include <android-base/stringprintf.h>

#if (NXP_UWB_EXTNS == TRUE)
#include "phNxpUciHalProp.h"
#endif
#include <phNxpUciHal_LC.h>

using android::base::StringPrintf;

phNxpUciHal_lcfwdl_Control_t nxpucihal_lcfwdl_ctrl;
extern phNxpUciHal_Control_t nxpucihal_ctrl;
extern bool uwb_get_platform_id;
extern bool uwb_device_initialized;
static uint8_t Rx_buffer[UCI_MAX_DATA_LEN];

/**************** local methods used in this file only ************************/
static tHAL_UWB_STATUS phNxpUciHal_performLcRotation();
static void* phNxpUciHal_lcfwdl_thread(void* arg);

/******************************************************************************
 * Function         phNxpUciHal_lcfwdl_thread
 *
 * Description      This function is a thread handler which handles all TML and
 *                  UCI messages.
 *
 * Returns          void
 *
 ******************************************************************************/
static void* phNxpUciHal_lcfwdl_thread(void* arg) {
  phNxpUciHal_lcfwdl_Control_t* p_nxpucihal_lcfwdl_ctrl = (phNxpUciHal_lcfwdl_Control_t*)arg;
  tHAL_UWB_STATUS status = UWBSTATUS_FAILED;

  NXPLOG_UCIHAL_D("lcfwdl thread started");
  p_nxpucihal_lcfwdl_ctrl->lcfwdl_thread_running = 1;

  while (p_nxpucihal_lcfwdl_ctrl->lcfwdl_thread_running == 1) {

    if (p_nxpucihal_lcfwdl_ctrl->lcfwdl_thread_running == 0) {
      break;
    }

    if(p_nxpucihal_lcfwdl_ctrl->isPlatformIdSet) {
      status = phNxpUciHal_performLcRotation();
      if (nxpucihal_ctrl.p_uwb_stack_cback != NULL) {
        /* Send binding status cached ntf event */
        if ((nxpucihal_ctrl.p_uwb_stack_data_cback != NULL) && (nxpucihal_lcfwdl_ctrl.rcv_data_len <= UCI_MAX_PAYLOAD_LEN)) {
          if(status != UWBSTATUS_SUCCESS) {
            /* lc rotation FW fwdl failed cased */
            NXPLOG_UCIHAL_E("phNxpUciHal_performLcRotation failed...");
            nxpucihal_lcfwdl_ctrl.rcv_data[4] = 0x04;
          }
          (*nxpucihal_ctrl.p_uwb_stack_data_cback)(nxpucihal_lcfwdl_ctrl.rcv_data_len, nxpucihal_lcfwdl_ctrl.rcv_data);
        }
      }
    } else {
      if (nxpucihal_ctrl.p_uwb_stack_cback != NULL) {
        /* Send binding status cached ntf event */
        if ((nxpucihal_ctrl.p_uwb_stack_data_cback != NULL) && (nxpucihal_lcfwdl_ctrl.rcv_data_len <= UCI_MAX_PAYLOAD_LEN)) {
            /* lc rotation FW fwdl failed cased */
            NXPLOG_UCIHAL_E("%s Platform Id is not set...", __func__);
            nxpucihal_lcfwdl_ctrl.rcv_data[4] = 0x04;
          (*nxpucihal_ctrl.p_uwb_stack_data_cback)(nxpucihal_lcfwdl_ctrl.rcv_data_len, nxpucihal_lcfwdl_ctrl.rcv_data);
        }
      }
    }
    p_nxpucihal_lcfwdl_ctrl->isLcRotationOngoing = 0;
    p_nxpucihal_lcfwdl_ctrl->lcfwdl_thread_running = 0;

    break;
  }

  NXPLOG_UCIHAL_D("lcfwdl thread stopped");
  pthread_attr_destroy(&nxpucihal_lcfwdl_ctrl.attr_thread);
  pthread_exit(NULL);
  return NULL;
}

/******************************************************************************
 * Function         phNxpUciHal_parsePlatformId
 *
 * Description      This function parse GetPlatformId response.
 *
 * Returns          void
 *
 ******************************************************************************/
void phNxpUciHal_parsePlatformId(uint8_t * p_rx_data , uint16_t rx_data_len) {
  uint8_t index = UCI_MSG_HDR_SIZE; // Excluding the header and Versions
  uint8_t * pUwbsRsp = NULL;
  uint16_t uwbsRspLen = 0;
  uint8_t getCalibStatus = UWBSTATUS_FAILED;
  uint8_t getCalibState;
  uint8_t count = 0;
  NXPLOG_UCIHAL_D("phNxpUciHal_parsePlatformId enter ....");

  pUwbsRsp = (uint8_t*)malloc(sizeof(uint8_t) * rx_data_len);
  if(pUwbsRsp == NULL) {
    NXPLOG_UCIHAL_E("pUwbsRsp memory allocation failed");
    return;
  }
  memcpy(&pUwbsRsp[0], &p_rx_data[0], rx_data_len);
  if (rx_data_len < UCI_MSG_HDR_SIZE){
    NXPLOG_UCIHAL_E("%s : Invalid rsp length", __func__);
    free(pUwbsRsp);
    return;
  }
  uwbsRspLen = rx_data_len ;
  getCalibStatus = pUwbsRsp[index++];
  NXPLOG_UCIHAL_D("getCalibStatus %d" , getCalibStatus);
  if(getCalibStatus == UWBSTATUS_SUCCESS) {
    getCalibState = pUwbsRsp[index++];
    if (getCalibState == 0x08) {
      NXPLOG_UCIHAL_D("Platform ID not Set");
      uwb_get_platform_id = false;
      free(pUwbsRsp);
      return;
    } else {
      do {
          nxpucihal_lcfwdl_ctrl.uwbsPlatformId[count++] = pUwbsRsp[index++];
      } while(index < uwbsRspLen);
    }
    nxpucihal_lcfwdl_ctrl.isPlatformIdSet = true;
    uwb_get_platform_id = false;
    NXPLOG_UCIHAL_D("Platform ID: %s", nxpucihal_lcfwdl_ctrl.uwbsPlatformId);
  }
  free(pUwbsRsp);
  return;
}

/******************************************************************************
 * Function         phNxpUciHal_parseUWBSLifecycle
 *
 * Description      This function parse UWBS Lifecycle response.
 *
 * Returns          UWBS Lifecycle.
 *
 ******************************************************************************/
uint32_t phNxpUciHal_parseUWBSLifecycle(uint8_t * p_rx_data , uint16_t rx_data_len) {
  uint8_t index = UCI_MSG_HDR_SIZE; // Excluding the header and Versions
  uint8_t paramId = 0;
  uint8_t length = 0;
  uint32_t uwbsLc = 0;
  uint8_t * pUwbsDeviceInfo = NULL;
  uint16_t pUwbsDeviceInfoLen = 0;
  uint8_t getDeviceInfostatus = UWBSTATUS_FAILED;
  NXPLOG_UCIHAL_D("phNxpUciHal_parseUWBSLifecycle enter ....");

  pUwbsDeviceInfo = (uint8_t*)malloc(sizeof(uint8_t) * rx_data_len);
  if(pUwbsDeviceInfo == NULL) {
    NXPLOG_UCIHAL_E("pUwbsDeviceInfo memory allocation failed");
    return uwbsLc;
  }
  memcpy(&pUwbsDeviceInfo[0], &p_rx_data[0], rx_data_len);
  pUwbsDeviceInfoLen = rx_data_len ;
  getDeviceInfostatus = pUwbsDeviceInfo[index++];
  NXPLOG_UCIHAL_D("getDeviceInfostatus %d" , getDeviceInfostatus);
  if(getDeviceInfostatus == UWBSTATUS_SUCCESS)
  {
    index = index + UWB_INDEX_TO_RETRIEVE_PARAMS;
    uint8_t parameterLength = pUwbsDeviceInfo[index++];
    if (parameterLength > 0) {
      do {
          paramId = pUwbsDeviceInfo[index++];
          length = pUwbsDeviceInfo[index++];
          if ((paramId == UWBS_LIFECYCLE) && (length == UWBS_LIFECYCLE_LENGTH)) {
            uwbsLc = (pUwbsDeviceInfo[index] | (pUwbsDeviceInfo[index+1] << 8) | (pUwbsDeviceInfo[index+2] <<16) | (pUwbsDeviceInfo[index+3] <<24));
            break;
          } else {
            index = index + length;
          }
      } while(index < pUwbsDeviceInfoLen);
    }
  }
  NXPLOG_UCIHAL_D("UWBS Lifecycle: 0x%x", uwbsLc);
  free(pUwbsDeviceInfo);
  return uwbsLc;
}

/******************************************************************************
 * Function         phNxpUciHal_sendSetCoreConfigurations
 *
 * Description      This function send Core device Config command.
 *
 * Returns          status.
 *
 ******************************************************************************/
static uint8_t phNxpUciHal_sendSetCoreConfigurations(){
  /* Disable Low power mode */
  const uint8_t setCoreConfigurations[] = {0x20, 0x04, 0x00, 0x04, 0x01, 0x01, 0x01, 0x00};
  tHAL_UWB_STATUS status = phNxpUciHal_send_ext_cmd(sizeof(setCoreConfigurations), setCoreConfigurations);
  if(status != UWBSTATUS_SUCCESS) {
    return status;
  }
  return status;
}

/******************************************************************************
 * Function         phNxpUciHal_sendGetDeviceCapInfo
 *
 * Description      This function send Device Caps Info command.
 *
 * Returns          status.
 *
 ******************************************************************************/
static uint8_t phNxpUciHal_sendGetDeviceCapInfo(){
  const uint8_t buffer[] = {0x20, 0x03, 0x00, 0x00};
  tHAL_UWB_STATUS status = phNxpUciHal_send_ext_cmd(sizeof(buffer), buffer);
  if(status != UWBSTATUS_SUCCESS) {
    return status;
  }
  return status;
}

/******************************************************************************
 * Function         phNxpUciHal_setSecureConfig
 *
 * Description      This function set secure calibration parameters from config file.
 *
 * Returns          tHAL_UWB_STATUS.
 *
 ******************************************************************************/
tHAL_UWB_STATUS phNxpUciHal_setSecureConfig() {
  NXPLOG_UCIHAL_D(" phNxpUciHal_setSecureConfig Enter..");
  std::array<uint8_t, NXP_MAX_CONFIG_STRING_LEN> buffer;
  uint8_t* vendorConfig = NULL;
  tHAL_UWB_STATUS status;
  buffer.fill(0);
  long retlen = 0;
  // Apply secure calibration
  for(int i = 1;i <= 10;i++) {
    std::string str = NAME_NXP_SECURE_CONFIG_BLK;
    std::string value = std::to_string(i);
    std::string name = str + value;
    NXPLOG_UCIHAL_D(" phNxpUciHal_setSecureConfig :: Name of the config block is %s", name.c_str());
    if (GetNxpConfigByteArrayValue(name.c_str(), (char*)buffer.data(), buffer.size(), &retlen)) {
      if ((retlen > 0) && (retlen <= UCI_MAX_DATA_LEN)) {
        vendorConfig = buffer.data();
        status = phNxpUciHal_send_ext_cmd(retlen,vendorConfig);
        NXPLOG_UCIHAL_D(" phNxpUciHal_send_ext_cmd :: status value for %s is %d ", name.c_str(),status);
        if(status != UWBSTATUS_SUCCESS) {
          NXPLOG_UCIHAL_D(" phNxpUciHal_send_ext_cmd :: setting %s is failed ", name.c_str());
          //skip returning error and go ahead with remaining blocks
          continue;
        }
      }
    } else {
      NXPLOG_UCIHAL_D(" phNxpUciHal_setSecureConfig::%s not available in the config file", name.c_str());
    }
  }
  return UWBSTATUS_SUCCESS;
}

/******************************************************************************
 * Function         phNxpUciHal_getPlatformId
 *
 * Description      This function use to get platform ID.
 *
 * Returns          tHAL_UWB_STATUS.
 *
 ******************************************************************************/
tHAL_UWB_STATUS phNxpUciHal_getPlatformId() {
  NXPLOG_UCIHAL_D(" phNxpUciHal_getPlatformId Enter..");
  const uint8_t buffer[] = {0x2F, EXT_UCI_MSG_GET_CALIBRATION, 0x00, 0x02, 0x00, UCI_CALIB_PARAM_PLATFORM_ID};
  uwb_get_platform_id = true;
  tHAL_UWB_STATUS status = phNxpUciHal_send_ext_cmd(sizeof(buffer), buffer);
  if(status != UWBSTATUS_SUCCESS) {
    return status;
  }
  return UWBSTATUS_SUCCESS;
}

/******************************************************************************
 * Function         phNxpUciHal_setPlatformId
 *
 * Description      This function set platform ID given in config file.
 *
 * Returns          tHAL_UWB_STATUS.
 *
 ******************************************************************************/
tHAL_UWB_STATUS phNxpUciHal_setPlatformId() {
  NXPLOG_UCIHAL_D(" phNxpUciHal_setPlatformId Enter..");
  uint8_t *platformId = NULL;
  uint8_t buffer[UCI_MAX_DATA_LEN] = {0x00};
  tHAL_UWB_STATUS status;

  platformId = (uint8_t *)malloc(NXP_MAX_CONFIG_STRING_LEN * sizeof(uint8_t));
  if (platformId == NULL) {
    NXPLOG_FWDNLD_E("malloc of platformId failed ");
    return UWBSTATUS_FAILED;
  }

  if (GetNxpConfigStrValue(NAME_PLATFORM_ID, (char *)platformId, NXP_MAX_CONFIG_STRING_LEN)) {
    int platformIdLen = strlen((char*)platformId);
    NXPLOG_UCIHAL_D(" %s Platform ID: %s",__func__, platformId);
    buffer[0] = 0x2F;
    buffer[1] = EXT_UCI_MSG_SET_CALIBRATION;
    buffer[2] = 0x00;
    buffer[3] = platformIdLen + 3; //payload (channelid+calibparam+length+calibValue)
    buffer[4] = 0x00; //channel id
    buffer[5] = UCI_CALIB_PARAM_PLATFORM_ID;
    buffer[6] = platformIdLen;
    for(int i = 0 ; i < platformIdLen ; i++)
    {
      buffer[7 + i] = platformId[i];
    }
    int cmdLen = buffer[3] + UCI_MSG_HDR_SIZE;

    status = phNxpUciHal_send_ext_cmd(cmdLen,buffer);
    NXPLOG_UCIHAL_D(" phNxpUciHal_send_ext_cmd :: status value for PLATFORM_ID is %d ", status);
  } else {
    NXPLOG_UCIHAL_D(" %s :: PLATFORM_ID not available in the config file", __func__);
    status = UWBSTATUS_FAILED;
  }

  if (platformId != NULL) {
      free(platformId);
  }
  return status;
}

tHAL_UWB_STATUS phNxpUciHal_start_lcfwdl_thread() {
  NXPLOG_UCIHAL_D("phNxpUciHal_start_lcfwdl_thread enter....");

  nxpucihal_lcfwdl_ctrl.rcv_data_len = nxpucihal_ctrl.rx_data_len;
  memcpy(&nxpucihal_lcfwdl_ctrl.rcv_data[0], nxpucihal_ctrl.p_rx_data, nxpucihal_lcfwdl_ctrl.rcv_data_len);

  CONCURRENCY_LOCK();
  pthread_attr_init(&nxpucihal_lcfwdl_ctrl.attr_thread);
  pthread_attr_setdetachstate(&nxpucihal_lcfwdl_ctrl.attr_thread, PTHREAD_CREATE_DETACHED);
  if (pthread_create(&nxpucihal_lcfwdl_ctrl.lcfwdl_tread, &nxpucihal_lcfwdl_ctrl.attr_thread,
               phNxpUciHal_lcfwdl_thread, &nxpucihal_lcfwdl_ctrl) != 0) {
    NXPLOG_UCIHAL_E("pthread_create failed");
    CONCURRENCY_UNLOCK();
    return UWBSTATUS_FAILED;
  }
  CONCURRENCY_UNLOCK();
  return UWBSTATUS_SUCCESS;
}

static tHAL_UWB_STATUS phNxpUciHal_performLcRotation() {
  tHAL_UWB_STATUS status = UWBSTATUS_FAILED;
  uint8_t fwd_retry_count = 0;

  phTmlUwb_Spi_Reset();
  NXPLOG_UCIHAL_D(" Start LC rotation FW download");
  /* Create the local semaphore */
  if (phNxpUciHal_init_cb_data(&nxpucihal_ctrl.dev_status_ntf_wait, NULL) !=
      UWBSTATUS_SUCCESS) {
    NXPLOG_UCIHAL_E("Create dev_status_ntf_wait failed");
    return status;
  }

  uwb_device_initialized = false;
fwd_retry:
      nxpucihal_ctrl.fw_dwnld_mode = true; /* system in FW download mode*/
      nxpucihal_ctrl.uwbc_device_state = UWB_DEVICE_STATE_UNKNOWN;
      status = phNxpUciHal_fw_lcrotation();
      if(status == UWBSTATUS_SUCCESS) {
          nxpucihal_ctrl.isSkipPacket = 1;
          status = phTmlUwb_Read( Rx_buffer, UCI_MAX_DATA_LEN,
                    (pphTmlUwb_TransactCompletionCb_t)&phNxpUciHal_read_complete, NULL);
          if (status != UWBSTATUS_PENDING) {
            NXPLOG_UCIHAL_E("read status error status = %x", status);
            goto failure;
          }
          phNxpUciHal_sem_timed_wait(&nxpucihal_ctrl.dev_status_ntf_wait);
          if (nxpucihal_ctrl.dev_status_ntf_wait.status != UWBSTATUS_SUCCESS) {
            NXPLOG_UCIHAL_E("UWB_DEVICE_READY dev_status_ntf_wait semaphore timed out");
            goto failure;
          }
          NXPLOG_UCIHAL_D("uwbc_device_state: %d",nxpucihal_ctrl.uwbc_device_state);
          if(nxpucihal_ctrl.uwbc_device_state != UWB_DEVICE_READY) {
            NXPLOG_UCIHAL_E("UWB_DEVICE_READY not received uwbc_device_state = %x",nxpucihal_ctrl.uwbc_device_state);
            goto failure;
          }
          nxpucihal_ctrl.isSkipPacket = 0;
          status = phNxpUciHal_set_board_config();
          if (status != UWBSTATUS_SUCCESS) {
            NXPLOG_UCIHAL_E("%s: Set Board Config Failed", __func__);
            goto failure;
          }
          phNxpUciHal_sem_timed_wait(&nxpucihal_ctrl.dev_status_ntf_wait);
          if (nxpucihal_ctrl.dev_status_ntf_wait.status != UWBSTATUS_SUCCESS) {
            NXPLOG_UCIHAL_E("UWB_DEVICE_READY dev_status_ntf_wait semaphore timed out");
            goto failure;
          }
          if(nxpucihal_ctrl.uwbc_device_state != UWB_DEVICE_READY) {
            NXPLOG_UCIHAL_E("UWB_DEVICE_READY not received uwbc_device_state = %x",nxpucihal_ctrl.uwbc_device_state);
            goto failure;
          }
          NXPLOG_UCIHAL_D("%s: Send device reset", __func__);
          status = phNxpUciHal_uwb_reset();
          if (status != UWBSTATUS_SUCCESS) {
            NXPLOG_UCIHAL_E("%s: device reset Failed", __func__);
            goto failure;
          }
          phNxpUciHal_sem_timed_wait(&nxpucihal_ctrl.dev_status_ntf_wait);
          if (nxpucihal_ctrl.dev_status_ntf_wait.status != UWBSTATUS_SUCCESS) {
            NXPLOG_UCIHAL_E("UWB_DEVICE_READY dev_status_ntf_wait semaphore timed out");
            goto failure;
          }
          if(nxpucihal_ctrl.uwbc_device_state != UWB_DEVICE_READY) {
            NXPLOG_UCIHAL_E("UWB_DEVICE_READY not received uwbc_device_state = %x",nxpucihal_ctrl.uwbc_device_state);
            goto failure;
          }

          status = phNxpUciHal_applyVendorConfig();
          if (status != UWBSTATUS_SUCCESS) {
            // If vendor config is failed after LC rotation , as of now skip reporting error
            NXPLOG_UCIHAL_E("%s: Apply vendor Config Failed", __func__);
          }

          status = phNxpUciHal_setSecureConfig();
          if (status != UWBSTATUS_SUCCESS) {
            // If set secure calib param failed , as of now skip reporting error
            NXPLOG_UCIHAL_E("%s: Apply secure Config Failed", __func__);
          }

          status = phNxpUciHal_sendGetCoreDeviceInfo();
          if (status != UWBSTATUS_SUCCESS) {
            NXPLOG_UCIHAL_E("%s: phNxpUciHal_sendGetCoreDeviceInfo Failed", __func__);
            goto failure;
          }

          status = phNxpUciHal_sendSetCoreConfigurations();
          if (status != UWBSTATUS_SUCCESS) {
            NXPLOG_UCIHAL_E("%s: phNxpUciHal_setCoreConfigurations Failed", __func__);
            goto failure;
          }
          status = phNxpUciHal_sendGetDeviceCapInfo();
          if (status != UWBSTATUS_SUCCESS) {
            NXPLOG_UCIHAL_E("%s: phNxpUciHal_sendGetDeviceCapInfo Failed", __func__);
            goto failure;
          }
          uwb_device_initialized = true;
      } else if(status == UWBSTATUS_FILE_NOT_FOUND) {
        NXPLOG_UCIHAL_E("FW download File Not found: status= %x", status);
        goto failure;
      } else {
        NXPLOG_UCIHAL_E("FW download is failed FW download recovery starts: status= %x", status);
        fwd_retry_count++;
          if(fwd_retry_count <= FWD_MAX_RETRY_COUNT) {
            phTmlUwb_Chip_Reset();
            usleep(5000);
            goto fwd_retry;
          } else {
            goto failure;
          }
      }
      phNxpUciHal_cleanup_cb_data(&nxpucihal_ctrl.dev_status_ntf_wait);
      return status;
    failure:
        if(nxpucihal_ctrl.uwbc_device_state == UWB_DEVICE_ERROR) {
          phNxpUciHalProp_dump_fw_crash_log();
          if (UWBSTATUS_SUCCESS != phNxpUciHal_uwb_reset()) {
            NXPLOG_UCIHAL_E("%s: device reset Failed", __func__);
          } else {
            NXPLOG_UCIHAL_E("%s: device reset success", __func__);
          }
          phTmlUwb_Spi_Reset();
          goto fwd_retry;
        }
        phNxpUciHal_cleanup_cb_data(&nxpucihal_ctrl.dev_status_ntf_wait);

        return UWBSTATUS_FAILED;
}

#endif