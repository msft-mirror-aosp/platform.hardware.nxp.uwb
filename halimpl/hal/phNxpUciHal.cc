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

#include <log/log.h>
#include <phNxpLog.h>
#include <cutils/properties.h>
#include <phNxpUciHal.h>
#include <phNxpUciHal_Adaptation.h>
#include <phNxpUciHal_ext.h>
#include <phTmlUwb_spi.h>
#include <sys/stat.h>
#include <string.h>
#include <array>
#include <map>
#include <vector>
#include "hal_nxpuwb.h"
#include "phNxpConfig.h"
#include <android-base/stringprintf.h>
#include "phNxpUciHal_utils.h"

using namespace std;
using android::base::StringPrintf;

extern map<uint16_t, vector<uint16_t>> input_map;
extern map<uint16_t, vector<uint16_t>> conf_map;

/*********************** Global Variables *************************************/
/* UCI HAL Control structure */
phNxpUciHal_Control_t nxpucihal_ctrl;
uint8_t *gpCoreDeviceInfoRsp = NULL;
/* TML Context */
extern phTmlUwb_Context_t* gpphTmlUwb_Context;

bool uwb_debug_enabled = false;
bool uwb_device_initialized = false;
bool uwb_get_platform_id = false;
uint32_t timeoutTimerId = 0;
char persistant_log_path[120];
static uint8_t Rx_data[UCI_MAX_DATA_LEN];
uint8_t deviceType = '\0';
phNxpUciHal_FW_Version_t fw_version;
uint8_t channel_5_support = 1;
uint8_t channel_9_support = 1;
short conf_tx_power = 0;
bool uwb_enable = true;
/* AOA support handling */
bool isAntennaRxPairDefined = false;
int numberOfAntennaPairs = 0;

/**************** local methods used in this file only ************************/
static void phNxpUciHal_open_complete(tHAL_UWB_STATUS status);
static void phNxpUciHal_write_complete(void* pContext,
                                       phTmlUwb_TransactInfo_t* pInfo);
static void phNxpUciHal_close_complete(tHAL_UWB_STATUS status);
static void phNxpUciHal_kill_client_thread(
    phNxpUciHal_Control_t* p_nxpucihal_ctrl);
static void* phNxpUciHal_client_thread(void* arg);
extern int phNxpUciHal_fw_download();
static void phNxpUciHal_print_response_status(uint8_t *p_rx_data,
                                              uint16_t p_len);
static void phNxpUciHal_getVersionInfo();

/******************************************************************************
 * Function         phNxpUciHal_client_thread
 *
 * Description      This function is a thread handler which handles all TML and
 *                  UCI messages.
 *
 * Returns          void
 *
 ******************************************************************************/
static void* phNxpUciHal_client_thread(void* arg) {
  phNxpUciHal_Control_t* p_nxpucihal_ctrl = (phNxpUciHal_Control_t*)arg;
  phLibUwb_Message_t msg;

  NXPLOG_UCIHAL_D("thread started");

  p_nxpucihal_ctrl->thread_running = 1;

  while (p_nxpucihal_ctrl->thread_running == 1) {
    /* Fetch next message from the UWB stack message queue */
    if (phDal4Uwb_msgrcv(p_nxpucihal_ctrl->gDrvCfg.nClientId, &msg, 0, 0) ==
        -1) {
      NXPLOG_UCIHAL_E("UWB client received bad message");
      continue;
    }

    if (p_nxpucihal_ctrl->thread_running == 0) {
      break;
    }

    switch (msg.eMsgType) {
      case PH_LIBUWB_DEFERREDCALL_MSG: {
        phLibUwb_DeferredCall_t* deferCall =
            (phLibUwb_DeferredCall_t*)(msg.pMsgData);

        REENTRANCE_LOCK();
        deferCall->pCallback(deferCall->pParameter);
        REENTRANCE_UNLOCK();

        break;
      }

      case UCI_HAL_OPEN_CPLT_MSG: {
        REENTRANCE_LOCK();
        if (nxpucihal_ctrl.p_uwb_stack_cback != NULL) {
          /* Send the event */
          (*nxpucihal_ctrl.p_uwb_stack_cback)(HAL_UWB_OPEN_CPLT_EVT,
                                              HAL_UWB_STATUS_OK);
        }
        REENTRANCE_UNLOCK();
        break;
      }

      case UCI_HAL_CLOSE_CPLT_MSG: {
        REENTRANCE_LOCK();
        if (nxpucihal_ctrl.p_uwb_stack_cback != NULL) {
          /* Send the event */
          (*nxpucihal_ctrl.p_uwb_stack_cback)(HAL_UWB_CLOSE_CPLT_EVT,
                                              HAL_UWB_STATUS_OK);
          phNxpUciHal_kill_client_thread(&nxpucihal_ctrl);
        }
        REENTRANCE_UNLOCK();
        break;
      }

      case UCI_HAL_INIT_CPLT_MSG: {
        REENTRANCE_LOCK();
        if (nxpucihal_ctrl.p_uwb_stack_cback != NULL) {
          /* Send the event */
          (*nxpucihal_ctrl.p_uwb_stack_cback)(HAL_UWB_INIT_CPLT_EVT,
                                              HAL_UWB_STATUS_OK);
        }
        REENTRANCE_UNLOCK();
        break;
      }

      case UCI_HAL_ERROR_MSG: {
        REENTRANCE_LOCK();
        if (nxpucihal_ctrl.p_uwb_stack_cback != NULL) {
          /* Send the event */
          (*nxpucihal_ctrl.p_uwb_stack_cback)(HAL_UWB_ERROR_EVT,
                                              HAL_UWB_ERROR_EVT);
        }
        REENTRANCE_UNLOCK();
        break;
      }
    }
  }

  NXPLOG_UCIHAL_D("NxpUciHal thread stopped");
  pthread_exit(NULL);
  return NULL;
}

/*************************************************************************************
 * Function         handlingVendorSpecificAppConfig
 *
 * Description      This function removes the vendor specific app config from
 *UCI command
 *
 * Returns          void
 *
 *************************************************************************************/
void handlingVendorSpecificAppConfig(uint16_t *data_len, uint8_t *p_data) {
  uint8_t mt = 0, gid = 0, oid = 0;

  mt = (*(p_data)&UCI_MT_MASK) >> UCI_MT_SHIFT;
  gid = p_data[0] & UCI_GID_MASK;
  oid = p_data[1] & UCI_OID_MASK;

  if (mt == UCI_MT_CMD) {
    if ((gid == UCI_GID_SESSION_MANAGE) &&
        (oid == UCI_MSG_SESSION_SET_APP_CONFIG)) {
        uint16_t dataLength = *data_len, numOfbytes = 0, numOfConfigs = 0,
                 appConfigLength = 0;

        /* Create local copy of cmd_data for data manipulation*/
        uint8_t uciCmd[UCI_MAX_DATA_LEN];
        memcpy(uciCmd, p_data, dataLength);

        uint16_t startOfByteManipulation = UCI_MSG_HDR_SIZE +
                                           UCI_CMD_SESSION_ID_OFFSET +
                                           UCI_CMD_NUM_CONFIG_PARAM_LENGTH;

        uint8_t tlvLength = UCI_CMD_TAG_BYTE_LENGTH +
                            UCI_CMD_PARAM_SIZE_BYTE_LENGTH +
                            UCI_CMD_PAYLOAD_BYTE_LENGTH;

        for (uint16_t i = startOfByteManipulation, j = startOfByteManipulation;
             i < dataLength;) {
          // static condition checks
          // removing the vendor specific app config as it's not supported by FW
          if ((uciCmd[i] == UCI_PARAM_ID_AOA_AZIMUTH_MEASUREMENTS) &&
              (uciCmd[i + tlvLength] ==
               UCI_PARAM_ID_AOA_ELEVATION_MEASUREMENTS) &&
              (uciCmd[i + (2 * tlvLength)] ==
               UCI_PARAM_ID_RANGE_MEASUREMENTS)) {

            numOfConfigs = 3; // removing 3 TAG ID's
            numOfbytes = 9;   // removing 9 bytes out of payload
            i += 9;           // skipping 9 bytes in byte copying
            NXPLOG_UCIHAL_D(
                "Removed param payload with Tag ID:0x%x, 0x%x, 0x%x",
                UCI_PARAM_ID_AOA_AZIMUTH_MEASUREMENTS,
                UCI_PARAM_ID_AOA_ELEVATION_MEASUREMENTS,
                UCI_PARAM_ID_RANGE_MEASUREMENTS);
          } else {
            p_data[j] = uciCmd[i];
            i++;
            j++;
          }
        }

        // uci number of config params update
        p_data[UCI_CMD_NUM_CONFIG_PARAM_BYTE] -= numOfConfigs;

        // uci command length update
        dataLength -= numOfbytes;
        *data_len = dataLength;

        // uci cmd app config length update
        appConfigLength = (p_data[UCI_CMD_LENGTH_PARAM_BYTE1] & 0xFF) |
                          ((p_data[UCI_CMD_LENGTH_PARAM_BYTE2] & 0xFF) << 8);
        appConfigLength -= numOfbytes;
        p_data[UCI_CMD_LENGTH_PARAM_BYTE2] = (appConfigLength & 0xFF00) >> 8;
        p_data[UCI_CMD_LENGTH_PARAM_BYTE1] = (appConfigLength & 0xFF);
    }
  }
}

bool isCountryCodeMapCreated = false;
/******************************************************************************
 * Function         phNxpUciHal_parse
 *
 * Description      This function parses all the data passing through the HAL.
 *
 * Returns          It returns true if the incoming command to be skipped.
 *
 ******************************************************************************/
bool phNxpUciHal_parse(uint16_t data_len, const uint8_t *p_data) {
  uint8_t mt = 0, gid = 0, oid = 0;
  uint16_t arrLen = 0, tag = 0, idx = 0;
  bool ret = false;

  map<uint16_t, vector<uint16_t>>::iterator itr;
  vector<uint16_t>::iterator v_itr;
  mt = (*(p_data)&UCI_MT_MASK) >> UCI_MT_SHIFT;
  gid = p_data[0] & UCI_GID_MASK;
  oid = p_data[1] & UCI_OID_MASK;

  if (mt == UCI_MT_CMD) {
    if ((gid == UCI_GID_ANDROID) && (oid == UCI_MSG_ANDROID_SET_COUNTRY_CODE)) {
        if (data_len < 6) {
          return true;
        }
        char country_code[2];
        country_code[0] = (char)p_data[4];
        country_code[1] = (char)p_data[5];

        if ((country_code[0] == '0') && (country_code[1] == '0')) {
          NXPLOG_UCIHAL_D("Country code %c%c is Invalid!", country_code[0], country_code[1]);
        } else {
          NxpConfig_SetCountryCode(country_code);
        }

        // send country code response to upper layer
        nxpucihal_ctrl.rx_data_len = 5;
        static uint8_t rsp_data[5];
        rsp_data[0] = 0x4c;
        rsp_data[1] = 0x01;
        rsp_data[2] = 0x00;
        rsp_data[3] = 0x01;
        if (uwb_enable) {
          rsp_data[4] = 0x00; // Response Success
        } else {
          NXPLOG_UCIHAL_D("Country code uwb disable "
                          "UCI_STATUS_CODE_ANDROID_REGULATION_UWB_OFF!");
          rsp_data[4] = UCI_STATUS_CODE_ANDROID_REGULATION_UWB_OFF;
        }
        ret = true;
        (*nxpucihal_ctrl.p_uwb_stack_data_cback)(nxpucihal_ctrl.rx_data_len,
                                                 rsp_data);
    } else if ((gid == UCI_GID_PROPRIETARY_0x0F) &&
               (oid == SET_VENDOR_SET_CALIBRATION)) {
        if (p_data[UCI_MSG_HDR_SIZE + 1] ==
            VENDOR_CALIB_PARAM_TX_POWER_PER_ANTENNA) {
          phNxpUciHal_processCalibParamTxPowerPerAntenna(conf_tx_power, p_data,
                                                         data_len);
        }
    } else if ((gid == UCI_GID_SESSION_MANAGE) &&
               (oid == UCI_MSG_SESSION_SET_APP_CONFIG)) {
        uint8_t index =
            4 /*UCI_MSG_HDR_SIZE*/ + 4 /*Session_Id*/ + 1 /*Num of configs*/;
        uint8_t len = p_data[UCI_MSG_HDR_SIZE - 1];
        len = len + UCI_MSG_HDR_SIZE; // length should be size of payload + size
                                      // of header
        uint8_t tagId, length, channel_number, no_of_config;
        no_of_config = p_data[8];

        while (index < len) {
          tagId = p_data[index++];
          length = p_data[index++];
          if (tagId == UCI_PARAM_ID_CHANNEL_NUMBER) {

            channel_number = p_data[index];

            if (((channel_number == CHANNEL_NUM_5) && !channel_5_support) ||
                ((channel_number == CHANNEL_NUM_9) && !channel_9_support)) {
              NXPLOG_UCIHAL_D("Country code blocked channel");

              // send setAppConfig response with COUNTRY_CODE_BLOCKED response
              nxpucihal_ctrl.rx_data_len = 8;
              static uint8_t rsp_data[8];
              rsp_data[0] = 0x41;
              rsp_data[1] = 0x03;
              rsp_data[2] = 0x00;
              rsp_data[3] = 0x04; // Length
              rsp_data[4] = 0x02;
              rsp_data[5] = 0x01;
              rsp_data[6] = 0x04;
              rsp_data[7] = UCI_STATUS_COUNTRY_CODE_BLOCKED_CHANNEL;
              ret = true;
              (*nxpucihal_ctrl.p_uwb_stack_data_cback)(
                  nxpucihal_ctrl.rx_data_len, rsp_data);
              break;
            }
          }
          index += length;
        }
    }
  } else if (mt == UCI_MT_RSP) {
    if ((gid == UCI_GID_CORE) && (oid == UCI_MSG_CORE_GET_CAPS_INFO)) {
        // do not modify caps if the country code is not received from upper
        // layer.
        if (isCountryCodeMapCreated == false) {
          return false;
        }
        // Check UWBS Caps response status
        if (p_data[4] == 0) {
          idx = UCI_PKT_HDR_LEN + UCI_PKT_PAYLOAD_STATUS_LEN +
                UCI_PKT_NUM_CAPS_LEN;
          if (get_input_map(p_data, data_len, idx)) {
            NXPLOG_UCIHAL_D("Input Map created");
          } else {
            NXPLOG_UCIHAL_D("Input Map creation failed");
            return false;
          }
        } else {
          return false;
        }
        // Compare the maps for Tags and modify input map if Values are
        // different
        for (itr = input_map.begin(); itr != input_map.end(); ++itr) {
          tag = itr->first;
          // Check for the Tag in both maps
          if ((conf_map.count(tag)) == 1) {
            if (tag == UWB_CHANNELS || tag == CCC_UWB_CHANNELS) {
              NXPLOG_UCIHAL_D(
                  "Tag = 0x%02X , modify UWB_CHANNELS based on country conf ",
                  tag);
              for (uint32_t j = 0; j < (itr->second).size(); j++) {
                (input_map[tag])[j] =
                    ((conf_map[tag])[j]) & ((input_map[tag])[j]);
              }
            }
          } else {
            // TAG not found do nothing
          }
        }
        // convert the modified input map to p_caps_resp array
        memset(nxpucihal_ctrl.p_caps_resp, 0, UCI_MAX_DATA_LEN);
        // Header information from Input array is updated in initial bytes
        nxpucihal_ctrl.p_caps_resp[0] = p_data[0];
        nxpucihal_ctrl.p_caps_resp[1] = p_data[1];
        nxpucihal_ctrl.p_caps_resp[2] = p_data[2];
        nxpucihal_ctrl.p_caps_resp[4] = p_data[4];
        for (itr = input_map.begin(); itr != input_map.end(); ++itr) {
          tag = itr->first;
          // If Tag is 0xE0 or 0xE1 or 0xE2,Tag will be of 2 bytes
          if (((tag >> 8) >= 0xE0) && ((tag >> 8) <= 0xE2)) {
            nxpucihal_ctrl.p_caps_resp[idx++] = (tag & 0xFF00) >> 8;
            nxpucihal_ctrl.p_caps_resp[idx++] = (tag & 0x00FF);
          } else {
            nxpucihal_ctrl.p_caps_resp[idx++] = tag;
          }
          for (v_itr = itr->second.begin(); v_itr != itr->second.end();
               ++v_itr) {
            nxpucihal_ctrl.p_caps_resp[idx++] = (*v_itr);
          }
        }
        arrLen = idx;
        // exclude the initial header data
        nxpucihal_ctrl.p_caps_resp[3] = arrLen - UCI_PKT_HDR_LEN;
        // update the number of parameter TLVs.
        nxpucihal_ctrl.p_caps_resp[5] = input_map.size();
        input_map.clear();
        // send GET CAPS INFO response to the Upper Layer
        (*nxpucihal_ctrl.p_uwb_stack_data_cback)(arrLen,
                                                 nxpucihal_ctrl.p_caps_resp);
        // skip the incoming packet as we have send the modified response
        // already
        nxpucihal_ctrl.isSkipPacket = 1;
        ret = false;
    }
  } else {
    ret = false;
  }
  return ret;
}
/******************************************************************************
 * Function         phNxpUciHal_kill_client_thread
 *
 * Description      This function safely kill the client thread and clean all
 *                  resources.
 *
 * Returns          void.
 *
 ******************************************************************************/
static void phNxpUciHal_kill_client_thread(
    phNxpUciHal_Control_t* p_nxpucihal_ctrl) {
  NXPLOG_UCIHAL_D("Terminating phNxpUciHal client thread...");

  p_nxpucihal_ctrl->p_uwb_stack_cback = NULL;
  p_nxpucihal_ctrl->p_uwb_stack_data_cback = NULL;
  p_nxpucihal_ctrl->thread_running = 0;

  return;
}

/******************************************************************************
 * Function         phNxpUciHal_open
 *
 * Description      This function is called by libuwb-uci during the
 *                  initialization of the UWBC. It opens the physical connection
 *                  with UWBC (SRXXX) and creates required client thread for
 *                  operation.
 *                  After open is complete, status is informed to libuwb-uci
 *                  through callback function.
 *
 * Returns          This function return UWBSTATUS_SUCCES (0) in case of success
 *                  In case of failure returns other failure value.
 *
 ******************************************************************************/
tHAL_UWB_STATUS phNxpUciHal_open(uwb_stack_callback_t* p_cback,
                     uwb_stack_data_callback_t* p_data_cback) {
  phOsalUwb_Config_t tOsalConfig;
  phTmlUwb_Config_t tTmlConfig;
  char* uwb_dev_node = NULL;
  const uint16_t max_len = 260;
  tHAL_UWB_STATUS wConfigStatus = UWBSTATUS_SUCCESS;
  pthread_attr_t attr;

  if (nxpucihal_ctrl.halStatus == HAL_STATUS_OPEN) {
    NXPLOG_UCIHAL_E("phNxpUciHal_open already open");
    return UWBSTATUS_SUCCESS;
  }

  NxpConfig_Init();

  /* initialize trace level */
  phNxpLog_InitializeLogLevel();
  /*Create the timer for extns write response*/
  timeoutTimerId = phOsalUwb_Timer_Create();

  if (phNxpUciHal_init_monitor() == NULL) {
    NXPLOG_UCIHAL_E("Init monitor failed");
    return UWBSTATUS_FAILED;
  }

  CONCURRENCY_LOCK();

  memset(&nxpucihal_ctrl, 0x00, sizeof(nxpucihal_ctrl));
  memset(&tOsalConfig, 0x00, sizeof(tOsalConfig));
  memset(&tTmlConfig, 0x00, sizeof(tTmlConfig));
  uwb_dev_node = (char*)nxp_malloc(max_len * sizeof(char));
  if (uwb_dev_node == NULL) {
      NXPLOG_UCIHAL_E("malloc of uwb_dev_node failed ");
      goto clean_and_return;
  } else {
      NXPLOG_UCIHAL_E("Assigning the default helios Node: dev/srxxx");
      strcpy(uwb_dev_node, "/dev/srxxx");
  }
  /* By default HAL status is HAL_STATUS_OPEN */
  nxpucihal_ctrl.halStatus = HAL_STATUS_OPEN;

  nxpucihal_ctrl.p_uwb_stack_cback = p_cback;
  nxpucihal_ctrl.p_uwb_stack_data_cback = p_data_cback;
  nxpucihal_ctrl.fw_dwnld_mode = false;

  /* Configure hardware link */
  nxpucihal_ctrl.gDrvCfg.nClientId = phDal4Uwb_msgget(0, 0600);
  nxpucihal_ctrl.gDrvCfg.nLinkType = ENUM_LINK_TYPE_SPI;
  tTmlConfig.pDevName = (int8_t*)uwb_dev_node;
  tOsalConfig.dwCallbackThreadId = (uintptr_t)nxpucihal_ctrl.gDrvCfg.nClientId;
  tOsalConfig.pLogFile = NULL;
  tTmlConfig.dwGetMsgThreadId = (uintptr_t)nxpucihal_ctrl.gDrvCfg.nClientId;

  /* Initialize TML layer */
  wConfigStatus = phTmlUwb_Init(&tTmlConfig);
  if (wConfigStatus != UWBSTATUS_SUCCESS) {
    NXPLOG_UCIHAL_E("phTmlUwb_Init Failed");
    goto clean_and_return;
  } else {
    if (uwb_dev_node != NULL) {
      free(uwb_dev_node);
      uwb_dev_node = NULL;
    }
  }

  /* Create the client thread */
  pthread_attr_init(&attr);
  pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
  if (pthread_create(&nxpucihal_ctrl.client_thread, &attr,
                     phNxpUciHal_client_thread, &nxpucihal_ctrl) != 0) {
    NXPLOG_UCIHAL_E("pthread_create failed");
    wConfigStatus = phTmlUwb_Shutdown();
    goto clean_and_return;
  }

  CONCURRENCY_UNLOCK();

#if 0
  /* call read pending */
  status = phTmlUwb_Read(
      nxpucihal_ctrl.p_cmd_data, UCI_MAX_DATA_LEN,
      (pphTmlUwb_TransactCompletionCb_t)&phNxpUciHal_read_complete, NULL);
  if (status != UWBSTATUS_PENDING) {
    NXPLOG_UCIHAL_E("TML Read status error status = %x", status);
    wConfigStatus = phTmlUwb_Shutdown();
    wConfigStatus = UWBSTATUS_FAILED;
    goto clean_and_return;
  }
#endif
  pthread_attr_destroy(&attr);
  /* Call open complete */
  phNxpUciHal_open_complete(wConfigStatus);
  return wConfigStatus;

clean_and_return:
  CONCURRENCY_UNLOCK();
  if (uwb_dev_node != NULL) {
    free(uwb_dev_node);
    uwb_dev_node = NULL;
  }

  /* Report error status */
  (*nxpucihal_ctrl.p_uwb_stack_cback)(HAL_UWB_OPEN_CPLT_EVT, HAL_UWB_ERROR_EVT);

  nxpucihal_ctrl.p_uwb_stack_cback = NULL;
  nxpucihal_ctrl.p_uwb_stack_data_cback = NULL;
  phNxpUciHal_cleanup_monitor();
  nxpucihal_ctrl.halStatus = HAL_STATUS_CLOSE;
  pthread_attr_destroy(&attr);
  return wConfigStatus;
}

/******************************************************************************
 * Function         phNxpUciHal_open_complete
 *
 * Description      This function inform the status of phNxpUciHal_open
 *                  function to libuwb-uci.
 *
 * Returns          void.
 *
 ******************************************************************************/
static void phNxpUciHal_open_complete(tHAL_UWB_STATUS status) {
  static phLibUwb_Message_t msg;

  if (status == UWBSTATUS_SUCCESS) {
    msg.eMsgType = UCI_HAL_OPEN_CPLT_MSG;
    nxpucihal_ctrl.hal_open_status = true;
    nxpucihal_ctrl.halStatus = HAL_STATUS_OPEN;
  } else {
    msg.eMsgType = UCI_HAL_ERROR_MSG;
  }

  msg.pMsgData = NULL;
  msg.Size = 0;

  phTmlUwb_DeferredCall(gpphTmlUwb_Context->dwCallbackThreadId,
                        (phLibUwb_Message_t*)&msg);

  return;
}

/******************************************************************************
 * Function         phNxpUciHal_write
 *
 * Description      This function write the data to UWBC through physical
 *                  interface (e.g. SPI) using the  driver interface.
 *                  Before sending the data to UWBC, phNxpUciHal_write_ext
 *                  is called to check if there is any extension processing
 *                  is required for the UCI packet being sent out.
 *
 * Returns          It returns number of bytes successfully written to UWBC.
 *
 ******************************************************************************/
tHAL_UWB_STATUS phNxpUciHal_write(uint16_t data_len, const uint8_t* p_data) {
  if (nxpucihal_ctrl.halStatus != HAL_STATUS_OPEN) {
    return UWBSTATUS_FAILED;
  }
  uint16_t len = 0;

  CONCURRENCY_LOCK();
  phNxpUciHal_process_ext_cmd_rsp(data_len, p_data, &len);
  CONCURRENCY_UNLOCK();

  /* No data written */
  return len;
}

/******************************************************************************
 * Function         phNxpUciHal_write_unlocked
 *
 * Description      This is the actual function which is being called by
 *                  phNxpUciHal_write. This function writes the data to UWBC.
 *                  It waits till write callback provide the result of write
 *                  process.
 *
 * Returns          It returns number of bytes successfully written to UWBC.
 *
 ******************************************************************************/
tHAL_UWB_STATUS phNxpUciHal_write_unlocked(uint16_t data_len, const uint8_t* p_data) {
  tHAL_UWB_STATUS status;
  uint8_t mt, pbf, gid, oid;

  phNxpUciHal_Sem_t cb_data;
  /* Create the local semaphore */
  if (phNxpUciHal_init_cb_data(&cb_data, NULL) != UWBSTATUS_SUCCESS) {
    NXPLOG_UCIHAL_D("phNxpUciHal_write_unlocked Create cb data failed");
    data_len = 0;
    goto clean_and_return;
  }

  if ((data_len > UCI_MAX_DATA_LEN) || (data_len < UCI_PKT_HDR_LEN)) {
    NXPLOG_UCIHAL_E("Invalid data_len");
    data_len = 0;
    goto clean_and_return;
  }

  /* Create local copy of cmd_data */
  memcpy(nxpucihal_ctrl.p_cmd_data, p_data, data_len);
  nxpucihal_ctrl.cmd_len = data_len;

  data_len = nxpucihal_ctrl.cmd_len;
  UCI_MSG_PRS_HDR0(p_data, mt, pbf, gid);
  UCI_MSG_PRS_HDR1(p_data, oid);

  /*vendor specific params handling*/
  handlingVendorSpecificAppConfig(&nxpucihal_ctrl.cmd_len,
                                  nxpucihal_ctrl.p_cmd_data);

  /* Vendor Specific Parsing logic */
  nxpucihal_ctrl.hal_parse_enabled =
      phNxpUciHal_parse(nxpucihal_ctrl.cmd_len, nxpucihal_ctrl.p_cmd_data);
  if (nxpucihal_ctrl.hal_parse_enabled) {
    goto clean_and_return;
  }
  status = phTmlUwb_Write(
      (uint8_t*)nxpucihal_ctrl.p_cmd_data, (uint16_t)nxpucihal_ctrl.cmd_len,
      (pphTmlUwb_TransactCompletionCb_t)&phNxpUciHal_write_complete,
      (void*)&cb_data);


  if (status != UWBSTATUS_PENDING) {
    NXPLOG_UCIHAL_E("write_unlocked status error");
    data_len = 0;
    goto clean_and_return;
  }

  /* Wait for callback response */
  if (SEM_WAIT(cb_data)) {
    NXPLOG_UCIHAL_E("write_unlocked semaphore error");
    data_len = 0;
    goto clean_and_return;
  }

clean_and_return:
  phNxpUciHal_cleanup_cb_data(&cb_data);
  return data_len;
}

/******************************************************************************
 * Function         phNxpUciHal_write_complete
 *
 * Description      This function handles write callback.
 *
 * Returns          void.
 *
 ******************************************************************************/
static void phNxpUciHal_write_complete(void* pContext,
                                       phTmlUwb_TransactInfo_t* pInfo) {
  phNxpUciHal_Sem_t* p_cb_data = (phNxpUciHal_Sem_t*)pContext;

  if (pInfo->wStatus == UWBSTATUS_SUCCESS) {
    NXPLOG_UCIHAL_D("write successful status = 0x%x", pInfo->wStatus);
  } else {
    NXPLOG_UCIHAL_E("write error status = 0x%x", pInfo->wStatus);
  }
  p_cb_data->status = pInfo->wStatus;

  SEM_POST(p_cb_data);

  return;
}

/******************************************************************************
 * Function         phNxpUciHal_parse_get_capsInfo
 *
 * Description      parse the caps info response as per FIRA 2.0.
 *
 * Returns          void.
 *
 ******************************************************************************/
void phNxpUciHal_parse_get_capsInfo(uint16_t data_len, uint8_t *p_data) {
  uint8_t *p = p_data;
  uint8_t pDeviceCapsInfo[UCI_MAX_DATA_LEN];
  uint8_t *pp = pDeviceCapsInfo;
  uint8_t tagId = 0, subTagId = 0, len = 0;
  uint8_t mt = 0, gid = 0, oid = 0;
  uint8_t capsLen = p_data[5];
  uint8_t dataLen = p_data[3];
  mt = (*(p_data)&UCI_MT_MASK) >> UCI_MT_SHIFT;
  gid = p_data[0] & UCI_GID_MASK;
  oid = p_data[1] & UCI_OID_MASK;
  uint8_t *p_caps_value;
  if (mt == UCI_MT_RSP) {
    if ((gid == UCI_GID_CORE) && (oid == UCI_MSG_CORE_GET_CAPS_INFO)) {
      if (p_data[4] == 0) {
          for (uint16_t index = 6; index < data_len;) {
            tagId = p_data[index++];
            if (tagId != 0xE0 && tagId != 0xE3) {
              len = p_data[index++];
              /* b0 = Azimuth AoA -90 degree to 90 degree
               * b1 = Azimuth AoA -180 degree to 180 degree
               * b2 = Elevation AoA
               * b3 = AoA FOM
               */
              if (AOA_SUPPORT_TAG_ID == tagId) {
                if (isAntennaRxPairDefined) {
                  if (numberOfAntennaPairs == 1) {
                    *p_caps_value = 1;
                  } else if (numberOfAntennaPairs > 1) {
                    // If antenna pair more than 1 then it will support both b0
                    // = Azimuth AoA -90° to 90° and b2=Elevation AoA then value
                    // will set to 5
                    *p_caps_value = 5;
                  }
                } else {
                  *p_caps_value = 0;
                }
              } else {
                p_caps_value = (uint8_t *)(p_data + index);
              }
              UINT8_TO_STREAM(pp, tagId);
              UINT8_TO_STREAM(pp, len);
              if (tagId == CCC_SUPPORTED_PROTOCOL_VERSIONS_ID) {
                UINT8_TO_STREAM(pp, p_caps_value[len - 1]);
                UINT8_TO_STREAM(pp, p_caps_value[0]);
              } else {
                ARRAY_TO_STREAM(pp, p_caps_value, len);
              }
              index = index + len;
            } else { // ignore vendor specific data
              if ((index + 1) >= data_len) {
                break;
              }
              subTagId = p_data[index++];
              if ((index + 1) > data_len) {
                break;
              }
              len = p_data[index++];
              index = index + len;
              capsLen--;
              dataLen =
                  dataLen - (len + 3); // from datalen substract tagId,
                                       // subTagId, len and value of config
            }
          }

          // mapping device caps according to Fira 2.0
          // TODO: Remove once FW support available
          std::array<uint8_t, NXP_MAX_CONFIG_STRING_LEN> buffer;
          buffer.fill(0);
          uint8_t *vendorConfig = NULL;
          long retlen = 0;
          int numberOfParams = 0;

          if (NxpConfig_GetByteArray(NAME_UWB_VENDOR_CAPABILITY,
                                         buffer.data(), buffer.size(),
                                         &retlen)) {
            if (retlen > 0) {
              vendorConfig = buffer.data();
              ARRAY_TO_STREAM(pp, vendorConfig, retlen);
              dataLen += retlen;

              // calculate number of params
              int index = 0, paramId, length;
              do {
                paramId = vendorConfig[index++];
                length = vendorConfig[index++];
                index = index + length;
                numberOfParams++;
              } while (index < retlen);

              NXPLOG_UCIHAL_D("Get caps read info from config file, length: "
                              "%ld, numberOfParams: %d",
                              retlen, numberOfParams);

              nxpucihal_ctrl.rx_data_len = UCI_MSG_HDR_SIZE + dataLen;
              UCI_MSG_BLD_HDR0(p, UCI_MT_RSP, UCI_GID_CORE);
              UCI_MSG_BLD_HDR1(p, UCI_MSG_CORE_GET_CAPS_INFO);
              UINT8_TO_STREAM(p, 0x00);
              UINT8_TO_STREAM(p, dataLen);
              UINT8_TO_STREAM(p, 0x00); // status
              UINT8_TO_STREAM(p, (capsLen + numberOfParams));
              ARRAY_TO_STREAM(p, pDeviceCapsInfo, dataLen);
            } else {
              NXPLOG_UCIHAL_E("Reading config file for %s failed!!!",
                              NAME_UWB_VENDOR_CAPABILITY);
            }
          }
      }
      phNxpUciHal_print_packet("RECV", nxpucihal_ctrl.p_rx_data,
                               nxpucihal_ctrl.rx_data_len);
    }
  }
}

/******************************************************************************
 * Function         phNxpUciHal_read_complete
 *
 * Description      This function is called whenever there is an UCI packet
 *                  received from UWBC. It could be RSP or NTF packet. This
 *                  function provide the received UCI packet to libuwb-uci
 *                  using data callback of libuwb-uci.
 *                  There is a pending read called from each
 *                  phNxpUciHal_read_complete so each a packet received from
 *                  UWBC can be provide to libuwb-uci.
 *
 * Returns          void.
 *
 ******************************************************************************/
void phNxpUciHal_read_complete(void* pContext,
                                      phTmlUwb_TransactInfo_t* pInfo) {
  tHAL_UWB_STATUS status;
  uint8_t gid = 0, oid = 0, pbf = 0, mt = 0;
  UNUSED(pContext);
  if (nxpucihal_ctrl.read_retry_cnt == 1) {
    nxpucihal_ctrl.read_retry_cnt = 0;
  }
  if (pInfo->wStatus == UWBSTATUS_SUCCESS) {
    NXPLOG_UCIHAL_D("read successful status = 0x%x", pInfo->wStatus);
    nxpucihal_ctrl.p_rx_data = pInfo->pBuff;
    nxpucihal_ctrl.rx_data_len = pInfo->wLength;

    mt = ((nxpucihal_ctrl.p_rx_data[0]) & UCI_MT_MASK) >> UCI_MT_SHIFT;
    gid = nxpucihal_ctrl.p_rx_data[0] & UCI_GID_MASK;
    oid = nxpucihal_ctrl.p_rx_data[1] & UCI_OID_MASK;
    pbf = (nxpucihal_ctrl.p_rx_data[0] & UCI_PBF_MASK) >> UCI_PBF_SHIFT;

    if (gid == UCI_GID_CORE && oid == UCI_OID_GET_CAPS_INFO) {
      phNxpUciHal_parse_get_capsInfo(nxpucihal_ctrl.rx_data_len,
                                     nxpucihal_ctrl.p_rx_data);
    }
    nxpucihal_ctrl.isSkipPacket = 0;
    phNxpUciHal_parse(nxpucihal_ctrl.rx_data_len, nxpucihal_ctrl.p_rx_data);

    phNxpUciHal_process_response();

    if(!uwb_device_initialized) {
      if((gid == UCI_GID_CORE) && (oid == UCI_MSG_CORE_DEVICE_STATUS_NTF)) {
        nxpucihal_ctrl.uwbc_device_state = nxpucihal_ctrl.p_rx_data[UCI_RESPONSE_STATUS_OFFSET];
        if(nxpucihal_ctrl.uwbc_device_state == UWB_DEVICE_INIT || nxpucihal_ctrl.uwbc_device_state == UWB_DEVICE_READY) {
          nxpucihal_ctrl.isSkipPacket = 1;
          SEM_POST(&(nxpucihal_ctrl.dev_status_ntf_wait));
        }
      }
      if ((gid == UCI_GID_PROPRIETARY) && (oid == UCI_MSG_BINDING_STATUS_NTF)) {
        nxpucihal_ctrl.uwb_binding_status =
            nxpucihal_ctrl.p_rx_data[UCI_RESPONSE_STATUS_OFFSET];
        nxpucihal_ctrl.isSkipPacket = 1;
        SEM_POST(&(nxpucihal_ctrl.uwb_binding_status_ntf_wait));
      }

      if ((mt == UCI_MT_NTF) && (gid == UCI_GID_PROPRIETARY_0X0F) &&
          (oid == UCI_MSG_UWB_ESE_BINDING_NTF)) {
        nxpucihal_ctrl.uwb_binding_count =
            nxpucihal_ctrl.p_rx_data[UCI_RESPONSE_STATUS_OFFSET + 1];
        nxpucihal_ctrl.uwb_binding_status =
            nxpucihal_ctrl.p_rx_data[UCI_RESPONSE_STATUS_OFFSET + 2];
        nxpucihal_ctrl.isSkipPacket = 1;
        SEM_POST(&(nxpucihal_ctrl.uwb_do_bind_ntf_wait));
      }

      if ((mt == UCI_MT_NTF) && (gid == UCI_GID_PROPRIETARY_0X0F) &&
          (oid == UWB_ESE_BINDING_CHECK_NTF)) {
        nxpucihal_ctrl.uwb_binding_status =
            nxpucihal_ctrl.p_rx_data[UCI_RESPONSE_STATUS_OFFSET];
        nxpucihal_ctrl.p_rx_data[0] = 0x6E;
        nxpucihal_ctrl.p_rx_data[1] = 0x06;
        nxpucihal_ctrl.p_rx_data[2] = 0x00;
        nxpucihal_ctrl.p_rx_data[3] = 0x01;
        nxpucihal_ctrl.p_rx_data[4] = nxpucihal_ctrl.uwb_binding_status;
        nxpucihal_ctrl.rx_data_len = 5;
        SEM_POST(&(nxpucihal_ctrl.uwb_get_binding_status_ntf_wait));
      }
    }

    if (nxpucihal_ctrl.isSkipPacket == 0) {
      if((nxpucihal_ctrl.p_rx_data[0x00] & 0xF0) == 0x40){
        if (nxpucihal_ctrl.hal_ext_enabled) {
          nxpucihal_ctrl.isSkipPacket = 1;
        }
        if(nxpucihal_ctrl.p_rx_data[UCI_RESPONSE_STATUS_OFFSET] == UCI_STATUS_OK){
          nxpucihal_ctrl.ext_cb_data.status = UWBSTATUS_SUCCESS;
          if((gid == UCI_GID_CORE) && (oid == UCI_MSG_CORE_DEVICE_INFO)) {
            gpCoreDeviceInfoRsp = (uint8_t*)malloc(sizeof(uint8_t) * nxpucihal_ctrl.rx_data_len);
            if(gpCoreDeviceInfoRsp != NULL) {
              memcpy(&gpCoreDeviceInfoRsp[0], &nxpucihal_ctrl.p_rx_data[0], nxpucihal_ctrl.rx_data_len);
            }
          }
        } else if ((gid == UCI_GID_CORE) && (oid == UCI_MSG_CORE_SET_CONFIG)){
          NXPLOG_UCIHAL_E(" status = 0x%x",nxpucihal_ctrl.p_rx_data[UCI_RESPONSE_STATUS_OFFSET]);
          /* check if any configurations are not supported then ignore the
           * UWBSTATUS_FEATURE_NOT_SUPPORTED status code*/
          nxpucihal_ctrl.ext_cb_data.status = phNxpUciHal_process_ext_rsp(nxpucihal_ctrl.rx_data_len, nxpucihal_ctrl.p_rx_data);
        } else {
          if (gid == UCI_GID_SESSION_MANAGE &&
              oid == UCI_MSG_SESSION_QUERY_DATA_SIZE) {
            nxpucihal_ctrl.ext_cb_data.status =
                nxpucihal_ctrl
                    .p_rx_data[UCI_MSG_SESSION_QUERY_DATA_SIZE_STATUS_OFFSET];
          } else {
            nxpucihal_ctrl.ext_cb_data.status = UWBSTATUS_FAILED;
            NXPLOG_UCIHAL_E(
                "command failed! status = 0x%x",
                nxpucihal_ctrl.p_rx_data[UCI_RESPONSE_STATUS_OFFSET]);
          }
        }
        usleep(1);
        SEM_POST(&(nxpucihal_ctrl.ext_cb_data));
      } else if (gid == UCI_GID_CORE && oid == UCI_MSG_CORE_GENERIC_ERROR_NTF &&
                 nxpucihal_ctrl.p_rx_data[4] == UCI_STATUS_COMMAND_RETRY) {
        if (nxpucihal_ctrl.hal_ext_enabled) {
          nxpucihal_ctrl.ext_cb_data.status = UWBSTATUS_COMMAND_RETRANSMIT;
        }
        SEM_POST(&(nxpucihal_ctrl.ext_cb_data));
      } else if (gid == UCI_GID_CORE && oid == UCI_MSG_CORE_GENERIC_ERROR_NTF &&
                 nxpucihal_ctrl.p_rx_data[4] == UCI_STATUS_INVALID_MSG_SIZE) {
        if (nxpucihal_ctrl.hal_ext_enabled) {
          nxpucihal_ctrl.ext_cb_data.status = UWBSTATUS_INVALID_COMMAND_LENGTH;
          nxpucihal_ctrl.isSkipPacket = 1;
        }
        SEM_POST(&(nxpucihal_ctrl.ext_cb_data));
      }
    } else if (nxpucihal_ctrl.hal_ext_enabled == 0) {
      SEM_POST(&(nxpucihal_ctrl.ext_cb_data));
    }
    if ((mt == UCI_MT_NTF) && (gid == UCI_GID_INTERNAL) &&
        (oid == UCI_EXT_PARAM_DBG_RFRAME_LOG_NTF)) {
      nxpucihal_ctrl.isSkipPacket = 1;
    }

    /* if Debug Notification, then skip sending to application */
    if(nxpucihal_ctrl.isSkipPacket == 0) {
      phNxpUciHal_print_response_status(nxpucihal_ctrl.p_rx_data, nxpucihal_ctrl.rx_data_len);
      /* Read successful, send the event to higher layer */
         if ((nxpucihal_ctrl.p_uwb_stack_data_cback != NULL) && (nxpucihal_ctrl.rx_data_len <= UCI_MAX_PAYLOAD_LEN)) {
        (*nxpucihal_ctrl.p_uwb_stack_data_cback)(nxpucihal_ctrl.rx_data_len,
                                                 nxpucihal_ctrl.p_rx_data);
         }
    }
  } else {
    NXPLOG_UCIHAL_E("read error status = 0x%x", pInfo->wStatus);
  }

  if (nxpucihal_ctrl.halStatus == HAL_STATUS_CLOSE) {
    return;
  }
  /* Disable junk data check for each UCI packet*/
  if(nxpucihal_ctrl.fw_dwnld_mode) {
    if((gid == UCI_GID_CORE) && (oid == UCI_MSG_CORE_DEVICE_STATUS_NTF)){
      nxpucihal_ctrl.fw_dwnld_mode = false;
    }
  }
  /* Read again because read must be pending always.*/
  status = phTmlUwb_Read(
      Rx_data, UCI_MAX_DATA_LEN,
      (pphTmlUwb_TransactCompletionCb_t)&phNxpUciHal_read_complete, NULL);
  if (status != UWBSTATUS_PENDING) {
    NXPLOG_UCIHAL_E("read status error status = %x", status);
    /* TODO: Not sure how to handle this ? */
  }
  return;
}

/******************************************************************************
 * Function         phNxpUciHal_close
 *
 * Description      This function close the UWBC interface and free all
 *                  resources.This is called by libuwb-uci on UWB service stop.
 *
 * Returns          Always return UWBSTATUS_SUCCESS (0).
 *
 ******************************************************************************/
tHAL_UWB_STATUS phNxpUciHal_close() {
  tHAL_UWB_STATUS status;
  if (nxpucihal_ctrl.halStatus == HAL_STATUS_CLOSE) {
    NXPLOG_UCIHAL_E("phNxpUciHal_close is already closed, ignoring close");
    return UWBSTATUS_FAILED;
  }

  uwb_device_initialized = false;

  CONCURRENCY_LOCK();

  nxpucihal_ctrl.halStatus = HAL_STATUS_CLOSE;

  if (NULL != gpphTmlUwb_Context->pDevHandle) {
    phNxpUciHal_close_complete(UWBSTATUS_SUCCESS);
    /* Abort any pending read and write */
    status = phTmlUwb_ReadAbort();
    status = phTmlUwb_WriteAbort();

    phOsalUwb_Timer_Cleanup();

    status = phTmlUwb_Shutdown();

    phDal4Uwb_msgrelease(nxpucihal_ctrl.gDrvCfg.nClientId);

    memset(&nxpucihal_ctrl, 0x00, sizeof(nxpucihal_ctrl));

    NXPLOG_UCIHAL_D("phNxpUciHal_close - phOsalUwb_DeInit completed");
  }

  CONCURRENCY_UNLOCK();

  phNxpUciHal_cleanup_monitor();

  /* Return success always */
  return UWBSTATUS_SUCCESS;
}
/******************************************************************************
 * Function         phNxpUciHal_close_complete
 *
 * Description      This function inform libuwb-uci about result of
 *                  phNxpUciHal_close.
 *
 * Returns          void.
 *
 ******************************************************************************/
void phNxpUciHal_close_complete(tHAL_UWB_STATUS status) {
  static phLibUwb_Message_t msg;

  if (status == UWBSTATUS_SUCCESS) {
    msg.eMsgType = UCI_HAL_CLOSE_CPLT_MSG;
  } else {
    msg.eMsgType = UCI_HAL_ERROR_MSG;
  }
  msg.pMsgData = NULL;
  msg.Size = 0;

  phTmlUwb_DeferredCall(gpphTmlUwb_Context->dwCallbackThreadId, &msg);

  return;
}

/******************************************************************************
 * Function         phNxpUciHal_init_complete
 *
 * Description      This function inform libuwb-uci about result of
 *                  phNxpUciHal_coreInitialization.
 *
 * Returns          void.
 *
 ******************************************************************************/
void phNxpUciHal_init_complete(tHAL_UWB_STATUS status) {
  static phLibUwb_Message_t msg;

  if (status == UWBSTATUS_SUCCESS) {
    msg.eMsgType = UCI_HAL_INIT_CPLT_MSG;
  } else {
    msg.eMsgType = UCI_HAL_ERROR_MSG;
  }
  msg.pMsgData = NULL;
  msg.Size = 0;

  phTmlUwb_DeferredCall(gpphTmlUwb_Context->dwCallbackThreadId, &msg);

  return;
}

/******************************************************************************
 * Function         phNxpUciHal_parseCoreDeviceInfoRsp
 *
 * Description      This function parse Core device Info response.
 *
 * Returns          void.
 *
 ******************************************************************************/
static void phNxpUciHal_parseCoreDeviceInfoRsp(uint8_t *fwBootMode, uint8_t *device) {
  NXPLOG_UCIHAL_D("phNxpUciHal_parseCoreDeviceInfoRsp Enter..");
  uint8_t index = 13; // Excluding the header and Versions
  uint8_t paramId = 0;
  uint8_t length = 0;

  if(fwBootMode == NULL || gpCoreDeviceInfoRsp == NULL){
    return;
  }

  uint8_t len = gpCoreDeviceInfoRsp[index++];
  len = len + index;
  while (index < len) {
    paramId = gpCoreDeviceInfoRsp[index++];
    length = gpCoreDeviceInfoRsp[index++];
    if (paramId == DEVICE_NAME_PARAM_ID) {
      *device  = gpCoreDeviceInfoRsp[index + 5];
      NXPLOG_UCIHAL_D("phNxpUciHal_parseCoreDeviceInfoRsp DeviceType %c", *device);
    } else if (paramId == FW_VERSION_PARAM_ID) {
      fw_version.major_version = gpCoreDeviceInfoRsp[index];
      fw_version.minor_version = gpCoreDeviceInfoRsp[index + 1];
      fw_version.rc_version = gpCoreDeviceInfoRsp[index + 2];
    } else if (paramId == FW_BOOT_MODE_PARAM_ID) {
      *fwBootMode = gpCoreDeviceInfoRsp[index];
      break;
    }
    index = index + length;
  }
  if (gpCoreDeviceInfoRsp != NULL) {
    free(gpCoreDeviceInfoRsp);
    gpCoreDeviceInfoRsp = NULL;
  }
  return;
}

/******************************************************************************
 * Function         phNxpUciHal_sendGetCoreDeviceInfo
 *
 * Description      This function send Core device Info command.
 *
 * Returns          status.
 *
 ******************************************************************************/
uint8_t phNxpUciHal_sendGetCoreDeviceInfo(){
  std::array<uint8_t, NXP_MAX_CONFIG_STRING_LEN> buffer;
  buffer.fill(0);
  const uint8_t getCoreDeviceInfoConfig[] = {0x20, 0x02, 0x00, 0x00};
  uint8_t getCoreDeviceInfoCmdLen = 4;
  tHAL_UWB_STATUS status = phNxpUciHal_send_ext_cmd(getCoreDeviceInfoCmdLen, getCoreDeviceInfoConfig);
  if(status != UWBSTATUS_SUCCESS) {
    return status;
  } else {
    phNxpUciHal_parseCoreDeviceInfoRsp(&nxpucihal_ctrl.fw_boot_mode,
                                       &deviceType);
    NXPLOG_UCIHAL_D(" fw_boot_mode is : %d, deviceType is :%d",
                    nxpucihal_ctrl.fw_boot_mode, deviceType);
  }
  return status;
}

/******************************************************************************
 * Function         parseAntennaConfig
 *
 * Description      This function parse the antenna config and update required
 *                  params
 *
 * Returns          void
 *
 ******************************************************************************/
void parseAntennaConfig(uint16_t dataLength, const uint8_t *data) {
  if (dataLength > 0) {
    uint8_t index =
        UCI_MSG_HDR_SIZE + 1; // Excluding the header and number of params
    uint8_t tagId, subTagId;
    int length;
    while (index < dataLength) {
      tagId = data[index++];
      subTagId = data[index++];
      length = data[index++];
      if ((ANTENNA_RX_PAIR_DEFINE_TAG_ID == tagId) &&
          (ANTENNA_RX_PAIR_DEFINE_SUB_TAG_ID == subTagId)) {
        isAntennaRxPairDefined = true;
        numberOfAntennaPairs = data[index];
        NXPLOG_UCIHAL_D("numberOfAntennaPairs:%d", numberOfAntennaPairs);
        break;
      } else {
        index = index + length;
      }
    };
  } else {
    NXPLOG_UCIHAL_E("Reading config file for %s failed!!!",
                    NAME_UWB_CORE_EXT_DEVICE_DEFAULT_CONFIG);
  }
}

/******************************************************************************
 * Function         phNxpUciHal_applyVendorConfig
 *
 * Description      This function applies the vendor config from config file
 *
 * Returns          status
 *
 ******************************************************************************/
tHAL_UWB_STATUS phNxpUciHal_applyVendorConfig() {
  NXPLOG_UCIHAL_D(" phNxpUciHal_applyVendorConfig Enter..");
  std::array<uint8_t, NXP_MAX_CONFIG_STRING_LEN> buffer;
  uint8_t *vendorConfig = NULL;
  tHAL_UWB_STATUS status;
  buffer.fill(0);
  long retlen = 0;
  if (nxpucihal_ctrl.fw_boot_mode == USER_FW_BOOT_MODE) {
    if (NxpConfig_GetByteArray(NAME_UWB_USER_FW_BOOT_MODE_CONFIG,
                               buffer.data(), buffer.size(),
                               &retlen)) {
      if ((retlen > 0) && (retlen <= UCI_MAX_DATA_LEN)) {
        vendorConfig = buffer.data();
        status = phNxpUciHal_send_ext_cmd(retlen, vendorConfig);
        NXPLOG_UCIHAL_D(
            " phNxpUciHal_send_ext_cmd :: status value for %s is %d ",
            NAME_UWB_USER_FW_BOOT_MODE_CONFIG, status);
        if (status != UWBSTATUS_SUCCESS) {
          return status;
        }
      }
    }
  }
  if (NxpConfig_GetByteArray(NAME_NXP_UWB_EXTENDED_NTF_CONFIG,
                                 buffer.data(), buffer.size(),
                                 &retlen)) {
    if (retlen > 0) {
      vendorConfig = buffer.data();
      status = phNxpUciHal_send_ext_cmd(retlen, vendorConfig);
      NXPLOG_UCIHAL_D(" phNxpUciHal_send_ext_cmd :: status value for %s is %d ",
                      NAME_NXP_UWB_EXTENDED_NTF_CONFIG, status);
      if (status != UWBSTATUS_SUCCESS) {
        return status;
      }
    }
  }
  if (deviceType == SR1xxT) {
    if (NxpConfig_GetByteArray(NAME_UWB_CORE_EXT_DEVICE_SR1XX_T_CONFIG,
                               buffer.data(), buffer.size(),
                               &retlen)) {
      if (retlen > 0) {
        vendorConfig = buffer.data();
        status = phNxpUciHal_send_ext_cmd(retlen, vendorConfig);
        NXPLOG_UCIHAL_D(
            " phNxpUciHal_send_ext_cmd :: status value for %s is %d ",
            NAME_UWB_CORE_EXT_DEVICE_SR1XX_T_CONFIG, status);

        // AOA support handling
        parseAntennaConfig(retlen, vendorConfig);

        if (status != UWBSTATUS_SUCCESS) {
          return status;
        }
      }
    }
  } else if (deviceType == SR1xxS) {
    if (NxpConfig_GetByteArray(NAME_UWB_CORE_EXT_DEVICE_SR1XX_S_CONFIG,
                            buffer.data(), buffer.size(),
                            &retlen)) {
      if (retlen > 0) {
        vendorConfig = buffer.data();
        status = phNxpUciHal_send_ext_cmd(retlen, vendorConfig);
        NXPLOG_UCIHAL_D(
            " phNxpUciHal_send_ext_cmd :: status value for %s is %d ",
            NAME_UWB_CORE_EXT_DEVICE_SR1XX_S_CONFIG, status);

        // AOA support handling
        parseAntennaConfig(retlen, vendorConfig);

        if (status != UWBSTATUS_SUCCESS) {
          return status;
        }
      }
    }
  } else {
    NXPLOG_UCIHAL_D("phNxpUciHal_sendGetCoreDeviceInfo deviceType default");
    if (NxpConfig_GetByteArray(NAME_UWB_CORE_EXT_DEVICE_DEFAULT_CONFIG,
                                   buffer.data(), buffer.size(),
                                   &retlen)) {
      if (retlen > 0) {
        vendorConfig = buffer.data();
        status = phNxpUciHal_send_ext_cmd(retlen, vendorConfig);
        NXPLOG_UCIHAL_D(
            " phNxpUciHal_send_ext_cmd :: status value for %s is %d ",
            NAME_UWB_CORE_EXT_DEVICE_DEFAULT_CONFIG, status);

        // AOA support handling
        parseAntennaConfig(retlen, vendorConfig);

        if (status != UWBSTATUS_SUCCESS) {
          return status;
        }
      }
    }
  }
  if (NxpConfig_GetByteArray(NAME_NXP_UWB_XTAL_38MHZ_CONFIG,
                                 buffer.data(), buffer.size(),
                                 &retlen)) {
    if (retlen > 0) {
      vendorConfig = buffer.data();
      status = phNxpUciHal_send_ext_cmd(retlen, vendorConfig);
      NXPLOG_UCIHAL_D(" phNxpUciHal_send_ext_cmd :: status value for %s is %d ",
                      NAME_NXP_UWB_XTAL_38MHZ_CONFIG, status);
      if (status != UWBSTATUS_SUCCESS) {
        return status;
      }
    }
  }
  for(int i = 1;i <= 10;i++) {
    std::string str = NAME_NXP_CORE_CONF_BLK;
    std::string value = std::to_string(i);
    std::string name = str + value;
    NXPLOG_UCIHAL_D(" phNxpUciHal_applyVendorConfig :: Name of the config block is %s", name.c_str());
    if (NxpConfig_GetByteArray(name.c_str(), buffer.data(), buffer.size(), &retlen)) {
      if ((retlen > 0) && (retlen <= UCI_MAX_DATA_LEN)) {
        vendorConfig = buffer.data();
        status = phNxpUciHal_send_ext_cmd(retlen,vendorConfig);
        NXPLOG_UCIHAL_D(" phNxpUciHal_send_ext_cmd :: status value for %s is %d ", name.c_str(),status);
        if(status != UWBSTATUS_SUCCESS) {
          return status;
        }
      }
    } else {
      NXPLOG_UCIHAL_D(
          " phNxpUciHal_applyVendorConfig::%s not available in the config file",
          name.c_str());
    }
  }
  return UWBSTATUS_SUCCESS;
}

/******************************************************************************
 * Function         phNxpUciHal_uwb_reset
 *
 * Description      This function will send UWB reset command
 *
 * Returns          status
 *
 ******************************************************************************/
tHAL_UWB_STATUS phNxpUciHal_uwb_reset() {
  tHAL_UWB_STATUS status;
  uint8_t buffer[] = {0x20, 0x00, 0x00, 0x01, 0x00};
  status = phNxpUciHal_send_ext_cmd(sizeof(buffer), buffer);
  if(status != UWBSTATUS_SUCCESS) {
    return status;
  }
  return UWBSTATUS_SUCCESS;
}

/******************************************************************************
 * Function         phNxpUciHal_disable_dpd
 *
 * Description      This function sends disable dpd command
 *
 * Returns          status
 *
 ******************************************************************************/
static tHAL_UWB_STATUS phNxpUciHal_disable_dpd() {
  tHAL_UWB_STATUS status;
  uint8_t buffer[] = {0x20, 0x04, 0x00, 0x04, 0x01, 0x01, 0x01, 0x00};
  status = phNxpUciHal_send_ext_cmd(sizeof(buffer), buffer);
  if (status != UWBSTATUS_SUCCESS) {
    return status;
  }
  return UWBSTATUS_SUCCESS;
}

/******************************************************************************
 * Function         phNxpUciHal_do_bind
 *
 * Description      This function sends bind request command
 *
 * Returns          status
 *
 ******************************************************************************/
static tHAL_UWB_STATUS phNxpUciHal_do_bind() {
  tHAL_UWB_STATUS status;
  uint8_t buffer[] = {0x2F, 0x31, 0x00, 0x00};
  status = phNxpUciHal_send_ext_cmd(sizeof(buffer), buffer);
  if (status != UWBSTATUS_SUCCESS) {
    return status;
  }

  phNxpUciHal_sem_timed_wait(&nxpucihal_ctrl.uwb_do_bind_ntf_wait);
  if (nxpucihal_ctrl.uwb_do_bind_ntf_wait.status != UWBSTATUS_SUCCESS) {
    NXPLOG_UCIHAL_E("uwb_do_bind_ntf_wait semaphore timed out");
    return UWBSTATUS_FAILED;
  }
  return UWBSTATUS_SUCCESS;
}

/******************************************************************************
 * Function         phNxpUciHal_get_binding_status
 *
 * Description      This function request for ese binding status
 *
 * Returns          status
 *
 ******************************************************************************/
static tHAL_UWB_STATUS phNxpUciHal_get_binding_status() {
  tHAL_UWB_STATUS status;
  uint8_t lock_cmd[] = {0x2F, 0x32, 0x00, 0x00};
  status = phNxpUciHal_send_ext_cmd(sizeof(lock_cmd), lock_cmd);
  if (status != UWBSTATUS_SUCCESS) {
    return status;
  }
  phNxpUciHal_sem_timed_wait(&nxpucihal_ctrl.uwb_get_binding_status_ntf_wait);
  if (nxpucihal_ctrl.uwb_get_binding_status_ntf_wait.status !=
      UWBSTATUS_SUCCESS) {
    NXPLOG_UCIHAL_E("uwb_get_binding_status_ntf_wait semaphore timed out");
    return UWBSTATUS_FAILED;
  }
  return UWBSTATUS_SUCCESS;
}

/******************************************************************************
 * Function         phNxpUciHal_parse_binding_status_ntf
 *
 * Description      This function parses the binding status notification
 *
 * Returns          status
 *
 ******************************************************************************/
static tHAL_UWB_STATUS phNxpUciHal_parse_binding_status_ntf() {
  tHAL_UWB_STATUS status = UWBSTATUS_SUCCESS;
  uint8_t binding_status = nxpucihal_ctrl.uwb_binding_status;
  switch (binding_status) {
  case UWB_DEVICE_NOT_BOUND: {
    /* disable dpd */
    status = phNxpUciHal_disable_dpd();
    if (status != UWBSTATUS_SUCCESS) {
      return status;
    }

    /* perform bind using do_bind command */
    status = phNxpUciHal_do_bind();
    if (status != UWBSTATUS_SUCCESS) {
      return status;
    }

    if (nxpucihal_ctrl.uwb_binding_status == UWB_DEVICE_BOUND_UNLOCKED &&
        nxpucihal_ctrl.uwb_binding_count < 3) {
      /* perform lock using get_binding_status command */
      status = phNxpUciHal_get_binding_status();
      if (status != UWBSTATUS_SUCCESS) {
        return status;
      }
    }
  } break;

  case UWB_DEVICE_BOUND_UNLOCKED: {
    /* disable dpd */
    uint8_t status = phNxpUciHal_disable_dpd();
    if (status != UWBSTATUS_SUCCESS) {
      return status;
    }

    /* perform lock using get_binding_status command */
    status = phNxpUciHal_get_binding_status();
    if (status != UWBSTATUS_SUCCESS) {
      return status;
    }
    phNxpUciHal_sem_timed_wait(&nxpucihal_ctrl.uwb_get_binding_status_ntf_wait);
    if (nxpucihal_ctrl.uwb_get_binding_status_ntf_wait.status !=
        UWBSTATUS_SUCCESS) {
      NXPLOG_UCIHAL_E("uwb_get_binding_status_ntf_wait semaphore timed out");
      /* Sending originial binding status notification to upper layer */
      uint8_t data_len = 5;
      uint8_t buffer[data_len];
      buffer[0] = 0x6E;
      buffer[1] = 0x06;
      buffer[2] = 0x00;
      buffer[3] = 0x01;
      buffer[4] = nxpucihal_ctrl.uwb_binding_status;
      if (nxpucihal_ctrl.p_uwb_stack_data_cback != NULL) {
        (*nxpucihal_ctrl.p_uwb_stack_data_cback)(data_len, buffer);
      }
    }
  } break;

  case UWB_DEVICE_BOUND_LOCKED: {
    // do nothing
  } break;

  default: {
    NXPLOG_UCIHAL_E("[%s] unknown binding status:%d", __func__, binding_status);
  }
  }

  return status;
}

/******************************************************************************
 * Function         phNxpUciHal_coreInitialization
 *
 * Description      This function performs core initialization
 *
 * Returns          status
 *
 ******************************************************************************/
tHAL_UWB_STATUS phNxpUciHal_coreInitialization() {
  tHAL_UWB_STATUS status;
  uint8_t fwd_retry_count = 0;
  uint8_t dev_ready_ntf[] = {0x60, 0x01, 0x00, 0x01, 0x01};
  nxpucihal_ctrl.isRecoveryTimerStarted = false;

  if (nxpucihal_ctrl.halStatus != HAL_STATUS_OPEN) {
    NXPLOG_UCIHAL_E("HAL not initialized");
    return UWBSTATUS_FAILED;
  }

  NXPLOG_UCIHAL_D(" Start FW download");
  /* Create the local semaphore */
  if (phNxpUciHal_init_cb_data(&nxpucihal_ctrl.dev_status_ntf_wait, NULL) !=
      UWBSTATUS_SUCCESS) {
    NXPLOG_UCIHAL_E("Create dev_status_ntf_wait failed");
    return UWBSTATUS_FAILED;
  }

  if (phNxpUciHal_init_cb_data(&nxpucihal_ctrl.uwb_binding_status_ntf_wait, NULL) !=
      UWBSTATUS_SUCCESS) {
    NXPLOG_UCIHAL_E("Create uwb_binding_status_ntf_wait failed");
    return UWBSTATUS_FAILED;
  }

  if (phNxpUciHal_init_cb_data(&nxpucihal_ctrl.uwb_get_binding_status_ntf_wait,
                               NULL) != UWBSTATUS_SUCCESS) {
    NXPLOG_UCIHAL_E("Create uwb_get_binding_status_ntf_wait failed");
    return UWBSTATUS_FAILED;
  }

  if (phNxpUciHal_init_cb_data(&nxpucihal_ctrl.uwb_do_bind_ntf_wait, NULL) !=
      UWBSTATUS_SUCCESS) {
    NXPLOG_UCIHAL_E("Create uwb_do_bind_ntf_wait failed");
    return UWBSTATUS_FAILED;
  }
  nxpucihal_ctrl.fw_dwnld_mode = true; /* system in FW download mode*/
  uwb_device_initialized = false;

fwd_retry:
      nxpucihal_ctrl.uwbc_device_state = UWB_DEVICE_ERROR;
      nxpucihal_ctrl.uwb_binding_status = UWB_DEVICE_UNKNOWN;
      status = phNxpUciHal_fw_download();
      if(status == UWBSTATUS_SUCCESS) {
          status = phTmlUwb_Read( Rx_data, UCI_MAX_DATA_LEN,
                    (pphTmlUwb_TransactCompletionCb_t)&phNxpUciHal_read_complete, NULL);
          if (status != UWBSTATUS_PENDING) {
            NXPLOG_UCIHAL_E("read status error status = %x", status);
            goto failure;
          }
          phNxpUciHal_sem_timed_wait(&nxpucihal_ctrl.dev_status_ntf_wait);
          if (nxpucihal_ctrl.dev_status_ntf_wait.status != UWBSTATUS_SUCCESS) {
            NXPLOG_UCIHAL_E("UWB_DEVICE_INIT dev_status_ntf_wait semaphore timed out");
            goto failure;
          }
          if(nxpucihal_ctrl.uwbc_device_state != UWB_DEVICE_INIT) {
            NXPLOG_UCIHAL_E("UWB_DEVICE_INIT not received uwbc_device_state = %x",nxpucihal_ctrl.uwbc_device_state);
            goto failure;
          }
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
          status = phNxpUciHal_sendGetCoreDeviceInfo();
          NXPLOG_UCIHAL_D("phNxpUciHal_sendGetCoreDeviceInfo status %d ",
                          status);
          if (status != UWBSTATUS_SUCCESS) {
            return status;
          }
          phNxpUciHal_sem_timed_wait(&nxpucihal_ctrl.uwb_binding_status_ntf_wait);
          if (nxpucihal_ctrl.uwb_binding_status_ntf_wait.status == UWBSTATUS_SUCCESS) {
            NXPLOG_UCIHAL_D("binding status notification received");
            if (nxpucihal_ctrl.fw_boot_mode == USER_FW_BOOT_MODE) {
        status = phNxpUciHal_parse_binding_status_ntf();
        if (status != UWBSTATUS_SUCCESS) {
          NXPLOG_UCIHAL_E("binding failed with status %d", status);
        }
            }
          } else {
            NXPLOG_UCIHAL_D("%s:binding status notification timed out",
                            __func__);
          }
          status = phNxpUciHal_applyVendorConfig();
          if (status != UWBSTATUS_SUCCESS) {
            NXPLOG_UCIHAL_E("%s: Apply vendor Config Failed", __func__);
            goto failure;
          }
          phNxpUciHal_extcal_handle_coreinit();

          uwb_device_initialized = true;
          phNxpUciHal_getVersionInfo();
          phNxpUciHal_init_complete(UWBSTATUS_SUCCESS);
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
      if (nxpucihal_ctrl.p_uwb_stack_data_cback != NULL) {
        (*nxpucihal_ctrl.p_uwb_stack_data_cback)((sizeof(dev_ready_ntf)/sizeof(uint8_t)),
                                                 dev_ready_ntf);
      }
      phNxpUciHal_cleanup_cb_data(&nxpucihal_ctrl.dev_status_ntf_wait);
      phNxpUciHal_cleanup_cb_data(
          &nxpucihal_ctrl.uwb_get_binding_status_ntf_wait);
      phNxpUciHal_cleanup_cb_data(&nxpucihal_ctrl.uwb_do_bind_ntf_wait);
      phNxpUciHal_cleanup_cb_data(&nxpucihal_ctrl.uwb_binding_status_ntf_wait);
      return status;
    failure:
        phNxpUciHal_init_complete(UWBSTATUS_FAILED);
        phNxpUciHal_cleanup_cb_data(&nxpucihal_ctrl.dev_status_ntf_wait);
        phNxpUciHal_cleanup_cb_data(
            &nxpucihal_ctrl.uwb_get_binding_status_ntf_wait);
        phNxpUciHal_cleanup_cb_data(&nxpucihal_ctrl.uwb_do_bind_ntf_wait);
        phNxpUciHal_cleanup_cb_data(&nxpucihal_ctrl.uwb_binding_status_ntf_wait);
        return UWBSTATUS_FAILED;
}

/******************************************************************************
 * Function         phNxpUciHal_sessionInitialization
 *
 * Description      This function performs session initialization
 *
 * Returns          status
 *
 ******************************************************************************/
tHAL_UWB_STATUS phNxpUciHal_sessionInitialization(uint32_t sessionId) {
  NXPLOG_UCIHAL_D(" %s: Enter", __func__);
  std::array<uint8_t, NXP_MAX_CONFIG_STRING_LEN> buffer;
  uint8_t vendorConfig[NXP_MAX_CONFIG_STRING_LEN] = {0x2F, 0x00, 0x00};
  tHAL_UWB_STATUS status = UWBSTATUS_SUCCESS;
  buffer.fill(0);
  int max_config_length = NXP_MAX_CONFIG_STRING_LEN - UCI_MSG_HDR_SIZE
                            - sizeof(sessionId);
  long retlen = 0, cmdlen = 0;
  bool appConfigStatus = false;

  if (nxpucihal_ctrl.halStatus != HAL_STATUS_OPEN) {
    NXPLOG_UCIHAL_E("HAL not initialized");
    return UWBSTATUS_FAILED;
  }
  if(deviceType == SR1xxT) {
    appConfigStatus = NxpConfig_GetByteArray(NAME_NXP_UWB_EXT_APP_SR1XX_T_CONFIG,
                                   buffer.data(), buffer.size(),
                                   &retlen);
  } else if (deviceType == SR1xxS) {
    appConfigStatus = NxpConfig_GetByteArray(NAME_NXP_UWB_EXT_APP_SR1XX_S_CONFIG,
                                   buffer.data(), buffer.size(),
                                   &retlen);
  } else {
    appConfigStatus = NxpConfig_GetByteArray(NAME_NXP_UWB_EXT_APP_DEFAULT_CONFIG,
                                   buffer.data(), buffer.size(),
                                   &retlen);
  }

  if (appConfigStatus) {
    if ((retlen > 0) && (retlen <= max_config_length)) {
      vendorConfig[3] = sizeof(sessionId) + retlen;
      memcpy(vendorConfig + 4, &sessionId, sizeof(sessionId));
      memcpy(vendorConfig + 8, buffer.data(), retlen);
      cmdlen = UCI_MSG_HDR_SIZE + sizeof(sessionId) + retlen;
      status = phNxpUciHal_send_ext_cmd(cmdlen, vendorConfig);
      if (status != UWBSTATUS_SUCCESS) {
        NXPLOG_UCIHAL_D(" %s: Apply vendor App Config Failed", __func__);
        return UWBSTATUS_SUCCESS;
      }
    } else {
      NXPLOG_UCIHAL_D(" %s: Invalid retlen", __func__);
      return UWBSTATUS_SUCCESS;
    }
  }
  return status;
}

/******************************************************************************
 * Function         phNxpUciHal_print_response_status
 *
 * Description      This function logs the response status
 *
 * Returns          status
 *
 ******************************************************************************/
static void phNxpUciHal_print_response_status(uint8_t* p_rx_data, uint16_t p_len) {
  uint8_t mt;
  int status_byte;
  const uint8_t response_buf[][30] = {"STATUS_OK",
                                      "STATUS_REJECTED",
                                      "STATUS_FAILED",
                                      "STATUS_SYNTAX_ERROR",
                                      "STATUS_INVALID_PARAM",
                                      "STATUS_INVALID_RANGE",
                                      "STATUS_INAVALID_MSG_SIZE",
                                      "STATUS_UNKNOWN_GID",
                                      "STATUS_UNKNOWN_OID",
                                      "STATUS_RFU",
                                      "STATUS_READ_ONLY",
                                      "STATUS_COMMAND_RETRY",
                                      "STATUS_UNKNOWN"};
  if(p_len > UCI_PKT_HDR_LEN) {
    mt = ((p_rx_data[0]) & UCI_MT_MASK) >> UCI_MT_SHIFT;
    status_byte = p_rx_data[UCI_RESPONSE_STATUS_OFFSET];
    if((mt == UCI_MT_RSP) && (status_byte <= MAX_RESPONSE_STATUS)) {
      NXPLOG_UCIHAL_D(" %s: Response Status = %s", __func__,
                      response_buf[status_byte]);
    }
  }
}

/******************************************************************************
 * Function         phNxpUciHal_GetMwVersion
 *
 * Description      This function gets the middleware version
 *
 * Returns          phNxpUciHal_MW_Version_t
 *
 ******************************************************************************/
phNxpUciHal_MW_Version_t phNxpUciHal_GetMwVersion() {
  phNxpUciHal_MW_Version_t mwVer;
  mwVer.validation = NXP_CHIP_SR100;
  mwVer.android_version = NXP_ANDROID_VERSION;
  NXPLOG_UCIHAL_D("0x%x:UWB MW Major Version:", UWB_NXP_MW_VERSION_MAJ);
  NXPLOG_UCIHAL_D("0x%x:UWB MW Minor Version:", UWB_NXP_MW_VERSION_MIN);
  mwVer.major_version = UWB_NXP_MW_VERSION_MAJ;
  mwVer.minor_version = UWB_NXP_MW_VERSION_MIN;
  mwVer.rc_version = UWB_NXP_ANDROID_MW_RC_VERSION;
  mwVer.mw_drop = UWB_NXP_ANDROID_MW_DROP_VERSION;
  return mwVer;
}

/******************************************************************************
 * Function         phNxpUciHal_getVersionInfo
 *
 * Description      This function request for version information
 *
 * Returns          void
 *
 ******************************************************************************/
void phNxpUciHal_getVersionInfo() {
  phNxpUciHal_MW_Version_t mwVersion = phNxpUciHal_GetMwVersion();
  if (mwVersion.rc_version) { /* for RC release*/
    ALOGI("MW Version: UWB_SW_Android_U_HKY_D%02x.%02x_RC%02x",
          mwVersion.major_version, mwVersion.minor_version,
          mwVersion.rc_version);
  } else if (mwVersion.mw_drop) { /* For Drops */
    ALOGI("MW Version: UWB_SW_Android_U_HKY_D%02x.%02x_DROP%02x",
          mwVersion.major_version, mwVersion.minor_version, mwVersion.mw_drop);
  } else { /* for Major Releases*/
    ALOGI("MW Version: UWB_SW_Android_U_HKY_D%02x.%02x",
          mwVersion.major_version, mwVersion.minor_version);
  }

  if (fw_version.rc_version) {
    ALOGI("FW Version: %02x.%02x_RC%02x", fw_version.major_version,
          fw_version.minor_version, fw_version.rc_version);
  } else {
    ALOGI("FW Version: %02x.%02x", fw_version.major_version,
          fw_version.minor_version);
  }
}
