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
#include <string.h>
#include <sys/stat.h>

#include <atomic>
#include <bitset>

#include <cutils/properties.h>

#include "phDal4Uwb_messageQueueLib.h"
#include "phNxpConfig.h"
#include "phNxpLog.h"
#include "phNxpUciHal_ext.h"
#include "phNxpUciHal.h"
#include "phTmlUwb.h"
#include "phUwbCommon.h"

/* Timeout value to wait for response from DEVICE_TYPE_SR1xx */
#define HAL_EXTNS_WRITE_RSP_TIMEOUT (100)
#define HAL_HW_RESET_NTF_TIMEOUT 10000 /* 10 sec wait */

/******************* Global variables *****************************************/
extern phNxpUciHal_Control_t nxpucihal_ctrl;

extern uint32_t cleanup_timer;
extern bool uwb_debug_enabled;
extern uint32_t timeoutTimerId;
extern uint8_t channel_5_support;
extern uint8_t channel_9_support;
extern short conf_tx_power;
extern bool uwb_enable;
uint8_t *gtx_power = NULL, *gRMS_tx_power = NULL;
uint8_t gtx_power_length = 0;
phNxpUciHalProp_Control_t extNxpucihal_ctrl;
uint32_t hwResetTimer;

/************** HAL extension functions ***************************************/
static void hal_extns_write_rsp_timeout_cb(uint32_t TimerId, void *pContext);
static void phNxpUciHal_send_dev_status_ntf();
static bool phNxpUciHal_is_retry_required(uint8_t uci_octet0);
static void phNxpUciHal_clear_thermal_runaway_status();
static void phNxpUciHal_hw_reset_ntf_timeout_cb(uint32_t timerId,
                                                void *pContext);
tHAL_UWB_STATUS phNxpUciHal_handle_thermal_runaway_status();

/******************************************************************************
 * Function         phNxpUciHal_process_ext_cmd_rsp
 *
 * Description      This function process the extension command response. It
 *                  also checks the received response to expected response.
 *
 * Returns          returns UWBSTATUS_SUCCESS if response is as expected else
 *                  returns failure.
 *
 ******************************************************************************/
tHAL_UWB_STATUS phNxpUciHal_process_ext_cmd_rsp(uint16_t cmd_len,
                                                const uint8_t *p_cmd,
                                                uint16_t *data_written) {
  tHAL_UWB_STATUS status = UWBSTATUS_FAILED;
  uint8_t ext_cmd_retry_cnt = 0, invalid_len_retry_cnt = 0;
  bool exit_loop = 0, isRetryRequired = false;
  /* Create the local semaphore */
  if (phNxpUciHal_init_cb_data(&nxpucihal_ctrl.ext_cb_data, NULL) !=
      UWBSTATUS_SUCCESS) {
    NXPLOG_UCIHAL_D("Create ext_cb_data failed");
    return UWBSTATUS_FAILED;
  }

  isRetryRequired = phNxpUciHal_is_retry_required(p_cmd[0]);

  /* Send ext command */
  do {
    NXPLOG_UCIHAL_D("Entered do while loop");

    nxpucihal_ctrl.ext_cb_data.status = UWBSTATUS_SUCCESS;
    nxpucihal_ctrl.ext_cb_waiting = true;

    *data_written = phNxpUciHal_write_unlocked(cmd_len, p_cmd);

    if (*data_written != cmd_len) {
      NXPLOG_UCIHAL_D("phNxpUciHal_write failed for hal ext");
      goto clean_and_return;
    }
    if (nxpucihal_ctrl.hal_parse_enabled) {
      goto clean_and_return;
    }

    NXPLOG_UCIHAL_D("ext_cmd_retry_cnt is %d",ext_cmd_retry_cnt);

    if (isRetryRequired) {
      NXPLOG_UCIHAL_D("Received chained command or data command, no need to "
                      "wait for response");
      exit_loop = 1;
    } else {
      /* Start timer */
      status =
          phOsalUwb_Timer_Start(timeoutTimerId, HAL_EXTNS_WRITE_RSP_TIMEOUT,
                                &hal_extns_write_rsp_timeout_cb, NULL);
      if (UWBSTATUS_SUCCESS == status) {
        NXPLOG_UCIHAL_D("Response timer started");
      } else {
        NXPLOG_UCIHAL_E("Response timer not started!!!");
        status = UWBSTATUS_FAILED;
        goto clean_and_return;
      }

      /* Wait for rsp */
      NXPLOG_UCIHAL_D("Waiting after ext cmd sent");
      if (SEM_WAIT(nxpucihal_ctrl.ext_cb_data)) {
        NXPLOG_UCIHAL_E("p_hal_ext->ext_cb_data.sem semaphore error");
        goto clean_and_return;
      }
      nxpucihal_ctrl.ext_cb_waiting = false;

      switch (nxpucihal_ctrl.ext_cb_data.status) {
      case UWBSTATUS_RESPONSE_TIMEOUT:
      case UWBSTATUS_COMMAND_RETRANSMIT:
        ext_cmd_retry_cnt++;
        break;
      case UWBSTATUS_INVALID_COMMAND_LENGTH:
        // XXX: Why retrying here?
        invalid_len_retry_cnt++;
        break;
      default:
        exit_loop = 1;
        break;
      }
      if ((ext_cmd_retry_cnt >= MAX_COMMAND_RETRY_COUNT) ||
          (invalid_len_retry_cnt >= 0x03)) {
        exit_loop = 1;
        phNxpUciHal_send_dev_status_ntf();
      }
    }
  } while(exit_loop == 0);

  if (!isRetryRequired) {
    /* Stop Timer */
    status = phOsalUwb_Timer_Stop(timeoutTimerId);

    if (UWBSTATUS_SUCCESS == status) {
      NXPLOG_UCIHAL_D("Response timer stopped");
    } else {
      NXPLOG_UCIHAL_E("Response timer stop ERROR!!!");
      status = UWBSTATUS_FAILED;
      goto clean_and_return;
    }

    if (nxpucihal_ctrl.ext_cb_data.status != UWBSTATUS_SUCCESS) {
      NXPLOG_UCIHAL_E("Response Status = 0x%x",
                      nxpucihal_ctrl.ext_cb_data.status);
      status = UWBSTATUS_FAILED;
      goto clean_and_return;
    }
    NXPLOG_UCIHAL_D("Checking response");
    status = UWBSTATUS_SUCCESS;
  }
clean_and_return:
  phNxpUciHal_cleanup_cb_data(&nxpucihal_ctrl.ext_cb_data);

  return status;
}

/******************************************************************************
 * Function         phNxpUciHal_send_ext_cmd
 *
 * Description      This function send the extension command to UWBC. No
 *                  response is checked by this function but it waits for
 *                  the response to come.
 *
 * Returns          Returns UWBSTATUS_SUCCESS if sending cmd is successful and
 *                  response is received.
 *
 ******************************************************************************/
tHAL_UWB_STATUS phNxpUciHal_send_ext_cmd(uint16_t cmd_len, const uint8_t* p_cmd) {
  tHAL_UWB_STATUS status;

  if (cmd_len >= UCI_MAX_DATA_LEN) {
    status = UWBSTATUS_FAILED;
    return status;
  }
  uint16_t data_written = 0;
  HAL_ENABLE_EXT();
  nxpucihal_ctrl.cmd_len = cmd_len;
  memcpy(nxpucihal_ctrl.p_cmd_data, p_cmd, cmd_len);
  status = phNxpUciHal_process_ext_cmd_rsp(
      nxpucihal_ctrl.cmd_len, nxpucihal_ctrl.p_cmd_data, &data_written);
  HAL_DISABLE_EXT();

  return status;
}

/******************************************************************************
 * Function         hal_extns_write_rsp_timeout_cb
 *
 * Description      Timer call back function
 *
 * Returns          None
 *
 ******************************************************************************/
static void hal_extns_write_rsp_timeout_cb(uint32_t timerId, void* pContext) {
  UNUSED(timerId);
  UNUSED(pContext);
  NXPLOG_UCIHAL_E("hal_extns_write_rsp_timeout_cb - write timeout!!!");
  nxpucihal_ctrl.ext_cb_data.status = UWBSTATUS_RESPONSE_TIMEOUT;
  usleep(1);
  SEM_POST(&(nxpucihal_ctrl.ext_cb_data));

  return;
}

/******************************************************************************
 * Function         phNxpUciHal_set_board_config
 *
 * Description      This function is called to set the board varaint config
 * Returns          return 0 on success and -1 on fail, On success
 *                  update the acutual state of operation in arg pointer
 *
 ******************************************************************************/
tHAL_UWB_STATUS phNxpUciHal_set_board_config(){
  tHAL_UWB_STATUS status;
  uint8_t buffer[] = {0x2E,0x00,0x00,0x02,0x01,0x01};
  /* Set the board variant configurations */
  unsigned long num = 0;
  NXPLOG_UCIHAL_D("%s: enter; ", __func__);
  uint8_t boardConfig = 0, boardVersion = 0;

  if(NxpConfig_GetNum(NAME_UWB_BOARD_VARIANT_CONFIG, &num, sizeof(num))){
    boardConfig = (uint8_t)num;
    NXPLOG_UCIHAL_D("%s: NAME_UWB_BOARD_VARIANT_CONFIG: %x", __func__,boardConfig);
  } else {
    NXPLOG_UCIHAL_D("%s: NAME_UWB_BOARD_VARIANT_CONFIG: failed %x", __func__,boardConfig);
  }
  if(NxpConfig_GetNum(NAME_UWB_BOARD_VARIANT_VERSION, &num, sizeof(num))){
    boardVersion = (uint8_t)num;
    NXPLOG_UCIHAL_D("%s: NAME_UWB_BOARD_VARIANT_VERSION: %x", __func__,boardVersion);
  } else{
    NXPLOG_UCIHAL_D("%s: NAME_UWB_BOARD_VARIANT_VERSION: failed %lx", __func__,num);
  }
  buffer[4] = boardConfig;
  buffer[5] = boardVersion;

  status = phNxpUciHal_send_ext_cmd(sizeof(buffer), buffer);

  return status;
}

/*******************************************************************************
**
** Function         phNxpUciHal_process_ext_rsp
**
** Description      Process extension function response
**
** Returns          UWBSTATUS_SUCCESS if success
**
*******************************************************************************/
tHAL_UWB_STATUS phNxpUciHal_process_ext_rsp(uint16_t rsp_len, uint8_t* p_buff){
  tHAL_UWB_STATUS status;
  int NumOfTlv, index;
  uint8_t paramId, extParamId, IdStatus;
  index = UCI_NTF_PAYLOAD_OFFSET; // index for payload start
  status = p_buff[index++];
  if(status  == UCI_STATUS_OK){
    NXPLOG_UCIHAL_D("%s: status success %d", __func__, status);
    return UWBSTATUS_SUCCESS;
  }
  NumOfTlv = p_buff[index++];
  while (index < rsp_len) {
      paramId = p_buff[index++];
      if(paramId == EXTENDED_DEVICE_CONFIG_ID) {
        extParamId = p_buff[index++];
        IdStatus = p_buff[index++];

        switch(extParamId) {
          case UCI_EXT_PARAM_DELAY_CALIBRATION_VALUE:
          case UCI_EXT_PARAM_AOA_CALIBRATION_CTRL:
          case UCI_EXT_PARAM_DPD_WAKEUP_SRC:
          case UCI_EXT_PARAM_WTX_COUNT_CONFIG:
          case UCI_EXT_PARAM_DPD_ENTRY_TIMEOUT:
          case UCI_EXT_PARAM_WIFI_COEX_FEATURE:
          case UCI_EXT_PARAM_TX_BASE_BAND_CONFIG:
          case UCI_EXT_PARAM_DDFS_TONE_CONFIG:
          case UCI_EXT_PARAM_TX_PULSE_SHAPE_CONFIG:
          case UCI_EXT_PARAM_CLK_CONFIG_CTRL:
            if(IdStatus == UCI_STATUS_FEATURE_NOT_SUPPORTED){
              NXPLOG_UCIHAL_E("%s: Vendor config param: %x %x is Not Supported", __func__, paramId, extParamId);
              status = UWBSTATUS_SUCCESS;
            } else {
              status = UWBSTATUS_FAILED;
              return status;
            }
            break;
          default:
            NXPLOG_UCIHAL_D("%s: Vendor param ID: %x", __func__, extParamId);
            break;
        }
      } else {
        IdStatus = p_buff[index++];
        switch(paramId) {
          case UCI_PARAM_ID_LOW_POWER_MODE:
            if(IdStatus == UCI_STATUS_FEATURE_NOT_SUPPORTED){
              NXPLOG_UCIHAL_E("%s: Generic config param: %x is Not Supported", __func__, paramId);
              status = UWBSTATUS_SUCCESS;
            } else {
              status = UWBSTATUS_FAILED;
              return status;
            }
            break;
          default:
            NXPLOG_UCIHAL_D("%s: Generic param ID: %x", __func__, paramId);
            break;
        }
      }
    }
 NXPLOG_UCIHAL_D("%s: exit %d", __func__, status);
 return status;
}

/*******************************************************************************
 * Function      phNxpUciHal_reset_country_code_config
 *
 * Description   Reset the country code config
 *
 * Returns       void
 *
 *******************************************************************************/
void phNxpUciHal_reset_country_code_config() {
 uwb_enable = true;
 channel_5_support = 1;
 channel_9_support = 1;
 conf_tx_power = 0;
}

/*******************************************************************************
 * Function      phNxpUciHal_getCountryCaps
 *
 * Description   Creates supported channel's and Tx power TLV format for
 *specific country code  and updates map.
 *
 * Returns       void
 *
 *******************************************************************************/
void phNxpUciHal_getCountryCaps(const uint8_t *cc_resp, const char country_code[2],
                                uint8_t *cc_data, uint32_t *retlen) {
 uint16_t idx = 0;
 uint16_t index = 0;
 bool country_code_found = false;
 uint8_t tx_power_len = 0;
 uint32_t cc_resp_len = *retlen;
 phNxpUciHal_reset_country_code_config();
 while (idx < cc_resp_len) {
      if (cc_resp[idx++] == COUNTRY_CODE_TAG) { // ISO country code TAG
        uint16_t len_idx = idx++;
        uint8_t len = cc_resp[len_idx];
        for (uint8_t index = 0; index < len; index++) {
            if (cc_resp[idx] != country_code[index]) {
              idx = len_idx + len + 1;
              country_code_found = false;
              break;
            } else {
              idx++;
              country_code_found = true;
            }
        }
        while (cc_resp[idx] != COUNTRY_CODE_TAG) {
            uint8_t cc_tag = cc_resp[idx++];
            len = cc_resp[idx++];
            if (country_code_found) {
              switch (cc_tag) {
              case UWB_ENABLE_TAG:
                uwb_enable = cc_resp[idx++];
                break;
              case CHANNEL_5_TAG:
                channel_5_support = cc_resp[idx++];
                break;
              case CHANNEL_9_TAG:
                channel_9_support = cc_resp[idx++];
                break;
              case TX_POWER_TAG:
                conf_tx_power = (cc_resp[idx++] << RMS_TX_POWER_SHIFT);
                conf_tx_power |= (cc_resp[idx++]);
                phNxpUciHal_setCalibParamTxPower(conf_tx_power);
                tx_power_len = len;
                break;
              }
            } else {
              idx += len;
            }
        }
      }
 }
 NXPLOG_UCIHAL_D("channel_5_support = %d", channel_5_support);
 NXPLOG_UCIHAL_D("channel_9_support = %d", channel_9_support);

 uint8_t channel_info = (channel_5_support | CHANNEL_5_MASK) & 0xFF &
                        ((channel_9_support << 3) | CHANNEL_9_MASK);
 uint8_t ccc_channel_info =
     (channel_5_support | (channel_9_support << 1)) & CCC_CHANNEL_INFO_BIT_MASK;
 NXPLOG_UCIHAL_D("channel_info = %d", channel_info);
 cc_data[index++] = UWB_CHANNELS;
 cc_data[index++] = 0x01;
 cc_data[index++] = channel_info;
 cc_data[index++] = CCC_UWB_CHANNELS;
 cc_data[index++] = 0x01;
 cc_data[index++] = ccc_channel_info;
 *retlen = index;
}

/*******************************************************************************
 * Function      phNxpUciHal_send_dev_status_ntf
 *
 * Description   send device status notification
 *
 * Returns       void
 *
 *******************************************************************************/
static void phNxpUciHal_send_dev_status_ntf() {
 NXPLOG_UCIHAL_D("phNxpUciHal_send_dev_status_ntf ");
 nxpucihal_ctrl.rx_data_len = 5;
 static uint8_t rsp_data[5] = {0x60, 0x01, 0x00, 0x01, 0xFF};
 (*nxpucihal_ctrl.p_uwb_stack_data_cback)(nxpucihal_ctrl.rx_data_len, rsp_data);
}

/*******************************************************************************
 * Function      phNxpUciHal_is_retry_required
 *
 * Description   UCI command retry check
 *
 * Returns       true/false
 *
 *******************************************************************************/
static bool phNxpUciHal_is_retry_required(uint8_t uci_octet0) {
 bool isRetryRequired = false, isChained_cmd = false, isData_Msg = false;
 isChained_cmd = (bool)((uci_octet0 & UCI_PBF_ST_CONT) >> UCI_PBF_SHIFT);
 isData_Msg = ((uci_octet0 & UCI_MT_MASK) >> UCI_MT_SHIFT) == UCI_MT_DATA;
 isRetryRequired = isChained_cmd | isData_Msg;
 return isRetryRequired;
}

/******************************************************************************
 * Function         phNxpUciHal_sendSetCalibration
 *
 * Description      This function send set calibration command
 *
 * Returns          void
 *
 ******************************************************************************/
static void phNxpUciHal_sendSetCalibration(const uint8_t *setCalibData,
                                           uint8_t length) {
 // GID : 0xF / OID : 0x21
 const uint8_t setCalibHeader[] = {0x2F, 0x21, 0x00};
 uint8_t *setCalibCmd = NULL;
 setCalibCmd = (uint8_t *)malloc(sizeof(uint8_t) * (length + UCI_MSG_HDR_SIZE));
 memcpy(&setCalibCmd[0], &setCalibHeader[0], 3);
 setCalibCmd[UCI_MSG_HDR_SIZE - 1] = length;
 memcpy(&setCalibCmd[UCI_MSG_HDR_SIZE], &setCalibData[0], length);
 length += UCI_MSG_HDR_SIZE;
 tHAL_UWB_STATUS status = phNxpUciHal_send_ext_cmd(length, setCalibCmd);
 if (status != UWBSTATUS_SUCCESS) {
      NXPLOG_UCIHAL_D("%s: send failed", __func__);
 }
 if (setCalibCmd != NULL) {
      free(setCalibCmd);
 }
 if (gtx_power != NULL) {
      free(gtx_power);
      gtx_power = NULL;
 }
 if (gRMS_tx_power != NULL) {
      free(gRMS_tx_power);
      gRMS_tx_power = NULL;
 }
}

/*******************************************************************************
 * Function      phNxpUciHal_processCalibParamTxPowerPerAntenna
 *
 * Description  Stores Tx power set during calibration
 *
 * Returns      void
 *
 *******************************************************************************/
void phNxpUciHal_processCalibParamTxPowerPerAntenna(const short conf_tx_power,
                                                    const uint8_t *p_data,
                                                    uint16_t data_len) {
 // RMS Tx power -> Octet [4, 5] in calib data
 NXPLOG_UCIHAL_D("phNxpUciHal_processCalibParamTxPowerPerAntenna %d",
                 conf_tx_power);

 if (gtx_power != NULL) {
      free(gtx_power);
      gtx_power = NULL;
 }
 gtx_power = (uint8_t *)malloc(sizeof(uint8_t) * data_len);

 if (gtx_power != NULL) {
      gtx_power_length = p_data[UCI_MSG_HDR_SIZE - 1];
      memcpy(&gtx_power[0], &p_data[UCI_MSG_HDR_SIZE],
             data_len - UCI_MSG_HDR_SIZE);
 }

 if (conf_tx_power != 0) {
      phNxpUciHal_updateTxPower(conf_tx_power);
 }

 if (gtx_power != NULL) {
      memcpy(&nxpucihal_ctrl.p_cmd_data[UCI_MSG_HDR_SIZE], &gtx_power[0],
             data_len - UCI_MSG_HDR_SIZE);
 }
}

/******************************************************************************
 * Function         phNxpUciHal_updateTxPower
 *
 * Description      This function updates the tx antenna power
 *
 * Returns          true/false
 *
 ******************************************************************************/
bool phNxpUciHal_updateTxPower(short conf_tx_power) {
 if (gtx_power != NULL) {
      uint8_t index = 0;
      index++; // channel num
      index++; // param ID
      if (gtx_power[index++]) {
        uint8_t num_of_antennas = gtx_power[index++];
        while (num_of_antennas--) {
            index++;    // antenna Id
            index += 2; // Peak Tx
            short calib_tx_pow =
                gtx_power[index] << RMS_TX_POWER_SHIFT | gtx_power[index + 1];
            gtx_power[index++] =
                (conf_tx_power + calib_tx_pow) >> RMS_TX_POWER_SHIFT;
            gtx_power[index++] = (conf_tx_power + calib_tx_pow);
        }
        return true;
      }
 }
 return false;
}

/******************************************************************************
 * Function         phNxpUciHal_setCalibParamTxPower
 *
 * Description      This function sets the TX power
 *
 * Returns          true/false
 *
 ******************************************************************************/
bool phNxpUciHal_setCalibParamTxPower(short conf_tx_power) {

 phNxpUciHal_updateTxPower(conf_tx_power);
 if (gtx_power != NULL) {
      phNxpUciHal_sendSetCalibration(gtx_power, gtx_power_length);
 }
 return true;
}

/******************************************************************************
 * Function         phNxpUciHal_hw_reset_ntf_timeout_cb
 *
 * Description      Timer call back function
 *
 * Returns          None
 *
 ******************************************************************************/
static void phNxpUciHal_hw_reset_ntf_timeout_cb(uint32_t timerId,
                                                void *pContext) {
 UNUSED(timerId);
 UNUSED(pContext);
 NXPLOG_UCIHAL_E("phNxpUciHal_hw_reset_ntf_timeout_cb!!!");
 tHAL_UWB_STATUS status;

 status = phOsalUwb_Timer_Stop(hwResetTimer);
 if (UWBSTATUS_SUCCESS == status) {
      NXPLOG_UCIHAL_D("Response timer stopped");
 } else {
      NXPLOG_UCIHAL_E("Response timer stop ERROR!!!");
 }
 pthread_cond_signal(&extNxpucihal_ctrl.mCondVar);
 extNxpucihal_ctrl.isThermalRecoveryOngoing = false;

 return;
}

/******************************************************************************
 * Function         phNxpUciHal_handle_thermal_runaway_status
 *
 * Description     This function handles the core generic error ntf with status
 *                 temperature exceeded(0x54)
 *
 * Returns          return uwb status, On success
 *                  update the acutual state of operation in arg pointer
 *
 ******************************************************************************/
tHAL_UWB_STATUS phNxpUciHal_handle_thermal_runaway_status() {

 tHAL_UWB_STATUS status = UWBSTATUS_FAILED;
 extNxpucihal_ctrl.isThermalRecoveryOngoing = true;

 /* Send FW crash NTF to upper layer for triggering MW recovery */
 nxpucihal_ctrl.rx_data_len = 5;
 nxpucihal_ctrl.p_rx_data[0] = 0x60;
 nxpucihal_ctrl.p_rx_data[1] = 0x01;
 nxpucihal_ctrl.p_rx_data[2] = 0x00;
 nxpucihal_ctrl.p_rx_data[3] = 0x01;
 nxpucihal_ctrl.p_rx_data[4] = 0xFF;
 (*nxpucihal_ctrl.p_uwb_stack_data_cback)(nxpucihal_ctrl.rx_data_len,
                                          nxpucihal_ctrl.p_rx_data);

 hwResetTimer = phOsalUwb_Timer_Create();

 status = phOsalUwb_Timer_Start(hwResetTimer, HAL_HW_RESET_NTF_TIMEOUT,
                                &phNxpUciHal_hw_reset_ntf_timeout_cb, NULL);

 if (UWBSTATUS_SUCCESS == status) {
      nxpucihal_ctrl.isRecoveryTimerStarted = true;
      NXPLOG_UCIHAL_E("HW Reset Ntf timer started");
 } else {
      NXPLOG_UCIHAL_E("HW Reset Ntf timer not started!!!");
      pthread_cond_signal(&extNxpucihal_ctrl.mCondVar);
      return UWBSTATUS_FAILED;
 }
 return UWBSTATUS_SUCCESS;
}

/******************************************************************************
 * Function         phNxpUciHal_clear_thermal_runaway_status
 *
 * Description     This function is used to clear thermal runaway context.
 *
 * Returns          void
 *
 ******************************************************************************/
static void phNxpUciHal_clear_thermal_runaway_status() {
 tHAL_UWB_STATUS status = UWBSTATUS_FAILED;
 nxpucihal_ctrl.isSkipPacket = 1;
 NXPLOG_UCIHAL_D("received hw reset ntf");
 pthread_cond_signal(&extNxpucihal_ctrl.mCondVar);
 extNxpucihal_ctrl.isThermalRecoveryOngoing = false;
 if (nxpucihal_ctrl.isRecoveryTimerStarted == true) {
      status = phOsalUwb_Timer_Stop(hwResetTimer);
      if (UWBSTATUS_SUCCESS == status) {
        NXPLOG_UCIHAL_D("Response timer stopped");
      } else {
        NXPLOG_UCIHAL_E("Response timer stop ERROR!!!");
      }
 }
}

struct ReadOtpCookie {
  ReadOtpCookie(uint8_t param_id, uint8_t *buffer, size_t len) :
    m_valid(false), m_id(param_id), m_buffer(buffer), m_len(len) {
  }
  std::atomic_bool  m_valid;
  uint8_t m_id;
  uint8_t *m_buffer;
  size_t  m_len;
};

/******************************************************************************
 * Function         otp_read_data
 *
 * Description      Read OTP calibration data
 *
 * Returns          true on success
 *
 ******************************************************************************/
static bool otp_read_data(const uint8_t channel, const uint8_t param_id, uint8_t *buffer, size_t len)
{
  ReadOtpCookie cookie(param_id, buffer, len);

  if (phNxpUciHal_init_cb_data(&nxpucihal_ctrl.calib_data_ntf_wait, &cookie) != UWBSTATUS_SUCCESS) {
    NXPLOG_UCIHAL_E("Failed to create call back data for reading otp");
    return false;
  }

  // READ_CALIB_DATA_CMD
  std::vector<uint8_t> packet{(UCI_MT_CMD << UCI_MT_SHIFT) | UCI_GID_PROPRIETARY_0X0A, UCI_MSG_READ_CALIB_DATA, 0x00, 0x03};
  packet.push_back(channel);
  packet.push_back(0x01);      // OTP read option
  packet.push_back(param_id);

  tHAL_UWB_STATUS status = phNxpUciHal_send_ext_cmd(packet.size(), packet.data());
  if (status != UWBSTATUS_SUCCESS) {
    goto fail_otp_read_data;
  }

  phNxpUciHal_sem_timed_wait_sec(&nxpucihal_ctrl.calib_data_ntf_wait, 3);
  if (!cookie.m_valid) {
    goto fail_otp_read_data;
  }

  phNxpUciHal_cleanup_cb_data(&nxpucihal_ctrl.calib_data_ntf_wait);
  return true;

fail_otp_read_data:
  phNxpUciHal_cleanup_cb_data(&nxpucihal_ctrl.calib_data_ntf_wait);
  NXPLOG_UCIHAL_E("Failed to read OTP data id=%u", param_id);
  return false;
}

/******************************************************************************
 * Function         phNxpUciHal_parseCoreDeviceInfoRsp
 *
 * Description      This function parse Core device Info response.
 *
 * Returns          void.
 *
 ******************************************************************************/
static void phNxpUciHal_parseCoreDeviceInfoRsp(const uint8_t *p_rx_data, size_t rx_data_len)
{
  uint8_t index = 13; // Excluding the header and Versions
  uint8_t paramId = 0;
  uint8_t length = 0;

  if (nxpucihal_ctrl.isDevInfoCached) {
    return;
  }

  NXPLOG_UCIHAL_D("phNxpUciHal_parseCoreDeviceInfoRsp Enter..");

  if (rx_data_len > sizeof(nxpucihal_ctrl.dev_info_resp)) {
      NXPLOG_UCIHAL_E("FIXME: CORE_DEVICE_INFO_RSP buffer overflow!");
      return;
  }

  memcpy(nxpucihal_ctrl.dev_info_resp, nxpucihal_ctrl.p_rx_data, nxpucihal_ctrl.rx_data_len);

  uint8_t len = p_rx_data[index++];
  len = len + index;
  while (index < len) {
    paramId = p_rx_data[index++];
    length = p_rx_data[index++];
    if (paramId == DEVICE_NAME_PARAM_ID && length >= 5) {
      /* SR100T --> T */
      switch(p_rx_data[index + 5]) {
      case DEVICE_TYPE_SR1xxS:
        nxpucihal_ctrl.device_type = DEVICE_TYPE_SR1xxS;
        break;
      case DEVICE_TYPE_SR1xxT:
        nxpucihal_ctrl.device_type = DEVICE_TYPE_SR1xxT;
        break;
      default:
        nxpucihal_ctrl.device_type = DEVICE_TYPE_UNKNOWN;
        break;
      }
    } else if (paramId == FW_VERSION_PARAM_ID && length >= 3) {
      nxpucihal_ctrl.fw_version.major_version = p_rx_data[index];
      nxpucihal_ctrl.fw_version.minor_version = p_rx_data[index + 1];
      nxpucihal_ctrl.fw_version.rc_version = p_rx_data[index + 2];
    } else if (paramId == FW_BOOT_MODE_PARAM_ID && length >= 1) {
      nxpucihal_ctrl.fw_boot_mode = p_rx_data[index];
      break;
    }
    index = index + length;
  }
  NXPLOG_UCIHAL_D("phNxpUciHal_parseCoreDeviceInfoRsp: Device Info cached.");
  nxpucihal_ctrl.isDevInfoCached = true;
  return;
}


/******************************************************************************
 * Function         phNxpUciHal_process_response
 *
 * Description      This function handles all the propriotory hal
 *functionalities.
 *
 * Returns          void.
 *
 ******************************************************************************/
void phNxpUciHal_process_response() {
 tHAL_UWB_STATUS status;

 uint8_t mt, gid, oid, pbf;

 mt = (nxpucihal_ctrl.p_rx_data[0]  & UCI_MT_MASK) >> UCI_MT_SHIFT;
 gid = nxpucihal_ctrl.p_rx_data[0] & UCI_GID_MASK;
 oid = nxpucihal_ctrl.p_rx_data[1] & UCI_OID_MASK;
 pbf = (nxpucihal_ctrl.p_rx_data[0] & UCI_PBF_MASK) >> UCI_PBF_SHIFT;

 if ((gid == UCI_GID_CORE) && (oid == UCI_MSG_CORE_GENERIC_ERROR_NTF) &&
     (nxpucihal_ctrl.p_rx_data[UCI_RESPONSE_STATUS_OFFSET] ==
      UCI_STATUS_THERMAL_RUNAWAY)) {
      nxpucihal_ctrl.isSkipPacket = 1;
      status = phNxpUciHal_handle_thermal_runaway_status();
      if (status != UCI_STATUS_OK) {
        NXPLOG_UCIHAL_E("phNxpUciHal_handle_thermal_runaway_status failed");
      }
 }

 if ((gid == UCI_GID_CORE) && (oid == UCI_MSG_CORE_DEVICE_STATUS_NTF) &&
     (nxpucihal_ctrl.p_rx_data[UCI_RESPONSE_STATUS_OFFSET] ==
      UCI_STATUS_HW_RESET)) {
      phNxpUciHal_clear_thermal_runaway_status();
 }

  // Remember CORE_DEVICE_INFO_RSP
  if (mt == UCI_MT_RSP && (gid == UCI_GID_CORE) && (oid == UCI_MSG_CORE_DEVICE_INFO)) {
    if (pbf) {
      // FIXME: Fix the whole logic if this really happens
      NXPLOG_UCIHAL_E("FIXME: CORE_DEVICE_INFO_RSP is fragmented!");
    } else {
      phNxpUciHal_parseCoreDeviceInfoRsp(nxpucihal_ctrl.p_rx_data, nxpucihal_ctrl.rx_data_len);
    }
  }

  //
  // Handle NXP_READ_CALIB_DATA_NTF
  //
  if ((mt == UCI_MT_NTF) && (gid == UCI_GID_PROPRIETARY_0X0A) && (oid == UCI_MSG_READ_CALIB_DATA)) {
    // READ_CALIB_DATA_NTF: status(1), length-of-payload(1), payload(N)
    const uint8_t plen = nxpucihal_ctrl.p_rx_data[3]; // payload-length
    const uint8_t *p = &nxpucihal_ctrl.p_rx_data[4];  // payload
    ReadOtpCookie *cookie = (ReadOtpCookie*)nxpucihal_ctrl.calib_data_ntf_wait.pContext;

    if (!cookie) {
      NXPLOG_UCIHAL_E("Otp read: unexpected OTP read.");
    } else if (cookie->m_valid) {
      // cookie is already valid
      NXPLOG_UCIHAL_E("Otp read: unexpected OTP read, param-id=0x%x", cookie->m_id);
    } else if (plen < 2) {
      NXPLOG_UCIHAL_E("Otp read: bad payload length %u", plen);
    } else if (p[0] != UCI_STATUS_OK) {
      NXPLOG_UCIHAL_E("Otp read: bad status=0x%x", nxpucihal_ctrl.p_rx_data[4]);
    } else if (p[1] != cookie->m_len) {
      NXPLOG_UCIHAL_E("Otp read: size mismatch %u (expected %zu for param 0x%x)",
        p[1], cookie->m_len, cookie->m_id);
    } else {
      memcpy(cookie->m_buffer, &p[2], cookie->m_len);
      cookie->m_valid = true;
      SEM_POST(&nxpucihal_ctrl.calib_data_ntf_wait);
    }
  }
}

/******************************************************************************
 * Function         phNxpUciHal_extcal_handle_coreinit
 *
 * Description      Apply additional core device settings
 *
 * Returns          void.
 *
 ******************************************************************************/
void phNxpUciHal_extcal_handle_coreinit(void)
{
  long retlen = 0;

  // Channels
  const uint8_t cal_channels[] = {5, 6, 8, 9};

  // Antenna Definitions: rx_antenna_mask(1), tx_antenna_mask(1)
  uint8_t rx_antenna_mask_n = 0xff;
  NxpConfig_GetNum("cal.rx_antenna_mask", &rx_antenna_mask_n, 1);
  std::bitset<8> rx_antenna_mask(rx_antenna_mask_n);
  const uint8_t n_rx_antennas = rx_antenna_mask.size();

  // SET_CALIBRATION_CMD header: GID=0xF OID=0x21
  const std::vector<uint8_t> packet_header({ (0x20 | UCI_GID_PROPRIETARY_0X0F), UCI_MSG_SET_DEVICE_CALIBRATION, 0x00, 0x00});

  // Supported calibrations,
  //  current HAL impl only supports "xtal" read from otp
  //  others should be existed in .conf files
  //
  // | name          |otp-id|cal-id| size |per-   |per-   | otp |
  // |               |      |      |      |channel|antenna|     |
  // |---------------|------|------|------|-------|-------|-----|
  // | xtal          | 0x02 | 0x02 | 3    | n     | n     | y   |
  // | tx_power      | 0x04 | 0x17 | 2    | y     | y     |     |
  // | ant_delay     | 0x0b | 0x02 | 2    | y     | y     |     |

  // XTAL_CAP_GM_CTRL
  // Check otp.xtal
  uint8_t otp_xtal_flag = 0;
  uint8_t otp_xtal_data[3];
  uint8_t xtal_data[6];
  bool need_xtal_calibration = false;

  if (NxpConfig_GetNum("cal.otp.xtal", &otp_xtal_flag, 1) && otp_xtal_flag) {
    if (otp_read_data(0x09, OTP_ID_XTAL_CAP_GM_CTRL, otp_xtal_data, sizeof(otp_xtal_data))) {
      // convert OTP_ID_XTAL_CAP_GM_CTRL to RF_CLK_ACCURACY_CALIB
      memset(xtal_data, 0, sizeof(xtal_data));
      xtal_data[0] = otp_xtal_data[0];
      xtal_data[2] = otp_xtal_data[1];
      xtal_data[4] = otp_xtal_data[2];
      need_xtal_calibration = true;
    } else {
      NXPLOG_UCIHAL_E("Failed to read OTP XTAL_CAP_GM_CTRL");
    }
  } else if (NxpConfig_GetByteArray("cal.xtal", otp_xtal_data, sizeof(otp_xtal_data), &retlen) &&
             retlen == sizeof(otp_xtal_data)) {
    need_xtal_calibration = true;
  }

  if (need_xtal_calibration) {
    std::vector<uint8_t> payload;

    // Channel, 9, don't care for RF_CLK_ACCURACY_CALIB
    payload.push_back(9);
    // Tag
    payload.push_back(UCI_PARAM_ID_RF_CLK_ACCURACY_CALIB);
    // Length = 7
    payload.push_back(1 + sizeof(xtal_data));
    // octet[0] = 3
    payload.push_back(3);
    // octet[6:1] = cap1(2), cap2(2), gm_current_control(2)
    payload.insert(payload.end(), &xtal_data[0], &xtal_data[6]);

    std::vector<uint8_t> packet(packet_header);
    packet[3] = payload.size();
    packet.insert(packet.end(), payload.begin(), payload.end());

    tHAL_UWB_STATUS status = phNxpUciHal_send_ext_cmd(packet.size(), packet.data());
    if (status != UWBSTATUS_SUCCESS) {
      NXPLOG_UCIHAL_E("Failed to apply XTAL_CAP_GM_CTRL={%02x,%02x,%02x,%02x,%02x,%02x}",
      xtal_data[0], xtal_data[1], xtal_data[2], xtal_data[3], xtal_data[4], xtal_data[5]);
    }
  }

  // RX_ANT_DELAY_CALIB(0x0F)
  // Read configuration file ant1.ch5.ant_delay
  // N(1) + N * {AntennaID(1), Rxdelay(Q14.2)}
  if (n_rx_antennas) {
    for (auto ch : cal_channels) {
      std::vector<uint8_t> entries;
      uint8_t n_entries = 0;

      for (auto i = 0; i < n_rx_antennas; i++) {
        if (!rx_antenna_mask[i])
          continue;

        const uint8_t ant_id = i + 1;
        uint16_t delay_value;
        char key[32];
        std::snprintf(key, sizeof(key), "cal.ant%u.ch%u.ant_delay", ant_id, ch);

        if (!NxpConfig_GetNum(key, &delay_value, 2))
          continue;

        NXPLOG_UCIHAL_D("RX_ANT_DELAY_CALIB: found %s = %u", key, delay_value);
        entries.push_back(ant_id);
        // Little Endian
        entries.push_back(delay_value & 0xff);
        entries.push_back(delay_value >> 8);
        n_entries++;
      }

      if (!n_entries)
        continue;

      entries.insert(entries.begin(), n_entries);

      std::vector<uint8_t> payload;
      payload.push_back(ch);
      payload.push_back(UCI_PARAM_ID_RX_ANT_DELAY_CALIB);
      payload.push_back(entries.size());
      payload.insert(payload.end(), entries.begin(), entries.end());

      std::vector<uint8_t> packet(packet_header);
      packet[3] = payload.size();
      packet.insert(packet.end(), payload.begin(), payload.end());

      tHAL_UWB_STATUS status = phNxpUciHal_send_ext_cmd(packet.size(), packet.data());
      if (status != UWBSTATUS_SUCCESS) {
        NXPLOG_UCIHAL_E("Failed to apply RX_ANT_DELAY_CALIB for channel %u", ch);
      } else {
        NXPLOG_UCIHAL_E("RX_ANT_DELAY_CALIB: applied for channel %u", ch);
      }
    }
  }
}
