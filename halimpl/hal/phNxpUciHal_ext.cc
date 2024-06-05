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
#include <map>
#include <vector>

#include <cutils/properties.h>

#include "phNxpConfig.h"
#include "phNxpLog.h"
#include "phNxpUciHal.h"
#include "phNxpUciHal_ext.h"
#include "phNxpUciHal_utils.h"
#include "phTmlUwb.h"
#include "phUwbCommon.h"
#include "sessionTrack.h"

/* Timeout value to wait for response from DEVICE_TYPE_SR1xx */
#define MAX_COMMAND_RETRY_COUNT           5
#define HAL_EXTNS_WRITE_RSP_TIMEOUT_MS    100
#define HAL_HW_RESET_NTF_TIMEOUT          10000 /* 10 sec wait */

/******************* Global variables *****************************************/
extern phNxpUciHal_Control_t nxpucihal_ctrl;

static std::vector<uint8_t> gtx_power;

/************** HAL extension functions ***************************************/
static bool phNxpUciHal_is_retry_not_required(uint8_t uci_octet0);
static void phNxpUciHal_clear_thermal_error_status();
static void phNxpUciHal_hw_reset_ntf_timeout_cb(uint32_t timerId,
                                                void *pContext);

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
  // PBF=1 or DATA packet: don't check RSP
  bool isRetryNotRequired = phNxpUciHal_is_retry_not_required(p_cmd[0]) || (cmd_len < 4);

  const uint8_t mt = (p_cmd[0] & UCI_MT_MASK) >> UCI_MT_SHIFT;
  const uint8_t gid = p_cmd[0] & UCI_GID_MASK;
  const uint8_t oid = p_cmd[1] & UCI_OID_MASK;

  if (mt == UCI_MT_CMD && gid == UCI_GID_SESSION_CONTROL && oid == UCI_MSG_SESSION_START) {
    SessionTrack_onSessionStart(cmd_len, p_cmd);
  }

  // upper-layer should handle the case of UWBSTATUS_COMMAND_RETRANSMIT && isRetryNotRequired
  if (isRetryNotRequired) {
    *data_written = phNxpUciHal_write_unlocked(cmd_len, p_cmd);

    if (*data_written != cmd_len) {
      NXPLOG_UCIHAL_D("phNxpUciHal_write failed for hal ext");
      return UWBSTATUS_FAILED;
    } else {
      return UWBSTATUS_SUCCESS;
    }
  }

  /* Create the local semaphore */
  if (phNxpUciHal_init_cb_data(&nxpucihal_ctrl.ext_cb_data, NULL) != UWBSTATUS_SUCCESS) {
    NXPLOG_UCIHAL_D("Create ext_cb_data failed");
    return UWBSTATUS_FAILED;
  }

  tHAL_UWB_STATUS status = UWBSTATUS_FAILED;
  int nr_retries = 0;
  int nr_timedout = 0;
  bool exit_loop = false;

  while(!exit_loop) {
    nxpucihal_ctrl.ext_cb_data.status = UWBSTATUS_FAILED;
    nxpucihal_ctrl.ext_cb_waiting = true;

    *data_written = phNxpUciHal_write_unlocked(cmd_len, p_cmd);

    if (*data_written != cmd_len) {
      status = UWBSTATUS_FAILED;
      NXPLOG_UCIHAL_D("phNxpUciHal_write failed for hal ext");
      goto clean_and_return;
    }

    if (nxpucihal_ctrl.hal_parse_enabled) {
      status = UWBSTATUS_SUCCESS;
      goto clean_and_return;
    }

    // Wait for rsp
    phNxpUciHal_sem_timed_wait_msec(&nxpucihal_ctrl.ext_cb_data, HAL_EXTNS_WRITE_RSP_TIMEOUT_MS);

    nxpucihal_ctrl.ext_cb_waiting = false;

    switch (nxpucihal_ctrl.ext_cb_data.status) {
    case UWBSTATUS_RESPONSE_TIMEOUT:
      nr_timedout++;
      [[fallthrough]];
    case UWBSTATUS_COMMAND_RETRANSMIT:
      // TODO: Do not retransmit CMD by here when !nxpucihal_ctrl.hal_ext_enabled,
      // Upper layer should take care of it.
      nr_retries++;
      break;
    default:
      status = nxpucihal_ctrl.ext_cb_data.status;
      exit_loop = true;
      break;
    }

    if (nr_retries >= MAX_COMMAND_RETRY_COUNT) {
      NXPLOG_UCIHAL_E("Failed to process cmd/rsp 0x%x", nxpucihal_ctrl.ext_cb_data.status);
      status = UWBSTATUS_FAILED;
      exit_loop = true;
      phNxpUciHal_send_dev_error_status_ntf();
    }
  }

  if (nr_timedout > 0) {
    NXPLOG_UCIHAL_E("Warning: CMD/RSP retried %d times (timeout:%d)\n",
                    nr_retries, nr_timedout);
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
 * Function      phNxpUciHal_resetRuntimeSettings
 *
 * Description   reset per-country code settigs to default
 *
 *******************************************************************************/
static void phNxpUciHal_resetRuntimeSettings(void)
{
  phNxpUciHal_Runtime_Settings_t *rt_set = &nxpucihal_ctrl.rt_settings;
  rt_set->uwb_enable = true;
  rt_set->restricted_channel_mask = 0;
  rt_set->tx_power_offset = 0;
}

/*******************************************************************************
 * Function      phNxpUciHal_applyCountryCaps
 *
 * Description   Update runtime settings with given COUNTRY_CODE_CAPS byte array
 *
 * Returns       void
 *
 *******************************************************************************/
static void phNxpUciHal_applyCountryCaps(const char country_code[2],
    const uint8_t *cc_resp, uint32_t cc_resp_len)
{
  phNxpUciHal_Runtime_Settings_t *rt_set = &nxpucihal_ctrl.rt_settings;

  uint16_t idx = 1; // first byte = number countries
  bool country_code_found = false;

  while (idx < cc_resp_len) {
    uint8_t tag = cc_resp[idx++];
    uint8_t len = cc_resp[idx++];

    if (country_code_found) {
      switch (tag) {
      case UWB_ENABLE_TAG:
        if (len == 1) {
          rt_set->uwb_enable = cc_resp[idx];
          NXPLOG_UCIHAL_D("CountryCaps uwb_enable = %u", cc_resp[idx]);
        }
        break;
      case CHANNEL_5_TAG:
        if (len == 1 && !cc_resp[idx]) {
          rt_set->restricted_channel_mask |= 1<< 5;
          NXPLOG_UCIHAL_D("CountryCaps channel 5 support = %u", cc_resp[idx]);
        }
        break;
      case CHANNEL_9_TAG:
        if (len == 1 && !cc_resp[idx]) {
          rt_set->restricted_channel_mask |= 1<< 9;
          NXPLOG_UCIHAL_D("CountryCaps channel 9 support = %u", cc_resp[idx]);
        }
        break;
      case TX_POWER_TAG:
        if (len == 2) {
          rt_set->tx_power_offset = (short)((cc_resp[idx + 0]) | (((cc_resp[idx + 1]) << RMS_TX_POWER_SHIFT) & 0xFF00));
          NXPLOG_UCIHAL_D("CountryCaps tx_power_offset = %d", rt_set->tx_power_offset);
        }
        break;
      default:
        break;
      }
    }
    if (tag == COUNTRY_CODE_TAG) {
      country_code_found = (cc_resp[idx + 0] == country_code[0]) && (cc_resp[idx + 1] == country_code[1]);
    }
    idx += len;
  }
}

/*******************************************************************************
 * Function      phNxpUciHal_is_retry_not_required
 *
 * Description   UCI command retry check
 *
 * Returns       true/false
 *
 *******************************************************************************/
static bool phNxpUciHal_is_retry_not_required(uint8_t uci_octet0) {
  bool isRetryRequired = false, isChained_cmd = false, isData_Msg = false;
  isChained_cmd = (bool)((uci_octet0 & UCI_PBF_ST_CONT) >> UCI_PBF_SHIFT);
  isData_Msg = ((uci_octet0 & UCI_MT_MASK) >> UCI_MT_SHIFT) == UCI_MT_DATA;
  isRetryRequired = isChained_cmd | isData_Msg;
  return isRetryRequired;
}

/******************************************************************************
 * Function         CountryCodeCapsGenTxPowerPacket
 *
 * Description      This function makes tx antenna power calibration command
 *                  with gtx_power[] + tx_power_offset
 *
 * Returns          true if packet has been updated
 *
 ******************************************************************************/
static bool CountryCodeCapsGenTxPowerPacket(uint8_t *packet, size_t packet_len, uint16_t *out_len)
{
  phNxpUciHal_Runtime_Settings_t *rt_set = &nxpucihal_ctrl.rt_settings;

  if (rt_set->tx_power_offset == 0)
    return false;

  if (gtx_power.empty())
    return false;

  if (gtx_power.size() > packet_len)
    return false;

  uint16_t gtx_power_len = gtx_power.size();
  memcpy(packet, gtx_power.data(), gtx_power_len);
  uint8_t index = UCI_MSG_HDR_SIZE + 2;  // channel + Tag

  // Length
  if (index > gtx_power_len) {
    NXPLOG_UCIHAL_E("Upper-layer calibration command for tx_power is invalid.");
    return false;
  }
  if (!packet[index] || (packet[index] + index) > gtx_power_len) {
    NXPLOG_UCIHAL_E("Upper-layer calibration command for tx_power is invalid.");
    return false;
  }
  index++;

  // Value[0] = Number of antennas
  uint8_t num_of_antennas = packet[index++];

  while (num_of_antennas--) {
    // each entry = { antenna-id(1) + peak_tx(2) + id_rms(2) }
    if ((index + 5) < gtx_power_len) {
      NXPLOG_UCIHAL_E("Upper-layer calibration command for tx_power is invalid.");
      return false;
    }

    index += 3; // antenna Id(1) + Peak Tx(2)

    // Adjust id_rms part
    long tx_power_long = packet[index]  | ((packet[index + 1] << RMS_TX_POWER_SHIFT) & 0xFF00);
    tx_power_long += rt_set->tx_power_offset;

    if (tx_power_long < 0)
      tx_power_long = 0;

    uint16_t tx_power_u16 = (uint16_t)tx_power_long;
    packet[index++] = tx_power_u16 & 0xff;
    packet[index++] = tx_power_u16 >> RMS_TX_POWER_SHIFT;
  }

  if (out_len)
    *out_len = gtx_power_len;

  return true;
}

/*******************************************************************************
 * Function     phNxpUciHal_handle_set_calibration
 *
 * Description  Remembers SET_VENDOR_SET_CALIBRATION_CMD packet
 *
 * Returns      void
 *
 *******************************************************************************/
void phNxpUciHal_handle_set_calibration(const uint8_t *p_data, uint16_t data_len)
{
  // Only saves the SET_CALIBRATION_CMD from upper-layer
  if (nxpucihal_ctrl.hal_ext_enabled) {
    return;
  }

  // SET_DEVICE_CALIBRATION_CMD Packet format: Channel(1) + TLV
  if (data_len < 6) {
    return;
  }
  const uint8_t channel = p_data[UCI_MSG_HDR_SIZE + 0];
  const uint8_t tag = p_data[UCI_MSG_HDR_SIZE + 1];
  if (tag != NXP_PARAM_ID_TX_POWER_PER_ANTENNA) {
    return;
  }

  phNxpUciHal_Runtime_Settings_t *rt_set = &nxpucihal_ctrl.rt_settings;

  // RMS Tx power -> Octet [4, 5] in calib data
  NXPLOG_UCIHAL_D("phNxpUciHal_handle_set_calibration channel=%u tx_power_offset=%d", channel, rt_set->tx_power_offset);

  // Backup the packet to gtx_power[]
  gtx_power = std::move(std::vector<uint8_t> {p_data, p_data + data_len});

  // Patch SET_CALIBRATION_CMD per gtx_power + tx_power_offset
  CountryCodeCapsGenTxPowerPacket(nxpucihal_ctrl.p_cmd_data, sizeof(nxpucihal_ctrl.p_cmd_data), &nxpucihal_ctrl.cmd_len);
}

/******************************************************************************
 * Function         CountryCodeCapsApplyTxPower
 *
 * Description      This function sets the TX power from COUNTRY_CODE_CAPS
 *
 * Returns          false if no tx_power_offset or no Upper-layer Calibration cmd was given
 *                  true if it was successfully applied.
 *
 ******************************************************************************/
static bool CountryCodeCapsApplyTxPower(void)
{
  if (gtx_power.empty())
    return false;

  // use whole packet as-is from upper-layer command (gtx_power[])
  std::vector<uint8_t> packet(gtx_power.size());
  uint16_t packet_size = 0;
  if (!CountryCodeCapsGenTxPowerPacket(packet.data(), packet.size(), &packet_size))
    return false;

  tHAL_UWB_STATUS status = phNxpUciHal_send_ext_cmd(packet_size, packet.data());
  if (status != UWBSTATUS_SUCCESS) {
      NXPLOG_UCIHAL_D("%s: send failed", __func__);
  }

  gtx_power.clear();

  return true;
}

// Channels
const static uint8_t cal_channels[] = {5, 6, 8, 9};

static void extcal_do_xtal(void)
{
  int ret;

  // RF_CLK_ACCURACY_CALIB (otp supported)
  // parameters: cal.otp.xtal=0|1, cal.xtal=X
  uint8_t otp_xtal_flag = 0;
  uint8_t xtal_data[32];
  size_t xtal_data_len = 0;

  if (NxpConfig_GetNum("cal.otp.xtal", &otp_xtal_flag, 1) && otp_xtal_flag) {
    nxpucihal_ctrl.uwb_chip->read_otp(EXTCAL_PARAM_CLK_ACCURACY, xtal_data, sizeof(xtal_data), &xtal_data_len);
  }
  if (!xtal_data_len) {
    long retlen = 0;
    if (NxpConfig_GetByteArray("cal.xtal", xtal_data, sizeof(xtal_data), &retlen)) {
      xtal_data_len = retlen;
    }
  }

  if (xtal_data_len) {
    NXPLOG_UCIHAL_D("Apply CLK_ACCURARY (len=%zu, from-otp=%c)", xtal_data_len, otp_xtal_flag ? 'y' : 'n');

    ret = nxpucihal_ctrl.uwb_chip->apply_calibration(EXTCAL_PARAM_CLK_ACCURACY, 0, xtal_data, xtal_data_len);

    if (ret != UWBSTATUS_SUCCESS) {
      NXPLOG_UCIHAL_E("Failed to apply CLK_ACCURACY (len=%zu, from-otp=%c)",
          xtal_data_len, otp_xtal_flag ? 'y' : 'n');
    }
  }
}

static void extcal_do_ant_delay(void)
{
  std::bitset<8> rx_antenna_mask(nxpucihal_ctrl.cal_rx_antenna_mask);
  const uint8_t n_rx_antennas = rx_antenna_mask.size();

  // RX_ANT_DELAY_CALIB
  // parameter: cal.ant<N>.ch<N>.ant_delay=X
  // N(1) + N * {AntennaID(1), Rxdelay(Q14.2)}
  if (n_rx_antennas) {

    const int16_t extra_delay = nxpucihal_ctrl.uwb_chip->extra_group_delay();

    if (extra_delay) {
      NXPLOG_UCIHAL_D("RX_ANT_DELAY_CALIB: Extra compensation '%d'", extra_delay);
    }

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

        delay_value = delay_value + extra_delay;
        NXPLOG_UCIHAL_D("Apply RX_ANT_DELAY_CALIB: %s = %u", key, delay_value);
        entries.push_back(ant_id);
        // Little Endian
        entries.push_back(delay_value & 0xff);
        entries.push_back(delay_value >> 8);
        n_entries++;
      }

      if (!n_entries)
        continue;

      entries.insert(entries.begin(), n_entries);
      tHAL_UWB_STATUS ret = nxpucihal_ctrl.uwb_chip->apply_calibration(EXTCAL_PARAM_RX_ANT_DELAY, ch, entries.data(), entries.size());
      if (ret != UWBSTATUS_SUCCESS) {
        NXPLOG_UCIHAL_E("Failed to apply RX_ANT_DELAY for channel %u", ch);
      }
    }
  }
}

static void extcal_do_tx_power(void)
{
  std::bitset<8> tx_antenna_mask(nxpucihal_ctrl.cal_tx_antenna_mask);
  const uint8_t n_tx_antennas = tx_antenna_mask.size();

  // TX_POWER
  // parameter: cal.ant<N>.ch<N>.tx_power={...}
  if (n_tx_antennas) {
    for (auto ch : cal_channels) {
      std::vector<uint8_t> entries;
      uint8_t n_entries = 0;

      for (auto i = 0; i < n_tx_antennas; i++) {
        if (!tx_antenna_mask[i])
          continue;

        char key[32];
        const uint8_t ant_id = i + 1;
        std::snprintf(key, sizeof(key), "cal.ant%u.ch%u.tx_power", ant_id, ch);

        uint8_t power_value[32];
        long retlen = 0;
        if (!NxpConfig_GetByteArray(key, power_value, sizeof(power_value), &retlen)) {
          continue;
        }

        NXPLOG_UCIHAL_D("Apply TX_POWER: %s = { %lu bytes }", key, retlen);
        entries.push_back(ant_id);
        entries.insert(entries.end(), power_value, power_value + retlen);
        n_entries++;
      }

      if (!n_entries)
        continue;

      entries.insert(entries.begin(), n_entries);
      tHAL_UWB_STATUS ret = nxpucihal_ctrl.uwb_chip->apply_calibration(EXTCAL_PARAM_TX_POWER, ch, entries.data(), entries.size());
      if (ret != UWBSTATUS_SUCCESS) {
        NXPLOG_UCIHAL_E("Failed to apply TX_POWER for channel %u", ch);
      }
    }
  }
}

static void extcal_do_tx_pulse_shape(void)
{
  // parameters: cal.tx_pulse_shape={...}
  long retlen = 0;
  uint8_t data[64];

  if (NxpConfig_GetByteArray("cal.tx_pulse_shape", data, sizeof(data), &retlen) && retlen) {
      NXPLOG_UCIHAL_D("Apply TX_PULSE_SHAPE: data = { %lu bytes }", retlen);

      tHAL_UWB_STATUS ret = nxpucihal_ctrl.uwb_chip->apply_calibration(EXTCAL_PARAM_TX_PULSE_SHAPE, 0, data, (size_t)retlen);
      if (ret != UWBSTATUS_SUCCESS) {
        NXPLOG_UCIHAL_E("Failed to apply TX_PULSE_SHAPE.");
      }
  }
}

static void extcal_do_tx_base_band(void)
{
  // TX_BASE_BAND_CONTROL, DDFS_TONE_CONFIG
  // parameters: cal.ddfs_enable=1|0, cal.dc_suppress=1|0, ddfs_tone_config={...}
  uint8_t ddfs_enable = 0, dc_suppress = 0;
  uint8_t ddfs_tone[256];
  long retlen = 0;
  tHAL_UWB_STATUS ret;

  if (NxpConfig_GetNum("cal.ddfs_enable", &ddfs_enable, 1)) {
    NXPLOG_UCIHAL_D("Apply TX_BASE_BAND_CONTROL: ddfs_enable=%u", ddfs_enable);
  }
  if (NxpConfig_GetNum("cal.dc_suppress", &dc_suppress, 1)) {
    NXPLOG_UCIHAL_D("Apply TX_BASE_BAND_CONTROL: dc_suppress=%u", dc_suppress);
  }

  // DDFS_TONE_CONFIG
  if (ddfs_enable) {
    if (!NxpConfig_GetByteArray("cal.ddfs_tone_config", ddfs_tone, sizeof(ddfs_tone), &retlen) || !retlen) {
      NXPLOG_UCIHAL_E("cal.ddfs_tone_config is not supplied while cal.ddfs_enable=1, ddfs was not enabled.");
      ddfs_enable = 0;
    } else {
      NXPLOG_UCIHAL_D("Apply DDFS_TONE_CONFIG: ddfs_tone_config = { %lu bytes }", retlen);

      ret = nxpucihal_ctrl.uwb_chip->apply_calibration(EXTCAL_PARAM_DDFS_TONE_CONFIG, 0, ddfs_tone, (size_t)retlen);
      if (ret != UWBSTATUS_SUCCESS) {
        NXPLOG_UCIHAL_E("Failed to apply DDFS_TONE_CONFIG, ddfs was not enabled.");
        ddfs_enable = 0;
      }
    }
  }

  // TX_BASE_BAND_CONTROL
  {
    uint8_t flag = 0;
    if (ddfs_enable)
      flag |= 0x01;
    if (dc_suppress)
      flag |= 0x02;
    ret = nxpucihal_ctrl.uwb_chip->apply_calibration(EXTCAL_PARAM_TX_BASE_BAND_CONTROL, 0, &flag, 1);
    if (ret) {
      NXPLOG_UCIHAL_E("Failed to apply TX_BASE_BAND_CONTROL");
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
  // read rx_aantenna_mask, tx_antenna_mask
  uint8_t rx_antenna_mask_n = 0x1;
  uint8_t tx_antenna_mask_n = 0x1;
  if (!NxpConfig_GetNum("cal.rx_antenna_mask", &rx_antenna_mask_n, 1)) {
      NXPLOG_UCIHAL_E("cal.rx_antenna_mask is not specified, use default 0x%x", rx_antenna_mask_n);
  }
  if (!NxpConfig_GetNum("cal.tx_antenna_mask", &tx_antenna_mask_n, 1)) {
      NXPLOG_UCIHAL_E("cal.tx_antenna_mask is not specified, use default 0x%x", tx_antenna_mask_n);
  }
  nxpucihal_ctrl.cal_rx_antenna_mask = rx_antenna_mask_n;
  nxpucihal_ctrl.cal_tx_antenna_mask = tx_antenna_mask_n;

  extcal_do_xtal();
  extcal_do_ant_delay();
}

void apply_per_country_calibrations(void)
{
  // TX-POWER can be provided by
  // 1) COUNTRY_CODE_CAPS with offset values.
  // 2) Extra calibration files with absolute tx power values
  // only one should be applied if both were provided by platform
  if (!CountryCodeCapsApplyTxPower()) {
    extcal_do_tx_power();
  }

  // These are only available from extra calibration files
  extcal_do_tx_pulse_shape();
  extcal_do_tx_base_band();

}

/******************************************************************************
 * Function         phNxpUciHal_handle_set_country_code
 *
 * Description      Apply per-country settings
 *
 * Returns          void.
 *
 ******************************************************************************/
void phNxpUciHal_handle_set_country_code(const char country_code[2])
{
  NXPLOG_UCIHAL_D("Apply country code %c%c", country_code[0], country_code[1]);

  phNxpUciHal_Runtime_Settings_t *rt_set = &nxpucihal_ctrl.rt_settings;
  phNxpUciHal_resetRuntimeSettings();

  if (!is_valid_country_code(country_code)) {
    NXPLOG_UCIHAL_D("Country code %c%c is invalid, UWB should be disabled", country_code[0], country_code[1]);
  }

  if (NxpConfig_SetCountryCode(country_code)) {
    // Load ExtraCal restrictions
    uint16_t mask= 0;
    if (NxpConfig_GetNum("cal.restricted_channels", &mask, sizeof(mask))) {
      NXPLOG_UCIHAL_D("Restriction flag, restricted channel mask=0x%x", mask);
      rt_set->restricted_channel_mask = mask;
    }

    uint8_t uwb_disable = 0;
    if (NxpConfig_GetNum("cal.uwb_disable", &uwb_disable, sizeof(uwb_disable))) {
      NXPLOG_UCIHAL_D("Restriction flag, uwb_disable=%u", uwb_disable);
      rt_set->uwb_enable = !uwb_disable;
    }

    // Apply COUNTRY_CODE_CAPS
    uint8_t cc_caps[UCI_MAX_DATA_LEN];
    long retlen = 0;
    if (NxpConfig_GetByteArray(NAME_NXP_UWB_COUNTRY_CODE_CAPS, cc_caps, sizeof(cc_caps), &retlen) && retlen) {
      NXPLOG_UCIHAL_D("COUNTRY_CODE_CAPS is provided.");
      phNxpUciHal_applyCountryCaps(country_code, cc_caps, retlen);
    }

    // Check country code validity
    if (!is_valid_country_code(country_code) && rt_set->uwb_enable) {
      NXPLOG_UCIHAL_E("UWB is not disabled by configuration files with invalid country code %c%c,"
                      " forcing it disabled", country_code[0], country_code[1]);
      rt_set->uwb_enable = false;
    }

    // Apply per-country calibration, it's handled by SessionTrack
    SessionTrack_onCountryCodeChanged();
  }

  // send country code response to upper layer
  nxpucihal_ctrl.rx_data_len = 5;
  static uint8_t rsp_data[5] = { 0x4c, 0x01, 0x00, 0x01 };
  if (rt_set->uwb_enable) {
    rsp_data[4] = UWBSTATUS_SUCCESS;
  } else {
    rsp_data[4] = UCI_STATUS_CODE_ANDROID_REGULATION_UWB_OFF;
  }
  (*nxpucihal_ctrl.p_uwb_stack_data_cback)(nxpucihal_ctrl.rx_data_len, rsp_data);
}

// TODO: support fragmented packets
/*************************************************************************************
 * Function         phNxpUciHal_handle_set_app_config
 *
 * Description      Handle SESSION_SET_APP_CONFIG_CMD packet,
 *                  remove unsupported parameters
 *
 * Returns          true  : SESSION_SET_APP_CONFIG_CMD/RSP was handled by this function
 *                  false : This packet should go to chip
 *
 *************************************************************************************/
bool phNxpUciHal_handle_set_app_config(uint16_t *data_len, uint8_t *p_data)
{
  const phNxpUciHal_Runtime_Settings_t *rt_set = &nxpucihal_ctrl.rt_settings;
  // Android vendor specific app configs not supported by FW
  const uint8_t tags_to_del[] = {
    UCI_PARAM_ID_TX_ADAPTIVE_PAYLOAD_POWER,
    UCI_PARAM_ID_AOA_AZIMUTH_MEASUREMENTS,
    UCI_PARAM_ID_AOA_ELEVATION_MEASUREMENTS,
    UCI_PARAM_ID_RANGE_MEASUREMENTS
  };

  // check basic validity
  uint16_t payload_len = (p_data[UCI_CMD_LENGTH_PARAM_BYTE1] & 0xFF) |
                         ((p_data[UCI_CMD_LENGTH_PARAM_BYTE2] & 0xFF) << 8);
  if (payload_len != (*data_len - UCI_MSG_HDR_SIZE)) {
    NXPLOG_UCIHAL_E("SESSION_SET_APP_CONFIG_CMD: payload length mismatch");
    return false;
  }
  if (!p_data[UCI_CMD_NUM_CONFIG_PARAM_BYTE]) {
    return false;
  }

  uint32_t session_handle = le_bytes_to_cpu<uint32_t>(&p_data[UCI_MSG_SESSION_SET_APP_CONFIG_HANDLE_OFFSET]);
  uint8_t ch = 0;

  // Create local copy of cmd_data for data manipulation
  uint8_t uciCmd[UCI_MAX_DATA_LEN];
  uint16_t packet_len = *data_len;
  if (sizeof(uciCmd) < packet_len) {
    NXPLOG_UCIHAL_E("SESSION_SET_APP_CONFIG_CMD packet size %u is too big to handle, skip patching.", packet_len);
    return false;
  }
  // 9 = Header 4 + SessionID 4 + NumOfConfigs 1
  uint8_t i = 9, j = 9;
  uint8_t nr_deleted = 0, bytes_deleted = 0;
  memcpy(uciCmd, p_data, i);

  while (i < packet_len) {
    if ( (i + 2) >= packet_len) {
      NXPLOG_UCIHAL_E("SESSION_SET_APP_CONFIG_CMD parse error at %u", i);
      return false;
    }

    // All parameters should have 1 byte tag
    uint8_t tlv_tag = p_data[i + 0];
    uint8_t tlv_len = p_data[i + 1];
    uint8_t param_len = 2 + tlv_len;
    if ((i + param_len) > packet_len) {
      NXPLOG_UCIHAL_E("SESSION_SET_APP_CONFIG_CMD parse error at %u", i);
    }

    // check restricted channel
    if (tlv_tag == UCI_PARAM_ID_CHANNEL_NUMBER && tlv_len == 1) {
      ch = p_data[i + 2];

      if (((ch == CHANNEL_NUM_5) && (rt_set->restricted_channel_mask & (1 << 5))) ||
          ((ch == CHANNEL_NUM_9) && (rt_set->restricted_channel_mask & (1 << 9)))) {
        phNxpUciHal_print_packet(NXP_TML_UCI_CMD_AP_2_UWBS, p_data, packet_len);
        NXPLOG_UCIHAL_D("Country code blocked channel %u", ch);

        // send setAppConfig response with UCI_STATUS_CODE_ANDROID_REGULATION_UWB_OFF response
        static uint8_t rsp_data[] = { 0x41, 0x03, 0x04, 0x04,
          UCI_STATUS_FAILED, 0x01, tlv_tag, UCI_STATUS_CODE_ANDROID_REGULATION_UWB_OFF
        };
        nxpucihal_ctrl.rx_data_len = sizeof(rsp_data);
        (*nxpucihal_ctrl.p_uwb_stack_data_cback)(nxpucihal_ctrl.rx_data_len, rsp_data);
        return true;
      }
    }

    // remove unsupported parameters
    if (std::find(std::begin(tags_to_del), std::end(tags_to_del), tlv_tag) == std::end(tags_to_del)) {
      memcpy(&uciCmd[j], &p_data[i], param_len);
      j += param_len;
    } else {
      NXPLOG_UCIHAL_D("Removed param payload with Tag ID:0x%02x", tlv_tag);
      nr_deleted++;
      bytes_deleted += param_len;
    }
    i += param_len;
  }
  if (nr_deleted) {
    phNxpUciHal_print_packet(NXP_TML_UCI_CMD_AP_2_UWBS, p_data, packet_len);

    // uci number of config params update
    if (uciCmd[UCI_CMD_NUM_CONFIG_PARAM_BYTE] < nr_deleted) {
      NXPLOG_UCIHAL_E("SESSION_SET_APP_CONFIG_CMD cannot parse packet: wrong nr_parameters");
      return false;
    }
    uciCmd[UCI_CMD_NUM_CONFIG_PARAM_BYTE] -= nr_deleted;

    // uci command length update
    if (packet_len < (UCI_MSG_HDR_SIZE + bytes_deleted)) {
      NXPLOG_UCIHAL_E("SESSION_SET_APP_CONFIG_CMD cannot parse packet: underflow");
      return false;
    }
    packet_len -= bytes_deleted;
    payload_len = packet_len - UCI_MSG_HDR_SIZE;
    uciCmd[UCI_CMD_LENGTH_PARAM_BYTE2] = (payload_len & 0xFF00) >> 8;
    uciCmd[UCI_CMD_LENGTH_PARAM_BYTE1] = (payload_len & 0xFF);

    // Swap
    memcpy(p_data, uciCmd, packet_len);
    *data_len = packet_len;
  }

  SessionTrack_onAppConfig(session_handle, ch);

  return false;
}

void phNxpUciHal_handle_get_caps_info(uint16_t data_len, uint8_t *p_data)
{
  if (data_len < UCI_MSG_CORE_GET_CAPS_INFO_NR_OFFSET)
    return;

  uint8_t status = p_data[UCI_RESPONSE_STATUS_OFFSET];
  uint8_t nr = p_data[UCI_MSG_CORE_GET_CAPS_INFO_NR_OFFSET];
  if (status != UWBSTATUS_SUCCESS || nr < 1)
    return;

  auto tlvs = decodeTlvBytes({0xe0, 0xe1, 0xe2, 0xe3}, &p_data[UCI_MSG_CORE_GET_CAPS_INFO_TLV_OFFSET], data_len - UCI_MSG_CORE_GET_CAPS_INFO_TLV_OFFSET);
  if (tlvs.size() != nr) {
    NXPLOG_UCIHAL_E("Failed to parse DevCaps %zu != %u", tlvs.size(), nr);
  }

  // Remove all NXP vendor specific parameters
  for (auto it = tlvs.begin(); it != tlvs.end();) {
    if (it->first > 0xff)
      it = tlvs.erase(it);
    else
      it++;
  }

  // Override AOA_SUPPORT_TAG_ID
  auto it = tlvs.find(AOA_SUPPORT_TAG_ID);
  if (it != tlvs.end()) {
    if (nxpucihal_ctrl.numberOfAntennaPairs == 1) {
      it->second = std::vector<uint8_t>{0x01};
    } else if (nxpucihal_ctrl.numberOfAntennaPairs > 1) {
      it->second = std::vector<uint8_t>{0x05};
    } else {
      it->second = std::vector<uint8_t>{0x00};
    }
  }

  // Byteorder of CCC_SUPPORTED_PROTOCOL_VERSIONS_ID
  it = tlvs.find(CCC_SUPPORTED_PROTOCOL_VERSIONS_ID);
  if (it != tlvs.end() && it->second.size() == 2) {
    std::swap(it->second[0], it->second[1]);
  }

  // Append UWB_VENDOR_CAPABILITY from configuration files
  {
    std::array<uint8_t, NXP_MAX_CONFIG_STRING_LEN> buffer;
    long retlen = 0;
    if (NxpConfig_GetByteArray(NAME_UWB_VENDOR_CAPABILITY, buffer.data(),
                               buffer.size(), &retlen) && retlen) {
      auto vendorTlvs = decodeTlvBytes({}, buffer.data(), retlen);
      for (auto const& [key, val] : vendorTlvs) {
        tlvs[key] = val;
      }
    }
  }

  // Apply restrictions
  const phNxpUciHal_Runtime_Settings_t *rt_set = &nxpucihal_ctrl.rt_settings;

  uint8_t fira_channels = 0xff;
  if (rt_set->restricted_channel_mask & (1 << 5))
    fira_channels &= CHANNEL_5_MASK;
  if (rt_set->restricted_channel_mask & (1 << 9))
    fira_channels &= CHANNEL_9_MASK;

  uint8_t ccc_channels = 0;
  if (!(rt_set->restricted_channel_mask & (1 << 5)))
    ccc_channels |= 0x01;
  if (!(rt_set->restricted_channel_mask & (1 << 9)))
    ccc_channels |= 0x02;

  tlvs[UWB_CHANNELS] = std::vector{fira_channels};
  tlvs[CCC_UWB_CHANNELS] = std::vector{ccc_channels};

  // Convert it back to raw packet bytes
  uint8_t packet[256];

  // header
  memcpy(packet, p_data, UCI_MSG_HDR_SIZE);
  // status
  packet[UCI_RESPONSE_STATUS_OFFSET] = UWBSTATUS_SUCCESS;
  // nr
  packet[UCI_MSG_CORE_GET_CAPS_INFO_NR_OFFSET] = tlvs.size();

  // tlvs
  auto tlv_bytes = encodeTlvBytes(tlvs);
  if ((tlv_bytes.size() + UCI_MSG_CORE_GET_CAPS_INFO_TLV_OFFSET) > sizeof(packet)) {
    NXPLOG_UCIHAL_E("DevCaps overflow!");
  } else {
    uint8_t packet_len = UCI_MSG_CORE_GET_CAPS_INFO_TLV_OFFSET + tlv_bytes.size();
    packet[UCI_PAYLOAD_LENGTH_OFFSET] = packet_len - UCI_MSG_HDR_SIZE;
    memcpy(&packet[UCI_MSG_CORE_GET_CAPS_INFO_TLV_OFFSET], tlv_bytes.data(), tlv_bytes.size());

    phNxpUciHal_print_packet(NXP_TML_UCI_RSP_NTF_UWBS_2_AP, packet, packet_len);

    // send GET CAPS INFO response to the Upper Layer
    (*nxpucihal_ctrl.p_uwb_stack_data_cback)(packet_len, packet);
    // skip the incoming packet as we have send the modified response
    // already
    nxpucihal_ctrl.isSkipPacket = 1;
  }
}
