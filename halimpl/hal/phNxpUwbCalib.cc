/*
 * Copyright 2021-2023 NXP
 * Copyright 2024 Google
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


#include "phNxpUwbCalib.h"
#include "phUwbStatus.h"
#include "phNxpUciHal_ext.h"

//
// SR1XX Device Calibrations:
//
// Based on NXP SR1xx UCI v2.0.5
// current HAL impl only supports "xtal" read from otp
// others should be existed in .conf files

static tHAL_UWB_STATUS sr1xx_set_calibration(uint8_t channel, const std::vector<uint8_t> &tlv)
{
  // SET_CALIBRATION_CMD header: GID=0xF OID=0x21
  std::vector<uint8_t> packet({ (0x20 | UCI_GID_PROPRIETARY_0X0F), UCI_MSG_SET_DEVICE_CALIBRATION, 0x00, 0x00});

  // use 9 for channel-independent parameters
  if (!channel) {
    channel = 9;
  }
  packet.push_back(channel);
  packet.insert(packet.end(), tlv.begin(), tlv.end());
  packet[3] = packet.size() - 4;
  return phNxpUciHal_send_ext_cmd(packet.size(), packet.data());
}

static tHAL_UWB_STATUS sr1xx_set_conf(const std::vector<uint8_t> &tlv)
{
  // SET_CALIBRATION_CMD header: GID=0xF OID=0x21
  std::vector<uint8_t> packet({ (0x20 | UCI_GID_CORE), UCI_MSG_CORE_SET_CONFIG, 0x00, 0x00});
  packet.push_back(1);  // number of parameters
  packet.insert(packet.end(), tlv.begin(), tlv.end());
  packet[3] = packet.size() - 4;
  return phNxpUciHal_send_ext_cmd(packet.size(), packet.data());
}

tHAL_UWB_STATUS sr1xx_apply_calibration(extcal_param_id_t id, const uint8_t ch, const uint8_t *data, size_t data_len)
{
  // Device Calibration
  const uint8_t UCI_PARAM_ID_RF_CLK_ACCURACY_CALIB    = 0x01;
  const uint8_t UCI_PARAM_ID_RX_ANT_DELAY_CALIB       = 0x02;
  const uint8_t UCI_PARAM_ID_TX_POWER_PER_ANTENNA     = 0x04;

  // Device Configurations
  const uint16_t UCI_PARAM_ID_TX_BASE_BAND_CONFIG     = 0xe426;
  const uint16_t UCI_PARAM_ID_DDFS_TONE_CONFIG        = 0xe427;
  const uint16_t UCI_PARAM_ID_TX_PULSE_SHAPE_CONFIG   = 0xe428;

  switch (id) {
  case EXTCAL_PARAM_CLK_ACCURACY:
    {
      if (data_len != 6) {
        return UWBSTATUS_FAILED;
      }

      std::vector<uint8_t> tlv;
      // Tag
      tlv.push_back(UCI_PARAM_ID_RF_CLK_ACCURACY_CALIB);
      // Length
      tlv.push_back((uint8_t)data_len + 1);
      // Value
      tlv.push_back(3); // number of register (must be 0x03)
      tlv.insert(tlv.end(), data, data + data_len);

      return sr1xx_set_calibration(ch, tlv);
    }
  case EXTCAL_PARAM_RX_ANT_DELAY:
    {
      if (!ch || data_len < 1 || !data[0] || (data[0] * 3) != (data_len - 1)) {
        return UWBSTATUS_FAILED;
      }

      std::vector<uint8_t> tlv;
      // Tag
      tlv.push_back(UCI_PARAM_ID_RX_ANT_DELAY_CALIB);
      // Length
      tlv.push_back((uint8_t)data_len);
      // Value
      tlv.insert(tlv.end(), data, data + data_len);

      return sr1xx_set_calibration(ch, tlv);
    }
  case EXTCAL_PARAM_TX_POWER:
    {
      if (!ch || data_len < 1 || !data[0] || (data[0] * 5) != (data_len - 1)) {
        return UWBSTATUS_FAILED;
      }

      std::vector<uint8_t> tlv;
      // Tag
      tlv.push_back(UCI_PARAM_ID_TX_POWER_PER_ANTENNA);
      // Length
      tlv.push_back((uint8_t)data_len);
      // Value
      tlv.insert(tlv.end(), data, data + data_len);

      return sr1xx_set_calibration(ch, tlv);
    }
  case EXTCAL_PARAM_TX_BASE_BAND_CONTROL:
    {
      if (data_len != 1) {
        return UWBSTATUS_FAILED;
      }

      std::vector<uint8_t> tlv;
      // Tag
      tlv.push_back(UCI_PARAM_ID_TX_BASE_BAND_CONFIG >> 8);
      tlv.push_back(UCI_PARAM_ID_TX_BASE_BAND_CONFIG & 0xff);
      // Length
      tlv.push_back(1);
      // Value
      tlv.push_back(data[0]);

      return sr1xx_set_conf(tlv);
    }
  case EXTCAL_PARAM_DDFS_TONE_CONFIG:
    {
      if (!data_len) {
        return UWBSTATUS_FAILED;
      }

      std::vector<uint8_t> tlv;
      // Tag
      tlv.push_back(UCI_PARAM_ID_DDFS_TONE_CONFIG >> 8);
      tlv.push_back(UCI_PARAM_ID_DDFS_TONE_CONFIG & 0xff);
      // Length
      tlv.push_back(data_len);
      // Value
      tlv.insert(tlv.end(), data, data + data_len);

      return sr1xx_set_conf(tlv);
    }
  case EXTCAL_PARAM_TX_PULSE_SHAPE:
    {
      if (!data_len) {
        return UWBSTATUS_FAILED;
      }

      std::vector<uint8_t> tlv;
      // Tag
      tlv.push_back(UCI_PARAM_ID_TX_PULSE_SHAPE_CONFIG >> 8);
      tlv.push_back(UCI_PARAM_ID_TX_PULSE_SHAPE_CONFIG & 0xff);
      // Length
      tlv.push_back(data_len);
      // Value
      tlv.insert(tlv.end(), data, data + data_len);

      return sr1xx_set_conf(tlv);
    }
  default:
    NXPLOG_UCIHAL_E("Unsupported parameter: 0x%x", id);
    return UWBSTATUS_FAILED;
  }
}
