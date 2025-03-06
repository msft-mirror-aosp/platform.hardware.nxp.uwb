/*
 * Copyright 2018-2024 NXP
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

/******************************************************************************
 *
 *  This file contains the definition from UCI specification
 *
 ******************************************************************************/

#ifndef UWB_UCI_DEFS_H
#define UWB_UCI_DEFS_H

#include <stdint.h>

/* Define the message header size for all UCI Commands and Notifications */
#define UCI_MSG_HDR_SIZE 4 /* per UCI spec */
#define UCI_RESPONSE_STATUS_OFFSET 4
#define UCI_CMD_SESSION_ID_OFFSET 4
#define UCI_RESPONSE_PAYLOAD_OFFSET 5

/* UCI Command and Notification Format:
 * 4 byte message header:
 * byte 0: MT PBF GID
 * byte 1: OID
 * byte 2: RFU - To be used for extended playload length
 * byte 3: Message Length */

/* MT: Message Type (byte 0) */
#define UCI_MT_MASK 0xE0
#define UCI_MT_SHIFT 5
#define UCI_MT_DATA 0
#define UCI_MT_CMD 1 /* (UCI_MT_CMD << UCI_MT_SHIFT) = 0x20 */
#define UCI_MT_RSP 2 /* (UCI_MT_RSP << UCI_MT_SHIFT) = 0x40 */
#define UCI_MT_NTF 3 /* (UCI_MT_NTF << UCI_MT_SHIFT) = 0x60 */

/* PBF: Packet Boundary Flag (byte 0) */
#define UCI_PBF_MASK 0x10
#define UCI_PBF_SHIFT 4
#define UCI_PBF_ST_CONT 0x10    /* start or continuing fragment */

/* Ocet 3 = Payload Length(L) */
#define UCI_PAYLOAD_LENGTH_OFFSET 3

/* GID: Group Identifier (byte 0) */
#define UCI_GID_MASK              0x0F
#define UCI_GID_CORE              0x00 /* UCI Core group */
#define UCI_GID_SESSION_MANAGE    0x01 /* Session Config Group */
#define UCI_GID_SESSION_CONTROL   0x02 /* Session Control Group */
#define UCI_GID_ANDROID           0x0C /* Android vendor group */
#define UCI_GID_PROPRIETARY_0X0A  0x0A /* Proprietary Group */
#define UCI_GID_PROPRIETARY       0x0E /* Proprietary Group */
#define UCI_GID_PROPRIETARY_0X0F  0x0F /* Proprietary Group */
#define UCI_GID_INTERNAL          0x0B /* Internal Group */

/* OID: Opcode Identifier (byte 1) */
#define UCI_OID_MASK 0x3F
#define UCI_OID_SHIFT 0

/**********************************************
 * UCI Core Group-0: Opcodes and size of commands
 **********************************************/
#define UCI_MSG_CORE_DEVICE_STATUS_NTF 1
#define UCI_MSG_CORE_DEVICE_INFO 2
#define UCI_MSG_CORE_GET_CAPS_INFO 3
#define UCI_MSG_CORE_GET_CAPS_INFO_NR_OFFSET  5
#define UCI_MSG_CORE_GET_CAPS_INFO_TLV_OFFSET 6

#define UCI_MSG_CORE_SET_CONFIG 4
#define UCI_MSG_CORE_GENERIC_ERROR_NTF 7

/*********************************************************
 * UCI session config Group-1: Opcodes and size of command
 ********************************************************/
#define UCI_MSG_SESSION_STATUS_NTF 2
#define UCI_MSG_SESSION_STATUS_NTF_HANDLE_OFFSET      4
#define UCI_MSG_SESSION_STATUS_NTF_STATE_OFFSET       8
#define UCI_MSG_SESSION_STATUS_NTF_REASON_OFFSET      9
#define UCI_MSG_SESSION_STATUS_NTF_LENGTH             10

#define UCI_MSG_SESSION_STATE_INIT                    (0x00)
#define UCI_MSG_SESSION_STATE_INIT_CMD_LEN            (9)
#define UCI_MSG_SESSION_STATE_INIT_CMD_ID_OFFSET      (4)
#define UCI_MSG_SESSION_STATE_INIT_CMD_TYPE_OFFSET    (8)
#define UCI_MSG_SESSION_STATE_INIT_RSP_LEN            (9)
#define UCI_MSG_SESSION_STATE_INIT_RSP_STATUS_OFFSET  (4)
#define UCI_MSG_SESSION_STATE_INIT_RSP_HANDLE_OFFSET  (5)
#define UCI_MSG_SESSION_STATE_DEINIT                  (0x01)
#define UCI_MSG_SESSION_STATE_ACTIVE                  (0x02)
#define UCI_MSG_SESSION_STATE_IDLE                    (0x03)
#define UCI_MSG_SESSION_STATE_UNDEFINED               (0xFF)  // SW defined

#define UCI_MSG_SESSION_SET_APP_CONFIG                3
#define UCI_MSG_SESSION_SET_APP_CONFIG_HANDLE_OFFSET  4
#define UCI_MSG_SESSION_GET_APP_CONFIG                4

// Parameter Tags
#define UCI_APP_CONFIG_FIRA_STS_INDEX                 0x0A
#define UCI_APP_CONFIG_CCC_LAST_STS_INDEX_USED        0xA8

#define UCI_MSG_SESSION_QUERY_DATA_SIZE               0x0B
#define UCI_MSG_SESSION_QUERY_DATA_SIZE_STATUS_OFFSET 8

// Session Type field in SESSION_INIT_CMD
constexpr uint8_t kSessionType_Ranging = 0x00;
constexpr uint8_t kSessionType_RangingAndData = 0x01;
constexpr uint8_t kSessionType_CCCRanging = 0xA0;

/*********************************************************
 * UCI session config Group-2: Opcodes and size of command
 ********************************************************/
#define UCI_MSG_SESSION_START                         0x00
#define UCI_MSG_SESSION_START_CMD_LENGTH              (8)
#define UCI_MSG_SESSION_START_HANDLE_OFFSET           (4)

#define UCI_MSG_SESSION_STOP                          0x01

/**********************************************************
 * UCI Android Vendor Group-C: Opcodes and size of commands
 **********************************************************/
#define UCI_MSG_ANDROID_SET_COUNTRY_CODE 0x01

/**********************************************
 * UWB Prop Group Opcode-E Opcodes
 **********************************************/
#define UCI_MSG_BINDING_STATUS_NTF 0x06

/**********************************************************
 * OTP Calibration
 **********************************************************/
#define UCI_MSG_WRITE_CALIB_DATA  0x00
#define UCI_MSG_READ_CALIB_DATA   0x01

#define OTP_ID_XTAL_CAP_GM_CTRL   0x02

/**********************************************
 * UWB Prop Group Opcode-F Opcodes
 **********************************************/
#define UCI_MSG_URSK_DELETE               0x01
#define UCI_MSG_SET_DEVICE_CALIBRATION    0x21
#define UCI_MSG_GET_DEVICE_CALIBRATION    0x22
#define UCI_MSG_UWB_ESE_BINDING           0x31
#define UCI_MSG_UWB_ESE_BINDING_CHECK     0x32

/**********************************************
 * UCI Parameter IDs : Device Configurations
 **********************************************/
#define UCI_PARAM_ID_LOW_POWER_MODE 0x01

/*************************************************
 * UCI Parameter IDs : Application Configurations
 ************************************************/
#define UCI_PARAM_ID_CHANNEL_NUMBER 0x04
#define UCI_PARAM_ID_TX_ADAPTIVE_PAYLOAD_POWER  0x1C  /* 2.0.0-0.9r4 CR-1038 removed this */
#define UCI_PARAM_ID_AOA_AZIMUTH_MEASUREMENTS 0xE3
#define UCI_PARAM_ID_AOA_ELEVATION_MEASUREMENTS 0xE4
#define UCI_PARAM_ID_RANGE_MEASUREMENTS 0xE5

/*************************************************
 * Device Calibration Parameters IDs
 ************************************************/
// XXX: Should this be chip-dependant?
#define NXP_PARAM_ID_TX_POWER_PER_ANTENNA       0x04

/*************************************************
 * Status codes
 ************************************************/
/* Generic Status Codes */
#define UCI_STATUS_OK 0x00
#define UCI_STATUS_FAILED 0x02
#define UCI_STATUS_SYNTAX_ERROR 0x03
#define UCI_STATUS_INVALID_PARAM 0x04
#define UCI_STATUS_COMMAND_RETRY 0x0A
#define UCI_STATUS_UNKNOWN 0x0B
#define UCI_STATUS_THERMAL_RUNAWAY 0x54
#define UCI_STATUS_BUFFER_UNDERFLOW 0x58
#define UCI_STATUS_LOW_VBAT 0x59
#define UCI_STATUS_HW_RESET 0xFE
#define UWBS_STATUS_ERROR 0xFF /* error occurred in UWBS*/

/* Status code for feature not supported */
#define UCI_STATUS_FEATURE_NOT_SUPPORTED 0x55

#define UCI_STATUS_COUNTRY_CODE_BLOCKED_CHANNEL 0x56
#define UCI_STATUS_CODE_ANDROID_REGULATION_UWB_OFF 0x53

#endif /* UWB_UCI_DEFS_H */
