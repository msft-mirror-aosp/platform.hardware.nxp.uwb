/*
 * Copyright 2012-2023 NXP
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

#ifndef _PHNXPUCIHAL_H_
#define _PHNXPUCIHAL_H_

#include <thread>

#include "hal_nxpuwb.h"
#include "NxpUwbChip.h"
#include "phNxpUciHal_Adaptation.h"
#include "phNxpUciHal_utils.h"
#include "phTmlUwb.h"
#include "uci_defs.h"

/********************* Definitions and structures *****************************/
#define MAX_RETRY_COUNT 0x05
#define UCI_MAX_DATA_LEN 4200 // maximum data packet size
#define UCI_MAX_PAYLOAD_LEN 4200
// #define UCI_RESPONSE_STATUS_OFFSET 0x04
#define UCI_PKT_HDR_LEN 0x04
#define UCI_PKT_PAYLOAD_STATUS_LEN 0x01
#define UCI_PKT_NUM_CAPS_LEN 0x01
#define UCI_PKT_UCI_GENERIC 0x09
#define UWB_CHANNELS 0x0E
#define CCC_UWB_CHANNELS 0xA3
#define COUNTRY_CODE_TAG 0x00
#define UWB_ENABLE_TAG 0x01
#define CHANNEL_5_TAG 0x02
#define CHANNEL_9_TAG 0x03
#define TX_POWER_TAG 0x05
#define CHANNEL_5_MASK 0xFE
#define CHANNEL_9_MASK 0xF7
#define CHANNEL_NUM_5 0x05
#define CHANNEL_NUM_9 0x09
#define CCC_CHANNEL_INFO_BIT_MASK 0x03

#define MAX_RESPONSE_STATUS 0x0C

#define UCI_MT_MASK 0xE0
#define UCI_PBF_MASK 0x10
#define UCI_GID_MASK 0x0F
#define UCI_OID_MASK 0x3F
#define UCI_OID_RANGE_DATA_NTF 0x00

#define UCI_NTF_PAYLOAD_OFFSET 0x04
#define NORMAL_MODE_LENGTH_OFFSET 0x03
#define EXTENDED_MODE_LEN_OFFSET 0x02
#define EXTENDED_MODE_LEN_SHIFT 0x08
#define EXTND_LEN_INDICATOR_OFFSET 0x01
#define EXTND_LEN_INDICATOR_OFFSET_MASK 0x80
#define UCI_SESSION_ID_OFFSET 0x04
#define FWD_MAX_RETRY_COUNT 0x05

#define USER_FW_BOOT_MODE 0x01
#define FW_VERSION_PARAM_ID 0x01

#define FW_BOOT_MODE_PARAM_ID 0x63

#define CCC_SUPPORTED_PROTOCOL_VERSIONS_ID 0xA4

/* Low power mode */
#define LOW_POWER_MODE_TAG_ID 0x01
#define LOW_POWER_MODE_LENGTH 0x01

/* AOA support handling */
#define AOA_SUPPORT_TAG_ID 0x13
#define ANTENNA_RX_PAIR_DEFINE_TAG_ID 0xE4
#define ANTENNA_RX_PAIR_DEFINE_SUB_TAG_ID 0x62

#define DEVICE_NAME_PARAM_ID 0x00

/* Mem alloc. with 8 byte alignment */
#define nxp_malloc(x) malloc(((x - 1) | 7) + 1)

/* UCI Message set application config specific parameters*/
#define UCI_CMD_NUM_CONFIG_PARAM_LENGTH 1
#define UCI_CMD_NUM_CONFIG_PARAM_BYTE 8
#define UCI_CMD_LENGTH_PARAM_BYTE1 3
#define UCI_CMD_LENGTH_PARAM_BYTE2 2
#define UCI_CMD_TAG_BYTE_LENGTH 1
#define UCI_CMD_PARAM_SIZE_BYTE_LENGTH 1
#define UCI_CMD_PAYLOAD_BYTE_LENGTH 1

/* FW debug and crash log path */
const char debug_log_path[] = "/data/vendor/uwb/";

/* UCI Data */
#define NXP_MAX_CONFIG_STRING_LEN 260
typedef struct uci_data {
  uint16_t len;
  uint8_t p_data[UCI_MAX_DATA_LEN];
} uci_data_t;

typedef enum {
  HAL_STATUS_CLOSE = 0,
  HAL_STATUS_OPEN
} phNxpUci_HalStatus;

typedef enum {
  UWB_DEVICE_INIT = 0,
  UWB_DEVICE_READY,
  UWB_DEVICE_BUSY,
  UWB_DEVICE_STATE_UNKNOWN = 0XA0,
  UWB_DEVICE_ERROR = 0xFF
}phNxpUci_UwbcState;

typedef enum {
  UWB_DEVICE_NOT_BOUND = 0,
  UWB_DEVICE_BOUND_UNLOCKED,
  UWB_DEVICE_BOUND_LOCKED,
  UWB_DEVICE_UNKNOWN
} phNxpUci_UwbBindingStatus;

typedef enum {
  HAL_STATE_CLOSE = 0,
  HAL_STATE_CORE_INIT,
  HAL_STATE_OPEN,
  HAL_STATE_READ_BINDING_NTF
} phNxpUci_HalState;

/* Macros to enable and disable extensions */
#define HAL_ENABLE_EXT() (nxpucihal_ctrl.hal_ext_enabled = 1)
#define HAL_DISABLE_EXT() (nxpucihal_ctrl.hal_ext_enabled = 0)

typedef struct {
  uint8_t
      validation; /* indicates on which platform validation is done like SR100*/
  uint8_t android_version; /* android version */
  uint8_t major_version;   /* major version of the MW */
  uint8_t minor_version;   /* Minor Version of MW */
  uint8_t rc_version;      /* RC version */
  uint8_t mw_drop;         /* MW early drops */
} phNxpUciHal_MW_Version_t;

typedef struct {
  uint8_t major_version;  /* major */
  uint8_t minor_version;  /* minor/maintenance */
  uint8_t rc_version;     /* patch */
} phNxpUciHal_FW_Version_t;

typedef struct {
  uint16_t restricted_channel_mask;
  bool uwb_enable;
  short tx_power_offset;    // From UWB_COUNTRY_CODE_CAPS
} phNxpUciHal_Runtime_Settings_t;

/* UCI Control structure */
typedef struct phNxpUciHal_Control {
  phNxpUci_HalStatus halStatus; /* Indicate if hal is open or closed */
  std::thread client_thread;    /* Integration thread handle */
  phLibUwb_sConfig_t gDrvCfg;   /* Driver config data */

  std::unique_ptr<NxpUwbChip> uwb_chip;

  /* Rx data */
  uint8_t* p_rx_data;
  uint16_t rx_data_len;

  /* libuwb-uci callbacks */
  uwb_stack_callback_t* p_uwb_stack_cback;
  uwb_stack_data_callback_t* p_uwb_stack_data_cback;

  /* HAL extensions */
  uint8_t hal_ext_enabled;
  bool_t hal_parse_enabled;

  /* Waiting semaphore */
  phNxpUciHal_Sem_t ext_cb_data;

  // in case of fragmented response,
  // ext_cb_data is flagged only from the 1st response packet
  bool ext_cb_waiting;

  uint16_t cmd_len;
  uint8_t p_cmd_data[UCI_MAX_DATA_LEN];
  uint16_t rsp_len;
  uint8_t p_rsp_data[UCI_MAX_DATA_LEN];
  uint8_t p_caps_resp[UCI_MAX_DATA_LEN];

  /* CORE_DEVICE_INFO_RSP cache */
  bool isDevInfoCached;
  uint8_t dev_info_resp[256];

  phNxpUciHal_FW_Version_t fw_version;
  device_type_t device_type;
  uint8_t fw_boot_mode;

  /* To skip sending packets to upper layer from HAL*/
  uint8_t isSkipPacket;
  bool_t fw_dwnld_mode;

  // Per-country settings
  phNxpUciHal_Runtime_Settings_t rt_settings;

  // AOA support handling
  int numberOfAntennaPairs;

  // Extra calibration
  // Antenna Definitions for extra calibration, b0=Antenna1, b1=Antenna2, ...
  uint8_t cal_rx_antenna_mask;
  uint8_t cal_tx_antenna_mask;
} phNxpUciHal_Control_t;

// RX packet handler
struct phNxpUciHal_RxHandler;

/* Internal messages to handle callbacks */
#define UCI_HAL_OPEN_CPLT_MSG 0x411
#define UCI_HAL_CLOSE_CPLT_MSG 0x412
#define UCI_HAL_INIT_CPLT_MSG 0x413
#define UCI_HAL_ERROR_MSG 0x415

#define UCIHAL_CMD_CODE_LEN_BYTE_OFFSET (2U)
#define UCIHAL_CMD_CODE_BYTE_LEN (3U)

#define NXP_CHIP_SR100 1
#define NXP_CHIP_SR200 2
#define NXP_ANDROID_VERSION (14U)     /* Android version */
#define UWB_NXP_MW_VERSION_MAJ (0x00) /* MW major version */
#define UWB_NXP_MW_VERSION_MIN                                                 \
  (0x00) /* MS Nibble is MW minor version and LS Nibble is Test Object/Patch*/
#define UWB_NXP_ANDROID_MW_RC_VERSION (0x02)   /* Android MW RC Version */
#define UWB_NXP_ANDROID_MW_DROP_VERSION (0x07) /* Android MW early drops */
/******************** UCI HAL exposed functions *******************************/
tHAL_UWB_STATUS phNxpUciHal_init_hw();
tHAL_UWB_STATUS phNxpUciHal_write_unlocked(uint16_t data_len, const uint8_t *p_data);
void phNxpUciHal_read_complete(void* pContext, phTmlUwb_TransactInfo_t* pInfo);
tHAL_UWB_STATUS phNxpUciHal_uwb_reset();
tHAL_UWB_STATUS phNxpUciHal_applyVendorConfig();
tHAL_UWB_STATUS phNxpUciHal_process_ext_cmd_rsp(uint16_t cmd_len, const uint8_t *p_cmd, uint16_t *data_written);
void phNxpUciHal_send_dev_error_status_ntf();

std::shared_ptr<phNxpUciHal_RxHandler> phNxpUciHal_rx_handler_add(
  uint8_t mt, uint8_t gid, uint8_t oid,
  bool skip_reporting, bool run_once,
  std::function<void(size_t packet_len, const uint8_t *packet)> callback);
void phNxpUciHal_rx_handler_del(std::shared_ptr<phNxpUciHal_RxHandler> handler);

// Helper class for rx handler with once=false
// auto-unregistered from destructor
class UciHalRxHandler {
public:
  UciHalRxHandler() {
  }
  UciHalRxHandler(uint8_t mt, uint8_t gid, uint8_t oid,
                 bool skip_reporting,
                 std::function<void(size_t packet_len, const uint8_t *packet)> callback) {
    handler_ = phNxpUciHal_rx_handler_add(mt, gid, oid, skip_reporting, false, callback);
  }
  UciHalRxHandler& operator=(UciHalRxHandler &&handler) {
    handler_ = std::move(handler.handler_);
    return *this;
  }
  virtual ~UciHalRxHandler() {
    if (handler_) {
      phNxpUciHal_rx_handler_del(handler_);
      handler_.reset();
    }
  }
private:
  std::shared_ptr<phNxpUciHal_RxHandler> handler_;
};

#endif /* _PHNXPUCIHAL_H_ */
