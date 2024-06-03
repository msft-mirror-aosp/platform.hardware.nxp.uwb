/*
 * Copyright 2021-2023 NXP
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

#ifndef _PHNXPUCIHAL_FW_H
#define _PHNXPUCIHAL_FW_H
#include <stdint.h>

#define PHHDLL_LEN_LRC (1U)
#define PHHDLL_MAX_MISO_DATA_LEN (256U)
#define PHHDLL_MAX_LEN_PAYLOAD_MISO (PHHDLL_MAX_MISO_DATA_LEN + PHHDLL_LEN_LRC)

#define FILEPATH_MAXLEN 500
#define FILENAME_MAXLEN 260

#define HCP_MSG_HEADER_LEN 2
#define HDLL_HEADER_LEN 2
#define HDLL_FOOTER_LEN 2
#define HDLL_CRC_LEN 2
#define HDLL_PKT_CHUNK_BITMASK 0x2000
#define HDLL_PKT_LEN_BITMASK 0x1FFF
#define HCP_GROUP_LEN 6 // bits

#define HDLL_CHUNK_OFFSET 0
#define HDLL_LEN_OFFSET 1
#define HDLL_TYPE_OFFSET 2
#define HDLL_GROUP_OFFSET 2
#define HDLL_OPERATION_OFFSET 3
#define HDLL_PAYLOAD_OFFSET 4

#define HDLL_RSP_STATUS_OFFSET 4
#define HDLL_RSP_PAYLOAD_OFFSET 5

#define HDLL_RSP_GROUP_BIT_MASK 0x3F
#define HDLL_MIN_RSP_LEN 8
#define MW_MAJOR_FW_VER_OFFSET 4
#define MW_MINOR_FW_VER_OFFSET 5

#define HDLL_READ_BUFF_SIZE 64
#define HDLL_READ_OP_TIMEOUT 2000 /* 2 seconds timeout */

#define BIN_FILE_BASED_FW_DOWNLOAD 0x00
#define SO_FILE_BASED_FW_DOWNLOAD  0x01
#define SESSION_CONTROL_OPEN 0x55

/* Struct to frame HDLL command */
typedef struct phHDLLCmd {
  uint8_t group;
  uint8_t operation;
  uint8_t chunk_size;
  uint16_t payload_len;
  uint16_t frame_size;
  uint8_t *payload;
} phHDLLCmd_t;

/* Struct to process HDLL response */
typedef struct phHDLLCmdRsp {
  uint8_t *rsp_buf;
  uint8_t rsp_buf_len;
  uint8_t group;
  uint8_t operation;
  uint8_t status;
  uint8_t type;
} phHDLLCmdRsp_t;

/* HCP Operation Group */
typedef enum {
  HCP_OPERATION_GROUP_PROTOCOL = 0x01,
  HCP_OPERATION_GROUP_GENERIC,
  HCP_OPERATION_GROUP_EDL
} eHCP_OPERATION_GROUP_t;

/* operation codes under protocol group */
typedef enum {
  PROTOCOL_GROUP_OP_CODE_HDLL = 0x01,
  PROTOCOL_GROUP_OP_CODE_HCP,
  PROTOCOL_GROUP_OP_CODE_EDL
} ePROTOCOL_GROUP_OP_CODE_t;

/* operation codes under generic group */
typedef enum {
  GENERIC_GROUP_OP_CODE_RESET = 0x01,
  GENERIC_GROUP_OP_CODE_GETINFO
} eGENERIC_GROUP_OP_CODE_t;

/* operation code under EDL group */
typedef enum {
  EDL_DOWNLOAD_CERTIFICATE = 0x01,
  EDL_DOWNLOAD_FLASH_WRITE_FIRST = 0x02,
  EDL_DOWNLOAD_FLASH_WRITE = 0x03,
  EDL_DOWNLOAD_FLASH_WRITE_LAST = 0x04,
  EDL_DOWNLOAD_SRAM_WRITE_FIRST = 0x05,
  EDL_DOWNLOAD_SRAM_WRITE = 0x06,
  EDL_DOWNLOAD_SRAM_WRITE_LAST = 0x07,
  EDL_LIFECYCLE_CERTIFICATE = 0x11,
  EDL_LIFECYCLE_WRITE_FIRST = 0x12,
  EDL_LIFECYCLE_WRITE_LAST = 0x13,
  EDL_PATCH_SRAM_WRITE = 0x21,
  EDL_PATCH_SRAM_WRITE_LAST = 0x22,
  EDL_PATCH_FLASH_WRITE = 0x23
} eEDL_GROUP_OP_CODE_t;

/* UWB Device ROM Version */
typedef enum {
  VER_A1V1 = 0x02,
  VER_A1V2 = 0x03,
} eUWBD_Rom_Version_t;

/* UWB AT page status */
typedef enum {
  STATUS_PAGE_OK = 0x55,
  STATUS_RECOVERED_N_1 = 0x5A,
  STATUS_RECOVERED_N_2 = 0xA5,
  STATUS_PAGE_ERROR = 0xAA,
} eUWBD_AT_Page_status_t;

/* UWB Device lifecycle mode */
typedef enum {
  UNKNOWN = 0xCCCCCCCC,
  DEGRADED_MODE = 0x5C5C5C5C,
  FLASH_TEST_MODE = 0xAAAAAAAA,
  DEVELOPMENT_MODE = 0xC5C5C5C5,
  CUSTOMER_MODE = 0xA5A5A5A5,
  PROTECTED_MODE = 0x55555555,
  NXP_RMA_MODE = 0x5A5A5A5A,
} eUWBD_LC_mode_t;

/* Struct to store the getinfo response */
typedef struct phHDLLGetInfo {
  uint8_t boot_status;
  uint8_t session_control;
  uint8_t session_type;
  eUWBD_Rom_Version_t rom_version;
  eUWBD_AT_Page_status_t AT_page_status;
  uint8_t chip_major_ver;
  uint8_t chip_minor_ver;
  uint8_t fw_minor_ver;
  uint8_t fw_major_ver;
  uint8_t chip_variant[4];
  eUWBD_LC_mode_t device_life_cycle;
  uint8_t chip_id[16];
  uint8_t chip_id_crc[4];
} phHDLLGetInfo_t;

/* HCP type */
typedef enum {
  HCP_TYPE_COMMAND = 0x00,
  HCP_TYPE_RESPONSE,
  HCP_TYPE_NOTIFICATION
} eHCP_TYPE_t;

/* Application status codes */
typedef enum {
  /* Success */
  GENERIC_SUCCESS = 0x00,
  ACKNOWLEDGE = 0x01,
  READY = 0x02,

  /* Generic errors */
  GENERIC_ERROR = 0x80,
  MEMORY_ERROR = 0x81,
  TIMEOUT_ERROR = 0x82,
  CRC_ERROR = 0x83,
  INVALID_ERROR = 0x84,

  /* Verification errors */
  INVALID_LENGTH_ERROR = 0x90,
  INVALID_ADDRESS_ERROR = 0x91,
  ECC_SIGNATURE_ERROR = 0x92,
  SHA384_HASH_ERROR = 0x93,
  LIFECYCLE_VALIDITY_ERROR = 0x94,
  CHIP_ID_ERROR = 0x95,
  CHIP_VERSION_ERROR = 0x96,
  CERTIFICATE_VERSION_ERROR = 0x97,
  FIRMWARE_VERSION_ERROR = 0x98,
  SRAM_DOWNLOAD_ALLOW_ERROR = 0x99,

  /* Encryption errors */
  KEY_DERIVATION_ERROR = 0xA0,
  ENCRYPTED_PAYLOAD_DECRYPTION_ERROR = 0xA1,
  INVALID_ENCRYPTED_PAYLOAD_ERROR = 0xA2,

  /* N-1 & N-2 errors */
  PROTECTED_CACHE_LOAD_ERROR = 0xB0,
  PROTECTED_CACHE_DEPLOY_ERROR = 0xB1,
  LIFECYCLE_UPDATE_ERROR = 0xB2,

  /* Flash errors */
  FLASH_BLANK_PAGE_ERROR = 0xC0,
  FLASH_CHECK_MARGIN_ERROR = 0xC1
} eAPPLICATION_STATUS_CODES_t;

/* FW download status */
typedef enum phFWD_Status {
  FW_DNLD_SUCCESS = 0x00,
  FW_DNLD_FAILURE = 0x01,
  FW_DNLD_REQUIRED = 0x02,
  FW_DNLD_NOT_REQUIRED = 0x03,
  FW_DNLD_FILE_NOT_FOUND = 0x14,
} phFWD_Status_t;

/* FW download flash config status */
typedef enum phFWD_flash_Status {
  FLASH_UPPER_VER_UPDATE = 0x01,
  FLASH_FORCE_UPDATE = 0x02,
  FLASH_DIFFERENT_VER_UPDATE = 0x03,
} phFWD_flash_Status_t;

typedef struct phUwbFWImageContext
{
    /* pointer to the FW image to be used */
    uint8_t *fwImage;
    /* size of fw image */
    uint32_t fwImgSize;
    /* FW FLASH update Options Configurations */
    uint8_t fw_flash_config;
    /* FW Download file Options Configurations */
    uint8_t fw_dnld_config;
    /* FW recovery */
    bool fwRecovery;
    void *gFwLib;
    /* default fw file path */
    char default_fw_path[FILEPATH_MAXLEN];
    /* Device Info */
    phHDLLGetInfo_t *deviceInfo;
} phUwbFWImageContext_t;

/* SR200 device config */
typedef struct phPalSr200_Config {
  void* pDevHandle;
} phPalSr200_Config_t;

/* PWR States */
typedef enum phSetPwrState{
  PWR_DISABLE = 0,
  PWR_ENABLE,
  ABORT_READ_PENDING
} phSetPwrState_t;

phPalSr200_Config_t tPalConfig;

phFWD_Status_t phHdll_GetApdu(uint8_t *pApdu, uint16_t sz,
                              uint16_t *rsp_buf_len);
phFWD_Status_t phHdll_PutApdu(uint8_t *pApdu, uint16_t sz);

#endif /* _PHNXPUCIHAL_FW_H */
