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

#include <sys/ioctl.h>
#include <dlfcn.h>

#include "fwd_hdll.h"
#include "phNxpConfig.h"
#include "phNxpLog.h"
#include "phNxpUciHal_fwd_utils.h"
#include "phNxpUciHal_utils.h"
#include "phTmlUwb_spi.h"

#define MAX_FRAME_LEN 4200
static uint8_t is_fw_download_log_enabled = 0x00;

static phFWD_Status_t openFwBinFile(phUwbFWImageContext_t *pfwImageCtx);
static phFWD_Status_t openFwSoFile(phUwbFWImageContext_t *pfwImageCtx);
static phFWD_Status_t phNxpUciHal_fw_recovery(phUwbFWImageContext_t *pfwImageCtx);

char default_fw_path[FILEPATH_MAXLEN] = "/vendor/firmware/uwb/";
const char *default_dev_fw_bin = "libsr200t_fw.bin";
const char *default_dev_fw_so = "libsr200t_fw.so";
const char *default_so_file_extn = ".so";
extern uint32_t timeoutTimerId;
static bool isHdllReadTmeoutExpired = false;
static bool bSkipEdlCheck = false;
static bool glcRotation = false;

phUwbFWImageContext_t fwImageCtx;

/*******************************************************************************
**
** Function    :   phGenericSendAndRecv
**
** Description :   This function sends the HDLL commands to HeliosX chip over
                   SPI using phHdll_PutApdu() and gets the response using
                   phHdll_GetApdu().
**
** Parameters  :   payload     - HDLL command to be sent
                   len         - HDLL command length
                   readbuff    - HDLL command response buffer
                   rsp_buf_len - HDLL command rsponse buffer length
**
** Returns     :   phFWD_Status_t : 0 - success
                                     1 - failure
**
**
*******************************************************************************/
phFWD_Status_t phGenericSendAndRecv(uint8_t *payload, uint16_t len,
                                    uint8_t *read_buff, uint16_t *rsp_buf_len) {
  phFWD_Status_t ret = FW_DNLD_FAILURE;
  if (FW_DNLD_SUCCESS != (ret = phHdll_PutApdu((uint8_t *)&payload[0], len))) {
    return ret;
  }
  if (FW_DNLD_SUCCESS !=
      (ret = phHdll_GetApdu((uint8_t *)&read_buff[0], HDLL_READ_BUFF_SIZE,
                            rsp_buf_len))) {
    return ret;
  }
  return ret;
}

/*******************************************************************************
**
** Function    :   print_getInfoRsp
**
** Description :   This function prints the HDLL GetInfo command's response
**
** Parameters  :   getInfoRsp  - Struct which has the GetInfo response details.
**
** Returns     :   None
**
**
*******************************************************************************/
void print_getInfoRsp(phHDLLGetInfo_t *getInfoRsp) {
  uint8_t i = 0, offset = 0;
  char buff[HDLL_READ_BUFF_SIZE] = {0};
  if (NULL == getInfoRsp) {
    return;
  }
  NXPLOG_FWDNLD_D("=====================GET_INFO =======================\n");
  NXPLOG_FWDNLD_D("Boot Status: 0x%02X\n", getInfoRsp->boot_status);
  NXPLOG_FWDNLD_D("Session Control: 0x%02X\n", getInfoRsp->session_control);
  NXPLOG_FWDNLD_D("Session Type: 0x%02X\n", getInfoRsp->session_type);
  NXPLOG_FWDNLD_D("ROM Version: 0x%02X\n", getInfoRsp->rom_version);
  NXPLOG_FWDNLD_D("AT Page Status: 0x%02X\n", getInfoRsp->AT_page_status);
  NXPLOG_FWDNLD_D("Chip Version: Major.Minor: %02X.%02X\n",
                  getInfoRsp->chip_major_ver, getInfoRsp->chip_minor_ver);
  NXPLOG_FWDNLD_D("FW Version: Major.Minor: %02X.%02X\n",
                  getInfoRsp->fw_major_ver, getInfoRsp->fw_minor_ver);

  for (i = 0; i != 8; i += 2) { // 4bytes
    sprintf(&buff[i], "%02X", getInfoRsp->chip_variant[offset++]);
  }
  buff[i] = '\0';
  NXPLOG_FWDNLD_D("Chip Variant: 0x%s\n", buff);
  NXPLOG_FWDNLD_D("Device Lifecycle: 0x%X\n", getInfoRsp->device_life_cycle);

  for (i = 0, offset = 0; i != 32; i += 2) { // 16bytes
    sprintf(&buff[i], "%02X", getInfoRsp->chip_id[offset++]);
  }
  buff[i] = '\0';
  NXPLOG_FWDNLD_D("Chip ID: 0x%s\n", buff);

  for (i = 0, offset = 0; i != 8; i += 2) { // 4bytes
    sprintf(&buff[i], "%02X", getInfoRsp->chip_id_crc[offset++]);
  }
  buff[i] = '\0';
  NXPLOG_FWDNLD_D("Chip ID CRC:0x%s\n", buff);
  NXPLOG_FWDNLD_D("=====================================================\n");
}

/*******************************************************************************
**
** Function    :   process_getInfo_rsp
**
** Description :   This function processes the HDLL GetInfo command's response
**
** Parameters  :   payload  - Struct in which the processed info will be kept
**
** Returns     :   On failure - NULL
                   On success - Pointer to the phHDLLGetInfo_t struct
**
**
*******************************************************************************/
phHDLLGetInfo_t *process_getInfo_rsp(uint8_t *payload) {
  uint8_t offset = 0;
  phHDLLGetInfo_t *getInfoRsp = NULL;
  uint8_t device_lc_mode[4] = {0};

  getInfoRsp = (phHDLLGetInfo_t *)malloc(sizeof(phHDLLGetInfo_t));
  if (NULL == getInfoRsp) {
    return NULL;
  }
  memset(getInfoRsp, 0, sizeof(phHDLLGetInfo_t));
  getInfoRsp->boot_status = payload[offset++];
  getInfoRsp->session_control = payload[offset++];
  getInfoRsp->session_type = payload[offset++];
  getInfoRsp->rom_version = (eUWBD_Rom_Version_t)payload[offset++];
  getInfoRsp->AT_page_status = (eUWBD_AT_Page_status_t)payload[offset++];
  offset += 2; // padding bytes
  getInfoRsp->chip_major_ver = payload[offset++];
  getInfoRsp->chip_minor_ver = payload[offset++];
  getInfoRsp->fw_major_ver = payload[offset++];
  getInfoRsp->fw_minor_ver = payload[offset++];
  memcpy(getInfoRsp->chip_variant, payload + offset, sizeof(uint8_t) * 4);
  offset += 4;
  memcpy(device_lc_mode, payload + offset, sizeof(uint8_t) * 4);
  getInfoRsp->device_life_cycle = (eUWBD_LC_mode_t)(device_lc_mode[0] | (device_lc_mode[1] << 8) | (device_lc_mode[2] << 16) | (device_lc_mode[3] << 24));
  offset += 4;
  memcpy(getInfoRsp->chip_id, payload + offset, sizeof(uint8_t) * 16);
  offset += 16;
  memcpy(getInfoRsp->chip_id_crc, payload + offset, sizeof(uint8_t) * 4);
  return getInfoRsp;
}

/*******************************************************************************
**
** Function    :   getFwImageCtx
**
** Description :   This function use to get the FW image context
**
** Parameters  :   pfwImageCtx -> pointer to fw image context
**
** Returns     :   On failure - returns FW_DNLD_FAILURE
                              - or FW_DNLD_FILE_NOT_FOUND if FW file not present
                                in the MW.
                   On success - returns FW_DNLD_SUCCESS.
**
**
*******************************************************************************/
phFWD_Status_t getFwImageCtx(phUwbFWImageContext_t *pfwImageCtx) {
  phFWD_Status_t status = FW_DNLD_SUCCESS;
  char *configured_fw_name = NULL;
  const uint16_t fw_file_max_len = FILENAME_MAXLEN;
  const char *pDefaultFwFileName = NULL;
  char* ret = NULL;

  configured_fw_name = (char *)malloc(fw_file_max_len * sizeof(char));
  int maxSrcLen = (FILEPATH_MAXLEN - strlen(pfwImageCtx->default_fw_path)) - 1;
  if (configured_fw_name == NULL) {
    NXPLOG_FWDNLD_E("malloc of configured_fw_name failed ");
    return FW_DNLD_FAILURE;
  }

  /* Default FW download configset to bin file */
  pDefaultFwFileName = default_dev_fw_bin;

  if (!NxpConfig_GetStr(NAME_NXP_UWB_FW_FILENAME, configured_fw_name,
                            fw_file_max_len)) {
    NXPLOG_FWDNLD_D("Invalid Dev Fw  name keeping the default name: %s",
                    pDefaultFwFileName);
    strncat(pfwImageCtx->default_fw_path, pDefaultFwFileName, maxSrcLen);
  } else {
    NXPLOG_FWDNLD_D("configured_fw_name : %s", configured_fw_name);
    strncat(pfwImageCtx->default_fw_path, configured_fw_name, maxSrcLen);
  }

  NXPLOG_FWDNLD_D("fw file path : %s", pfwImageCtx->default_fw_path);
  // Search for so extension in filename
  ret = strstr(configured_fw_name, default_so_file_extn);
  if(ret) {
    pfwImageCtx->fw_dnld_config = SO_FILE_BASED_FW_DOWNLOAD;
    /* Get Fw Context from so file */
    status = openFwSoFile(pfwImageCtx);
  } else {
    /* Get Fw Context from bin file */
    status = openFwBinFile(pfwImageCtx);
  }

  if (configured_fw_name != NULL) {
      free(configured_fw_name);
    }
  memset(pfwImageCtx->default_fw_path, '\0', sizeof(char) * FILEPATH_MAXLEN);
  strcpy(pfwImageCtx->default_fw_path, "/vendor/firmware/uwb/");
  return status;
}

/*******************************************************************************
**
** Function    :   printManifestInfo
**
** Description :   This function is use to get UWB Manifest info
**
** Parameters  :   pfwImageCtx -> pointer to fw image context
**
** Returns     :   On failure - returns FW_DNLD_FAILURE
                              - or FW_DNLD_FILE_NOT_FOUND if FW file not present
                                in the MW.
                   On success - returns FW_DNLD_SUCCESS.
**
**
*******************************************************************************/
void printManifest_info(UWBManifest_t *fwLibManifest) {

  if(fwLibManifest == NULL) {
    return;
  }
  NXPLOG_FWDNLD_D("================= FW Lib Manifest ====================\n");
  NXPLOG_FWDNLD_D("UWB manifest version = %x\n",fwLibManifest->layout_version);
  NXPLOG_FWDNLD_D("UWB manifest creation year = %d\n",fwLibManifest->creation_date_yy);
  NXPLOG_FWDNLD_D("UWB manifest creation month = %d\n",fwLibManifest->creation_date_month);
  NXPLOG_FWDNLD_D("UWB manifest creation day = %d\n",fwLibManifest->creation_date_day);
  NXPLOG_FWDNLD_D("UWB manifest creation hour = %d\n",fwLibManifest->creation_date_hour);
  NXPLOG_FWDNLD_D("UWB manifest creation minutes = %d\n",fwLibManifest->creation_date_minutes);
  NXPLOG_FWDNLD_D("UWB manifest creation seconds = %d\n",fwLibManifest->creation_date_seconds);
  NXPLOG_FWDNLD_D("UWB manifest count  = %d\n",fwLibManifest->countMWCESFW);

  return;

}

/*******************************************************************************
**
** Function    :   openFwSoFile
**
** Description :   This function loads the FW shared library context
                   if the FW file exists otherwise returns failure.
**
** Parameters  :   pfwImageCtx -> pointer to fw image context
**
** Returns     :   On failure - returns FW_DNLD_FAILURE
                              - or FW_DNLD_FILE_NOT_FOUND if FW file not present
                                in the MW.
                   On success - returns FW_DNLD_SUCCESS.
**
**
*******************************************************************************/
static phFWD_Status_t openFwSoFile(phUwbFWImageContext_t *pfwImageCtx) {
  void *flibptr = NULL;
  UWBManifest_t *currentFwLib = NULL;
  pfwImageCtx->gFwLib = NULL;
  phFWD_Status_t status = FW_DNLD_SUCCESS;

  NXPLOG_FWDNLD_D("%s:%d enter", __func__,__LINE__);

  pfwImageCtx->gFwLib = dlopen(pfwImageCtx->default_fw_path, RTLD_LAZY);
  if (pfwImageCtx->gFwLib == NULL) {
    // Apparently, the library could not be opened
    NXPLOG_FWDNLD_E("%s: Error! opening FW file %s\n", __func__,
                    pfwImageCtx->default_fw_path);
    status = FW_DNLD_FILE_NOT_FOUND;
    goto cleanup;
  }
  flibptr = dlsym(pfwImageCtx->gFwLib, "gUWBManifest");
  if (!flibptr) {
    NXPLOG_FWDNLD_E("%s: Could not get function pointer\n", __func__);
    status = FW_DNLD_FAILURE;
    goto cleanup;
  }

  currentFwLib = (UWBManifest_t *)flibptr;
  if (currentFwLib == NULL) {
    NXPLOG_FWDNLD_E("%s:%d UwbManifest is null exiting.....", __func__, __LINE__);
    status = FW_DNLD_FAILURE;
    goto cleanup;
  }

  printManifest_info(currentFwLib);

  // read the FW bytes into buffer
  if (pfwImageCtx->deviceInfo->rom_version == VER_A1V1) {
    if(currentFwLib->mwCESFW[MWCESFW_A1V1_RECOVERY_FW_OFFSET] == NULL || currentFwLib->mwCESFW[MWCESFW_A1V1_FW_OFFSET] == NULL) {
        NXPLOG_FWDNLD_E("%s:%d UwbManifest mwCESFW is null exiting.....", __func__, __LINE__);
        status = FW_DNLD_FAILURE;
        goto cleanup;
    }
    if(pfwImageCtx->deviceInfo->AT_page_status == STATUS_PAGE_ERROR) {
      pfwImageCtx->fwRecovery = true;
      pfwImageCtx->fwImgSize = currentFwLib->mwCESFW[MWCESFW_A1V1_RECOVERY_FW_OFFSET]->lenCESFW;
      pfwImageCtx->fwImage = currentFwLib->mwCESFW[MWCESFW_A1V1_RECOVERY_FW_OFFSET]->pCESFW;
    } else if((pfwImageCtx->deviceInfo->device_life_cycle == CUSTOMER_MODE) && glcRotation == true) {
      if(currentFwLib->mwCESFW[MWCESFW_A1V1_LC_FW_OFFSET] == NULL ) {
        NXPLOG_FWDNLD_E("%s:%d LC FW does not exist.....", __func__, __LINE__);
        status = FW_DNLD_FAILURE;
        goto cleanup;
      } else {
        pfwImageCtx->fwImgSize = currentFwLib->mwCESFW[MWCESFW_A1V1_LC_FW_OFFSET]->lenCESFW;
        pfwImageCtx->fwImage = currentFwLib->mwCESFW[MWCESFW_A1V1_LC_FW_OFFSET]->pCESFW;
      }
    }else {
      pfwImageCtx->fwImgSize = currentFwLib->mwCESFW[MWCESFW_A1V1_FW_OFFSET]->lenCESFW;
      pfwImageCtx->fwImage = currentFwLib->mwCESFW[MWCESFW_A1V1_FW_OFFSET]->pCESFW;
    }
  }
  else if (pfwImageCtx->deviceInfo->rom_version == VER_A1V2) {
    if(currentFwLib->mwCESFW[MWCESFW_A1V2_RECOVERY_FW_OFFSET] == NULL || currentFwLib->mwCESFW[MWCESFW_A1V2_FW_OFFSET] == NULL) {
        NXPLOG_FWDNLD_E("%s:%d UwbManifest mwCESFW is null exiting.....", __func__, __LINE__);
        status = FW_DNLD_FAILURE;
        goto cleanup;
    }
    if(pfwImageCtx->deviceInfo->AT_page_status == STATUS_PAGE_ERROR) {
      pfwImageCtx->fwRecovery = true;
      pfwImageCtx->fwImgSize = currentFwLib->mwCESFW[MWCESFW_A1V2_RECOVERY_FW_OFFSET]->lenCESFW;
      pfwImageCtx->fwImage = currentFwLib->mwCESFW[MWCESFW_A1V2_RECOVERY_FW_OFFSET]->pCESFW;
    } else if((pfwImageCtx->deviceInfo->device_life_cycle == CUSTOMER_MODE) && glcRotation == true) {
      if(currentFwLib->mwCESFW[MWCESFW_A1V2_LC_FW_OFFSET] == NULL ) {
        NXPLOG_FWDNLD_E("%s:%d LC FW does not exist.....", __func__, __LINE__);
        status = FW_DNLD_FAILURE;
        goto cleanup;
      } else {
        pfwImageCtx->fwImgSize = currentFwLib->mwCESFW[MWCESFW_A1V2_LC_FW_OFFSET]->lenCESFW;
        pfwImageCtx->fwImage = currentFwLib->mwCESFW[MWCESFW_A1V2_LC_FW_OFFSET]->pCESFW;
      }
    } else {
      pfwImageCtx->fwImgSize = currentFwLib->mwCESFW[MWCESFW_A1V2_FW_OFFSET]->lenCESFW;
      pfwImageCtx->fwImage = currentFwLib->mwCESFW[MWCESFW_A1V2_FW_OFFSET]->pCESFW;
    }
  }
  if ((!(pfwImageCtx->fwImgSize)) || (NULL == pfwImageCtx->fwImage)) {
    NXPLOG_FWDNLD_E("%s: Error! File %s is empty\n", __func__, pfwImageCtx->default_fw_path);
    status = FW_DNLD_FAILURE;
    goto cleanup;
  }

  NXPLOG_FWDNLD_E("exiting %s fwImgSize %d" , __func__, pfwImageCtx->fwImgSize);

  return status;

cleanup:
  if (pfwImageCtx->gFwLib != NULL) {
        dlclose(pfwImageCtx->gFwLib);
        pfwImageCtx->gFwLib = NULL;
  }
  return status;

}

/*******************************************************************************
**
** Function    :   openFwBinFile
**
** Description :   This function copies the entire Bin FW file content into a buffer
                   if the FW file exists otherwise returns failure.
**
** Parameters  :   pfwImageCtx -> pointer to fw image context
**
** Returns     :   On failure - returns FW_DNLD_FAILURE
                              - or FW_DNLD_FILE_NOT_FOUND if FW file not present
                                in the MW.
                   On success - returns FW_DNLD_SUCCESS.
**
**
*******************************************************************************/
static phFWD_Status_t openFwBinFile(phUwbFWImageContext_t *pfwImageCtx) {
  phFWD_Status_t status = FW_DNLD_SUCCESS;
  long int file_size = 0;
  size_t ret_size = 0;
  FILE *fptr = NULL;

  NXPLOG_FWDNLD_D("%s:%d enter", __func__,__LINE__);

  // open FW binary file
  if ((fptr = fopen(pfwImageCtx->default_fw_path, "rb")) == NULL) {
    NXPLOG_FWDNLD_E("%s: Error! opening FW file %s\n", __func__,
                    pfwImageCtx->default_fw_path);
    status = FW_DNLD_FILE_NOT_FOUND;
    goto exit;
  }

  // find the FW binary file size
  fseek(fptr, 0L, SEEK_END);
  file_size = ftell(fptr);
  if (!file_size || (-1L == file_size)) {
    NXPLOG_FWDNLD_E("%s: Error! File %s is empty\n", __func__, pfwImageCtx->default_fw_path);
    status = FW_DNLD_FAILURE;
    goto exit;
  }
  else {
    pfwImageCtx->fwImgSize = file_size;
  }

  // read the FW bytes into buffer
  pfwImageCtx->fwImage = (uint8_t *)malloc(sizeof(uint8_t) * pfwImageCtx->fwImgSize);
  if (NULL == pfwImageCtx->fwImage)
  {
    status = FW_DNLD_FAILURE;
    NXPLOG_FWDNLD_E("%s: Error in allocating memory\n", __func__);
    goto exit;
  }
  rewind(fptr);
  ret_size = fread(pfwImageCtx->fwImage, sizeof(uint8_t), pfwImageCtx->fwImgSize, fptr);
  if (ret_size != pfwImageCtx->fwImgSize) {
    if (feof(fptr))
    {
      NXPLOG_FWDNLD_E("%s: Error reading file %s, unexpected end of file\n",
                      __func__, pfwImageCtx->default_fw_path);
    }
    else if (ferror(fptr))
    {
      NXPLOG_FWDNLD_E("%s: Error reading file %s\n", __func__, pfwImageCtx->default_fw_path);
    }
    status = FW_DNLD_FAILURE;
    goto exit;
  }

exit:
  if (NULL != fptr)
  {
    fclose(fptr);
  }

  return status;
}

/*******************************************************************************
**
** Function    :   check_fw_update_required
**
** Description :   This function checks whether FW update is required or not
                   based on FW version from MW binary and FW version present in
                   the HeliosX chip.
**
** Parameters  :   getInfoRsp  - Struct which has the GetInfo response details.
**
** Returns     :   FW_DNLD_FAILURE - If any un expected failure
                   FW_DNLD_NOT_REQUIRED - FW update not required
                   FW_DNLD_REQUIRED - FW update required
                   FW_DNLD_FILE_NOT_FOUND - if the FW bin file is unable to
                                                open or not present
**
**
*******************************************************************************/
phFWD_Status_t check_fw_update_required(phHDLLGetInfo_t *getInfoRsp) {
  uint32_t next_frame_first_byte_index = 0;
  uint32_t index = 0;
  uint8_t mw_fw_major_ver = 0;
  uint8_t mw_fw_minor_ver = 0;
  uint32_t frame_payload_length = 0;
  uint32_t frame_length = 0;
  unsigned long num = 0;
  phFWD_Status_t status = FW_DNLD_FAILURE;

  fwImageCtx.deviceInfo = getInfoRsp;
  fwImageCtx.fw_dnld_config = BIN_FILE_BASED_FW_DOWNLOAD;
  fwImageCtx.fw_flash_config = FLASH_UPPER_VER_UPDATE;
  fwImageCtx.fwRecovery = false;
  strcpy(fwImageCtx.default_fw_path, default_fw_path);

  status = getFwImageCtx(&fwImageCtx);
  if (status != FW_DNLD_SUCCESS) {
    return status;
  }

  if (NxpConfig_GetNum(NAME_NXP_UWB_FLASH_CONFIG, &num, sizeof(num))) {
    fwImageCtx.fw_flash_config = (uint8_t)num;
    NXPLOG_FWDNLD_D("NAME_NXP_UWB_FLASH_CONFIG: 0x%02x\n", fwImageCtx.fw_flash_config);
    if (!(fwImageCtx.fw_flash_config == FLASH_UPPER_VER_UPDATE ||
          fwImageCtx.fw_flash_config == FLASH_DIFFERENT_VER_UPDATE ||
          fwImageCtx.fw_flash_config == FLASH_FORCE_UPDATE))
    {
      fwImageCtx.fw_flash_config = FLASH_UPPER_VER_UPDATE;
    }
  }
  else {
    NXPLOG_FWDNLD_D("NAME_NXP_UWB_FLASH_CONFIG: failed 0x%02x\n",
                    fwImageCtx.fw_flash_config);
  }

  frame_payload_length = (fwImageCtx.fwImage[next_frame_first_byte_index] << 8) +
                         (fwImageCtx.fwImage[next_frame_first_byte_index + 1]);
  frame_length = frame_payload_length + HDLL_HEADER_LEN + HDLL_FOOTER_LEN;

  // get the index of first_write_cmd_payload
  next_frame_first_byte_index = next_frame_first_byte_index + frame_length;
  index = next_frame_first_byte_index;
  mw_fw_major_ver = fwImageCtx.fwImage[index + MW_MAJOR_FW_VER_OFFSET];
  mw_fw_minor_ver = fwImageCtx.fwImage[index + MW_MINOR_FW_VER_OFFSET];
  NXPLOG_FWDNLD_D("mw_fw_ver: %02X.%02X chip_fw_ver: %02X.%02X\n",
                  mw_fw_major_ver, mw_fw_minor_ver, getInfoRsp->fw_major_ver,
                  getInfoRsp->fw_minor_ver);

  if(getInfoRsp->session_control == SESSION_CONTROL_OPEN){
    NXPLOG_FWDNLD_D("FW Update required as session control is open \n");
    status = FW_DNLD_REQUIRED;
  } else {
    switch (fwImageCtx.fw_flash_config) {
    case FLASH_UPPER_VER_UPDATE: {
      if (mw_fw_major_ver > getInfoRsp->fw_major_ver) {
        NXPLOG_FWDNLD_D("FLASH_UPPER_VER_UPDATE:FW Update required\n");
        status = FW_DNLD_REQUIRED;
      } else if (mw_fw_major_ver == getInfoRsp->fw_major_ver) {
        if (mw_fw_minor_ver > getInfoRsp->fw_minor_ver) {
          NXPLOG_FWDNLD_D("FLASH_UPPER_VER_UPDATE:FW Update required\n");
          status = FW_DNLD_REQUIRED;
        } else {
          NXPLOG_FWDNLD_E(
              "FLASH_UPPER_VER_UPDATE:FW lower Minor version is not supported\n");
          status = FW_DNLD_NOT_REQUIRED;
        }
      } else {
        NXPLOG_FWDNLD_E(
            "FLASH_UPPER_VER_UPDATE:FW lower Major version is not supported\n");
        status = FW_DNLD_NOT_REQUIRED;
      }
    } break;
    case FLASH_FORCE_UPDATE: {
      if (mw_fw_major_ver < getInfoRsp->fw_major_ver) {
        NXPLOG_FWDNLD_E(
            "FLASH_FORCE_UPDATE:FW lower Major version is not supported\n");
        status = FW_DNLD_NOT_REQUIRED;
      } else {
        NXPLOG_FWDNLD_D("FLASH_FORCE_UPDATE:FW Update required\n");
        status = FW_DNLD_REQUIRED;
      }
    } break;
    case FLASH_DIFFERENT_VER_UPDATE: {
      if (mw_fw_major_ver > getInfoRsp->fw_major_ver) {
        NXPLOG_FWDNLD_D("FLASH_DIFFERENT_VER_UPDATE:FW Update required\n");
        status = FW_DNLD_REQUIRED;
      } else if(mw_fw_major_ver == getInfoRsp->fw_major_ver) {
        if(mw_fw_minor_ver == getInfoRsp->fw_minor_ver) {
          NXPLOG_FWDNLD_E(
            "FLASH_DIFFERENT_VER_UPDATE:Same Minor FW version update is not supported\n");
            status = FW_DNLD_NOT_REQUIRED;
        } else {
          NXPLOG_FWDNLD_E(
            "FLASH_DIFFERENT_VER_UPDATE:FW Update required\n");
            status = FW_DNLD_REQUIRED;
        }
      } else {
        NXPLOG_FWDNLD_D("FLASH_DIFFERENT_VER_UPDATE:lower Major FW version update is not supported\n");
        status = FW_DNLD_NOT_REQUIRED;;
      }
    } break;
    }
  }
  return status;
}

/*******************************************************************************
**
** Function    :   handleGetInfoRsp
**
** Description :   This function handles the GetInfo response that is received
                   from the HeliosX chip.
**
** Parameters  :   hdll_payload  - HDLL response buffer
**
** Returns     :   FW_DNLD_FAILURE - If any un expected failure
                   FW_DNLD_NOT_REQUIRED - FW update not required
                   FW_DNLD_REQUIRED - FW update required
                   FW_DNLD_FILE_NOT_FOUND - if the FW bin file is unable to
                                                open or not present
**
**
*******************************************************************************/
phFWD_Status_t handleGetInfoRsp(uint8_t *hdll_payload) {
  phFWD_Status_t ret = FW_DNLD_FAILURE;
  phHDLLGetInfo_t *getInfoRsp = NULL;

  getInfoRsp = process_getInfo_rsp(hdll_payload);
  if (NULL == getInfoRsp) {
    return ret;
  }
  print_getInfoRsp(getInfoRsp);
  ret = check_fw_update_required(getInfoRsp);

  if (NULL != getInfoRsp) {
    free(getInfoRsp);
  }
  return ret;
}

/*******************************************************************************
**
** Function    :   printHDLLRspStatus
**
** Description :   This function prints the HDLL response status string based on
                   the given status code

** Parameters  :   status  - status code
**
** Returns     :   None
**
**
*******************************************************************************/

void printHDLLRspStatus(uint8_t status) {
  switch (status) {
  case GENERIC_SUCCESS:
    NXPLOG_FWDNLD_D("Received status: GENERIC_SUCCESS");
    break;
  case ACKNOWLEDGE:
    NXPLOG_FWDNLD_D("Received status: ACKNOWLEDGE");
    break;
  case READY:
    NXPLOG_FWDNLD_D("Received status: READY");
    break;
  case GENERIC_ERROR:
    NXPLOG_FWDNLD_D("Received status: GENERIC_ERROR");
    break;
  case MEMORY_ERROR:
    NXPLOG_FWDNLD_D("Received status: MEMORY_ERROR");
    break;
  case TIMEOUT_ERROR:
    NXPLOG_FWDNLD_D("Received status: TIMEOUT_ERROR");
    break;
  case CRC_ERROR:
    NXPLOG_FWDNLD_D("Received status: CRC_ERROR");
    break;
  case INVALID_ERROR:
    NXPLOG_FWDNLD_D("Received status: INVALID_ERROR");
    break;
  case INVALID_LENGTH_ERROR:
    NXPLOG_FWDNLD_D("Received status: INVALID_LENGTH_ERROR");
    break;
  case INVALID_ADDRESS_ERROR:
    NXPLOG_FWDNLD_D("Received status: INVALID_ADDRESS_ERROR");
    break;
  case ECC_SIGNATURE_ERROR:
    NXPLOG_FWDNLD_D("Received status: ECC_SIGNATURE_ERROR");
    break;
  case SHA384_HASH_ERROR:
    NXPLOG_FWDNLD_D("Received status: SHA384_HASH_ERROR");
    break;
  case LIFECYCLE_VALIDITY_ERROR:
    NXPLOG_FWDNLD_D("Received status: LIFECYCLE_VALIDITY_ERROR");
    break;
  case CHIP_ID_ERROR:
    NXPLOG_FWDNLD_D("Received status: CHIP_ID_ERROR");
    break;
  case CHIP_VERSION_ERROR:
    NXPLOG_FWDNLD_D("Received status: CHIP_VERSION_ERROR");
    break;
  case CERTIFICATE_VERSION_ERROR:
    NXPLOG_FWDNLD_D("Received status: CERTIFICATE_VERSION_ERROR");
    break;
  case FIRMWARE_VERSION_ERROR:
    NXPLOG_FWDNLD_D("Received status: FIRMWARE_VERSION_ERROR");
    break;
  case SRAM_DOWNLOAD_ALLOW_ERROR:
    NXPLOG_FWDNLD_D("Received status: SRAM_DOWNLOAD_ALLOW_ERROR");
    break;
  case KEY_DERIVATION_ERROR:
    NXPLOG_FWDNLD_D("Received status: KEY_DERIVATION_ERROR");
    break;
  case ENCRYPTED_PAYLOAD_DECRYPTION_ERROR:
    NXPLOG_FWDNLD_D("Received status: ENCRYPTED_PAYLOAD_DECRYPTION_ERROR");
    break;
  case INVALID_ENCRYPTED_PAYLOAD_ERROR:
    NXPLOG_FWDNLD_D("Received status: INVALID_ENCRYPTED_PAYLOAD_ERROR");
    break;
  case PROTECTED_CACHE_LOAD_ERROR:
    NXPLOG_FWDNLD_D("Received status: PROTECTED_CACHE_LOAD_ERROR");
    break;
  case PROTECTED_CACHE_DEPLOY_ERROR:
    NXPLOG_FWDNLD_D("Received status: PROTECTED_CACHE_DEPLOY_ERROR");
    break;
  case LIFECYCLE_UPDATE_ERROR:
    NXPLOG_FWDNLD_D("Received status: LIFECYCLE_UPDATE_ERROR");
    break;
  case FLASH_BLANK_PAGE_ERROR:
    NXPLOG_FWDNLD_D("Received status: FLASH_BLANK_PAGE_ERROR");
    break;
  case FLASH_CHECK_MARGIN_ERROR:
    NXPLOG_FWDNLD_D("Received status: FLASH_CHECK_MARGIN_ERROR");
    break;
  default:
    break;
  };
}

/*******************************************************************************
**
** Function    :   process_hdll_response
**
** Description :   This function processes the HDLL response

** Parameters  :   hdllCmdRsp  - HDLL command response structure which has the
                                 received response info as well as the expected
                                 response info.
**
** Returns     :   FW_DNLD_FAILURE - If any undesired response received
                   FW_DNLD_SUCCESS - On proper response
**
**
*******************************************************************************/

/*
 * HDLL Response:
 * <-------HDLL Header--->|<------------------HDLL payload--------------------->
 * <-------HDLL (2bytes)->|<-----HCP (2bytes)------->|<-Application--> <--CRC-->
 * <31 30> <29>  <28 -16> |<15 -14><13 - 8> <7 - 0>  |<status><Payload><2 bytes>
 * <--R--><Chunk><length> |< Type ><Group><Operation>|<1 byte>
 *
 */
phFWD_Status_t process_hdll_response(phHDLLCmdRsp_t *hdllCmdRsp) {
  uint8_t hdll_msg_type = 0;
  uint8_t hdll_rsp_status = 0;
  uint16_t hdll_packet_len = 0;
  uint8_t hdll_group = 0;
  uint8_t hdll_operation = 0;
  uint8_t *hdll_payload = NULL;
  uint16_t hdll_payload_len = 0;
  phFWD_Status_t ret = FW_DNLD_FAILURE;

  if (hdllCmdRsp == NULL || hdllCmdRsp->rsp_buf == NULL) {
    NXPLOG_FWDNLD_E("%s HDLL response buffer is NULL\n", __func__);
    return ret;
  }
  if (hdllCmdRsp->rsp_buf_len < HDLL_MIN_RSP_LEN) {
    NXPLOG_FWDNLD_E(
        "%s Error! HDLL response buffer length is %d, expected min %d bytes\n",
        __func__, hdllCmdRsp->rsp_buf_len, HDLL_MIN_RSP_LEN);
    return ret;
  }

  // parse hdll frame
  hdll_packet_len = (uint16_t)(hdllCmdRsp->rsp_buf[0] << 8) |
                    (hdllCmdRsp->rsp_buf[HDLL_LEN_OFFSET]);
  hdll_packet_len &= HDLL_PKT_LEN_BITMASK;
  NXPLOG_FWDNLD_D("Received RSP packet len      :0x%04X\n", hdll_packet_len);
  if (hdll_packet_len == 0) {
    NXPLOG_FWDNLD_D("Error in hdll response.. hdll_packet_len = 0\n");
    return ret;
  }

  hdll_msg_type = hdllCmdRsp->rsp_buf[HDLL_TYPE_OFFSET] >> HCP_GROUP_LEN;
  hdll_group =
      (hdllCmdRsp->rsp_buf[HDLL_GROUP_OFFSET] & HDLL_RSP_GROUP_BIT_MASK);
  hdll_operation = hdllCmdRsp->rsp_buf[HDLL_OPERATION_OFFSET];
  hdll_rsp_status = hdllCmdRsp->rsp_buf[HDLL_RSP_STATUS_OFFSET];

  NXPLOG_FWDNLD_D("Received RSP msg type        :0x%02X\n", hdll_msg_type);
  NXPLOG_FWDNLD_D("Received RSP group operation :0x%02X%02X\n", hdll_group,
                  hdll_operation);
  NXPLOG_FWDNLD_D("Received RSP status code     :0x%02X\n", hdll_rsp_status);
  printHDLLRspStatus(hdll_rsp_status);

  hdll_payload_len = hdllCmdRsp->rsp_buf_len - (HDLL_RSP_PAYLOAD_OFFSET + HDLL_CRC_LEN);
  NXPLOG_FWDNLD_D("hdll payload len = 0x%02x" , hdll_payload_len);

  if (hdll_payload_len > 0) {
    hdll_payload = (uint8_t *)malloc(
        sizeof(uint8_t) *
        (hdll_payload_len));
    if (NULL == hdll_payload) {
      return ret;
    }
    memcpy(hdll_payload, &hdllCmdRsp->rsp_buf[HDLL_RSP_PAYLOAD_OFFSET],
           hdll_payload_len);
  }

  // validate the response
  if (hdllCmdRsp->status != hdll_rsp_status) {
    NXPLOG_FWDNLD_D("Error! expected response status code is 0x%02X  but "
                    "received 0x%02X\n",
                    hdllCmdRsp->status, hdll_rsp_status);
    ret = FW_DNLD_FAILURE;
  } else if (hdllCmdRsp->type != hdll_msg_type) {
    NXPLOG_FWDNLD_D(
        "Error! expected HDLL type code is 0x%02X but received 0x%02X\n",
        hdllCmdRsp->type, hdll_msg_type);
    ret = FW_DNLD_FAILURE;
  } else if ((hdllCmdRsp->group != hdll_group) ||
           (hdllCmdRsp->operation != hdll_operation)) {
    NXPLOG_FWDNLD_D("Error! expected response operation code is 0x%02X%02X but "
                    "received 0x%02X%02X \n",
                    hdllCmdRsp->group, hdllCmdRsp->operation, hdll_group,
                    hdll_operation);
    ret = FW_DNLD_FAILURE;
  } else
  {
    ret = FW_DNLD_SUCCESS;
  }

  if (ret == FW_DNLD_FAILURE){
    goto exit;
  }

  // Handle the response according to the operation
  switch (hdll_group) {
  case HCP_OPERATION_GROUP_PROTOCOL: {
    switch (hdll_operation) {
    case PROTOCOL_GROUP_OP_CODE_HDLL: {
      NXPLOG_FWDNLD_D("Received PROTOCOL_GROUP_HDLL_OP_CODE\n");
    } break;
    case PROTOCOL_GROUP_OP_CODE_HCP: {
      NXPLOG_FWDNLD_D("Received PROTOCOL_GROUP_HCP_OP_CODE\n");
    } break;
    case PROTOCOL_GROUP_OP_CODE_EDL: {
      NXPLOG_FWDNLD_D("Received PROTOCOL_GROUP_EDL_OP_CODE\n");
    } break;
    }
  } break;

  case HCP_OPERATION_GROUP_GENERIC: {
    switch (hdll_operation) {
    case GENERIC_GROUP_OP_CODE_RESET: {
      NXPLOG_FWDNLD_D("Received OP_GENERIC_RESET\n");
      // Generic reset cmd will have the rsp only in case of error.
      // How to handle the situation.
    } break;
    case GENERIC_GROUP_OP_CODE_GETINFO: {
      NXPLOG_FWDNLD_D("Received OP_GENERIC_GET_INFO\n");
      if (hdll_payload != NULL) {
        ret = handleGetInfoRsp(hdll_payload);
      }
    } break;
    }
  } break;

  case HCP_OPERATION_GROUP_EDL: {
    switch (hdll_operation) {
    case EDL_DOWNLOAD_CERTIFICATE: {
      NXPLOG_FWDNLD_D("Received OP_EDL_DOWNLOAD_CERTIFICATE\n");
    } break;
    case EDL_DOWNLOAD_FLASH_WRITE_FIRST: {
      NXPLOG_FWDNLD_D("Received OP_EDL_DOWNLOAD_FLASH_WRITE_FIRST\n");
    }
    break;
    case EDL_DOWNLOAD_FLASH_WRITE: {
      NXPLOG_FWDNLD_D("Received OP_EDL_DOWNLOAD_FLASH_WRITE\n");
    } break;
    case EDL_DOWNLOAD_FLASH_WRITE_LAST: {
      NXPLOG_FWDNLD_D("Received OP_EDL_DOWNLOAD_FLASH_WRITE_LAST\n");
    } break;
    case EDL_DOWNLOAD_SRAM_WRITE_FIRST: {
      NXPLOG_FWDNLD_D("Received OP_EDL_DOWNLOAD_SRAM_WRITE_FIRST\n");
    } break;
    case EDL_DOWNLOAD_SRAM_WRITE: {
      NXPLOG_FWDNLD_D("Received OP_EDL_DOWNLOAD_SRAM_WRITE\n");
    } break;
    case EDL_DOWNLOAD_SRAM_WRITE_LAST: {
      NXPLOG_FWDNLD_D("Received OP_EDL_DOWNLOAD_SRAM_WRITE_LAST\n");
    } break;
    case EDL_LIFECYCLE_CERTIFICATE: {
      NXPLOG_FWDNLD_D("Received OP_EDL_LIFECYCLE_CERTIFICATE\n");
    } break;
    case EDL_LIFECYCLE_WRITE_FIRST: {
      NXPLOG_FWDNLD_D("Received OP_EDL_LIFECYCLE_WRITE_FIRST\n");
    } break;
    case EDL_LIFECYCLE_WRITE_LAST: {
      NXPLOG_FWDNLD_D("Received OP_EDL_LIFECYCLE_WRITE_LAST\n");
    } break;
    case EDL_PATCH_SRAM_WRITE: {
      NXPLOG_FWDNLD_D("Received OP_EDL_PATCH_SRAM_WRITE\n");
    } break;
    case EDL_PATCH_SRAM_WRITE_LAST: {
      NXPLOG_FWDNLD_D("Received OP_EDL_PATCH_SRAM_WRITE_LAST\n");
    } break;
    case EDL_PATCH_FLASH_WRITE: {
      NXPLOG_FWDNLD_D("Received OP_EDL_PATCH_FLASH_WRITE\n");
    } break;
    }
  } break;
  default:
    break;
  }

exit:
  if (hdll_payload != NULL) {
    free(hdll_payload);
  }
  return ret;
}

/*******************************************************************************
**
** Function    :   sendEdlDownloadCertificateCmd
**
** Description :   This function frames the EdlDownloadCertificateCmd which
                   needs to be sent as part of FW download sequence.
**
** Parameters  :   payload  - HDLL command buffer
                   len - command buffer length
                   rsp_buf - response buffer that will be received from the
                   HeliosX chip.
**
** Returns     :   FW_DNLD_FAILURE - If any undesired response received
                   FW_DNLD_SUCCESS - On proper response
**
**
*******************************************************************************/
phFWD_Status_t sendEdlDownloadCertificateCmd(uint8_t *payload, uint16_t len,
                                             uint8_t *rsp_buf) {

  uint16_t rsp_buf_len = 0x0;
  phFWD_Status_t ret = FW_DNLD_SUCCESS;
  phHDLLCmdRsp_t *hdllCmdRsp = NULL;

  ret = phGenericSendAndRecv(payload, len, rsp_buf, &rsp_buf_len);
  if (!rsp_buf_len || ret == FW_DNLD_FAILURE) {
    NXPLOG_FWDNLD_D("Error in sending/receiving OP_EDL_DOWNLOAD_CERTIFICATE "
                    "cmd/response\n");
    return ret;
  }

  hdllCmdRsp = (phHDLLCmdRsp_t *)malloc(sizeof(phHDLLCmdRsp_t));
  if (NULL == hdllCmdRsp) {
    return ret;
  }

  hdllCmdRsp->group = HCP_OPERATION_GROUP_EDL;
  hdllCmdRsp->operation = EDL_DOWNLOAD_CERTIFICATE;
  hdllCmdRsp->rsp_buf = rsp_buf;
  hdllCmdRsp->rsp_buf_len = rsp_buf_len;
  hdllCmdRsp->status = GENERIC_SUCCESS;
  hdllCmdRsp->type = HCP_TYPE_RESPONSE;
  ret = process_hdll_response(hdllCmdRsp);

  if (NULL != hdllCmdRsp) {
    free(hdllCmdRsp);
  }

  return ret;
}

/*******************************************************************************
**
** Function    :   sendEdlFlashWriteFirstCmd
**
** Description :   This function frames the EdlFlashWriteFirstCmd which
                   needs to be sent as part of FW download sequence.
**
** Parameters  :   payload  - HDLL command buffer
                   len - command buffer length
                   rsp_buf - response buffer that will be received from the
                   HeliosX chip.
**
** Returns     :   FW_DNLD_FAILURE - If any undesired response received
                   FW_DNLD_SUCCESS - On proper response
**
**
*******************************************************************************/
phFWD_Status_t sendEdlFlashWriteFirstCmd(uint8_t *payload, uint16_t len,
                                         uint8_t *rsp_buf) {
  uint16_t rsp_buf_len = 0x0;
  phFWD_Status_t ret = FW_DNLD_SUCCESS;
  phHDLLCmdRsp_t *hdllCmdRsp = NULL;

  ret = phGenericSendAndRecv(payload, len, rsp_buf, &rsp_buf_len);
  if (!rsp_buf_len || ret == FW_DNLD_FAILURE) {
    NXPLOG_FWDNLD_D("Error in sending/receiving "
                    "OP_EDL_DOWNLOAD_FLASH_WRITE_FIRST cmd/response\n");
    return ret;
  }

  hdllCmdRsp = (phHDLLCmdRsp_t *)malloc(sizeof(phHDLLCmdRsp_t));
  if (NULL == hdllCmdRsp) {
    return ret;
  }

  hdllCmdRsp->group = HCP_OPERATION_GROUP_EDL;
  hdllCmdRsp->operation = EDL_DOWNLOAD_FLASH_WRITE_FIRST;
  hdllCmdRsp->rsp_buf = rsp_buf;
  hdllCmdRsp->rsp_buf_len = rsp_buf_len;
  hdllCmdRsp->status = GENERIC_SUCCESS;
  hdllCmdRsp->type = HCP_TYPE_RESPONSE;
  ret = process_hdll_response(hdllCmdRsp);

  if (NULL != hdllCmdRsp) {
    free(hdllCmdRsp);
  }

  return ret;
}

/*******************************************************************************
**
** Function    :   sendEdlFlashWriteCmd
**
** Description :   This function frames the sendEdlFlashWriteCmd which
                   will have the actual FW chunk.
**
** Parameters  :   payload  - HDLL command buffer
                   len - command buffer length
                   rsp_buf - response buffer that will be received from the
                   HeliosX chip.
**
** Returns     :   FW_DNLD_FAILURE - If any undesired response received
                   FW_DNLD_SUCCESS - On proper response
**
**
*******************************************************************************/
phFWD_Status_t sendEdlFlashWriteCmd(uint8_t *payload, uint16_t len,
                                    uint8_t *rsp_buf) {
  uint16_t rsp_buf_len = 0x0;
  phFWD_Status_t ret = FW_DNLD_SUCCESS;
  phHDLLCmdRsp_t *hdllCmdRsp = NULL;

  ret = phGenericSendAndRecv(payload, len, rsp_buf, &rsp_buf_len);
  if (!rsp_buf_len || ret == FW_DNLD_FAILURE) {
    NXPLOG_FWDNLD_D("Error in sending/receiving OP_EDL_DOWNLOAD_FLASH_WRITE "
                    "cmd/response\n");
    return ret;
  }

  hdllCmdRsp = (phHDLLCmdRsp_t *)malloc(sizeof(phHDLLCmdRsp_t));
  if (NULL == hdllCmdRsp) {
    return ret;
  }

  hdllCmdRsp->group = HCP_OPERATION_GROUP_EDL;
  hdllCmdRsp->operation = EDL_DOWNLOAD_FLASH_WRITE;
  hdllCmdRsp->rsp_buf = rsp_buf;
  hdllCmdRsp->rsp_buf_len = rsp_buf_len;
  hdllCmdRsp->status = GENERIC_SUCCESS;
  hdllCmdRsp->type = HCP_TYPE_RESPONSE;
  ret = process_hdll_response(hdllCmdRsp);

  if (NULL != hdllCmdRsp) {
    free(hdllCmdRsp);
  }

  return ret;
}

/*******************************************************************************
**
** Function    :   sendEdlFlashWriteLastCmd
**
** Description :   This function frames the EdlFlashWriteLastCmd which
                   needs to be sent as part of FW download sequence.
**
** Parameters  :   payload  - HDLL command buffer
                   len - command buffer length
                   rsp_buf - response buffer that will be received from the
                   HeliosX chip.
**
** Returns     :   FW_DNLD_FAILURE - If any undesired response received
                   FW_DNLD_SUCCESS - On proper response
**
**
*******************************************************************************/
phFWD_Status_t sendEdlFlashWriteLastCmd(uint8_t *payload, uint16_t len,
                                        uint8_t *rsp_buf) {
  uint16_t rsp_buf_len = 0x0;
  phFWD_Status_t ret = FW_DNLD_SUCCESS;
  phHDLLCmdRsp_t *hdllCmdRsp = NULL;

  ret = phGenericSendAndRecv(payload, len, rsp_buf, &rsp_buf_len);
  if (!rsp_buf_len || ret == FW_DNLD_FAILURE) {
    NXPLOG_FWDNLD_D("Error in sending/receiving "
                    "OP_EDL_DOWNLOAD_FLASH_WRITE_LAST cmd/response\n");
    return ret;
  }

  hdllCmdRsp = (phHDLLCmdRsp_t *)malloc(sizeof(phHDLLCmdRsp_t));
  if (NULL == hdllCmdRsp) {
    return ret;
  }

  hdllCmdRsp->group = HCP_OPERATION_GROUP_EDL;
  hdllCmdRsp->operation = EDL_DOWNLOAD_FLASH_WRITE_LAST;
  hdllCmdRsp->rsp_buf = rsp_buf;
  hdllCmdRsp->rsp_buf_len = rsp_buf_len;
  hdllCmdRsp->status = GENERIC_SUCCESS;
  hdllCmdRsp->type = HCP_TYPE_RESPONSE;
  ret = process_hdll_response(hdllCmdRsp);

  if (NULL != hdllCmdRsp) {
    free(hdllCmdRsp);
  }

  return ret;
}

/*******************************************************************************
**
** Function    :   sendEdlLifecycleCertificateCmd
**
** Description :   This function frames the EdlLifecycleCertificateCmd which
                   needs to be sent as part of Lifecycle update.
**
** Parameters  :   payload  - HDLL command buffer
                   len - command buffer length
                   rsp_buf - response buffer that will be received from the
                   HeliosX chip.
**
** Returns     :   FW_DNLD_FAILURE - If any undesired response received
                   FW_DNLD_SUCCESS - On proper response
**
**
*******************************************************************************/
phFWD_Status_t sendEdlLifecycleCertificateCmd(uint8_t *payload, uint16_t len,
                                              uint8_t *rsp_buf) {
  uint16_t rsp_buf_len = 0x0;
  phFWD_Status_t ret = FW_DNLD_SUCCESS;
  phHDLLCmdRsp_t *hdllCmdRsp = NULL;

  ret = phGenericSendAndRecv(payload, len, rsp_buf, &rsp_buf_len);
  if (!rsp_buf_len || ret == FW_DNLD_FAILURE) {
    NXPLOG_FWDNLD_D("Error in sending/receiving "
                    "EDL_LIFECYCLE_CERTIFICATE cmd/response\n");
    return ret;
  }

  hdllCmdRsp = (phHDLLCmdRsp_t *)malloc(sizeof(phHDLLCmdRsp_t));
  if (NULL == hdllCmdRsp) {
    return ret;
  }

  hdllCmdRsp->group = HCP_OPERATION_GROUP_EDL;
  hdllCmdRsp->operation = EDL_LIFECYCLE_CERTIFICATE;
  hdllCmdRsp->rsp_buf = rsp_buf;
  hdllCmdRsp->rsp_buf_len = rsp_buf_len;
  hdllCmdRsp->status = GENERIC_SUCCESS;
  hdllCmdRsp->type = HCP_TYPE_RESPONSE;
  ret = process_hdll_response(hdllCmdRsp);

  if (NULL != hdllCmdRsp) {
    free(hdllCmdRsp);
  }

  return ret;
}

/*******************************************************************************
**
** Function    :   sendEdlLifecycleWriteFirstCmd
**
** Description :   This function frames the EdlLifecycleWriteFirstCmd which
                   needs to be sent as part of Lifecycle update.
**
** Parameters  :   payload  - HDLL command buffer
                   len - command buffer length
                   rsp_buf - response buffer that will be received from the
                   HeliosX chip.
**
** Returns     :   FW_DNLD_FAILURE - If any undesired response received
                   FW_DNLD_SUCCESS - On proper response
**
**
*******************************************************************************/
phFWD_Status_t sendEdlLifecycleWriteFirstCmd(uint8_t *payload, uint16_t len,
                                             uint8_t *rsp_buf) {
  uint16_t rsp_buf_len = 0x0;
  phFWD_Status_t ret = FW_DNLD_SUCCESS;
  phHDLLCmdRsp_t *hdllCmdRsp = NULL;

  ret = phGenericSendAndRecv(payload, len, rsp_buf, &rsp_buf_len);
  if (!rsp_buf_len || ret == FW_DNLD_FAILURE) {
    NXPLOG_FWDNLD_D("Error in sending/receiving "
                    "EDL_LIFECYCLE_WRITE_FIRST cmd/response\n");
    return ret;
  }

  hdllCmdRsp = (phHDLLCmdRsp_t *)malloc(sizeof(phHDLLCmdRsp_t));
  if (NULL == hdllCmdRsp) {
    return ret;
  }

  hdllCmdRsp->group = HCP_OPERATION_GROUP_EDL;
  hdllCmdRsp->operation = EDL_LIFECYCLE_WRITE_FIRST;
  hdllCmdRsp->rsp_buf = rsp_buf;
  hdllCmdRsp->rsp_buf_len = rsp_buf_len;
  hdllCmdRsp->status = GENERIC_SUCCESS;
  hdllCmdRsp->type = HCP_TYPE_RESPONSE;
  ret = process_hdll_response(hdllCmdRsp);

  if (NULL != hdllCmdRsp) {
    free(hdllCmdRsp);
  }

  return ret;
}

/*******************************************************************************
**
** Function    :   sendEdlLifecycleWriteLastCmd
**
** Description :   This function frames the EdlLifecycleWriteLastCmd which
                   needs to be sent as part of Lifecycle update.
**
** Parameters  :   payload  - HDLL command buffer
                   len - command buffer length
                   rsp_buf - response buffer that will be received from the
                   HeliosX chip.
**
** Returns     :   FW_DNLD_FAILURE - If any undesired response received
                   FW_DNLD_SUCCESS - On proper response
**
**
*******************************************************************************/
phFWD_Status_t sendEdlLifecycleWriteLastCmd(uint8_t *payload, uint16_t len,
                                            uint8_t *rsp_buf) {
  uint16_t rsp_buf_len = 0x0;
  phFWD_Status_t ret = FW_DNLD_SUCCESS;
  phHDLLCmdRsp_t *hdllCmdRsp = NULL;

  ret = phGenericSendAndRecv(payload, len, rsp_buf, &rsp_buf_len);
  if (!rsp_buf_len || ret == FW_DNLD_FAILURE) {
    NXPLOG_FWDNLD_D("Error in sending/receiving "
                    "EDL_LIFECYCLE_WRITE_LAST cmd/response\n");
    return ret;
  }

  hdllCmdRsp = (phHDLLCmdRsp_t *)malloc(sizeof(phHDLLCmdRsp_t));
  if (NULL == hdllCmdRsp) {
    return ret;
  }

  hdllCmdRsp->group = HCP_OPERATION_GROUP_EDL;
  hdllCmdRsp->operation = EDL_LIFECYCLE_WRITE_LAST;
  hdllCmdRsp->rsp_buf = rsp_buf;
  hdllCmdRsp->rsp_buf_len = rsp_buf_len;
  hdllCmdRsp->status = GENERIC_SUCCESS;
  hdllCmdRsp->type = HCP_TYPE_RESPONSE;
  ret = process_hdll_response(hdllCmdRsp);

  if (NULL != hdllCmdRsp) {
    free(hdllCmdRsp);
  }

  return ret;
}

/*******************************************************************************
**
** Function    :   sendEdlPatchFlashWriteCmd
**
** Description :   This function frames the sendEdlPatchlFlashWriteCmd which
                   will send the EDL Patch Flash Write cmd
**
** Parameters  :   payload  - HDLL command buffer
                   len - command buffer length
                   rsp_buf - response buffer that will be received from the
                   HeliosX chip.
**
** Returns     :   FW_DNLD_FAILURE - If any undesired response received
                   FW_DNLD_SUCCESS - On proper response
**
**
*******************************************************************************/
phFWD_Status_t sendEdlPatchFlashWriteCmd(uint8_t *payload, uint16_t len,
                                    uint8_t *rsp_buf) {
  uint16_t rsp_buf_len = 0x0;
  phFWD_Status_t ret = FW_DNLD_SUCCESS;
  phHDLLCmdRsp_t *hdllCmdRsp = NULL;

  ret = phGenericSendAndRecv(payload, len, rsp_buf, &rsp_buf_len);
  if (!rsp_buf_len || ret == FW_DNLD_FAILURE) {
    NXPLOG_FWDNLD_D("Error in sending/receiving OP_EDL_PATCH_FLASH_WRITE "
                    "cmd/response\n");
    return ret;
  }

  hdllCmdRsp = (phHDLLCmdRsp_t *)malloc(sizeof(phHDLLCmdRsp_t));
  if (NULL == hdllCmdRsp) {
    return ret;
  }

  hdllCmdRsp->group = HCP_OPERATION_GROUP_EDL;
  hdllCmdRsp->operation = EDL_PATCH_FLASH_WRITE;
  hdllCmdRsp->rsp_buf = rsp_buf;
  hdllCmdRsp->rsp_buf_len = rsp_buf_len;
  hdllCmdRsp->status = GENERIC_SUCCESS;
  hdllCmdRsp->type = HCP_TYPE_RESPONSE;
  ret = process_hdll_response(hdllCmdRsp);

  if (NULL != hdllCmdRsp) {
    free(hdllCmdRsp);
  }

  return ret;
}

/*******************************************************************************
**
** Function    :   phHal_Host_CalcCrc16
**
** Description :   This function calculates the HDLL command's CRC
**
** Parameters  :   p  - HDLL command buffer
                   dwLength - command buffer length
**
** Returns     :   the calculated CRC value
**
**
*******************************************************************************/
static uint16_t phHal_Host_CalcCrc16(uint8_t *p, uint32_t dwLength) {
  uint32_t i;
  uint16_t crc_new;
  uint16_t crc = 0xffffU;

  for (i = 0; i < dwLength; i++) {
    crc_new = (uint8_t)(crc >> 8) | (crc << 8);
    crc_new ^= p[i];
    crc_new ^= (uint8_t)(crc_new & 0xff) >> 4;
    crc_new ^= crc_new << 12;
    crc_new ^= (crc_new & 0xff) << 5;
    crc = crc_new;
  }
  return crc;
}

/*******************************************************************************
**
** Function    :   phBuildHdllCmd
**
** Description :   This function frames the final HDLL command (HDLL header +
                   HDLL payload + CRC) by framing HDLL payload and HDLL frame
                   using 2 different APIs.
**
** Parameters  :   hdllCmd - HDLL command structure which has the information
                             to build the corresponding HDLL command.
**
** Returns     :   NULL - on failure
                   HDLL command buffer - On success
**
**
*******************************************************************************/

/*
 * HDLL Command:
 * <--------HDLL Header---->|<------------------HDLL payload------------------->
 * <--------HDLL (2bytes)-->|<-----HCP (2bytes)------->|<-Application-><--CRC-->
 * <31 30> <29>    <28 -16> |<15 -14><13 - 8><7 - 0>   |<---Payload---><2 bytes>
 * <--R--> <Chunk> <length> |< Type ><Group><Operation>|
 *
 */

uint8_t *phBuildHdllCmd(phHDLLCmd_t *hdllCmd) {
  uint8_t type = 0;
  uint8_t *hdll_frame = NULL;
  uint16_t hdll_frame_size = 0;
  uint16_t hdll_crc = 0x0;
  uint16_t hdll_header = 0x0;
  NXPLOG_FWDNLD_D("phBuildHdllCmd:\n");

  if (NULL == hdllCmd) {
    return NULL;
  }
  // header len =2 bytes + hdll_payload_len + crc =2 bytes
  hdll_frame_size = HDLL_HEADER_LEN + HCP_MSG_HEADER_LEN +
                    hdllCmd->payload_len + HDLL_CRC_LEN;
  hdll_frame = (uint8_t *)malloc(sizeof(uint8_t) * hdll_frame_size);
  if (NULL == hdll_frame) {
    return hdll_frame;
  }

  // build hdll frame
  hdll_header |= hdllCmd->payload_len + HCP_MSG_HEADER_LEN;
  hdll_header &= HDLL_PKT_LEN_BITMASK;
  hdll_header = hdllCmd->chunk_size ? (HDLL_PKT_CHUNK_BITMASK | hdll_header)
                                    : hdll_header;

  // hdll_header uint16 to uint8
  hdll_frame[HDLL_CHUNK_OFFSET] = (hdll_header >> 8);
  hdll_frame[HDLL_LEN_OFFSET] = (hdll_header & 0xFF);

  type = HCP_TYPE_COMMAND;
  type <<= HCP_GROUP_LEN;
  hdll_frame[HDLL_TYPE_OFFSET] = type | hdllCmd->group;
  hdll_frame[HDLL_OPERATION_OFFSET] = hdllCmd->operation;

  if (hdllCmd->payload_len > 0 && hdllCmd->payload != NULL) {
    // copy hdll payload into hdll frame
    memcpy(&hdll_frame[HDLL_PAYLOAD_OFFSET], hdllCmd->payload,
           hdllCmd->payload_len);
  }

  hdll_crc = phHal_Host_CalcCrc16(hdll_frame, hdll_frame_size - 2);
  hdll_frame[hdll_frame_size - 2] = (hdll_crc >> 8);
  hdll_frame[hdll_frame_size - 1] = (hdll_crc & 0xFF);

  hdllCmd->frame_size = hdll_frame_size;
  return hdll_frame;
}

/*******************************************************************************
**
** Function    :   sendEdlResetCmd
**
** Description :   This function frames the EdlResetCmd and sends to the HeliosX
                   chip
**
** Parameters  :   None
**
** Returns     :   FW_DNLD_FAILURE - If any failure occurs while framing or
                                    sending the command or while receiving the
                                    response
                   FW_DNLD_SUCCESS - On success
**
**
*******************************************************************************/
phFWD_Status_t sendEdlResetCmd() {
  uint8_t rsp_buf[HDLL_READ_BUFF_SIZE] = {0};
  uint8_t *hdll_frame = NULL;
  phFWD_Status_t ret = FW_DNLD_FAILURE;
  uint16_t rsp_buf_len = 0x0;
  phHDLLCmd_t *hdllCmd = NULL;
  phHDLLCmdRsp_t *hdllCmdRsp = NULL;

  hdllCmd = (phHDLLCmd_t *)malloc(sizeof(phHDLLCmd_t));
  if (NULL == hdllCmd) {
    goto exit;
  }

  hdllCmd->group = HCP_OPERATION_GROUP_GENERIC;
  hdllCmd->operation = GENERIC_GROUP_OP_CODE_RESET;
  hdllCmd->chunk_size = 0;
  hdllCmd->frame_size = 0;
  hdllCmd->payload = NULL;
  hdllCmd->payload_len = 0;

  hdll_frame = phBuildHdllCmd(hdllCmd);
  if (NULL == hdll_frame) {
    goto exit;
  }
  NXPLOG_FWDNLD_D("Sending operation: OP_GENERIC_RESET\n");
  ret = phGenericSendAndRecv(hdll_frame, hdllCmd->frame_size, rsp_buf,
                             &rsp_buf_len);
  if (ret == FW_DNLD_FAILURE) {
    // treat is as success as generic reset will have response only if there
    // is an error.
    ret = FW_DNLD_SUCCESS;
  }
  if (rsp_buf_len > 0) {
    hdllCmdRsp = (phHDLLCmdRsp_t *)malloc(sizeof(phHDLLCmdRsp_t));
    if (NULL == hdllCmdRsp) {
      ret = FW_DNLD_FAILURE;
      goto exit;
    }
    hdllCmdRsp->group = HCP_OPERATION_GROUP_GENERIC;
    hdllCmdRsp->operation = GENERIC_GROUP_OP_CODE_RESET;
    hdllCmdRsp->rsp_buf = rsp_buf;
    hdllCmdRsp->rsp_buf_len = rsp_buf_len;
    hdllCmdRsp->status = GENERIC_SUCCESS;
    hdllCmdRsp->type = HCP_TYPE_RESPONSE;
    ret = process_hdll_response(hdllCmdRsp);
  }
exit:
  if (hdll_frame != NULL) {
    free(hdll_frame);
  }
  if (NULL != hdllCmd) {
    free(hdllCmd);
  }
  if (NULL != hdllCmdRsp) {
    free(hdllCmdRsp);
  }
  return ret;
}

/*******************************************************************************
**
** Function    :   phGetEdlReadyNtf
**
** Description :   This function frames the GetEdlReadyNtf command and sends to
                   the HeliosX chip
**
** Parameters  :   None
**
** Returns     :   FW_DNLD_FAILURE - If any failure occurs while framing or
                                    sending the command or while receiving the
                                    response
                   FW_DNLD_SUCCESS - On success
**
**
*******************************************************************************/
phFWD_Status_t phGetEdlReadyNtf() {
  uint8_t rsp_buf[HDLL_READ_BUFF_SIZE] = {0};
  phFWD_Status_t ret = FW_DNLD_FAILURE;
  uint16_t rsp_buf_len = 0x0;
  phHDLLCmdRsp_t *hdllCmdRsp = NULL;

  NXPLOG_FWDNLD_D("Wait for EDL_READY notification\n");
  ret =
      phHdll_GetApdu((uint8_t *)&rsp_buf[0], HDLL_READ_BUFF_SIZE, &rsp_buf_len);

  if (!rsp_buf_len || ret == FW_DNLD_FAILURE) {
    NXPLOG_FWDNLD_D("Error in sending/receiving GET_EDL_READY cmd/response\n");
    return ret;
  }

  hdllCmdRsp = (phHDLLCmdRsp_t *)malloc(sizeof(phHDLLCmdRsp_t));
  if (NULL == hdllCmdRsp) {
    return ret;
  }

  hdllCmdRsp->group = HCP_OPERATION_GROUP_PROTOCOL;
  hdllCmdRsp->operation = PROTOCOL_GROUP_OP_CODE_EDL;
  hdllCmdRsp->rsp_buf = rsp_buf;
  hdllCmdRsp->rsp_buf_len = rsp_buf_len;
  hdllCmdRsp->status = READY;
  hdllCmdRsp->type = HCP_TYPE_NOTIFICATION;
  ret = process_hdll_response(hdllCmdRsp);

  if (NULL != hdllCmdRsp) {
    free(hdllCmdRsp);
  }
  return ret;
}

/*******************************************************************************
**
** Function    :   phGenericGetInfo
**
** Description :   This function frames the GenericGetInfo command and sends to
                   the HeliosX chip
**
** Parameters  :   None
**
** Returns     :   FW_DNLD_FAILURE - If any failure occurs while framing or
                                    sending the command or while receiving the
                                    response
                   FW_DNLD_SUCCESS - On success
**
**
*******************************************************************************/
phFWD_Status_t phGenericGetInfo() {
  uint8_t rsp_buf[HDLL_READ_BUFF_SIZE] = {0};
  uint8_t *hdll_frame = NULL;
  phFWD_Status_t ret = FW_DNLD_FAILURE;
  uint16_t rsp_buf_len = 0x0;
  phHDLLCmd_t *hdllCmd = NULL;
  phHDLLCmdRsp_t *hdllCmdRsp = NULL;

  hdllCmd = (phHDLLCmd_t *)malloc(sizeof(phHDLLCmd_t));
  if (NULL == hdllCmd) {
    ret = FW_DNLD_FAILURE;
    goto exit;
  }
  hdllCmd->group = HCP_OPERATION_GROUP_GENERIC;
  hdllCmd->operation = GENERIC_GROUP_OP_CODE_GETINFO;
  hdllCmd->chunk_size = 0;
  hdllCmd->frame_size = 0;
  hdllCmd->payload = NULL;
  hdllCmd->payload_len = 0;

  hdll_frame = phBuildHdllCmd(hdllCmd);
  if (NULL == hdll_frame) {
    goto exit;
  }
  NXPLOG_FWDNLD_D("Sending operation: OP_GENERIC_GET_INFO\n");
  ret = phGenericSendAndRecv(hdll_frame, hdllCmd->frame_size, rsp_buf,
                             &rsp_buf_len);
  if (!rsp_buf_len || ret == FW_DNLD_FAILURE) {
    NXPLOG_FWDNLD_D("Error in sending/receiving hdll cmd/response\n");
    return ret;
  }

  hdllCmdRsp = (phHDLLCmdRsp_t *)malloc(sizeof(phHDLLCmdRsp_t));
  if (NULL == hdllCmdRsp) {
    ret = FW_DNLD_FAILURE;
    goto exit;
  }
  hdllCmdRsp->group = HCP_OPERATION_GROUP_GENERIC;
  hdllCmdRsp->operation = GENERIC_GROUP_OP_CODE_GETINFO;
  hdllCmdRsp->rsp_buf = rsp_buf;
  hdllCmdRsp->rsp_buf_len = rsp_buf_len;
  hdllCmdRsp->status = GENERIC_SUCCESS;
  hdllCmdRsp->type = HCP_TYPE_RESPONSE;
  ret = process_hdll_response(hdllCmdRsp);
exit:
  if (NULL != hdll_frame) {
    free(hdll_frame);
  }
  if (NULL != hdllCmd) {
    free(hdllCmd);
  }
  if (NULL != hdllCmdRsp) {
    free(hdllCmdRsp);
  }
  return ret;
}

/*******************************************************************************
**
** Function    :   phHdll_GetHdllReadyNtf
**
** Description :   This function frames the GetHdllReadyNtf command and sends to
                   the HeliosX chip
**
** Parameters  :   None
**
** Returns     :   FW_DNLD_FAILURE - If any failure occurs while framing or
                                    sending the command or while receiving the
                                    response
                   FW_DNLD_SUCCESS - On success
**
**
*******************************************************************************/
phFWD_Status_t phHdll_GetHdllReadyNtf() {
  uint8_t rsp_buf[HDLL_READ_BUFF_SIZE] = {0};
  phFWD_Status_t ret = FW_DNLD_FAILURE;
  uint16_t rsp_buf_len = 0x0;
  phHDLLCmdRsp_t *hdllCmdRsp = NULL;

  NXPLOG_FWDNLD_D("Wait for HDL_READY notification\n");
  ret =
      phHdll_GetApdu((uint8_t *)&rsp_buf[0], HDLL_READ_BUFF_SIZE, &rsp_buf_len);

  if (!rsp_buf_len || ret == FW_DNLD_FAILURE) {
    NXPLOG_FWDNLD_D("Error in reading GET_HDL_READY notification\n");
    return ret;
  }

  hdllCmdRsp = (phHDLLCmdRsp_t *)malloc(sizeof(phHDLLCmdRsp_t));
  if (NULL == hdllCmdRsp) {
    return ret;
  }

  hdllCmdRsp->group = HCP_OPERATION_GROUP_PROTOCOL;
  hdllCmdRsp->operation = PROTOCOL_GROUP_OP_CODE_HDLL;
  hdllCmdRsp->rsp_buf = rsp_buf;
  hdllCmdRsp->rsp_buf_len = rsp_buf_len;
  hdllCmdRsp->status = READY;
  hdllCmdRsp->type = HCP_TYPE_NOTIFICATION;
  ret = process_hdll_response(hdllCmdRsp);

  if (FW_DNLD_SUCCESS != ret) {
    // check whether we received EDL ready notification or not
    // if yes, perform FW download directly.
    hdllCmdRsp->group = HCP_OPERATION_GROUP_PROTOCOL;
    hdllCmdRsp->operation = PROTOCOL_GROUP_OP_CODE_EDL;
    hdllCmdRsp->rsp_buf = rsp_buf;
    hdllCmdRsp->rsp_buf_len = rsp_buf_len;
    hdllCmdRsp->status = READY;
    hdllCmdRsp->type = HCP_TYPE_NOTIFICATION;
    ret = process_hdll_response(hdllCmdRsp);

    if (FW_DNLD_SUCCESS == ret) {
      bSkipEdlCheck = true;
    }
  }

  if (NULL != hdllCmdRsp) {
    free(hdllCmdRsp);
  }

  return ret;
}

/*******************************************************************************
**
** Function    :   phEdl_send_and_recv
**
** Description :   This function sends and receives the EDL group commands and
                   responses based on the given operation code.
**
** Parameters  :   hdll_data - HDLL command buffer
                   hdll_data_len - HDLL command buffer len
                   group - HCP group code
                   operation - operation code.
**
** Returns     :   FW_DNLD_FAILURE - If any failure occurs while framing or
                                    sending the command or while receiving the
                                    response
                   FW_DNLD_SUCCESS - On success
**
**
*******************************************************************************/

phFWD_Status_t phEdl_send_and_recv(uint8_t *hdll_data, uint32_t hdll_data_len,
                                   uint8_t group, uint8_t operation) {
  phFWD_Status_t ret = FW_DNLD_FAILURE;
  uint8_t rsp_buff[HDLL_READ_BUFF_SIZE] = {0};

  if (group != HCP_OPERATION_GROUP_EDL) {
    NXPLOG_FWDNLD_D("Error! HCP operation group is not EDL\n");
    return ret;
  }
  switch (operation) {
  case EDL_DOWNLOAD_CERTIFICATE: {
    ret = sendEdlDownloadCertificateCmd(hdll_data, hdll_data_len, rsp_buff);
  } break;
  case EDL_DOWNLOAD_FLASH_WRITE_FIRST: {
    ret = sendEdlFlashWriteFirstCmd(hdll_data, hdll_data_len, rsp_buff);
  } break;
  case EDL_DOWNLOAD_FLASH_WRITE: {
    ret = sendEdlFlashWriteCmd(hdll_data, hdll_data_len, rsp_buff);
  } break;
  case EDL_DOWNLOAD_FLASH_WRITE_LAST: {
    ret = sendEdlFlashWriteLastCmd(hdll_data, hdll_data_len, rsp_buff);
  } break;
  case EDL_LIFECYCLE_CERTIFICATE: {
    ret = sendEdlLifecycleCertificateCmd(hdll_data, hdll_data_len, rsp_buff);
  } break;
  case EDL_LIFECYCLE_WRITE_FIRST: {
    ret = sendEdlLifecycleWriteFirstCmd(hdll_data, hdll_data_len, rsp_buff);
  } break;
  case EDL_LIFECYCLE_WRITE_LAST: {
    ret = sendEdlLifecycleWriteLastCmd(hdll_data, hdll_data_len, rsp_buff);
  } break;
  case EDL_PATCH_FLASH_WRITE: {
    ret = sendEdlPatchFlashWriteCmd(hdll_data, hdll_data_len, rsp_buff);
  } break;

  default:
    break;
  }
  return ret;
}

/*******************************************************************************
**
** Function    :   phLoadFwBinary
**
** Description :   This function reads the MW FW binary file and writes to
                   HeliosX chip.
**
** Parameters  :   pfwImageCtx -> pointer to fw image context
**
** Returns     :   FW_DNLD_FAILURE - on failure
                   FW_DNLD_SUCCESS - On success
**
**
*******************************************************************************/
phFWD_Status_t phLoadFwBinary(phUwbFWImageContext_t *pfwImageCtx) {
  uint32_t next_frame_first_byte_index = 0;
  uint8_t current_op_group;
  uint8_t current_op;
  uint32_t frame_payload_length = 0;
  uint32_t frame_length = 0;
  phFWD_Status_t status = FW_DNLD_FAILURE;
  uint8_t current_frame[MAX_FRAME_LEN] = {0};

  if (NULL == pfwImageCtx->fwImage) {
    return status;
  }
  NXPLOG_FWDNLD_D("phLoadFwBinary\n");
  while (1) {
    // compute next frame payload length
    // TODO: warning this is not HDLL fragmentation compatible (valid header can
    // have chunk flag (biy 10 (13)) set) Assuming header length is 2 bytes
    frame_payload_length = (pfwImageCtx->fwImage[next_frame_first_byte_index] << 8) +
                           (pfwImageCtx->fwImage[next_frame_first_byte_index + 1]);

    // if max_payload_length is not None and (frame_payload_length >=
    // max_payload_length): raise Exception('Invalid SFWU content (not an HDLL
    // header).')

    // copy the header, the payload and the footer (crc) from the file bytes
    // into a byte array
    frame_length = frame_payload_length + HDLL_HEADER_LEN + HDLL_FOOTER_LEN;
    if (frame_length > MAX_FRAME_LEN) {
      NXPLOG_FWDNLD_E("%s: Error while performing FW download frame_length > "
                      "MAX_FRAME_LEN\n",
                      __func__);
      status = FW_DNLD_FAILURE;
      break;
    }
    memcpy(current_frame, &pfwImageCtx->fwImage[next_frame_first_byte_index],
           frame_length);
    current_op_group = current_frame[2];
    current_op = current_frame[3];

    status = phEdl_send_and_recv(current_frame, frame_length, current_op_group,
                                 current_op);
    if (status != FW_DNLD_SUCCESS) {
      NXPLOG_FWDNLD_E("%s: Error while performing FW download\n", __func__);
      break;
    }

    // update byte index
    next_frame_first_byte_index = next_frame_first_byte_index + frame_length;

    // check end of file
    if (next_frame_first_byte_index >= pfwImageCtx->fwImgSize) {
      break;
    }
  }

  // clean-up
  if (pfwImageCtx->fwImage != NULL) {
    if (pfwImageCtx->fw_dnld_config == BIN_FILE_BASED_FW_DOWNLOAD) {
      free(pfwImageCtx->fwImage);
    } else if (pfwImageCtx->fw_dnld_config == SO_FILE_BASED_FW_DOWNLOAD) {
      if (pfwImageCtx->gFwLib != NULL) {
        dlclose(pfwImageCtx->gFwLib);
        pfwImageCtx->gFwLib = NULL;
      }
    }

    pfwImageCtx->fwImage = NULL;
  }
  return status;
}

/******************************************************************************
 * Function         phHandle_hdll_read_timeout_cb
 *
 * Description      Timer call back function
 *
 * Returns          None
 *
 ******************************************************************************/
static void phHandle_hdll_read_timeout_cb(uint32_t timerId, void *pContext) {
  UNUSED(timerId);
  UNUSED(pContext);
  NXPLOG_FWDNLD_E("ERROR: phHandle_hdll_read_timeout_cb - HDLL read timeout\n");
  ioctl((intptr_t)tPalConfig.pDevHandle, SRXXX_SET_PWR, ABORT_READ_PENDING);
  isHdllReadTmeoutExpired = true;
}

/******************************************************************************/
/*   GLOBAL FUNCTIONS                                                         */
/******************************************************************************/

/*******************************************************************************
**
** Function    :   phHdll_GetApdu
**
** Description :   This function reads the HDLL command's response from HeliosX
                   chip over SPI.
**
** Parameters  :   pApdu     - HDLL response buffer
                   sz        - Max buffer size to be read
                   rsp_buf_len - HDLL response buffer length
**
** Returns     :   phFWD_Status_t : 0 - success
                                     1 - failure
**
**
*******************************************************************************/

phFWD_Status_t phHdll_GetApdu(uint8_t *pApdu, uint16_t sz,
                              uint16_t *rsp_buf_len) {
  // NXPLOG_FWDNLD_D("phHdll_GetApdu Enter\n");
  int ret_len = 0;
  int status = 0;

  if (sz == 0 || sz > PHHDLL_MAX_LEN_PAYLOAD_MISO) {
    NXPLOG_FWDNLD_E("ERROR: phHdll_GetApdu data len is 0 or greater than max "
                    "palyload length supported\n");
    return FW_DNLD_FAILURE;
  }

  /* Start timer */
  status = phOsalUwb_Timer_Start(timeoutTimerId, HDLL_READ_OP_TIMEOUT,
                                 &phHandle_hdll_read_timeout_cb, NULL);
  if (UWBSTATUS_SUCCESS != status) {
    NXPLOG_FWDNLD_E("%s: Response timer not started!!!", __func__);
    return FW_DNLD_FAILURE;
  }
  ret_len = read((intptr_t)tPalConfig.pDevHandle, (void *)pApdu, (sz));

  if (true == isHdllReadTmeoutExpired) {
    isHdllReadTmeoutExpired = false;
    return FW_DNLD_FAILURE;
  } else {
    /* Stop Timer */
    status = phOsalUwb_Timer_Stop(timeoutTimerId);
    if (UWBSTATUS_SUCCESS != status) {
      NXPLOG_FWDNLD_E("%s: Response timer stop ERROR!!!", __func__);
      return FW_DNLD_FAILURE;
    }
  }

  if (ret_len <= 0) {
    NXPLOG_FWDNLD_E("ERROR: Get APDU %u bytes failed!\n", sz);
    return FW_DNLD_FAILURE;
  }
  *rsp_buf_len = ret_len;
  if (is_fw_download_log_enabled == 0x01) {
    phNxpUciHal_print_packet(NXP_TML_FW_DNLD_RSP_UWBS_2_AP, pApdu, ret_len);
  }

  return FW_DNLD_SUCCESS;
}

/*******************************************************************************
**
** Function    :   phHdll_PutApdu
**
** Description :   This function sends the HDLL command to HeliosX chip over SPI
**
** Parameters  :   pApdu     - HDLL command to be sent
                   sz        - HDLL command length
**
** Returns     :   phFWD_Status_t : 0 - success
                                     1 - failure
**
**
*******************************************************************************/

phFWD_Status_t phHdll_PutApdu(uint8_t *pApdu, uint16_t sz) {
  int ret;
  int numWrote = 0;
  if (is_fw_download_log_enabled == 0x01) {
    phNxpUciHal_print_packet(NXP_TML_FW_DNLD_CMD_AP_2_UWBS, pApdu, sz);
  }

  ret = write((intptr_t)tPalConfig.pDevHandle, pApdu, sz);
  if (ret > 0) {
    numWrote += ret;
  } else if (ret == 0) {
    NXPLOG_FWDNLD_E("_spi_write() EOF");
    return FW_DNLD_FAILURE;
  } else {
    NXPLOG_FWDNLD_E("_spi_write() errno : %x", ret);
    return FW_DNLD_FAILURE;
  }
  return FW_DNLD_SUCCESS;
}

/*******************************************************************************
 * Function         hdll_fw_download
 *
 * Description      This function is called by jni when wired mode is
 *                  performed.First SRXXX driver will give the access
 *                  permission whether wired mode is allowed or not
 *                  arg (0):
 * Returns          FW_DNLD_SUCCESS - on success
                    FW_DNLD_FAILURE - on failure
                    FW_DNLD_FILE_NOT_FOUND - if the FW binary is not found or
                                             unable to open
 *
 ******************************************************************************/
int hdll_fw_download()
{
  phFWD_Status_t ret = FW_DNLD_FAILURE;
  unsigned long num = 0;
  NXPLOG_FWDNLD_D("hdll_fw_download enter.....\n");

  isHdllReadTmeoutExpired = false;
  bSkipEdlCheck = false;
  if (NxpConfig_GetNum(NAME_UWB_FW_DOWNLOAD_LOG, &num, sizeof(num))) {
    is_fw_download_log_enabled = (uint8_t)num;
    ALOGD("NAME_UWB_FW_DOWNLOAD_LOG: 0x%02x\n", is_fw_download_log_enabled);
  } else {
    ALOGD("NAME_UWB_FW_DOWNLOAD_LOG: failed 0x%02x\n",
          is_fw_download_log_enabled);
  }
  ioctl((intptr_t)tPalConfig.pDevHandle, SRXXX_SET_FWD, PWR_ENABLE);

  ret = phHdll_GetHdllReadyNtf();
  if (ret != FW_DNLD_SUCCESS) {
    NXPLOG_FWDNLD_E("%s:%d error in getting the hdll ready notification...\n",
                    __func__,__LINE__);
    return ret;
  }
  /* Get the Device information */
  ret = phGenericGetInfo();
  if (ret == FW_DNLD_FILE_NOT_FOUND) {
      goto exit;
  }

  if (ret == FW_DNLD_FAILURE) {
    NXPLOG_FWDNLD_E("%s: error in getting the getInfo notification...\n",
                      __func__);
    return ret;
  }

  if (!bSkipEdlCheck) {
    if (ret == FW_DNLD_NOT_REQUIRED)
    {
      goto exit;
    }
    ret = phGetEdlReadyNtf();
    if (ret != FW_DNLD_SUCCESS) {
      NXPLOG_FWDNLD_E("%s: error in getting the EDL ready notification...\n",
                      __func__);
      return ret;
    }
  }

  if(fwImageCtx.fwRecovery)
  {
    /* perform FW recovery */
    ret = phNxpUciHal_fw_recovery(&fwImageCtx);
    if (ret == FW_DNLD_FAILURE) {
      NXPLOG_FWDNLD_E("%s: error downloading recovery FW...\n",
                      __func__);
      return ret;
    }
    // TODO: Remove this after recovrry FW tested added to avoid endless loop of fw download.
    fwImageCtx.fwRecovery = false;
  }

  /*  */
  ret = phLoadFwBinary(&fwImageCtx);
  if (ret != FW_DNLD_SUCCESS) {
    NXPLOG_FWDNLD_E("%s: error in phLoadFwBinary...\n", __func__);
    return ret;
  }

exit:
  // do chip reset
  phTmlUwb_Chip_Reset();
  ret = phHdll_GetHdllReadyNtf();

  ioctl((intptr_t)tPalConfig.pDevHandle, SRXXX_SET_FWD, PWR_DISABLE);
  NXPLOG_FWDNLD_D("hdll_fw_download completed.....\n");
  return ret;
}

/*******************************************************************************
 * Function         phNxpUciHal_fw_recovery
 *
 * Description      This function is use to download recovery FW
 * Returns          FW_DNLD_SUCCESS - on success
                    FW_DNLD_FAILURE - on failure
                    FW_DNLD_FILE_NOT_FOUND - if the FW binary is not found or
                                             unable to open
 *
 ******************************************************************************/

static phFWD_Status_t phNxpUciHal_fw_recovery(phUwbFWImageContext_t *pfwImageCtx) {
  phFWD_Status_t ret = FW_DNLD_FAILURE;
  NXPLOG_FWDNLD_D("phNxpUciHal_fw_recovery enter.....\n");

  ret = phLoadFwBinary(pfwImageCtx);
  if (ret != FW_DNLD_SUCCESS) {
    NXPLOG_FWDNLD_E("%s: error in phLoadFwBinary...\n", __func__);
    return ret;
  }

  // do chip reset
  phTmlUwb_Chip_Reset();
  ret = phHdll_GetHdllReadyNtf();
  if (ret != FW_DNLD_SUCCESS) {
    NXPLOG_FWDNLD_E("%s:%d error in getting the hdll ready notification...\n",
                    __func__,__LINE__);
    return ret;
  }
  /* Get the Device information */
  ret = phGenericGetInfo();
  if (ret == FW_DNLD_FAILURE || ret == FW_DNLD_FILE_NOT_FOUND) {
      NXPLOG_FWDNLD_E("%s:%d error in getting the getInfo notification...\n",
                      __func__,__LINE__);
      return ret;
  }

  if (!bSkipEdlCheck) {
    if (ret == FW_DNLD_NOT_REQUIRED) {
      return ret;
    }

    ret = phGetEdlReadyNtf();
    if (ret != FW_DNLD_SUCCESS) {
      NXPLOG_FWDNLD_E("%s:%d error in getting the EDL ready notification...\n",
                      __func__,__LINE__);
      return ret;
    }
  }

  return ret;
}

/*******************************************************************************
 * Function         phNxpUciHal_fw_lcrotation
 *
 * Description      This function is use to download recovery FW
 * Returns          FW_DNLD_SUCCESS - on success
                    FW_DNLD_FAILURE - on failure
                    FW_DNLD_FILE_NOT_FOUND - if the FW binary is not found or
                                             unable to open
 *
 ******************************************************************************/

phFWD_Status_t phNxpUciHal_fw_lcrotation() {
  phFWD_Status_t ret = FW_DNLD_FAILURE;
  glcRotation = true;
  NXPLOG_FWDNLD_D("phNxpUciHal_fw_lcrotation enter.....\n");

  ioctl((intptr_t)tPalConfig.pDevHandle, SRXXX_SET_FWD, PWR_ENABLE);

  ret = phHdll_GetHdllReadyNtf();
  if (ret != FW_DNLD_SUCCESS) {
    NXPLOG_FWDNLD_E("%s:%d error in getting the hdll ready notification...\n",
                    __func__,__LINE__);
    return ret;
  }
    /* Get the Device information */
  ret = phGenericGetInfo();
  if (ret == FW_DNLD_FILE_NOT_FOUND) {
    goto exit;
  }

  if (ret == FW_DNLD_FAILURE) {
      NXPLOG_FWDNLD_E("%s:%d error in getting the getInfo notification...\n",
                      __func__,__LINE__);
      return ret;
  }

  if (!bSkipEdlCheck) {

    ret = phGetEdlReadyNtf();
    if (ret != FW_DNLD_SUCCESS) {
      NXPLOG_FWDNLD_E("%s:%d error in getting the EDL ready notification...\n",
                      __func__,__LINE__);
      return ret;
    }
  }
  ret = phLoadFwBinary(&fwImageCtx);
  if (ret != FW_DNLD_SUCCESS) {
    NXPLOG_FWDNLD_E("%s: error in phLoadFwBinary...\n", __func__);
    glcRotation = false;
    return ret;
  }
  glcRotation = false;

exit:
  // do chip reset
  phTmlUwb_Chip_Reset();
  ret = phHdll_GetHdllReadyNtf();

  ioctl((intptr_t)tPalConfig.pDevHandle, SRXXX_SET_FWD, PWR_DISABLE);
  NXPLOG_FWDNLD_D("hdll_fw_download completed.....\n");
  return ret;
}

/*******************************************************************************
 * Function         setDeviceHandle
 *
 * Description      This function sets the SPI device handle that needs to be
                    used in this file for SPI communication
 * Parameters       pDevHandle - SPI device handle
 * Returns          None
 *
 ******************************************************************************/
void setDeviceHandle(void *pDevHandle) {
  NXPLOG_FWDNLD_D("Set the device handle!\n");
  if (pDevHandle == NULL) {
    NXPLOG_FWDNLD_E("device handle is NULL!\n");
  } else {
    tPalConfig.pDevHandle = (void *)((intptr_t)pDevHandle);
  }
}
