/*
 *
 * Copyright 2025 NXP.
 *
 * NXP Confidential. This software is owned or controlled by NXP and may only be
 * used strictly in accordance with the applicable license terms. By expressly
 * accepting such terms or by downloading,installing, activating and/or
 * otherwise using the software, you are agreeing that you have read,and that
 * you agree to comply with and are bound by, such license terms. If you do not
 * agree to be bound by the applicable license terms, then you may not retain,
 * install, activate or otherwise use the software.
 *
 */

#include "phNxpUciHal_ext.h"
#include "phUwbTypes.h"
#include <stdio.h>
#include <sys/stat.h>
#include <time.h>

/******************* Global variables *****************************************/
phNxpUciHalLog_Control_t nxpucihallog_ctrl;
extern phNxpUciHal_Control_t nxpucihal_ctrl;
extern uci_debug_log_file_t gLogFile;

/******************************************************************************
 * Function         phNxpUciLog_initialize
 *
 * Description      This function is called during the initialization of the UWB
 *
 * Returns          void
 *
 ******************************************************************************/
void phNxpUciLog_initialize() {

  char UCI_Logger_log_path[100] = {0};

  if (!gLogFile.is_log_file_required) {
    return;
  }

  gLogFile.debuglogFile = NULL;
  sprintf(UCI_Logger_log_path, "%suci_debug_log.txt", debug_log_path);
  if (NULL == (gLogFile.debuglogFile = fopen(UCI_Logger_log_path, "rb+"))) {
    NXPLOG_UCIHAL_D("unable to open log file");
    if (NULL == (gLogFile.debuglogFile = fopen(UCI_Logger_log_path, "wb"))) {
      NXPLOG_UCIHAL_D("unable to create log file");
    } else {
      long offset = 0;
      NXPLOG_UCIHAL_D("Created debug log file set 0 as offset");
      fwrite(&offset, sizeof(offset), 1, gLogFile.debuglogFile);
      fwrite("\n", sizeof(char), 1, gLogFile.debuglogFile);
    }
  } else {
    long offset = 0;
    NXPLOG_UCIHAL_D("debug log file exist set offset");
    if (1 != fread(&offset, sizeof(long), 1, gLogFile.debuglogFile)) {
      NXPLOG_UCIHAL_D("phNxpUciPropHal_initialize: fread() failed at %d",
                      __LINE__);
      return;
    }
    if (fseek(gLogFile.debuglogFile, offset, SEEK_SET)) {
      NXPLOG_UCIHAL_E("phNxpUciHalProp_print_log: fseek() failed at %d",
                      __LINE__);
      return;
    }
  }

  if (chmod(UCI_Logger_log_path, 0744) != 0) {
    NXPLOG_UCIHAL_E("Can't change chmod log");
  }
}

/******************************************************************************
 * Function         phNxpUciHalProp_fw_crash
 *
 * Description      FW crash dump log function
 *
 * Returns          None
 *
 ******************************************************************************/
static void phNxpUciHalProp_fw_crash() {
  NXPLOG_UCIHAL_D("[%s]", __func__);
  tHAL_UWB_STATUS status;
  // Debug get error log command: GID = UCI_GID_PROPRIETARY
  // OID = EXT_UCI_MSG_DBG_GET_ERROR_LOG
  std::vector<uint8_t> payload = {0x2E, 0x02, 0x00, 0x00};
  phNxpUciHal_rx_handler_add(UCI_MT_RSP, UCI_GID_PROPRIETARY,
                             EXT_UCI_MSG_DBG_GET_ERROR_LOG, true,
                             phNxpUciHal_dump_log);
  status = phNxpUciHal_send_ext_cmd(payload.size(), payload.data());

  if (status != HAL_UWB_STATUS_OK) {
    NXPLOG_UCIHAL_E("Failed to send firmware crash command");
    return;
  }

  /* Send FW crash NTF to upper layer for triggering MW recovery */
  phNxpUciHal_send_dev_error_status_ntf();

  NXPLOG_UCIHAL_D("[%s] Firmware crash handling completed", __func__);
}

/******************************************************************************
 * Function         phNxpUciHalProp_trigger_fw_crash_log_dump
 *
 * Description      dump FW crash log when fw is crashed
 *
 *
 ******************************************************************************/
void phNxpUciHalProp_trigger_fw_crash_log_dump() {
  nxpucihallog_ctrl.log_thread_handler = std::thread(&phNxpUciHalProp_fw_crash);
  nxpucihallog_ctrl.log_thread_handler.detach();
}

/******************************************************************************
 * Function         phNxpUciHalProp_dump_log
 *
 * Description      This function is responsible for collecting and processing
 *                  debug logs. It is triggered whenever debug log data needs
 *                  to be retrieved and analyzed.
 *
 * Returns          void.
 *
 ******************************************************************************/
bool phNxpUciHal_dump_log(size_t data_len, const uint8_t *p_rx_data) {
  int cmd_len, len;
  bool isSkipPacket = false;
  const uint8_t mt = ((p_rx_data[0]) & UCI_MT_MASK) >> UCI_MT_SHIFT;
  const uint8_t gid = p_rx_data[0] & UCI_GID_MASK;
  const uint8_t oid = p_rx_data[1] & UCI_OID_MASK;
  const uint8_t pbf = (p_rx_data[0] & UCI_PBF_MASK) >> UCI_PBF_SHIFT;

  uint8_t isExtendedLength =
      (p_rx_data[EXTND_LEN_INDICATOR_OFFSET] & EXTND_LEN_INDICATOR_OFFSET_MASK);
  cmd_len = p_rx_data[NORMAL_MODE_LENGTH_OFFSET];

  if (isExtendedLength) {
    cmd_len = ((cmd_len << EXTENDED_MODE_LEN_SHIFT) |
               p_rx_data[EXTENDED_MODE_LEN_OFFSET]);
  }

  if ((gid == UCI_GID_PROPRIETARY) && (oid == EXT_UCI_MSG_DBG_GET_ERROR_LOG)) {
    if (nxpucihal_ctrl.hal_ext_enabled == 1) {
      char FW_crash_log_path[100] = {0};
      sprintf(FW_crash_log_path, "%suwb_FW_crash.log", debug_log_path);
      if (NULL ==
          (nxpucihallog_ctrl.FwCrashLogFile = fopen(FW_crash_log_path, "wb"))) {
        NXPLOG_UCIHAL_E("unable to open log file %s", FW_crash_log_path);
        nxpucihal_ctrl.cmdrsp.WakeupError(UWBSTATUS_FAILED);
      } else {
        len = fwrite(&p_rx_data[UCI_NTF_PAYLOAD_OFFSET], 1, cmd_len,
                     nxpucihallog_ctrl.FwCrashLogFile);
        fflush(nxpucihallog_ctrl.FwCrashLogFile);
        NXPLOG_UCIHAL_D("FW crash dump: %d bytes written", len);
        fclose(nxpucihallog_ctrl.FwCrashLogFile);
      }
      if (!pbf) {
        nxpucihal_ctrl.cmdrsp.Wakeup(gid, oid);
      }
    }
    isSkipPacket = true;
  }
  return isSkipPacket;
}

void phNxpUciHalProp_print_log(uint8_t what, const uint8_t *p_data,
                               uint16_t len) {
  char print_buffer[len * 3 + 1];
  char dd_mm_buffer[8];
  char UCI_Logger_log_path[100] = {0};
  const uint8_t mt = ((p_data[0]) & UCI_MT_MASK) >> UCI_MT_SHIFT;
  const uint8_t gid = p_data[0] & UCI_GID_MASK;
  const uint8_t oid = p_data[1] & UCI_OID_MASK;
  bool is_range_ntf = false;
  uint8_t status_index = 29;

  if (!gLogFile.is_log_file_required) {
    return;
  }

  if (gLogFile.debuglogFile == NULL) {
    NXPLOG_UCIHAL_E("debuglogFile file pointer is null...");
    return;
  }

  char yy_time[20];
  time_t current_time = time(0);
  tm *dd_mm_tm = localtime(&current_time);
  strftime(yy_time, sizeof(yy_time), "%x %T", dd_mm_tm);
  if (gLogFile.fileSize < 100000) {
    if (!nxpucihal_ctrl.uwb_device_initialized) {
      // Check file size
      if (ftell(gLogFile.debuglogFile) + 5 + strlen(yy_time) +
              strlen(NXPLOG_ITEM_UCIR) + 4 >
          gLogFile.fileSize) {
        if (fseek(gLogFile.debuglogFile, 9L, SEEK_SET)) {
          NXPLOG_UCIHAL_E("phNxpUciHalProp_print_log: fseek() failed at %d",
                          __LINE__);
          return;
        }
        if (ftell(gLogFile.debuglogFile) > gLogFile.fileSize) {
          return;
        }
      }
      if (mt == UCI_MT_RSP && p_data[4] != UCI_STATUS_OK) {
        fprintf(gLogFile.debuglogFile, "\n%s %s:", yy_time, NXPLOG_ITEM_UCIR);
        len = fwrite(p_data, 1, len, gLogFile.debuglogFile);
        fwrite("\n", 1, 1, gLogFile.debuglogFile);
        gLogFile.init_sequence_started = false;
      }
      if (!gLogFile.init_sequence_started) {
        fprintf(gLogFile.debuglogFile, "\n%s INIT", yy_time);
      }
      gLogFile.init_sequence_started = true;
      return;
    }
    gLogFile.init_sequence_started = false;
    if (((gid != UCI_GID_SESSION_MANAGE) ||
         (oid != UCI_MSG_SESSION_SET_APP_CONFIG)) &&
        ((gid != UCI_GID_PROPRIETARY_0X0F) ||
         (oid != SET_VENDOR_SET_CALIBRATION))) {
      switch (mt) {
      case UCI_MT_CMD:
        len = UCI_MSG_HDR_SIZE;
        break;
      case UCI_MT_RSP:
        len = UCI_RESPONSE_PAYLOAD_OFFSET;
        break;
      case UCI_MT_NTF:
        // Handle range data ntf
        if ((gid == UCI_GID_SESSION_CONTROL) &&
            (oid == UCI_OID_RANGE_DATA_NTF)) {
          // Sequence number - 4
          // first 4 bytes
          // session handle - 4
          // status
          if (p_data[4 + 15] == 0x00) {
            status_index += 2;
          } else {
            status_index += 8;
          }
          is_range_ntf = true;
        }
        break;
      default:
        break;
      }
    }
  }

  if ((gid == UCI_GID_PROPRIETARY && oid == EXT_UCI_MSG_DBG_DATA_LOGGER_NTF) ||
      ((gid == UCI_GID_PROPRIETARY_0X0F) &&
       (oid == EXT_UCI_MSG_DBG_PSDU_LOG_NTF)) ||
      ((gid == UCI_GID_PROPRIETARY_0X0F) &&
       (oid == EXT_UCI_MSG_DBG_CIR_LOG_NTF))) {
    return;
  }

  uint32_t file_size = ftell(gLogFile.debuglogFile);

  if ((file_size + (strlen(yy_time) + 1 + strlen(NXPLOG_ITEM_UCIX) + 1 + len) >=
       gLogFile.fileSize)) {
    int val = fseek(gLogFile.debuglogFile, 9L, SEEK_SET);
    if (ftell(gLogFile.debuglogFile) > gLogFile.fileSize) {
      return;
    }
  }

  switch (what) {
  case 0: {
    fprintf(gLogFile.debuglogFile, "\n%s %s:", yy_time, NXPLOG_ITEM_UCIX);
  } break;
  case 1: {
    fprintf(gLogFile.debuglogFile, "\n%s %s:", yy_time, NXPLOG_ITEM_UCIR);
  } break;
  default:
    return;
    break;
  }
  memset(print_buffer, 0, sizeof(print_buffer));
  int i = 0, j = 0;
  if (is_range_ntf) {
    fwrite(&p_data[j], 1, 9, gLogFile.debuglogFile);
    fwrite(&p_data[status_index], 1, 1, gLogFile.debuglogFile);
  } else {

    len = fwrite(&p_data[j], 1, len, gLogFile.debuglogFile);
    fflush(gLogFile.debuglogFile);
  }
}

/******************************************************************************
 * Function         phNxpUciLog_deinitialize
 *
 * Description      This function close files and frees up memory used by
 *                  proprietary hal.
 *
 * Returns          void
 *
 ******************************************************************************/
void phNxpUciLog_deinitialize() {
  /* FW debug log dump file closed */
  if (nxpucihallog_ctrl.FwCrashLogFile != NULL) {
    fclose(nxpucihallog_ctrl.FwCrashLogFile);
  }

  if (gLogFile.debuglogFile != NULL) {
    long offset = ftell(gLogFile.debuglogFile);
    fseek(gLogFile.debuglogFile, 0L, SEEK_SET);
    fwrite(&offset, sizeof(long), 1, gLogFile.debuglogFile);
    fwrite("\n", sizeof(char), 1, gLogFile.debuglogFile);
    fclose(gLogFile.debuglogFile);
    gLogFile.debuglogFile = NULL;
  }
}
