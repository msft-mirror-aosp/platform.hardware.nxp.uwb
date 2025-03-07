/*
 * Copyright 2012-2018, 2023 NXP
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

#define LOG_TAG "NxpUwbHal"

#include <string>

#include <cutils/properties.h>
#include <log/log.h>

#include "phNxpLog.h"
#include "phNxpConfig.h"

// TODO: use constexpr and move to header
const char* NXPLOG_ITEM_EXTNS = "NxpExtns";
const char* NXPLOG_ITEM_UCIHAL = "NxpUwbHal";
const char* NXPLOG_ITEM_UCIX = "NxpUciX";
const char* NXPLOG_ITEM_UCIR = "NxpUciR";
const char* NXPLOG_ITEM_FWDNLD = "NxpFwDnld";
const char* NXPLOG_ITEM_TML = "NxpUwbTml";
const char *NXP_HAL_ERROR = "NxpHalE";

/* global log level structure */
uci_log_level_t gLog_level;
uci_debug_log_file_t gLogFile;

namespace {

uint8_t phNxpLog_SetGlobalLogLevel(void) {
  uint8_t level = NXPLOG_DEFAULT_LOGLEVEL;
  unsigned long num = 0;
  char valueStr[PROPERTY_VALUE_MAX] = {0};

  // TODO: use property_get_int32()
  int len = property_get(PROP_NAME_NXPLOG_GLOBAL_LOGLEVEL, valueStr, "");
  if (len > 0) {
    // let Android property override .conf variable
    sscanf(valueStr, "%lu", &num);
    level = (unsigned char)num;
  }
  memset(&gLog_level, level, sizeof(uci_log_level_t));
  return level;
}

// TODO: add helper function for reading property + configuration
void phNxpLog_SetHALLogLevel(uint8_t level) {
  int32_t prop_level = property_get_int32(PROP_NAME_NXPLOG_HAL_LOGLEVEL, 0);
  if (prop_level > 0) {
    gLog_level.hal_log_level = prop_level;
    return;
  }
  uint8_t conf_level = NxpConfig_GetNum<uint8_t>(NAME_NXPLOG_HAL_LOGLEVEL).value_or(0);
  gLog_level.hal_log_level = std::max(level, conf_level);
}

void phNxpLog_SetExtnsLogLevel(uint8_t level) {
  int32_t prop_level = property_get_int32(PROP_NAME_NXPLOG_EXTNS_LOGLEVEL, 0);
  if (prop_level > 0) {
    gLog_level.extns_log_level = prop_level;
    return;
  }
  uint8_t conf_level = NxpConfig_GetNum<uint8_t>(NAME_NXPLOG_EXTNS_LOGLEVEL).value_or(0);
  gLog_level.extns_log_level = std::max(level, conf_level);
}

void phNxpLog_SetTmlLogLevel(uint8_t level) {
  int32_t prop_level = property_get_int32(PROP_NAME_NXPLOG_TML_LOGLEVEL, 0);
  if (prop_level > 0) {
    gLog_level.tml_log_level = prop_level;
    return;
  }

  uint8_t conf_level = NxpConfig_GetNum<uint8_t>(NAME_NXPLOG_TML_LOGLEVEL).value_or(0);
  gLog_level.tml_log_level = std::max(level, conf_level);
}

void phNxpLog_SetDnldLogLevel(uint8_t level) {
  int32_t prop_level = property_get_int32(PROP_NAME_NXPLOG_FWDNLD_LOGLEVEL, 0);
  if (prop_level > 0) {
    gLog_level.dnld_log_level = prop_level;
    return;
  }

  uint8_t conf_level = NxpConfig_GetNum<uint8_t>(NAME_NXPLOG_FWDNLD_LOGLEVEL).value_or(0);
  gLog_level.dnld_log_level = std::max(level, conf_level);
}

void phNxpLog_SetUciTxLogLevel(uint8_t level) {
  int32_t prop_level = property_get_int32(PROP_NAME_NXPLOG_UCI_LOGLEVEL, 0);
  if (prop_level > 0) {
    gLog_level.ucix_log_level = prop_level;
    gLog_level.ucir_log_level = prop_level;
    return;
  }

  uint8_t conf_level_x = NxpConfig_GetNum<uint8_t>(NAME_NXPLOG_UCIX_LOGLEVEL).value_or(0);
  uint8_t conf_level_r = NxpConfig_GetNum<uint8_t>(NAME_NXPLOG_UCIR_LOGLEVEL).value_or(0);
  gLog_level.ucix_log_level = std::max(level, conf_level_x);
  gLog_level.ucir_log_level = std::max(level, conf_level_r);
}

void phNxpLog_EnableDebugLogFile() {
  gLogFile.is_log_file_required =
      NxpConfig_GetBool(NAME_UWB_UCIX_UCIR_ERROR_LOG).value_or(false);
}

void phNxpLog_MaxFileSize() {
  gLogFile.fileSize =
      NxpConfig_GetNum<uint32_t>(NAME_UWB_DEBUG_LOG_FILE_SIZE).value_or(0);
}

}   // namespace

/******************************************************************************
 * Function         phNxpLog_InitializeLogLevel
 *
 * Description      Initialize and get log level of module from libuwb-nxp.conf
 *or
 *                  Android runtime properties.
 *                  The Android property uwb.nxp_global_log_level is to
 *                  define log level for all modules. Modules log level will
 *overwide global level.
 *                  The Android property will overwide the level
 *                  in libuwb-nxp.conf
 *
 *                  Android property names:
 *                      uwb.nxp_log_level_global    * defines log level for all
 *modules
 *                      uwb.nxp_log_level_extns     * extensions module log
 *                      uwb.nxp_log_level_hal       * Hal module log
 *                      uwb.nxp_log_level_dnld      * firmware download module
 *log
 *                      uwb.nxp_log_level_tml       * TML module log
 *                      uwb.nxp_log_level_uci       * UCI transaction log
 *
 *                  Log Level values:
 *                      NXPLOG_LOG_SILENT_LOGLEVEL  0        * No trace to show
 *                      NXPLOG_LOG_ERROR_LOGLEVEL   1        * Show Error trace
 *only
 *                      NXPLOG_LOG_WARN_LOGLEVEL    2        * Show Warning
 *trace and Error trace
 *                      NXPLOG_LOG_DEBUG_LOGLEVEL   3        * Show all traces
 *
 * Returns          void
 *
 ******************************************************************************/
void phNxpLog_InitializeLogLevel(void) {
  uint8_t level = phNxpLog_SetGlobalLogLevel();
  phNxpLog_SetHALLogLevel(level);
  phNxpLog_SetExtnsLogLevel(level);
  phNxpLog_SetTmlLogLevel(level);
  phNxpLog_SetDnldLogLevel(level);
  phNxpLog_SetUciTxLogLevel(level);
  phNxpLog_EnableDebugLogFile();
  phNxpLog_MaxFileSize();

  ALOGV("%s: global =%u, Fwdnld =%u, extns =%u, \
                hal =%u, tml =%u, ucir =%u, \
                ucix =%u",
           __func__, gLog_level.global_log_level, gLog_level.dnld_log_level,
           gLog_level.extns_log_level, gLog_level.hal_log_level,
           gLog_level.tml_log_level, gLog_level.ucir_log_level,
           gLog_level.ucix_log_level);
}

void phNxpLog_printErrorLogsTime(const char *format, ...) {
  char yy_time[20] = "";
  time_t current_time = time(0);
  tm *dd_mm_tm = localtime(&current_time);
  if (gLogFile.is_log_file_required && gLogFile.debuglogFile != NULL) {
    if ((ftell(gLogFile.debuglogFile) + strlen(yy_time) +
         strlen(NXP_HAL_ERROR) + 100) > gLogFile.fileSize) {
      if (fseek(gLogFile.debuglogFile, 9L, SEEK_SET)) {
        NXPLOG_UCIHAL_E("phNxpUciHalProp_print_log: fseek() failed at %d",
                        __LINE__);
        return;
      }
      if (ftell(gLogFile.debuglogFile) > gLogFile.fileSize) {
        return;
      }
    }

    strftime(yy_time, sizeof(yy_time), "%x %T", dd_mm_tm);
    fprintf(gLogFile.debuglogFile, "\n%s %s:", yy_time, NXP_HAL_ERROR);
    va_list arg;
    va_start(arg, format);
    // fprintf (stdout, format, arg);
    vfprintf(gLogFile.debuglogFile, format, arg);
    va_end(arg);
  }
}
