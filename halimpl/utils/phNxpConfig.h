/******************************************************************************
 *
 *  Copyright (C) 2011-2012 Broadcom Corporation
 *  Copyright 2018-2019, 2023 NXP
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at:
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 ******************************************************************************/

#ifndef __CONFIG_H
#define __CONFIG_H

#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <span>
#include <string_view>

#include "phNxpLog.h"

void NxpConfig_Init(void);
void NxpConfig_Deinit(void);
bool NxpConfig_SetCountryCode(const char country_code[2]);

std::optional<std::string_view> NxpConfig_GetStr(std::string_view key, bool include_factory = true);

std::optional<std::span<const uint8_t>> NxpConfig_GetByteArray(std::string_view key, bool include_factory = true);

std::optional<uint64_t> NxpConfig_GetUint64(std::string_view key, bool include_factory = true);

template <typename T>
inline std::optional<T> NxpConfig_GetNum(std::string_view key, bool include_factory = true) {
    static_assert(std::is_integral<T>::value);
    auto res = NxpConfig_GetUint64(key, include_factory);
    if (res.has_value() && *res > std::numeric_limits<T>::max()) {
        std::string strkey(key);
        NXPLOG_UCIHAL_W("Config %s overflow", strkey.c_str());
    }
    return res;
}

// Returns true or false if key is existed as a number type parameter.
std::optional<bool> NxpConfig_GetBool(std::string_view key, bool include_factory = true);

// Returns an array of string.
std::vector<std::string> NxpConfig_GetStrArray(std::string_view key, bool include_factory = true);

// TODO: use constexpr
/* libuwb-nxp.conf parameters */
#define NAME_UWB_BOARD_VARIANT_CONFIG "UWB_BOARD_VARIANT_CONFIG"
#define NAME_UWB_BOARD_VARIANT_VERSION "UWB_BOARD_VARIANT_VERSION"
#define NAME_UWB_CORE_EXT_DEVICE_DEFAULT_CONFIG "UWB_CORE_EXT_DEVICE_DEFAULT_CONFIG"
#define NAME_UWB_DEBUG_DEFAULT_CONFIG "UWB_DEBUG_DEFAULT_CONFIG"
#define NAME_ANTENNA_PAIR_SELECTION_CONFIG1 "ANTENNA_PAIR_SELECTION_CONFIG1"
#define NAME_ANTENNA_PAIR_SELECTION_CONFIG2 "ANTENNA_PAIR_SELECTION_CONFIG2"
#define NAME_ANTENNA_PAIR_SELECTION_CONFIG3 "ANTENNA_PAIR_SELECTION_CONFIG3"
#define NAME_ANTENNA_PAIR_SELECTION_CONFIG4 "ANTENNA_PAIR_SELECTION_CONFIG4"
#define NAME_ANTENNA_PAIR_SELECTION_CONFIG5 "ANTENNA_PAIR_SELECTION_CONFIG5"
#define NAME_NXP_CORE_CONF_BLK "NXP_CORE_CONF_BLK_"
#define NAME_UWB_FW_DOWNLOAD_LOG "UWB_FW_DOWNLOAD_LOG"
#define NAME_NXP_UWB_FLASH_CONFIG "NXP_UWB_FLASH_CONFIG"
#define NAME_NXP_UWB_XTAL_38MHZ_CONFIG "NXP_UWB_XTAL_38MHZ_CONFIG"
#define NAME_NXP_UWB_EXTENDED_NTF_CONFIG "NXP_UWB_EXTENDED_NTF_CONFIG"
#define NAME_UWB_CORE_EXT_DEVICE_SR1XX_T_CONFIG "UWB_CORE_EXT_DEVICE_SR1XX_T_CONFIG"
#define NAME_UWB_CORE_EXT_DEVICE_SR1XX_S_CONFIG "UWB_CORE_EXT_DEVICE_SR1XX_S_CONFIG"
#define NAME_COUNTRY_CODE_CAP_FILE_LOCATION "COUNTRY_CODE_CAP_FILE_LOCATION"
#define NAME_UWB_VENDOR_CAPABILITY "UWB_VENDOR_CAPABILITY"

#define NAME_UWB_BINDING_LOCKING_ALLOWED "UWB_BINDING_LOCKING_ALLOWED"
#define NAME_NXP_UWB_PROD_FW_FILENAME "NXP_UWB_PROD_FW_FILENAME"
#define NAME_NXP_UWB_DEV_FW_FILENAME "NXP_UWB_DEV_FW_FILENAME"
#define NAME_NXP_UWB_FW_FILENAME "NXP_UWB_FW_FILENAME"
#define NAME_NXP_UWB_EXT_APP_DEFAULT_CONFIG "NXP_UWB_EXT_APP_DEFAULT_CONFIG"
#define NAME_NXP_UWB_EXT_APP_SR1XX_T_CONFIG "NXP_UWB_EXT_APP_SR1XX_T_CONFIG"
#define NAME_NXP_UWB_EXT_APP_SR1XX_S_CONFIG "NXP_UWB_EXT_APP_SR1XX_S_CONFIG"
#define NAME_UWB_USER_FW_BOOT_MODE_CONFIG "UWB_USER_FW_BOOT_MODE_CONFIG"
#define NAME_NXP_UWB_COUNTRY_CODE_CAPS "UWB_COUNTRY_CODE_CAPS"

#define NAME_NXP_SECURE_CONFIG_BLK "NXP_SECURE_CONFIG_BLK_"
#define NAME_PLATFORM_ID "PLATFORM_ID"

#define NAME_REGION_MAP_PATH "REGION_MAP_PATH"

#define NAME_NXP_UCI_CONFIG_PATH "NXP_UCI_CONFIG_PATH"

/* libuwb-uci.conf parameters */
#define NAME_NXP_UWB_LOW_POWER_MODE "UWB_LOW_POWER_MODE"

/* libuwb-countrycode.conf parameters */
#define NAME_NXP_COUNTRY_CODE_VERSION "VERSION"

#define NAME_AUTO_SUSPEND_ENABLE        "AUTO_SUSPEND_ENABLE"
#define NAME_AUTO_SUSPEND_TIMEOUT_MS    "AUTO_SUSPEND_TIMEOUT_MS"

#define NAME_DELETE_URSK_FOR_CCC_SESSION    "DELETE_URSK_FOR_CCC_SESSION"

/* In case the HAL has to set STS index for CCC */
#define NAME_OVERRIDE_STS_INDEX_FOR_CCC_SESSION    "OVERRIDE_STS_INDEX_FOR_CCC_SESSION"

/* default configuration */
#define default_storage_location "/data/vendor/uwb"

#endif
