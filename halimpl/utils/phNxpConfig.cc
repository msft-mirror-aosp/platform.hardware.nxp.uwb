/******************************************************************************
 *
 *  Copyright (C) 2011-2012 Broadcom Corporation
 *  Copyright 2018-2019, 2023-2024 NXP
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
//#define LOG_NDEBUG 0
#define LOG_TAG "NxpUwbConf"

#include <filesystem>
#include <limits.h>
#include <sys/stat.h>

#include <charconv>
#include <cinttypes>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <list>
#include <memory>
#include <optional>
#include <span>
#include <sstream>
#include <sstream>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <android-base/logging.h>
#include <cutils/properties.h>
#include <log/log.h>

#include "phNxpConfig.h"
#include "phNxpUciHal.h"
#include "phNxpUciHal_ext.h"
#include "phNxpUciHal_utils.h"
#include "phNxpLog.h"

namespace {

constexpr std::string_view default_nxp_config_path = "/vendor/etc/libuwb-nxp.conf";
constexpr std::string_view country_code_config_name = "libuwb-countrycode.conf";
constexpr std::string_view nxp_uci_config_file = "libuwb-uci.conf";
constexpr std::string_view default_uci_config_path = "/vendor/etc/";
constexpr std::string_view factory_file_prefix = "cal-factory";

constexpr std::string_view country_code_specifier = "<country>";
constexpr std::string_view sku_specifier = "<sku>";
constexpr std::string_view extid_specifier = "<extid>";
constexpr std::string_view revision_specifier = "<revision>";

constexpr std::string_view extid_config_name = "cal.extid";
constexpr std::string_view extid_default_value = "defaultextid";

constexpr char prop_name_calsku[] = "persist.vendor.uwb.cal.sku";
constexpr char prop_default_calsku[] = "defaultsku";

constexpr char prop_name_revision[] = "persist.vendor.uwb.cal.revision";
constexpr char prop_default_revision[] = "defaultrevision";

class uwbParam
{
public:
    enum class type { STRING, NUMBER, BYTEARRAY, STRINGARRAY };

    uwbParam() : m_numValue(0), m_type(type::NUMBER) {}

    // only movable.
    uwbParam(const uwbParam &param)  = delete;

    uwbParam(uwbParam &&param) :
        m_numValue(param.m_numValue),
        m_str_value(std::move(param.m_str_value)),
        m_arrValue(std::move(param.m_arrValue)),
        m_arrStrValue(std::move(param.m_arrStrValue)),
        m_type(param.m_type) {}

    uwbParam(const std::string& value) :
        m_numValue(0),
        m_str_value(value),
        m_type(type::STRING) {}

    uwbParam(uint64_t value) :
        m_numValue(value),
        m_type(type::NUMBER) {}

    uwbParam(std::vector<uint8_t> &&value) :
        m_arrValue(std::move(value)),
        m_type(type::BYTEARRAY) {}

    uwbParam(std::vector<std::string> &&value) :
        m_arrStrValue(std::move(value)),
        m_type(type::STRINGARRAY) {}

    type getType() const { return m_type; }

    uint64_t numValue() const { return m_numValue; }

    std::string_view str_value() const { return m_str_value; }

    std::span<const uint8_t> arr_value() const { return m_arrValue; }

    std::vector<std::string> str_arr_value() const { return m_arrStrValue; }

    void dump(const std::string &tag) const {
        if (m_type == type::NUMBER) {
            ALOGV(" - %s = 0x%" PRIx64, tag.c_str(), m_numValue);
        } else if (m_type == type::STRING) {
            ALOGV(" - %s = %s", tag.c_str(), m_str_value.c_str());
        } else if (m_type == type::BYTEARRAY) {
            std::stringstream ss_hex;
            ss_hex.fill('0');
            for (auto b : m_arrValue) {
                ss_hex << std::setw(2) << std::hex << (int)b << " ";
            }
            ALOGV(" - %s = { %s}", tag.c_str(), ss_hex.str().c_str());
        } else if (m_type == type::STRINGARRAY) {
            std::stringstream ss;
            for (auto s : m_arrStrValue) {
                ss << "\"" << s << "\", ";
            }
            ALOGV(" - %s = { %s}", tag.c_str(), ss.str().c_str());
        }
    }
private:
    uint64_t m_numValue;
    std::string m_str_value;
    std::vector<uint8_t>  m_arrValue;
    std::vector<std::string>  m_arrStrValue;
    type m_type;
};

class CUwbNxpConfig
{
public:
    using HashType = std::unordered_map<std::string, uwbParam>;

    CUwbNxpConfig();
    CUwbNxpConfig(std::string_view filepath);

    // only movable
    CUwbNxpConfig(CUwbNxpConfig&& config);
    CUwbNxpConfig& operator=(CUwbNxpConfig&& config);

    CUwbNxpConfig(CUwbNxpConfig& config) = delete;
    CUwbNxpConfig& operator=(CUwbNxpConfig& config) = delete;

    virtual ~CUwbNxpConfig();

    bool isValid() const { return mValidFile; }
    bool isFactory() const { return mFactoryFile; }
    void reset() {
        m_map.clear();
        mValidFile = false;
    }

    const uwbParam* find(std::string_view key) const;
    void    setCountry(const std::string& strCountry);
    const char* getFilePath() const {
        return mFilePath.c_str();
    }

    void dump() const;

    const HashType& get_data() const {
        return m_map;
    }

private:
    bool readConfig();

    std::filesystem::path mFilePath;
    bool mFactoryFile = false;
    bool mValidFile = false;

    HashType m_map;
};

/*******************************************************************************
**
** Function:    isPrintable()
**
** Description: determine if 'c' is printable
**
** Returns:     1, if printable, otherwise 0
**
*******************************************************************************/
static inline bool isPrintable(char c)
{
    return  (c >= 'A' && c <= 'Z') ||
            (c >= 'a' && c <= 'z') ||
            (c >= '0' && c <= '9') ||
            c == '/' || c == '_' || c == '-' || c == '.' || c == ',';
}

/*******************************************************************************
**
** Function:    isDigit()
**
** Description: determine if 'c' is numeral digit
**
** Returns:     true, if numerical digit
**
*******************************************************************************/
static inline bool isDigit(char c, int base)
{
    if (base == 10) {
        return isdigit(c);
    } else if (base == 16) {
        return isxdigit(c);
    } else {
        return false;
    }
}

static inline bool isArrayDelimeter(char c)
{
    return (isspace(c) || c== ',' || c == ':' || c == '-' || c == '}');
}

/*******************************************************************************
**
** Function:    getDigitValue()
**
** Description: return numerical value of a decimal or hex char
**
** Returns:     numerical value if decimal or hex char, otherwise 0
**
*******************************************************************************/
inline int getDigitValue(char c, int base)
{
    if ('0' <= c && c <= '9')
        return c - '0';
    if (base == 16)
    {
        if ('A' <= c && c <= 'F')
            return c - 'A' + 10;
        else if ('a' <= c && c <= 'f')
            return c - 'a' + 10;
    }
    return 0;
}

/*******************************************************************************
**
** Function:    CUwbNxpConfig::readConfig()
**
** Description: read Config settings and parse them into a linked list
**              move the element from linked list to a array at the end
**
** Returns:     1, if there are any config data, 0 otherwise
**
*******************************************************************************/
bool CUwbNxpConfig::readConfig()
{
    enum {
        BEGIN_LINE = 1,
        TOKEN,
        STR_VALUE,
        NUM_VALUE,
        ARR_SPACE,
        ARR_STR,
        ARR_STR_SPACE,
        ARR_NUM,
        BEGIN_HEX,
        BEGIN_QUOTE,
        END_LINE
    };

    FILE*   fd;
    std::string  token;
    std::string  strValue;
    unsigned long    numValue = 0;
    std::vector<uint8_t> arrValue;
    std::vector<std::string> arrStr;
    int     base = 0;
    int     c;
    const char *name = mFilePath.c_str();
    unsigned long state = BEGIN_LINE;

    mValidFile = false;
    m_map.clear();

    /* open config file, read it into a buffer */
    if ((fd = fopen(name, "r")) == NULL)
    {
        ALOGV("Extra calibration file %s failed to open.", name);
        return false;
    }
    ALOGV("%s Opened config %s\n", __func__, name);

    for (;;) {
        c = fgetc(fd);

        switch (state) {
        case BEGIN_LINE:
            if (isPrintable(c)) {
                token.clear();
                numValue = 0;
                strValue.clear();
                arrValue.clear();
                arrStr.clear();
                state = TOKEN;
                token.push_back(c);
            } else {
                state = END_LINE;
            }
            break;
        case TOKEN:
            if (c == '=') {
                state = BEGIN_QUOTE;
            } else if (isPrintable(c)) {
                token.push_back(c);
            } else {
                state = END_LINE;
            }
            break;
        case BEGIN_QUOTE:
            if (c == '"') {
                state = STR_VALUE;
                base = 0;
            } else if (c == '0') {
                state = BEGIN_HEX;
            } else if (isDigit(c, 10)) {
                state = NUM_VALUE;
                base = 10;
                numValue = getDigitValue(c, base);
            } else if (c == '{') {
                state = ARR_SPACE;
                base = 16;
            } else {
                state = END_LINE;
            }
            break;
        case BEGIN_HEX:
            if (c == 'x' || c == 'X') {
                state = NUM_VALUE;
                base = 16;
                numValue = 0;
            } else if (isDigit(c, 10)) {
                state = NUM_VALUE;
                base = 10;
                numValue = getDigitValue(c, base);
            } else {
                m_map.try_emplace(token, uwbParam(numValue));
                state = END_LINE;
            }
            break;
        case NUM_VALUE:
            if (isDigit(c, base)) {
                numValue *= base;
                numValue += getDigitValue(c, base);
            } else {m_map.try_emplace(token, uwbParam(numValue));
                state = END_LINE;
            }
            break;
        case ARR_SPACE:
            if (isDigit(c, 16)) {
                numValue = getDigitValue(c, base);
                state = ARR_NUM;
            } else if (c == '}') {
                m_map.try_emplace(token, uwbParam(std::move(arrValue)));
                arrValue = {};
                state = END_LINE;
            } else if (c == '"') {
                state = ARR_STR;
            } else if (c == EOF) {
                state = END_LINE;
            }
            break;
        case ARR_STR:
            if (c == '"') {
                arrStr.emplace_back(strValue);
                strValue.clear();
                state = ARR_STR_SPACE;
            } else {
                strValue.push_back(c);
            }
            break;
        case ARR_STR_SPACE:
            if (c == '}') {
                m_map.try_emplace(token, uwbParam(std::move(arrStr)));
                arrStr = {};
                state = END_LINE;
            } else if (c == '"') {
                state = ARR_STR;
            }
            break;
        case ARR_NUM:
            if (isDigit(c, 16)) {
                numValue *= 16;
                numValue += getDigitValue(c, base);
            } else if (isArrayDelimeter(c)) {
                arrValue.push_back(numValue & 0xff);
                state = ARR_SPACE;
            } else {
                state = END_LINE;
            }
            if (c == '}') {
                m_map.try_emplace(token, uwbParam(std::move(arrValue)));
                arrValue = {};
                state = END_LINE;
            }
            break;
        case STR_VALUE:
            if (c == '"') {
                state = END_LINE;
                m_map.try_emplace(token, uwbParam(strValue));
            } else {
                strValue.push_back(c);
            }
            break;
        case END_LINE:
            // do nothing
        default:
            break;
        }
        if (c == EOF)
            break;
        else if (state == END_LINE && (c == '\n' || c == '\r'))
            state = BEGIN_LINE;
        else if (c == '#')
            state = END_LINE;
    }

    if (fclose(fd) != 0) {
      ALOGE("[%s] fclose failed", __func__);
    }

    if (m_map.size() > 0) {
        mValidFile = true;
        ALOGI("Extra calibration file %s opened.", name);
    }

    // Checks if this is a factory calibrated file by filename matching
    std::string filename = mFilePath.stem();
    if (filename.starts_with(factory_file_prefix)) {
        mFactoryFile = true;
    }

    return mValidFile;
}

/*******************************************************************************
**
** Function:    CUwbNxpConfig::CUwbNxpConfig()
**
** Description: class constructor
**
** Returns:     none
**
*******************************************************************************/
CUwbNxpConfig::CUwbNxpConfig() : mFactoryFile(false), mValidFile(false) {}

/*******************************************************************************
**
** Function:    CUwbNxpConfig::~CUwbNxpConfig()
**
** Description: class destructor
**
** Returns:     none
**
*******************************************************************************/
CUwbNxpConfig::~CUwbNxpConfig()
{
}

CUwbNxpConfig::CUwbNxpConfig(std::string_view filepath) : mFilePath(filepath)
{
    readConfig();
}

CUwbNxpConfig::CUwbNxpConfig(CUwbNxpConfig&& config)
{
    m_map = std::move(config.m_map);
    mValidFile = config.mValidFile;
    mFilePath = std::move(config.mFilePath);
    mFactoryFile = config.mFactoryFile;

    config.mValidFile = false;
}

CUwbNxpConfig& CUwbNxpConfig::operator=(CUwbNxpConfig&& config)
{
    m_map = std::move(config.m_map);
    mValidFile = config.mValidFile;
    mFilePath = std::move(config.mFilePath);
    mFactoryFile = config.mFactoryFile;

    config.mValidFile = false;
    return *this;
}

const uwbParam* CUwbNxpConfig::find(std::string_view key) const
{
    // TODO: how can we use the same hash function for string and string_view?
    const auto it = m_map.find(std::string(key));
    if (it == m_map.cend()) {
        return nullptr;
    }
    return &it->second;
}

/*******************************************************************************
**
** Function:    CUwbNxpConfig::dump()
**
** Description: prints all elements in the list
**
** Returns:     none
**
*******************************************************************************/
void CUwbNxpConfig::dump() const
{
    ALOGV("Dump configuration file %s : %s, %zu entries", mFilePath.c_str(),
        mValidFile ? "valid" : "invalid", m_map.size());

    for (auto &it : m_map) {
        auto &key = it.first;
        auto &param = it.second;
        param.dump(key);
    }
}

/*******************************************************************************/
class RegionCodeMap {
public:
    void loadMapping(std::string_view filepath) {
        CUwbNxpConfig config(filepath);
        if (!config.isValid()) {
            ALOGW("Region mapping was not provided.");
            return;
        }

        ALOGI("Region mapping was provided by %s", std::string(filepath).c_str());
        auto &all_params = config.get_data();
        for (auto &it : all_params) {
            const auto &region_str = it.first;
            const uwbParam *param = &it.second;

            // split space-separated strings into set
            std::stringstream ss(std::string(param->str_value()));
            std::string cc;
            std::unordered_set<std::string> cc_set;
            while (ss >> cc) {
              if (cc.length() == 2 && isupper(cc[0]) && isupper(cc[1])) {
                cc_set.emplace(std::move(cc));
              }
            }
            auto result = m_map.try_emplace(region_str, std::move(cc_set));
            if (!result.second) {
              // region conlifct : merge
              result.first->second.merge(std::move(cc_set));
            }
        }
        m_config = std::move(config);
    }
    std::string xlateCountryCode(const char country_code[2]) {
        std::string code{country_code[0], country_code[1]};
        if (m_config.isValid()) {
            for (auto &it : m_map) {
                const auto &region_str = it.first;
                const auto &cc_set = it.second;
                if (cc_set.find(code) != cc_set.end()) {
                    ALOGV("map country code %c%c --> %s",
                            country_code[0], country_code[1], region_str.c_str());
                    return region_str;
                }
            }
        }
        return code;
    }
    void reset() {
        m_config.reset();
        m_map.clear();
    }
    void dump() {
        ALOGV("Region mapping dump:");
        for (auto &entry : m_map) {
            const auto &region_str = entry.first;
            const auto &cc_set = entry.second;
            std::stringstream ss;
            for (const auto s : cc_set) {
                ss << "\"" << s << "\", ";
            }
            ALOGV("- %s = { %s}", region_str.c_str(), ss.str().c_str());
        }
    }
private:
    CUwbNxpConfig m_config;
    std::unordered_map<std::string, std::unordered_set<std::string>> m_map;
};

/*******************************************************************************/
class CascadeConfig {
public:
    CascadeConfig();

    void init(std::string_view main_config);
    void deinit();
    bool setCountryCode(const char country_code[2]);

    const uwbParam* find(std::string_view key, bool include_factory)  const;
private:
    // default_nxp_config_path
    CUwbNxpConfig mMainConfig;

    // uci config
    CUwbNxpConfig mUciConfig;

    // EXTRA_CONF_PATH[N]
    std::vector<std::pair<std::string, CUwbNxpConfig>> mExtraConfig;

    // [COUNTRY_CODE_CAP_FILE_LOCATION]/country_code_config_name
    CUwbNxpConfig mCapsConfig;

    // Region Code mapping
    RegionCodeMap mRegionMap;

    // current set of specifiers for EXTRA_CONF_PATH[]
    struct ExtraConfPathSpecifiers {
        std::string mCurSku;
        std::string mCurExtid;
        std::string mCurRegionCode;
        std::string mCurRevision;
        void reset() {
            mCurSku.clear();
            mCurExtid.clear();
            mCurRegionCode.clear();
            mCurRevision.clear();
        }
    };
    ExtraConfPathSpecifiers mExtraConfSpecifiers;

    // Re-evaluate filepaths of mExtraConfig with mExtraConfSpecifiers, and re-load them.
    // returns true if any of entries were updated.
    bool evaluateExtraConfPaths();

    void dump() {
        mMainConfig.dump();
        mUciConfig.dump();

        for (const auto &[filename, config] : mExtraConfig)
            config.dump();

        mCapsConfig.dump();
        mRegionMap.dump();
    }
};

CascadeConfig::CascadeConfig()
{
}

bool CascadeConfig::evaluateExtraConfPaths()
{
    int nr_updated = 0;

    for (auto& [filename, config] : mExtraConfig) {
        std::string new_filename(filename);

        auto posSku = new_filename.find(sku_specifier);
        if (posSku != std::string::npos && !mExtraConfSpecifiers.mCurSku.empty()) {
            new_filename.replace(posSku, sku_specifier.length(), mExtraConfSpecifiers.mCurSku);
        }

        auto posExtid = new_filename.find(extid_specifier);
        if (posExtid != std::string::npos && !mExtraConfSpecifiers.mCurExtid.empty()) {
            new_filename.replace(posExtid, extid_specifier.length(), mExtraConfSpecifiers.mCurExtid);
        }

        auto posCountry = new_filename.find(country_code_specifier);
        if (posCountry != std::string::npos && !mExtraConfSpecifiers.mCurRegionCode.empty()) {
            new_filename.replace(posCountry, country_code_specifier.length(), mExtraConfSpecifiers.mCurRegionCode);
        }

        auto posRevision = new_filename.find(revision_specifier);
        if (posRevision != std::string::npos && !mExtraConfSpecifiers.mCurRevision.empty()) {
            new_filename.replace(posRevision, revision_specifier.length(), mExtraConfSpecifiers.mCurRevision);
        }
        // re-open the file if filepath got re-evaluated.
        if (new_filename != config.getFilePath()) {
            config = CUwbNxpConfig(new_filename.c_str());
            ++nr_updated;
        }
    }
    ALOGI("%d new configuration files found.", nr_updated);
    return (nr_updated > 0);
}

void CascadeConfig::init(std::string_view main_config)
{
    ALOGV("CascadeConfig initialize with %s", std::string(main_config).c_str());

    // Main config file
    CUwbNxpConfig config(main_config);
    if (!config.isValid()) {
        ALOGW("Failed to load main config file");
        return;
    }
    mMainConfig = std::move(config);

    {
        // UCI config file
        std::string uciConfigFilePath(default_uci_config_path);
        uciConfigFilePath += nxp_uci_config_file;

        CUwbNxpConfig config(uciConfigFilePath);
        if (!config.isValid()) {
            ALOGW("Failed to load uci config file:%s",
                    uciConfigFilePath.c_str());
        } else {
            mUciConfig = std::move(config);
        }
    }

    char sku_value[PROPERTY_VALUE_MAX];
    char revision_value[PROPERTY_VALUE_MAX];
    property_get(prop_name_calsku, sku_value, prop_default_calsku);
    property_get(prop_name_revision, revision_value, prop_default_revision);

    // Read EXTRA_CONF_PATH[N]
    for (int i = 1; i <= 10; i++) {
        char key[32];
        snprintf(key, sizeof(key), "EXTRA_CONF_PATH_%d", i);
        const uwbParam *param = mMainConfig.find(key);
        if (!param)
            continue;

        std::string filename(param->str_value());

        auto entry = std::make_pair(param->str_value(), CUwbNxpConfig(filename.c_str()));
        mExtraConfig.emplace_back(std::move(entry));
    }

    // evaluate <sku> and <revision>
    mExtraConfSpecifiers.mCurSku = sku_value;
    mExtraConfSpecifiers.mCurRevision = revision_value;
    evaluateExtraConfPaths();

    // re-evaluate with "<extid>"
    mExtraConfSpecifiers.mCurExtid =
        NxpConfig_GetStr(extid_config_name).value_or(extid_default_value);
    evaluateExtraConfPaths();

    ALOGI("Provided specifiers: sku=[%s] revision=[%s] extid=[%s]", sku_value, revision_value,
        mExtraConfSpecifiers.mCurExtid.c_str());

    // Pick one libuwb-countrycode.conf with the highest VERSION number
    // from multiple directories specified by COUNTRY_CODE_CAP_FILE_LOCATION
    // XXX: Can't we just drop this feature of COUNTRY_CODE_CAP_FILE_LOCATION?
    std::vector<std::string> locations = NxpConfig_GetStrArray(NAME_COUNTRY_CODE_CAP_FILE_LOCATION);
    if ( locations.size() > 0) {
        int max_version = -1;
        std::string strPickedPath;
        bool foundCapFile = false;
        CUwbNxpConfig pickedConfig;

        for (const std::string& loc : locations) {
            if (loc.empty()) { continue; }

            std::string strPath(loc);
            strPath += country_code_config_name;
            ALOGV("Try to load %s", strPath.c_str());

            CUwbNxpConfig config(strPath.c_str());
            // This cannot be provided from factory cal file.
            if (config.isFactory()) { continue; }

            const uwbParam *param = config.find(NAME_NXP_COUNTRY_CODE_VERSION);
            int version = -2;
            if (param) {
                std::string_view v = param->str_value();
                int n;
                auto [ptr, ec] = std::from_chars(v.data(), v.data() + v.size(), n);
                if (ec == std::errc()) { version = n; }
            }
            if (version > max_version) {
                foundCapFile = true;
                pickedConfig = std::move(config);
                strPickedPath = std::move(strPath);
                max_version = version;
            }
        }
        if (foundCapFile) {
            mCapsConfig = std::move(pickedConfig);
            ALOGI("CountryCodeCaps file %s loaded with VERSION=%d", strPickedPath.c_str(), max_version);
        } else {
            ALOGI("No CountryCodeCaps specified");
        }
    } else {
        ALOGI(NAME_COUNTRY_CODE_CAP_FILE_LOCATION " was not specified, skip loading CountryCodeCaps");
    }

    // Load region mapping
    const uwbParam *param = find(NAME_REGION_MAP_PATH, /*include_factory=*/false);
    if (param) {
        mRegionMap.loadMapping(param->str_value());
    }

    ALOGD("CascadeConfig initialized");

    dump();
}

void CascadeConfig::deinit()
{
    mMainConfig.reset();
    mExtraConfig.clear();
    mCapsConfig.reset();
    mRegionMap.reset();
    mUciConfig.reset();
    mExtraConfSpecifiers.reset();
}

bool CascadeConfig::setCountryCode(const char country_code[2])
{
    std::string strRegion = mRegionMap.xlateCountryCode(country_code);

    if (strRegion == mExtraConfSpecifiers.mCurRegionCode) {
        ALOGI("Same region code(%c%c --> %s), per-country configuration not updated.",
              country_code[0], country_code[1], strRegion.c_str());
        return false;
    }

    ALOGI("Apply country code %c%c --> %s\n", country_code[0], country_code[1], strRegion.c_str());
    mExtraConfSpecifiers.mCurRegionCode = strRegion;

    return evaluateExtraConfPaths();
}

const uwbParam* CascadeConfig::find(std::string_view key, bool include_factory) const
{
    const uwbParam* param = NULL;

    param = mCapsConfig.find(key);
    if (param)
        return param;

    for (auto it = mExtraConfig.rbegin(); it != mExtraConfig.rend(); it++) {
        auto &config = it->second;
        if (!include_factory && config.isFactory()) { continue; }
        param = config.find(key);
        if (param)
            break;
    }
    if (!param) {
        param = mMainConfig.find(key);
    }
    if (!param) {
        param = mUciConfig.find(key);
    }
    return param;
}

}   // namespace

static CascadeConfig gConfig;

void NxpConfig_Init(void)
{
    gConfig.init(default_nxp_config_path);
}

void NxpConfig_Deinit(void)
{
    gConfig.deinit();
}

// return true if new per-country configuration file was load.
//        false if it can stay at the current configuration.
bool NxpConfig_SetCountryCode(const char country_code[2])
{
    return gConfig.setCountryCode(country_code);
}

std::optional<std::string_view> NxpConfig_GetStr(std::string_view key, bool include_factory)
{
    const uwbParam *param = gConfig.find(key, include_factory);
    if (param == nullptr || param->getType() != uwbParam::type::STRING) {
        return std::nullopt;
    }
    return param->str_value();
}

std::optional<std::span<const uint8_t>> NxpConfig_GetByteArray(std::string_view key, bool include_factory)
{
    const uwbParam *param = gConfig.find(key, include_factory);
    if (param == nullptr || param->getType() != uwbParam::type::BYTEARRAY) {
        return std::nullopt;
    }
    return param->arr_value();
}

std::optional<uint64_t> NxpConfig_GetUint64(std::string_view key, bool include_factory)
{
    const uwbParam* pParam = gConfig.find(key, include_factory);

    if ((pParam == nullptr) || (pParam->getType() != uwbParam::type::NUMBER)) {
        return std::nullopt;
    }
    return pParam->numValue();
}

std::optional<bool> NxpConfig_GetBool(std::string_view key, bool include_factory)
{
    const uwbParam* pParam = gConfig.find(key, include_factory);
    if (pParam == nullptr || pParam->getType() != uwbParam::type::NUMBER) {
        return std::nullopt;
    }
    return pParam->numValue();
}

std::vector<std::string> NxpConfig_GetStrArray(std::string_view key, bool include_factory)
{
    const uwbParam* param = gConfig.find(key, include_factory);
    if (param == nullptr || param->getType() != uwbParam::type::STRINGARRAY) {
        return std::vector<std::string>{};
    }
    return param->str_arr_value();
}
