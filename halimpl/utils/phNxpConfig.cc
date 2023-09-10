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
#define LOG_TAG "NxpUwbConf"

#include <sys/stat.h>

#include <iomanip>
#include <sstream>
#include <stdio.h>
#include <string>
#include <unordered_map>
#include <vector>

#include <android-base/logging.h>
#include <cutils/properties.h>
#include <log/log.h>

#include "phNxpConfig.h"
#include "phNxpLog.h"

extern bool uwb_debug_enabled;

#define nxp_config_name             "libuwb-nxp.conf"
#define country_code_config_name "libuwb-countrycode.conf"

const char alternative_config_path[] = "/vendor/etc/";

#define extra_config_base "libuwb-"
#define extra_config_ext ".conf"

const char nxp_uci_config_path[] = "/vendor/etc/libuwb-uci.conf";
const char default_nxp_config_path[] = "/vendor/etc/";

using namespace::std;

class uwbParam
{
public:
    enum class type { STRING, NUMBER, BYTEARRAY };
    uwbParam();
    uwbParam(const uwbParam& param);
    uwbParam(uwbParam&& param);
    uwbParam(const string& value);
    uwbParam(vector<uint8_t>&& value);
    uwbParam(unsigned long value);
    virtual ~uwbParam();

    type getType() const { return m_type; }
    unsigned long numValue() const {return m_numValue;}
    const char*   str_value() const {return m_str_value.c_str();}
    size_t        str_len() const   {return m_str_value.length();}
    const uint8_t* arr_value() const { return m_arrValue.data(); }
    size_t arr_len() const { return m_arrValue.size(); }

    void dump(const string &tag) const;
private:
    unsigned long   m_numValue;
    string          m_str_value;
    vector<uint8_t>  m_arrValue;
    type m_type;
};

class CUwbNxpConfig
{
public:
    virtual ~CUwbNxpConfig();
    static CUwbNxpConfig& GetInstance();
    friend void readOptionalConfig(const char* optional);
    friend void readCountryCodeConfig(const char *path);

    bool    getValue(const char* name, char* pValue, size_t len) const;
    bool    getValue(const char* name, unsigned long& rValue) const;
    bool    getValue(const char* name, uint8_t* pValue, long len, long* readlen) const;

    const uwbParam*    find(const char* p_name) const;
    void    readNxpConfig(const char* fileName) const;
private:
    CUwbNxpConfig();
    bool    readConfig(const char* name, bool bResetContent);
    void    dump() const;

    unordered_map<string, uwbParam> m_map;
    bool    mValidFile;
    string  mCurrentFile;
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
bool CUwbNxpConfig::readConfig(const char* name, bool bResetContent)
{
    enum {
        BEGIN_LINE = 1,
        TOKEN,
        STR_VALUE,
        NUM_VALUE,
        ARR_SPACE,
        ARR_NUM,
        BEGIN_HEX,
        BEGIN_QUOTE,
        END_LINE
    };

    FILE*   fd;
    struct stat buf;
    string  token;
    string  strValue;
    unsigned long    numValue = 0;
    vector<uint8_t> arrValue;
    int     base = 0;
    int     c;

    unsigned long state = BEGIN_LINE;
    /* open config file, read it into a buffer */
    if ((fd = fopen(name, "rb")) == NULL)
    {
        ALOGD_IF(uwb_debug_enabled, "%s Cannot open config file %s\n", __func__, name);
        if (bResetContent)
        {
            ALOGD_IF(uwb_debug_enabled, "%s Using default value for all settings\n", __func__);
            mValidFile = false;
        }
        return false;
    }
    ALOGD_IF(uwb_debug_enabled, "%s Opened %s config %s\n", __func__, (bResetContent ? "base" : "optional"), name);
    if(stat(name, &buf) < 0)
    {
        ALOGD_IF(uwb_debug_enabled, "Get File Information failed");
        fclose(fd);
        return false;
    }

    mValidFile = true;

    if (bResetContent) {
        m_map.clear();
        mCurrentFile = name;
    }

    for (;;) {
        c = fgetc(fd);

        switch (state) {
        case BEGIN_LINE:
            if (isPrintable(c)) {
                token.clear();
                numValue = 0;
                strValue.clear();
                arrValue.clear();
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
                m_map.try_emplace(token, move(uwbParam(numValue)));
                state = END_LINE;
            }
            break;
        case NUM_VALUE:
            if (isDigit(c, base)) {
                numValue *= base;
                numValue += getDigitValue(c, base);
            } else {m_map.try_emplace(token, move(uwbParam(numValue)));
                state = END_LINE;
            }
            break;
        case ARR_SPACE:
            if (isDigit(c, 16)) {
                numValue = getDigitValue(c, base);
                state = ARR_NUM;
            } else if (c == '}') {
                m_map.try_emplace(token, move(uwbParam(move(arrValue))));
                state = END_LINE;
            } else if (c == EOF) {
                state = END_LINE;
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
                m_map.try_emplace(token, move(uwbParam(move(arrValue))));
                state = END_LINE;
            }
            break;
        case STR_VALUE:
            if (c == '"') {
                state = END_LINE;
                m_map.try_emplace(token, move(uwbParam(strValue)));
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

    fclose(fd);
    dump();
    return true;
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
CUwbNxpConfig::CUwbNxpConfig() :
    mValidFile(false)
{
}

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

/*******************************************************************************
**
** Function:    CUwbNxpConfig::GetInstance()
**
** Description: get class singleton object
**
** Returns:     none
**
*******************************************************************************/
CUwbNxpConfig& CUwbNxpConfig::GetInstance()
{
  static CUwbNxpConfig theInstance;
  if (!theInstance.mValidFile) {
    string strPath;
    if (default_nxp_config_path[0] != '\0') {
      strPath.assign(default_nxp_config_path);
      strPath += nxp_config_name;
      theInstance.readConfig(strPath.c_str(), true);
    }
  }
  return theInstance;
}

/*******************************************************************************
**
** Function:    CUwbNxpConfig::getValue()
**
** Description: get a string value of a setting
**
** Returns:     true if setting exists
**              false if setting does not exist
**
*******************************************************************************/
bool CUwbNxpConfig::getValue(const char* name, char* pValue, size_t len) const
{
    const uwbParam *param = find(name);
    if (!param)
        return false;
    if (param->getType() != uwbParam::type::STRING)
        return false;
    if (len < (param->str_len() + 1))
        return false;

    strncpy(pValue, param->str_value(), len);
    return true;
}

/*******************************************************************************
**
** Function:    CUwbNxpConfig::getValue()
**
** Description: get a byte array value of a setting
**
** Returns:     true if setting exists
**              false if setting does not exist
**
*******************************************************************************/
bool CUwbNxpConfig::getValue(const char* name, uint8_t* pValue, long len, long* readlen) const
{
    const uwbParam *param = find(name);
    if (!param)
        return false;
    if (param->getType() != uwbParam::type::BYTEARRAY)
        return false;
    if (len < param->arr_len())
        return false;
    memcpy(pValue, param->arr_value(), param->arr_len());
    if (readlen)
        *readlen = param->arr_len();
    return true;
}

/*******************************************************************************
**
** Function:    CUwbNxpConfig::getValue()
**
** Description: get a long numerical value of a setting
**
** Returns:     true if setting exists
**              false if setting does not exist
**
*******************************************************************************/
bool CUwbNxpConfig::getValue(const char* name, unsigned long& rValue) const
{
    const uwbParam *param = find(name);
    if (!param)
        return false;
    if (param->getType() != uwbParam::type::NUMBER)
        return false;

    rValue = param->numValue();
    return true;
}

/*******************************************************************************
**
** Function:    CUwbNxpConfig::find()
**
** Description: search if a setting exist in the setting array
**
** Returns:     pointer to the setting object
**
*******************************************************************************/
const uwbParam* CUwbNxpConfig::find(const char* p_name) const
{
    const auto it = m_map.find(p_name);
    if (it == m_map.cend()) {
        return NULL;
    }
    return &it->second;
}
/*******************************************************************************
**
** Function:    CUwbNxpConfig::readNxpPHYConfig()
**
** Description: read Config settings from RF conf file
**
** Returns:     none
**
*******************************************************************************/
void CUwbNxpConfig::readNxpConfig(const char* fileName) const
{
    ALOGD_IF(uwb_debug_enabled, "readNxpConfig-Enter..Reading");
    CUwbNxpConfig::GetInstance().readConfig(fileName, false);
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
    ALOGD_IF(uwb_debug_enabled, "Dump configuration file %s, %zu entries",
        mCurrentFile.c_str(), m_map.size());
    for (auto &it : m_map) {
        auto &key = it.first;
        auto &param = it.second;
        param.dump(key);
    }
}

/*******************************************************************************/
uwbParam::uwbParam() :
    m_numValue(0),
    m_type(type::NUMBER)
{
}

uwbParam::~uwbParam()
{
}

uwbParam::uwbParam(const uwbParam &param) :
    m_numValue(param.m_numValue),
    m_str_value(param.m_str_value),
    m_arrValue(param.m_arrValue),
    m_type(param.m_type)
{
}

uwbParam::uwbParam(uwbParam &&param) :
    m_numValue(param.m_numValue),
    m_str_value(move(param.m_str_value)),
    m_arrValue(move(param.m_arrValue)),
    m_type(param.m_type)
{
}

uwbParam::uwbParam(const string& value) :
    m_numValue(0),
    m_str_value(value),
    m_type(type::STRING)
{
}

uwbParam::uwbParam(unsigned long value) :
    m_numValue(value),
    m_type(type::NUMBER)
{
}

uwbParam::uwbParam(vector<uint8_t> &&value) :
    m_arrValue(move(value)),
    m_type(type::BYTEARRAY)
{
}

void uwbParam::dump(const string &tag) const
{
    if (m_type == type::NUMBER) {
        ALOGD_IF(uwb_debug_enabled, " - %s = 0x%lx", tag.c_str(), m_numValue);
    } else if (m_type == type::STRING) {
        ALOGD_IF(uwb_debug_enabled, " - %s = %s", tag.c_str(), m_str_value.c_str());
    } else if (m_type == type::BYTEARRAY) {
        stringstream ss_hex;
        ss_hex.fill('0');
        for (auto b : m_arrValue) {
            ss_hex << setw(2) << hex << (int)b << " ";
        }
        ALOGD_IF(uwb_debug_enabled, " - %s = { %s}", tag.c_str(), ss_hex.str().c_str());
    }
}

/*******************************************************************************
**
** Function:    readOptionalConfig()
**
** Description: read Config settings from an optional conf file
**
** Returns:     none
**
*******************************************************************************/
void readOptionalConfig(const char* extra)
{
    string strPath;
    if (alternative_config_path[0] != '\0')
        strPath.assign(alternative_config_path);

    strPath += extra_config_base;
    strPath += extra;
    strPath += extra_config_ext;
    CUwbNxpConfig::GetInstance().readConfig(strPath.c_str(), false);
}

/*******************************************************************************
**
** Function:    readCountryCodeConfig()
**
** Description: read Config settings from a country code conf file
**
** Returns:     none
**
*******************************************************************************/
void readCountryCodeConfig(const char *path) {

    string strPath;
    if (path[0] != '\0')
        strPath.assign(path);

    strPath += country_code_config_name;

    CUwbNxpConfig::GetInstance().readConfig(strPath.c_str(), false);
}
/*******************************************************************************
**
** Function:    GetStrValue
**
** Description: API function for getting a string value of a setting
**
** Returns:     True if found, otherwise False.
**
*******************************************************************************/
extern "C" int GetNxpConfigStrValue(const char* name, char* pValue, unsigned long len)
{
    CUwbNxpConfig& rConfig = CUwbNxpConfig::GetInstance();

    return rConfig.getValue(name, pValue, len);
}

/*******************************************************************************
**
** Function:    GetNxpConfigCountryCodeByteArrayValue()
**
** Description: Read byte array value from the config file.
**
** Parameters:
**              name    - name of the config param to read.
**              fName   - name of the Country code.
**              pValue  - pointer to input buffer.
**              bufflen - input buffer length.
**              len     - out parameter to return the number of bytes read from config file,
**                        return -1 in case bufflen is not enough.
**
** Returns:     TRUE[1] if config param name is found in the config file, else FALSE[0]
**
*******************************************************************************/
extern "C" int GetNxpConfigCountryCodeByteArrayValue(const char* name,const char* fName, uint8_t* pValue,long bufflen, long *len)
{
    ALOGD_IF(uwb_debug_enabled, "GetNxpConfigCountryCodeByteArrayValue enter....");
    CUwbNxpConfig& rConfig = CUwbNxpConfig::GetInstance();
    readOptionalConfig(fName);
    ALOGD_IF(uwb_debug_enabled, "GetNxpConfigCountryCodeByteArrayValue exit....");
    return rConfig.getValue(name, pValue, bufflen, len);
}
extern "C" int GetNxpConfigCountryCodeVersion(const char *name,
                                              const char *path, char *pValue,
                                              long bufflen) {
    CUwbNxpConfig &rConfig = CUwbNxpConfig::GetInstance();
    readCountryCodeConfig(path);
    return rConfig.getValue(name, pValue, bufflen);
}

extern "C" int GetNxpConfigCountryCodeCapsByteArrayValue(
    const char *name, const char *cc_path, const char *country_code,
    uint8_t *pValue, long bufflen, long *len) {
    CUwbNxpConfig &rConfig = CUwbNxpConfig::GetInstance();
    readCountryCodeConfig(cc_path);
    return rConfig.getValue(name, pValue, bufflen, len);
}
/*******************************************************************************
**
** Function:    GetNxpConfigUciByteArrayValue()
**
** Description: Read byte array value from the config file.
**
** Parameters:
**              name    - name of the config param to read.
**              pValue  - pointer to input buffer.
**              bufflen - input buffer length.
**              len     - out parameter to return the number of bytes read from config file,
**                        return -1 in case bufflen is not enough.
**
** Returns:     TRUE[1] if config param name is found in the config file, else FALSE[0]
**
*******************************************************************************/
extern "C" int GetNxpConfigUciByteArrayValue(const char* name, uint8_t* pValue,long bufflen, long *len)
{
    ALOGD_IF(uwb_debug_enabled, "GetNxpConfigUciByteArrayValue enter....");
    CUwbNxpConfig& rConfig = CUwbNxpConfig::GetInstance();
    rConfig.readNxpConfig(nxp_uci_config_path);
    ALOGD_IF(uwb_debug_enabled, "GetNxpConfigUciByteArrayValue exit....");
    return rConfig.getValue(name, pValue, bufflen,len);
}

/*******************************************************************************
**
** Function:    GetNxpByteArrayValue()
**
** Description: Read byte array value from the config file.
**
** Parameters:
**              name    - name of the config param to read.
**              pValue  - pointer to input buffer.
**              bufflen - input buffer length.
**              len     - out parameter to return the number of bytes read from config file,
**                        return -1 in case bufflen is not enough.
**
** Returns:     TRUE[1] if config param name is found in the config file, else FALSE[0]
**
*******************************************************************************/
extern "C" int GetNxpConfigByteArrayValue(const char* name, uint8_t* pValue,long bufflen, long *len)
{
    ALOGD_IF(uwb_debug_enabled, "GetNxpConfigByteArrayValue enter....");
    CUwbNxpConfig& rConfig = CUwbNxpConfig::GetInstance();
    rConfig.readNxpConfig(default_nxp_config_path);
    ALOGD_IF(uwb_debug_enabled, "GetNxpConfigByteArrayValue1 enter....");
    return rConfig.getValue(name, pValue, bufflen,len);
}

/*******************************************************************************
**
** Function:    GetNumValue
**
** Description: API function for getting a numerical value of a setting
**
** Returns:     true, if successful
**
*******************************************************************************/
extern "C" int GetNxpConfigNumValue(const char* name, void* pValue, unsigned long len)
{
    ALOGD_IF(uwb_debug_enabled, "GetNxpConfigNumValue... enter....");
    if (pValue == NULL){
        return false;
    }
    CUwbNxpConfig& rConfig = CUwbNxpConfig::GetInstance();
    const uwbParam* pParam = rConfig.find(name);

    if (pParam == NULL)
        return false;
    if (pParam->getType() != uwbParam::type::NUMBER)
        return false;

    unsigned long v = pParam->numValue();
    switch (len)
    {
    case sizeof(unsigned long):
        *(static_cast<unsigned long*>(pValue)) = (unsigned long)v;
        break;
    case sizeof(unsigned short):
        *(static_cast<unsigned short*>(pValue)) = (unsigned short)v;
        break;
    case sizeof(unsigned char):
        *(static_cast<unsigned char*> (pValue)) = (unsigned char)v;
        break;
    default:
        ALOGD_IF(uwb_debug_enabled, "GetNxpConfigNumValue default");
        return false;
    }
    return true;
}
