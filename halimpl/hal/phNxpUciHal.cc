/*
 * Copyright 2012-2019, 2022-2023 NXP
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
#include <sys/stat.h>

#include <array>
#include <functional>
#include <string.h>
#include <list>
#include <map>
#include <mutex>
#include <unordered_set>
#include <vector>

#include <android-base/stringprintf.h>
#include <cutils/properties.h>
#include <log/log.h>

#include <phNxpLog.h>
#include <phNxpUciHal.h>
#include <phNxpUciHal_Adaptation.h>
#include <phNxpUciHal_ext.h>
#include <phTmlUwb_spi.h>

#include "hal_nxpuwb.h"
#include "phNxpConfig.h"
#include "phNxpUciHal_utils.h"
#include "sessionTrack.h"

using namespace std;
using android::base::StringPrintf;

extern map<uint16_t, vector<uint16_t>> input_map;
extern map<uint16_t, vector<uint16_t>> conf_map;

/*********************** Global Variables *************************************/
/* UCI HAL Control structure */
phNxpUciHal_Control_t nxpucihal_ctrl;

/* TML Context */
extern phTmlUwb_Context_t* gpphTmlUwb_Context;

bool uwb_device_initialized = false;
bool uwb_get_platform_id = false;
uint32_t timeoutTimerId = 0;
char persistant_log_path[120];
static uint8_t Rx_data[UCI_MAX_DATA_LEN];

/* AOA support handling */
bool isAntennaRxPairDefined = false;
int numberOfAntennaPairs = 0;

/**************** local methods used in this file only ************************/
static void phNxpUciHal_write_complete(void* pContext,
                                       phTmlUwb_TransactInfo_t* pInfo);
extern int phNxpUciHal_fw_download();
static void phNxpUciHal_getVersionInfo();

/*******************************************************************************
 * RX packet handler
 ******************************************************************************/
struct phNxpUciHal_RxHandler {
  // mt, gid, oid: packet type
  uint8_t mt;
  uint8_t gid;
  uint8_t oid;

  // skip_reporting: not reports the packet to upper layer if it's true
  bool skip_reporting;
  bool run_once;

  std::function<void(size_t packet_len, const uint8_t *packet)> callback;

  phNxpUciHal_RxHandler(uint8_t mt, uint8_t gid, uint8_t oid,
    bool skip_reporting, bool run_once,
    std::function<void(size_t packet_len, const uint8_t *packet)> callback) :
      mt(mt), gid(gid), oid(oid),
      skip_reporting(skip_reporting),
      run_once(run_once),
      callback(callback) { }
};

static std::list<std::shared_ptr<phNxpUciHal_RxHandler>> rx_handlers;
static std::mutex rx_handlers_lock;

std::shared_ptr<phNxpUciHal_RxHandler> phNxpUciHal_rx_handler_add(
  uint8_t mt, uint8_t gid, uint8_t oid,
  bool skip_reporting, bool run_once,
  std::function<void(size_t packet_len, const uint8_t *packet)> callback)
{
  auto handler = std::make_shared<phNxpUciHal_RxHandler>(mt, gid, oid,
    skip_reporting, run_once, callback);
  std::lock_guard<std::mutex> guard(rx_handlers_lock);
  rx_handlers.push_back(handler);
  return handler;
}

void phNxpUciHal_rx_handler_del(std::shared_ptr<phNxpUciHal_RxHandler> handler)
{
  std::lock_guard<std::mutex> guard(rx_handlers_lock);
  rx_handlers.remove(handler);
}

static void phNxpUciHal_rx_handler_check(size_t packet_len, const uint8_t *packet)
{
  const uint8_t mt = ((packet[0]) & UCI_MT_MASK) >> UCI_MT_SHIFT;
  const uint8_t gid = packet[0] & UCI_GID_MASK;
  const uint8_t oid = packet[1] & UCI_OID_MASK;

  std::lock_guard<std::mutex> guard(rx_handlers_lock);

  for (auto handler : rx_handlers) {
    if (mt == handler->mt && gid == handler->gid && oid == handler->oid) {
      handler->callback(packet_len, packet);
      if (handler->skip_reporting) {
        nxpucihal_ctrl.isSkipPacket = 1;
      }
    }
  }
  rx_handlers.remove_if([mt, gid, oid](auto& handler) {
    return mt == handler->mt && gid == handler->gid && oid == handler->oid && handler->run_once;
  });
}

static void phNxpUciHal_rx_handler_destroy(void)
{
  std::lock_guard<std::mutex> guard(rx_handlers_lock);
  rx_handlers.clear();
}

/******************************************************************************
 * Function         phNxpUciHal_client_thread
 *
 * Description      This function is a thread handler which handles all TML and
 *                  UCI messages.
 *
 * Returns          void
 *
 ******************************************************************************/
static void phNxpUciHal_client_thread(phNxpUciHal_Control_t* p_nxpucihal_ctrl)
{
  NXPLOG_UCIHAL_D("thread started");

  bool thread_running = true;

  while (thread_running) {
    /* Fetch next message from the UWB stack message queue */
    auto msg = p_nxpucihal_ctrl->gDrvCfg.pClientMq->recv();

    if (!thread_running) {
      break;
    }

    switch (msg->eMsgType) {
      case PH_LIBUWB_DEFERREDCALL_MSG: {
        phLibUwb_DeferredCall_t* deferCall = (phLibUwb_DeferredCall_t*)(msg->pMsgData);

        REENTRANCE_LOCK();
        deferCall->pCallback(deferCall->pParameter);
        REENTRANCE_UNLOCK();

        break;
      }

      case UCI_HAL_OPEN_CPLT_MSG: {
        REENTRANCE_LOCK();
        if (nxpucihal_ctrl.p_uwb_stack_cback != NULL) {
          /* Send the event */
          (*nxpucihal_ctrl.p_uwb_stack_cback)(HAL_UWB_OPEN_CPLT_EVT,
                                              HAL_UWB_STATUS_OK);
        }
        REENTRANCE_UNLOCK();
        break;
      }

      case UCI_HAL_CLOSE_CPLT_MSG: {
        REENTRANCE_LOCK();
        if (nxpucihal_ctrl.p_uwb_stack_cback != NULL) {
          /* Send the event */
          (*nxpucihal_ctrl.p_uwb_stack_cback)(HAL_UWB_CLOSE_CPLT_EVT,
                                              HAL_UWB_STATUS_OK);
        }
        thread_running = false;
        REENTRANCE_UNLOCK();
        break;
      }

      case UCI_HAL_INIT_CPLT_MSG: {
        REENTRANCE_LOCK();
        if (nxpucihal_ctrl.p_uwb_stack_cback != NULL) {
          /* Send the event */
          (*nxpucihal_ctrl.p_uwb_stack_cback)(HAL_UWB_INIT_CPLT_EVT,
                                              HAL_UWB_STATUS_OK);
        }
        REENTRANCE_UNLOCK();
        break;
      }

      case UCI_HAL_ERROR_MSG: {
        REENTRANCE_LOCK();
        if (nxpucihal_ctrl.p_uwb_stack_cback != NULL) {
          /* Send the event */
          (*nxpucihal_ctrl.p_uwb_stack_cback)(HAL_UWB_ERROR_EVT,
                                              HAL_UWB_ERROR_EVT);
        }
        REENTRANCE_UNLOCK();
        break;
      }
    }
  }

  NXPLOG_UCIHAL_D("NxpUciHal thread stopped");
}

bool isCountryCodeMapCreated = false;
/******************************************************************************
 * Function         phNxpUciHal_parse
 *
 * Description      This function parses all the data passing through the HAL.
 *
 * Returns          It returns true if the incoming command to be skipped.
 *
 ******************************************************************************/
bool phNxpUciHal_parse(uint16_t data_len, const uint8_t *p_data)
{
  bool ret = false;

  if (data_len < UCI_MSG_HDR_SIZE)
    return false;

  const uint8_t mt = (p_data[0] &UCI_MT_MASK) >> UCI_MT_SHIFT;
  const uint8_t gid = p_data[0] & UCI_GID_MASK;
  const uint8_t oid = p_data[1] & UCI_OID_MASK;

  if (mt == UCI_MT_CMD) {
    if ((gid == UCI_GID_ANDROID) && (oid == UCI_MSG_ANDROID_SET_COUNTRY_CODE)) {
      char country_code[2];
      if (data_len == 6) {
        country_code[0] = (char)p_data[4];
        country_code[1] = (char)p_data[5];
      } else {
        NXPLOG_UCIHAL_E("Unexpected payload length for ANDROID_SET_COUNTRY_CODE, handle this with 00 country code");
        country_code[0] = '0';
        country_code[1] = '0';
      }
      phNxpUciHal_handle_set_country_code(country_code);
      return true;
    } else if ((gid == UCI_GID_PROPRIETARY_0x0F) && (oid == SET_VENDOR_SET_CALIBRATION)) {
        if (p_data[UCI_MSG_HDR_SIZE + 1] ==
            VENDOR_CALIB_PARAM_TX_POWER_PER_ANTENNA) {
          phNxpUciHal_handle_set_calibration(p_data, data_len);
        }
    } else if ((gid == UCI_GID_SESSION_MANAGE) && (oid == UCI_MSG_SESSION_SET_APP_CONFIG)) {
      return phNxpUciHal_handle_set_app_config(&nxpucihal_ctrl.cmd_len, nxpucihal_ctrl.p_cmd_data);
    } else if ((gid == UCI_GID_SESSION_MANAGE) && (oid == UCI_MSG_SESSION_STATE_INIT)) {
      SessionTrack_onSessionInit(nxpucihal_ctrl.cmd_len, nxpucihal_ctrl.p_cmd_data);
    }
  } else if (mt == UCI_MT_RSP) {
    if ((gid == UCI_GID_CORE) && (oid == UCI_MSG_CORE_GET_CAPS_INFO)) {
        map<uint16_t, vector<uint16_t>>::iterator itr;
        vector<uint16_t>::iterator v_itr;
        uint16_t arrLen, tag, idx;

        // do not modify caps if the country code is not received from upper
        // layer.
        if (isCountryCodeMapCreated == false) {
          return false;
        }
        // Check UWBS Caps response status
        if (p_data[4] == 0) {
          idx = UCI_PKT_HDR_LEN + UCI_PKT_PAYLOAD_STATUS_LEN +
                UCI_PKT_NUM_CAPS_LEN;
          if (get_input_map(p_data, data_len, idx)) {
            NXPLOG_UCIHAL_D("Input Map created");
          } else {
            NXPLOG_UCIHAL_D("Input Map creation failed");
            return false;
          }
        } else {
          return false;
        }
        // Compare the maps for Tags and modify input map if Values are
        // different
        for (itr = input_map.begin(); itr != input_map.end(); ++itr) {
          tag = itr->first;
          // Check for the Tag in both maps
          if ((conf_map.count(tag)) == 1) {
            if (tag == UWB_CHANNELS || tag == CCC_UWB_CHANNELS) {
              NXPLOG_UCIHAL_D(
                  "Tag = 0x%02X , modify UWB_CHANNELS based on country conf ",
                  tag);
              for (uint32_t j = 0; j < (itr->second).size(); j++) {
                (input_map[tag])[j] =
                    ((conf_map[tag])[j]) & ((input_map[tag])[j]);
              }
            }
          } else {
            // TAG not found do nothing
          }
        }
        // convert the modified input map to p_caps_resp array
        memset(nxpucihal_ctrl.p_caps_resp, 0, UCI_MAX_DATA_LEN);
        // Header information from Input array is updated in initial bytes
        nxpucihal_ctrl.p_caps_resp[0] = p_data[0];
        nxpucihal_ctrl.p_caps_resp[1] = p_data[1];
        nxpucihal_ctrl.p_caps_resp[2] = p_data[2];
        nxpucihal_ctrl.p_caps_resp[4] = p_data[4];
        for (itr = input_map.begin(); itr != input_map.end(); ++itr) {
          tag = itr->first;
          // If Tag is 0xE0 or 0xE1 or 0xE2,Tag will be of 2 bytes
          if (((tag >> 8) >= 0xE0) && ((tag >> 8) <= 0xE2)) {
            nxpucihal_ctrl.p_caps_resp[idx++] = (tag & 0xFF00) >> 8;
            nxpucihal_ctrl.p_caps_resp[idx++] = (tag & 0x00FF);
          } else {
            nxpucihal_ctrl.p_caps_resp[idx++] = tag;
          }
          for (v_itr = itr->second.begin(); v_itr != itr->second.end();
               ++v_itr) {
            nxpucihal_ctrl.p_caps_resp[idx++] = (*v_itr);
          }
        }
        arrLen = idx;
        // exclude the initial header data
        nxpucihal_ctrl.p_caps_resp[3] = arrLen - UCI_PKT_HDR_LEN;
        // update the number of parameter TLVs.
        nxpucihal_ctrl.p_caps_resp[5] = input_map.size();
        input_map.clear();
        // send GET CAPS INFO response to the Upper Layer
        (*nxpucihal_ctrl.p_uwb_stack_data_cback)(arrLen,
                                                 nxpucihal_ctrl.p_caps_resp);
        // skip the incoming packet as we have send the modified response
        // already
        nxpucihal_ctrl.isSkipPacket = 1;
        ret = false;
    }
  } else {
    ret = false;
  }
  return ret;
}

/******************************************************************************
 * Function         phNxpUciHal_open
 *
 * Description      This function is called by libuwb-uci during the
 *                  initialization of the UWBC. It opens the physical connection
 *                  with UWBC (SRXXX) and creates required client thread for
 *                  operation.
 *                  After open is complete, status is informed to libuwb-uci
 *                  through callback function.
 *
 * Returns          This function return UWBSTATUS_SUCCES (0) in case of success
 *                  In case of failure returns other failure value.
 *
 ******************************************************************************/
tHAL_UWB_STATUS phNxpUciHal_open(uwb_stack_callback_t* p_cback, uwb_stack_data_callback_t* p_data_cback)
{
  static const char uwb_dev_node[256] = "/dev/srxxx";
  tHAL_UWB_STATUS wConfigStatus = UWBSTATUS_SUCCESS;

  if (nxpucihal_ctrl.halStatus == HAL_STATUS_OPEN) {
    NXPLOG_UCIHAL_E("phNxpUciHal_open already open");
    return UWBSTATUS_SUCCESS;
  }

  NxpConfig_Init();

  /* initialize trace level */
  phNxpLog_InitializeLogLevel();
  /*Create the timer for extns write response*/
  timeoutTimerId = phOsalUwb_Timer_Create();

  if (phNxpUciHal_init_monitor() == NULL) {
    NXPLOG_UCIHAL_E("Init monitor failed");
    return UWBSTATUS_FAILED;
  }

  CONCURRENCY_LOCK();

  NXPLOG_UCIHAL_E("Assigning the default helios Node: %s", uwb_dev_node);
  /* By default HAL status is HAL_STATUS_OPEN */
  nxpucihal_ctrl.halStatus = HAL_STATUS_OPEN;

  nxpucihal_ctrl.p_uwb_stack_cback = p_cback;
  nxpucihal_ctrl.p_uwb_stack_data_cback = p_data_cback;
  nxpucihal_ctrl.fw_dwnld_mode = false;

  /* Configure hardware link */
  nxpucihal_ctrl.gDrvCfg.pClientMq = std::make_shared<MessageQueue<phLibUwb_Message>>("Client");
  nxpucihal_ctrl.gDrvCfg.nLinkType = ENUM_LINK_TYPE_SPI;

  /* Initialize TML layer */
  wConfigStatus = phTmlUwb_Init(uwb_dev_node, nxpucihal_ctrl.gDrvCfg.pClientMq);
  if (wConfigStatus != UWBSTATUS_SUCCESS) {
    NXPLOG_UCIHAL_E("phTmlUwb_Init Failed");
    goto clean_and_return;
  }

  /* Create the client thread */
  nxpucihal_ctrl.client_thread =
    std::thread{ &phNxpUciHal_client_thread, &nxpucihal_ctrl };

  nxpucihal_ctrl.halStatus = HAL_STATUS_OPEN;

  CONCURRENCY_UNLOCK();

  /* Call open complete */
  phTmlUwb_DeferredCall(std::make_shared<phLibUwb_Message>(UCI_HAL_OPEN_CPLT_MSG));

  return UWBSTATUS_SUCCESS;

clean_and_return:
  CONCURRENCY_UNLOCK();

  /* Report error status */
  (*nxpucihal_ctrl.p_uwb_stack_cback)(HAL_UWB_OPEN_CPLT_EVT, HAL_UWB_ERROR_EVT);

  nxpucihal_ctrl.p_uwb_stack_cback = NULL;
  nxpucihal_ctrl.p_uwb_stack_data_cback = NULL;
  phNxpUciHal_cleanup_monitor();
  nxpucihal_ctrl.halStatus = HAL_STATUS_CLOSE;
  return wConfigStatus;
}

/******************************************************************************
 * Function         phNxpUciHal_write
 *
 * Description      This function write the data to UWBC through physical
 *                  interface (e.g. SPI) using the  driver interface.
 *                  Before sending the data to UWBC, phNxpUciHal_write_ext
 *                  is called to check if there is any extension processing
 *                  is required for the UCI packet being sent out.
 *
 * Returns          It returns number of bytes successfully written to UWBC.
 *
 ******************************************************************************/
tHAL_UWB_STATUS phNxpUciHal_write(uint16_t data_len, const uint8_t* p_data) {
  if (nxpucihal_ctrl.halStatus != HAL_STATUS_OPEN) {
    return UWBSTATUS_FAILED;
  }
  uint16_t len = 0;

  SessionTrack_keepAlive();

  CONCURRENCY_LOCK();
  phNxpUciHal_process_ext_cmd_rsp(data_len, p_data, &len);
  CONCURRENCY_UNLOCK();

  /* No data written */
  return len;
}

/******************************************************************************
 * Function         phNxpUciHal_write_unlocked
 *
 * Description      This is the actual function which is being called by
 *                  phNxpUciHal_write. This function writes the data to UWBC.
 *                  It waits till write callback provide the result of write
 *                  process.
 *
 * Returns          It returns number of bytes successfully written to UWBC.
 *
 ******************************************************************************/
tHAL_UWB_STATUS phNxpUciHal_write_unlocked(uint16_t data_len, const uint8_t* p_data) {
  tHAL_UWB_STATUS status;
  uint8_t mt, pbf, gid, oid;

  phNxpUciHal_Sem_t cb_data;
  /* Create the local semaphore */
  if (phNxpUciHal_init_cb_data(&cb_data, NULL) != UWBSTATUS_SUCCESS) {
    NXPLOG_UCIHAL_D("phNxpUciHal_write_unlocked Create cb data failed");
    data_len = 0;
    goto clean_and_return;
  }

  if ((data_len > UCI_MAX_DATA_LEN) || (data_len < UCI_PKT_HDR_LEN)) {
    NXPLOG_UCIHAL_E("Invalid data_len");
    data_len = 0;
    goto clean_and_return;
  }

  /* Create local copy of cmd_data */
  memcpy(nxpucihal_ctrl.p_cmd_data, p_data, data_len);
  nxpucihal_ctrl.cmd_len = data_len;

  data_len = nxpucihal_ctrl.cmd_len;
  UCI_MSG_PRS_HDR0(p_data, mt, pbf, gid);
  UCI_MSG_PRS_HDR1(p_data, oid);

  /* Vendor Specific Parsing logic */
  nxpucihal_ctrl.hal_parse_enabled =
      phNxpUciHal_parse(nxpucihal_ctrl.cmd_len, nxpucihal_ctrl.p_cmd_data);
  if (nxpucihal_ctrl.hal_parse_enabled) {
    goto clean_and_return;
  }
  status = phTmlUwb_Write(
      (uint8_t*)nxpucihal_ctrl.p_cmd_data, (uint16_t)nxpucihal_ctrl.cmd_len,
      (pphTmlUwb_TransactCompletionCb_t)&phNxpUciHal_write_complete,
      (void*)&cb_data);


  if (status != UWBSTATUS_PENDING) {
    NXPLOG_UCIHAL_E("write_unlocked status error");
    data_len = 0;
    goto clean_and_return;
  }

  /* Wait for callback response */
  if (SEM_WAIT(&cb_data)) {
    NXPLOG_UCIHAL_E("write_unlocked semaphore error");
    data_len = 0;
    goto clean_and_return;
  }

clean_and_return:
  phNxpUciHal_cleanup_cb_data(&cb_data);
  return data_len;
}

/******************************************************************************
 * Function         phNxpUciHal_write_complete
 *
 * Description      This function handles write callback.
 *
 * Returns          void.
 *
 ******************************************************************************/
static void phNxpUciHal_write_complete(void* pContext,
                                       phTmlUwb_TransactInfo_t* pInfo) {
  phNxpUciHal_Sem_t* p_cb_data = (phNxpUciHal_Sem_t*)pContext;

  if (pInfo->wStatus == UWBSTATUS_SUCCESS) {
    NXPLOG_UCIHAL_D("write successful status = 0x%x", pInfo->wStatus);
  } else {
    NXPLOG_UCIHAL_E("write error status = 0x%x", pInfo->wStatus);
  }
  p_cb_data->status = pInfo->wStatus;

  SEM_POST(p_cb_data);

  return;
}

/******************************************************************************
 * Function         phNxpUciHal_parse_get_capsInfo
 *
 * Description      parse the caps info response as per FIRA 2.0.
 *
 * Returns          void.
 *
 ******************************************************************************/
void phNxpUciHal_parse_get_capsInfo(uint16_t data_len, uint8_t *p_data) {
  uint8_t *p = p_data;
  uint8_t pDeviceCapsInfo[UCI_MAX_DATA_LEN];
  uint8_t *pp = pDeviceCapsInfo;
  uint8_t tagId = 0, subTagId = 0, len = 0;
  uint8_t mt = 0, gid = 0, oid = 0;
  uint8_t capsLen = p_data[5];
  uint8_t dataLen = p_data[3];
  mt = (*(p_data)&UCI_MT_MASK) >> UCI_MT_SHIFT;
  gid = p_data[0] & UCI_GID_MASK;
  oid = p_data[1] & UCI_OID_MASK;
  uint8_t *p_caps_value;
  if (mt == UCI_MT_RSP) {
    if ((gid == UCI_GID_CORE) && (oid == UCI_MSG_CORE_GET_CAPS_INFO)) {
      if (p_data[4] == 0) {
          for (uint16_t index = 6; index < data_len;) {
            tagId = p_data[index++];
            if (tagId != 0xE0 && tagId != 0xE3) {
              len = p_data[index++];
              /* b0 = Azimuth AoA -90 degree to 90 degree
               * b1 = Azimuth AoA -180 degree to 180 degree
               * b2 = Elevation AoA
               * b3 = AoA FOM
               */
              if (AOA_SUPPORT_TAG_ID == tagId) {
                if (isAntennaRxPairDefined) {
                  if (numberOfAntennaPairs == 1) {
                    *p_caps_value = 1;
                  } else if (numberOfAntennaPairs > 1) {
                    // If antenna pair more than 1 then it will support both b0
                    // = Azimuth AoA -90° to 90° and b2=Elevation AoA then value
                    // will set to 5
                    *p_caps_value = 5;
                  }
                } else {
                  *p_caps_value = 0;
                }
              } else {
                p_caps_value = (uint8_t *)(p_data + index);
              }
              UINT8_TO_STREAM(pp, tagId);
              UINT8_TO_STREAM(pp, len);
              if (tagId == CCC_SUPPORTED_PROTOCOL_VERSIONS_ID) {
                UINT8_TO_STREAM(pp, p_caps_value[len - 1]);
                UINT8_TO_STREAM(pp, p_caps_value[0]);
              } else {
                ARRAY_TO_STREAM(pp, p_caps_value, len);
              }
              index = index + len;
            } else { // ignore vendor specific data
              if ((index + 1) >= data_len) {
                break;
              }
              subTagId = p_data[index++];
              if ((index + 1) > data_len) {
                break;
              }
              len = p_data[index++];
              index = index + len;
              capsLen--;
              dataLen =
                  dataLen - (len + 3); // from datalen substract tagId,
                                       // subTagId, len and value of config
            }
          }

          // mapping device caps according to Fira 2.0
          std::array<uint8_t, NXP_MAX_CONFIG_STRING_LEN> buffer;
          buffer.fill(0);
          uint8_t *vendorConfig = NULL;
          long retlen = 0;
          int numberOfParams = 0;

          if (NxpConfig_GetByteArray(NAME_UWB_VENDOR_CAPABILITY,
                                         buffer.data(), buffer.size(),
                                         &retlen)) {
            if (retlen > 0) {
              vendorConfig = buffer.data();
              ARRAY_TO_STREAM(pp, vendorConfig, retlen);
              dataLen += retlen;

              // calculate number of params
              int index = 0, paramId, length;
              do {
                paramId = vendorConfig[index++];
                length = vendorConfig[index++];
                index = index + length;
                numberOfParams++;
              } while (index < retlen);

              NXPLOG_UCIHAL_D("Get caps read info from config file, length: "
                              "%ld, numberOfParams: %d",
                              retlen, numberOfParams);

              nxpucihal_ctrl.rx_data_len = UCI_MSG_HDR_SIZE + dataLen;
              UCI_MSG_BLD_HDR0(p, UCI_MT_RSP, UCI_GID_CORE);
              UCI_MSG_BLD_HDR1(p, UCI_MSG_CORE_GET_CAPS_INFO);
              UINT8_TO_STREAM(p, 0x00);
              UINT8_TO_STREAM(p, dataLen);
              UINT8_TO_STREAM(p, 0x00); // status
              UINT8_TO_STREAM(p, (capsLen + numberOfParams));
              ARRAY_TO_STREAM(p, pDeviceCapsInfo, dataLen);
            } else {
              NXPLOG_UCIHAL_E("Reading config file for %s failed!!!",
                              NAME_UWB_VENDOR_CAPABILITY);
            }
          }
      }
      phNxpUciHal_print_packet(NXP_TML_UCI_RSP_NTF_UWBS_2_AP, nxpucihal_ctrl.p_rx_data,
                               nxpucihal_ctrl.rx_data_len);
    }
  }
}

/******************************************************************************
 * Function         phNxpUciHal_read_complete
 *
 * Description      This function is called whenever there is an UCI packet
 *                  received from UWBC. It could be RSP or NTF packet. This
 *                  function provide the received UCI packet to libuwb-uci
 *                  using data callback of libuwb-uci.
 *                  There is a pending read called from each
 *                  phNxpUciHal_read_complete so each a packet received from
 *                  UWBC can be provide to libuwb-uci.
 *
 * Returns          void.
 *
 ******************************************************************************/
void phNxpUciHal_read_complete(void* pContext,
                                      phTmlUwb_TransactInfo_t* pInfo) {
  tHAL_UWB_STATUS status;
  uint8_t gid = 0, oid = 0, pbf = 0, mt = 0;
  UNUSED(pContext);

  if (nxpucihal_ctrl.read_retry_cnt == 1) {
    nxpucihal_ctrl.read_retry_cnt = 0;
  }

  int32_t totalLength = pInfo->wLength;
  int32_t length = 0;
  int32_t index = 0;
  while (totalLength > index)
  {
    uint8_t extBitSet = (pInfo->pBuff[index + EXTND_LEN_INDICATOR_OFFSET] & EXTND_LEN_INDICATOR_OFFSET_MASK);
    length = pInfo->pBuff[index + NORMAL_MODE_LENGTH_OFFSET];
    if (extBitSet || ((pInfo->pBuff[index] & UCI_MT_MASK) == 0x00)) {
     length = (length << EXTENDED_MODE_LEN_SHIFT) | pInfo->pBuff[index + EXTENDED_MODE_LEN_OFFSET] ;
    }
    length += UCI_MSG_HDR_SIZE;
    NXPLOG_UCIHAL_D("read successful length = %d", length);

    if (pInfo->wStatus == UWBSTATUS_SUCCESS) {
      NXPLOG_UCIHAL_D("read successful status = 0x%x", pInfo->wStatus);
      nxpucihal_ctrl.p_rx_data = &pInfo->pBuff[index];
      nxpucihal_ctrl.rx_data_len = length;
      phNxpUciHal_print_packet(NXP_TML_UCI_RSP_NTF_UWBS_2_AP, nxpucihal_ctrl.p_rx_data, nxpucihal_ctrl.rx_data_len);

      mt = ((nxpucihal_ctrl.p_rx_data[0]) & UCI_MT_MASK) >> UCI_MT_SHIFT;
      gid = nxpucihal_ctrl.p_rx_data[0] & UCI_GID_MASK;
      oid = nxpucihal_ctrl.p_rx_data[1] & UCI_OID_MASK;
      pbf = (nxpucihal_ctrl.p_rx_data[0] & UCI_PBF_MASK) >> UCI_PBF_SHIFT;

      // mapping device caps according to Fira 2.0
      if (gid == UCI_GID_CORE && oid == UCI_OID_GET_CAPS_INFO) {
        phNxpUciHal_parse_get_capsInfo(nxpucihal_ctrl.rx_data_len,
         nxpucihal_ctrl.p_rx_data);
      }

      nxpucihal_ctrl.isSkipPacket = 0;
      phNxpUciHal_parse(nxpucihal_ctrl.rx_data_len, nxpucihal_ctrl.p_rx_data);
      phNxpUciHal_rx_handler_check(pInfo->wLength, pInfo->pBuff);
      phNxpUciHal_process_response();

      if(!uwb_device_initialized) {
        if (pbf) {
          /* XXX: fix the whole logic if this really happens */
          NXPLOG_UCIHAL_E("FIXME: Fragmented packets received during device-init!");
        }
        if ((gid == UCI_GID_PROPRIETARY) && (oid == UCI_MSG_BINDING_STATUS_NTF)) {
          nxpucihal_ctrl.uwb_binding_status =
              nxpucihal_ctrl.p_rx_data[UCI_RESPONSE_STATUS_OFFSET];
          nxpucihal_ctrl.isSkipPacket = 1;
          nxpucihal_ctrl.uwb_binding_status_ntf_wait.status = UWBSTATUS_SUCCESS;
          SEM_POST(&(nxpucihal_ctrl.uwb_binding_status_ntf_wait));
        }
        if ((mt == UCI_MT_NTF) && (gid == UCI_GID_PROPRIETARY_0X0F) &&
           (oid == UCI_MSG_UWB_ESE_BINDING_NTF)) {
         nxpucihal_ctrl.uwb_binding_count =
          nxpucihal_ctrl.p_rx_data[UCI_RESPONSE_STATUS_OFFSET + 1];
         nxpucihal_ctrl.uwb_binding_status =
             nxpucihal_ctrl.p_rx_data[UCI_RESPONSE_STATUS_OFFSET + 2];
         nxpucihal_ctrl.isSkipPacket = 1;
         nxpucihal_ctrl.uwb_do_bind_ntf_wait.status = UWBSTATUS_SUCCESS;
         SEM_POST(&(nxpucihal_ctrl.uwb_do_bind_ntf_wait));
        }

        if ((mt == UCI_MT_NTF) && (gid == UCI_GID_PROPRIETARY_0X0F) &&
          (oid == UWB_ESE_BINDING_CHECK_NTF)) {
          nxpucihal_ctrl.uwb_binding_status =
              nxpucihal_ctrl.p_rx_data[UCI_RESPONSE_STATUS_OFFSET];
          nxpucihal_ctrl.p_rx_data[0] = 0x6E;
          nxpucihal_ctrl.p_rx_data[1] = 0x06;
          nxpucihal_ctrl.p_rx_data[2] = 0x00;
          nxpucihal_ctrl.p_rx_data[3] = 0x01;
          nxpucihal_ctrl.p_rx_data[4] = nxpucihal_ctrl.uwb_binding_status;
          nxpucihal_ctrl.rx_data_len = 5;
          nxpucihal_ctrl.uwb_get_binding_status_ntf_wait.status = UWBSTATUS_SUCCESS;
          SEM_POST(&(nxpucihal_ctrl.uwb_get_binding_status_ntf_wait));
        }
      }

      // phNxpUciHal_process_ext_cmd_rsp() is waiting for the response packet
      // set this true to wake it up for other reasons
      bool bWakeupExtCmd = (mt == UCI_MT_RSP);
      if (bWakeupExtCmd && nxpucihal_ctrl.ext_cb_waiting) {
        nxpucihal_ctrl.ext_cb_data.status = UWBSTATUS_SUCCESS;
      }

      /* DBG packets not yet supported, just ignore them silently */
      if (!nxpucihal_ctrl.isSkipPacket) {
        if ((mt == UCI_MT_NTF) && (gid == UCI_GID_INTERNAL) &&
            (oid == UCI_EXT_PARAM_DBG_RFRAME_LOG_NTF)) {
          nxpucihal_ctrl.isSkipPacket = 1;
        }
      }

      if (!nxpucihal_ctrl.isSkipPacket) {
        if (!pbf && mt == UCI_MT_NTF && gid == UCI_GID_CORE && oid == UCI_MSG_CORE_GENERIC_ERROR_NTF) {
          uint8_t status_code = nxpucihal_ctrl.p_rx_data[UCI_RESPONSE_STATUS_OFFSET];

          if (status_code == UCI_STATUS_COMMAND_RETRY) {
            // Handle retransmissions
            // TODO: Do not retransmit it when !nxpucihal_ctrl.hal_ext_enabled,
            // Upper layer should take care of it.
            nxpucihal_ctrl.ext_cb_data.status = UWBSTATUS_COMMAND_RETRANSMIT;
            nxpucihal_ctrl.isSkipPacket = 1;
            bWakeupExtCmd = true;
          } else  if (status_code == UCI_STATUS_INVALID_MSG_SIZE || status_code == UCI_STATUS_SYNTAX_ERROR) {
            nxpucihal_ctrl.ext_cb_data.status = UWBSTATUS_INVALID_COMMAND_LENGTH;
            bWakeupExtCmd = true;
          }
        }
      }

      // Check status code only for extension commands
      if (!nxpucihal_ctrl.isSkipPacket) {
        if (mt == UCI_MT_RSP) {
          if (nxpucihal_ctrl.hal_ext_enabled) {
            nxpucihal_ctrl.isSkipPacket = 1;

            if (pbf) {
              /* XXX: fix the whole logic if this really happens */
              NXPLOG_UCIHAL_E("FIXME: Fragmented packets received while processing internal commands!");
            }

            uint8_t status_code = (nxpucihal_ctrl.rx_data_len > UCI_RESPONSE_STATUS_OFFSET) ?
              nxpucihal_ctrl.p_rx_data[UCI_RESPONSE_STATUS_OFFSET] : UCI_STATUS_UNKNOWN;

            if (status_code == UCI_STATUS_OK) {
              nxpucihal_ctrl.ext_cb_data.status = UWBSTATUS_SUCCESS;
            } else if ((gid == UCI_GID_CORE) && (oid == UCI_MSG_CORE_SET_CONFIG)){
              /* check if any configurations are not supported then ignore the
               * UWBSTATUS_FEATURE_NOT_SUPPORTED status code*/
              nxpucihal_ctrl.ext_cb_data.status = phNxpUciHal_process_ext_rsp(nxpucihal_ctrl.rx_data_len, nxpucihal_ctrl.p_rx_data);
            } else {
              nxpucihal_ctrl.ext_cb_data.status = UWBSTATUS_FAILED;
              NXPLOG_UCIHAL_E("Got error status code(0x%x) from internal command.", status_code);
              usleep(1);  // XXX: not sure if it's really needed
            }
          }
        }
      }

      if (bWakeupExtCmd && nxpucihal_ctrl.ext_cb_waiting) {
        SEM_POST(&(nxpucihal_ctrl.ext_cb_data));
      }

      if (!nxpucihal_ctrl.isSkipPacket) {
        /* Read successful, send the event to higher layer */
        if ((nxpucihal_ctrl.p_uwb_stack_data_cback != NULL) && (nxpucihal_ctrl.rx_data_len <= UCI_MAX_PAYLOAD_LEN)) {
          (*nxpucihal_ctrl.p_uwb_stack_data_cback)(nxpucihal_ctrl.rx_data_len, nxpucihal_ctrl.p_rx_data);
        }
      }
    } else {  // pInfo->wStatus != UWBSTATUS_SUCCESS
      NXPLOG_UCIHAL_E("read error status = 0x%x", pInfo->wStatus);
    }

    /* Disable junk data check for each UCI packet*/
    if(nxpucihal_ctrl.fw_dwnld_mode) {
      if((gid == UCI_GID_CORE) && (oid == UCI_MSG_CORE_DEVICE_STATUS_NTF)){
        nxpucihal_ctrl.fw_dwnld_mode = false;
      }
    }
    index += length;
  } //End of while loop

  /* Read again because read must be pending always.*/
  if (nxpucihal_ctrl.halStatus != HAL_STATUS_CLOSE) {
    status = phTmlUwb_Read(
        Rx_data, UCI_MAX_DATA_LEN,
        (pphTmlUwb_TransactCompletionCb_t)&phNxpUciHal_read_complete, NULL);
    if (status != UWBSTATUS_PENDING) {
      NXPLOG_UCIHAL_E("read status error status = %x", status);
    }
  }
}

/******************************************************************************
 * Function         phNxpUciHal_close
 *
 * Description      This function close the UWBC interface and free all
 *                  resources.This is called by libuwb-uci on UWB service stop.
 *
 * Returns          Always return UWBSTATUS_SUCCESS (0).
 *
 ******************************************************************************/
tHAL_UWB_STATUS phNxpUciHal_close() {
  tHAL_UWB_STATUS status;
  if (nxpucihal_ctrl.halStatus == HAL_STATUS_CLOSE) {
    NXPLOG_UCIHAL_E("phNxpUciHal_close is already closed, ignoring close");
    return UWBSTATUS_FAILED;
  }

  uwb_device_initialized = false;

  CONCURRENCY_LOCK();

  SessionTrack_deinit();

  nxpucihal_ctrl.halStatus = HAL_STATUS_CLOSE;

  if (NULL != gpphTmlUwb_Context->pDevHandle) {
    NXPLOG_UCIHAL_D("Terminating phNxpUciHal client thread...");
    phTmlUwb_DeferredCall(std::make_shared<phLibUwb_Message>(UCI_HAL_CLOSE_CPLT_MSG));
    nxpucihal_ctrl.client_thread.join();

    /* Abort any pending read and write */
    status = phTmlUwb_ReadAbort();
    status = phTmlUwb_WriteAbort();

    phOsalUwb_Timer_Cleanup();

    status = phTmlUwb_Shutdown();

    phNxpUciHal_rx_handler_destroy();

    NXPLOG_UCIHAL_D("phNxpUciHal_close - phOsalUwb_DeInit completed");
  }

  CONCURRENCY_UNLOCK();

  phNxpUciHal_cleanup_monitor();

  NxpConfig_Deinit();

  /* Return success always */
  return UWBSTATUS_SUCCESS;
}


/******************************************************************************
 * Function         phNxpUciHal_sendGetCoreDeviceInfo
 *
 * Description      This function send Core device Info command.
 *
 * Returns          status.
 *
 ******************************************************************************/
static uint8_t phNxpUciHal_sendGetCoreDeviceInfo(void)
{
  if (nxpucihal_ctrl.isDevInfoCached) {
    return UWBSTATUS_SUCCESS;
  }

  const uint8_t getCoreDeviceInfoConfig[] = {0x20, 0x02, 0x00, 0x00};
  uint8_t getCoreDeviceInfoCmdLen = 4;

  tHAL_UWB_STATUS status = phNxpUciHal_send_ext_cmd(getCoreDeviceInfoCmdLen, getCoreDeviceInfoConfig);
  return status;
}

/******************************************************************************
 * Function         parseAntennaConfig
 *
 * Description      This function parse the antenna config and update required
 *                  params
 *
 * Returns          void
 *
 ******************************************************************************/
void parseAntennaConfig(uint16_t dataLength, const uint8_t *data) {
  if (dataLength > 0) {
    uint8_t index =
        UCI_MSG_HDR_SIZE + 1; // Excluding the header and number of params
    uint8_t tagId, subTagId;
    int length;
    while (index < dataLength) {
      tagId = data[index++];
      subTagId = data[index++];
      length = data[index++];
      if ((ANTENNA_RX_PAIR_DEFINE_TAG_ID == tagId) &&
          (ANTENNA_RX_PAIR_DEFINE_SUB_TAG_ID == subTagId)) {
        isAntennaRxPairDefined = true;
        numberOfAntennaPairs = data[index];
        NXPLOG_UCIHAL_D("numberOfAntennaPairs:%d", numberOfAntennaPairs);
        break;
      } else {
        index = index + length;
      }
    };
  } else {
    NXPLOG_UCIHAL_E("Reading config file for %s failed!!!",
                    NAME_UWB_CORE_EXT_DEVICE_DEFAULT_CONFIG);
  }
}

/******************************************************************************
 * Function         phNxpUciHal_configureLowPowerMode
 *
 * Description      This function applies low power mode value from config file
 *
 * Returns          success/Failure
 *
 ******************************************************************************/
static bool phNxpUciHal_configureLowPowerMode() {
  uint8_t configValue;
  unsigned long num = 1;
  bool isSendSuccess = false;

  if (NxpConfig_GetNum(NAME_NXP_UWB_LOW_POWER_MODE, &configValue, num)) {
    // Core set config packet: GID=0x00 OID=0x04
    const std::vector<uint8_t> packet(
        {((UCI_MT_CMD << UCI_MT_SHIFT) | UCI_GID_CORE), UCI_MSG_CORE_SET_CONFIG,
         0x00, 0x04, 0x01, LOW_POWER_MODE_TAG_ID, LOW_POWER_MODE_LENGTH,
         configValue});

    if (phNxpUciHal_send_ext_cmd(packet.size(), packet.data()) ==
        UWBSTATUS_SUCCESS) {
      isSendSuccess = true;
    }
  } else {
    NXPLOG_UCIHAL_E("NAME_NXP_UWB_LOW_POWER_MODE config read failed");
  }
  return isSendSuccess;
}

/******************************************************************************
 * Function         phNxpUciHal_applyVendorConfig
 *
 * Description      This function applies the vendor config from config file
 *
 * Returns          status
 *
 ******************************************************************************/
tHAL_UWB_STATUS phNxpUciHal_applyVendorConfig() {
  NXPLOG_UCIHAL_D(" phNxpUciHal_applyVendorConfig Enter..");
  std::array<uint8_t, NXP_MAX_CONFIG_STRING_LEN> buffer;
  uint8_t *vendorConfig = NULL;
  tHAL_UWB_STATUS status;
  buffer.fill(0);
  long retlen = 0;
  if (nxpucihal_ctrl.fw_boot_mode == USER_FW_BOOT_MODE) {
    if (NxpConfig_GetByteArray(NAME_UWB_USER_FW_BOOT_MODE_CONFIG,
                               buffer.data(), buffer.size(),
                               &retlen)) {
      if ((retlen > 0) && (retlen <= UCI_MAX_DATA_LEN)) {
        vendorConfig = buffer.data();
        status = phNxpUciHal_send_ext_cmd(retlen, vendorConfig);
        NXPLOG_UCIHAL_D(
            " phNxpUciHal_send_ext_cmd :: status value for %s is %d ",
            NAME_UWB_USER_FW_BOOT_MODE_CONFIG, status);
        if (status != UWBSTATUS_SUCCESS) {
          return status;
        }
      }
    }
  }
  if (NxpConfig_GetByteArray(NAME_NXP_UWB_EXTENDED_NTF_CONFIG,
                                 buffer.data(), buffer.size(),
                                 &retlen)) {
    if (retlen > 0) {
      vendorConfig = buffer.data();
      status = phNxpUciHal_send_ext_cmd(retlen, vendorConfig);
      NXPLOG_UCIHAL_D(" phNxpUciHal_send_ext_cmd :: status value for %s is %d ",
                      NAME_NXP_UWB_EXTENDED_NTF_CONFIG, status);
      if (status != UWBSTATUS_SUCCESS) {
        return status;
      }
    }
  }
  if (nxpucihal_ctrl.device_type == DEVICE_TYPE_SR1xxT) {
    if (NxpConfig_GetByteArray(NAME_UWB_CORE_EXT_DEVICE_SR1XX_T_CONFIG,
                               buffer.data(), buffer.size(),
                               &retlen)) {
      if (retlen > 0) {
        vendorConfig = buffer.data();
        status = phNxpUciHal_send_ext_cmd(retlen, vendorConfig);
        NXPLOG_UCIHAL_D(
            " phNxpUciHal_send_ext_cmd :: status value for %s is %d ",
            NAME_UWB_CORE_EXT_DEVICE_SR1XX_T_CONFIG, status);

        // AOA support handling
        parseAntennaConfig(retlen, vendorConfig);

        if (status != UWBSTATUS_SUCCESS) {
          return status;
        }
      }
    }
  } else if (nxpucihal_ctrl.device_type == DEVICE_TYPE_SR1xxS) {
    if (NxpConfig_GetByteArray(NAME_UWB_CORE_EXT_DEVICE_SR1XX_S_CONFIG,
                            buffer.data(), buffer.size(),
                            &retlen)) {
      if (retlen > 0) {
        vendorConfig = buffer.data();
        status = phNxpUciHal_send_ext_cmd(retlen, vendorConfig);
        NXPLOG_UCIHAL_D(
            " phNxpUciHal_send_ext_cmd :: status value for %s is %d ",
            NAME_UWB_CORE_EXT_DEVICE_SR1XX_S_CONFIG, status);

        // AOA support handling
        parseAntennaConfig(retlen, vendorConfig);

        if (status != UWBSTATUS_SUCCESS) {
          return status;
        }
      }
    }
  } else {
    NXPLOG_UCIHAL_D("phNxpUciHal_sendGetCoreDeviceInfo deviceType default");
    if (NxpConfig_GetByteArray(NAME_UWB_CORE_EXT_DEVICE_DEFAULT_CONFIG,
                                   buffer.data(), buffer.size(),
                                   &retlen)) {
      if (retlen > 0) {
        vendorConfig = buffer.data();
        status = phNxpUciHal_send_ext_cmd(retlen, vendorConfig);
        NXPLOG_UCIHAL_D(
            " phNxpUciHal_send_ext_cmd :: status value for %s is %d ",
            NAME_UWB_CORE_EXT_DEVICE_DEFAULT_CONFIG, status);

        // AOA support handling
        parseAntennaConfig(retlen, vendorConfig);

        if (status != UWBSTATUS_SUCCESS) {
          return status;
        }
      }
    }
  }
  if (NxpConfig_GetByteArray(NAME_NXP_UWB_XTAL_38MHZ_CONFIG,
                                 buffer.data(), buffer.size(),
                                 &retlen)) {
    if (retlen > 0) {
      vendorConfig = buffer.data();
      status = phNxpUciHal_send_ext_cmd(retlen, vendorConfig);
      NXPLOG_UCIHAL_D(" phNxpUciHal_send_ext_cmd :: status value for %s is %d ",
                      NAME_NXP_UWB_XTAL_38MHZ_CONFIG, status);
      if (status != UWBSTATUS_SUCCESS) {
        return status;
      }
    }
  }
  for(int i = 1;i <= 10;i++) {
    std::string str = NAME_NXP_CORE_CONF_BLK;
    std::string value = std::to_string(i);
    std::string name = str + value;
    NXPLOG_UCIHAL_D(" phNxpUciHal_applyVendorConfig :: Name of the config block is %s", name.c_str());
    if (NxpConfig_GetByteArray(name.c_str(), buffer.data(), buffer.size(), &retlen)) {
      if ((retlen > 0) && (retlen <= UCI_MAX_DATA_LEN)) {
        vendorConfig = buffer.data();
        status = phNxpUciHal_send_ext_cmd(retlen,vendorConfig);
        NXPLOG_UCIHAL_D(" phNxpUciHal_send_ext_cmd :: status value for %s is %d ", name.c_str(),status);
        if(status != UWBSTATUS_SUCCESS) {
          return status;
        }
      }
    } else {
      NXPLOG_UCIHAL_D(
          " phNxpUciHal_applyVendorConfig::%s not available in the config file",
          name.c_str());
    }
  }

  // low power mode
  if (!phNxpUciHal_configureLowPowerMode()) {
    NXPLOG_UCIHAL_E("phNxpUciHal_send_ext_cmd for %s failed",
                    NAME_NXP_UWB_LOW_POWER_MODE);
  }

  return UWBSTATUS_SUCCESS;
}

/******************************************************************************
 * Function         phNxpUciHal_uwb_reset
 *
 * Description      This function will send UWB reset command
 *
 * Returns          status
 *
 ******************************************************************************/
tHAL_UWB_STATUS phNxpUciHal_uwb_reset() {
  tHAL_UWB_STATUS status;
  uint8_t buffer[] = {0x20, 0x00, 0x00, 0x01, 0x00};
  status = phNxpUciHal_send_ext_cmd(sizeof(buffer), buffer);
  if(status != UWBSTATUS_SUCCESS) {
    return status;
  }
  return UWBSTATUS_SUCCESS;
}

/******************************************************************************
 * Function         phNxpUciHal_disable_dpd
 *
 * Description      This function sends disable dpd command
 *
 * Returns          status
 *
 ******************************************************************************/
static tHAL_UWB_STATUS phNxpUciHal_disable_dpd() {
  tHAL_UWB_STATUS status;
  uint8_t buffer[] = {0x20, 0x04, 0x00, 0x04, 0x01, 0x01, 0x01, 0x00};
  status = phNxpUciHal_send_ext_cmd(sizeof(buffer), buffer);
  if (status != UWBSTATUS_SUCCESS) {
    return status;
  }
  return UWBSTATUS_SUCCESS;
}

/******************************************************************************
 * Function         phNxpUciHal_do_bind
 *
 * Description      This function sends bind request command
 *
 * Returns          status
 *
 ******************************************************************************/
static tHAL_UWB_STATUS phNxpUciHal_do_bind() {
  tHAL_UWB_STATUS status;
  uint8_t buffer[] = {0x2F, 0x31, 0x00, 0x00};
  status = phNxpUciHal_send_ext_cmd(sizeof(buffer), buffer);
  if (status != UWBSTATUS_SUCCESS) {
    return status;
  }

  if (phNxpUciHal_sem_timed_wait(&nxpucihal_ctrl.uwb_do_bind_ntf_wait) ||
      nxpucihal_ctrl.uwb_do_bind_ntf_wait.status != UWBSTATUS_SUCCESS) {
    NXPLOG_UCIHAL_E("uwb_do_bind_ntf_wait semaphore timed out");
    return UWBSTATUS_FAILED;
  }
  return UWBSTATUS_SUCCESS;
}

/******************************************************************************
 * Function         phNxpUciHal_get_binding_status
 *
 * Description      This function request for ese binding status
 *
 * Returns          status
 *
 ******************************************************************************/
static tHAL_UWB_STATUS phNxpUciHal_get_binding_status() {
  tHAL_UWB_STATUS status;
  uint8_t lock_cmd[] = {0x2F, 0x32, 0x00, 0x00};
  status = phNxpUciHal_send_ext_cmd(sizeof(lock_cmd), lock_cmd);
  if (status != UWBSTATUS_SUCCESS) {
    return status;
  }
  if (phNxpUciHal_sem_timed_wait(&nxpucihal_ctrl.uwb_get_binding_status_ntf_wait) ||
      nxpucihal_ctrl.uwb_get_binding_status_ntf_wait.status !=
      UWBSTATUS_SUCCESS) {
    NXPLOG_UCIHAL_E("uwb_get_binding_status_ntf_wait semaphore timed out");
    return UWBSTATUS_FAILED;
  }
  return UWBSTATUS_SUCCESS;
}

/******************************************************************************
 * Function         phNxpUciHal_parse_binding_status_ntf
 *
 * Description      This function parses the binding status notification
 *
 * Returns          status
 *
 ******************************************************************************/
static tHAL_UWB_STATUS phNxpUciHal_parse_binding_status_ntf() {
  tHAL_UWB_STATUS status = UWBSTATUS_SUCCESS;
  uint8_t binding_status = nxpucihal_ctrl.uwb_binding_status;
  switch (binding_status) {
  case UWB_DEVICE_NOT_BOUND: {
    /* disable dpd */
    status = phNxpUciHal_disable_dpd();
    if (status != UWBSTATUS_SUCCESS) {
      return status;
    }

    /* perform bind using do_bind command */
    status = phNxpUciHal_do_bind();
    if (status != UWBSTATUS_SUCCESS) {
      return status;
    }

    if (nxpucihal_ctrl.uwb_binding_status == UWB_DEVICE_BOUND_UNLOCKED &&
        nxpucihal_ctrl.uwb_binding_count < 3) {
      /* perform lock using get_binding_status command */
      status = phNxpUciHal_get_binding_status();
      if (status != UWBSTATUS_SUCCESS) {
        return status;
      }
    }
  } break;

  case UWB_DEVICE_BOUND_UNLOCKED: {
    /* disable dpd */
    uint8_t status = phNxpUciHal_disable_dpd();
    if (status != UWBSTATUS_SUCCESS) {
      return status;
    }

    /* perform lock using get_binding_status command */
    status = phNxpUciHal_get_binding_status();
    if (status != UWBSTATUS_SUCCESS) {
      return status;
    }
    if (phNxpUciHal_sem_timed_wait(&nxpucihal_ctrl.uwb_get_binding_status_ntf_wait) ||
        nxpucihal_ctrl.uwb_get_binding_status_ntf_wait.status !=
        UWBSTATUS_SUCCESS) {
      NXPLOG_UCIHAL_E("uwb_get_binding_status_ntf_wait semaphore timed out");
      /* Sending originial binding status notification to upper layer */
      uint8_t data_len = 5;
      uint8_t buffer[data_len];
      buffer[0] = 0x6E;
      buffer[1] = 0x06;
      buffer[2] = 0x00;
      buffer[3] = 0x01;
      buffer[4] = nxpucihal_ctrl.uwb_binding_status;
      if (nxpucihal_ctrl.p_uwb_stack_data_cback != NULL) {
        (*nxpucihal_ctrl.p_uwb_stack_data_cback)(data_len, buffer);
      }
    }
  } break;

  case UWB_DEVICE_BOUND_LOCKED: {
    // do nothing
  } break;

  default: {
    NXPLOG_UCIHAL_E("[%s] unknown binding status:%d", __func__, binding_status);
  }
  }

  return status;
}

static bool phNxpUciHal_getBindingConfig()
{
  uint32_t num = 0;
  bool isBindingLockingAllowed = false;
  NxpConfig_GetNum(NAME_UWB_BINDING_LOCKING_ALLOWED, &num, sizeof(num));
  isBindingLockingAllowed = (bool)num;
  return isBindingLockingAllowed;
}

/******************************************************************************
 * Function         phNxpUciHal_init_hw
 *
 * Description      Init the chip.
 *
 * Returns          status
 *
 ******************************************************************************/
tHAL_UWB_STATUS phNxpUciHal_init_hw()
{
  tHAL_UWB_STATUS status;

  if (nxpucihal_ctrl.halStatus != HAL_STATUS_OPEN) {
    NXPLOG_UCIHAL_E("HAL not initialized");
    return UWBSTATUS_FAILED;
  }

  // Device Status Notification
  std::shared_ptr<phNxpUciHal_RxHandler> dev_status_ntf_handler;
  phNxpUciHal_Sem_t dev_status_ntf_wait;
  phNxpUciHal_init_cb_data(&dev_status_ntf_wait, NULL);
  uint8_t dev_status = UWB_DEVICE_ERROR;
  auto dev_status_ntf_cb = [&dev_status, &dev_status_ntf_wait](size_t packet_len, const uint8_t *packet) mutable {
    if (packet_len >= 5) {
      dev_status = packet[UCI_RESPONSE_STATUS_OFFSET];
      SEM_POST(&dev_status_ntf_wait);
    }
  };

  /* Create the local semaphore */
  if (phNxpUciHal_init_cb_data(&nxpucihal_ctrl.uwb_binding_status_ntf_wait, NULL) != UWBSTATUS_SUCCESS) {
    return UWBSTATUS_FAILED;
  }
  if (phNxpUciHal_init_cb_data(&nxpucihal_ctrl.uwb_get_binding_status_ntf_wait, NULL) != UWBSTATUS_SUCCESS) {
    return UWBSTATUS_FAILED;
  }
  if (phNxpUciHal_init_cb_data(&nxpucihal_ctrl.uwb_do_bind_ntf_wait, NULL) != UWBSTATUS_SUCCESS) {
    return UWBSTATUS_FAILED;
  }

  nxpucihal_ctrl.isRecoveryTimerStarted = false;
  nxpucihal_ctrl.uwb_binding_status = UWB_DEVICE_UNKNOWN;
  nxpucihal_ctrl.fw_dwnld_mode = true; /* system in FW download mode*/
  uwb_device_initialized = false;

  NXPLOG_UCIHAL_D(" Start FW download");

  int fwd_retry_count = FWD_MAX_RETRY_COUNT;
  while (fwd_retry_count--) {
    status = phNxpUciHal_fw_download();
    if (status == UWBSTATUS_SUCCESS)
      break;

    if(status == UWBSTATUS_FILE_NOT_FOUND) {
      NXPLOG_UCIHAL_E("FW download File Not found: status= %x", status);
      break;
    }

    NXPLOG_UCIHAL_E("FW download is failed FW download recovery starts: status= %x", status);
    phTmlUwb_Chip_Reset();
    usleep(5000);
  }

  if (status != UWBSTATUS_SUCCESS)
    goto end_phNxpUciHal_init_hw;

  NXPLOG_UCIHAL_D("Complete FW download");

  dev_status_ntf_handler =
    phNxpUciHal_rx_handler_add(UCI_MT_NTF, UCI_GID_CORE, UCI_MSG_CORE_DEVICE_STATUS_NTF,
                               true, false, dev_status_ntf_cb);

  status = phTmlUwb_Read( Rx_data, UCI_MAX_DATA_LEN,
            (pphTmlUwb_TransactCompletionCb_t)&phNxpUciHal_read_complete, NULL);
  if (status != UWBSTATUS_PENDING) {
    NXPLOG_UCIHAL_E("read status error status = %x", status);
    goto end_phNxpUciHal_init_hw;
  }

  // wait for the first Device Status Notification
  phNxpUciHal_sem_timed_wait(&dev_status_ntf_wait);
  if(dev_status != UWB_DEVICE_INIT) {
    NXPLOG_UCIHAL_E("UWB_DEVICE_INIT not received uwbc_device_state = %x", dev_status);
    goto end_phNxpUciHal_init_hw;
  }

  // Set board-config and wait for Device Status Notification
  status = phNxpUciHal_set_board_config();
  if (status != UWBSTATUS_SUCCESS) {
    NXPLOG_UCIHAL_E("%s: Set Board Config Failed", __func__);
    goto end_phNxpUciHal_init_hw;
  }
  phNxpUciHal_sem_timed_wait(&dev_status_ntf_wait);
  if (dev_status != UWB_DEVICE_READY) {
    NXPLOG_UCIHAL_E("UWB_DEVICE_READY dev_status_ntf_wait semaphore timed out");
    goto end_phNxpUciHal_init_hw;
  }

  // Send SW reset and wait for Device Status Notification
  dev_status = UWB_DEVICE_ERROR;
  status = phNxpUciHal_uwb_reset();
  if (status != UWBSTATUS_SUCCESS) {
    NXPLOG_UCIHAL_E("%s: device reset Failed", __func__);
    goto end_phNxpUciHal_init_hw;
  }
  phNxpUciHal_sem_timed_wait(&dev_status_ntf_wait);
  if(dev_status != UWB_DEVICE_READY) {
    NXPLOG_UCIHAL_E("UWB_DEVICE_READY not received uwbc_device_state = %x", dev_status);
    goto end_phNxpUciHal_init_hw;
  }

  status = phNxpUciHal_sendGetCoreDeviceInfo();
  NXPLOG_UCIHAL_D("phNxpUciHal_sendGetCoreDeviceInfo status %d ", status);
  if (status != UWBSTATUS_SUCCESS) {
    goto end_phNxpUciHal_init_hw;
  }

  phNxpUciHal_sem_timed_wait(&nxpucihal_ctrl.uwb_binding_status_ntf_wait);
  if (nxpucihal_ctrl.uwb_binding_status_ntf_wait.status == UWBSTATUS_SUCCESS) {
    NXPLOG_UCIHAL_D("binding status notification received");
    if (nxpucihal_ctrl.fw_boot_mode == USER_FW_BOOT_MODE) {
      bool isBindingAllowed = phNxpUciHal_getBindingConfig();
      if (isBindingAllowed) {
        status = phNxpUciHal_parse_binding_status_ntf();
        if (status != UWBSTATUS_SUCCESS) {
          NXPLOG_UCIHAL_E("binding failed with status %d", status);
        }
      }
    }
  } else {
    NXPLOG_UCIHAL_D("%s:binding status notification timed out", __func__);
  }

  status = phNxpUciHal_applyVendorConfig();
  if (status != UWBSTATUS_SUCCESS) {
    NXPLOG_UCIHAL_E("%s: Apply vendor Config Failed", __func__);
    goto end_phNxpUciHal_init_hw;
  }
  phNxpUciHal_extcal_handle_coreinit();

  uwb_device_initialized = true;
  phNxpUciHal_getVersionInfo();

end_phNxpUciHal_init_hw:
  phNxpUciHal_rx_handler_del(dev_status_ntf_handler);
  phNxpUciHal_cleanup_cb_data(&dev_status_ntf_wait);
  phNxpUciHal_cleanup_cb_data(
      &nxpucihal_ctrl.uwb_get_binding_status_ntf_wait);
  phNxpUciHal_cleanup_cb_data(&nxpucihal_ctrl.uwb_do_bind_ntf_wait);
  phNxpUciHal_cleanup_cb_data(&nxpucihal_ctrl.uwb_binding_status_ntf_wait);
  return status;
}

/******************************************************************************
 * Function         phNxpUciHal_coreInitialization
 *
 * Description      This function performs core initialization
 *
 * Returns          status
 *
 ******************************************************************************/
tHAL_UWB_STATUS phNxpUciHal_coreInitialization()
{
  tHAL_UWB_STATUS status = phNxpUciHal_init_hw();
  if (status != UWBSTATUS_SUCCESS) {
    phTmlUwb_DeferredCall(std::make_shared<phLibUwb_Message>(UCI_HAL_ERROR_MSG));
    return status;
  }

  SessionTrack_init();

  // report to upper-layer
  phTmlUwb_DeferredCall(std::make_shared<phLibUwb_Message>(UCI_HAL_INIT_CPLT_MSG));

  if (nxpucihal_ctrl.p_uwb_stack_data_cback != NULL) {
    uint8_t dev_ready_ntf[] = {0x60, 0x01, 0x00, 0x01, 0x01};
    (*nxpucihal_ctrl.p_uwb_stack_data_cback)((sizeof(dev_ready_ntf)/sizeof(uint8_t)), dev_ready_ntf);
  }

  return UWBSTATUS_SUCCESS;
}

/******************************************************************************
 * Function         phNxpUciHal_sessionInitialization
 *
 * Description      This function performs session initialization
 *
 * Returns          status
 *
 ******************************************************************************/
tHAL_UWB_STATUS phNxpUciHal_sessionInitialization(uint32_t sessionId) {
  NXPLOG_UCIHAL_D(" %s: Enter", __func__);
  std::array<uint8_t, NXP_MAX_CONFIG_STRING_LEN> buffer;
  uint8_t vendorConfig[NXP_MAX_CONFIG_STRING_LEN] = {0x2F, 0x00, 0x00};
  tHAL_UWB_STATUS status = UWBSTATUS_SUCCESS;
  buffer.fill(0);
  int max_config_length = NXP_MAX_CONFIG_STRING_LEN - UCI_MSG_HDR_SIZE
                            - sizeof(sessionId);
  long retlen = 0, cmdlen = 0;
  bool appConfigStatus = false;

  if (nxpucihal_ctrl.halStatus != HAL_STATUS_OPEN) {
    NXPLOG_UCIHAL_E("HAL not initialized");
    return UWBSTATUS_FAILED;
  }
  if(nxpucihal_ctrl.device_type == DEVICE_TYPE_SR1xxT) {
    appConfigStatus = NxpConfig_GetByteArray(NAME_NXP_UWB_EXT_APP_SR1XX_T_CONFIG,
                                   buffer.data(), buffer.size(),
                                   &retlen);
  } else if (nxpucihal_ctrl.device_type == DEVICE_TYPE_SR1xxS) {
    appConfigStatus = NxpConfig_GetByteArray(NAME_NXP_UWB_EXT_APP_SR1XX_S_CONFIG,
                                   buffer.data(), buffer.size(),
                                   &retlen);
  } else {
    appConfigStatus = NxpConfig_GetByteArray(NAME_NXP_UWB_EXT_APP_DEFAULT_CONFIG,
                                   buffer.data(), buffer.size(),
                                   &retlen);
  }

  if (appConfigStatus) {
    if ((retlen > 0) && (retlen <= max_config_length)) {
      vendorConfig[3] = sizeof(sessionId) + retlen;
      memcpy(vendorConfig + 4, &sessionId, sizeof(sessionId));
      memcpy(vendorConfig + 8, buffer.data(), retlen);
      cmdlen = UCI_MSG_HDR_SIZE + sizeof(sessionId) + retlen;
      status = phNxpUciHal_send_ext_cmd(cmdlen, vendorConfig);
      if (status != UWBSTATUS_SUCCESS) {
        NXPLOG_UCIHAL_D(" %s: Apply vendor App Config Failed", __func__);
        return UWBSTATUS_SUCCESS;
      }
    } else {
      NXPLOG_UCIHAL_D(" %s: Invalid retlen", __func__);
      return UWBSTATUS_SUCCESS;
    }
  }
  return status;
}

/******************************************************************************
 * Function         phNxpUciHal_GetMwVersion
 *
 * Description      This function gets the middleware version
 *
 * Returns          phNxpUciHal_MW_Version_t
 *
 ******************************************************************************/
phNxpUciHal_MW_Version_t phNxpUciHal_GetMwVersion() {
  phNxpUciHal_MW_Version_t mwVer;
  mwVer.validation = NXP_CHIP_SR100;
  mwVer.android_version = NXP_ANDROID_VERSION;
  NXPLOG_UCIHAL_D("0x%x:UWB MW Major Version:", UWB_NXP_MW_VERSION_MAJ);
  NXPLOG_UCIHAL_D("0x%x:UWB MW Minor Version:", UWB_NXP_MW_VERSION_MIN);
  mwVer.major_version = UWB_NXP_MW_VERSION_MAJ;
  mwVer.minor_version = UWB_NXP_MW_VERSION_MIN;
  mwVer.rc_version = UWB_NXP_ANDROID_MW_RC_VERSION;
  mwVer.mw_drop = UWB_NXP_ANDROID_MW_DROP_VERSION;
  return mwVer;
}

/******************************************************************************
 * Function         phNxpUciHal_getVersionInfo
 *
 * Description      This function request for version information
 *
 * Returns          void
 *
 ******************************************************************************/
void phNxpUciHal_getVersionInfo() {
  phNxpUciHal_MW_Version_t mwVersion = phNxpUciHal_GetMwVersion();
  if (mwVersion.rc_version) { /* for RC release*/
    ALOGI("MW Version: UWB_SW_Android_U_HKY_D%02x.%02x_RC%02x",
          mwVersion.major_version, mwVersion.minor_version,
          mwVersion.rc_version);
  } else if (mwVersion.mw_drop) { /* For Drops */
    ALOGI("MW Version: UWB_SW_Android_U_HKY_D%02x.%02x_DROP%02x",
          mwVersion.major_version, mwVersion.minor_version, mwVersion.mw_drop);
  } else { /* for Major Releases*/
    ALOGI("MW Version: UWB_SW_Android_U_HKY_D%02x.%02x",
          mwVersion.major_version, mwVersion.minor_version);
  }

  if (nxpucihal_ctrl.fw_version.rc_version) {
    ALOGI("FW Version: %02x.%02x_RC%02x", nxpucihal_ctrl.fw_version.major_version,
          nxpucihal_ctrl.fw_version.minor_version, nxpucihal_ctrl.fw_version.rc_version);
  } else {
    ALOGI("FW Version: %02x.%02x", nxpucihal_ctrl.fw_version.major_version,
          nxpucihal_ctrl.fw_version.minor_version);
  }
}
