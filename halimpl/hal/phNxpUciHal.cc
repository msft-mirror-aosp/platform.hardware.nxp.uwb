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

/*********************** Global Variables *************************************/
/* UCI HAL Control structure */
phNxpUciHal_Control_t nxpucihal_ctrl;

bool uwb_device_initialized = false;
bool uwb_get_platform_id = false;
uint32_t timeoutTimerId = 0;
char persistant_log_path[120];
static uint8_t Rx_data[UCI_MAX_DATA_LEN];

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

  // Per-chip (SR1XX or SR200) implementation
  nxpucihal_ctrl.uwb_chip = GetUwbChip();

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
    NXPLOG_UCIHAL_V("write successful status = 0x%x", pInfo->wStatus);
  } else {
    NXPLOG_UCIHAL_E("write error status = 0x%x", pInfo->wStatus);
  }
  p_cb_data->status = pInfo->wStatus;

  SEM_POST(p_cb_data);

  return;
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
void phNxpUciHal_read_complete(void* pContext, phTmlUwb_TransactInfo_t* pInfo)
{
  UNUSED(pContext);

  if (pInfo->wStatus != UWBSTATUS_SUCCESS) {
    NXPLOG_UCIHAL_E("read error status = 0x%x", pInfo->wStatus);
    return;
  }

  NXPLOG_UCIHAL_D("read successful status = 0x%x", pInfo->wStatus);

  for (int32_t index = 0; index < pInfo->wLength; )
  {
    uint8_t extBitSet = (pInfo->pBuff[index + EXTND_LEN_INDICATOR_OFFSET] & EXTND_LEN_INDICATOR_OFFSET_MASK);
    int32_t length = pInfo->pBuff[index + NORMAL_MODE_LENGTH_OFFSET];
    if (extBitSet || ((pInfo->pBuff[index] & UCI_MT_MASK) == 0x00)) {
     length = (length << EXTENDED_MODE_LEN_SHIFT) | pInfo->pBuff[index + EXTENDED_MODE_LEN_OFFSET] ;
    }
    length += UCI_MSG_HDR_SIZE;
    NXPLOG_UCIHAL_V("read successful length = %d", length);

    nxpucihal_ctrl.p_rx_data = &pInfo->pBuff[index];
    nxpucihal_ctrl.rx_data_len = length;
    phNxpUciHal_print_packet(NXP_TML_UCI_RSP_NTF_UWBS_2_AP, nxpucihal_ctrl.p_rx_data, nxpucihal_ctrl.rx_data_len);

    uint8_t mt = ((nxpucihal_ctrl.p_rx_data[0]) & UCI_MT_MASK) >> UCI_MT_SHIFT;
    uint8_t gid = nxpucihal_ctrl.p_rx_data[0] & UCI_GID_MASK;
    uint8_t oid = nxpucihal_ctrl.p_rx_data[1] & UCI_OID_MASK;
    uint8_t pbf = (nxpucihal_ctrl.p_rx_data[0] & UCI_PBF_MASK) >> UCI_PBF_SHIFT;

    nxpucihal_ctrl.isSkipPacket = 0;

    phNxpUciHal_rx_handler_check(pInfo->wLength, pInfo->pBuff);

    // mapping device caps according to Fira 2.0
    if (mt == UCI_MT_RSP && gid == UCI_GID_CORE && oid == UCI_MSG_CORE_GET_CAPS_INFO) {
      phNxpUciHal_handle_get_caps_info(nxpucihal_ctrl.rx_data_len, nxpucihal_ctrl.p_rx_data);
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

    /* Disable junk data check for each UCI packet*/
    if(nxpucihal_ctrl.fw_dwnld_mode) {
      if((gid == UCI_GID_CORE) && (oid == UCI_MSG_CORE_DEVICE_STATUS_NTF)){
        nxpucihal_ctrl.fw_dwnld_mode = false;
      }
    }
    index += length;
  } //End of loop
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

  NXPLOG_UCIHAL_D("Terminating phNxpUciHal client thread...");
  phTmlUwb_DeferredCall(std::make_shared<phLibUwb_Message>(UCI_HAL_CLOSE_CPLT_MSG));
  nxpucihal_ctrl.client_thread.join();

  status = phTmlUwb_Shutdown();

  phNxpUciHal_rx_handler_destroy();

  nxpucihal_ctrl.halStatus = HAL_STATUS_CLOSE;

  CONCURRENCY_UNLOCK();

  nxpucihal_ctrl.uwb_chip.reset();

  phOsalUwb_Timer_Cleanup();

  phNxpUciHal_cleanup_monitor();

  NxpConfig_Deinit();

  NXPLOG_UCIHAL_D("phNxpUciHal_close completed");

  /* Return success always */
  return UWBSTATUS_SUCCESS;
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
static void parseAntennaConfig(const char *configName)
{
  std::array<uint8_t, NXP_MAX_CONFIG_STRING_LEN> buffer;
  long retlen = 0;
  int gotConfig = NxpConfig_GetByteArray(configName, buffer.data(), buffer.size(), &retlen);
  if (gotConfig) {
    if (retlen <= UCI_MSG_HDR_SIZE) {
      NXPLOG_UCIHAL_E("parseAntennaConfig: %s is too short. Aborting.", configName);
      return;
    }
  }
  else
  {
    NXPLOG_UCIHAL_E("parseAntennaConfig: Failed to get '%s'. Aborting.", configName);
    return;
  }

  const uint16_t dataLength = retlen;
  const uint8_t *data = buffer.data();

  uint8_t index = UCI_MSG_HDR_SIZE + 1; // Excluding the header and number of params
  uint8_t tagId, subTagId;
  int length;
  while (index < dataLength) {
    tagId = data[index++];
    subTagId = data[index++];
    length = data[index++];
    if ((ANTENNA_RX_PAIR_DEFINE_TAG_ID == tagId) &&
        (ANTENNA_RX_PAIR_DEFINE_SUB_TAG_ID == subTagId)) {
      nxpucihal_ctrl.numberOfAntennaPairs = data[index];
      NXPLOG_UCIHAL_D("numberOfAntennaPairs:%d", nxpucihal_ctrl.numberOfAntennaPairs);
      break;
    } else {
      index = index + length;
    }
  }
}

/******************************************************************************
 * Function         phNxpUciHal_applyVendorConfig
 *
 * Description      This function applies the vendor config from config file
 *
 * Returns          status
 *
 ******************************************************************************/
tHAL_UWB_STATUS phNxpUciHal_applyVendorConfig()
{
  std::vector<const char*> vendorParamNames;

  // Base parameter names
  if (nxpucihal_ctrl.fw_boot_mode == USER_FW_BOOT_MODE) {
    vendorParamNames.push_back(NAME_UWB_USER_FW_BOOT_MODE_CONFIG);
  }
  vendorParamNames.push_back(NAME_NXP_UWB_EXTENDED_NTF_CONFIG);

  // Chip parameter names
  const char *per_chip_param = NAME_UWB_CORE_EXT_DEVICE_DEFAULT_CONFIG;
  if (nxpucihal_ctrl.device_type == DEVICE_TYPE_SR1xxT) {
    per_chip_param = NAME_UWB_CORE_EXT_DEVICE_SR1XX_T_CONFIG;
  } else if (nxpucihal_ctrl.device_type == DEVICE_TYPE_SR1xxS) {
    per_chip_param = NAME_UWB_CORE_EXT_DEVICE_SR1XX_S_CONFIG;
  }
  vendorParamNames.push_back(per_chip_param);

  // Parse Antenna config from chip-parameter
  parseAntennaConfig(per_chip_param);

  // Extra parameter names, XTAL, NXP_CORE_CONF_BLK[1..10]
  vendorParamNames.push_back(NAME_NXP_UWB_XTAL_38MHZ_CONFIG);
  vendorParamNames.push_back(NAME_NXP_CORE_CONF_BLK "1");
  vendorParamNames.push_back(NAME_NXP_CORE_CONF_BLK "2");
  vendorParamNames.push_back(NAME_NXP_CORE_CONF_BLK "3");
  vendorParamNames.push_back(NAME_NXP_CORE_CONF_BLK "4");
  vendorParamNames.push_back(NAME_NXP_CORE_CONF_BLK "5");
  vendorParamNames.push_back(NAME_NXP_CORE_CONF_BLK "6");
  vendorParamNames.push_back(NAME_NXP_CORE_CONF_BLK "7");
  vendorParamNames.push_back(NAME_NXP_CORE_CONF_BLK "8");
  vendorParamNames.push_back(NAME_NXP_CORE_CONF_BLK "9");
  vendorParamNames.push_back(NAME_NXP_CORE_CONF_BLK "10");

  // Execute
  for (const auto paramName : vendorParamNames) {
    std::array<uint8_t, NXP_MAX_CONFIG_STRING_LEN> buffer;
    long retlen = 0;
    if (NxpConfig_GetByteArray(paramName, buffer.data(), buffer.size(), &retlen)) {
      if (retlen > 0 && retlen < UCI_MAX_DATA_LEN) {
        NXPLOG_UCIHAL_D("VendorConfig: apply %s", paramName);
        tHAL_UWB_STATUS status = phNxpUciHal_send_ext_cmd(retlen, buffer.data());
        if (status != UWBSTATUS_SUCCESS) {
          NXPLOG_UCIHAL_E("VendorConfig: failed to apply %s", paramName);
          return status;
        }
      }
    }
  }

  // Low Power Mode
  // TODO: remove this out, this can be move to Chip parameter names
  uint8_t lowPowerMode = 0;
  if (NxpConfig_GetNum(NAME_NXP_UWB_LOW_POWER_MODE, &lowPowerMode, sizeof(lowPowerMode))) {
    NXPLOG_UCIHAL_D("VendorConfig: apply %s", NAME_NXP_UWB_LOW_POWER_MODE);

    // Core set config packet: GID=0x00 OID=0x04
    const std::vector<uint8_t> packet(
        {((UCI_MT_CMD << UCI_MT_SHIFT) | UCI_GID_CORE), UCI_MSG_CORE_SET_CONFIG,
         0x00, 0x04, 0x01, LOW_POWER_MODE_TAG_ID, LOW_POWER_MODE_LENGTH,
         lowPowerMode});

    if (phNxpUciHal_send_ext_cmd(packet.size(), packet.data()) != UWBSTATUS_SUCCESS) {
      NXPLOG_UCIHAL_E("VendorConfig: failed to apply NAME_NXP_UWB_LOW_POWER_MODE");
    }
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

static bool cacheDevInfoRsp()
{
  auto dev_info_cb = [](size_t packet_len, const uint8_t *packet) mutable {
    if (packet_len < 5 || packet[UCI_RESPONSE_STATUS_OFFSET] != UWBSTATUS_SUCCESS) {
      NXPLOG_UCIHAL_E("Failed to get valid CORE_DEVICE_INFO_RSP");
      return;
    }
    if (packet_len > sizeof(nxpucihal_ctrl.dev_info_resp)) {
      NXPLOG_UCIHAL_E("FIXME: CORE_DEVICE_INFO_RSP buffer overflow!");
      return;
    }

    // FIRA UCIv2.0 packet size = 14
    // [13] = Vendor Specific Info Length
    constexpr uint8_t firaDevInfoRspSize = 14;
    constexpr uint8_t firaDevInfoVendorLenOffset = 13;

    if (packet_len < firaDevInfoRspSize) {
      NXPLOG_UCIHAL_E("DEVICE_INFO_RSP packet size mismatched.");
      return;
    }

    const uint8_t vendorSpecificLen = packet[firaDevInfoVendorLenOffset];
    if (packet_len != (firaDevInfoRspSize + vendorSpecificLen)) {
      NXPLOG_UCIHAL_E("DEVICE_INFO_RSP packet size mismatched.");
    }

    for (uint8_t i = firaDevInfoRspSize; (i + 2) <= packet_len; ) {
      uint8_t paramId = packet[i++];
      uint8_t length = packet[i++];

      if (i + length > packet_len)
        break;

      if (paramId == DEVICE_NAME_PARAM_ID && length >= 6) {
        nxpucihal_ctrl.device_type = nxpucihal_ctrl.uwb_chip->get_device_type(&packet[i], length);
      } else if (paramId == FW_VERSION_PARAM_ID && length >= 3) {
        nxpucihal_ctrl.fw_version.major_version = packet[i];
        nxpucihal_ctrl.fw_version.minor_version = packet[i + 1];
        nxpucihal_ctrl.fw_version.rc_version = packet[i + 2];
      } else if (paramId == FW_BOOT_MODE_PARAM_ID && length >= 1) {
        nxpucihal_ctrl.fw_boot_mode = packet[i];
      }
      i += length;
    }
    memcpy(nxpucihal_ctrl.dev_info_resp, packet, packet_len);
    nxpucihal_ctrl.isDevInfoCached = true;
    NXPLOG_UCIHAL_D("Device Info cached.");
  };

  nxpucihal_ctrl.isDevInfoCached = false;
  UciHalRxHandler devInfoRspHandler(UCI_MT_RSP, UCI_GID_CORE, UCI_MSG_CORE_DEVICE_INFO, true, dev_info_cb);

  const uint8_t CoreGetDevInfoCmd[] = {(UCI_MT_CMD << UCI_MT_SHIFT) | UCI_GID_CORE, UCI_MSG_CORE_DEVICE_INFO, 0, 0};
  tHAL_UWB_STATUS status = phNxpUciHal_send_ext_cmd(sizeof(CoreGetDevInfoCmd), CoreGetDevInfoCmd);
  if (status != UWBSTATUS_SUCCESS) {
    return false;
  }
  return true;
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

  uwb_device_initialized = false;

  // FW download and enter UCI operating mode
  status = nxpucihal_ctrl.uwb_chip->chip_init();
  if (status != UWBSTATUS_SUCCESS) {
    return status;
  }

  // Device Status Notification
  UciHalSemaphore devStatusNtfWait;
  uint8_t dev_status = UWB_DEVICE_ERROR;
  auto dev_status_ntf_cb = [&dev_status, &devStatusNtfWait](size_t packet_len, const uint8_t *packet) mutable {
    if (packet_len >= 5) {
      dev_status = packet[UCI_RESPONSE_STATUS_OFFSET];
      devStatusNtfWait.post();
    }
  };
  UciHalRxHandler devStatusNtfHandler(UCI_MT_NTF, UCI_GID_CORE, UCI_MSG_CORE_DEVICE_STATUS_NTF,
                                      true, dev_status_ntf_cb);

  // Initiate UCI packet read
  status = phTmlUwb_StartRead( Rx_data, UCI_MAX_DATA_LEN,
            (pphTmlUwb_TransactCompletionCb_t)&phNxpUciHal_read_complete, NULL);
  if (status != UWBSTATUS_SUCCESS) {
    NXPLOG_UCIHAL_E("read status error status = %x", status);
    return status;
  }

  // Wait for the first Device Status Notification
  devStatusNtfWait.wait();
  if(dev_status != UWB_DEVICE_INIT) {
    NXPLOG_UCIHAL_E("UWB_DEVICE_INIT not received uwbc_device_state = %x", dev_status);
    return UWBSTATUS_FAILED;
  }

  // Set board-config and wait for Device Status Notification
  status = phNxpUciHal_set_board_config();
  if (status != UWBSTATUS_SUCCESS) {
    NXPLOG_UCIHAL_E("%s: Set Board Config Failed", __func__);
    return status;
  }
  devStatusNtfWait.wait();
  if (dev_status != UWB_DEVICE_READY) {
    NXPLOG_UCIHAL_E("Cannot receive UWB_DEVICE_READY");
    return UWBSTATUS_FAILED;
  }

  // Send SW reset and wait for Device Status Notification
  dev_status = UWB_DEVICE_ERROR;
  status = phNxpUciHal_uwb_reset();
  if (status != UWBSTATUS_SUCCESS) {
    NXPLOG_UCIHAL_E("%s: device reset Failed", __func__);
    return status;
  }
  devStatusNtfWait.wait();
  if(dev_status != UWB_DEVICE_READY) {
    NXPLOG_UCIHAL_E("UWB_DEVICE_READY not received uwbc_device_state = %x", dev_status);
    return UWBSTATUS_FAILED;
  }

  // Cache CORE_GET_DEVICE_INFO
  cacheDevInfoRsp();

  status = nxpucihal_ctrl.uwb_chip->core_init();
  if (status != UWBSTATUS_SUCCESS) {
    return status;
  }

  status = phNxpUciHal_applyVendorConfig();
  if (status != UWBSTATUS_SUCCESS) {
    NXPLOG_UCIHAL_E("%s: Apply vendor Config Failed", __func__);
    return status;
  }
  phNxpUciHal_extcal_handle_coreinit();

  uwb_device_initialized = true;
  phNxpUciHal_getVersionInfo();

  return UWBSTATUS_SUCCESS;
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

/*******************************************************************************
 * Function      phNxpUciHal_send_dev_error_status_ntf
 *
 * Description   send device status notification. Upper layer might restart
 *               HAL service.
 *
 * Returns       void
 *
 ******************************************* ***********************************/
void phNxpUciHal_send_dev_error_status_ntf()
{
 NXPLOG_UCIHAL_D("phNxpUciHal_send_dev_error_status_ntf ");
 nxpucihal_ctrl.rx_data_len = 5;
 static uint8_t rsp_data[5] = {0x60, 0x01, 0x00, 0x01, 0xFF};
 (*nxpucihal_ctrl.p_uwb_stack_data_cback)(nxpucihal_ctrl.rx_data_len, rsp_data);
}
