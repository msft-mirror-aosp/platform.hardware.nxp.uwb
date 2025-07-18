/*
 * Copyright 2012-2019, 2022-2024 NXP
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
#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <span>
#include <string>
#include <thread>
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

bool uwb_get_platform_id = false;
uint32_t timeoutTimerId = 0;
char persistant_log_path[120];
constexpr long HAL_WRITE_TIMEOUT_MS = 1000;
/**************** local methods used in this file only ************************/
static void phNxpUciHal_write_complete(void* pContext, phTmlUwb_WriteTransactInfo* pInfo);
extern int phNxpUciHal_fw_download();
static void phNxpUciHal_getVersionInfo();
static tHAL_UWB_STATUS phNxpUciHal_sendCoreConfig(const uint8_t *p_cmd,
                                                  long buffer_size);

/*******************************************************************************
 * RX packet handler
 ******************************************************************************/
struct phNxpUciHal_RxHandler {
  phNxpUciHal_RxHandler(uint8_t mt, uint8_t gid, uint8_t oid,
    bool run_once, RxHandlerCallback callback) :
      mt(mt), gid(gid), oid(oid),
      run_once(run_once),
      callback(callback) { }

  // mt, gid, oid: packet type
  uint8_t mt;
  uint8_t gid;
  uint8_t oid;
  bool run_once;
  RxHandlerCallback callback;
};

static std::list<std::shared_ptr<phNxpUciHal_RxHandler>> rx_handlers;
static std::mutex rx_handlers_lock;

std::shared_ptr<phNxpUciHal_RxHandler> phNxpUciHal_rx_handler_add(
  uint8_t mt, uint8_t gid, uint8_t oid,
  bool run_once,
  RxHandlerCallback callback)
{
  auto handler = std::make_shared<phNxpUciHal_RxHandler>(
    mt, gid, oid, run_once, callback);
  std::lock_guard<std::mutex> guard(rx_handlers_lock);
  rx_handlers.push_back(handler);
  return handler;
}

void phNxpUciHal_rx_handler_del(std::shared_ptr<phNxpUciHal_RxHandler> handler)
{
  std::lock_guard<std::mutex> guard(rx_handlers_lock);
  rx_handlers.remove(handler);
}

// Returns true when this packet is handled by one of the handler.
static bool phNxpUciHal_rx_handler_check(size_t packet_len, const uint8_t *packet)
{
  const uint8_t mt = ((packet[0]) & UCI_MT_MASK) >> UCI_MT_SHIFT;
  const uint8_t gid = packet[0] & UCI_GID_MASK;
  const uint8_t oid = packet[1] & UCI_OID_MASK;
  bool skip_packet = false;

  // Copy the whole list to allow rx handlers to call rx_handler_add().
  std::list<std::shared_ptr<phNxpUciHal_RxHandler>> handlers;
  {
    std::lock_guard<std::mutex> guard(rx_handlers_lock);
    handlers = rx_handlers;
  }

  for (auto handler : handlers) {
    if (mt == handler->mt && gid == handler->gid && oid == handler->oid) {
      if (handler->callback(packet_len, packet)) {
        skip_packet = true;
      }
    }
  }

  std::lock_guard<std::mutex> guard(rx_handlers_lock);
  rx_handlers.remove_if([mt, gid, oid](auto& handler) {
    return mt == handler->mt && gid == handler->gid && oid == handler->oid && handler->run_once;
  });

  return skip_packet;
}

static void phNxpUciHal_rx_handler_destroy(void)
{
  std::lock_guard<std::mutex> guard(rx_handlers_lock);
  rx_handlers.clear();
}


bool  nxp_properitory_ntf_skip_cb(size_t data_len, const uint8_t *p_data) {
  bool is_handled = false;
  const uint8_t mt = (p_data[0] & UCI_MT_MASK) >> UCI_MT_SHIFT;
  const uint8_t gid = p_data[0] & UCI_GID_MASK;
  const uint8_t oid = p_data[1] & UCI_OID_MASK;
  if (mt == UCI_MT_NTF) { // must be true.
    if (gid == UCI_GID_PROPRIETARY
      && oid == EXT_UCI_PROP_GEN_DEBUG_NTF_0x18
      && data_len == 9
      && p_data[4] == 0x07
      && p_data[5] == 0x29
      && p_data[6] == 0x01
    ) {
      //  0  1  2  3  4  5  6  7  8
      // 6E 18 00 05 07 29 01 00 64.
      // b/381330041
      NXPLOG_UCIHAL_D("%s: Skip 6E180015072901.... packet", __FUNCTION__);
      is_handled = true;
    }
  }
  else
  {
    // Not possible. We registered only for NTF
    NXPLOG_UCIHAL_E("%s: Wrong MT: %d", __FUNCTION__, mt);
  }
  return is_handled;
};

bool phNxpUciHal_handle_dev_error_ntf(size_t packet_len,
                                      const uint8_t *packet) {
  if (packet_len > UCI_RESPONSE_STATUS_OFFSET) {
    if (UWBS_STATUS_ERROR == packet[UCI_RESPONSE_STATUS_OFFSET]) {
      if (nxpucihal_ctrl.recovery_ongoing == true) {
        NXPLOG_UCIHAL_D("Fw crashed during recovery, ignore packet");
      } else {
        nxpucihal_ctrl.recovery_ongoing = true;
        phNxpUciHalProp_trigger_fw_crash_log_dump();
      }
      return true;
    }
  } else {
    NXPLOG_UCIHAL_E("[%s] Invalid packet length: %d", __func__, packet_len);
  }
  return false;
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
    auto msg = p_nxpucihal_ctrl->pClientMq->recv();

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
bool phNxpUciHal_parse(size_t* cmdlen, uint8_t* cmd)
{
  bool ret = false;

  if ((*cmdlen) < UCI_MSG_HDR_SIZE) {
    return false;
  }

  const uint8_t mt = (cmd[0] &UCI_MT_MASK) >> UCI_MT_SHIFT;
  const uint8_t gid = cmd[0] & UCI_GID_MASK;
  const uint8_t oid = cmd[1] & UCI_OID_MASK;
  if (mt != UCI_MT_CMD) {
    return false;
  }

  if ((gid == UCI_GID_ANDROID) && (oid == UCI_MSG_ANDROID_SET_COUNTRY_CODE)) {
    char country_code[2];
    if ((*cmdlen) == 6) {
      country_code[0] = (char)cmd[4];
      country_code[1] = (char)cmd[5];
    } else {
      NXPLOG_UCIHAL_E("Unexpected payload length for ANDROID_SET_COUNTRY_CODE, handle this with 00 country code");
      country_code[0] = '0';
      country_code[1] = '0';
    }
    phNxpUciHal_handle_set_country_code(country_code);
    return true;
  } else if ((gid == UCI_GID_PROPRIETARY_0x0F) && (oid == SET_VENDOR_SET_CALIBRATION)) {
    if (cmd[UCI_MSG_HDR_SIZE + 1] == VENDOR_CALIB_PARAM_TX_POWER_PER_ANTENNA) {
      // XXX: packet can be patched by here.
      phNxpUciHal_handle_set_calibration(cmd, *cmdlen);
    }
  } else if ((gid == UCI_GID_SESSION_MANAGE) && (oid == UCI_MSG_SESSION_SET_APP_CONFIG)) {
    // XXX: packet can be patched by here.
    return phNxpUciHal_handle_set_app_config(cmdlen, cmd);
  } else if ((gid == UCI_GID_SESSION_MANAGE) && (oid == UCI_MSG_SESSION_STATE_INIT)) {
    SessionTrack_onSessionInit(*cmdlen, cmd);
  } if (mt == UCI_MT_CMD && gid == UCI_GID_SESSION_CONTROL && oid == UCI_MSG_SESSION_START) {
    SessionTrack_onSessionStart(*cmdlen, cmd);
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
  tHAL_UWB_STATUS wConfigStatus = UWBSTATUS_SUCCESS;

  if (nxpucihal_ctrl.halStatus == HAL_STATUS_OPEN) {
    NXPLOG_UCIHAL_E("phNxpUciHal_open already open");
    return UWBSTATUS_SUCCESS;
  }

  NxpConfig_Init();

  /* initialize trace level */
  phNxpLog_InitializeLogLevel();
  phNxpUciLog_initialize();

  /*Create the timer for extns write response*/
  timeoutTimerId = phOsalUwb_Timer_Create();

  if (!phNxpUciHal_init_monitor()) {
    NXPLOG_UCIHAL_E("Init monitor failed");
    return UWBSTATUS_FAILED;
  }

  CONCURRENCY_LOCK();

  NXPLOG_UCIHAL_D("Assigning the default helios Node: %s", uwb_dev_node);
  /* By default HAL status is HAL_STATUS_OPEN */
  nxpucihal_ctrl.halStatus = HAL_STATUS_OPEN;

  nxpucihal_ctrl.p_uwb_stack_cback = p_cback;
  nxpucihal_ctrl.p_uwb_stack_data_cback = p_data_cback;
  nxpucihal_ctrl.fw_dwnld_mode = false;

  // Create a main message queue.
  nxpucihal_ctrl.pClientMq = std::make_shared<MessageQueue<phLibUwb_Message>>("Client");

  // Default country code = '00'
  nxpucihal_ctrl.country_code[0] = '0';
  nxpucihal_ctrl.country_code[1] = '0';

  /* Create the client thread */
  nxpucihal_ctrl.client_thread =
    std::thread{ &phNxpUciHal_client_thread, &nxpucihal_ctrl };

  nxpucihal_ctrl.halStatus = HAL_STATUS_OPEN;

  CONCURRENCY_UNLOCK();

  // Per-chip (SR1XX or SR200) implementation
  nxpucihal_ctrl.uwb_chip = GetUwbChip();

  // Install rx packet handlers
  phNxpUciHal_rx_handler_add(UCI_MT_RSP, UCI_GID_CORE, UCI_MSG_CORE_GET_CAPS_INFO,
    false, phNxpUciHal_handle_get_caps_info);


  phNxpUciHal_rx_handler_add(UCI_MT_NTF, UCI_GID_PROPRIETARY, EXT_UCI_PROP_GEN_DEBUG_NTF_0x18, false,
                                      nxp_properitory_ntf_skip_cb);

  phNxpUciHal_rx_handler_add(UCI_MT_NTF, UCI_GID_CORE,
                             UCI_MSG_CORE_DEVICE_STATUS_NTF, false,
                             phNxpUciHal_handle_dev_error_ntf);

  /* Call open complete */
  nxpucihal_ctrl.pClientMq->send(std::make_shared<phLibUwb_Message>(UCI_HAL_OPEN_CPLT_MSG));

  return UWBSTATUS_SUCCESS;
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
int32_t phNxpUciHal_write(size_t data_len, const uint8_t* p_data) {
  if (nxpucihal_ctrl.halStatus != HAL_STATUS_OPEN) {
    return UWBSTATUS_FAILED;
  }
  SessionTrack_keepAlive();

  CONCURRENCY_LOCK();
  auto status = phNxpUciHal_process_ext_cmd_rsp(data_len, p_data);
  CONCURRENCY_UNLOCK();

  /* No data written */
  return (status == UWBSTATUS_SUCCESS) ? data_len : 0;
}

/******************************************************************************
 * Function         phNxpUciHal_write_unlocked
 *
 * Description      This is the actual function which is being called by
 *                  phNxpUciHal_write. This function writes the data to UWBC.
 *                  It waits till write callback provide the result of write
 *                  process.
 *
 * Returns          Status code.
 *
 ******************************************************************************/
tHAL_UWB_STATUS phNxpUciHal_write_unlocked(size_t data_len, const uint8_t* p_data) {
  tHAL_UWB_STATUS status;

  if ((data_len > UCI_MAX_DATA_LEN) || (data_len < UCI_PKT_HDR_LEN)) {
    NXPLOG_UCIHAL_E("Invalid data_len");
    return UWBSTATUS_INVALID_PARAMETER;
  }

  /* Create the local semaphore */
  UciHalSemaphore cb_data;

  status = phTmlUwb_Write(p_data, data_len, phNxpUciHal_write_complete, &cb_data);

  if (status != UWBSTATUS_PENDING) {
    return UWBSTATUS_FAILED;
  }

  /* Wait for callback response */
  if (cb_data.wait_timeout_msec(HAL_WRITE_TIMEOUT_MS)) {
    NXPLOG_UCIHAL_E("write_unlocked semaphore error");
    return UWBSTATUS_FAILED;
  }

  return UWBSTATUS_SUCCESS;
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
                                       phTmlUwb_WriteTransactInfo* pInfo) {
  UciHalSemaphore* p_cb_data = (UciHalSemaphore*)pContext;

  if (pInfo->wStatus == UWBSTATUS_SUCCESS) {
    NXPLOG_UCIHAL_V("write successful status = 0x%x", pInfo->wStatus);
  } else {
    NXPLOG_UCIHAL_E("write error status = 0x%x", pInfo->wStatus);
  }
  p_cb_data->post(pInfo->wStatus);
}

void report_uci_message(const uint8_t* buffer, size_t len)
{
  if ((nxpucihal_ctrl.p_uwb_stack_data_cback != NULL) && (len <= UCI_MAX_PAYLOAD_LEN)) {
    (*nxpucihal_ctrl.p_uwb_stack_data_cback)(len, buffer);
  }
}

static void handle_rx_packet(uint8_t *buffer, size_t length)
{
  phNxpUciHal_print_packet(NXP_TML_UCI_RSP_NTF_UWBS_2_AP, buffer, length);

  uint8_t mt = ((buffer[0]) & UCI_MT_MASK) >> UCI_MT_SHIFT;
  uint8_t gid = buffer[0] & UCI_GID_MASK;
  uint8_t oid = buffer[1] & UCI_OID_MASK;
  uint8_t pbf = (buffer[0] & UCI_PBF_MASK) >> UCI_PBF_SHIFT;

  bool isSkipPacket = false;

  if (phNxpUciHal_rx_handler_check(length, buffer)) {
    isSkipPacket = true;
  }

  if (mt == UCI_MT_NTF) {
    if (!pbf && gid == UCI_GID_CORE && oid == UCI_MSG_CORE_GENERIC_ERROR_NTF) {
      uint8_t status_code = buffer[UCI_RESPONSE_STATUS_OFFSET];

      if (status_code == UCI_STATUS_COMMAND_RETRY ||
          status_code == UCI_STATUS_SYNTAX_ERROR) {
        // Handle retransmissions
        // TODO: Do not retransmit it when !nxpucihal_ctrl.hal_ext_enabled,
        // Upper layer should take care of it.
        isSkipPacket = true;
        nxpucihal_ctrl.cmdrsp.WakeupError(UWBSTATUS_COMMAND_RETRANSMIT);
      } else if (status_code == UCI_STATUS_BUFFER_UNDERFLOW) {
        if (nxpucihal_ctrl.hal_ext_enabled) {
          NXPLOG_UCIHAL_E("Got Underflow error for ext cmd, retransmit");
          isSkipPacket = true;
          nxpucihal_ctrl.cmdrsp.WakeupError(UWBSTATUS_COMMAND_RETRANSMIT);
        } else {
          // uci to handle retransmission
          buffer[UCI_RESPONSE_STATUS_OFFSET] = UCI_STATUS_COMMAND_RETRY;
          // TODO: Why this should be treated as fail? once we already patched
          // the status code here. Write operation should be treated as success.
          nxpucihal_ctrl.cmdrsp.WakeupError(UWBSTATUS_FAILED);
        }
      } else {
        // TODO: Why should we wake up the user thread here?
        nxpucihal_ctrl.cmdrsp.WakeupError(UWBSTATUS_FAILED);
      }
    }
    // End of UCI_MT_NTF
  } else if (mt == UCI_MT_RSP) {
    if (nxpucihal_ctrl.hal_ext_enabled && !isSkipPacket) {
      isSkipPacket = true;

      if (pbf) {
        /* XXX: fix the whole logic if this really happens */
        NXPLOG_UCIHAL_E("FIXME: Fragmented packets received while processing internal commands!");
      }

      uint8_t status_code = (length > UCI_RESPONSE_STATUS_OFFSET) ?
        buffer[UCI_RESPONSE_STATUS_OFFSET] : UCI_STATUS_UNKNOWN;

      if (status_code == UCI_STATUS_OK) {
        nxpucihal_ctrl.cmdrsp.Wakeup(gid, oid);
      } else if ((gid == UCI_GID_CORE) && (oid == UCI_MSG_CORE_SET_CONFIG)){
        /* check if any configurations are not supported then ignore the
          * UWBSTATUS_FEATURE_NOT_SUPPORTED status code*/
        uint8_t status = phNxpUciHal_process_ext_rsp(length, buffer);
        if (status == UWBSTATUS_SUCCESS) {
          nxpucihal_ctrl.cmdrsp.Wakeup(gid, oid);
        } else {
          nxpucihal_ctrl.cmdrsp.WakeupError(status);
        }
      } else {
        NXPLOG_UCIHAL_E("Got error status code(0x%x) from internal command.", status_code);
        usleep(1);  // XXX: not sure if it's really needed
        nxpucihal_ctrl.cmdrsp.WakeupError(UWBSTATUS_FAILED);
      }
    } else {
      nxpucihal_ctrl.cmdrsp.Wakeup(gid, oid);
    }
  } // End of UCI_MT_RSP

  if (!isSkipPacket) {
    /* Read successful, send the event to higher layer */
    report_uci_message(buffer, length);
  }

  /* Disable junk data check for each UCI packet*/
  if(nxpucihal_ctrl.fw_dwnld_mode) {
    if((gid == UCI_GID_CORE) && (oid == UCI_MSG_CORE_DEVICE_STATUS_NTF)){
      nxpucihal_ctrl.fw_dwnld_mode = false;
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
void phNxpUciHal_read_complete(void* pContext, phTmlUwb_ReadTransactInfo* pInfo)
{
  UNUSED(pContext);

  if (pInfo->wStatus != UWBSTATUS_SUCCESS) {
    NXPLOG_UCIHAL_E("read error status = 0x%x", pInfo->wStatus);
    return;
  }

  NXPLOG_UCIHAL_V("read successful status = 0x%x , total len = 0x%x",
                  pInfo->wStatus, pInfo->wLength);

  for (int32_t index = 0; index < pInfo->wLength; )
  {
    uint8_t extBitSet = (pInfo->pBuff[index + EXTND_LEN_INDICATOR_OFFSET] & EXTND_LEN_INDICATOR_OFFSET_MASK);
    int32_t length = pInfo->pBuff[index + NORMAL_MODE_LENGTH_OFFSET];
    if (extBitSet || ((pInfo->pBuff[index] & UCI_MT_MASK) == 0x00)) {
     length = (length << EXTENDED_MODE_LEN_SHIFT) | pInfo->pBuff[index + EXTENDED_MODE_LEN_OFFSET] ;
    }
    length += UCI_MSG_HDR_SIZE;

    if ((index + length) > pInfo->wLength) {
      NXPLOG_UCIHAL_E("RX Packet misaligned! given length=%u, offset=%d, len=%d",
        pInfo->wLength, index, length);
      return;
    }
    handle_rx_packet(&pInfo->pBuff[index], length);

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

  CONCURRENCY_LOCK();

  SessionTrack_deinit();

  NXPLOG_UCIHAL_D("Terminating phNxpUciHal client thread...");
  nxpucihal_ctrl.pClientMq->send(std::make_shared<phLibUwb_Message>(UCI_HAL_CLOSE_CPLT_MSG));
  nxpucihal_ctrl.client_thread.join();

  nxpucihal_ctrl.halStatus = HAL_STATUS_CLOSE;

  CONCURRENCY_UNLOCK();

  phNxpUciHal_hw_deinit();

  phNxpUciHal_rx_handler_destroy();

  nxpucihal_ctrl.uwb_chip = nullptr;

  phOsalUwb_Timer_Cleanup();

  phNxpUciHal_cleanup_monitor();

  NxpConfig_Deinit();

  phNxpUciLog_deinitialize();

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
  auto res = NxpConfig_GetByteArray(configName);
  if (!res.has_value()) {
    NXPLOG_UCIHAL_D("No antenna pair info found, %s is missing.", configName);
    return;
  }
  std::span<const uint8_t> data = *res;
  if (data.size() <= UCI_MSG_HDR_SIZE) {
    NXPLOG_UCIHAL_D("No antenna pair info found, %s is too short.", configName);
    return;
  }

  int index = 1;  // Excluding number of params
  while (index < data.size()) {
    if ((index + 3) > data.size()) {
      break;
    }
    uint8_t tagId = data[index++];
    uint8_t subTagId = data[index++];
    uint8_t length = data[index++];

    if ((ANTENNA_RX_PAIR_DEFINE_TAG_ID == tagId) &&
        (ANTENNA_RX_PAIR_DEFINE_SUB_TAG_ID == subTagId)) {
      nxpucihal_ctrl.numberOfAntennaPairs = data[index];
      NXPLOG_UCIHAL_D("numberOfAntennaPairs:%d", nxpucihal_ctrl.numberOfAntennaPairs);
      break;
    } else {
      index = index + length;
    }
  }
  NXPLOG_UCIHAL_D("No antenna pair info found in from %s.", configName)
}

/******************************************************************************
 * Function         phNxpUciHal_applyVendorConfig
 *
 * Description      This function applies the vendor config from config file
 *
 * Returns          status
 *
 ******************************************************************************/
static tHAL_UWB_STATUS phNxpUciHal_applyVendorConfig()
{
  std::vector<const char *> vendorParamNames;

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

  // TODO: split this into a function.
  auto chip_pkt = NxpConfig_GetByteArray(per_chip_param);
  if (chip_pkt.has_value()) {
    NXPLOG_UCIHAL_D("VendorConfig: apply %s", per_chip_param);
    tHAL_UWB_STATUS status =
      phNxpUciHal_sendCoreConfig((*chip_pkt).data(), (*chip_pkt).size());
    if (status != UWBSTATUS_SUCCESS) {
      NXPLOG_UCIHAL_E("VendorConfig: failed to apply %s", per_chip_param);
      return status;
    }
  }

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
    auto extra_pkt = NxpConfig_GetByteArray(paramName);
    if (extra_pkt.has_value()) {
      NXPLOG_UCIHAL_D("VendorConfig: apply %s", paramName);
      tHAL_UWB_STATUS status =
        phNxpUciHal_send_ext_cmd((*extra_pkt).size(), (*extra_pkt).data());
      if (status != UWBSTATUS_SUCCESS) {
        NXPLOG_UCIHAL_E("VendorConfig: failed to apply %s", paramName);
        return status;
      }
    }
  }

  // Low Power Mode
  // TODO: remove this out, this can be move to Chip parameter names
  bool lowPowerMode =  NxpConfig_GetBool(NAME_NXP_UWB_LOW_POWER_MODE).value_or(false);
  NXPLOG_UCIHAL_D("VendorConfig: apply %s", NAME_NXP_UWB_LOW_POWER_MODE);

  if (lowPowerMode) {
    // Core set config packet: GID=0x00 OID=0x04
    const std::vector<uint8_t> packet(
        {((UCI_MT_CMD << UCI_MT_SHIFT) | UCI_GID_CORE), UCI_MSG_CORE_SET_CONFIG,
         0x00, 0x04, 0x01, LOW_POWER_MODE_TAG_ID, LOW_POWER_MODE_LENGTH, 0x01 });

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
  auto dev_info_cb = [](size_t packet_len, const uint8_t *packet) mutable -> bool {
    if (packet_len < 5 || packet[UCI_RESPONSE_STATUS_OFFSET] != UWBSTATUS_SUCCESS) {
      NXPLOG_UCIHAL_E("Failed to get valid CORE_DEVICE_INFO_RSP");
      return true;
    }
    if (packet_len > sizeof(nxpucihal_ctrl.dev_info_resp)) {
      NXPLOG_UCIHAL_E("FIXME: CORE_DEVICE_INFO_RSP buffer overflow!");
      return true;
    }

    // FIRA UCIv2.0 packet size = 14
    // [13] = Vendor Specific Info Length
    constexpr uint8_t firaDevInfoRspSize = 14;
    constexpr uint8_t firaDevInfoVendorLenOffset = 13;

    if (packet_len < firaDevInfoRspSize) {
      NXPLOG_UCIHAL_E("DEVICE_INFO_RSP packet size mismatched.");
      return true;
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
    return true;
  };

  nxpucihal_ctrl.isDevInfoCached = false;
  UciHalRxHandler devInfoRspHandler(UCI_MT_RSP, UCI_GID_CORE, UCI_MSG_CORE_DEVICE_INFO, dev_info_cb);

  const uint8_t CoreGetDevInfoCmd[] = {(UCI_MT_CMD << UCI_MT_SHIFT) | UCI_GID_CORE, UCI_MSG_CORE_DEVICE_INFO, 0, 0};
  tHAL_UWB_STATUS status = phNxpUciHal_send_ext_cmd(sizeof(CoreGetDevInfoCmd), CoreGetDevInfoCmd);
  if (status != UWBSTATUS_SUCCESS) {
    return false;
  }
  return true;
}

/******************************************************************************
 * Function         phNxpUciHal_hw_init
 *
 * Description      Init the chip.
 *
 * Returns          status
 *
 ******************************************************************************/
tHAL_UWB_STATUS phNxpUciHal_hw_init()
{
  tHAL_UWB_STATUS status;

  if (nxpucihal_ctrl.halStatus != HAL_STATUS_OPEN) {
    NXPLOG_UCIHAL_E("HAL not initialized");
    return UWBSTATUS_FAILED;
  }
  nxpucihal_ctrl.uwb_device_initialized = false;

  // Initiates TML.
  status = phTmlUwb_Init(uwb_dev_node, nxpucihal_ctrl.pClientMq);
  if (status != UWBSTATUS_SUCCESS) {
    NXPLOG_UCIHAL_E("phTmlUwb_Init Failed");
    return status;
  }

  // Device Status Notification
  UciHalSemaphore devStatusNtfWait;
  uint8_t dev_status = UWB_DEVICE_ERROR;
  auto dev_status_ntf_cb = [&dev_status, &devStatusNtfWait]
      (size_t packet_len, const uint8_t *packet) mutable -> bool {
    if (packet_len >= 5) {
      dev_status = packet[UCI_RESPONSE_STATUS_OFFSET];
      devStatusNtfWait.post();
    }
    return true;
  };
  UciHalRxHandler devStatusNtfHandler(UCI_MT_NTF, UCI_GID_CORE, UCI_MSG_CORE_DEVICE_STATUS_NTF,
                                      dev_status_ntf_cb);

  // FW download and enter UCI operating mode
  status = nxpucihal_ctrl.uwb_chip->chip_init();
  if (status != UWBSTATUS_SUCCESS) {
    return status;
  }

  status = phTmlUwb_StartRead(&phNxpUciHal_read_complete, NULL);
  if (status != UWBSTATUS_SUCCESS) {
    NXPLOG_UCIHAL_E("read status error status = %x", status);
    return status;
  }

  // Wait for the first Device Status Notification
  devStatusNtfWait.wait_timeout_msec(3000);
  if(dev_status != UWB_DEVICE_INIT && dev_status != UWB_DEVICE_READY) {
    NXPLOG_UCIHAL_E("First Device Status NTF was not received or it's invalid state. 0x%x", dev_status);
    return UWBSTATUS_FAILED;
  }

  // Set board-config and wait for Device Status Notification
  status = phNxpUciHal_set_board_config();
  if (status != UWBSTATUS_SUCCESS) {
    NXPLOG_UCIHAL_E("%s: Set Board Config Failed", __func__);
    return status;
  }
  devStatusNtfWait.wait_timeout_msec(3000);
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
  devStatusNtfWait.wait_timeout_msec(3000);
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
  nxpucihal_ctrl.uwb_device_initialized = true;
  nxpucihal_ctrl.recovery_ongoing = false;

  phNxpUciHal_getVersionInfo();

  return UWBSTATUS_SUCCESS;
}

void phNxpUciHal_hw_deinit()
{
  phTmlUwb_Shutdown();
}

void phNxpUciHal_hw_suspend()
{
  nxpucihal_ctrl.uwb_chip->suspend();
}

void phNxpUciHal_hw_resume()
{
  nxpucihal_ctrl.uwb_chip->resume();
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
  tHAL_UWB_STATUS status = phNxpUciHal_hw_init();
  if (status != UWBSTATUS_SUCCESS) {
    nxpucihal_ctrl.pClientMq->send(std::make_shared<phLibUwb_Message>(UCI_HAL_ERROR_MSG));
    return status;
  }

  SessionTrack_init();

  // report to upper-layer
  nxpucihal_ctrl.pClientMq->send(std::make_shared<phLibUwb_Message>(UCI_HAL_INIT_CPLT_MSG));

  constexpr uint8_t dev_ready_ntf[] = {0x60, 0x01, 0x00, 0x01, 0x01};
  report_uci_message(dev_ready_ntf, sizeof(dev_ready_ntf));

  return UWBSTATUS_SUCCESS;
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

/******************************************************************************
 * Function         phNxpUciHal_sendCoreConfig
 *
 * Description      This function send set core config command in chunks when
 *                  config size greater than 255 bytes.
 *
 * Returns          status
 *
 ******************************************************************************/
tHAL_UWB_STATUS phNxpUciHal_sendCoreConfig(const uint8_t *p_cmd,
                                           long buffer_size) {
  std::array<uint8_t, NXP_MAX_CONFIG_STRING_LEN> payload_data;
  tHAL_UWB_STATUS status = UWBSTATUS_FAILED;
  uint16_t i = 0;

  while (buffer_size > 0) {
    uint16_t chunk_size = (buffer_size <= UCI_MAX_CONFIG_PAYLOAD_LEN)
                              ? buffer_size
                              : UCI_MAX_CONFIG_PAYLOAD_LEN;

    payload_data[0] = (buffer_size <= UCI_MAX_CONFIG_PAYLOAD_LEN) ? 0x20 : 0x30;
    payload_data[1] = 0x04;
    payload_data[2] = 0x00;
    payload_data[3] = chunk_size;

    std::memcpy(&payload_data[UCI_PKT_HDR_LEN], &p_cmd[i], chunk_size);

    status = phNxpUciHal_send_ext_cmd(chunk_size + UCI_PKT_HDR_LEN,
                                      payload_data.data());

    i += chunk_size;
    buffer_size -= chunk_size;
  }

  return status;
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
  constexpr uint8_t rsp_data[5] = {0x60, 0x01, 0x00, 0x01, 0xFF};
  report_uci_message(rsp_data, sizeof(rsp_data));
}
