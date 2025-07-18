/*
 * Copyright 2012-2020, 2023 NXP
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
#include <errno.h>
#include <pthread.h>
#include <log/log.h>

#include <phNxpLog.h>
#include <phNxpUciHal.h>
#include <phNxpUciHal_utils.h>

using namespace std;
map<uint16_t, vector<uint16_t>> input_map;
map<uint16_t, vector<uint16_t>> conf_map;

/****************** Semaphore and mutex helper functions **********************/
/* Semaphore and mutex monitor */
struct phNxpUciHal_Monitor {
public:
  static std::unique_ptr<phNxpUciHal_Monitor> Create() {
    //auto monitor = std::unique_ptr<phNxpUciHal_Monitor>(new phNxpUciHal_Monitor());
    auto monitor = std::make_unique<phNxpUciHal_Monitor>();
    if (pthread_mutex_init(&monitor->reentrance_mutex_, NULL) == -1) {
      return nullptr;
    }
    if (pthread_mutex_init(&monitor->concurrency_mutex_, NULL) == -1) {
      pthread_mutex_destroy(&monitor->reentrance_mutex_);
      return nullptr;
    }
    return monitor;
  }

  virtual ~phNxpUciHal_Monitor() {
    pthread_mutex_destroy(&concurrency_mutex_);
    ReentranceUnlock();
    pthread_mutex_destroy(&reentrance_mutex_);
    for (auto p : sems_) {
      NXPLOG_UCIHAL_E("Unreleased semaphore %p", p);
      p->status = UWBSTATUS_FAILED;
      sem_post(&p->sem);
    }
    sems_.clear();
  }

  void AddSem(phNxpUciHal_Sem_t* pCallbackData) {
    std::lock_guard<std::mutex> lock(lock_);
    auto it = sems_.find(pCallbackData);
    if (it == sems_.end()) {
      sems_.insert(pCallbackData);
    } else {
      NXPLOG_UCIHAL_E("phNxpUciHal_init_cb_data: duplicated semaphore %p",
        pCallbackData);
    }
  }

  void RemoveSem(phNxpUciHal_Sem_t* pCallbackData) {
    std::lock_guard<std::mutex> lock(lock_);
    auto it = sems_.find(pCallbackData);
    if (it == sems_.end()) {
      NXPLOG_UCIHAL_E("phNxpUciHal_cleanup_cb_data: orphan semaphore %p",
        pCallbackData);
    } else {
      sems_.erase(it);
    }
  }

  void Reentrancelock() {
    pthread_mutex_lock(&reentrance_mutex_);
  }

  void ReentranceUnlock() {
    pthread_mutex_unlock(&reentrance_mutex_);
  }

  void Concurrencylock() {
    pthread_mutex_lock(&concurrency_mutex_);
  }

  void ConcurrencyUnlock() {
    pthread_mutex_unlock(&concurrency_mutex_);
  }

private:
  std::unordered_set<phNxpUciHal_Sem_t*> sems_;
  std::mutex lock_;
  // Mutex protecting native library against reentrance
  pthread_mutex_t reentrance_mutex_;
  // Mutex protecting native library against concurrency
  pthread_mutex_t concurrency_mutex_;
};

static std::unique_ptr<phNxpUciHal_Monitor> nxpucihal_monitor;

/*******************************************************************************
**
** Function         phNxpUciHal_init_monitor
**
** Description      Initialize the semaphore monitor
**
** Returns          Pointer to monitor, otherwise NULL if failed
**
*******************************************************************************/
bool phNxpUciHal_init_monitor(void) {
  NXPLOG_UCIHAL_D("Entering phNxpUciHal_init_monitor");

  nxpucihal_monitor = phNxpUciHal_Monitor::Create();

  if (nxpucihal_monitor == nullptr) {
    NXPLOG_UCIHAL_E("nxphal_monitor creation failed");
    return false;
  }
  return true;
}

/*******************************************************************************
**
** Function         phNxpUciHal_cleanup_monitor
**
** Description      Clean up semaphore monitor
**
** Returns          None
**
*******************************************************************************/
void phNxpUciHal_cleanup_monitor(void) {
  nxpucihal_monitor = nullptr;
}

/* Initialize the callback data */
tHAL_UWB_STATUS phNxpUciHal_init_cb_data(phNxpUciHal_Sem_t* pCallbackData) {
  /* Create semaphore */
  if (sem_init(&pCallbackData->sem, 0, 0) == -1) {
    NXPLOG_UCIHAL_E("Semaphore creation failed");
    return UWBSTATUS_FAILED;
  }

  /* Set default status value */
  pCallbackData->status = UWBSTATUS_FAILED;

  /* Add to active semaphore list */
  if (nxpucihal_monitor != nullptr) {
    nxpucihal_monitor->AddSem(pCallbackData);
  }

  return UWBSTATUS_SUCCESS;
}

/*******************************************************************************
**
** Function         phNxpUciHal_cleanup_cb_data
**
** Description      Clean up callback data
**
** Returns          None
**
*******************************************************************************/
void phNxpUciHal_cleanup_cb_data(phNxpUciHal_Sem_t* pCallbackData) {
  /* Destroy semaphore */
  if (sem_destroy(&pCallbackData->sem)) {
    NXPLOG_UCIHAL_E(
        "phNxpUciHal_cleanup_cb_data: Failed to destroy semaphore");
  }
  if (nxpucihal_monitor != nullptr) {
    nxpucihal_monitor->RemoveSem(pCallbackData);
  }
}

void REENTRANCE_LOCK() {
  if (nxpucihal_monitor != nullptr) {
    nxpucihal_monitor->Reentrancelock();
  }
}
void REENTRANCE_UNLOCK() {
  if (nxpucihal_monitor != nullptr) {
    nxpucihal_monitor->ReentranceUnlock();
  }
}
void CONCURRENCY_LOCK() {
  if (nxpucihal_monitor != nullptr) {
    nxpucihal_monitor->Concurrencylock();
  }
}
void CONCURRENCY_UNLOCK() {
  if (nxpucihal_monitor != nullptr) {
    nxpucihal_monitor->ConcurrencyUnlock();
  }
}

int phNxpUciHal_sem_timed_wait_msec(phNxpUciHal_Sem_t* pCallbackData, long msec)
{
  int ret;
  struct timespec absTimeout;
  if (clock_gettime(CLOCK_MONOTONIC, &absTimeout) == -1) {
    NXPLOG_UCIHAL_E("clock_gettime failed");
    return -1;
  }

  if (msec > 1000L) {
    absTimeout.tv_sec += msec / 1000L;
    msec = msec % 1000L;
  }
  absTimeout.tv_nsec += msec * 1000000L;
  if (absTimeout.tv_nsec > 1000000000L) {
    absTimeout.tv_nsec -= 1000000000L;
    absTimeout.tv_sec += 1;
  }

  while ((ret = sem_timedwait_monotonic_np(&pCallbackData->sem, &absTimeout)) == -1 && errno == EINTR) {
    continue;
  }
  if (ret == -1 && errno == ETIMEDOUT) {
    pCallbackData->status = UWBSTATUS_RESPONSE_TIMEOUT;
    NXPLOG_UCIHAL_E("wait semaphore timed out");
    return -1;
  }
  return 0;
}

/* END Semaphore and mutex helper functions */

/**************************** Other functions *********************************/

/*******************************************************************************
**
** Function         phNxpUciHal_print_packet
**
** Description      Print packet
**
** Returns          None
**
*******************************************************************************/
void phNxpUciHal_print_packet(enum phNxpUciHal_Pkt_Type what, const uint8_t* p_data,
                              uint16_t len) {
  uint32_t i;
  char print_buffer[len * 3 + 1];

  if ((gLog_level.ucix_log_level >= NXPLOG_LOG_DEBUG_LOGLEVEL)) {
    /* OK to print */
  }
  else
  {
    /* Nothing to print...
     * Why prepare buffer without printing?
     */
    return;
  }

  memset(print_buffer, 0, sizeof(print_buffer));
  for (i = 0; i < len; i++) {
    snprintf(&print_buffer[i * 2], 3, "%02X", p_data[i]);
  }
  switch(what) {
    case NXP_TML_UCI_CMD_AP_2_UWBS:
    {
      NXPLOG_UCIX_D("len = %3d > %s", len, print_buffer);
    }
    break;
    case NXP_TML_UCI_RSP_NTF_UWBS_2_AP:
    {
      NXPLOG_UCIR_D("len = %3d < %s", len, print_buffer);
    }
    break;
    case NXP_TML_FW_DNLD_CMD_AP_2_UWBS:
    {
      // TODO: Should be NXPLOG_FWDNLD_D
      NXPLOG_UCIX_D("len = %3d > (FW)%s", len, print_buffer);
    }
    break;
    case NXP_TML_FW_DNLD_RSP_UWBS_2_AP:
    {
      // TODO: Should be NXPLOG_FWDNLD_D
      NXPLOG_UCIR_D("len = %3d < (FW)%s", len, print_buffer);
    }
    break;
  }

  phNxpUciHalProp_print_log(what, p_data, len);

  return;
}

/*******************************************************************************
**
** Function         phNxpUciHal_emergency_recovery
**
** Description      Emergency recovery in case of no other way out
**
** Returns          None
**
*******************************************************************************/

void phNxpUciHal_emergency_recovery(void) {
  NXPLOG_UCIHAL_E("%s: abort()", __func__);
  abort();
}

/*******************************************************************************
**
** Function         phNxpUciHal_byteArrayToDouble
**
** Description      convert byte array to double
**
** Returns          double
**
*******************************************************************************/
double phNxpUciHal_byteArrayToDouble(const uint8_t* p_data) {
  double d;
  int size_d = sizeof(d);
  uint8_t ptr[size_d],ptr_1[size_d];
  memcpy(&ptr, p_data, size_d);
  for(int i=0;i<size_d;i++) {
    ptr_1[i] = ptr[size_d - 1 - i];
  }
  memcpy(&d, &ptr_1, sizeof(d));
  return d;                                                       \
}

std::map<uint16_t, std::vector<uint8_t>>
decodeTlvBytes(const std::vector<uint8_t> &ext_ids, const uint8_t *tlv_bytes, size_t tlv_len)
{
  std::map<uint16_t, std::vector<uint8_t>> ret;

  size_t i = 0;
  while ((i + 1) < tlv_len) {
    uint16_t tag;
    uint8_t len;

    uint8_t byte0 = tlv_bytes[i++];
    uint8_t byte1 = tlv_bytes[i++];
    if (std::find(ext_ids.begin(), ext_ids.end(), byte0) != ext_ids.end()) {
      if (i >= tlv_len) {
        NXPLOG_UCIHAL_E("Failed to decode TLV bytes (offset=%zu).", i);
        break;
      }
      tag = (byte0 << 8) | byte1; // 2 bytes tag as big endiann
      len = tlv_bytes[i++];
    } else {
      tag = byte0;
      len = byte1;
    }
    if ((i + len) > tlv_len) {
      NXPLOG_UCIHAL_E("Failed to decode TLV bytes (offset=%zu).", i);
      break;
    }
    ret[tag] = std::vector(&tlv_bytes[i], &tlv_bytes[i + len]);
    i += len;
  }

  return ret;
}

std::vector<uint8_t> encodeTlvBytes(const std::map<uint16_t, std::vector<uint8_t>> &tlvs)
{
  std::vector<uint8_t> bytes;

  for (auto const & [tag, val] : tlvs) {
    // Tag
    if (tag > 0xff) {
      bytes.push_back(tag >> 8);
    }
    bytes.push_back(tag & 0xff);

    // Length
    bytes.push_back(val.size());

    // Value
    bytes.insert(bytes.end(), val.begin(), val.end());
  }

  return bytes;
}
