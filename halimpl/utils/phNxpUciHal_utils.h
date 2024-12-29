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

#ifndef _PHNXPUCIHAL_UTILS_H_
#define _PHNXPUCIHAL_UTILS_H_

#include <pthread.h>
#include <semaphore.h>
#include <time.h>
#include <unordered_set>

#include <assert.h>
#include <bit>
#include <cstring>
#include <map>
#include <memory>
#include <thread>
#include <type_traits>
#include <vector>


#include "phNxpLog.h"
#include "phUwbStatus.h"

/********************* Definitions and structures *****************************/
/* Which is the direction of UWB Packet.
 *
 * Used by the @ref phNxpUciHal_print_packet API.
 */
enum phNxpUciHal_Pkt_Type {
  NXP_TML_UCI_CMD_AP_2_UWBS,
  NXP_TML_UCI_RSP_NTF_UWBS_2_AP,
  NXP_TML_FW_DNLD_CMD_AP_2_UWBS,
  NXP_TML_FW_DNLD_RSP_UWBS_2_AP,
};

// TODO: 1) use UciHalSemaphore, 2) use std::binary_semaphore
/* Semaphore handling structure */
typedef struct phNxpUciHal_Sem {
  sem_t sem;
  tHAL_UWB_STATUS status;
} phNxpUciHal_Sem_t;

/* Semaphore helper macros */
static inline int SEM_WAIT(phNxpUciHal_Sem_t* pCallbackData)
{
  return sem_wait(&pCallbackData->sem);
}

static inline int SEM_POST(phNxpUciHal_Sem_t* pCallbackData)
{
  return sem_post(&pCallbackData->sem);
}

/************************ Exposed functions ***********************************/
/* NXP UCI HAL utility functions */
bool phNxpUciHal_init_monitor(void);
void phNxpUciHal_cleanup_monitor(void);

tHAL_UWB_STATUS phNxpUciHal_init_cb_data(phNxpUciHal_Sem_t* pCallbackData);

int phNxpUciHal_sem_timed_wait_msec(phNxpUciHal_Sem_t* pCallbackData, long msec);

static inline int phNxpUciHal_sem_timed_wait_sec(phNxpUciHal_Sem_t* pCallbackData, time_t sec)
{
  return phNxpUciHal_sem_timed_wait_msec(pCallbackData, sec * 1000L);
}

static inline int phNxpUciHal_sem_timed_wait(phNxpUciHal_Sem_t* pCallbackData)
{
  /* default 1 second timeout*/
  return phNxpUciHal_sem_timed_wait_msec(pCallbackData, 1000L);
}

void phNxpUciHal_cleanup_cb_data(phNxpUciHal_Sem_t* pCallbackData);

// helper class for Semaphore
// phNxpUciHal_init_cb_data(), phNxpUciHal_cleanup_cb_data(),
// SEM_WAIT(), SEM_POST()
class UciHalSemaphore {
public:
  UciHalSemaphore() {
    phNxpUciHal_init_cb_data(&sem);
  }
  virtual ~UciHalSemaphore() {
    phNxpUciHal_cleanup_cb_data(&sem);
  }
  int wait() {
    return sem_wait(&sem.sem);
  }
  int wait_timeout_msec(long msec) {
    return phNxpUciHal_sem_timed_wait_msec(&sem, msec);
  }
  int post() {
    sem.status = UWBSTATUS_SUCCESS;
    return sem_post(&sem.sem);
  }
  int post(tHAL_UWB_STATUS status) {
    sem.status = status;
    return sem_post(&sem.sem);
  }
  tHAL_UWB_STATUS getStatus() {
    return sem.status;
  }
private:
  phNxpUciHal_Sem_t sem;
};

/*
 * Print an UWB Packet.
 *
 * @param what The type and direction of packet
 *
 * @param p_data The packet to be printed/logged.
 *
 * @param len Tenth of the packet.
 *
 */

void phNxpUciHal_print_packet(enum phNxpUciHal_Pkt_Type what, const uint8_t* p_data,
                              uint16_t len);
void phNxpUciHal_emergency_recovery(void);
double phNxpUciHal_byteArrayToDouble(const uint8_t* p_data);

template <typename T>
static inline T le_bytes_to_cpu(const uint8_t *p)
{
  static_assert(std::is_integral_v<T>, "bytes_to_cpu must be used with an integral type");
  T val = 0;
  if (std::endian::native == std::endian::little) {
    std::memcpy(&val, p, sizeof(T));
  } else {
    size_t i = sizeof(T);
    while (i--) {
      val = (val << 8) | p[i];
    }
  }
  return val;
}

template <typename T>
static inline void cpu_to_le_bytes(uint8_t *p, const T num)
{
  static_assert(std::is_integral_v<T>, "cpu_to_le_bytes must be used with an integral type");
  T val = num;
  if (std::endian::native == std::endian::little) {
    std::memcpy(p, &val, sizeof(T));
  } else {
    for (size_t i = 0; i < sizeof(T); i++) {
      p[i] = val & 0xff;
      val = val >> 8;
    }
  }
}

/* Lock unlock helper macros */
void REENTRANCE_LOCK();
void REENTRANCE_UNLOCK();
void CONCURRENCY_LOCK();
void CONCURRENCY_UNLOCK();

// Decode bytes into map<key=T, val=LV>
std::map<uint16_t, std::vector<uint8_t>>
decodeTlvBytes(const std::vector<uint8_t> &ext_ids, const uint8_t *tlv_bytes, size_t tlv_len);

// Encode map<key=T, val=LV> into TLV bytes
std::vector<uint8_t> encodeTlvBytes(const std::map<uint16_t, std::vector<uint8_t>> &tlvs);

#endif /* _PHNXPUCIHAL_UTILS_H_ */
