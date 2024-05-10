/*
 * Copyright 2012-2020 NXP
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

#ifndef PHTMLUWB_H
#define PHTMLUWB_H

#include <memory>

#include <phUwbCommon.h>
#include <phMessageQueue.h>

/*
 * Message posted by Reader thread uponl
 * completion of requested operation
 */
#define PH_TMLUWB_READ_MESSAGE (0xAA)

/*
 * Message posted by Writer thread upon
 * completion of requested operation
 */
#define PH_TMLUWB_WRITE_MESSAGE (0x55)

/*
 * Value indicates to reset device
 */
#define PH_TMLUWB_RESETDEVICE (0x00008001)

/*
***************************Globals,Structure and Enumeration ******************
*/

/*
 * Transaction (Tx/Rx) completion information structure of TML
 *
 * This structure holds the completion callback information of the
 * transaction passed from the TML layer to the Upper layer
 * along with the completion callback.
 *
 * The value of field wStatus can be interpreted as:
 *
 *     - UWBSTATUS_SUCCESS                    Transaction performed
 * successfully.
 *     - UWBSTATUS_FAILED                     Failed to wait on Read/Write
 * operation.
 *     - UWBSTATUS_INSUFFICIENT_STORAGE       Not enough memory to store data in
 * case of read.
 *     - UWBSTATUS_BOARD_COMMUNICATION_ERROR  Failure to Read/Write from the
 * file or timeout.
 */

typedef struct phTmlUwb_TransactInfo {
  tHAL_UWB_STATUS wStatus;       /* Status of the Transaction Completion*/
  uint8_t* pBuff;          /* Response Data of the Transaction*/
  uint16_t wLength;        /* Data size of the Transaction*/
} phTmlUwb_TransactInfo_t; /* Instance of Transaction structure */

/*
 * TML transreceive completion callback to Upper Layer
 *
 * pContext - Context provided by upper layer
 * pInfo    - Transaction info. See phTmlUwb_TransactInfo
 */
typedef void (*pphTmlUwb_TransactCompletionCb_t)(
    void* pContext, phTmlUwb_TransactInfo_t* pInfo);

/*
 * Structure containing details related to read and write operations
 *
 */
typedef struct phTmlUwb_ReadWriteInfo {
  volatile bool bThreadShouldStop;
  volatile bool bThreadRunning;
  uint8_t
      bThreadBusy; /*Flag to indicate thread is busy on respective operation */
  /* Transaction completion Callback function */
  pphTmlUwb_TransactCompletionCb_t pThread_Callback;
  void* pContext;        /*Context passed while invocation of operation */
  uint8_t* pBuffer;      /*Buffer passed while invocation of operation */
  uint16_t wLength;      /*Length of data read/written */
  tHAL_UWB_STATUS wWorkStatus; /*Status of the transaction performed */
} phTmlUwb_ReadWriteInfo_t;

/*
 *Base Context Structure containing members required for entire session
 */
typedef struct phTmlUwb_Context {
  pthread_t readerThread; /*Handle to the thread which handles write and read
                             operations */
  pthread_t writerThread;

  phTmlUwb_ReadWriteInfo_t tReadInfo;  /*Pointer to Reader Thread Structure */
  phTmlUwb_ReadWriteInfo_t tWriteInfo; /*Pointer to Writer Thread Structure */
  void* pDevHandle;                    /* Pointer to Device Handle */
  std::shared_ptr<MessageQueue<phLibUwb_Message>> pClientMq; /* Pointer to Client thread message queue */
  uint8_t bEnableCrc;           /*Flag to validate/not CRC for input buffer */
  sem_t rxSemaphore;
  sem_t txSemaphore;      /* Lock/Acquire txRx Semaphore */

  pthread_cond_t wait_busy_condition; /*Condition to wait reader thread*/
  pthread_mutex_t wait_busy_lock;     /*Condition lock to wait reader thread*/
  pthread_mutex_t read_abort_lock;    /*Condition lock to wait read abort*/
  pthread_cond_t read_abort_condition;  /*Condition to wait read abort*/
  volatile uint8_t wait_busy_flag;    /*Condition flag to wait reader thread*/
  volatile uint8_t is_read_abort;    /*Condition flag for read abort*/
  volatile uint8_t gWriterCbflag;    /* flag to indicate write callback message is pushed to
                           queue*/
} phTmlUwb_Context_t;

/*
 * Enum definition contains  supported ioctl control codes.
 *
 * phTmlUwb_Spi_IoCtl
 */
enum class phTmlUwb_ControlCode_t {
  Invalid = 0,
  SetPower,
  EnableFwdMode,
  EnableThroughPut,
  EseReset,
};

/* Function declarations */
tHAL_UWB_STATUS phTmlUwb_Init(const char* pDevName, std::shared_ptr<MessageQueue<phLibUwb_Message>> pClientMq);
tHAL_UWB_STATUS phTmlUwb_Shutdown(void);
void phTmlUwb_Suspend(void);
void phTmlUwb_Resume(void);

// Writer: caller should call this for every write io
tHAL_UWB_STATUS phTmlUwb_Write(uint8_t* pBuffer, uint16_t wLength,
                         pphTmlUwb_TransactCompletionCb_t pTmlWriteComplete,
                         void* pContext);

// Reader: caller calls this once, callback will be called for every received packet.
//         and call StopRead() to unscribe RX packet.
tHAL_UWB_STATUS phTmlUwb_StartRead(uint8_t* pBuffer, uint16_t wLength,
                        pphTmlUwb_TransactCompletionCb_t pTmlReadComplete,
                        void* pContext);
void phTmlUwb_StopRead();

void phTmlUwb_Spi_Reset(void);
void phTmlUwb_Chip_Reset(void);
void phTmlUwb_DeferredCall(std::shared_ptr<phLibUwb_Message> msg);
#endif /*  PHTMLUWB_H  */
