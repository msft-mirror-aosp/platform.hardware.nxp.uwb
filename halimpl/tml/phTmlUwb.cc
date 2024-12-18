/*
 * Copyright 2012-2020, 2024 NXP
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

#include <phNxpLog.h>
#include <phNxpUciHal_utils.h>
#include <phOsalUwb_Timer.h>
#include <phTmlUwb.h>
#include <phTmlUwb_spi.h>
#include <phNxpUciHal.h>
#include <errno.h>

extern phNxpUciHal_Control_t nxpucihal_ctrl;

/*
 * Duration of Timer to wait after sending an Uci packet
 */
#define PHTMLUWB_MAXTIME_RETRANSMIT (200U)
#define MAX_WRITE_RETRY_COUNT 0x03
/* Value to reset variables of TML  */
#define PH_TMLUWB_RESET_VALUE (0x00)

/* Indicates a Initial or offset value */
#define PH_TMLUWB_VALUE_ONE (0x01)

namespace {

// Structure containing details related to read and write operations
struct phTmlUwb_ReadInfo {
  volatile bool bThreadShouldStop;
  volatile bool bThreadRunning;
  uint8_t
      bThreadBusy; /*Flag to indicate thread is busy on respective operation */
  /* Transaction completion Callback function */
  ReadCallback* pThread_Callback;
  void* pContext;        /*Context passed while invocation of operation */
  uint8_t* pBuffer;      /*Buffer passed while invocation of operation */
  size_t wLength;      /*Length of data read/written */
  tHAL_UWB_STATUS wWorkStatus; /*Status of the transaction performed */
};

struct phTmlUwb_WriteInfo {
  volatile bool bThreadShouldStop;
  volatile bool bThreadRunning;
  uint8_t
      bThreadBusy; /*Flag to indicate thread is busy on respective operation */
  /* Transaction completion Callback function */
  WriteCallback* pThread_Callback;
  void* pContext;        /*Context passed while invocation of operation */
  const uint8_t* pBuffer;      /*Buffer passed while invocation of operation */
  size_t wLength;      /*Length of data read/written */
  tHAL_UWB_STATUS wWorkStatus; /*Status of the transaction performed */
};

// Base Context Structure containing members required for entire session
struct phTmlUwb_Context {
  pthread_t readerThread;
  pthread_t writerThread;

  phTmlUwb_ReadInfo tReadInfo;  /*Pointer to Reader Thread Structure */
  phTmlUwb_WriteInfo tWriteInfo; /*Pointer to Writer Thread Structure */
  void* pDevHandle;                    /* Pointer to Device Handle */
  std::shared_ptr<MessageQueue<phLibUwb_Message>> pClientMq; /* Pointer to Client thread message queue */
  sem_t rxSemaphore;
  sem_t txSemaphore;      /* Lock/Acquire txRx Semaphore */

  pthread_cond_t wait_busy_condition; /*Condition to wait reader thread*/
  pthread_mutex_t wait_busy_lock;     /*Condition lock to wait reader thread*/
  volatile uint8_t wait_busy_flag;    /*Condition flag to wait reader thread*/
  volatile uint8_t gWriterCbflag;    /* flag to indicate write callback message is pushed to
                           queue*/
};

std::unique_ptr<phTmlUwb_Context> gpphTmlUwb_Context;

}   // namespace

/* Local Function prototypes */
static tHAL_UWB_STATUS phTmlUwb_StartWriterThread(void);
static void phTmlUwb_StopWriterThread(void);

static void phTmlUwb_CleanUp(void);
static void phTmlUwb_ReadDeferredCb(void* pParams);
static void phTmlUwb_WriteDeferredCb(void* pParams);
static void* phTmlUwb_TmlReaderThread(void* pParam);
static void* phTmlUwb_TmlWriterThread(void* pParam);

extern void setDeviceHandle(void* pDevHandle);

static void phTmlUwb_WaitWriteComplete(void);
static void phTmlUwb_SignalWriteComplete(void);
static int phTmlUwb_WaitReadInit(void);

/* Function definitions */

/*******************************************************************************
**
** Function         phTmlUwb_Init
**
** Description      Provides initialization of TML layer and hardware interface
**                  Configures given hardware interface and sends handle to the
**                  caller
**
** Parameters       pConfig - TML configuration details as provided by the upper
**                            layer
**
** Returns          UWB status:
**                  UWBSTATUS_SUCCESS - initialization successful
**                  UWBSTATUS_INVALID_PARAMETER - at least one parameter is
**                                                invalid
**                  UWBSTATUS_FAILED - initialization failed (for example,
**                                     unable to open hardware interface)
**                  UWBSTATUS_INVALID_DEVICE - device has not been opened or has
**                                             been disconnected
**
*******************************************************************************/
tHAL_UWB_STATUS phTmlUwb_Init(const char* pDevName,
    std::shared_ptr<MessageQueue<phLibUwb_Message>> pClientMq)
{
  if (gpphTmlUwb_Context != nullptr) {
    return PHUWBSTVAL(CID_UWB_TML, UWBSTATUS_ALREADY_INITIALISED);
  }

  if (!pDevName || !pClientMq) {
    return PHUWBSTVAL(CID_UWB_TML, UWBSTATUS_INVALID_PARAMETER);
  }

  // Allocate memory for TML context
  gpphTmlUwb_Context = std::make_unique<phTmlUwb_Context>();
  if (gpphTmlUwb_Context == nullptr) {
    return PHUWBSTVAL(CID_UWB_TML, UWBSTATUS_FAILED);
  }

  // Open the device file to which data is read/written
  tHAL_UWB_STATUS wInitStatus =
    phTmlUwb_spi_open_and_configure(pDevName, &(gpphTmlUwb_Context->pDevHandle));
  if (UWBSTATUS_SUCCESS != wInitStatus) {
    gpphTmlUwb_Context->pDevHandle = NULL;
    return PHUWBSTVAL(CID_UWB_TML, UWBSTATUS_INVALID_DEVICE);
  }

  gpphTmlUwb_Context->tWriteInfo.bThreadBusy = false;
  gpphTmlUwb_Context->pClientMq = pClientMq;

  setDeviceHandle(gpphTmlUwb_Context->pDevHandle);  // To set device handle for FW download usecase

  if (sem_init(&gpphTmlUwb_Context->rxSemaphore, 0, 0)) {
    return UWBSTATUS_FAILED;
  }
  if (sem_init(&gpphTmlUwb_Context->txSemaphore, 0, 0)) {
    return UWBSTATUS_FAILED;
  }
  if(phTmlUwb_WaitReadInit()) {
    return UWBSTATUS_FAILED;
  }

  // Start TML thread (to handle write and read operations)
  if (UWBSTATUS_SUCCESS != phTmlUwb_StartWriterThread()) {
    return PHUWBSTVAL(CID_UWB_TML, UWBSTATUS_FAILED);
  }

  return UWBSTATUS_SUCCESS;
}

/*******************************************************************************
**
** Function         phTmlUwb_TmlReaderThread
**
** Description      Read the data from the lower layer driver
**
** Parameters       pParam  - parameters for Writer thread function
**
** Returns          None
**
*******************************************************************************/
static void* phTmlUwb_TmlReaderThread(void* pParam)
{
  UNUSED(pParam);

  /* Transaction info buffer to be passed to Callback Thread */
  static phTmlUwb_ReadTransactInfo tTransactionInfo;
  /* Structure containing Tml callback function and parameters to be invoked
     by the callback thread */
  static phLibUwb_DeferredCall_t tDeferredInfo;

  gpphTmlUwb_Context->tReadInfo.bThreadRunning = true;
  NXPLOG_TML_D("TmlReader: Thread Started");

  /* Writer thread loop shall be running till shutdown is invoked */
  while (!gpphTmlUwb_Context->tReadInfo.bThreadShouldStop) {
    NXPLOG_TML_V("TmlReader: Running");
    if(sem_wait(&gpphTmlUwb_Context->rxSemaphore)) {
      NXPLOG_TML_E("TmlReader: Failed to wait rxSemaphore err=%d", errno);
      break;
    }

    tHAL_UWB_STATUS wStatus = UWBSTATUS_SUCCESS;

    /* Read the data from the file onto the buffer */
    if (!gpphTmlUwb_Context->pDevHandle) {
      NXPLOG_TML_E("TmlRead: invalid file handle");
      break;
    }

    NXPLOG_TML_V("TmlReader:  Invoking SPI Read");

    uint8_t temp[UCI_MAX_DATA_LEN];
    int dwNoBytesWrRd =
        phTmlUwb_spi_read(gpphTmlUwb_Context->pDevHandle, temp, UCI_MAX_DATA_LEN);

    if(gpphTmlUwb_Context->tReadInfo.bThreadShouldStop) {
      break;
    }

    if (-1 == dwNoBytesWrRd) {
      NXPLOG_TML_E("TmlReader: Error in SPI Read");
      sem_post(&gpphTmlUwb_Context->rxSemaphore);
    } else if (dwNoBytesWrRd > UCI_MAX_DATA_LEN) {
      NXPLOG_TML_E("TmlReader: Numer of bytes read exceeds the limit");
      sem_post(&gpphTmlUwb_Context->rxSemaphore);
    } else if(0 == dwNoBytesWrRd) {
      NXPLOG_TML_E("TmlReader: Empty packet Read, Ignore read and try new read");
      sem_post(&gpphTmlUwb_Context->rxSemaphore);
    } else {
      memcpy(gpphTmlUwb_Context->tReadInfo.pBuffer, temp, dwNoBytesWrRd);

      NXPLOG_TML_V("TmlReader: SPI Read successful");

      /* Update the actual number of bytes read including header */
      gpphTmlUwb_Context->tReadInfo.wLength = dwNoBytesWrRd;

      dwNoBytesWrRd = PH_TMLUWB_RESET_VALUE;

      NXPLOG_TML_V("TmlReader: Posting read message");

      /* Fill the Transaction info structure to be passed to Callback
       * Function */
      tTransactionInfo.wStatus = wStatus;
      tTransactionInfo.pBuff = gpphTmlUwb_Context->tReadInfo.pBuffer;
      tTransactionInfo.wLength = gpphTmlUwb_Context->tReadInfo.wLength;

      /* Read operation completed successfully. Post a Message onto Callback
       * Thread*/
      /* Prepare the message to be posted on User thread */
      tDeferredInfo.pCallback = &phTmlUwb_ReadDeferredCb;
      tDeferredInfo.pParameter = &tTransactionInfo;

      /* TML reader writer callback synchronization mutex lock --- START */
      if (pthread_mutex_lock(&gpphTmlUwb_Context->wait_busy_lock)) {
        NXPLOG_TML_E("[%s] Mutex lock failed for wait_busy_lock at line: %d", __func__, __LINE__);
      }

      if ((gpphTmlUwb_Context->gWriterCbflag == false) &&
        ((gpphTmlUwb_Context->tReadInfo.pBuffer[0] & 0x60) != 0x60)) {
        phTmlUwb_WaitWriteComplete();
      }
      /* TML reader writer callback synchronization mutex lock --- END */
      if (pthread_mutex_unlock(&gpphTmlUwb_Context->wait_busy_lock)) {
        NXPLOG_TML_E("[%s] Mutex unlock failed for wait_busy_lock at line: %d", __func__, __LINE__);
      }

      auto msg = std::make_shared<phLibUwb_Message>(PH_LIBUWB_DEFERREDCALL_MSG, &tDeferredInfo);
      phTmlUwb_DeferredCall(msg);
    }
  } /* End of While loop */

  gpphTmlUwb_Context->tReadInfo.bThreadRunning = false;
  NXPLOG_TML_D("Tml Reader: Thread stopped");

  return NULL;
}

/*******************************************************************************
**
** Function         phTmlUwb_TmlWriterThread
**
** Description      Writes the requested data onto the lower layer driver
**
** Parameters       pParam  - context provided by upper layer
**
** Returns          None
**
*******************************************************************************/
static void* phTmlUwb_TmlWriterThread(void* pParam)
{
  UNUSED(pParam);

  /* Transaction info buffer to be passed to Callback Thread */
  static phTmlUwb_WriteTransactInfo tTransactionInfo;
  /* Structure containing Tml callback function and parameters to be invoked
     by the callback thread */
  static phLibUwb_DeferredCall_t tDeferredInfo;

  gpphTmlUwb_Context->tWriteInfo.bThreadRunning = true;
  NXPLOG_TML_D("TmlWriter: Thread Started");

  /* Writer thread loop shall be running till shutdown is invoked */
  while (!gpphTmlUwb_Context->tWriteInfo.bThreadShouldStop) {
    NXPLOG_TML_V("TmlWriter: Running");
    if (sem_wait(&gpphTmlUwb_Context->txSemaphore)) {
      NXPLOG_TML_E("TmlWriter: Failed to wait txSemaphore, err=%d", errno);
      break;
    }

    if (gpphTmlUwb_Context->tWriteInfo.bThreadShouldStop) {
      break;
    }

    tHAL_UWB_STATUS wStatus = UWBSTATUS_SUCCESS;

    if (!gpphTmlUwb_Context->pDevHandle) {
      NXPLOG_TML_E("TmlWriter: invalid file handle");
      break;
    }

    NXPLOG_TML_V("TmlWriter: Invoking SPI Write");

    /* TML reader writer callback synchronization mutex lock --- START
      */
    if (pthread_mutex_lock(&gpphTmlUwb_Context->wait_busy_lock)) {
      NXPLOG_TML_E("[%s] Mutex lock failed for wait_busy_lock at line: %d", __func__, __LINE__);
    }
    gpphTmlUwb_Context->gWriterCbflag = false;
    int32_t dwNoBytesWrRd =
        phTmlUwb_spi_write(gpphTmlUwb_Context->pDevHandle,
                            gpphTmlUwb_Context->tWriteInfo.pBuffer,
                            gpphTmlUwb_Context->tWriteInfo.wLength);
    /* TML reader writer callback synchronization mutex lock --- END */
    if (pthread_mutex_unlock(&gpphTmlUwb_Context->wait_busy_lock)) {
      NXPLOG_TML_E("[%s] Mutex unlock failed for wait_busy_lock at line: %d", __func__, __LINE__);
    }

    /* Try SPI Write Five Times, if it fails :*/
    if (-1 == dwNoBytesWrRd) {
      NXPLOG_TML_E("TmlWriter: Error in SPI Write");
      wStatus = PHUWBSTVAL(CID_UWB_TML, UWBSTATUS_FAILED);
    } else {
      phNxpUciHal_print_packet(NXP_TML_UCI_CMD_AP_2_UWBS,
                                gpphTmlUwb_Context->tWriteInfo.pBuffer,
                                gpphTmlUwb_Context->tWriteInfo.wLength);
    }
    if (UWBSTATUS_SUCCESS == wStatus) {
      NXPLOG_TML_V("TmlWriter: SPI Write successful");
      dwNoBytesWrRd = PH_TMLUWB_VALUE_ONE;
    }

    NXPLOG_TML_V("TmlWriter: Posting write message");

    /* Fill the Transaction info structure to be passed to Callback Function
     */
    tTransactionInfo.wStatus = wStatus;
    tTransactionInfo.pBuff = gpphTmlUwb_Context->tWriteInfo.pBuffer;
    tTransactionInfo.wLength = dwNoBytesWrRd;

    /* Prepare the message to be posted on the User thread */
    tDeferredInfo.pCallback = &phTmlUwb_WriteDeferredCb;
    tDeferredInfo.pParameter = &tTransactionInfo;

    auto msg = std::make_shared<phLibUwb_Message>(PH_LIBUWB_DEFERREDCALL_MSG, &tDeferredInfo);
    phTmlUwb_DeferredCall(msg);

    if (UWBSTATUS_SUCCESS == wStatus) {
      /* TML reader writer callback synchronization mutex lock --- START
          */
      if (pthread_mutex_lock(&gpphTmlUwb_Context->wait_busy_lock)) {
        NXPLOG_TML_E("[%s] Mutex lock failed for wait_busy_lock at line: %d", __func__, __LINE__);
      }
      gpphTmlUwb_Context->gWriterCbflag = true;
      phTmlUwb_SignalWriteComplete();
        /* TML reader writer callback synchronization mutex lock --- END */
      if (pthread_mutex_unlock(&gpphTmlUwb_Context->wait_busy_lock)) {
        NXPLOG_TML_E("[%s] Mutex unlock failed for wait_busy_lock at line: %d", __func__, __LINE__);
      }
    }
  } /* End of While loop */

  gpphTmlUwb_Context->tWriteInfo.bThreadRunning = false;
  NXPLOG_TML_D("TmlWriter: Thread stopped");

  return NULL;
}

/*******************************************************************************
**
** Function         phTmlUwb_CleanUp
**
** Description      Clears all handles opened during TML initialization
**
** Parameters       None
**
** Returns          None
**
*******************************************************************************/
static void phTmlUwb_CleanUp(void) {
  if (gpphTmlUwb_Context == nullptr) {
    return;
  }

  if (NULL != gpphTmlUwb_Context->pDevHandle) {
    (void)phTmlUwb_Spi_Ioctl(gpphTmlUwb_Context->pDevHandle, phTmlUwb_ControlCode_t::SetPower, 0);
  }

  sem_destroy(&gpphTmlUwb_Context->rxSemaphore);
  sem_destroy(&gpphTmlUwb_Context->txSemaphore);
  if (pthread_mutex_destroy(&gpphTmlUwb_Context->wait_busy_lock)) {
    NXPLOG_TML_E("[%s] Failed to destroy mutex 'wait_busy_lock' at line: %d", __func__, __LINE__);
  }
  if (pthread_cond_destroy(&gpphTmlUwb_Context->wait_busy_condition)) {
    NXPLOG_TML_E("[%s] Failed to destroy conditional variable 'wait_busy_condition' at line: %d",
        __func__, __LINE__);
  }
  phTmlUwb_spi_close(gpphTmlUwb_Context->pDevHandle);
  gpphTmlUwb_Context->pDevHandle = NULL;

  gpphTmlUwb_Context.reset();
}

/*******************************************************************************
**
** Function         phTmlUwb_Shutdown
**
** Description      Uninitializes TML layer and hardware interface
**
** Parameters       None
**
** Returns          UWB status:
**                  UWBSTATUS_SUCCESS - TML configuration released successfully
**                  UWBSTATUS_INVALID_PARAMETER - at least one parameter is
**                                                invalid
**                  UWBSTATUS_FAILED - un-initialization failed (example: unable
**                                     to close interface)
**
*******************************************************************************/
tHAL_UWB_STATUS phTmlUwb_Shutdown(void)
{
  if (!gpphTmlUwb_Context) {
    return PHUWBSTVAL(CID_UWB_TML, UWBSTATUS_NOT_INITIALISED);
  }

  // Abort io threads
  phTmlUwb_StopRead();

  phTmlUwb_StopWriterThread();

  phTmlUwb_CleanUp();

  return UWBSTATUS_SUCCESS;
}

/*******************************************************************************
**
** Function         phTmlUwb_Write
**
** Description      Asynchronously writes given data block to hardware
**                  interface/driver. Enables writer thread if there are no
**                  write requests pending. Returns successfully once writer
**                  thread completes write operation. Notifies upper layer using
**                  callback mechanism.
**
**                  NOTE:
**                  * it is important to post a message with id
**                    PH_TMLUWB_WRITE_MESSAGE to IntegrationThread after data
**                    has been written to SRxxx
**                  * if CRC needs to be computed, then input buffer should be
**                    capable to store two more bytes apart from length of
**                    packet
**
** Parameters       pBuffer - data to be sent
**                  wLength - length of data buffer
**                  pTmlWriteComplete - pointer to the function to be invoked
**                                      upon completion
**                  pContext - context provided by upper layer
**
** Returns          UWB status:
**                  UWBSTATUS_PENDING - command is yet to be processed
**                  UWBSTATUS_INVALID_PARAMETER - at least one parameter is
**                                                invalid
**                  UWBSTATUS_BUSY - write request is already in progress
**
*******************************************************************************/
tHAL_UWB_STATUS phTmlUwb_Write(const uint8_t* pBuffer, size_t wLength,
                         WriteCallback pTmlWriteComplete,
                         void* pContext) {
  tHAL_UWB_STATUS wWriteStatus;

  /* Check whether TML is Initialized */

  if (NULL != gpphTmlUwb_Context) {
    if ((NULL != gpphTmlUwb_Context->pDevHandle) && (NULL != pBuffer) &&
        (PH_TMLUWB_RESET_VALUE != wLength) && (NULL != pTmlWriteComplete)) {
      if (!gpphTmlUwb_Context->tWriteInfo.bThreadBusy) {
        /* Setting the flag marks beginning of a Write Operation */
        gpphTmlUwb_Context->tWriteInfo.bThreadBusy = true;
        /* Copy the buffer, length and Callback function,
           This shall be utilized while invoking the Callback function in thread
           */
        gpphTmlUwb_Context->tWriteInfo.pBuffer = pBuffer;
        gpphTmlUwb_Context->tWriteInfo.wLength = wLength;
        gpphTmlUwb_Context->tWriteInfo.pThread_Callback = pTmlWriteComplete;
        gpphTmlUwb_Context->tWriteInfo.pContext = pContext;

        wWriteStatus = UWBSTATUS_PENDING;
        /* Set event to invoke Writer Thread */
        sem_post(&gpphTmlUwb_Context->txSemaphore);
      } else {
        wWriteStatus = PHUWBSTVAL(CID_UWB_TML, UWBSTATUS_BUSY);
      }
    } else {
      wWriteStatus = PHUWBSTVAL(CID_UWB_TML, UWBSTATUS_INVALID_PARAMETER);
    }
  } else {
    wWriteStatus = PHUWBSTVAL(CID_UWB_TML, UWBSTATUS_NOT_INITIALISED);
  }

  return wWriteStatus;
}

/*******************************************************************************
**
** Function         phTmlUwb_Read
**
** Description      Asynchronously reads data from the driver
**                  Number of bytes to be read and buffer are passed by upper
**                  layer.
**                  Enables reader thread if there are no read requests pending
**                  Returns successfully once read operation is completed
**                  Notifies upper layer using callback mechanism
**
** Parameters       pBuffer - location to send read data to the upper layer via
**                            callback
**                  wLength - length of read data buffer passed by upper layer
**                  pTmlReadComplete - pointer to the function to be invoked
**                                     upon completion of read operation
**                  pContext - context provided by upper layer
**
** Returns          UWB status:
**                  UWBSTATUS_PENDING - command is yet to be processed
**                  UWBSTATUS_INVALID_PARAMETER - at least one parameter is
**                                                invalid
**                  UWBSTATUS_BUSY - read request is already in progress
**
*******************************************************************************/
tHAL_UWB_STATUS phTmlUwb_StartRead(ReadCallback pTmlReadComplete, void* pContext)
{
  // TODO: move this to gpphTmlUwb_Context
  static uint8_t shared_rx_buffer[UCI_MAX_DATA_LEN];

  /* Check whether TML is Initialized */
  if (!gpphTmlUwb_Context || !gpphTmlUwb_Context->pDevHandle) {
    return PHUWBSTVAL(CID_UWB_TML, UWBSTATUS_NOT_INITIALISED);
  }

  if (!pTmlReadComplete) {
    return PHUWBSTVAL(CID_UWB_TML, UWBSTATUS_INVALID_PARAMETER);
  }

  if (gpphTmlUwb_Context->tReadInfo.bThreadRunning) {
    return PHUWBSTVAL(CID_UWB_TML, UWBSTATUS_BUSY);
  }

  /* Setting the flag marks beginning of a Read Operation */
  gpphTmlUwb_Context->tReadInfo.pBuffer = shared_rx_buffer;
  gpphTmlUwb_Context->tReadInfo.wLength = sizeof(shared_rx_buffer);
  gpphTmlUwb_Context->tReadInfo.pThread_Callback = pTmlReadComplete;
  gpphTmlUwb_Context->tReadInfo.pContext = pContext;

  /* Set first event to invoke Reader Thread */
  sem_post(&gpphTmlUwb_Context->rxSemaphore);

  /* Create Reader threads */
  gpphTmlUwb_Context->tReadInfo.bThreadShouldStop = false;
  int ret = pthread_create(&gpphTmlUwb_Context->readerThread, NULL,
                       &phTmlUwb_TmlReaderThread, NULL);
  if (ret) {
    return UWBSTATUS_FAILED;
  } else {
    return UWBSTATUS_SUCCESS;
  }
}

void phTmlUwb_StopRead()
{
  gpphTmlUwb_Context->tReadInfo.bThreadShouldStop = true;

  if (gpphTmlUwb_Context->tReadInfo.bThreadRunning) {
    // to wakeup from blocking read()
    phTmlUwb_Spi_Ioctl(gpphTmlUwb_Context->pDevHandle, phTmlUwb_ControlCode_t::SetPower, ABORT_READ_PENDING);
    sem_post(&gpphTmlUwb_Context->rxSemaphore);

    if (pthread_join(gpphTmlUwb_Context->readerThread, NULL)) {
      NXPLOG_TML_E("[%s] pthread_join failed for reader thread at line: %d ", __func__, __LINE__);
    }
  }
}

/*******************************************************************************
**
** Function         phTmlUwb_StartWriterThread
**
** Description      start writer thread
**
** Parameters       None
**
** Returns          UWB status:
**                  UWBSTATUS_SUCCESS - threads initialized successfully
**                  UWBSTATUS_FAILED - initialization failed due to system error
**
*******************************************************************************/
static tHAL_UWB_STATUS phTmlUwb_StartWriterThread(void)
{
  int ret;

  gpphTmlUwb_Context->tWriteInfo.bThreadShouldStop = false;

  /*Start Writer Thread*/
  ret = pthread_create(&gpphTmlUwb_Context->writerThread, NULL,
                       &phTmlUwb_TmlWriterThread, NULL);
  if (ret) {
    return UWBSTATUS_FAILED;
  } else {
    return UWBSTATUS_SUCCESS;
  }
}


/*******************************************************************************
**
** Function         phTmlUwb_StopWriterThread
**
** Description      Stop writer thread
**
** Parameters       None
**
** Returns          None
**
*******************************************************************************/
static void phTmlUwb_StopWriterThread(void)
{
  gpphTmlUwb_Context->tWriteInfo.bThreadBusy = false;
  gpphTmlUwb_Context->tWriteInfo.bThreadShouldStop = true;

  if (gpphTmlUwb_Context->tWriteInfo.bThreadRunning) {
    sem_post(&gpphTmlUwb_Context->txSemaphore);

    if (pthread_join(gpphTmlUwb_Context->writerThread, NULL)) {
      NXPLOG_TML_E("[%s] pthread_join failed for writer thread at line: %d ", __func__, __LINE__);
    }
  }
}

/*******************************************************************************
**
** Function         phTmlUwb_DeferredCall
**
** Description      Posts message on upper layer thread
**                  upon successful read or write operation
**
** Parameters       msg - message to be posted
**
** Returns          None
**
*******************************************************************************/
void phTmlUwb_DeferredCall(std::shared_ptr<phLibUwb_Message> msg)
{
  gpphTmlUwb_Context->pClientMq->send(msg);
}

/*******************************************************************************
**
** Function         phTmlUwb_ReadDeferredCb
**
** Description      Read thread call back function
**
** Parameters       pParams - context provided by upper layer
**
** Returns          None
**
*******************************************************************************/
static void phTmlUwb_ReadDeferredCb(void* pParams)
{
  /* Transaction info buffer to be passed to Callback Function */
  phTmlUwb_ReadTransactInfo* pTransactionInfo = (phTmlUwb_ReadTransactInfo*)pParams;

  /* Reset the flag to accept another Read Request */
  gpphTmlUwb_Context->tReadInfo.pThread_Callback(
      gpphTmlUwb_Context->tReadInfo.pContext, pTransactionInfo);

  sem_post(&gpphTmlUwb_Context->rxSemaphore);
}

/*******************************************************************************
**
** Function         phTmlUwb_WriteDeferredCb
**
** Description      Write thread call back function
**
** Parameters       pParams - context provided by upper layer
**
** Returns          None
**
*******************************************************************************/
static void phTmlUwb_WriteDeferredCb(void* pParams) {
  /* Transaction info buffer to be passed to Callback Function */
  phTmlUwb_WriteTransactInfo* pTransactionInfo = (phTmlUwb_WriteTransactInfo*)pParams;

  /* Reset the flag to accept another Write Request */
  gpphTmlUwb_Context->tWriteInfo.bThreadBusy = false;
  gpphTmlUwb_Context->tWriteInfo.pThread_Callback(
      gpphTmlUwb_Context->tWriteInfo.pContext, pTransactionInfo);

  return;
}

/*******************************************************************************
**
** Function         phTmlUwb_WaitWriteComplete
**
** Description      wait function for reader thread
**
** Parameters       None
**
** Returns          None
**
*******************************************************************************/
static void phTmlUwb_WaitWriteComplete(void) {
  int ret;
  struct timespec absTimeout;
  if (clock_gettime(CLOCK_MONOTONIC, &absTimeout) == -1) {
    NXPLOG_TML_E("Reader Thread clock_gettime failed");
  } else {
    absTimeout.tv_sec += 1; /*1 second timeout*/
    gpphTmlUwb_Context->wait_busy_flag = true;
    NXPLOG_TML_D("phTmlUwb_WaitWriteComplete - enter");
    ret = pthread_cond_timedwait(&gpphTmlUwb_Context->wait_busy_condition,
                                 &gpphTmlUwb_Context->wait_busy_lock,
                                 &absTimeout);
    if ((ret != 0) && (ret != ETIMEDOUT)) {
      NXPLOG_TML_E("Reader Thread wait failed");
    }
    NXPLOG_TML_D("phTmlUwb_WaitWriteComplete - exit");
  }
}

/*******************************************************************************
**
** Function         phTmlUwb_SignalWriteComplete
**
** Description      function to invoke reader thread
**
** Parameters       None
**
** Returns          None
**
*******************************************************************************/
static void phTmlUwb_SignalWriteComplete(void) {
  int ret;
  if (gpphTmlUwb_Context->wait_busy_flag == true) {
    NXPLOG_TML_D("phTmlUwb_SignalWriteComplete - enter");
    gpphTmlUwb_Context->wait_busy_flag = false;
    ret = pthread_cond_signal(&gpphTmlUwb_Context->wait_busy_condition);
    if (ret) {
      NXPLOG_TML_E(" phTmlUwb_SignalWriteComplete failed, error = 0x%X", ret);
    }
    NXPLOG_TML_D("phTmlUwb_SignalWriteComplete - exit");
  }
}

/*******************************************************************************
**
** Function         phTmlUwb_WaitReadInit
**
** Description      init function for reader thread
**
** Parameters       None
**
** Returns          int
**
*******************************************************************************/
static int phTmlUwb_WaitReadInit(void) {
  int ret;
  pthread_condattr_t attr;
  if (pthread_condattr_init(&attr)) {
    NXPLOG_TML_E(" [%s] conditional attr init failed at line: %d", __func__, __LINE__);
  }
  if (pthread_condattr_setclock(&attr, CLOCK_MONOTONIC)) {
    NXPLOG_TML_E(" [%s] conditional attr setClock failed at line: %d", __func__, __LINE__);
  }
  memset(&gpphTmlUwb_Context->wait_busy_condition, 0,
         sizeof(gpphTmlUwb_Context->wait_busy_condition));
  if (pthread_mutex_init(&gpphTmlUwb_Context->wait_busy_lock, NULL)) {
    NXPLOG_TML_E(" [%s] mutex init failed for wait busy lock at line: %d", __func__,  __LINE__);
  }
  ret = pthread_cond_init(&gpphTmlUwb_Context->wait_busy_condition, &attr);
  if (ret) {
    NXPLOG_TML_E("[%s] pthread_cond_init failed for wait_busy_condition at line: %d", __func__,
        __LINE__);
  }
  return ret;
}

/*******************************************************************************
**
** Function         phTmlUwb_Chip_Reset
**
** Description      Invoke this API to Chip enable/Disable
**
** Parameters       None
**
** Returns          void
**
*******************************************************************************/
void phTmlUwb_Chip_Reset(void){
  if (NULL != gpphTmlUwb_Context->pDevHandle) {
    phTmlUwb_Spi_Ioctl(gpphTmlUwb_Context->pDevHandle, phTmlUwb_ControlCode_t::SetPower, 0);
    usleep(1000);
    phTmlUwb_Spi_Ioctl(gpphTmlUwb_Context->pDevHandle, phTmlUwb_ControlCode_t::SetPower, 1);
  }
}

void phTmlUwb_Suspend(void)
{
  NXPLOG_TML_D("Suspend");
  phTmlUwb_Spi_Ioctl(gpphTmlUwb_Context->pDevHandle, phTmlUwb_ControlCode_t::SetPower, PWR_SUSPEND);

}

void phTmlUwb_Resume(void)
{
  NXPLOG_TML_D("Resume");
  phTmlUwb_Spi_Ioctl(gpphTmlUwb_Context->pDevHandle, phTmlUwb_ControlCode_t::SetPower, PWR_RESUME);
}
