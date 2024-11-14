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

struct phTmlUwb_WriteTransactInfo {
  tHAL_UWB_STATUS wStatus;
  const uint8_t* pBuff;
  size_t wLength;
};

struct phTmlUwb_ReadTransactInfo {
  tHAL_UWB_STATUS wStatus;
  uint8_t* pBuff;
  size_t wLength;
};

// IO completion callback to Upper Layer
// pContext - Context provided by upper layer
// pInfo    - Transaction info. See phTmlUwb_[Read|Write]TransactInfo
using ReadCallback = void (void *pContext, phTmlUwb_ReadTransactInfo* pInfo);
using WriteCallback = void (void *pContext, phTmlUwb_WriteTransactInfo* pInfo);

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
tHAL_UWB_STATUS phTmlUwb_Write(const uint8_t* pBuffer, size_t wLength,
                         WriteCallback pTmlWriteComplete,
                         void* pContext);

// Reader: caller calls this once, callback will be called for every received packet.
//         and call StopRead() to unscribe RX packet.
tHAL_UWB_STATUS phTmlUwb_StartRead(ReadCallback pTmlReadComplete, void* pContext);
void phTmlUwb_StopRead();

void phTmlUwb_Chip_Reset(void);
void phTmlUwb_DeferredCall(std::shared_ptr<phLibUwb_Message> msg);
#endif /*  PHTMLUWB_H  */
