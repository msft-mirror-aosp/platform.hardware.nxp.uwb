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

#ifndef PHUWBTYPES_H
#define PHUWBTYPES_H

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdbool.h>

#include <memory>

#include <phMessageQueue.h>

#ifndef true
#define true (0x01) /* Logical True Value */
#endif
#ifndef TRUE
#define TRUE (0x01) /* Logical True Value */
#endif
#ifndef false
#define false (0x00) /* Logical False Value */
#endif
#ifndef FALSE
#define FALSE (0x00) /* Logical False Value */
#endif
typedef uint8_t utf8_t;     /* UTF8 Character String */
typedef uint8_t bool_t;     /* boolean data type */
typedef uint16_t tHAL_UWB_STATUS; /* Return values */

#define STATIC static

#define PHUWB_MAX_UID_LENGTH 0x0AU /* Maximum UID length expected */
/* Maximum ATR_RES (General Bytes) length expected */
#define PHUWB_MAX_ATR_LENGTH 0x30U
#define PHUWB_UWBID_LENGTH 0x0AU /* Maximum length of UWBID 1.3*/
#define PHUWB_ATQA_LENGTH 0x02U  /* ATQA length */

/*
 * UWB Data structure
 */
typedef struct phUwb_sData {
  uint8_t* buffer; /* Buffer to store data */
  uint32_t length; /* Buffer length */
} phUwb_sData_t;

#define UNUSED(X) (void)(X);
/*
 * Possible Hardware Configuration exposed to upper layer.
 * Typically this should be port name (Ex:"COM1","COM2") to which SR100 is
 * connected.
 */
typedef enum {
  ENUM_LINK_TYPE_COM1,
  ENUM_LINK_TYPE_COM2,
  ENUM_LINK_TYPE_COM3,
  ENUM_LINK_TYPE_COM4,
  ENUM_LINK_TYPE_COM5,
  ENUM_LINK_TYPE_COM6,
  ENUM_LINK_TYPE_COM7,
  ENUM_LINK_TYPE_COM8,
  ENUM_LINK_TYPE_I2C,
  ENUM_LINK_TYPE_SPI,
  ENUM_LINK_TYPE_USB,
  ENUM_LINK_TYPE_TCP,
  ENUM_LINK_TYPE_NB
} phLibUwb_eConfigLinkType;

/*
 * Deferred message. This message type will be posted to the client application
 * thread
 * to notify that a deferred call must be invoked.
 */
#define PH_LIBUWB_DEFERREDCALL_MSG (0x311)

/*
 * Deferred call declaration.
 * This type of API is called from ClientApplication ( main thread) to notify
 * specific callback.
 */
typedef void (*pphLibUwb_DeferredCallback_t)(void*);

/*
 * UWB Message structure contains message specific details like
 * message type, message specific data block details, etc.
 */
struct phLibUwb_Message {
  uint32_t eMsgType; /* Type of the message to be posted*/
  void* pMsgData;    /* Pointer to message specific data block in case any*/
  phLibUwb_Message(uint32_t type) : eMsgType(type), pMsgData(NULL) {}
  phLibUwb_Message(uint32_t type, void *data) : eMsgType(type), pMsgData(data) {}
};

/*
 * Deferred message specific info declaration.
 * This type of information is packed as message data when
 * PH_LIBUWB_DEFERREDCALL_MSG
 * type message is posted to message handler thread.
 */
typedef struct phLibUwb_DeferredCall {
  pphLibUwb_DeferredCallback_t pCallback;   /* pointer to Deferred callback */
  void *pParameter; /* pointer to Deferred parameter */
} phLibUwb_DeferredCall_t;

/*
 * Definitions for supported protocol
 */

#endif /* PHUWBTYPES_H */
