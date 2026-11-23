/*
 * SPDX-License-Identifier: MIT
 */

#ifndef AR_IPC_COMMAND_H
#define AR_IPC_COMMAND_H

#include <stdint.h>

#define AR_IPC_MAX_PAYLOAD_SIZE 120U
#define AR_IPC_REQUEST_ID_SIZE  4U

typedef enum {
    AR_IPC_MESSAGE_REQUEST = 1,
    AR_IPC_MESSAGE_REPLY   = 2,
} ar_ipc_message_kind_t;

typedef enum {
    AR_IPC_COMMAND_GET_STATUS              = 1,
    AR_IPC_COMMAND_STATUS_READY            = 2,
    AR_IPC_COMMAND_TYPE_MODULE_LOAD        = 3,
    AR_IPC_COMMAND_TYPE_MODULE_LOAD_RESULT = 5,
    AR_IPC_COMMAND_ERROR_REPLY             = 6,
} ar_ipc_command_type_t;

typedef enum {
    AR_IPC_COMMAND_ERROR_UNSUPPORTED_TYPE = 1,
    AR_IPC_COMMAND_ERROR_INVALID_LENGTH   = 2,
} ar_ipc_command_error_t;

/* Common wire header. The receiver echoes the four request ID bytes in the reply. */
typedef struct {
    uint8_t kind;
    uint8_t type;
    uint8_t request_id[AR_IPC_REQUEST_ID_SIZE]; /* Little-endian uint32_t. */
} ar_ipc_command_t;

typedef struct {
    ar_ipc_command_t header;
    uint8_t status;
} ar_ipc_command_error_reply_t;

#endif /* AR_IPC_COMMAND_H */
