/*
 * SPDX-License-Identifier: MIT
 */

#ifndef AR_IPC_MODULE_LOAD_H
#define AR_IPC_MODULE_LOAD_H

#include <stdbool.h>

#include "ar_ipc_command.h"
#include "rpmsg_driver.h"

/* Module load wire definitions shared with the Linux peer. */
typedef enum {
    AR_IPC_MODULE_LOAD_CPU = 1,
    AR_IPC_MODULE_LOAD_GPU = 2,
    AR_IPC_MODULE_LOAD_DDR = 3,
} ar_ipc_module_load_target_t;

typedef enum {
    AR_IPC_MODULE_LOAD_START = 1,
    AR_IPC_MODULE_LOAD_STOP  = 2,
} ar_ipc_module_load_operation_t;

typedef enum {
    AR_IPC_MODULE_LOAD_OK                    = 0,
    AR_IPC_MODULE_LOAD_INVALID_PERCENT       = 1,
    AR_IPC_MODULE_LOAD_UNAVAILABLE           = 2,
    AR_IPC_MODULE_LOAD_UNSUPPORTED_MODULE    = 3,
    AR_IPC_MODULE_LOAD_UNSUPPORTED_OPERATION = 4,
} ar_ipc_module_load_status_t;

typedef struct {
    ar_ipc_command_t header;
    uint8_t module;
    uint8_t operation;
    uint8_t percent;
} ar_module_load_command_t;

typedef struct {
    ar_ipc_command_t header;
    uint8_t module;
    uint8_t status;
    uint8_t applied_percent;
} ar_module_load_result_t;

/* Creates the request queue and task before the scheduler starts. */
bool ar_module_load_task_start(void);
/* Dispatch waits for the module load task. */
rpmsg_driver_status_t ar_module_load_dispatch(const uint8_t *data, uint16_t len);
/* Waits for the task to stop and releases its request queue. */
void ar_module_load_task_stop(void);

#endif /* AR_IPC_MODULE_LOAD_H */
