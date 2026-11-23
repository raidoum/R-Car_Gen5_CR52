/*
 * SPDX-License-Identifier: MIT
 */

#ifndef AR_IPC_SEND_H
#define AR_IPC_SEND_H

#include "ar_ipc_command.h"
#include "rpmsg_driver.h"

/* Queue one response and wait for the driver's send result. */
rpmsg_driver_status_t ar_ipc_send_request(const void *data, uint16_t len);
rpmsg_driver_status_t ar_ipc_send_error_reply(const ar_ipc_command_t *request, ar_ipc_command_error_t error);

#endif /* AR_IPC_SEND_H */
