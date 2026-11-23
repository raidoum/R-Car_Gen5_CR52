/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef RPMSG_DRIVER_OS_H
#define RPMSG_DRIVER_OS_H

#include <stdint.h>
#include <stdbool.h>

/*
 * OS hooks for routes that wake receive() from a notification interrupt.
 *
 *   event_create : new binary event, NULL on failure.
 *   event_wait   : wait for a signal or timeout_ms; consumes the signal.
 *   event_signal : latch a signal. May run in interrupt context.
 */
typedef struct {
    void *(*event_create)(void);
    bool  (*event_wait)(void *ev, uint32_t timeout_ms);
    void  (*event_signal)(void *ev);
} rpmsg_driver_os_hooks_t;

/*
 * Provided by the selected OS port.
 */
extern const rpmsg_driver_os_hooks_t rpmsg_driver_os_hooks;

#endif /* RPMSG_DRIVER_OS_H */
