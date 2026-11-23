/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: MIT
 */

#include "FreeRTOS.h"
#include "semphr.h"

#include "rpmsg_driver_os.h"

/* Event = binary semaphore (latches a give with no waiter). */

static void *freertos_event_create(void)
{
    return (void *)xSemaphoreCreateBinary();
}

static bool freertos_event_wait(void *ev, uint32_t timeout_ms)
{
    TickType_t ticks = pdMS_TO_TICKS(timeout_ms);
    if (ticks == 0U) {
        ticks = 1U;
    }
    return xSemaphoreTake((SemaphoreHandle_t)ev, ticks) == pdTRUE;
}

/* The notification interrupt priority must allow FreeRTOS FromISR calls. */
static void freertos_event_signal(void *ev)
{
    BaseType_t woken = pdFALSE;
    (void)xSemaphoreGiveFromISR((SemaphoreHandle_t)ev, &woken);
    portYIELD_FROM_ISR(woken);
}

const rpmsg_driver_os_hooks_t rpmsg_driver_os_hooks = {
    .event_create = freertos_event_create,
    .event_wait   = freertos_event_wait,
    .event_signal = freertos_event_signal,
};

