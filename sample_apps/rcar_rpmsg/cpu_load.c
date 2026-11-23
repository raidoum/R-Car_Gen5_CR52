/*
 * SPDX-License-Identifier: MIT
 */

#include "FreeRTOS.h"
#include "task.h"

#include "cpu_load.h"

#define CPU_LOAD_MAX_PERCENT    100U
#define CPU_LOAD_STEP_PERCENT   10U
#define CPU_LOAD_EXIT           (CPU_LOAD_MAX_PERCENT + 1U)
#define CPU_LOAD_PERIOD_TICKS   pdMS_TO_TICKS(100U)

_Static_assert(CPU_LOAD_PERIOD_TICKS >= CPU_LOAD_MAX_PERCENT / CPU_LOAD_STEP_PERCENT,
               "CPU load period requires at least one tick for every 10 percent");

static TaskHandle_t s_load_task;
static uint8_t s_applied_percent;

static BaseType_t run_cpu_load_period(uint32_t *percent)
{
    const TickType_t start = xTaskGetTickCount();
    const TickType_t busy_ticks = CPU_LOAD_PERIOD_TICKS * *percent / CPU_LOAD_MAX_PERCENT;
    TickType_t elapsed = 0U;

    while (elapsed < busy_ticks) {
        TickType_t next = xTaskGetTickCount() - start;
        if (next != elapsed) {
            elapsed = next;
            if (xTaskNotifyWait(0U, 0U, percent, 0U) == pdTRUE) {
                return pdTRUE;
            }
        }
    }

    /* Let the idle task run even during a continuous workload. */
    if (elapsed >= CPU_LOAD_PERIOD_TICKS) {
        taskYIELD();
        return xTaskNotifyWait(0U, 0U, percent, 0U);
    }
    return xTaskNotifyWait(0U, 0U, percent, CPU_LOAD_PERIOD_TICKS - elapsed);
}

static void cpu_load_task(void *pvParameters)
{
    TaskHandle_t application = pvParameters;
    uint32_t percent = 0U;

    while (percent != CPU_LOAD_EXIT) {
        BaseType_t updated = percent == 0U ? xTaskNotifyWait(0U, 0U, &percent, portMAX_DELAY)
                                         : run_cpu_load_period(&percent);
        if (updated == pdTRUE) {
            (void)xTaskNotifyGive(application);
        }
    }
    vTaskDelete(NULL);
}

static void set_cpu_load(uint32_t percent)
{
    (void)xTaskNotify(s_load_task, percent, eSetValueWithOverwrite);
    (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
}

bool cpu_load_init(void)
{
    return xTaskCreate(cpu_load_task, "CPU_LOAD", configMINIMAL_STACK_SIZE,
                       xTaskGetCurrentTaskHandle(), tskIDLE_PRIORITY, &s_load_task) == pdPASS;
}

cpu_load_status_t cpu_load_start(uint8_t percent)
{
    if (percent > CPU_LOAD_MAX_PERCENT || percent % CPU_LOAD_STEP_PERCENT != 0U) {
        return CPU_LOAD_INVALID_PERCENT;
    }
    if (s_load_task == NULL) {
        return CPU_LOAD_UNAVAILABLE;
    }
    set_cpu_load(percent);
    s_applied_percent = percent;
    return CPU_LOAD_OK;
}

cpu_load_status_t cpu_load_stop(void)
{
    return cpu_load_start(0U);
}

uint8_t cpu_load_get_percent(void)
{
    return s_applied_percent;
}

void cpu_load_deinit(void)
{
    if (s_load_task != NULL) {
        set_cpu_load(CPU_LOAD_EXIT);
        s_load_task = NULL;
        s_applied_percent = 0U;
    }
}
