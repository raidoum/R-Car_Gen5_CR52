/*
 * SPDX-License-Identifier: MIT
 */

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include <string.h>

#include "logging_stack.h"
#include "cpu_load.h"
#include "ar_ipc_module_load.h"
#include "ar_ipc_send.h"

_Static_assert(sizeof(ar_module_load_command_t) == 9U, "invalid module load command layout");
_Static_assert(sizeof(ar_module_load_result_t) == 9U, "invalid module load result layout");

typedef struct {
    const uint8_t *data;
    uint16_t len;
    TaskHandle_t caller;
    rpmsg_driver_status_t send_status;
} module_load_job_t;

static QueueHandle_t s_job_queue;

static ar_ipc_module_load_status_t apply_cpu_load(const ar_module_load_command_t *command)
{
    cpu_load_status_t status;

    switch (command->operation) {
    case AR_IPC_MODULE_LOAD_START:
        status = cpu_load_start(command->percent);
        break;
    case AR_IPC_MODULE_LOAD_STOP:
        status = cpu_load_stop();
        break;
    default:
        return AR_IPC_MODULE_LOAD_UNSUPPORTED_OPERATION;
    }

    switch (status) {
    case CPU_LOAD_OK:
        LogAlways(("[MODULE LOAD] CPU load set to %u%%\n", (unsigned int)cpu_load_get_percent()));
        return AR_IPC_MODULE_LOAD_OK;
    case CPU_LOAD_INVALID_PERCENT:
        return AR_IPC_MODULE_LOAD_INVALID_PERCENT;
    case CPU_LOAD_UNAVAILABLE:
    default:
        return AR_IPC_MODULE_LOAD_UNAVAILABLE;
    }
}

static void handle_module_load(module_load_job_t *job)
{
    ar_module_load_command_t command;

    if (job->len != sizeof(command)) {
        LogAlways(("[MODULE LOAD] invalid command length %u\n", (unsigned int)job->len));
        if (job->len >= sizeof(command.header)) {
            (void)memcpy(&command.header, job->data, sizeof(command.header));
            job->send_status = ar_ipc_send_error_reply(&command.header, AR_IPC_COMMAND_ERROR_INVALID_LENGTH);
        }
        return;
    }
    (void)memcpy(&command, job->data, sizeof(command));

    ar_module_load_result_t result = {
        .header = command.header,
        .module = command.module,
    };
    result.header.kind = AR_IPC_MESSAGE_REPLY;
    result.header.type = AR_IPC_COMMAND_TYPE_MODULE_LOAD_RESULT;
    switch (command.module) {
    case AR_IPC_MODULE_LOAD_CPU:
        result.status = (uint8_t)apply_cpu_load(&command);
        result.applied_percent = cpu_load_get_percent();
        break;
    case AR_IPC_MODULE_LOAD_GPU:
    case AR_IPC_MODULE_LOAD_DDR:
    default:
        result.status = AR_IPC_MODULE_LOAD_UNSUPPORTED_MODULE;
        break;
    }
    job->send_status = ar_ipc_send_request(&result, sizeof(result));
}

static void module_load_task(void *pvParameters)
{
    (void)pvParameters;

    const bool cpu_load_ready = cpu_load_init();
    if (!cpu_load_ready) {
        LogAlways(("[MODULE LOAD] failed to start CPU load task\n"));
    }

    module_load_job_t *job;
    for (;;) {
        (void)xQueueReceive(s_job_queue, &job, portMAX_DELAY);
        if (job->data == NULL) {
            break;
        }
        handle_module_load(job);
        (void)xTaskNotifyGive(job->caller);
    }

    if (cpu_load_ready) {
        cpu_load_deinit();
        LogAlways(("[MODULE LOAD] CPU load task stopped\n"));
    }
    (void)xTaskNotifyGive(job->caller);
    vTaskDelete(NULL);
}

static void run_module_load_job(module_load_job_t *job)
{
    /* The caller keeps the job and its input valid until the task replies. */
    job->caller = xTaskGetCurrentTaskHandle();
    (void)xQueueSend(s_job_queue, &job, portMAX_DELAY);
    (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
}

bool ar_module_load_task_start(void)
{
    s_job_queue = xQueueCreate(1U, sizeof(module_load_job_t *));
    if (s_job_queue == NULL) {
        return false;
    }
    if (xTaskCreate(module_load_task, "MODULE_LOAD", configMINIMAL_STACK_SIZE * 4,
                    NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
        vQueueDelete(s_job_queue);
        s_job_queue = NULL;
        return false;
    }
    return true;
}

rpmsg_driver_status_t ar_module_load_dispatch(const uint8_t *data, uint16_t len)
{
    module_load_job_t job = { .data = data, .len = len, .send_status = RPMSG_DRIVER_OK };

    run_module_load_job(&job);
    return job.send_status;
}

void ar_module_load_task_stop(void)
{
    module_load_job_t stop = { 0 };

    run_module_load_job(&stop);
    vQueueDelete(s_job_queue);
    s_job_queue = NULL;
}
