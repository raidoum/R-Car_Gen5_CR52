/*
 * SPDX-License-Identifier: MIT
 */

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"
#include "interrupts.h"

#include <string.h>

#include "pfc/r_pfc_api.h"
#include "logging_stack.h"
#include "ar_ipc_command.h"
#include "ar_ipc_module_load.h"
#include "ar_ipc_send.h"
#include "rpmsg_driver.h"

#define main_TASK_PRIORITY      (tskIDLE_PRIORITY + 1)
#define MAIN_TASK_STACK_SIZE    (configMINIMAL_STACK_SIZE * 10)

_Static_assert(sizeof(ar_ipc_command_t) == 6U, "invalid command header layout");
_Static_assert(sizeof(ar_ipc_command_error_reply_t) == 7U, "invalid error reply layout");

typedef struct {
    SemaphoreHandle_t done;
    rpmsg_driver_status_t *status; /* NULL marks a shutdown request. */
    uint16_t len;
    uint8_t data[AR_IPC_MAX_PAYLOAD_SIZE];
} ar_ipc_send_message_t;

static QueueHandle_t s_send_queue;

/* CR52 core 0 serves ar-ipc on Linux UIO channel 0. */
static const rpmsg_driver_open_cfg_t s_open_cfg = {
    .driver_id = RPMSG_DRIVER_ID_7,
    .side      = RPMSG_DRIVER_SIDE_BE,
    .name      = NULL,
};

static void prvSetupHardware(void)
{
    portDISABLE_INTERRUPTS();
    Irq_Setup();
    (void)pfcInitModules(getModuleConfigs());
}

static void ar_ipc_send_task(void *pvParameters)
{
    rpmsg_driver_channel_t *channel = pvParameters;
    ar_ipc_send_message_t message;

    for (;;) {
        if (xQueueReceive(s_send_queue, &message, portMAX_DELAY) != pdPASS) {
            continue;
        }
        if (message.status == NULL) {
            (void)xSemaphoreGive(message.done);
            break;
        }
        *message.status = rpmsg_driver_send(channel, message.data, message.len);
        (void)xSemaphoreGive(message.done);
    }

    vTaskDelete(NULL);
}

static void submit_message(ar_ipc_send_message_t *message)
{
    StaticSemaphore_t done_storage;

    message->done = xSemaphoreCreateBinaryStatic(&done_storage);
    while (xQueueSend(s_send_queue, message, portMAX_DELAY) != pdPASS) {
    }
    /* Keep the completion semaphore and result storage alive until the sender is done. */
    while (xSemaphoreTake(message->done, portMAX_DELAY) != pdPASS) {
    }
    vSemaphoreDelete(message->done);
}

static bool ar_ipc_send_init(rpmsg_driver_channel_t *channel)
{
    s_send_queue = xQueueCreate(1U, sizeof(ar_ipc_send_message_t));
    if (s_send_queue == NULL) {
        return false;
    }
    if (xTaskCreate(ar_ipc_send_task, "AR_IPC_TX", configMINIMAL_STACK_SIZE * 4, channel,
                    tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
        vQueueDelete(s_send_queue);
        s_send_queue = NULL;
        return false;
    }
    return true;
}

rpmsg_driver_status_t ar_ipc_send_request(const void *data, uint16_t len)
{
    if (len > AR_IPC_MAX_PAYLOAD_SIZE) {
        return RPMSG_DRIVER_ERR_PAYLOAD_TOO_LARGE;
    }
    if (s_send_queue == NULL || (len != 0U && data == NULL)) {
        return RPMSG_DRIVER_ERR_INTERNAL;
    }

    rpmsg_driver_status_t status = RPMSG_DRIVER_ERR_INTERNAL;
    ar_ipc_send_message_t message = { .status = &status, .len = len };
    if (len != 0U) {
        (void)memcpy(message.data, data, len);
    }
    submit_message(&message);
    return status;
}

rpmsg_driver_status_t ar_ipc_send_error_reply(const ar_ipc_command_t *request, ar_ipc_command_error_t error)
{
    ar_ipc_command_error_reply_t reply = { .header = *request, .status = (uint8_t)error };

    reply.header.kind = AR_IPC_MESSAGE_REPLY;
    reply.header.type = AR_IPC_COMMAND_ERROR_REPLY;
    return ar_ipc_send_request(&reply, sizeof(reply));
}

/* Stop the producers before shutting down the sender. */
static void ar_ipc_send_deinit(void)
{
    ar_ipc_send_message_t stop = { 0 };

    submit_message(&stop);
    vQueueDelete(s_send_queue);
    s_send_queue = NULL;
}

static bool ar_app_start(void)
{
    return ar_module_load_task_start();
}

static rpmsg_driver_status_t ar_app_dispatch(const uint8_t *data, uint16_t len)
{
    ar_ipc_command_t request;
    if (len < sizeof(request)) {
        LogAlways(("[AR APP] incomplete command header\n"));
        return RPMSG_DRIVER_OK;
    }
    (void)memcpy(&request, data, sizeof(request));
    if (request.kind == AR_IPC_MESSAGE_REPLY) {
        return RPMSG_DRIVER_OK;
    }
    if (request.kind != AR_IPC_MESSAGE_REQUEST) {
        LogAlways(("[AR APP] unsupported message kind %u\n", (unsigned int)request.kind));
        return RPMSG_DRIVER_OK;
    }

    const ar_ipc_command_type_t type = (ar_ipc_command_type_t)request.type;
    switch (type) {
        case AR_IPC_COMMAND_GET_STATUS:
            if (len != sizeof(ar_ipc_command_t)) {
                break;
            }
            ar_ipc_command_t reply = request;
            reply.kind = AR_IPC_MESSAGE_REPLY;
            reply.type = AR_IPC_COMMAND_STATUS_READY;
            return ar_ipc_send_request(&reply, sizeof(reply));
        case AR_IPC_COMMAND_TYPE_MODULE_LOAD:
            return ar_module_load_dispatch(data, len);
        default:
            LogAlways(("[AR APP] unsupported command type %u\n", (unsigned int)type));
            return ar_ipc_send_error_reply(&request, AR_IPC_COMMAND_ERROR_UNSUPPORTED_TYPE);
    }

    LogAlways(("[AR APP] invalid length %u for command type %u\n",
               (unsigned int)len, (unsigned int)type));
    return ar_ipc_send_error_reply(&request, AR_IPC_COMMAND_ERROR_INVALID_LENGTH);
}

static void ar_app_stop(void)
{
    ar_module_load_task_stop();
}

static void ar_ipc_receive_commands(rpmsg_driver_channel_t *channel)
{
    for (;;) {
        uint8_t data[AR_IPC_MAX_PAYLOAD_SIZE];
        uint16_t len = 0U;
        rpmsg_driver_status_t rc = rpmsg_driver_receive(channel, data, sizeof(data), &len,
                                                        RPMSG_DRIVER_WAIT_FOREVER);
        if (rc == RPMSG_DRIVER_ERR_PEER_DISCONNECTED) {
            LogAlways(("[AR APP] peer disconnected\n"));
            break;
        }
        if (rc == RPMSG_DRIVER_ERR_PAYLOAD_TOO_LARGE) {
            LogAlways(("[AR APP] payload exceeds receive buffer\n"));
            continue;
        }
        if (rc != RPMSG_DRIVER_OK) {
            LogAlways(("[AR APP] RPMsg receive failed (%d)\n", (int)rc));
            break;
        }

        rc = ar_app_dispatch(data, len);
        if (rc != RPMSG_DRIVER_OK) {
            LogAlways(("[AR APP] RPMsg send failed (%d)\n", (int)rc));
            break;
        }
    }
}

static void ar_ipc_task(void *pvParameters)
{
    rpmsg_driver_channel_t *channel = pvParameters;

    rpmsg_driver_status_t rc = rpmsg_driver_open(channel, s_open_cfg);
    if (rc != RPMSG_DRIVER_OK) {
        LogAlways(("[AR APP] RPMsg open failed (%d)\n", (int)rc));
        ar_app_stop();
        vTaskDelete(NULL);
        return;
    }
    LogAlways(("[AR APP] endpoint ready\n"));

    const bool send_ready = ar_ipc_send_init(channel);
    if (send_ready) {
        ar_ipc_receive_commands(channel);
    } else {
        LogAlways(("[AR APP] failed to start send task\n"));
    }

    ar_app_stop();
    if (send_ready) {
        ar_ipc_send_deinit();
    }
    rc = rpmsg_driver_close(channel);
    if (rc == RPMSG_DRIVER_OK) {
        LogAlways(("[AR APP] endpoint closed\n"));
    } else {
        LogAlways(("[AR APP] RPMsg close failed (%d)\n", (int)rc));
    }
    vTaskDelete(NULL);
}

int main(void)
{
    static rpmsg_driver_channel_t channel;
    TaskHandle_t ar_task;

    prvSetupHardware();

    if (xTaskCreate(ar_ipc_task, "AR_IPC", MAIN_TASK_STACK_SIZE, &channel,
                    main_TASK_PRIORITY, &ar_task) == pdPASS) {
        if (ar_app_start()) {
            vTaskStartScheduler();
        } else {
            LogAlways(("[AR APP] application startup failed\n"));
            vTaskDelete(ar_task);
        }
    }

    LogAlways(("[AR APP] task or scheduler startup FAILED\n"));
    for (;;) {
    }
}
