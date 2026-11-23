/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: MIT
 */

#include "FreeRTOS.h"
#include "task.h"
#include "interrupts.h"

#include <stdio.h>
#include <string.h>

#include "pfc/r_pfc_api.h"
#include "rcar_utils.h"

#include "rpmsg_driver.h"

#include "board.h"

/*
 * On-board test of the rcar_rpmsg API. Two cores, one binary, branched by
 * R_UTILS_GetCpuID():
 *   core0 = FE: drives every test case and prints PASS/FAIL per case.
 *   core1 = BE: echo server, plus a few commands the FE sends to make it
 *               misbehave on purpose. Prints its own PASS/FAIL cases.
 *
 * Channels (fixed in rpmsg_driver.c):
 *   RPMSG_DRIVER_ID_0   - vdev 0, MFIS, name service   -> control + echo
 *   RPMSG_DRIVER_ID_3 - vdev 3, polled, fixed address -> echo
 *
 * FE -> BE commands on smmu (anything else is echoed back):
 *   "!burst3" : BE sends "b1", "b2", "b3" back to back
 *   "!stop"   : BE stops reading smmu for BE_STOP_MS
 *   "!close"  : BE closes its smmu channel (last one: releases vdev 0)
 *
 * Each core ends with "=== <core> RESULT: <pass>/<total> PASS ===".
 * Cases marked [INFO] only report measurements.
 *
 * The close cases run last: closing the last channel of a driver_id
 * releases its vdev until reboot.
 */

#define FE_STARTUP_DELAY_MS   3000U   /* > BE's, so BE has written the rsc table */
#define BE_STARTUP_DELAY_MS   1000U
#define BE_STOP_MS            5000U
#define REPLY_TIMEOUT_MS      500U
#define LATENCY_ROUNDS        20

#define TEST_TASK_PRIORITY    (tskIDLE_PRIORITY + 1)
#define TEST_TASK_STACK_SIZE  (configMINIMAL_STACK_SIZE * 10)

/*-----------------------------------------------------------
 * Minimal test harness
 *-----------------------------------------------------------*/
static const char *s_core = "";
static int s_pass;
static int s_total;

static void check(const char *id, const char *name, bool ok, const char *detail)
{
    s_total++;
    if (ok) {
        s_pass++;
    }
    printf("[%s] %s %s: %s%s%s\n", s_core, id, name, ok ? "PASS" : "FAIL",
           (detail != NULL) ? " - " : "", (detail != NULL) ? detail : "");
}

static void check_rc(const char *id, const char *name, rpmsg_driver_status_t got, rpmsg_driver_status_t want)
{
    char detail[48];
    snprintf(detail, sizeof(detail), "rc=%d, expected %d", (int)got, (int)want);
    check(id, name, got == want, (got == want) ? NULL : detail);
}

static void summary(void)
{
    printf("=== %s RESULT: %d/%d PASS ===\n", s_core, s_pass, s_total);
}

static uint32_t ms_since(TickType_t t0)
{
    return (uint32_t)((xTaskGetTickCount() - t0) * portTICK_PERIOD_MS);
}

/*-----------------------------------------------------------
 * Helpers
 *-----------------------------------------------------------*/

/* Send, retrying while the peer is not bound yet or has no free buffer. */
static rpmsg_driver_status_t send_retry(rpmsg_driver_channel_t *ch, const void *data,
                                       uint16_t len, uint32_t timeout_ms)
{
    TickType_t t0 = xTaskGetTickCount();
    for (;;) {
        rpmsg_driver_status_t rc = rpmsg_driver_send(ch, data, len);
        if ((rc != RPMSG_DRIVER_ERR_NOT_CONNECTED) && (rc != RPMSG_DRIVER_ERR_BUFFER_FULL)) {
            return rc;
        }
        if (ms_since(t0) >= timeout_ms) {
            return rc;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

/* Drop whatever is waiting in the receive FIFO. */
static void flush(rpmsg_driver_channel_t *ch)
{
    uint8_t buf[RPMSG_DRIVER_MAX_PAYLOAD];
    uint16_t n;
    while (rpmsg_driver_receive(ch, buf, sizeof(buf), &n, 20U) == RPMSG_DRIVER_OK) {
    }
}

/* Send `data` and wait for the echo; true if it matches. */
static bool roundtrip(rpmsg_driver_channel_t *ch, const void *data, uint16_t len)
{
    uint8_t buf[RPMSG_DRIVER_MAX_PAYLOAD];
    uint16_t n = 0U;

    if (send_retry(ch, data, len, REPLY_TIMEOUT_MS) != RPMSG_DRIVER_OK) {
        return false;
    }
    if (rpmsg_driver_receive(ch, buf, sizeof(buf), &n, REPLY_TIMEOUT_MS) != RPMSG_DRIVER_OK) {
        return false;
    }
    return (n == len) && ((len == 0U) || (memcmp(buf, data, len) == 0));
}

/*-----------------------------------------------------------
 * Cases both cores run before any channel is open: no peer needed.
 *-----------------------------------------------------------*/
static void local_param_tests(rpmsg_driver_side_t side)
{
    static rpmsg_driver_channel_t ch;   /* zeroed: never opened */
    uint8_t buf[8];
    uint16_t n;

    rpmsg_driver_open_cfg_t cfg = { .driver_id = RPMSG_DRIVER_ID_0, .side = side };
    check_rc("P01", "open(NULL ch)", rpmsg_driver_open(NULL, cfg), RPMSG_DRIVER_ERR_INVALID_PARAM);

    cfg.side = (rpmsg_driver_side_t)7;
    check_rc("P02", "open(bad side)", rpmsg_driver_open(&ch, cfg), RPMSG_DRIVER_ERR_INVALID_PARAM);

    cfg = (rpmsg_driver_open_cfg_t){ .driver_id = RPMSG_DRIVER_ID_4, .side = side };
    check_rc("P03", "open(unrouted driver_id)", rpmsg_driver_open(&ch, cfg), RPMSG_DRIVER_ERR_INVALID_PARAM);

    cfg = (rpmsg_driver_open_cfg_t){ .driver_id = RPMSG_DRIVER_ID_0, .side = side,
                                   .name = "0123456789abcdef0123456789abcdef" };
    check_rc("P04", "open(name 32 chars)", rpmsg_driver_open(&ch, cfg), RPMSG_DRIVER_ERR_INVALID_PARAM);

    cfg.name = "";
    check_rc("P05", "open(empty name)", rpmsg_driver_open(&ch, cfg), RPMSG_DRIVER_ERR_INVALID_PARAM);

    check_rc("P06", "send(not open)", rpmsg_driver_send(&ch, "x", 1U), RPMSG_DRIVER_ERR_NOT_INITIALIZED);
    check_rc("P07", "receive(not open)",
             rpmsg_driver_receive(&ch, buf, sizeof(buf), &n, RPMSG_DRIVER_NO_WAIT),
             RPMSG_DRIVER_ERR_NOT_INITIALIZED);
    check_rc("P08", "close(not open)", rpmsg_driver_close(&ch), RPMSG_DRIVER_ERR_NOT_INITIALIZED);
    check_rc("P09", "send(NULL ch)", rpmsg_driver_send(NULL, "x", 1U), RPMSG_DRIVER_ERR_INVALID_PARAM);
    check_rc("P10", "receive(NULL ch)",
             rpmsg_driver_receive(NULL, buf, sizeof(buf), &n, RPMSG_DRIVER_NO_WAIT),
             RPMSG_DRIVER_ERR_INVALID_PARAM);
    check_rc("P11", "close(NULL ch)", rpmsg_driver_close(NULL), RPMSG_DRIVER_ERR_INVALID_PARAM);
}

/*===========================================================
 * CORE0 - FE: test driver
 *===========================================================*/
static volatile bool s_fe_unbound;
static void *volatile s_fe_unbind_ctx;

static void fe_on_unbind(rpmsg_driver_channel_t *ch, void *ctx)
{
    (void)ch;
    s_fe_unbind_ctx = ctx;
    s_fe_unbound = true;
}

static void fe_test_task(void *pv)
{
    (void)pv;
    static rpmsg_driver_channel_t smmu;
    static rpmsg_driver_channel_t cam;
    static const char smmu_ctx[] = "smmu";
    uint8_t buf[RPMSG_DRIVER_MAX_PAYLOAD + 1];
    uint8_t big[RPMSG_DRIVER_MAX_PAYLOAD + 1];
    uint16_t n;
    rpmsg_driver_status_t rc;
    char detail[64];

    s_core = "FE";
    local_param_tests(RPMSG_DRIVER_SIDE_FE);

    vTaskDelay(pdMS_TO_TICKS(FE_STARTUP_DELAY_MS));

    /* ---- bring-up ---- */
    rpmsg_driver_open_cfg_t cfg = { .driver_id = RPMSG_DRIVER_ID_0, .side = RPMSG_DRIVER_SIDE_FE,
                                  .on_unbind = fe_on_unbind, .ctx = (void *)smmu_ctx };
    rpmsg_driver_status_t rc_smmu = rpmsg_driver_open(&smmu, cfg);
    check_rc("T01", "open smmu (FE)", rc_smmu, RPMSG_DRIVER_OK);
    cfg = (rpmsg_driver_open_cfg_t){ .driver_id = RPMSG_DRIVER_ID_3, .side = RPMSG_DRIVER_SIDE_FE };
    rpmsg_driver_status_t rc_cam = rpmsg_driver_open(&cam, cfg);
    check_rc("T02", "open camera (FE)", rc_cam, RPMSG_DRIVER_OK);
    if ((rc_smmu != RPMSG_DRIVER_OK) || (rc_cam != RPMSG_DRIVER_OK)) {
        printf("[FE] cannot continue without both channels\n");
        summary();
        vTaskDelete(NULL);
        return;
    }

    /* NS channel: NOT_CONNECTED until BE's name-service message arrives. */
    TickType_t t0 = xTaskGetTickCount();
    int not_connected = 0;
    do {
        rc = rpmsg_driver_send(&smmu, "hi", 2U);
        if (rc == RPMSG_DRIVER_ERR_NOT_CONNECTED) {
            not_connected++;
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    } while ((rc == RPMSG_DRIVER_ERR_NOT_CONNECTED) && (ms_since(t0) < 3000U));
    snprintf(detail, sizeof(detail), "bound after %lu ms, %d NOT_CONNECTED",
             (unsigned long)ms_since(t0), not_connected);
    check("T03", "smmu (NS) binds", rc == RPMSG_DRIVER_OK, detail);
    flush(&smmu);

    /* Fixed-address channel: no binding step. */
    check_rc("T04", "camera (FIXED) first send", rpmsg_driver_send(&cam, "hi", 2U), RPMSG_DRIVER_OK);
    flush(&cam);

    /* ---- data path ---- */
    check("T05", "smmu round trip", roundtrip(&smmu, "ping", 4U), NULL);
    check("T06", "camera round trip", roundtrip(&cam, "ping", 4U), NULL);
    check("T07", "zero-length message", roundtrip(&smmu, "", 0U), NULL);

    for (int i = 0; i < (int)RPMSG_DRIVER_MAX_PAYLOAD; i++) {
        big[i] = (uint8_t)(i + 1);
    }
    check("T08", "max payload (120 B) round trip", roundtrip(&smmu, big, RPMSG_DRIVER_MAX_PAYLOAD), NULL);

    check_rc("T09", "send 121 B", rpmsg_driver_send(&smmu, big, RPMSG_DRIVER_MAX_PAYLOAD + 1U),
             RPMSG_DRIVER_ERR_PAYLOAD_TOO_LARGE);
    check_rc("T10", "send(NULL data, len 5)", rpmsg_driver_send(&smmu, NULL, 5U),
             RPMSG_DRIVER_ERR_INVALID_PARAM);
    check_rc("T11", "receive(NULL buf, size 5)",
             rpmsg_driver_receive(&smmu, NULL, 5U, &n, RPMSG_DRIVER_NO_WAIT), RPMSG_DRIVER_ERR_INVALID_PARAM);

    /* ---- receive semantics ---- */
    t0 = xTaskGetTickCount();
    rc = rpmsg_driver_receive(&smmu, buf, sizeof(buf), &n, RPMSG_DRIVER_NO_WAIT);
    snprintf(detail, sizeof(detail), "rc=%d in %lu ms", (int)rc, (unsigned long)ms_since(t0));
    check("T12", "receive NO_WAIT, empty -> TIMEOUT at once",
          (rc == RPMSG_DRIVER_ERR_TIMEOUT) && (ms_since(t0) <= 2U), detail);

    t0 = xTaskGetTickCount();
    rc = rpmsg_driver_receive(&smmu, buf, sizeof(buf), &n, 50U);
    uint32_t dt = ms_since(t0);
    snprintf(detail, sizeof(detail), "rc=%d after %lu ms", (int)rc, (unsigned long)dt);
    check("T13", "receive 50 ms, empty -> TIMEOUT after ~50 ms",
          (rc == RPMSG_DRIVER_ERR_TIMEOUT) && (dt >= 50U) && (dt <= 70U), detail);

    t0 = xTaskGetTickCount();
    rc = rpmsg_driver_receive(&cam, buf, sizeof(buf), &n, 50U);
    dt = ms_since(t0);
    snprintf(detail, sizeof(detail), "rc=%d after %lu ms", (int)rc, (unsigned long)dt);
    check("T14", "receive 50 ms on polled channel -> TIMEOUT ~50 ms",
          (rc == RPMSG_DRIVER_ERR_TIMEOUT) && (dt >= 50U) && (dt <= 70U), detail);

    rc = send_retry(&smmu, "0123456789", 10U, REPLY_TIMEOUT_MS);
    n = 0xFFFFU;
    if (rc == RPMSG_DRIVER_OK) {
        rc = rpmsg_driver_receive(&smmu, buf, 4U, &n, REPLY_TIMEOUT_MS);
    }
    snprintf(detail, sizeof(detail), "rc=%d out_len=%u", (int)rc, (unsigned)n);
    check("T15", "receive into 4-byte buffer -> PAYLOAD_TOO_LARGE, 4 bytes copied",
          (rc == RPMSG_DRIVER_ERR_PAYLOAD_TOO_LARGE) && (n == 4U) && (memcmp(buf, "0123", 4U) == 0), detail);

    rc = send_retry(&smmu, "abc", 3U, REPLY_TIMEOUT_MS);
    if (rc == RPMSG_DRIVER_OK) {
        rc = rpmsg_driver_receive(&smmu, buf, sizeof(buf), NULL, REPLY_TIMEOUT_MS);
    }
    check_rc("T16", "receive with out_len NULL", rc, RPMSG_DRIVER_OK);

    /* A burst smaller than the receive FIFO is delivered in order. */
    rc = send_retry(&smmu, "!burst3", 7U, REPLY_TIMEOUT_MS);
    static const char replies[][3] = { "b1", "b2", "b3" };
    bool burst_ok = (rc == RPMSG_DRIVER_OK);
    for (size_t i = 0; burst_ok && (i < sizeof(replies) / sizeof(replies[0])); ++i) {
        rc = rpmsg_driver_receive(&smmu, buf, sizeof(buf), &n, REPLY_TIMEOUT_MS);
        burst_ok = (rc == RPMSG_DRIVER_OK) && (n == 2U) && (memcmp(buf, replies[i], n) == 0);
    }
    check("T17", "burst of 3 -> all received in order", burst_ok, NULL);

    /* ---- latency (info) ---- */
    for (int c = 0; c < 2; c++) {
        rpmsg_driver_channel_t *ch = (c == 0) ? &smmu : &cam;
        uint32_t sum = 0U, max = 0U;
        int ok = 0;
        for (int i = 0; i < LATENCY_ROUNDS; i++) {
            t0 = xTaskGetTickCount();
            if (roundtrip(ch, "lat", 3U)) {
                uint32_t d = ms_since(t0);
                sum += d;
                max = (d > max) ? d : max;
                ok++;
            }
        }
        printf("[FE] [INFO] round trip %s: %d/%d ok, avg %lu ms, max %lu ms\n",
               (c == 0) ? "smmu (MFIS)" : "camera (polled)", ok, LATENCY_ROUNDS,
               (unsigned long)((ok > 0) ? (sum / (uint32_t)ok) : 0U), (unsigned long)max);
    }

    /* ---- backpressure: BE stops reading, FE floods until BUFFER_FULL ---- */
    rc = send_retry(&smmu, "!stop", 5U, REPLY_TIMEOUT_MS);
    vTaskDelay(pdMS_TO_TICKS(100));
    int sent = 0;
    rpmsg_driver_status_t last = RPMSG_DRIVER_OK;
    for (int i = 0; i < 400; i++) {
        char msg[8];
        snprintf(msg, sizeof(msg), "f%d", i);
        last = rpmsg_driver_send(&smmu, msg, (uint16_t)strlen(msg));
        if (last != RPMSG_DRIVER_OK) {
            break;
        }
        sent++;
    }
    snprintf(detail, sizeof(detail), "BUFFER_FULL after %d sends (last rc=%d)", sent, (int)last);
    check("T18", "flood without reader -> BUFFER_FULL",
          (rc == RPMSG_DRIVER_OK) && (last == RPMSG_DRIVER_ERR_BUFFER_FULL) && (sent >= 100), detail);

    /* BE resumes after BE_STOP_MS and echoes the messages retained in its FIFO. */
    t0 = xTaskGetTickCount();
    rc = send_retry(&smmu, "after", 5U, BE_STOP_MS + 3000U);
    snprintf(detail, sizeof(detail), "rc=%d after %lu ms", (int)rc, (unsigned long)ms_since(t0));
    check("T19", "send works again once BE reads", rc == RPMSG_DRIVER_OK, detail);
    vTaskDelay(pdMS_TO_TICKS(200));
    flush(&smmu);

    /* ---- peer disconnect (NS channel) ---- */
    s_fe_unbound = false;
    rc = send_retry(&smmu, "!close", 6U, REPLY_TIMEOUT_MS);
    t0 = xTaskGetTickCount();
    rpmsg_driver_status_t rrc;
    do {
        rrc = rpmsg_driver_receive(&smmu, buf, sizeof(buf), &n, 100U);
    } while ((rrc != RPMSG_DRIVER_ERR_PEER_DISCONNECTED) && (ms_since(t0) < 3000U));
    check_rc("T20", "receive after BE closed -> PEER_DISCONNECTED", rrc, RPMSG_DRIVER_ERR_PEER_DISCONNECTED);
    check("T21", "on_unbind called with cfg.ctx",
          s_fe_unbound && (s_fe_unbind_ctx == (void *)smmu_ctx), NULL);
    check_rc("T22", "send after BE closed", rpmsg_driver_send(&smmu, "x", 1U),
             RPMSG_DRIVER_ERR_PEER_DISCONNECTED);

    /* ---- close / release (last: final until reboot) ---- */
    check_rc("T23", "close smmu", rpmsg_driver_close(&smmu), RPMSG_DRIVER_OK);
    check_rc("T24", "close smmu twice", rpmsg_driver_close(&smmu), RPMSG_DRIVER_ERR_NOT_INITIALIZED);
    cfg = (rpmsg_driver_open_cfg_t){ .driver_id = RPMSG_DRIVER_ID_0, .side = RPMSG_DRIVER_SIDE_FE };
    check_rc("T25", "reopen smmu after its vdev was released", rpmsg_driver_open(&smmu, cfg),
             RPMSG_DRIVER_ERR_NOT_INITIALIZED);

    check("T26", "camera still works after smmu vdev released", roundtrip(&cam, "cam", 3U), NULL);
    check_rc("T27", "close camera", rpmsg_driver_close(&cam), RPMSG_DRIVER_OK);
    cfg = (rpmsg_driver_open_cfg_t){ .driver_id = RPMSG_DRIVER_ID_3, .side = RPMSG_DRIVER_SIDE_FE };
    check_rc("T28", "reopen camera after its vdev was released", rpmsg_driver_open(&cam, cfg),
             RPMSG_DRIVER_ERR_NOT_INITIALIZED);

    summary();
    vTaskDelete(NULL);
}

/*===========================================================
 * CORE1 - BE: echo servers
 *===========================================================*/
static volatile bool s_be_param_done;

static void be_send(rpmsg_driver_channel_t *ch, const void *data, uint16_t len)
{
    rpmsg_driver_status_t rc = send_retry(ch, data, len, REPLY_TIMEOUT_MS);
    if (rc != RPMSG_DRIVER_OK) {
        printf("[BE] send failed rc=%d\n", (int)rc);
    }
}

static void be_smmu_task(void *pv)
{
    (void)pv;
    static rpmsg_driver_channel_t smmu;
    uint8_t buf[RPMSG_DRIVER_MAX_PAYLOAD];
    uint16_t n;

    s_core = "BE";
    local_param_tests(RPMSG_DRIVER_SIDE_BE);
    s_be_param_done = true;

    vTaskDelay(pdMS_TO_TICKS(BE_STARTUP_DELAY_MS));
    rpmsg_driver_open_cfg_t cfg = { .driver_id = RPMSG_DRIVER_ID_0, .side = RPMSG_DRIVER_SIDE_BE };
    check_rc("B01", "open smmu (BE, waits for FE)", rpmsg_driver_open(&smmu, cfg), RPMSG_DRIVER_OK);

    for (;;) {
        rpmsg_driver_status_t rc = rpmsg_driver_receive(&smmu, buf, sizeof(buf), &n, RPMSG_DRIVER_WAIT_FOREVER);
        if ((rc != RPMSG_DRIVER_OK) && (rc != RPMSG_DRIVER_ERR_PAYLOAD_TOO_LARGE)) {
            printf("[BE] smmu receive rc=%d\n", (int)rc);
            continue;
        }

        if ((n == 7U) && (memcmp(buf, "!burst3", 7U) == 0)) {
            be_send(&smmu, "b1", 2U);
            be_send(&smmu, "b2", 2U);
            be_send(&smmu, "b3", 2U);
        } else if ((n == 5U) && (memcmp(buf, "!stop", 5U) == 0)) {
            printf("[BE] not reading smmu for %u ms\n", (unsigned)BE_STOP_MS);
            vTaskDelay(pdMS_TO_TICKS(BE_STOP_MS));
        } else if ((n == 6U) && (memcmp(buf, "!close", 6U) == 0)) {
            check_rc("B02", "close smmu (BE)", rpmsg_driver_close(&smmu), RPMSG_DRIVER_OK);
            check_rc("B03", "reopen smmu after its vdev was released (BE)",
                     rpmsg_driver_open(&smmu, cfg), RPMSG_DRIVER_ERR_NOT_INITIALIZED);
            break;
        } else {
            be_send(&smmu, buf, n);
        }
    }

    summary();
    vTaskDelete(NULL);
}

/* Echoes on the polled, fixed-address channel. Never told when FE closes
 * it (no name service): keeps waiting, by design. */
static void be_camera_task(void *pv)
{
    (void)pv;
    static rpmsg_driver_channel_t cam;
    uint8_t buf[RPMSG_DRIVER_MAX_PAYLOAD];
    uint16_t n;

    while (!s_be_param_done) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    vTaskDelay(pdMS_TO_TICKS(BE_STARTUP_DELAY_MS));
    rpmsg_driver_open_cfg_t cfg = { .driver_id = RPMSG_DRIVER_ID_3, .side = RPMSG_DRIVER_SIDE_BE };
    if (rpmsg_driver_open(&cam, cfg) != RPMSG_DRIVER_OK) {
        printf("[BE] open camera FAILED\n");
        vTaskDelete(NULL);
        return;
    }

    for (;;) {
        if (rpmsg_driver_receive(&cam, buf, sizeof(buf), &n, RPMSG_DRIVER_WAIT_FOREVER) == RPMSG_DRIVER_OK) {
            be_send(&cam, buf, n);
        }
    }
}

/*-----------------------------------------------------------*/
static void prvSetupHardware(void)
{
    portDISABLE_INTERRUPTS();
    Irq_Setup();
    (void)pfcInitModules(getModuleConfigs());
}

int main(void)
{
    prvSetupHardware();

    if (R_UTILS_GetCpuID() == 0U) {
        printf("=== CORE0 : rcar_rpmsg API test (FE) ===\n");
        xTaskCreate(fe_test_task, "FE_TEST", TEST_TASK_STACK_SIZE, NULL, TEST_TASK_PRIORITY, NULL);
    } else {
        printf("=== CORE1 : rcar_rpmsg API test (BE) ===\n");
        xTaskCreate(be_smmu_task, "BE_SMMU", TEST_TASK_STACK_SIZE, NULL, TEST_TASK_PRIORITY, NULL);
        xTaskCreate(be_camera_task, "BE_CAM", TEST_TASK_STACK_SIZE, NULL, TEST_TASK_PRIORITY, NULL);
    }

    vTaskStartScheduler();
    for (;;) {
    }
    return 0;
}

/*-----------------------------------------------------------*/
int printf_raw(const char *format, ...);

void vMainAssertCalled(const char *pcFileName, uint32_t ulLineNumber)
{
    printf_raw("ASSERT!  Line %d of file %s\n", ulLineNumber, pcFileName);
    taskENTER_CRITICAL();
    for (;;) {
    }
}

void vDeleteCallingTask(void)
{
    vTaskDelete(NULL);
}
