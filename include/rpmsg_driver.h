/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef RPMSG_DRIVER_H
#define RPMSG_DRIVER_H

#include <stdint.h>
#include <stdalign.h>

/* Keep the status values used by the original driver. */
typedef enum {
    RPMSG_DRIVER_OK = 0,
    RPMSG_DRIVER_ERR_INVALID_PARAM,
    RPMSG_DRIVER_ERR_NOT_INITIALIZED,
    RPMSG_DRIVER_ERR_PAYLOAD_TOO_LARGE = 6,
    RPMSG_DRIVER_ERR_BUFFER_FULL,
    RPMSG_DRIVER_ERR_TIMEOUT,
    RPMSG_DRIVER_ERR_TRANSPORT,
    RPMSG_DRIVER_ERR_INTERNAL,
    RPMSG_DRIVER_ERR_PEER_DISCONNECTED,
    RPMSG_DRIVER_ERR_NOT_CONNECTED,       /* peer not bound yet */
} rpmsg_driver_status_t;

/* Driver IDs are mapped to transport routes by the platform. */
typedef enum {
    RPMSG_DRIVER_ID_0 = 0,
    RPMSG_DRIVER_ID_1 = 1,
    RPMSG_DRIVER_ID_2 = 2,
    RPMSG_DRIVER_ID_3 = 3,
    RPMSG_DRIVER_ID_4 = 4,
    RPMSG_DRIVER_ID_5 = 5,

    /* Same services on the other chip, over UCIe CH0: 0x10 + local id. */
    RPMSG_DRIVER_ID_6 = 0x10,

    /* General-purpose A/R IPC over the local UIO link. */
    RPMSG_DRIVER_ID_7 = 7,
} rpmsg_driver_id_t;

/*
 * Inter-core messaging per driver_id over RPMsg.
 * One channel = one endpoint = one driver_id. Messages are processed
 * inside send()/receive(); no task, no RTOS call.
 *
 * Rules:
 *  1. One task calls receive() per channel; any task may send().
 *  2. Keep calling send()/receive() while a channel is open, or the
 *     peer's send() ends with RPMSG_DRIVER_ERR_BUFFER_FULL.
 *  3. Receive FIFO per channel: when full, a new message overwrites the oldest unread one.
 *  4. on_unbind may run in another task: keep it short.
 *  5. On BE, open() blocks until FE is up.
 */

/* Max payload per message, in bytes. */
#define RPMSG_DRIVER_MAX_PAYLOAD   (120U)

/* Number of unread messages retained per channel. */
#define RPMSG_DRIVER_RX_QUEUE_DEPTH  (5U)

/* rpmsg_driver_receive() timeout_ms values. */
#define RPMSG_DRIVER_NO_WAIT       (0U)
#define RPMSG_DRIVER_WAIT_FOREVER  UINT32_MAX

/* Role of this core for a driver_id: FE on one core, BE on the other. */
typedef enum {
    RPMSG_DRIVER_SIDE_FE = 0,
    RPMSG_DRIVER_SIDE_BE,
} rpmsg_driver_side_t;

/* Opaque channel. Allocate with static storage or initialize to zero. */
typedef struct rpmsg_driver_channel rpmsg_driver_channel_t;

/* Channel private storage size. */
#define RPMSG_DRIVER_CH_STORAGE_SIZE  (512U)

struct rpmsg_driver_channel {
    alignas(8) uint8_t priv[RPMSG_DRIVER_CH_STORAGE_SIZE];
};

/* Peer destroyed its endpoint. `ctx` is cfg.ctx. */
typedef void (*rpmsg_driver_unbind_handler_t)(rpmsg_driver_channel_t *ch, void *ctx);

/* Config for rpmsg_driver_open(). */
typedef struct {
    rpmsg_driver_id_t               driver_id;
    rpmsg_driver_side_t             side;
    const char                     *name;      /* NULL: default. < 32 bytes. Same on peer. */
    rpmsg_driver_unbind_handler_t    on_unbind;  /* optional */
    void                           *ctx;       /* passed to on_unbind */
} rpmsg_driver_open_cfg_t;

/*
 * Open a channel. On BE, blocks until FE is up.
 * RPMSG_DRIVER_ERR_INVALID_PARAM: bad driver_id, side or name.
 * RPMSG_DRIVER_ERR_NOT_INITIALIZED: vdev released by an earlier last close().
 * RPMSG_DRIVER_ERR_INTERNAL: receive FIFO allocation failed.
 */
rpmsg_driver_status_t rpmsg_driver_open(rpmsg_driver_channel_t *ch,
                                       rpmsg_driver_open_cfg_t cfg);

/*
 * Close a channel. Closing the last channel on a vdev releases the vdev
 * until reboot. RPMSG_DRIVER_ERR_NOT_INITIALIZED: not open.
 */
rpmsg_driver_status_t rpmsg_driver_close(rpmsg_driver_channel_t *ch);

/*
 * Send `len` bytes. Never blocks.
 *   RPMSG_DRIVER_ERR_NOT_CONNECTED     peer not bound yet, retry
 *   RPMSG_DRIVER_ERR_BUFFER_FULL       peer not reading, retry
 *   RPMSG_DRIVER_ERR_PEER_DISCONNECTED peer closed
 *   RPMSG_DRIVER_ERR_PAYLOAD_TOO_LARGE len > RPMSG_DRIVER_MAX_PAYLOAD
 */
rpmsg_driver_status_t rpmsg_driver_send(rpmsg_driver_channel_t *ch,
                                       const void *data, uint16_t len);

/*
 * Receive one message into `buf`; `*out_len` = bytes copied (may be NULL).
 * RPMSG_DRIVER_OK only when a message was received.
 *   RPMSG_DRIVER_ERR_TIMEOUT           nothing within timeout_ms (also NO_WAIT)
 *   RPMSG_DRIVER_ERR_PAYLOAD_TOO_LARGE truncated to buf_size
 *   RPMSG_DRIVER_ERR_PEER_DISCONNECTED peer closed
 */
rpmsg_driver_status_t rpmsg_driver_receive(rpmsg_driver_channel_t *ch,
                                          void *buf, uint16_t buf_size,
                                          uint16_t *out_len, uint32_t timeout_ms);

#endif /* RPMSG_DRIVER_H */
