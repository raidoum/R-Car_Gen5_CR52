/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: MIT
 */

#include <openamp/open_amp.h>
#include <metal/alloc.h>
#include <metal/atomic.h>
#include <metal/sleep.h>
#include <metal/time.h>
#include <metal/utilities.h>

#include "rpmsg_driver.h"
#include "rcar_msg.h"
#include "rpmsg_driver_os.h"
#include "rpmsg_platform.h"

#include <errno.h>
#include <string.h>

/* Logging Function include. */
#include "logging_stack.h"

/*
 * No RTOS calls: waits use metal_sleep_usec() or the OS hooks, locks are
 * atomics. No background task: send()/receive() drain their vdev first.
 */

/* Poll interval while waiting. */
#define RPMSG_DRIVER_POLL_INTERVAL_US   1000U

/* Interrupt-driven vdev: max time between drains, max event wait. */
#define RPMSG_DRIVER_SAFETY_DRAIN_MS    10U

/* Channels per vdev with a wake-up event; others poll. */
#define RPMSG_DRIVER_MAX_CH_PER_VDEV    4U

/* Max finite timeout (32-bit usec timestamp). */
#define RPMSG_DRIVER_MAX_TIMEOUT_MS     (0xFFFFFFFFU / 1000U)

/* Fixed addresses: one (BE, FE) pair per vdev. */
#define RPMSG_DRIVER_FIXED_ADDR_BASE    0x100U
#define RPMSG_DRIVER_FIXED_BE_ADDR(ch)  (RPMSG_DRIVER_FIXED_ADDR_BASE + ((uint32_t)(ch) * 2U))
#define RPMSG_DRIVER_FIXED_FE_ADDR(ch)  (RPMSG_DRIVER_FIXED_BE_ADDR(ch) + 1U)

/* vdev state. CLOSED is final: the peer is not reset. */
enum {
    VDEV_NONE = 0,
    VDEV_CREATING,
    VDEV_READY,
    VDEV_CLOSING,
    VDEV_CLOSED,
};

/* ev_used[] values. */
enum {
    EV_FREE = 0,
    EV_CLAIMED,
    EV_ACTIVE,       /* signalled by the ISR */
};

typedef struct {
    atomic_int            state;
    atomic_int            drain_lock;
    struct remoteproc    *platform;
    struct rpmsg_device  *rdev;
    unsigned int          open_count;    /* under drain_lock */

    bool                  has_notify;
    atomic_int            pending;       /* set by the ISR */
    uint32_t              last_drain_us; /* under drain_lock */

    /* Wake-up events (OS hooks only). Never destroyed: the ISR may still
     * signal a released slot. */
    void                 *ev[RPMSG_DRIVER_MAX_CH_PER_VDEV];
    atomic_int            ev_used[RPMSG_DRIVER_MAX_CH_PER_VDEV];
} rpmsg_driver_vdev_t;

static rpmsg_driver_vdev_t s_vdev[PLATFORM_LINK_MAX][2][PLATFORM_VDEV_MAX];   /* [link][side][vdev] */
static atomic_int        s_platform_lock[PLATFORM_LINK_MAX];
static const rpmsg_driver_os_hooks_t *const s_hooks = &rpmsg_driver_os_hooks;

/* Channel state, stored in rpmsg_driver_channel_t::priv[]. */
typedef struct {
    struct rpmsg_endpoint           ept;
    rpmsg_driver_channel_t         *ch;         /* owner, for on_unbind */
    platform_link_t                 link;
    rpmsg_driver_side_t             side;
    unsigned int                    vdev;
    int                             ev_slot;    /* or -1 */
    rpmsg_driver_id_t               driver_id;
    rpmsg_driver_unbind_handler_t    on_unbind;
    void                           *ctx;
    volatile bool                   is_open;
    volatile bool                   peer_gone;
    /* Receive FIFO, under drain_lock. */
    rcar_msg_t                     *rx_queue;
    unsigned int                    rx_head;
    unsigned int                    rx_count;
} rpmsg_ch_impl_t;

_Static_assert(sizeof(rpmsg_ch_impl_t) <= sizeof(((rpmsg_driver_channel_t *)0)->priv),
               "rpmsg_driver_channel_t::priv[] too small for rpmsg_ch_impl_t");
_Static_assert((size_t)alignof(rpmsg_ch_impl_t) <= (size_t)alignof(rpmsg_driver_channel_t),
               "rpmsg_driver_channel_t::priv[] alignment too small for rpmsg_ch_impl_t");
_Static_assert(RPMSG_DRIVER_MAX_PAYLOAD <= RCAR_MSG_PAYLOAD_MAX_SIZE,
               "RPMSG_DRIVER_MAX_PAYLOAD exceeds the wire frame payload");

static inline rpmsg_ch_impl_t *ch_impl(rpmsg_driver_channel_t *ch)
{
    return (rpmsg_ch_impl_t *)ch->priv;
}

static inline rpmsg_driver_vdev_t *ch_vdev(const rpmsg_ch_impl_t *c)
{
    return &s_vdev[c->link][c->side][c->vdev];
}

/* Locks: waiters sleep, never spin. */
static inline bool try_lock(atomic_int *lock)
{
    return atomic_exchange_explicit(lock, 1, memory_order_acquire) == 0;
}

static inline void lock(atomic_int *lock)
{
    while (!try_lock(lock)) {
        metal_sleep_usec(RPMSG_DRIVER_POLL_INTERVAL_US);
    }
}

static inline void unlock(atomic_int *lock)
{
    atomic_store_explicit(lock, 0, memory_order_release);
}

/* ms since start_us, wrap-safe. */
static inline uint32_t elapsed_ms(uint32_t start_us)
{
    return ((uint32_t)metal_get_timestamp() - start_us) / 1000U;
}

/* One lock per link lets independent links initialize concurrently. */
static rpmsg_driver_status_t get_platform(platform_link_t link, rpmsg_driver_side_t side,
                                          struct remoteproc **out)
{
    virtio_type_t type = (side == RPMSG_DRIVER_SIDE_BE) ? VIRTIO_BACK_END : VIRTIO_FRONT_END;

    lock(&s_platform_lock[link]);
    int ret = rpmsg_platform_init(link, type, out);
    unlock(&s_platform_lock[link]);

    return (ret == 0) ? RPMSG_DRIVER_OK : RPMSG_DRIVER_ERR_TRANSPORT;
}

/* Notification ISR: flag the vdev, wake its channels. */
static void vdev_notify_isr(void *arg)
{
    rpmsg_driver_vdev_t *v = (rpmsg_driver_vdev_t *)arg;
    const rpmsg_driver_os_hooks_t *hooks = s_hooks;

    atomic_store_explicit(&v->pending, 1, memory_order_release);
    if (hooks == NULL) {
        return;
    }
    for (unsigned int i = 0; i < RPMSG_DRIVER_MAX_CH_PER_VDEV; i++) {
        if (atomic_load_explicit(&v->ev_used[i], memory_order_acquire) == EV_ACTIVE) {
            hooks->event_signal(v->ev[i]);
        }
    }
}

/* Bring up the vdev once; other callers wait. Blocks on BE until FE is up. */
static rpmsg_driver_status_t ensure_vdev(rpmsg_driver_side_t side, const platform_rpmsg_route_t *route)
{
    rpmsg_driver_vdev_t *v = &s_vdev[route->link][side][route->vdev];

    for (;;) {
        int st = atomic_load_explicit(&v->state, memory_order_acquire);
        if (st == VDEV_READY) {
            return RPMSG_DRIVER_OK;
        }
        if ((st == VDEV_CLOSING) || (st == VDEV_CLOSED)) {
            return RPMSG_DRIVER_ERR_NOT_INITIALIZED;
        }

        int expected = VDEV_NONE;
        if ((st == VDEV_NONE) &&
            atomic_compare_exchange_strong(&v->state, &expected, VDEV_CREATING)) {
            struct remoteproc   *platform = NULL;
            struct rpmsg_device *rdev = NULL;

            v->has_notify = (route->notify_ch != PLATFORM_NO_NOTIFY);

            rpmsg_driver_status_t rc = get_platform(route->link, side, &platform);
            if (rc == RPMSG_DRIVER_OK) {
                int ret = platform_add_vdev(platform, route->vdev, route->notify_ch,
                                            vdev_notify_isr, v, &rdev);
                if (ret == -ENODEV) {
                    rc = RPMSG_DRIVER_ERR_INVALID_PARAM;   /* not in rsc table */
                } else if (ret != 0) {
                    rc = RPMSG_DRIVER_ERR_TRANSPORT;
                } else {
                    LogDebug(("Platform added vdev successfully\n"));
                }
            }

            if (rc != RPMSG_DRIVER_OK) {
                atomic_store_explicit(&v->state, VDEV_NONE, memory_order_release);
                return rc;
            }
            v->platform = platform;
            v->rdev     = rdev;
            atomic_store_explicit(&v->state, VDEV_READY, memory_order_release);
            return RPMSG_DRIVER_OK;
        }

        metal_sleep_usec(RPMSG_DRIVER_POLL_INTERVAL_US);
    }
}

/* Release the vdev if it has no open channel. Takes every drain_lock of
 * the side of the link, in index order: drains walk the vdev list being
 * edited. */
static void release_vdev(platform_link_t link, rpmsg_driver_side_t side, unsigned int vdev)
{
    rpmsg_driver_vdev_t *v = &s_vdev[link][side][vdev];

    for (unsigned int i = 0; i < PLATFORM_VDEV_MAX; i++) {
        lock(&s_vdev[link][side][i].drain_lock);
    }

    /* An open() may have come in meanwhile. */
    if ((atomic_load_explicit(&v->state, memory_order_acquire) == VDEV_READY) &&
        (v->open_count == 0U)) {
        atomic_store_explicit(&v->state, VDEV_CLOSING, memory_order_release);
        (void)platform_remove_vdev(v->platform, vdev);
        v->rdev = NULL;
        atomic_store_explicit(&v->state, VDEV_CLOSED, memory_order_release);
    }

    for (unsigned int i = PLATFORM_VDEV_MAX; i > 0U; i--) {
        unlock(&s_vdev[link][side][i - 1U].drain_lock);
    }
}

/* Caller holds drain_lock. Interrupt-driven vdev: drain on interrupt or every
 * RPMSG_DRIVER_SAFETY_DRAIN_MS. */
static void drain_locked(rpmsg_driver_vdev_t *v, unsigned int vdev)
{
    if (v->has_notify) {
        bool pending = atomic_exchange_explicit(&v->pending, 0, memory_order_acquire) != 0;
        uint32_t now_us = (uint32_t)metal_get_timestamp();
        if (!pending && ((now_us - v->last_drain_us) < (RPMSG_DRIVER_SAFETY_DRAIN_MS * 1000U))) {
            return;
        }
        v->last_drain_us = now_us;
    }
    (void)platform_drain_vdev(v->platform, vdev);
}

/* Give an interrupt-driven channel a wake-up event if OS hooks are present. */
static void attach_event(rpmsg_ch_impl_t *c, rpmsg_driver_vdev_t *v)
{
    const rpmsg_driver_os_hooks_t *hooks = s_hooks;

    c->ev_slot = -1;
    if (!v->has_notify || (hooks == NULL)) {
        return;
    }
    for (unsigned int i = 0; i < RPMSG_DRIVER_MAX_CH_PER_VDEV; i++) {
        int expected = EV_FREE;
        if (!atomic_compare_exchange_strong(&v->ev_used[i], &expected, EV_CLAIMED)) {
            continue;
        }
        if (v->ev[i] == NULL) {
            v->ev[i] = hooks->event_create();
        }
        if (v->ev[i] == NULL) {
            atomic_store_explicit(&v->ev_used[i], EV_FREE, memory_order_release);
            return;
        }
        c->ev_slot = (int)i;
        atomic_store_explicit(&v->ev_used[i], EV_ACTIVE, memory_order_release);
        return;
    }
}

static void detach_event(rpmsg_ch_impl_t *c, rpmsg_driver_vdev_t *v)
{
    if (c->ev_slot >= 0) {
        atomic_store_explicit(&v->ev_used[c->ev_slot], EV_FREE, memory_order_release);
        c->ev_slot = -1;
    }
}

/* Endpoint callbacks: run inside a drain, in the draining task. */
static int rpmsg_driver_rx_cb(struct rpmsg_endpoint *ept, void *data, size_t len,
                             uint32_t src, void *priv)
{
    (void)src;
    (void)priv;

    rpmsg_ch_impl_t *c = metal_container_of(ept, rpmsg_ch_impl_t, ept);
    if ((data == NULL) || (len < sizeof(rcar_msg_header_t))) {
        return RPMSG_SUCCESS;
    }

    unsigned int tail = (c->rx_head + c->rx_count) % RPMSG_DRIVER_RX_QUEUE_DEPTH;
    rcar_msg_t *message = &c->rx_queue[tail];
    size_t n = metal_min(len, sizeof(*message));
    (void)memcpy(message, data, n);

    /* Clamp payload_len: the peer may not send an rcar_msg_t frame. */
    size_t max_payload = 0U;
    if (n > (size_t)offsetof(rcar_msg_t, payload)) {
        max_payload = n - (size_t)offsetof(rcar_msg_t, payload);
    }
    if (message->hdr.payload_len > max_payload) {
        message->hdr.payload_len = (uint16_t)max_payload;
    }

    if (c->rx_count == RPMSG_DRIVER_RX_QUEUE_DEPTH) {
        /* The tail wrapped to the oldest slot, which now contains the new message. */
        c->rx_head = (c->rx_head + 1U) % RPMSG_DRIVER_RX_QUEUE_DEPTH;
    } else {
        ++c->rx_count;
    }
    return RPMSG_SUCCESS;
}

static void rpmsg_driver_unbind_cb(struct rpmsg_endpoint *ept)
{
    rpmsg_ch_impl_t *c = metal_container_of(ept, rpmsg_ch_impl_t, ept);

    c->peer_gone = true;
    if (c->on_unbind != NULL) {
        c->on_unbind(c->ch, c->ctx);
    }
}

/* Public API */
rpmsg_driver_status_t rpmsg_driver_open(rpmsg_driver_channel_t *ch,
                                       rpmsg_driver_open_cfg_t cfg)
{
    if ((ch == NULL) ||
        ((cfg.side != RPMSG_DRIVER_SIDE_FE) && (cfg.side != RPMSG_DRIVER_SIDE_BE))) {
        return RPMSG_DRIVER_ERR_INVALID_PARAM;
    }

    platform_rpmsg_route_t route;
    if (!platform_get_rpmsg_route(cfg.driver_id, &route)) {
        return RPMSG_DRIVER_ERR_INVALID_PARAM;
    }

    const char *name = (cfg.name != NULL) ? cfg.name : route.default_name;
    if ((name[0] == '\0') || (strlen(name) >= RPMSG_NAME_SIZE)) {
        return RPMSG_DRIVER_ERR_INVALID_PARAM;
    }

    rcar_msg_t *rx_queue = metal_allocate_memory(RPMSG_DRIVER_RX_QUEUE_DEPTH * sizeof(*rx_queue));
    if (rx_queue == NULL) {
        return RPMSG_DRIVER_ERR_INTERNAL;
    }

    rpmsg_driver_status_t rc = ensure_vdev(cfg.side, &route);
    if (rc != RPMSG_DRIVER_OK) {
        metal_free_memory(rx_queue);
        return rc;
    }

    rpmsg_ch_impl_t *c = ch_impl(ch);
    (void)memset(c, 0, sizeof(*c));
    c->ch        = ch;
    c->link      = route.link;
    c->side      = cfg.side;
    c->vdev      = route.vdev;
    c->ev_slot   = -1;
    c->driver_id = cfg.driver_id;
    c->on_unbind = cfg.on_unbind;
    c->ctx       = cfg.ctx;

    uint32_t src = RPMSG_ADDR_ANY;
    uint32_t dst = RPMSG_ADDR_ANY;
    if (route.fixed_addresses) {
        bool be = (cfg.side == RPMSG_DRIVER_SIDE_BE);
        src = be ? RPMSG_DRIVER_FIXED_BE_ADDR(route.vdev) : RPMSG_DRIVER_FIXED_FE_ADDR(route.vdev);
        dst = be ? RPMSG_DRIVER_FIXED_FE_ADDR(route.vdev) : RPMSG_DRIVER_FIXED_BE_ADDR(route.vdev);
    }

    /* Under drain_lock: a concurrent last close() cannot release the vdev. */
    rpmsg_driver_vdev_t *v = ch_vdev(c);
    lock(&v->drain_lock);
    if (atomic_load_explicit(&v->state, memory_order_acquire) != VDEV_READY) {
        unlock(&v->drain_lock);
        metal_free_memory(rx_queue);
        return RPMSG_DRIVER_ERR_NOT_INITIALIZED;
    }

    int ret = rpmsg_create_ept(&c->ept, v->rdev, name, src, dst,
                               rpmsg_driver_rx_cb, rpmsg_driver_unbind_cb);
    if (ret != 0) {
        unlock(&v->drain_lock);
        metal_free_memory(rx_queue);
        return RPMSG_DRIVER_ERR_TRANSPORT;
    }

    c->rx_queue = rx_queue;
    v->open_count++;
    attach_event(c, v);
    c->is_open = true;
    unlock(&v->drain_lock);
    return RPMSG_DRIVER_OK;
}

rpmsg_driver_status_t rpmsg_driver_close(rpmsg_driver_channel_t *ch)
{
    if (ch == NULL) {
        return RPMSG_DRIVER_ERR_INVALID_PARAM;
    }
    rpmsg_ch_impl_t *c = ch_impl(ch);
    if (!c->is_open) {
        return RPMSG_DRIVER_ERR_NOT_INITIALIZED;
    }

    /* Under drain_lock: no rx_cb runs during destroy. */
    rpmsg_driver_vdev_t *v = ch_vdev(c);
    lock(&v->drain_lock);
    if (!c->is_open) {
        unlock(&v->drain_lock);
        return RPMSG_DRIVER_ERR_NOT_INITIALIZED;
    }
    rpmsg_destroy_ept(&c->ept);
    detach_event(c, v);
    rcar_msg_t *rx_queue = c->rx_queue;
    c->rx_queue = NULL;
    c->rx_count = 0U;
    c->is_open = false;
    bool last = (--v->open_count == 0U);
    unlock(&v->drain_lock);
    metal_free_memory(rx_queue);

    if (last) {
        release_vdev(c->link, c->side, c->vdev);
    }
    return RPMSG_DRIVER_OK;
}

rpmsg_driver_status_t rpmsg_driver_send(rpmsg_driver_channel_t *ch,
                                       const void *data, uint16_t len)
{
    if (ch == NULL) {
        return RPMSG_DRIVER_ERR_INVALID_PARAM;
    }
    rpmsg_ch_impl_t *c = ch_impl(ch);
    if (!c->is_open) {
        return RPMSG_DRIVER_ERR_NOT_INITIALIZED;
    }
    if (len > RPMSG_DRIVER_MAX_PAYLOAD) {
        return RPMSG_DRIVER_ERR_PAYLOAD_TOO_LARGE;
    }
    if ((len > 0U) && (data == NULL)) {
        return RPMSG_DRIVER_ERR_INVALID_PARAM;
    }

    /* Drain first so a send-only channel still gets bound. */
    rpmsg_driver_vdev_t *v = ch_vdev(c);
    if (try_lock(&v->drain_lock)) {
        drain_locked(v, c->vdev);
        unlock(&v->drain_lock);
    }

    if (c->peer_gone) {
        return RPMSG_DRIVER_ERR_PEER_DISCONNECTED;
    }
    if (c->ept.dest_addr == RPMSG_ADDR_ANY) {
        return RPMSG_DRIVER_ERR_NOT_CONNECTED;
    }

    rcar_msg_t msg;
    (void)memset(&msg, 0, offsetof(rcar_msg_t, payload));
    msg.hdr.driver_id   = (uint8_t)c->driver_id;
    msg.hdr.payload_len = len;
    if (len > 0U) {
        (void)memcpy(msg.payload, data, len);
    }

    size_t wire_size = RCAR_MSG_WIRE_SIZE(len);
    int ret = rpmsg_trysend(&c->ept, &msg, (int)wire_size);
    if (ret >= 0) {
        return RPMSG_DRIVER_OK;
    }
    switch (ret) {
        case RPMSG_ERR_NO_BUFF: return RPMSG_DRIVER_ERR_BUFFER_FULL;
        case RPMSG_ERR_ADDR:    return RPMSG_DRIVER_ERR_NOT_CONNECTED;
        default:                return RPMSG_DRIVER_ERR_TRANSPORT;
    }
}

rpmsg_driver_status_t rpmsg_driver_receive(rpmsg_driver_channel_t *ch,
                                          void *buf, uint16_t buf_size,
                                          uint16_t *out_len, uint32_t timeout_ms)
{
    if (ch == NULL) {
        return RPMSG_DRIVER_ERR_INVALID_PARAM;
    }
    rpmsg_ch_impl_t *c = ch_impl(ch);
    if (!c->is_open) {
        return RPMSG_DRIVER_ERR_NOT_INITIALIZED;
    }
    if ((buf == NULL) && (buf_size > 0U)) {
        return RPMSG_DRIVER_ERR_INVALID_PARAM;
    }

    bool forever = (timeout_ms == RPMSG_DRIVER_WAIT_FOREVER);
    if (!forever && (timeout_ms > RPMSG_DRIVER_MAX_TIMEOUT_MS)) {
        timeout_ms = RPMSG_DRIVER_MAX_TIMEOUT_MS;
    }
    uint32_t start_us = (uint32_t)metal_get_timestamp();
    rpmsg_driver_vdev_t *v = ch_vdev(c);

    for (;;) {
        bool locked = try_lock(&v->drain_lock);
        if (locked) {
            if (!c->is_open) {
                unlock(&v->drain_lock);
                return RPMSG_DRIVER_ERR_NOT_INITIALIZED;
            }
            drain_locked(v, c->vdev);

            if (c->rx_count != 0U) {
                const rcar_msg_t *message = &c->rx_queue[c->rx_head];
                uint16_t n = message->hdr.payload_len;
                uint16_t copied = metal_min(n, buf_size);
                if (copied > 0U) {
                    (void)memcpy(buf, message->payload, copied);
                }
                c->rx_head = (c->rx_head + 1U) % RPMSG_DRIVER_RX_QUEUE_DEPTH;
                --c->rx_count;
                unlock(&v->drain_lock);

                if (out_len != NULL) {
                    *out_len = copied;
                }
                return (copied < n) ? RPMSG_DRIVER_ERR_PAYLOAD_TOO_LARGE : RPMSG_DRIVER_OK;
            }
            unlock(&v->drain_lock);
        }

        if (c->peer_gone) {
            return RPMSG_DRIVER_ERR_PEER_DISCONNECTED;
        }
        if (timeout_ms == RPMSG_DRIVER_NO_WAIT) {
            return RPMSG_DRIVER_ERR_TIMEOUT;
        }
        uint32_t waited_ms = elapsed_ms(start_us);
        if (!forever && (waited_ms >= timeout_ms)) {
            return RPMSG_DRIVER_ERR_TIMEOUT;
        }

        /* Lock busy: the holder may be filling our receive FIFO, retry soon. */
        const rpmsg_driver_os_hooks_t *hooks = s_hooks;
        if (locked && (c->ev_slot >= 0) && (hooks != NULL)) {
            uint32_t wait_ms = RPMSG_DRIVER_SAFETY_DRAIN_MS;
            if (!forever && ((timeout_ms - waited_ms) < wait_ms)) {
                wait_ms = timeout_ms - waited_ms;
            }
            (void)hooks->event_wait(v->ev[c->ev_slot], wait_ms);
        } else {
            metal_sleep_usec(RPMSG_DRIVER_POLL_INTERVAL_US);
        }
    }
}
