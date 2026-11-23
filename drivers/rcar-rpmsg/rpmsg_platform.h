/*
 * Copyright (c) 2014, Mentor Graphics Corporation
 * All rights reserved.
 * Copyright (c) 2017 Xilinx, Inc.
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef RPMSG_PLATFORM_H
#define RPMSG_PLATFORM_H

#include <stdbool.h>

#include <openamp/virtio.h>
#include "rpmsg_driver.h"

struct remoteproc;
struct rpmsg_device;

/* Max vdevs per side. */
#define PLATFORM_VDEV_MAX   4U

/* Routes without an interrupt use polling. */
#define PLATFORM_NO_NOTIFY  0xFFU

#if defined __cplusplus
extern "C" {
#endif

typedef enum virtio_type
{
    VIRTIO_FRONT_END = VIRTIO_DEV_DRIVER,
    VIRTIO_BACK_END = VIRTIO_DEV_DEVICE,
} virtio_type_t;

/* Each link has a resource table and a shared-memory region. */
typedef enum {
    PLATFORM_LINK_LOCAL = 0,
    PLATFORM_LINK_UCIE0,
    PLATFORM_LINK_MAX
} platform_link_t;

/* The two peers use the same route for a driver ID. */
typedef struct {
    unsigned int    vdev;
    unsigned int    notify_ch;  /* or PLATFORM_NO_NOTIFY */
    bool            fixed_addresses;
    const char     *default_name;
    platform_link_t link;       /* omitted: PLATFORM_LINK_LOCAL */
} platform_rpmsg_route_t;

bool platform_get_rpmsg_route(rpmsg_driver_id_t driver_id, platform_rpmsg_route_t *route);

/* Called from the notification ISR. Must not block. */
typedef void (*platform_notify_cb_t)(void *arg);

/*
 * Create the remoteproc of one side of @link (once; later calls return it).
 * BE writes the resource table, FE reads it. Caller serializes calls per link.
 * Returns -ENODEV if the app does not link the resource table of @link.
 */
int rpmsg_platform_init(platform_link_t link, virtio_type_t type, struct remoteproc **platform);

/*
 * Bring up vdev @ch with the same notification channel on both peers.
 * BE blocks until FE sets DRIVER_OK. Caller serializes calls.
 * Returns -ENODEV if the resource table has no entry @ch.
 */
int platform_add_vdev(struct remoteproc *platform, unsigned int ch,
                      unsigned int notify_ch,
                      platform_notify_cb_t notify_cb, void *cb_arg,
                      struct rpmsg_device **rdev);

/*
 * Release vdev @ch. Do not add it again. The peer is not told.
 * No platform_drain_vdev() may run on any vdev of @platform meanwhile.
 */
int platform_remove_vdev(struct remoteproc *platform, unsigned int ch);

/*
 * Process all pending messages of vdev @ch; endpoint callbacks run in the
 * caller. Serialize per @ch. Returns -EAGAIN if the vdev is not ready.
 */
int platform_drain_vdev(struct remoteproc *platform, unsigned int ch);

#if defined __cplusplus
}
#endif

#endif /* RPMSG_PLATFORM_H */
