/*
 * Copyright (c) 2014, Mentor Graphics Corporation
 * All rights reserved.
 * Copyright (c) 2017 Xilinx, Inc.
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef PLATFORM_RCAR_V2_H
#define PLATFORM_RCAR_V2_H

#include <openamp/remoteproc.h>
#include <openamp/rpmsg_virtio.h>
#include "mfis/mfis.h"
#include "rpmsg_platform.h"

/* Cortex R52 memory attributes */
#define DEVICE_SHARED       ((uint32_t)0x00000001U) /* device, shareable */
#define DEVICE_NONSHARED    ((uint32_t)0x00000010U) /* device, non shareable */
#define NORM_NSHARED_NCACHE ((uint32_t)0x00000008U) /* Non cacheable  non shareable */
#define NORM_SHARED_NCACHE  ((uint32_t)0x0000000CU) /* Non cacheable shareable */
#define PRIV_RW_USER_RW     ((uint32_t)0x00000003U<<8U) /* Full Access */
#define ATTR_NORM_SHARED_NCACHE_PRIV_RW_USER_RW     (NORM_SHARED_NCACHE | PRIV_RW_USER_RW)

/* Max vrings per vdev. */
#define PLATFORM_VRING_MAX  2U

#if defined __cplusplus
extern "C" {
#endif

/* Per-vdev state, owned by the platform. */
struct platform_vdev {
    struct mfis_channel          mfis;
    uint8_t                      has_mfis;
    uint8_t                      mfis_inited;
    uint8_t                      rsc_cached;    /* notify ids valid */
    volatile uint8_t             ready;

    /* Cached from the rsc table. */
    uint32_t                     vdev_notifyid;
    uint32_t                     vring_notifyid[PLATFORM_VRING_MAX];
    unsigned int                 num_vrings;

    metal_phys_addr_t            vring_pa;
    metal_phys_addr_t            shared_buf_pa;

    struct rpmsg_virtio_device   rpvdev;
    struct rpmsg_virtio_shm_pool shpool;        /* FE only */
};

struct remoteproc_priv {
    virtio_type_t        type;
    platform_link_t      link;
    metal_phys_addr_t    shm_base;      /* = rsc table of the link */
    struct platform_vdev vdev[PLATFORM_VDEV_MAX];
};

/* remoteproc ops, remoteproc_rcar_rpmsg.c */
extern const struct remoteproc_ops rcar_rpmsg_proc_ops;

#if defined __cplusplus
}
#endif

#endif /* PLATFORM_RCAR_V2_H */
