/*
 * SPDX-License-Identifier: MIT
 */

#ifndef CPU_LOAD_H
#define CPU_LOAD_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    CPU_LOAD_OK,
    CPU_LOAD_INVALID_PERCENT,
    CPU_LOAD_UNAVAILABLE,
} cpu_load_status_t;

/* Call from the same FreeRTOS task throughout. Updates use its task notification. */
bool cpu_load_init(void);
cpu_load_status_t cpu_load_start(uint8_t percent);
cpu_load_status_t cpu_load_stop(void);
uint8_t cpu_load_get_percent(void);
void cpu_load_deinit(void);

#endif /* CPU_LOAD_H */
