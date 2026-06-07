/*
 * Copyright (C) 2026 HAW Hamburg
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */
/**
 * @ingroup  cpu_rp2350_common
 * @ingroup  drivers_periph_core_voltage
 * @{
 *
 * @file
 * @brief    Core Voltage Scaling Implementation
 *
 * @author   Michel Rottleuthner <michel.rottleuthner@haw-hamburg.de>
 *
 * @}
 */
#include <stdint.h>
#include "cpu.h"
#include "periph/core_voltage.h"
#define LOG_LEVEL LOG_NONE
#include "log.h"


int core_voltage_init(void) {
    return 0;
}

void core_voltage_set(unsigned int voltage_idx) {
    (void)voltage_idx;
}

int core_voltage_set_lowest(uint32_t core_hz) {
    (void)core_hz;
    return 0;
}

int core_voltage_get(void) {
    return 0;
}

unsigned int core_voltage_idx2mv(unsigned voltage_idx) {
    (void)voltage_idx;
    return 1100;
}

unsigned int core_voltage_cnt(void) {
    return 1;
}


