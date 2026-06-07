/*
 * Copyright (C) 2026 HAW Hamburg
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */
/**
 * @ingroup  cpu_rp2350_common
 * @ingroup  drivers_periph_flash_opt
 * @{
 *
 * @file
 * @brief    Flash Option Configuration Interface Implementation
 *
 * @author   Michel Rottleuthner <michel.rottleuthner@haw-hamburg.de>
 *
 * @}
 */
#include "cpu.h"
#include "periph/flash_opt.h"

#define FLASHOPT_MAX_WS (0)

void flash_opt_set_wait_states(unsigned int ws) {
    (void)ws;
}

unsigned int flash_opt_get_wait_states(void) {
    return 0;
}

unsigned int flash_opt_get_max_wait_states(void) {
    return FLASHOPT_MAX_WS;
}
