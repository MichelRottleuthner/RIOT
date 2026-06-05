/*
 * Copyright (C) 2021 HAW Hamburg
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */
/**
 * @ingroup  cpu_efm32_common
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
#include "em_msc.h"
#include "periph/flash_opt.h"

#define FLASHOPT_MAX_WS (3)

void flash_opt_set_wait_states(unsigned int ws) {
    if (ws <= FLASHOPT_MAX_WS) {
        /* changing flash waitstates is only possible if flash controller is unlocked */
        bool locked = MSC->LOCK & _MSC_LOCK_MASK;
        MSC->LOCK = MSC_LOCK_LOCKKEY_UNLOCK;
        MSC->READCTRL = (MSC->READCTRL & ~_MSC_READCTRL_MODE_MASK) |
                        (ws << _MSC_READCTRL_MODE_SHIFT);
        if (locked) {
            MSC->LOCK = MSC_LOCK_LOCKKEY_LOCK;
        }
    }
}

unsigned int flash_opt_get_wait_states(void) {
    return (MSC->READCTRL & _MSC_READCTRL_MODE_MASK) >> _MSC_READCTRL_MODE_SHIFT;
}

unsigned int flash_opt_get_max_wait_states(void) {
    return FLASHOPT_MAX_WS;
}
