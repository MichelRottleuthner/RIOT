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

#include "hardware/platform_defs.h"
#include "regs/powman.h"
#include "RP2350.h"
/*
 * 00000 - 0.55V
 * 00001 - 0.60V
 * 00010 - 0.65V
 * 00011 - 0.70V
 * 00100 - 0.75V
 * 00101 - 0.80V
 * 00110 - 0.85V
 * 00111 - 0.90V
 * 01000 - 0.95V
 * 01001 - 1.00V
 * 01010 - 1.05V
 * 01011 - 1.10V (default)
 * 01100 - 1.15V
 * 01101 - 1.20V
 * 01110 - 1.25V
 * 01111 - 1.30V
 * 10000 - 1.35V
 * 10001 - 1.40V
 * 10010 - 1.50V
 * 10011 - 1.60V
 * 10100 - 1.65V
 * 10101 - 1.70V
 * 10110 - 1.80V
 * 10111 - 1.90V
 * 11000 - 2.00V
 * 11001 - 2.35V
 * 11010 - 2.50V
 * 11011 - 2.65V
 * 11100 - 2.80V
 * 11101 - 3.00V
 * 11110 - 3.15V
 * 11111 - 3.30V
*/
#define DVDD_BASE_VOLTAGE_MV         (550)
#define DVDD_VOLTAGE_STEP_MV         ( 50)
#define POWMAN_VREG_VSEL_MAX_VAL     (POWMAN_VREG_VSEL_BITS >> POWMAN_VREG_VSEL_LSB)
#define POWMAN_VREG_VSEL_DEFAULT_VAL (0xb)

/* write protected registers must include the passsword otherwise the write is ignored */
#define POWMAN_PASSWD                (0x5AFE << 16)
#define POWMAN_PASSWD_BITS           (0xFFFF << 16)

int core_voltage_init(void) {
    //reset_component(RESET_POWMAN, RESET_POWMAN);
    POWMAN->VREG_CTRL = (POWMAN->VREG_CTRL & ~(POWMAN_PASSWD_BITS))
                        | (POWMAN_PASSWD)
                        | (POWMAN_VREG_CTRL_UNLOCK_BITS);
    return 0;
}

void core_voltage_set(unsigned int voltage_idx) {
    if (voltage_idx > POWMAN_VREG_VSEL_MAX_VAL) {
        voltage_idx = POWMAN_VREG_VSEL_MAX_VAL;
    }
    POWMAN->VREG = (POWMAN->VREG & ~(POWMAN_VREG_VSEL_BITS | POWMAN_PASSWD_BITS))
                   | (POWMAN_PASSWD)
                   | (voltage_idx << POWMAN_VREG_VSEL_LSB);
}

int core_voltage_set_lowest(uint32_t core_hz) {
    (void)core_hz;
    // default voltage is 1.1V == index 11
    core_voltage_set(POWMAN_VREG_VSEL_DEFAULT_VAL);
    return 0;
}

int core_voltage_get(void) {
    //uint32_t vr =  POWMAN->VREG;
    uint32_t vr =  POWMAN->VREG;
    int vsel = (vr & POWMAN_VREG_VSEL_BITS) >> POWMAN_VREG_VSEL_LSB;
    //int mv = DVDD_BASE_VOLTAGE_MV + vsel * DVDD_VOLTAGE_STEP_MV;
    return vsel; /* vsel works as index directly */
}

unsigned int core_voltage_idx2mv(unsigned voltage_idx) {
    (void)voltage_idx;
    if (voltage_idx > POWMAN_VREG_VSEL_MAX_VAL) {
        voltage_idx = POWMAN_VREG_VSEL_MAX_VAL;
    }
    unsigned mv = DVDD_BASE_VOLTAGE_MV + voltage_idx * DVDD_VOLTAGE_STEP_MV;
    return mv;
}

unsigned int core_voltage_cnt(void) {
    return POWMAN_VREG_VSEL_MAX_VAL + 1;
}


