/*
 * Copyright (C) 2021 HAW Hamburg
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */
/**
 * @ingroup  cpu_efm32_common
 * @ingroup  drivers_periph_core_voltage
 * @{
 *
 * @file
 * @brief    Core Voltage Scaling Implementation
 *
 * @detail   Voltage scaling on the EFM32 has various dependencies with other
 *           operation mode settings. Section 9.3.9 of the reference manual
 *           list the following dependencies:
 *           - Flash write/erase is only suported at voltage scale level 2
 *           - TRNG is only supported at voltage scale level 2
 *           - HFXO is only supported at voltage scale level 2
 *           Additionaly, the available voltage scaling controls differ between
 *           them MCUs energy modes (EM0, EM1, etc.).
 *           - For EM0&EM1 the voltage can be scaled between level 2 and 0 (still being
 *             subject to frequency and peripheral dependencies).
 *           - For EM2&EM3 the voltage scale can optionaly be set to a voltage that is
 *             lower than EM0&EM1 voltage. This will be configured automatically on
 *             EM2&EM3 entry. @NOTE: if EM0&EM1 voltage is already lower than the
 *             configured EM2&EM3 voltage this lower voltage will also be applied in
 *             EM2&EM3 instead.
 *           - Any lower voltages set up in EM2&EM3 will persist even after waking
 *             up to EM0&EM1. Beforehand, any settings must therefore be set to options
 *             that are compatible with this lower voltage at return to EM0&EM1.
 *             - To take care of this there are two options available:
 *               - EM23VSCALEAUTOWSEN can take care of automatically configuring flash
 *                 waitstates and frequencies to support the low voltage on EM0&EM1 wakeup.
 *               - The developer (OS) must take care of this manually *before* even
 *                 entering EM2&EM3 with lower voltage settings.
 *          - For EM4H the same logic applies (lower voltages of higher energy modes
 *            overwrite the voltage setting of EM4H and this setting is kept after
 *            wakeup to EM0&EM1)
 *
 * @author   Michel Rottleuthner <michel.rottleuthner@haw-hamburg.de>
 *
 * @}
 */
#include <stdint.h>
#include "cpu.h"
#include "kernel_defines.h"
#include "periph/core_voltage.h"
#include "gclk.h"
#include "gclk_manager.h"
#include "gclk/generic_scaler.h"
#define LOG_LEVEL LOG_NONE
#include "log.h"

/* TODO: this type may be worth to generalize and move to the interface header */
typedef struct {
    const gclk_t *clk;
    uint32_t     freq_hz_max;
    uint8_t      v_idx_min;
} voltage_limit_t;

extern const gclk_clk_scaler_ll_t gclk_efm32_hfclk_scaler;

/* flash wait state dependencies for VSCALE2
 * according to table 10.2-10.4 (p.308) in the reference manual
 * HFCLK <= 25 MHz            -> WS0 or above
 * 25 MHz < HFCLK <= 40 MHz   -> WS1 or above
 *
 * Flash wait states dependencies for VSCALE0
 * HFCLK <= 7 MHz             -> WS0 or above
 * 7 MHz < HFCLK <= 14 MHz    -> WS1 or above
 * 14 MHz < HFCLK <= 20 MHz   -> WS2
 *
 * Low energy peripheral interface
 * HFBUSCLK_LE <= 32 MHz -> WSHFLE = 0 or 1
 * HFBUSCLK_LE > 32 MHz  -> WSHFLE = 1
*/
//static const voltage_limit_t limits[] = {
//    { .clk = &gclk_efm32_hfclk_scaler.base, .freq_hz_max = 20000000, .v_idx_min = 0 },
//    { .clk = &gclk_efm32_hfclk_scaler.base, .freq_hz_max = 40000000, .v_idx_min = 1 },
//};

//#define UNIQUE_CLOCKS_WITH_LIMITS (1)
//static gclk_clock_change_notify_list_t ccnl[UNIQUE_CLOCKS_WITH_LIMITS];
//static unsigned int reg_cnt;

/* these arrays are used for mapping register values to corresponding voltage
 * levels. They must be in sync so don't change the order */
uint16_t mv[]                = { 1000,                       1200 };
uint8_t  srv[ARRAY_SIZE(mv)] = { _EMU_STATUS_VSCALE_VSCALE0, _EMU_STATUS_VSCALE_VSCALE2 };
uint8_t  crv[ARRAY_SIZE(mv)] = { EMU_CMD_EM01VSCALE0,        EMU_CMD_EM01VSCALE2 };

int core_voltage_init(void) {
    return 0;
}

void core_voltage_set(unsigned int voltage_idx) {
    if (voltage_idx > ARRAY_SIZE(crv) - 1) {
        LOG_DEBUG("invalid voltage range!\n");
    } else {
        LOG_DEBUG("setting voltage to idx %u\n", voltage_idx);
        EMU->CMD = crv[voltage_idx];
    }
}

int core_voltage_set_lowest(uint32_t core_hz) {
    (void)core_hz;
    /* TODO: replace this with some actual dependency checking */
    uint32_t lowest_voltage_crv = crv[ARRAY_SIZE(crv) - 1];

    EMU->CMD = lowest_voltage_crv;
    /* no valid frequency found at all -> this shouldn't happen (given frequency invalid?) */
    return -1;
}

int core_voltage_get(void) {
    /* Neither the reference manual nor the datasheet explicitly mentions the actual voltage
     * of the two voltage scale settings. This post however mentions it is 1.2 V and 1.0 V
     * respectively:
     * https://www.silabs.com/community/mcu/32-bit/knowledge-base.entry.html/2017/04/21/voltage_scaling_one-90V0
     * This also matches footnote 2 of table 4.8 'Wake Up Times' from the reference manual, stating the scaling
     * process takes 20 µs at a rate of 10 mV/µs. */
    uint32_t vscale = (EMU->STATUS & _EMU_STATUS_VSCALE_MASK) >> _EMU_STATUS_VSCALE_SHIFT;
    /* Bit fields for EMU CMD */
    for (unsigned i = 0; i < ARRAY_SIZE(srv); i++) {
        if (srv[i] == vscale) {
            return i;
        }
    }
    LOG_ERROR("ERROR %s: invalid register value!\n", __FUNCTION__);
    return -1;
}

unsigned int core_voltage_idx2mv(unsigned voltage_idx) {
    return mv[voltage_idx];
}

unsigned int core_voltage_cnt(void) {
    return ARRAY_SIZE(crv);
}


