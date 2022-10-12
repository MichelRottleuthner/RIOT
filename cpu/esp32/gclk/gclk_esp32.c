/*
 * Copyright (C) 2022 HAW Hamburg
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     cpu_esp32_common
 * @{
 *
 * @file
 * @brief       ESP32-specific declarations for the gclk module
 *
 * @author      Michel Rottleuthner <michel.rottleuthner@haw-hamburg.de>
 * @}
 */
#include <stdint.h>
#include <stddef.h>

#include "cpu.h"
#include "board.h"

#include <stdio.h>
#include "gclk.h"
#include "gclk/generic_gate.h"
#include "gclk/generic_mux.h"
#include "gclk/generic_scaler.h"
#include "soc/rtc_cntl_reg.h"

typedef enum {
    GCLK_NULL_REG = 0,
    GCLK_ESP32_RTC_CNTL_CLK_CONF_REG,
} gclk_conf_reg_id_t;

volatile uint32_t* const conf_regs[] = {
    [GCLK_NULL_REG]                    = NULL,
    [GCLK_ESP32_RTC_CNTL_CLK_CONF_REG] = RTC_CNTL_CLK_CONF_REG,
};

/* forward declare clock instances to reference them in other clocks */

/* the PLL_CLK on the esp32 may run at either 320 MHz or 480 MHz.
 * This can be modelled as a 160 MHz fixed (gateable) source that gets multiplied by either
 * 2 or 3 */
const gclk_generic_gate_t gclk_esp32_pll_src_gate;

/* the APLL_CLK is an audio pll lock with a frequency range of 16 - 128 MHz.
 * The effective output frequency of APLL depends on multiple factors:
 *
 * f_out = f_xtal * (sdm2 + sdm1/2^8 + sdm0/2^16 + 4) / (2*(odiv+2))
 *
 * The numerator range is constrained to 350 MHz - 500 MHz.
 * sdm0 := 0 - 255
 * sdm1 := 0 - 255
 * sdm2 := 0 - 63
 * odiv := 0 - 31
 *
 * The APLL configuration uses the custom internal configuration bus via the regi2c_ctrl
 * interface so this needs some extra low-level adaptation
 * (opposed to the default memory mapped register access)
 * */
//const gclk_generic_gate_t gclk_esp32_apll_src_gate;

//const gclk_mux_ll_t gclk_esp32_some_mux;
//const gclk_clk_scaler_ll_t gclk_esp32_some_scaler;

const gclk_t * const gclks[] = {
    &gclk_esp32_pll_src_gate.base,
};
