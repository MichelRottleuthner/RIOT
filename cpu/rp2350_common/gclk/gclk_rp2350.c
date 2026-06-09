/*
 * Copyright (C) 2026 HAW Hamburg
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     cpu_rp2350_common
 * @{
 *
 * @file
 * @brief       Implementation of the gclk instances for RP2350
 *
 * @author      Michel Rottleuthner <michel.rottleuthner@haw-hamburg.de>
 * @}
 */

#include <stdlib.h>
#include <stdint.h>
#include "kernel_defines.h"
#include "cpu.h"
#include "gclk.h"
#include "gclk/generic_gate.h"
#include "gclk/generic_mux.h"
#include "gclk/generic_scaler.h"
#include "bitarithm.h"

#include "RP2350.h"
#include "board.h"
#include "macros/units.h"
#include "periph_cpu.h"
#include "regs/pll.h"
#include "regs/rosc.h"
#include "regs/clocks.h"
#include "regs/pads_bank0.h"
//TODO: the gpio function definitions would be accessible from here,
//      but there are dependency issues with the includes
//#include "hardware/structs/io_bank0.h"

/* included for the re-init callbacks */
#include "stdio_base.h"

#define LOG_LEVEL LOG_NONE
#include "log.h"

/* TODO: this switch should be integrated with pm.c
 *       Currently this has no effect except for silencing the compiler */
bool __use_default_clk_init = false;


uint32_t volatile * const conf_regs[] = {
    [GCLK_NULL_REG]                 = NULL,
    [GCLK_RP2350_PLL_SYS_CS]        = &PLL_SYS->CS,
    [GCLK_RP2350_PLL_SYS_FBDIV_INT] = &PLL_SYS->FBDIV_INT,
    [GCLK_RP2350_PLL_SYS_PRIM]      = &PLL_SYS->PRIM,
    [GCLK_RP2350_PLL_SYS_PWR]       = &PLL_SYS->PWR,
    //[GCLK_RP2350_PLL_FBDIV_INT]   = &CMU->DPLLCTRL1,
    //[GCLK_RP2350_PLL_PRIM]        = &CMU->DPLLCTRL1,
    [GCLK_RP2350_CLK_SYS_CTRL]      = &CLOCKS->CLK_SYS_CTRL,
    [GCLK_RP2350_CLK_SYS_DIV]       = &CLOCKS->CLK_SYS_DIV,
    [GCLK_RP2350_CLK_REF_CTRL]      = &CLOCKS->CLK_REF_CTRL,

    [GCLK_RP2350_PLL_USB_CS]        = &PLL_USB->CS,
    [GCLK_RP2350_PLL_USB_FBDIV_INT] = &PLL_USB->FBDIV_INT,
    [GCLK_RP2350_PLL_USB_PRIM]      = &PLL_USB->PRIM,
    [GCLK_RP2350_PLL_USB_PWR]       = &PLL_USB->PWR,

    [GCLK_RP2350_CLK_PERI_CTRL]     = &CLOCKS->CLK_PERI_CTRL,
    [GCLK_RP2350_CLK_PERI_DIV]      = &CLOCKS->CLK_PERI_DIV,
    [GCLK_RP2350_CLK_GPOUT0_CTRL]   = &CLOCKS->CLK_GPOUT0_CTRL,
    [GCLK_RP2350_CLK_GPOUT0_DIV]    = &CLOCKS->CLK_GPOUT0_DIV,

    [GCLK_RP2350_ROSC_CTRL]         = &ROSC->CTRL,
};

typedef struct {
  uint16_t msb: 5;
  uint16_t lsb: 5;
} gclk_register_mask_t;

static inline uint32_t _msb_lsb_idx_to_mask(uint8_t msb, uint8_t lsb) {
    uint32_t mask = 0xFFFFFFFF;
    mask = mask >> msb;
    mask = mask << lsb;
    mask = mask << (31 - msb);
    mask = mask >> (31 - msb);
    return mask;
}

static const gclk_range8_t _pll_refdiv_range = {
    .min =  1, .max =  63, /* numerical div values */
};

static const gclk_range16_t _pll_fbdiv_range = {
    .min =  16, .max =  320, /* numerical div values */
};

static const gclk_range8_t _pll_postdiv_range = {
    .min =  1, .max =  7, /* numerical div values */
};

static const gclk_range16_t _clk_int16_div_range = {
    .min =  1, .max =  0xffff, /* numerical div values */
};

/* forward declarations to be used in other nodes without taking care of definition order */
const gclk_clk_scaler_ll_t gclk_rp2350_pll_sys_scaler;
const gclk_clk_scaler_ll_t gclk_rp2350_pll_usb_scaler;
const gclk_clk_scaler_ll_t gclk_rp2350_rosc_scaler;
const gclk_generic_gate_t gclk_rp2350_xosc_gate;

const gclk_clk_scaler_ll_t gclk_rp2350_pll_sys_refdiv_scaler;
const gclk_clk_scaler_ll_t gclk_rp2350_pll_sys_fbdiv_scaler;
const gclk_clk_scaler_ll_t gclk_rp2350_pll_sys_vco_scaler;
const gclk_clk_scaler_ll_t gclk_rp2350_pll_sys_postdiv1_scaler;
const gclk_clk_scaler_ll_t gclk_rp2350_pll_sys_postdiv2_scaler;
const gclk_mux_ll_t gclk_rp2350_pll_sys_bypass_mux;
const gclk_generic_gate_t gclk_rp2350_pll_sys_vco_gate;

const gclk_clk_scaler_ll_t gclk_rp2350_pll_usb_refdiv_scaler;
const gclk_clk_scaler_ll_t gclk_rp2350_pll_usb_fbdiv_scaler;
const gclk_clk_scaler_ll_t gclk_rp2350_pll_usb_vco_scaler;
const gclk_clk_scaler_ll_t gclk_rp2350_pll_usb_postdiv1_scaler;
const gclk_clk_scaler_ll_t gclk_rp2350_pll_usb_postdiv2_scaler;
const gclk_mux_ll_t gclk_rp2350_pll_usb_bypass_mux;
const gclk_generic_gate_t gclk_rp2350_pll_usb_vco_gate;

const gclk_mux_ll_t gclk_rp2350_clk_sys_mux;
const gclk_generic_gate_t gclk_rp2350_clk_sys_gate;

#define GCLK_RP2350_PLAIN_SCALER_STATIC_INIT(NAME)\
.base.separated_ops  = gclk_plain_scaler_ops,\
.base.flags.scalable = 1,\
.base.name           = #NAME

/* we model the clock pins a s a source out of our control, we cannot even enable /disable it */
const gclk_t gclk_rp2350_gpin0_src = {
    .separated_ops    = NULL,
    .name             = "GPIN0",
    .flags.is_source  = 1,
   //TODO: The fixed frequency should be defined per board or by an application
   //      that actually connects a clock signal here. For now we just assume 1 MHz.
    .fixed_input_freq = 1000000,
};

const gclk_t gclk_rp2350_gpin1_src = {
    .separated_ops    = NULL,
    .name             = "GPIN1",
    .flags.is_source  = 1,
   //TODO: The fixed frequency should be defined per board or by an application
   //      that actually connects a clock signal here. For now we just assume 1 MHz.
    .fixed_input_freq = 1000000,
};

const gclk_clk_scaler_ll_t gclk_rp2350_pll_sys_refdiv_scaler = {
    GCLK_RP2350_PLAIN_SCALER_STATIC_INIT(PLL_SYS_REFDIV),
    .base.flags.topology_flags = 0,
    .base.fixed_parent         = &gclk_rp2350_xosc_gate.base,
    .regref                    = { .en  = GCLK_NULL_REG,
                                   .rdy = GCLK_NULL_REG,
                                   .conf = GCLK_RP2350_PLL_SYS_CS,
                                   .conf_msb = 5,
                                   .conf_lsb = 0,
                                 },
    GCLK_REGVAL_AS_NUMVAL_RANGE8_STATIC_INIT(_pll_refdiv_range),
    .base.flags.scaler_type    = GCLK_DIV,
};

const gclk_clk_scaler_ll_t gclk_rp2350_pll_sys_fbdiv_scaler = {
    GCLK_RP2350_PLAIN_SCALER_STATIC_INIT(PLL_SYS_FBDIV),
    .base.flags.topology_flags = 0,
    .base.fixed_parent         = &gclk_rp2350_pll_sys_refdiv_scaler.base,
    .regref                    = { .en  = GCLK_NULL_REG,
                                   .rdy = GCLK_NULL_REG,
                                   .conf = GCLK_RP2350_PLL_SYS_FBDIV_INT,
                                   .conf_msb = 11,
                                   .conf_lsb = 0,
                                 },
    GCLK_REGVAL_AS_NUMVAL_RANGE16_STATIC_INIT(_pll_fbdiv_range),
    .base.flags.scaler_type    = GCLK_MUL,
};

const gclk_clk_scaler_ll_t gclk_rp2350_pll_sys_postdiv1_scaler = {
    GCLK_RP2350_PLAIN_SCALER_STATIC_INIT(PLL_SYS_POSTDIV1),
    .base.flags.topology_flags = 0,
    .base.fixed_parent         = &gclk_rp2350_pll_sys_vco_gate.base,
    .regref                    = { .en  = GCLK_NULL_REG,
                                   .rdy = GCLK_NULL_REG,
                                   .conf = GCLK_RP2350_PLL_SYS_PRIM,
                                   .conf_msb = 18,
                                   .conf_lsb = 16,
                                 },
    GCLK_REGVAL_AS_NUMVAL_RANGE8_STATIC_INIT(_pll_postdiv_range),
    .base.flags.scaler_type    = GCLK_DIV,
};

const gclk_clk_scaler_ll_t gclk_rp2350_pll_sys_postdiv2_scaler = {
    GCLK_RP2350_PLAIN_SCALER_STATIC_INIT(PLL_SYS_POSTDIV2),
    .base.flags.topology_flags = 0,
    .base.fixed_parent         = &gclk_rp2350_pll_sys_postdiv1_scaler.base,
    .regref                    = { .en  = GCLK_NULL_REG,
                                   .rdy = GCLK_NULL_REG,
                                   .conf = GCLK_RP2350_PLL_SYS_PRIM,
                                   .conf_msb = 14,
                                   .conf_lsb = 12,
                                 },
    GCLK_REGVAL_AS_NUMVAL_RANGE8_STATIC_INIT(_pll_postdiv_range),
    .base.flags.scaler_type    = GCLK_DIV,
};

/* The BYPASS mux can either route the ref clock through the PLL or outut the ref clock directly */
static const gclk_parent_config_lut_t _pll_sys_bypass_mux_configs[] = {
    { .parent = &gclk_rp2350_pll_sys_postdiv2_scaler.base, .config_reg_val = 0 }, /* PLL out */
    { .parent = &gclk_rp2350_xosc_gate.base,               .config_reg_val = 1 }, /* FREF = XOSC */
};

const gclk_mux_ll_t gclk_rp2350_pll_sys_bypass_mux = {
    GCLK_PLAIN_MUX_STATIC_INIT(PLL_SYS_BYPASS,
                               GCLK_RP2350_PLL_SYS_CS,
                               PLL_CS_BYPASS_BITS),
    GCLK_PARENT_LUT_STATIC_INIT(_pll_sys_bypass_mux_configs),
};

/* USB PLL vvvvv */
const gclk_clk_scaler_ll_t gclk_rp2350_pll_usb_refdiv_scaler = {
    GCLK_RP2350_PLAIN_SCALER_STATIC_INIT(PLL_USB_REFDIV),
    .base.flags.topology_flags = 0,
    .base.fixed_parent         = &gclk_rp2350_xosc_gate.base,
    .regref                    = { .en  = GCLK_NULL_REG,
                                   .rdy = GCLK_NULL_REG,
                                   .conf = GCLK_RP2350_PLL_USB_CS,
                                   .conf_msb = 5,
                                   .conf_lsb = 0,
                                 },
    GCLK_REGVAL_AS_NUMVAL_RANGE8_STATIC_INIT(_pll_refdiv_range),
    .base.flags.scaler_type    = GCLK_DIV,
};

const gclk_clk_scaler_ll_t gclk_rp2350_pll_usb_fbdiv_scaler = {
    GCLK_RP2350_PLAIN_SCALER_STATIC_INIT(PLL_USB_FBDIV),
    .base.flags.topology_flags = 0,
    .base.fixed_parent         = &gclk_rp2350_pll_usb_refdiv_scaler.base,
    .regref                    = { .en  = GCLK_NULL_REG,
                                   .rdy = GCLK_NULL_REG,
                                   .conf = GCLK_RP2350_PLL_USB_FBDIV_INT,
                                   .conf_msb = 11,
                                   .conf_lsb = 0,
                                 },
    GCLK_REGVAL_AS_NUMVAL_RANGE16_STATIC_INIT(_pll_fbdiv_range),
    .base.flags.scaler_type    = GCLK_MUL,
};

const gclk_clk_scaler_ll_t gclk_rp2350_pll_usb_postdiv1_scaler = {
    GCLK_RP2350_PLAIN_SCALER_STATIC_INIT(PLL_USB_POSTDIV1),
    .base.flags.topology_flags = 0,
    .base.fixed_parent         = &gclk_rp2350_pll_usb_vco_gate.base,
    .regref                    = { .en  = GCLK_NULL_REG,
                                   .rdy = GCLK_NULL_REG,
                                   .conf = GCLK_RP2350_PLL_USB_PRIM,
                                   .conf_msb = 18,
                                   .conf_lsb = 16,
                                 },
    GCLK_REGVAL_AS_NUMVAL_RANGE8_STATIC_INIT(_pll_postdiv_range),
    .base.flags.scaler_type    = GCLK_DIV,
};

const gclk_clk_scaler_ll_t gclk_rp2350_pll_usb_postdiv2_scaler = {
    GCLK_RP2350_PLAIN_SCALER_STATIC_INIT(PLL_USB_POSTDIV2),
    .base.flags.topology_flags = 0,
    .base.fixed_parent         = &gclk_rp2350_pll_usb_postdiv1_scaler.base,
    .regref                    = { .en  = GCLK_NULL_REG,
                                   .rdy = GCLK_NULL_REG,
                                   .conf = GCLK_RP2350_PLL_USB_PRIM,
                                   .conf_msb = 14,
                                   .conf_lsb = 12,
                                 },
    GCLK_REGVAL_AS_NUMVAL_RANGE8_STATIC_INIT(_pll_postdiv_range),
    .base.flags.scaler_type    = GCLK_DIV,
};

/* The BYPASS mux can either route the ref clock through the PLL or outut the ref clock directly */
static const gclk_parent_config_lut_t _pll_usb_bypass_mux_configs[] = {
    { .parent = &gclk_rp2350_pll_usb_postdiv2_scaler.base, .config_reg_val = 0 }, /* PLL out */
    { .parent = &gclk_rp2350_xosc_gate.base,               .config_reg_val = 1 }, /* FREF = XOSC */
};

const gclk_mux_ll_t gclk_rp2350_pll_usb_bypass_mux = {
    GCLK_PLAIN_MUX_STATIC_INIT(PLL_USB_BYPASS,
                               GCLK_RP2350_PLL_USB_CS,
                               PLL_CS_BYPASS_BITS),
    GCLK_PARENT_LUT_STATIC_INIT(_pll_usb_bypass_mux_configs),
};


/* Based on datasheet 8.2.4 (1ms wait time) */
#define STARTUP_DELAY 47
#define MAX_XOSC_COUNTER_SIZE 0xFFFF
#define SLEEP_100HZ_SPEED 12000000UL
#define CYCLES_PER_MS (SLEEP_100HZ_SPEED / 1000)

void gclk_rp2350_xosc_enable(const gclk_t *clk, bool on) {
    gclk_generic_gate_t *gate = to_gclk_generic_gate_t(clk);
    gclk_reg_ref_t regref = gate->regref;
    (void)regref;
    if (on) {
        LOG_DEBUG("%s: enabling %s...\n", __FUNCTION__, gclk_get_name(clk));
        /* Set the FREQ_RANGE */
        XOSC->CTRL = XOSC_CTRL_FREQ_RANGE_VALUE_1_15MHZ;
        /* Set the startup delay (default 1ms) */
        XOSC->STARTUP = STARTUP_DELAY;
        /* set enable bit */
        atomic_set(&XOSC->CTRL, XOSC_CTRL_ENABLE_VALUE_ENABLE << XOSC_CTRL_ENABLE_LSB);

        while (!(XOSC->STATUS & XOSC_STATUS_STABLE_BITS)) {
            /* Wait for the crystal to stabilize */
        }
    } else {
        LOG_DEBUG("%s: disabling %s...\n", __FUNCTION__, gclk_get_name(clk));
        //gclk_reg_util_clear_mask(gclk_regref2enable_reg(regref), gclk_regref2enable_mask(regref));
        atomic_set(&XOSC->CTRL, XOSC_CTRL_ENABLE_VALUE_DISABLE << XOSC_CTRL_ENABLE_LSB);
    }
}

bool gclk_rp2350_xosc_is_enabled(const gclk_t *clk) {
    gclk_generic_gate_t *gate = to_gclk_generic_gate_t(clk);
    (void)gate;

    uint32_t enable_val = gclk_reg_util_read_masked(&XOSC->CTRL,
                                                    STATIC_BITRANGE_TO_MASK(23,12));

    if (enable_val == XOSC_CTRL_ENABLE_VALUE_ENABLE) {
        return true;
    } else if (enable_val == XOSC_CTRL_ENABLE_VALUE_DISABLE) {
        return false;
    } else {
        printf("invalid enable value!\n");
        assert(false);
    }
}

const gclk_op_t gclk_rp2350_oscx_gate_ops[] = {
  { .gate_ops = { .is_enabled = gclk_rp2350_xosc_is_enabled,
                  .enable     = gclk_rp2350_xosc_enable,}},
};

const gclk_generic_gate_t gclk_rp2350_xosc_gate = {
.base.separated_ops    = gclk_rp2350_oscx_gate_ops,
.base.fixed_input_freq = XOSC_HZ,
.base.flags.is_source  = 1,
.base.flags.gateable   = 1,
.base.name             = "XOSC",\
.regref                = { .en      = GCLK_NULL_REG,
                           .rdy     = GCLK_NULL_REG,
                           .en_bit  = 0,
                           .rdy_bit = 0, }
};

void gclk_rp2350_pll_enable(const gclk_t *clk, bool on) {
    gclk_generic_gate_t *gate = to_gclk_generic_gate_t(clk);
    //gclk_reg_ref_t regref = gate->regref;

    PLL_SYS_Type *pll;
    if (gate == &gclk_rp2350_pll_sys_vco_gate) {
        pll = PLL_SYS;
    } else if (gate == &gclk_rp2350_pll_usb_vco_gate) {
        pll = PLL_USB;
    } else {
        assert(false);
    }

    if (on) {
        LOG_DEBUG("%s: enabling %s...\n", __FUNCTION__, gclk_get_name(clk));
        atomic_clear(&pll->PWR,
                    PLL_PWR_PD_BITS | PLL_PWR_VCOPD_BITS | PLL_PWR_POSTDIVPD_BITS);
    } else {
        LOG_DEBUG("%s: disabling %s...\n", __FUNCTION__, gclk_get_name(clk));
        //gclk_reg_util_clear_mask(gclk_regref2enable_reg(regref), gclk_regref2enable_mask(regref)
        atomic_set(&pll->PWR,
                    PLL_PWR_PD_BITS | PLL_PWR_VCOPD_BITS | PLL_PWR_POSTDIVPD_BITS);
    }

    /* sleep 10ms to allow the PLL to stabilize */
    xosc_sleep(10);
}

bool gclk_rp2350_pll_is_enabled(const gclk_t *clk) {
    gclk_generic_gate_t *gate = to_gclk_generic_gate_t(clk);

    gclk_reg_ref_t regref = gate->regref;

    volatile uint32_t *en_reg = gclk_regref2enable_reg(regref);

    uint32_t pd_val = gclk_reg_util_read_masked(en_reg,
                                                    0x1 << regref.en_bit);

    /* if powerdown is set the gate is disabled */
    if (pd_val == 0) {
        return true;
    } else {
        return false;
    }
}

const gclk_op_t gclk_rp2350_pll_vco_gate_ops[] = {
  { .gate_ops = { .is_enabled = gclk_rp2350_pll_is_enabled,
                  .enable     = gclk_rp2350_pll_enable, }},
};

#define _PLL_VCO_GATE_STATIC_INIT(NAME,PARENT,EN_REG_ID,RDY_REG_ID,EN_BIT,RDY_BIT)\
.base.separated_ops    = gclk_rp2350_pll_vco_gate_ops,\
.base.fixed_parent     = PARENT,\
.base.flags.gateable   = 1,\
.base.name             = #NAME,\
.regref                = { .en      = EN_REG_ID,\
                           .rdy     = RDY_REG_ID,\
                           .en_bit  = EN_BIT,\
                           .rdy_bit = RDY_BIT, }

const gclk_generic_gate_t gclk_rp2350_pll_sys_vco_gate = {
_PLL_VCO_GATE_STATIC_INIT(PLL_SYS_VCO,
                        &gclk_rp2350_pll_sys_fbdiv_scaler.base,
                        GCLK_RP2350_PLL_SYS_PWR,
                        GCLK_RP2350_PLL_SYS_CS,
                        PLL_PWR_PD_MSB, // MSB and LSB are same, VCOPD should probably also be set
                                        // separately! (custom impl needed?)
                        PLL_CS_LOCK_MSB), // MSB and LSB are same
.base.flags.topology_flags = GCLK_STOP_FOR_UPDATE, // enforce that configuration updates use an intermediate topology
};

const gclk_generic_gate_t gclk_rp2350_pll_usb_vco_gate = {
_PLL_VCO_GATE_STATIC_INIT(PLL_USB_VCO,
                        &gclk_rp2350_pll_usb_fbdiv_scaler.base,
                        GCLK_RP2350_PLL_USB_PWR,
                        GCLK_RP2350_PLL_USB_CS,
                        PLL_PWR_PD_MSB, // MSB and LSB are same, VCOPD should probably also be set
                                        // separately! (custom impl needed?)
                        PLL_CS_LOCK_MSB), // MSB and LSB are same
.base.flags.topology_flags = GCLK_STOP_FOR_UPDATE,
};

/* CLK_SYS AUXSRC: Selects the auxiliary clock source, will glitch when switching. */
static const gclk_parent_config_lut_t _clk_ref_aux_src_mux_configs[] = {
    { .parent = &gclk_rp2350_pll_usb_bypass_mux.base, .config_reg_val = 0 }, /* 0x0 → CLKSRC_PLL_USB */
/* 0x1 → CLKSRC_GPIN0 */
/* 0x2 → CLKSRC_GPIN1 */
/* 0x3 → CLKSRC_PLL_USB_PRIMARY_REF_OPCG */
};

const gclk_mux_ll_t gclk_rp2350_clk_ref_auxsrc_mux = {
    GCLK_PLAIN_MUX_STATIC_INIT(CLK_REF_AUXSRC,
                               GCLK_RP2350_CLK_REF_CTRL,
                               CLOCKS_CLK_SYS_CTRL_AUXSRC_BITS),
    GCLK_PARENT_LUT_STATIC_INIT(_clk_ref_aux_src_mux_configs),
};

/* CLK_SYS SRC: Selects the clock source glitchlessly, can be changed on-the-fly */
static const gclk_parent_config_lut_t _clk_ref_src_mux_configs[] = {
/* 0x0 → ROSC_CLKSRC_PH */
    { .parent = &gclk_rp2350_clk_ref_auxsrc_mux.base, .config_reg_val = 1 }, /* 0x1 → CLKSRC_CLK_REF_AUX */
/* 0x2 → XOSC_CLKSRC */
/* 0x3 → LPOSC_CLKSRC */
};

const gclk_mux_ll_t gclk_rp2350_clk_ref_src_mux = {
    GCLK_PLAIN_MUX_STATIC_INIT(CLK_REF_SRC,
                               GCLK_RP2350_CLK_REF_CTRL,
                               CLOCKS_CLK_REF_CTRL_SRC_BITS),
    GCLK_PARENT_LUT_STATIC_INIT(_clk_ref_src_mux_configs),
};


/* CLK_SYS AUXSRC: Selects the auxiliary clock source, will glitch when switching. */
static const gclk_parent_config_lut_t _clk_sys_aux_src_mux_configs[] = {
    //{ .parent = &gclk_rp2350_pll_sys_postdiv2_scaler.base, .config_reg_val = 0 }, /* 0x0 → CLKSRC_PLL_SYS */
    //{ .parent = &gclk_rp2350_pll_usb_postdiv2_scaler.base, .config_reg_val = 1 }, /* 0x1 → CLKSRC_PLL_USB */
    { .parent = &gclk_rp2350_pll_sys_bypass_mux.base, .config_reg_val = 0 }, /* 0x0 → CLKSRC_PLL_SYS */
    { .parent = &gclk_rp2350_pll_usb_bypass_mux.base, .config_reg_val = 1 }, /* 0x1 → CLKSRC_PLL_USB */
    { .parent = &gclk_rp2350_rosc_scaler.base,        .config_reg_val = 2 }, /* 0x2 → ROSC_CLKSRC */
    { .parent = &gclk_rp2350_xosc_gate.base,          .config_reg_val = 3 }, /* 0x3 → XOSC_CLKSRC */
    { .parent = &gclk_rp2350_gpin0_src,               .config_reg_val = 4 }, /* 0x4 → CLKSRC_GPIN0 */
    { .parent = &gclk_rp2350_gpin1_src,               .config_reg_val = 5 }, /* 0x5 → CLKSRC_GPIN1 */
};

const gclk_mux_ll_t gclk_rp2350_clk_sys_auxsrc_mux = {
    GCLK_PLAIN_MUX_STATIC_INIT(CLK_SYS_AUXSRC,
                               GCLK_RP2350_CLK_SYS_CTRL,
                               CLOCKS_CLK_SYS_CTRL_AUXSRC_BITS),
    GCLK_PARENT_LUT_STATIC_INIT(_clk_sys_aux_src_mux_configs),
};

/* CLK_SYS SRC: Selects the clock source glitchlessly, can be changed on-the-fly */
static const gclk_parent_config_lut_t _clk_sys_src_mux_configs[] = {
    { .parent = &gclk_rp2350_clk_ref_src_mux.base,    .config_reg_val = 0 },            /* 0x0 → CLK_REF */
    { .parent = &gclk_rp2350_clk_sys_auxsrc_mux.base, .config_reg_val = 1 }, /* 0x1 → CLKSRC_CLK_SYS_AUX */
};

const gclk_mux_ll_t gclk_rp2350_clk_sys_src_mux = {
    GCLK_PLAIN_MUX_STATIC_INIT(CLK_SYS_SRC,
                               GCLK_RP2350_CLK_SYS_CTRL,
                               CLOCKS_CLK_SYS_CTRL_SRC_BITS),
    GCLK_PARENT_LUT_STATIC_INIT(_clk_sys_src_mux_configs),
};

/* this models only the int part as frac part creates a jittery clock */
const gclk_clk_scaler_ll_t gclk_rp2350_clk_sys_div_scaler = {
    GCLK_RP2350_PLAIN_SCALER_STATIC_INIT(CLK_SYS),
    .base.flags.topology_flags = 0,
    .base.fixed_parent         = &gclk_rp2350_clk_sys_src_mux.base,
    .regref                    = { .en  = GCLK_NULL_REG,
                                   .rdy = GCLK_NULL_REG,
                                   .conf = GCLK_RP2350_CLK_SYS_DIV,
                                   .conf_msb = 31,
                                   .conf_lsb = 16,
                                 },
    GCLK_REGVAL_AS_NUMVAL_RANGE16_STATIC_INIT(_clk_int16_div_range),
    .base.flags.scaler_type    = GCLK_DIV,
};


/* CLK_PERI AUXSRC: Selects the auxiliary clock source, will glitch when switching. */
static const gclk_parent_config_lut_t _clk_peri_aux_src_mux_configs[] = {
    { .parent = &gclk_rp2350_clk_sys_div_scaler.base, .config_reg_val = 0 }, /* 0x0 → CLK_SYS */
    { .parent = &gclk_rp2350_pll_sys_bypass_mux.base, .config_reg_val = 1 }, /* 0x1 → CLKSRC_PLL_SYS */
    { .parent = &gclk_rp2350_pll_usb_bypass_mux.base, .config_reg_val = 2 }, /* 0x2 → CLKSRC_PLL_USB */
    { .parent = &gclk_rp2350_rosc_scaler.base,        .config_reg_val = 3 }, /* 0x3 → ROSC_CLKSRC_PH */ // TODO: PH suffix same as ROSC directly?
    { .parent = &gclk_rp2350_xosc_gate.base,          .config_reg_val = 4 }, /* 0x4 → XOSC_CLKSRC */
    { .parent = &gclk_rp2350_gpin0_src,               .config_reg_val = 5 }, /* 0x5 → CLKSRC_GPIN0 */
    { .parent = &gclk_rp2350_gpin1_src,               .config_reg_val = 6 }, /* 0x6 → CLKSRC_GPIN1 */
};

const gclk_mux_ll_t gclk_rp2350_clk_peri_auxsrc_mux = {
    GCLK_PLAIN_MUX_STATIC_INIT(CLK_PERI_AUXSRC,
                               GCLK_RP2350_CLK_PERI_CTRL,
                               CLOCKS_CLK_PERI_CTRL_AUXSRC_BITS),
    GCLK_PARENT_LUT_STATIC_INIT(_clk_peri_aux_src_mux_configs),
};

static const gclk_range8_t _clk_peri_div_range = {
    .min =  1, .max =  3, /* numerical div values */
};

const gclk_clk_scaler_ll_t gclk_rp2350_clk_peri_div_scaler = {
    GCLK_RP2350_PLAIN_SCALER_STATIC_INIT(CLK_PERI_DIV),
    .base.flags.topology_flags = 0,
    .base.fixed_parent         = &gclk_rp2350_clk_peri_auxsrc_mux.base,
    .regref                    = { .en  = GCLK_NULL_REG,
                                   .rdy = GCLK_NULL_REG,
                                   .conf = GCLK_RP2350_CLK_PERI_DIV,
                                   .conf_msb = 17,
                                   .conf_lsb = 16,
                                 },
    GCLK_REGVAL_AS_NUMVAL_RANGE8_STATIC_INIT(_clk_peri_div_range),
    .base.flags.scaler_type    = GCLK_DIV,
};

void gclk_rp2350_rosc_enable(const gclk_t *clk, bool on) {
    gclk_generic_gate_t *gate = to_gclk_generic_gate_t(clk);
    gclk_reg_ref_t regref = gate->regref;
    (void)regref;
    uint32_t enable_val;
    if (on) {
        LOG_DEBUG("%s: enabling %s...\n", __FUNCTION__, gclk_get_name(clk));
        enable_val = ROSC_CTRL_ENABLE_VALUE_ENABLE;
        //TODO
    } else {
        LOG_DEBUG("%s: disabling %s...\n", __FUNCTION__, gclk_get_name(clk));
        enable_val = ROSC_CTRL_ENABLE_VALUE_DISABLE;
        //TODO
    }
    //atomic_set(&ROSC->CTRL, enable_val << regref.conf_lsb);
    //atomic_set(&ROSC->CTRL, enable_val << regref.conf_lsb);
    ROSC->CTRL = (ROSC->CTRL & ~STATIC_BITRANGE_TO_MASK(regref.conf_msb, regref.conf_lsb))
                 | (enable_val << regref.conf_lsb);
}

bool gclk_rp2350_rosc_is_enabled(const gclk_t *clk) {
    gclk_generic_gate_t *gate = to_gclk_generic_gate_t(clk);
    gclk_reg_ref_t regref = gate->regref;
    uint32_t enable_val = gclk_reg_util_read_masked(&ROSC->CTRL,
                                    STATIC_BITRANGE_TO_MASK(regref.conf_msb, regref.conf_lsb));
    if (enable_val == ROSC_CTRL_ENABLE_VALUE_ENABLE) {
        return true;
    } else if (enable_val == ROSC_CTRL_ENABLE_VALUE_DISABLE) {
        return false;
    } else {
        printf("enable val: 0x%08lx\n", enable_val);
        assert(false);
    }
}

const gclk_op_t gclk_rp2350_rosc_gate_ops[] = {
  { .gate_ops = { .is_enabled = gclk_rp2350_rosc_is_enabled,
                  .enable     = gclk_rp2350_rosc_enable,}},
};

const gclk_generic_gate_t gclk_rp2350_rosc_gate = {
.base.separated_ops    = gclk_rp2350_rosc_gate_ops,
/* nominal 11 MHZ (guaranteed range: 4.6MHz to 19.6MHz)
 * NOTE: On RP2350 A3 the ROSC base frequency is quadrupled */
//TODO: check if setting to higher base and dividing down or
//      setting to lower base and multiplying up fits better
.base.fixed_input_freq = 110000, // 11 MHz / 100
.base.flags.is_source  = 1,
.base.flags.gateable   = 1,
.base.name             = "ROSC_BASE",\
.regref                = { .en      = GCLK_NULL_REG,
                           .rdy     = GCLK_NULL_REG,
                           .conf    = GCLK_NULL_REG,
                           .en_bit  = 0,
                           .rdy_bit = 0,
                           .conf_msb = 23,
                           .conf_lsb = 12,
                         },
};

static const gclk_reg_val_factor_lut_t _rosc_mul_val_confs[] = {
    /* The behaviour has not been fully characterised, but the MEDIUM range will be
     * approximately 1.33 times the LOW range, the HIGH range will be 2 times the LOW
     * range and the TOOHIGH range will be 4 times the LOW range.
     * The TOOHIGH range is out of spec.
     * The modelled factor assumes an ROSC_BASE-internal clock of 110000 */
    { .factor = 100,  .reg_val = 0xfa4 }, /* 0xfa4 → LOW */
    { .factor = 133,  .reg_val = 0xfa5 }, /* 0xfa5 → MEDIUM */
    { .factor = 200,  .reg_val = 0xfa7 }, /* 0xfa7 → HIGH */
    //{ .factor = 400,  .reg_val = 0xfa6 }, /* 0xfa6 → TOOHIGH */
};

const gclk_clk_scaler_ll_t gclk_rp2350_rosc_scaler = {
    GCLK_RP2350_PLAIN_SCALER_STATIC_INIT(ROSC),
    .base.flags.topology_flags = 0,
    .base.fixed_parent         = &gclk_rp2350_rosc_gate.base,
    //TODO:
    .regref                    = { .en  = GCLK_NULL_REG,
                                   .rdy = GCLK_NULL_REG,
                                   .conf = GCLK_RP2350_ROSC_CTRL,
                                   .conf_msb = ROSC_CTRL_FREQ_RANGE_MSB,
                                   .conf_lsb = ROSC_CTRL_FREQ_RANGE_LSB,
                                 },
    GCLK_FACTOR_LUT_STATIC_INIT(_rosc_mul_val_confs),
    .base.flags.scaler_type    = GCLK_MUL,
};

/* initializes a basic gate that acts as a simple enable disable switch for a fixed-parent clock */
#define _BASIC_GATE_STATIC_INIT(NAME,PARENT,EN_REG_ID,RDY_REG_ID,EN_BIT,RDY_BIT)\
.base.separated_ops    = gclk_plain_gate_ops,\
.base.fixed_parent     = PARENT,\
.base.flags.gateable   = 1,\
.base.name             = #NAME,\
.regref                = { .en      = EN_REG_ID,\
                           .rdy     = RDY_REG_ID,\
                           .en_bit  = EN_BIT,\
                           .rdy_bit = RDY_BIT, }

const gclk_generic_gate_t gclk_rp2350_clk_peri_gate = {
_BASIC_GATE_STATIC_INIT(CLK_PERI,
                        &gclk_rp2350_clk_peri_div_scaler.base,
                        GCLK_RP2350_CLK_PERI_CTRL,
                        GCLK_RP2350_CLK_PERI_CTRL,
                        CLOCKS_CLK_PERI_CTRL_ENABLE_LSB,
                        CLOCKS_CLK_PERI_CTRL_ENABLED_LSB),
};


static const gclk_parent_config_lut_t _clk_gpout0_auxsrc_mux_configs[] = {
    { .parent = &gclk_rp2350_pll_sys_bypass_mux.base, .config_reg_val = 0 },   /* 0x0 → CLKSRC_PLL_SYS */
    { .parent = &gclk_rp2350_gpin0_src,               .config_reg_val = 1 },   /* 0x1 → CLKSRC_GPIN0 */
    { .parent = &gclk_rp2350_gpin1_src,               .config_reg_val = 2 },   /* 0x2 → CLKSRC_GPIN1 */
    { .parent = &gclk_rp2350_pll_usb_bypass_mux.base, .config_reg_val = 3 },   /* 0x3 → CLKSRC_PLL_USB */
  //{ .parent = &gclk_rp2350_pll_usb_bypass_mux.base, .config_reg_val = 4 },   /* 0x4 → CLKSRC_PLL_USB_PRIMARY_REF_OPCG */
    { .parent = &gclk_rp2350_rosc_scaler.base,        .config_reg_val = 5 },   /* 0x5 → ROSC_CLKSRC */
    { .parent = &gclk_rp2350_xosc_gate.base,          .config_reg_val = 6 },   /* 0x6 → XOSC_CLKSRC */
  //{ .parent = &gclk_rp2350_lposc_gate.base,         .config_reg_val = 7 },   /* 0x7 → LPOSC_CLKSRC */
    { .parent = &gclk_rp2350_clk_sys_div_scaler.base, .config_reg_val = 8 },   /* 0x8 → CLK_SYS */
  //{ .parent = &gclk_rp2350_clk_usb_gate.base,       .config_reg_val = 9 },   /* 0x9 → CLK_USB */
  //{ .parent = &gclk_rp2350_clk_adc_gate.base,       .config_reg_val = 0xa }, /* 0xa → CLK_ADC */
  //{ .parent = &gclk_rp2350_clk_ref_scaler.base,     .config_reg_val = 0xb }, /* 0xb → CLK_REF */
    { .parent = &gclk_rp2350_clk_peri_gate.base,      .config_reg_val = 0xc }, /* 0xc → CLK_PERI */
  //{ .parent = &gclk_rp2350_clk_hstx_gate.base,      .config_reg_val = 0xd }, /* 0xd → CLK_HSTX */
  //{ .parent = &gclk_rp2350_clk_otpclk2.base,        .config_reg_val = 0xe }, /* 0xe → OTP_CLK2FC */
};

const gclk_mux_ll_t gclk_rp2350_clk_gpout0_auxsrc_mux = {
    GCLK_PLAIN_MUX_STATIC_INIT(CLK_GPOUT0_AUXSRC,
                               GCLK_RP2350_CLK_GPOUT0_CTRL,
                               CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_BITS),
    GCLK_PARENT_LUT_STATIC_INIT(_clk_gpout0_auxsrc_mux_configs),
};

const gclk_clk_scaler_ll_t gclk_rp2350_clk_gpout0_div_scaler = {
    GCLK_RP2350_PLAIN_SCALER_STATIC_INIT(CLK_GPOUT0_DIV),
    .base.flags.topology_flags = 0,
    .base.fixed_parent         = &gclk_rp2350_clk_gpout0_auxsrc_mux.base,
    .regref                    = { .en  = GCLK_NULL_REG,
                                   .rdy = GCLK_NULL_REG,
                                   .conf = GCLK_RP2350_CLK_GPOUT0_DIV,
                                   .conf_msb = 31,
                                   .conf_lsb = 16,
                                 },
    GCLK_REGVAL_AS_NUMVAL_RANGE16_STATIC_INIT(_clk_int16_div_range),
    .base.flags.scaler_type    = GCLK_DIV,
};

const gclk_generic_gate_t gclk_rp2350_clk_gpout0_gate = {
_BASIC_GATE_STATIC_INIT(CLK_GPOUT0,
                        &gclk_rp2350_clk_gpout0_div_scaler.base,
                        GCLK_RP2350_CLK_GPOUT0_CTRL,
                        GCLK_RP2350_CLK_GPOUT0_CTRL,
                        CLOCKS_CLK_GPOUT0_CTRL_ENABLE_LSB,
                        CLOCKS_CLK_GPOUT0_CTRL_ENABLED_LSB),
};

/* the LOC value is implicitly encoded by the array position */
static const gpio_t _gpout0_pins[] = { 13, 21 };
//TODO: add support for the other clock outputs
//static const gpio_t _gpout1_pins[] = { 15, 23 };
//static const gpio_t _gpout2_pins[] = { 24 };
//static const gpio_t _gpout3_pins[] = { 25 };

//TODO: get replace with correct vendor header include
#define GPIO_FUNC_GPCK (9)
#define HAS_PADS_BANK0_ISOLATION (1)

/** LSB position of FUNCSEL (needed for selecting gpio alternate function for clk output) */
#define IO_BANK0_GPIO0_CTRL_FUNCSEL_LSB (0)

void gpio_set_gpck_function(unsigned pin) {
    /* get the pad register for the target GPIO */
    uint32_t *pad_reg = calculate_gpio_pad_register_addr(pin);

    // according to the SDK code: Set input enable on, output disable off
    *pad_reg = (*pad_reg ^ PADS_BANK0_GPIO0_IE_BITS) & (PADS_BANK0_GPIO0_IE_BITS | PADS_BANK0_GPIO0_OD_BITS);

    uint32_t *io_ctrl_reg = calculate_gpio_io_ctrl_register_addr(pin);

    // according to SDK: "Zero all fields apart from fsel;
    // we want this IO to do what the peripheral tells it. This doesn't affect e.g.
    // pullup/pulldown, as these are in pad controls.
    *io_ctrl_reg = GPIO_FUNC_GPCK << IO_BANK0_GPIO0_CTRL_FUNCSEL_LSB;

#if HAS_PADS_BANK0_ISOLATION
    // according to SDK: Remove pad isolation now that the correct peripheral is in control of the pad
    *pad_reg = *pad_reg & (~PADS_BANK0_GPIO0_ISO_BITS);
#endif
}

int gclk_enable_pin_output(const gclk_t *clk, const gpio_t pin) {
    const gpio_t *gpios = NULL;
    size_t len = 0;

    if (clk == &gclk_rp2350_clk_gpout0_gate.base) {
        gpios = _gpout0_pins;
        len = ARRAY_SIZE(_gpout0_pins);
    } else {
        return ENABLE_PIN_OUTPUT_INVALID_CLOCK;
    }

    for (size_t i = 0; i < len; i++) {
        if (pin == gpios[i]) {
            printf("setup clock output for %s on GPIO %lu\n", gclk_get_name(clk), pin);
            gpio_set_gpck_function(pin);

            return ENABLE_PIN_OUTPUT_OK;
        }
    }
    return ENABLE_PIN_OUTPUT_INVALID_PIN;
}

gclk_t const *gclock_handle_for_core_freq = &gclk_rp2350_clk_sys_div_scaler.base;

/* making this static leads to copies if included from multiple places, so avoid that */
const gclk_t *gclks[GCLK_NUM_OF_CLOCKS] = {
    // clock sources vvv
    &gclk_rp2350_xosc_gate.base,
    &gclk_rp2350_gpin0_src,
    &gclk_rp2350_gpin1_src,

    // pll_sys vvv
    &gclk_rp2350_pll_sys_refdiv_scaler.base,
    &gclk_rp2350_pll_sys_fbdiv_scaler.base,
    &gclk_rp2350_pll_sys_vco_gate.base,
    &gclk_rp2350_pll_sys_postdiv1_scaler.base,
    &gclk_rp2350_pll_sys_postdiv2_scaler.base,
    &gclk_rp2350_pll_sys_bypass_mux.base,

    // pll_usb vvv
    &gclk_rp2350_pll_usb_refdiv_scaler.base,
    &gclk_rp2350_pll_usb_fbdiv_scaler.base,
    &gclk_rp2350_pll_usb_vco_gate.base,
    &gclk_rp2350_pll_usb_postdiv1_scaler.base,
    &gclk_rp2350_pll_usb_postdiv2_scaler.base,
    &gclk_rp2350_pll_usb_bypass_mux.base,

    // clk_sys vvv
    &gclk_rp2350_clk_sys_auxsrc_mux.base,
    &gclk_rp2350_clk_sys_src_mux.base,
    &gclk_rp2350_clk_sys_div_scaler.base,

    // peripheral clocks vvv
    &gclk_rp2350_clk_peri_auxsrc_mux.base,
    &gclk_rp2350_clk_peri_div_scaler.base,
    &gclk_rp2350_clk_peri_gate.base,
    &gclk_rp2350_rosc_scaler.base,
    &gclk_rp2350_rosc_gate.base,
    &gclk_rp2350_clk_gpout0_auxsrc_mux.base,

    // gpio output clocks vvv
    &gclk_rp2350_clk_gpout0_gate.base,
    &gclk_rp2350_clk_gpout0_div_scaler.base,
};

