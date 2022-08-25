/*
 * Copyright (C) 2021 HAW Hamburg
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     cpu_efm32_common
 * @{
 *
 * @file
 * @brief       Implementation of the gclk instances for EFM32PG12b
 *
 * @author      Michel Rottleuthner <michel.rottleuthner@haw-hamburg.de>
 * @}
 */

#include <stdlib.h> /* needed for abs() */
#include <stdint.h>
#include "kernel_defines.h" // needed for container_of macro
#include "cpu.h" // needed for RCC register definitions
#include "gclk.h"
#include "gclk/generic_gate.h"   /* @todo: remove/replace (only needed for accessing the gate instances */
#include "gclk/generic_mux.h"    /* @todo: remove/replace (only needed for accessing the mux instances */
#include "gclk/generic_scaler.h" /* @todo: remove/replace (only needed for accessing the mux instances */
#include "gclk_efm32_types.h"
#include "bitarithm.h"

/* included for the re-init callbacks */
#include "xtimer.h"
#include "stdio_base.h"

#define LOG_LEVEL LOG_NONE
#include "log.h"

typedef struct {
  uint16_t msb: 5;
  uint16_t lsb: 5;
} gclk_register_mask_t;


#define GCLK_EFM32_MUX_STATIC_INIT_SEL_STATUS_REG(SEL_REG,STA_REG,SEL_MSK,STA_MSK) \
.regs.select_reg_idx     = SEL_REG,\
.regs.status_reg_idx     = STA_REG,\
.regs.select_msb         = STATIC_BITMASK_MSB(SEL_MSK),\
.regs.select_lsb         = STATIC_BITMASK_LSB(SEL_MSK),\
.regs.status_msb         = STATIC_BITMASK_MSB(STA_MSK),\
.regs.status_lsb         = STATIC_BITMASK_LSB(STA_MSK)

/**
 * @brief returns the pointer to the specific clock instance that holds the given generic clock base
 */
static inline gclk_efm32_mux_t *to_gclk_efm32_mux_t(const gclk_t *baseptr)
{
    return container_of(baseptr, gclk_efm32_mux_t, base);
}

extern uint32_t volatile * const conf_regs[];

static inline volatile uint32_t * _efm32_mux_select_reg(gclk_efm32_mux_t *mux) {
    return conf_regs[mux->regs.select_reg_idx];
}

static inline volatile uint32_t * _efm32_mux_status_reg(gclk_efm32_mux_t *mux) {
    return conf_regs[mux->regs.status_reg_idx];
}

static inline uint32_t _msb_lsb_idx_to_mask(uint8_t msb, uint8_t lsb) {
    uint32_t mask = 0xFFFFFFFF;
    mask = mask >> msb;
    mask = mask << lsb;
    mask = mask << (31 - msb);
    mask = mask >> (31 - msb);
    return mask;
}

static inline uint32_t _efm32_mux_select_mask(gclk_efm32_mux_t *mux) {
   return _msb_lsb_idx_to_mask(mux->regs.select_msb, mux->regs.select_lsb);
}

static inline uint32_t _efm32_mux_status_mask(gclk_efm32_mux_t *mux) {
   return _msb_lsb_idx_to_mask(mux->regs.status_msb, mux->regs.status_lsb);
}

/* Implementation of the get_parent interface for a mux that exposes its currently
 * selected parent via a status register, while a new selection is programmed
 * with a separate selection register */
static const gclk_t* _efm32_mux_stareg_get_parent(const gclk_t *clk) {
    gclk_efm32_mux_t *mux = to_gclk_efm32_mux_t(clk);
    uint32_t cur_config_reg_val = gclk_reg_util_read_masked(_efm32_mux_status_reg(mux),
                                                            _efm32_mux_status_mask(mux));

    LOG_DEBUG("%s: %s (config val) 0x%08lx\n", __FUNCTION__, gclk_get_name(clk), cur_config_reg_val);

    return gclk_regval2parent(clk, cur_config_reg_val);
}

static void _efm32_mux_selreg_set_parent(const gclk_t *clk, unsigned int idx) {
    gclk_efm32_mux_t *mux = to_gclk_efm32_mux_t(clk);

    const gclk_t *new_parent = gclk_idx2parent(clk,idx);
    uint32_t new_regval = gclk_parent2regval(clk, new_parent);

    gclk_reg_util_write_masked(_efm32_mux_select_reg(mux), _efm32_mux_select_mask(mux),
                               new_regval);
}

/* Implementation of the mux_ops interface for an efm32 mux that can be switched to
 * another source via a select register and its state can be read via a separate
 * status register */
static const gclk_op_t _efm32_plain_staselreg_mux_ops[] = {
  { .mux_ops = { .get_parent = _efm32_mux_stareg_get_parent,
                 .set_parent = _efm32_mux_selreg_set_parent,}},
};

/* Meant to be used with a flag that maps the factors 1-32 to config reg values from 0 to 31 (5 bits).
 * TODO: check if the flag limitation is still valid and if possible improve it */
static const gclk_range8_t _hfclkpresc_factor_range = {
    .min =  1, .max =  32,
    /* the numeric value that can be written to the register is 0 - 31 */
};
/* Meant to be used with a flag that maps the factors 1-512 to config reg values from 0 to 511 (9 bits).
 * TODO: check if the flag limitation is still valid and if possible improve it */
static const gclk_range16_t _hfperpresc_factor_range = {
    .min =  1, .max =  512,
    /* the numeric value that can be written to the register is 0 - 511 */
};

#if 0
/* high frequency crystal oscillator freq range */
static const gclk_range32_t _hfxo_freq_constraint = {
    .min =  38000000,  .max =  40000000,
};

/* high frequency RC oscillator freq range */
static const gclk_range32_t _hfrco_freq_constraint = {
    .min =  1000000,  .max =  38000000,
};

static const uint32_t _lfxo_fixed_freq   = 32768;
static const uint32_t _lfrco_fixed_freq  = 32768;
static const uint32_t _ulfrco_fixed_freq = 1000;
#endif

/* auxilary high frequency RC oscillator freq range */
//static const gclk_range32_t _auxhfrco_freq_constraint = {
//    .min =  1000000,  .max =  38000000,
//};

/* min and max value of HFCORECLK prescaler (as numeric value, i.e., not register value) */
static const gclk_range16_t _hfcorepresc_factor_range = {
    .min =  1,  .max =  0x1FF + 1,
};

/* min and max value of HFCORECLK prescaler (as numeric value, i.e., not register value) */
static const gclk_range8_t _hfexpclkpresc_factor_constraint = {
    .min =  1,  .max =  0x1F + 1,
};

/* forward declarations to be used in other nodes without taking care of definition order */
const gclk_efm32_mux_t gclk_efm32_hfsrcclk_mux;
const gclk_efm32_gate_t gclk_efm32_hfxo_base_gate;
const gclk_t gclk_efm32_hfperclk_wtimer;
const gclk_clk_scaler_ll_t gclk_efm32_hfrco_scaler;
const gclk_t gclk_efm32_ulfrco_src;
const gclk_clk_scaler_ll_t gclk_efm32_hfperpresc_scaler;
const gclk_generic_gate_t gclk_efm32_le_gate;
const gclk_clk_scaler_ll_t gclk_efm32_hfcorepresc_scaler;
const gclk_generic_gate_t gclk_efm32_hfperclk_gate;
const gclk_mux_ll_t gclk_efm32_dpllref_mux;
const gclk_clk_scaler_ll_t gclk_efm32_dppl_n_mul_scaler;

static inline uint32_t volatile *_gate_enable_reg(gclk_efm32_gate_t *gate)
{
    return conf_regs[gate->en_dis_reg_idx];
}

static inline uint32_t volatile * _gate_rdy_reg(gclk_efm32_gate_t *gate)
{
    return conf_regs[gate->rdy_reg_idx];
}

static inline uint32_t volatile * _gate_en_state_reg(gclk_efm32_gate_t *gate)
{
    return conf_regs[gate->ens_reg_idx];
}

static inline uint32_t _gate_en_bit(gclk_efm32_gate_t *gate)
{
    return gate->enable_bit;
}

static inline uint32_t _gate_dis_bit(gclk_efm32_gate_t *gate)
{
    return gate->disable_bit;
}

static inline uint32_t _gate_rdy_bit(gclk_efm32_gate_t *gate)
{
    return gate->ready_bit;
}

static inline uint32_t _gate_en_state_bit(gclk_efm32_gate_t *gate)
{
    return gate->en_state_bit;
}

static inline uint32_t _gate_en_mask(gclk_efm32_gate_t *gate)
{
    return 1 << _gate_en_bit(gate);
}

static inline uint32_t _gate_dis_mask(gclk_efm32_gate_t *gate)
{
    return 1 << _gate_dis_bit(gate);
}

static inline uint32_t _gate_rdy_mask(gclk_efm32_gate_t *gate)
{
    return 1 << _gate_rdy_bit(gate);
}

static inline uint32_t _gate_en_state_mask(gclk_efm32_gate_t *gate)
{
    return 1 << _gate_en_state_bit(gate);
}

/**
 * @brief returns the pointer to the specific clock instance that holds the given generic clock base
 */
static inline gclk_efm32_gate_t *to_gclk_efm32_gate_t(const gclk_t *baseptr)
{
    return container_of(baseptr, gclk_efm32_gate_t, base);
}


/* Implementation of the is_enabled interface for a gate that exposes its
 * enabled state via a status register. The enable and disable action
 * is accessible via a separate command regiter that maps individual
 * bits that trigger enable/disable actions.
 **/
static bool _efm32_gate_ensreg_is_enabled(const gclk_t *clk) {
    gclk_efm32_gate_t *gate = to_gclk_efm32_gate_t(clk);
    uint32_t volatile * const ensreg = _gate_en_state_reg(gate);

    /* TODO: on EFM32 the active state is determined by a read only register. I.e.,
     *       whether the clock was active can not be read from the enable
     *       register as it is write-only. There is a separate enable status reg. */
    /* if no enable register is defined, always assume the gate was previously enabled */
    if (ensreg) {
        return *ensreg & _gate_en_state_mask(gate);
    } else {
        return true;
    }
}

static void _efm32_gate_selreg_enable(const gclk_t *clk, bool on) {
    gclk_efm32_gate_t *gate = to_gclk_efm32_gate_t(clk);

    uint32_t volatile * const edreg  = _gate_enable_reg(gate);
    uint32_t volatile * const rdyreg = _gate_rdy_reg(gate);

    /* only access a valid register ( always-on gate could model a fixed source) */
    if (edreg) {
        if (on) {
            gclk_reg_util_set_mask(edreg, _gate_en_mask(gate));
        } else {
            gclk_reg_util_set_mask(edreg, _gate_dis_mask(gate));
        }
    }

    /* TODO: it might be preferable in some situations to skip this check to speed up
     *       operation. A flag that indicates if the operation should be blocking or
     *       asynchronous would be benefitial for that. */
    if (rdyreg) {/* only wait for a ready flag if this register is defined */
        if (on) {
            while (!(*(rdyreg) & _gate_rdy_mask(gate))) {}
        } else {
            while (*(rdyreg) & _gate_rdy_mask(gate)) {}
        }
    }
}

/* Implementation of the gate_ops interface for an efm32 gate that is interfaced
 * via a read-only enabled state register to get the current enable state.
 * Gate control is exposed via a spearate command register that has individual bits
 * to trigger enable/disable */
static const gclk_op_t _efm32_bare_gate_ensselreg_ops[] = {
  { .gate_ops = { .is_enabled = _efm32_gate_ensreg_is_enabled,
                  .enable     = _efm32_gate_selreg_enable,}},
};

#define GCLK_EFM32_OSCENCMD_GATE_STATIC_INIT(NAME,FIXFREQ)\
.base.separated_ops    = _efm32_bare_gate_ensselreg_ops,\
.base.fixed_input_freq = FIXFREQ,\
.base.flags.is_source  = 1,\
.base.flags.gateable   = 1,\
.base.name             = #NAME"_BASE",\
.en_dis_reg_idx        = GCLK_EFM32_CMU_OSCENCMD,\
.rdy_reg_idx           = GCLK_EFM32_CMU_STATUS,\
.ens_reg_idx           = GCLK_EFM32_CMU_STATUS,\
.enable_bit            = _CMU_OSCENCMD_##NAME##EN_SHIFT,\
.disable_bit           = _CMU_OSCENCMD_##NAME##DIS_SHIFT,\
.ready_bit             = _CMU_STATUS_##NAME##RDY_SHIFT,\
.en_state_bit          = _CMU_STATUS_##NAME##ENS_SHIFT

/* gates that act as a clock source that always has a fixed frequency that is either enabled or disabled
 * NOTE: when using AUXHFRCO as debug trace clock, it must be stopped before entering EM2 or EM3
 * NOTE: AUXHFRCO is disabled automatically when entering EM2, EM3, or EM4
 * NOTE: if used by ADC or LESENSE it is available in EM2/EM3
 * TODO: This clock poses a special case: the configuration values needed to setup a
 *       specific frequency must be taken from another register that contains factory
 *       calibration values.
 *       To model this there are a few potential options:
 *          - a new mapping type could be added that looks up config values from pointers
 *            - for pure "whole-datatype" lookups this could be easy
 *            - in reality it probably requires an additional mask to not overwrite reserved bits
 *            - special cases could also require multiple lookup regs and masks (?)
 *          - custom option-handlers with private data sections could also be used to abstract this
 *            - NOTE: even the current private data could be moved to a special "common" private
 *                    data structure */
const gclk_efm32_gate_t gclk_efm32_auxhfrco_base_gate = {
  /* A virtual base value to build a scaler with several
   * discrete values from 1 to 38 MHz */
  GCLK_EFM32_OSCENCMD_GATE_STATIC_INIT(AUXHFRCO,1000000),
};

const gclk_efm32_gate_t gclk_efm32_hfrco_base_gate = {
  /* A virtual base value to build a scaler with several
   * discrete values from 1 to 38 MHz */
  GCLK_EFM32_OSCENCMD_GATE_STATIC_INIT(HFRCO,1000000),
};

const gclk_efm32_gate_t gclk_efm32_hfxo_base_gate = {
  /* Crystal freq taken from EFM32_HFXO_FREQ in cpu/efm32/families/efm32pg12b/system.c
   * This should be defined by a board config file that describes the available crystal */
  GCLK_EFM32_OSCENCMD_GATE_STATIC_INIT(HFXO,40000000),
};

const gclk_efm32_gate_t gclk_efm32_lfrco_base_gate = {
  GCLK_EFM32_OSCENCMD_GATE_STATIC_INIT(LFRCO,32768),
};

const gclk_efm32_gate_t gclk_efm32_lfxo_base_gate = {
  GCLK_EFM32_OSCENCMD_GATE_STATIC_INIT(LFXO,32768),
};

/* @TODO for now we model the DPLL enable as a plain gate that sits after the
 * DPLL scalers. The split scalers are fine as is. But the enable part should be
 * mutually exlusive with the default HFRCO scaling options as they are overwritten
 * with the DPLL configuration. An alternative way to model this would be a virtual
 * mux that sits after the HFRCO scaler. That would then reuse the enable bit of DPLL
 * as a selection-configuration value as parent selection. The mux would need
 * custom register descriptions though, as it uses separate configure (write only)
 * and status bits (read only) instead of a combined read-write value.*/
const gclk_efm32_gate_t gclk_efm32_dpll_gate = {
  .base.separated_ops    = _efm32_bare_gate_ensselreg_ops,
  .base.fixed_parent     = &gclk_efm32_dppl_n_mul_scaler.base,
  .base.flags.gateable   = 1,
  .base.name             = "DPLL_EN",
  .en_dis_reg_idx        = GCLK_EFM32_CMU_OSCENCMD,
  .rdy_reg_idx           = GCLK_EFM32_CMU_STATUS,
  .ens_reg_idx           = GCLK_EFM32_CMU_STATUS,
  .enable_bit            = _CMU_OSCENCMD_DPLLEN_SHIFT,
  .disable_bit           = _CMU_OSCENCMD_DPLLDIS_SHIFT,
  .ready_bit             = _CMU_STATUS_DPLLRDY_SHIFT,
  .en_state_bit          = _CMU_STATUS_DPLLENS_SHIFT
};

const gclk_efm32_gate_t gclk_efm32_clkin0_base_gate = {
    /* here we could just reuse this driver even though there are
     * actually no registers for enable / disable / state on this
     * specific instance. The driver can handle those registers
     * being set to NULL (or respective uninitialized value).
     * Actually setting it to NULL should also be fine as the gclk
     * implementation should just access fixed_input_freq becasue of the
     * is_source flag being set.*/
    .base.separated_ops = _efm32_bare_gate_ensselreg_ops,
    .base.name         = "CLKIN0_BASE",
    /* TODO: overwrite fixed frequency with board/app-defined frequency */
    .base.fixed_input_freq   = 0,
    /* This must be specified to tell the generic clock driver that
     * this clock has a fixed frequency */
    .base.flags.is_source = 1,
};

/* This kind of gate just reuses the plain gate implementation that is interfaced
 * via a read/write register that uses a single bit to control enable/disable.
 * The optionaly register to indicate finishing of clock startup is not used. */
#define GCLK_EFM32_HFPERCLKEN0_GATE_STATIC_INIT(NAME)\
.base.separated_ops  = gclk_plain_gate_ops,\
.base.flags.gateable = 1,\
.base.name           = #NAME,\
.base.fixed_parent   = &gclk_efm32_hfperpresc_scaler.base,\
.regref              =  { .en      = GCLK_EFM32_CMU_HFPERCLKEN0,\
                          .en_bit  = _CMU_HFPERCLKEN0_##NAME##_SHIFT, }

const gclk_generic_gate_t gclk_efm32_trng0_gate = {
GCLK_EFM32_HFPERCLKEN0_GATE_STATIC_INIT(TRNG0),
};

const gclk_generic_gate_t gclk_efm32_csen_gate = {
GCLK_EFM32_HFPERCLKEN0_GATE_STATIC_INIT(CSEN),
};

const gclk_generic_gate_t gclk_efm32_vdac0_gate = {
GCLK_EFM32_HFPERCLKEN0_GATE_STATIC_INIT(VDAC0),
};

const gclk_generic_gate_t gclk_efm32_idac0_gate = {
GCLK_EFM32_HFPERCLKEN0_GATE_STATIC_INIT(IDAC0),
};

const gclk_generic_gate_t gclk_efm32_adc0_gate = {
GCLK_EFM32_HFPERCLKEN0_GATE_STATIC_INIT(ADC0),
};

const gclk_generic_gate_t gclk_efm32_cryotimer_gate = {
GCLK_EFM32_HFPERCLKEN0_GATE_STATIC_INIT(CRYOTIMER),
};

const gclk_generic_gate_t gclk_efm32_acmp1_gate = {
GCLK_EFM32_HFPERCLKEN0_GATE_STATIC_INIT(ACMP1),
};

const gclk_generic_gate_t gclk_efm32_acmp0_gate = {
GCLK_EFM32_HFPERCLKEN0_GATE_STATIC_INIT(ACMP0),
};

const gclk_generic_gate_t gclk_efm32_i2c1_gate = {
GCLK_EFM32_HFPERCLKEN0_GATE_STATIC_INIT(I2C1),
};

const gclk_generic_gate_t gclk_efm32_i2c0_gate = {
GCLK_EFM32_HFPERCLKEN0_GATE_STATIC_INIT(I2C0),
};

const gclk_generic_gate_t gclk_efm32_usart3_gate = {
GCLK_EFM32_HFPERCLKEN0_GATE_STATIC_INIT(USART3),
};

const gclk_generic_gate_t gclk_efm32_usart2_gate = {
GCLK_EFM32_HFPERCLKEN0_GATE_STATIC_INIT(USART2),
};

const gclk_generic_gate_t gclk_efm32_usart1_gate = {
GCLK_EFM32_HFPERCLKEN0_GATE_STATIC_INIT(USART1),
};

const gclk_generic_gate_t gclk_efm32_usart0_gate = {
GCLK_EFM32_HFPERCLKEN0_GATE_STATIC_INIT(USART0),
};

const gclk_generic_gate_t gclk_efm32_wtimer1_gate = {
GCLK_EFM32_HFPERCLKEN0_GATE_STATIC_INIT(WTIMER1),
};

const gclk_generic_gate_t gclk_efm32_wtimer0_gate = {
GCLK_EFM32_HFPERCLKEN0_GATE_STATIC_INIT(WTIMER0),
};

const gclk_generic_gate_t gclk_efm32_timer1_gate = {
GCLK_EFM32_HFPERCLKEN0_GATE_STATIC_INIT(TIMER1),
};

const gclk_generic_gate_t gclk_efm32_timer0_gate = {
GCLK_EFM32_HFPERCLKEN0_GATE_STATIC_INIT(TIMER0),
};

static const gclk_reg_val_ptr_lut_t auxhfrcoctrl_ptr_lut[] = {
    { .factor = 4,   .reg_val_ptr = &DEVINFO->AUXHFRCOCAL0},
    { .factor = 7,   .reg_val_ptr = &DEVINFO->AUXHFRCOCAL3},
    { .factor = 13,  .reg_val_ptr = &DEVINFO->AUXHFRCOCAL6},
    { .factor = 16,  .reg_val_ptr = &DEVINFO->AUXHFRCOCAL7},
    { .factor = 19,  .reg_val_ptr = &DEVINFO->AUXHFRCOCAL8},
    { .factor = 26,  .reg_val_ptr = &DEVINFO->AUXHFRCOCAL10},
    { .factor = 32,  .reg_val_ptr = &DEVINFO->AUXHFRCOCAL11},
    { .factor = 38,  .reg_val_ptr = &DEVINFO->AUXHFRCOCAL12},
};

static const gclk_reg_val_ptr_lut_t hfrcoctrl_ptr_lut[] = {
    { .factor = 4,   .reg_val_ptr = &DEVINFO->HFRCOCAL0},
    { .factor = 7,   .reg_val_ptr = &DEVINFO->HFRCOCAL3},
    { .factor = 13,  .reg_val_ptr = &DEVINFO->HFRCOCAL6},
    { .factor = 16,  .reg_val_ptr = &DEVINFO->HFRCOCAL7},
    { .factor = 19,  .reg_val_ptr = &DEVINFO->HFRCOCAL8},
    { .factor = 26,  .reg_val_ptr = &DEVINFO->HFRCOCAL10},
    { .factor = 32,  .reg_val_ptr = &DEVINFO->HFRCOCAL11},
    { .factor = 38,  .reg_val_ptr = &DEVINFO->HFRCOCAL12},
};

/* Reuses the plain scaler driver that only implements the scale_ops interface
 * for a scaler that is interfaced via a bitfield in a read/write register that
 * exposes the current selection and also allows changing it. */
#define GCLK_EFM32_PLAIN_SCALER_STATIC_INIT(NAME)\
.base.separated_ops  = gclk_plain_scaler_ops,\
.base.flags.scalable = 1,\
.base.name           = #NAME

/* Reuses the generic driver for a fixed (read-only scaler) */
const gclk_clk_scaler_ll_t gclk_efm32_hfrcodiv2_scaler = {
      GCLK_PLAIN_FIXED_SCALER_STATIC_INIT(HFRCODIV2,2),
      .base.flags.scaler_type  = GCLK_DIV,
      .base.fixed_parent       = &gclk_efm32_hfrco_scaler.base,
};

const gclk_clk_scaler_ll_t gclk_efm32_hfcoreclkcoretex_scaler = {
      GCLK_PLAIN_FIXED_SCALER_STATIC_INIT(HFCORECLK_CORETEX,1024),
      .base.flags.scaler_type  = GCLK_DIV,
      .base.fixed_parent       = &gclk_efm32_hfcorepresc_scaler.base,
};

/* this clock represents the prescaler that outputs the clock named HFCLK */
const gclk_clk_scaler_ll_t gclk_efm32_hfclk_scaler = {
    GCLK_EFM32_PLAIN_SCALER_STATIC_INIT(HFCLK),
    .base.flags.topology_flags = 0, // TODO: lookup what applies here
                              // | GCLK_STOP_FOR_UPDATE
                              // | GCLK_STOP_PARENT_FOR_UPDATE
                              // | GCLK_STOP_CHILDREN_FOR_UPDATE
                              // | GCLK_ONLY_FOR_STARTUP
    .base.fixed_parent         = &gclk_efm32_hfsrcclk_mux.base,
    /* regref should be converted to some generic format that allows using it as pointer OR
     * regref. Another pointer could then add individual custom functions (or one whole driver)
     * that unifies hardware access procedures */
    .regref                    = { .en  = GCLK_NULL_REG,
                                   .rdy = GCLK_NULL_REG,
                                   .conf = GCLK_EFM32_CMU_HFPRESC,
                                   .conf_msb = STATIC_BITMASK_MSB(_CMU_HFPRESC_PRESC_MASK),
                                   .conf_lsb = STATIC_BITMASK_LSB(_CMU_HFPRESC_PRESC_MASK),
                                 },
    GCLK_IDX_AS_REGVAL_RANGE8_STATIC_INIT(_hfclkpresc_factor_range),
    /* above transates to this: */
    //.range16                   = &_hfclkpresc_factor_range,
    //.base.flags.conf_cnt       = _hfclkpresc_factor_range.max - _hfclkpresc_factor_range.min + 1,

    //.out_freq_constraint       = &_vco_freq_out_constraint,
    .base.flags.scaler_type    = GCLK_DIV,
};

static const uint8_t _hfclkle_div_val_u8_list[] = { 2, 4 };

const gclk_clk_scaler_ll_t gclk_efm32_hfclkle_scaler = {
    GCLK_EFM32_PLAIN_SCALER_STATIC_INIT(HFCLKLE),
    .base.flags.topology_flags = 0, // TODO: lookup what applies here
    .base.fixed_parent         = &gclk_efm32_le_gate.base,
    .regref                    = { .conf = GCLK_EFM32_CMU_HFPRESC,
                                   .conf_msb = STATIC_BITMASK_MSB(_CMU_HFPRESC_HFCLKLEPRESC_MASK),
                                   .conf_lsb = STATIC_BITMASK_LSB(_CMU_HFPRESC_HFCLKLEPRESC_MASK),
                                 },
    GCLK_LIST8_STATIC_INIT(_hfclkle_div_val_u8_list),
    .base.flags.scaler_type    = GCLK_DIV,
};

/* This scaler uses a pointer-based LUT to lookup factory calibrated values for the config reg */
const gclk_clk_scaler_ll_t gclk_efm32_auxclk_scaler = {
    GCLK_EFM32_PLAIN_SCALER_STATIC_INIT(AUXCLK),
    .base.flags.topology_flags = 0, // TODO: lookup what applies here
                              // | GCLK_STOP_FOR_UPDATE
                              // | GCLK_STOP_PARENT_FOR_UPDATE
                              // | GCLK_STOP_CHILDREN_FOR_UPDATE
                              // | GCLK_ONLY_FOR_STARTUP
    .base.fixed_parent         = &gclk_efm32_auxhfrco_base_gate.base,
    .regref                    = { //.en  = GCLK_EFM32_CMU_OSCENCMD,
                                   //.dis = _CMU_OSCENCMD_AUXHFRCODIS_SHIFT,
                                   /* there is an additional busy flag that forbids updating the config reg */
                                   //.rdy = GCLK_EFM32_CMU_STATUS,
                                   .conf = GCLK_EFM32_CMU_AUXHFRCOCTRL,
                                   //.en_bit  = _CMU_OSCENCMD_AUXHFRCOEN_SHIFT,
                                   //_CMU_OSCENCMD_AUXHFRCODIS_SHIFT
                                   //.rdy_bit = _CMU_STATUS_AUXHFRCORDY_SHIFT,
                                   .conf_msb = STATIC_BITMASK_MSB(_CMU_AUXHFRCOCTRL_MASK),
                                   .conf_lsb = STATIC_BITMASK_LSB(_CMU_AUXHFRCOCTRL_MASK), },
    /* TODO: below is not necessarily needed as the value config limits this anyway */
    //.out_freq_constraint       = &_auxhfrco_freq_constraint,
    .base.flags.scaler_type    = GCLK_MUL,
    GCLK_PTR_LUT_STATIC_INIT(auxhfrcoctrl_ptr_lut),
};

/* This scaler uses a pointer-based LUT to lookup factory calibrated values for the config reg */
const gclk_clk_scaler_ll_t gclk_efm32_hfrco_scaler = {
    GCLK_EFM32_PLAIN_SCALER_STATIC_INIT(HFRCO),
    .base.flags.topology_flags = 0, // TODO: lookup what applies here
                              // | GCLK_STOP_FOR_UPDATE
                              // | GCLK_STOP_PARENT_FOR_UPDATE
                              // | GCLK_STOP_CHILDREN_FOR_UPDATE
                              // | GCLK_ONLY_FOR_STARTUP
    .base.fixed_parent         = &gclk_efm32_hfrco_base_gate.base,
    .regref                    = { .conf = GCLK_EFM32_CMU_HFRCOCTRL,
                                   .conf_msb = STATIC_BITMASK_MSB(_CMU_HFRCOCTRL_MASK),
                                   .conf_lsb = STATIC_BITMASK_LSB(_CMU_HFRCOCTRL_MASK),
                                 },
    .base.flags.scaler_type    = GCLK_MUL,
    GCLK_PTR_LUT_STATIC_INIT(hfrcoctrl_ptr_lut),
    /* TODO: below is not necessarily needed as the value config limits this anyway */
    //.out_freq_constraint       = &_auxhfrco_freq_constraint,
};

const gclk_clk_scaler_ll_t gclk_efm32_hfexpclk_scaler= {
    GCLK_EFM32_PLAIN_SCALER_STATIC_INIT(HFEXPCLK),
    /* NOTE: can be updaten on-the-fly, change takes effect immediately */
    .base.flags.topology_flags = 0, // TODO: lookup what applies here
                              // | GCLK_STOP_FOR_UPDATE
                              // | GCLK_STOP_PARENT_FOR_UPDATE
                              // | GCLK_STOP_CHILDREN_FOR_UPDATE
                              // | GCLK_ONLY_FOR_STARTUP
    .base.fixed_parent         = &gclk_efm32_hfclk_scaler.base,
    .regref                    = { .en  = GCLK_NULL_REG,
                                   .rdy = GCLK_NULL_REG,
                                   .conf = GCLK_EFM32_CMU_HFEXPPRESC,
                                   .conf_msb = STATIC_BITMASK_MSB(_CMU_HFEXPPRESC_MASK),
                                   .conf_lsb = STATIC_BITMASK_LSB(_CMU_HFEXPPRESC_MASK),
                                 },
    .base.flags.scaler_type    = GCLK_DIV,
    GCLK_IDX_AS_REGVAL_RANGE8_STATIC_INIT(_hfexpclkpresc_factor_constraint),
};

const gclk_clk_scaler_ll_t gclk_efm32_hfperpresc_scaler = {
    GCLK_EFM32_PLAIN_SCALER_STATIC_INIT(HFPERCLK),
    /* NOTE: can be updaten on-the-fly, change takes effect immediately */
    .base.flags.topology_flags = 0, // TODO: lookup what applies here
                              // | GCLK_STOP_FOR_UPDATE
                              // | GCLK_STOP_PARENT_FOR_UPDATE
                              // | GCLK_STOP_CHILDREN_FOR_UPDATE
                              // | GCLK_ONLY_FOR_STARTUP
    .base.fixed_parent         = &gclk_efm32_hfperclk_gate.base,
    .regref                    = { .en  = GCLK_NULL_REG,
                                   .rdy = GCLK_NULL_REG,
                                   .conf = GCLK_EFM32_CMU_HFPERPRESC,
                                   .conf_msb = STATIC_BITMASK_MSB(_CMU_HFPERPRESC_MASK),
                                   .conf_lsb = STATIC_BITMASK_LSB(_CMU_HFPERPRESC_MASK),
                                 },
    GCLK_IDX_AS_REGVAL_RANGE16_STATIC_INIT(_hfperpresc_factor_range),
    .base.flags.scaler_type    = GCLK_DIV,
};

/* the value that can be written to the register is 0 - 4095
 * representing a divisor of 1 - 4096 */
static const gclk_range16_t _dpll_m_range = {
    .min =  1, .max = 4096,
};

const gclk_clk_scaler_ll_t gclk_efm32_dppl_m_div_scaler = {
    GCLK_EFM32_PLAIN_SCALER_STATIC_INIT(DPLL_M),
    .base.fixed_parent         = &gclk_efm32_dpllref_mux.base,
    .regref                    = { .conf = GCLK_EFM32_CMU_DPLLCTRL1,
                                   .conf_msb = STATIC_BITMASK_MSB(_CMU_DPLLCTRL1_M_MASK),
                                   .conf_lsb = STATIC_BITMASK_LSB(_CMU_DPLLCTRL1_M_MASK),
                                 },
    GCLK_IDX_AS_REGVAL_RANGE16_STATIC_INIT(_dpll_m_range),
    .base.flags.scaler_type    = GCLK_DIV,
};

/* the numeric value that can be written to the register is 0 - 4095
 * but according to the reference manual the register value N must be
 * greater than 32. Whereas the register value represents a numerical
 * value of N + 1. Thus the effective numerical range is 34 - 4096.
 * It is also stated that N is recommended to be larger than 300 unless a specific
 * lock time is needed.
 **/
static const gclk_range16_t _dpll_n_range = {
    .min =  1, .max = 4096,
};

const gclk_clk_scaler_ll_t gclk_efm32_dppl_n_mul_scaler = {
    GCLK_EFM32_PLAIN_SCALER_STATIC_INIT(DPLL_N),
    .base.fixed_parent         = &gclk_efm32_dppl_m_div_scaler.base,
    .regref                    = { .conf = GCLK_EFM32_CMU_DPLLCTRL1,
                                   .conf_msb = STATIC_BITMASK_MSB(_CMU_DPLLCTRL1_N_MASK),
                                   .conf_lsb = STATIC_BITMASK_LSB(_CMU_DPLLCTRL1_N_MASK),
                                 },
    GCLK_IDX_AS_REGVAL_RANGE16_STATIC_INIT(_dpll_n_range),
    .base.flags.scaler_type    = GCLK_MUL,
};

const gclk_clk_scaler_ll_t gclk_efm32_hfcorepresc_scaler = {
    GCLK_EFM32_PLAIN_SCALER_STATIC_INIT(HFCORECLK),
    /* NOTE: can be updaten on-the-fly, change takes effect immediately */
    .base.flags.topology_flags = 0, // TODO: lookup what applies here
                              // | GCLK_STOP_FOR_UPDATE
                              // | GCLK_STOP_PARENT_FOR_UPDATE
                              // | GCLK_STOP_CHILDREN_FOR_UPDATE
                              // | GCLK_ONLY_FOR_STARTUP
    /* TODO: this flag is currently not considered but should have the effect that
     *       a register content of 2 represents a logical value of 3 */
    .base.fixed_parent         = &gclk_efm32_hfclk_scaler.base,
    .regref                    = { .en  = GCLK_NULL_REG,
                                   .rdy = GCLK_NULL_REG,
                                   .conf = GCLK_EFM32_CMU_HFCOREPRESC,
                                   .conf_msb = STATIC_BITMASK_MSB(_CMU_HFCOREPRESC_PRESC_MASK),
                                   .conf_lsb = STATIC_BITMASK_LSB(_CMU_HFCOREPRESC_PRESC_MASK),
                                 },
    .base.flags.scaler_type    = GCLK_DIV,
    GCLK_IDX_AS_REGVAL_RANGE16_STATIC_INIT(_hfcorepresc_factor_range),
};

const gclk_t * const core_clock_instance = &gclk_efm32_hfcorepresc_scaler.base;

/* This mux config mapping encodes the register value as index of the selected clock */
static const gclk_t * const _adcnclksel_mux_configs[] = {
    NULL,                             /* 0 = DISABLED */
    &gclk_efm32_auxclk_scaler.base,   /* 1 = AUXHFRCO */
    &gclk_efm32_hfxo_base_gate.base,  /* 2 = HFXO */
    &gclk_efm32_hfsrcclk_mux.base,    /* 3 = HFSRCCLK */
};

const gclk_mux_ll_t gclk_efm32_adc0clksel_mux = {
    GCLK_PLAIN_MUX_STATIC_INIT(ADC0CLKSEL,
                               GCLK_EFM32_ADC0_CTRL,
                               _CMU_ADCCTRL_ADC0CLKSEL_MASK),
    .base.parent_map_op        = gclk_map_parent_list,
    //.base.flags.reg_map_type   = GCLK_CONF_LIST32,
    .base.parent_mapping.plist = &_adcnclksel_mux_configs[0],
    .base.flags.conf_cnt       = ARRAY_SIZE(_adcnclksel_mux_configs),
};

/* This sync/async mode switch for the ADC clock is modeled as a mux between
 * the async and the sync clock */
static const gclk_t * const _adcclkmode_mux_configs[] = {
    &gclk_efm32_auxhfrco_base_gate.base, /* 0 = HFPERCLK (SYNC) default in RIOT */
    /* TODO: the below should actually point to the ADCnCLKINV clock
     * but currently phase properties or clocks are not considered and modeled
     * so we ignore that for now. */
    &gclk_efm32_adc0clksel_mux.base,     /* 1 = ADC0CLKSEL (ASYNC) */
};

/* TODO: there is a special sequence that must be performed in order to switch
 * this mode. Either special pre/post callbacks could be used for that or it
 * could be left to the ADC device driver to ensure this handling. Flags to
 * indicate "manual steps are needed" could be used to tell gclk to not alter
 * such clocks on autonomously */
const gclk_mux_ll_t gclk_efm32_adcclkmode_mux = {
    GCLK_PLAIN_MUX_STATIC_INIT(ADC_CLK,
                               GCLK_EFM32_ADC0_CTRL,
                               _ADC_CTRL_ADCCLKMODE_SHIFT),
    .base.parent_map_op        = gclk_map_parent_list,
    //.base.flags.reg_map_type   = GCLK_CONF_LIST32,
    .base.parent_mapping.plist = &_adcclkmode_mux_configs[0],
    .base.flags.conf_cnt       = ARRAY_SIZE(_adcclkmode_mux_configs),
};

/* The reference selection of DPLL is modeled as mux */
static const gclk_t * const _dpll_refsel_mux_configs[] = {
    &gclk_efm32_hfxo_base_gate.base,   /* 0 = HFXO */
    &gclk_efm32_lfxo_base_gate.base,   /* 1 = LFXO */
    &gclk_efm32_clkin0_base_gate.base, /* 2 = CLKIN0 */
};

const gclk_mux_ll_t gclk_efm32_dpllref_mux = {
    GCLK_PLAIN_MUX_STATIC_INIT(DPLL_REF,
                               GCLK_EFM32_CMU_DPLLCTRL,
                               _CMU_DPLLCTRL_REFSEL_MASK),
    .base.parent_map_op        = gclk_map_parent_list,
    //.base.flags.reg_map_type   = GCLK_CONF_LIST32,
    .base.parent_mapping.plist = &_dpll_refsel_mux_configs[0],
    .base.flags.conf_cnt       = ARRAY_SIZE(_adcclkmode_mux_configs),
};

static const gclk_t * const _dbgclk_mux_configs[] = {
    &gclk_efm32_auxclk_scaler.base, /* 0 = AUXHFRCO */
    &gclk_efm32_hfclk_scaler.base,  /* 1 = HFCLK */
};

const gclk_mux_ll_t gclk_efm32_dbgtraceclk_mux = {
    GCLK_PLAIN_MUX_STATIC_INIT(DBGCLK,
                               GCLK_EFM32_CMU_DBGCLKSEL,
                               _CMU_DBGCLKSEL_MASK),
    .base.parent_map_op        = gclk_map_parent_list,
    //.base.flags.reg_map_type   = GCLK_CONF_LIST32,
    .base.parent_mapping.plist = &_dbgclk_mux_configs[0],
    .base.flags.conf_cnt       = ARRAY_SIZE(_dbgclk_mux_configs),
};

/* TODO: update the clock reference list once the are implemented */
/* values for the HFCLKSEL register */
static const gclk_parent_config_lut_t _hfsrcclk_mux_configs_lut[] = {
  { .parent = &gclk_efm32_hfrco_scaler.base,        .config_reg_val = 1 }, /* 1 = HFRCO */
  { .parent = &gclk_efm32_hfxo_base_gate.base,      .config_reg_val = 2 }, /* 2 = HFXO */
  { .parent = &gclk_efm32_lfrco_base_gate.base,     .config_reg_val = 3 }, /* 3 = LFRCO */
  { .parent = &gclk_efm32_lfxo_base_gate.base,      .config_reg_val = 4 }, /* 4 = LFXO */
  { .parent = &gclk_efm32_hfrcodiv2_scaler.base,    .config_reg_val = 5 }, /* 5 = HFRCODIV2 */
  { .parent = &gclk_efm32_clkin0_base_gate.base,    .config_reg_val = 7 }, /* 7 = CLKIN0 */
};

const gclk_efm32_mux_t gclk_efm32_hfsrcclk_mux = {
    .base.separated_ops      = _efm32_plain_staselreg_mux_ops,
    .base.flags.muxable      = 1,
GCLK_PARENT_LUT_STATIC_INIT(_hfsrcclk_mux_configs_lut),
    .base.name               = "HFSRCCLK",
GCLK_EFM32_MUX_STATIC_INIT_SEL_STATUS_REG(GCLK_EFM32_CMU_HFCLKSEL,
                                          GCLK_EFM32_CMU_HFCLKSTATUS,
                                          _CMU_HFCLKSEL_HF_MASK,
                                          _CMU_HFCLKSTATUS_SELECTED_MASK),
};

/* For now only the qualified clocks are made visible as there are no separate instances for unqualified
 * clocks */
static const gclk_parent_config_lut_t _clkout_0_1_mux_configs_lut[] = {
  { .parent = NULL,                             .config_reg_val = 0 },  /* 0  = DISABLED */
  { .parent = &gclk_efm32_hfexpclk_scaler.base, .config_reg_val = 7 },  /* 7  = HFEXPCLK */
  { .parent = &gclk_efm32_ulfrco_src,           .config_reg_val = 9 },  /* 9  = ULFRCOQ */
  { .parent = &gclk_efm32_lfrco_base_gate.base, .config_reg_val = 10 }, /* 10 = LFRCOQ */
  { .parent = &gclk_efm32_lfxo_base_gate.base,  .config_reg_val = 11 }, /* 11 = LFXOQ */
  { .parent = &gclk_efm32_hfrco_scaler.base,    .config_reg_val = 12 }, /* 12 = HFRCOQ */
  { .parent = &gclk_efm32_hfxo_base_gate.base,  .config_reg_val = 14 }, /* 14 = HFXOQ */
  { .parent = &gclk_efm32_hfsrcclk_mux.base,    .config_reg_val = 15 }, /* 15 = HFSRCCLK */
};

#define GCLK_GENERIC_MUX_STATIC_INIT_CONF_REG(CONF_REG,CONF_MASK) \
.regref = { .conf = CONF_REG,\
            .conf_msb = STATIC_BITMASK_MSB(CONF_MASK),\
            .conf_lsb = STATIC_BITMASK_LSB(CONF_MASK)}

const gclk_mux_ll_t gclk_efm32_clkout0_mux = {
    GCLK_PLAIN_MUX_STATIC_INIT(CLKOUT0,
                               GCLK_EFM32_CMU_CTRL,
                               _CMU_CTRL_CLKOUTSEL0_MASK),
    GCLK_PARENT_LUT_STATIC_INIT(_clkout_0_1_mux_configs_lut),
};

const gclk_mux_ll_t gclk_efm32_clkout1_mux = {
GCLK_PLAIN_MUX_STATIC_INIT(CLKOUT1,
                           GCLK_EFM32_CMU_CTRL,
                           _CMU_CTRL_CLKOUTSEL1_MASK),
GCLK_PARENT_LUT_STATIC_INIT(_clkout_0_1_mux_configs_lut),
};

/* This mux LUT is used by LFA and LFE */
static const gclk_parent_config_lut_t _lfaeclksel_mux_configs_lut[] = {
  { .parent = NULL,                             .config_reg_val = 0 }, /* DISABLED */
  { .parent = &gclk_efm32_lfrco_base_gate.base, .config_reg_val = 1 }, /* LFRCO */
  { .parent = &gclk_efm32_lfxo_base_gate.base,  .config_reg_val = 2 }, /* LFXO */
  { .parent = &gclk_efm32_ulfrco_src,           .config_reg_val = 4 }, /* ULFRCO */
};

const gclk_mux_ll_t gclk_efm32_lfaclk_mux = {
GCLK_PLAIN_MUX_STATIC_INIT(LFACLK,
                           GCLK_EFM32_CMU_LFACLKSEL,
                           _CMU_LFACLKSEL_LFA_MASK),
GCLK_PARENT_LUT_STATIC_INIT(_lfaeclksel_mux_configs_lut),
};

const gclk_mux_ll_t gclk_efm32_lfeclk_mux = {
GCLK_PLAIN_MUX_STATIC_INIT(LFECLK,
                           GCLK_EFM32_CMU_LFECLKSEL,
                           _CMU_LFECLKSEL_LFE_MASK),
GCLK_PARENT_LUT_STATIC_INIT(_lfaeclksel_mux_configs_lut),
};

/* @TODO: list encoding should be more efficient here */
static const gclk_parent_config_lut_t _lfbclksel_mux_configs_lut[] = {
  { .parent = NULL,                             .config_reg_val = 0 }, /* DISABLED */
  { .parent = &gclk_efm32_lfrco_base_gate.base, .config_reg_val = 1 }, /* LFRCO */
  { .parent = &gclk_efm32_lfxo_base_gate.base,  .config_reg_val = 2 }, /* LFXO */
  { .parent = &gclk_efm32_hfclkle_scaler.base,  .config_reg_val = 3 }, /* HFCLKLE */
  { .parent = &gclk_efm32_ulfrco_src,           .config_reg_val = 4 }, /* ULFRCO */
};

const gclk_mux_ll_t gclk_efm32_lfbclk_mux = {
GCLK_PLAIN_MUX_STATIC_INIT(LFBCLK,
                           GCLK_EFM32_CMU_LFBCLKSEL,
                           _CMU_LFBCLKSEL_LFB_MASK),
GCLK_PARENT_LUT_STATIC_INIT(_lfbclksel_mux_configs_lut),
};

/* @TODO: list encoding should be more efficient here */
static const gclk_parent_config_lut_t _wdogclksel_mux_configs_lut[] = {
  { .parent = &gclk_efm32_ulfrco_src,                   .config_reg_val = 0 }, /* ULFRCO */
  { .parent = &gclk_efm32_lfrco_base_gate.base,         .config_reg_val = 1 }, /* LFRCO */
  { .parent = &gclk_efm32_lfxo_base_gate.base,          .config_reg_val = 2 }, /* LFXO */
  { .parent = &gclk_efm32_hfcoreclkcoretex_scaler.base, .config_reg_val = 3 }, /* HFCORECLK CORETEX */
};

const gclk_mux_ll_t gclk_efm32_wdog_mux = {
GCLK_PLAIN_MUX_STATIC_INIT(WDOG,
                           GCLK_EFM32_WDOG_CTRL,
                           _WDOG_CTRL_CLKSEL_MASK),
GCLK_PARENT_LUT_STATIC_INIT(_wdogclksel_mux_configs_lut),
};

/* The EFM32 has different types of gates: some are controlled with separate enable/disable bits,
 * and others - like this one - are controlled by the same single bit */
const gclk_generic_gate_t gclk_efm32_hfperclk_gate = {
    .base.separated_ops  = gclk_plain_gate_ops,\
    .base.flags.gateable = 1,\
    .base.name           = "HFPERCLKEN",
    .base.fixed_parent   = &gclk_efm32_hfclk_scaler.base,
    .regref              =  { .en      = GCLK_EFM32_CMU_CTRL,
                              .en_bit  = _CMU_CTRL_HFPERCLKEN_SHIFT },
};

/* The gate ops driver uses the simple generic gate driver that makes use of a
 * single enable/disable bit in one register. The drivers otional support
 * for the ready bit is just ignored in this case as HFBUSCLKEN gates have no ready
 * flag */
#define GCLK_EFM32_HFBUSCLKEN_GATE_STATIC_INIT(NAME)\
.base.separated_ops  = gclk_plain_gate_ops,\
.base.flags.gateable = 1,\
.base.name           = #NAME,\
.base.fixed_parent   = &gclk_efm32_hfclk_scaler.base,\
.regref              =  { .en      = GCLK_EFM32_CMU_HFBUSCLKEN0,\
                          .en_bit  = _CMU_HFBUSCLKEN0_##NAME##_SHIFT, }

const gclk_generic_gate_t gclk_efm32_gpcrc_gate = {
GCLK_EFM32_HFBUSCLKEN_GATE_STATIC_INIT(GPCRC),
};

const gclk_generic_gate_t gclk_efm32_ldma_gate = {
GCLK_EFM32_HFBUSCLKEN_GATE_STATIC_INIT(LDMA),
};

const gclk_generic_gate_t gclk_efm32_prs_gate = {
GCLK_EFM32_HFBUSCLKEN_GATE_STATIC_INIT(PRS),
};

const gclk_generic_gate_t gclk_efm32_gpio_gate = {
GCLK_EFM32_HFBUSCLKEN_GATE_STATIC_INIT(GPIO),
};

const gclk_generic_gate_t gclk_efm32_le_gate = {
GCLK_EFM32_HFBUSCLKEN_GATE_STATIC_INIT(LE),
};

const gclk_generic_gate_t gclk_efm32_crypto0_gate = {
GCLK_EFM32_HFBUSCLKEN_GATE_STATIC_INIT(CRYPTO0),
};

const gclk_generic_gate_t gclk_efm32_crypto1_gate = {
GCLK_EFM32_HFBUSCLKEN_GATE_STATIC_INIT(CRYPTO1),
};

const gclk_t gclk_efm32_ulfrco_src = {
    /* this is as pain a source as it may get. It only provides a fixed frequency
     * and can not even be switched of. THus, there are not any ops set */
    .separated_ops    = NULL,
    .name             = "ULFRCO",
    .flags.is_source  = 1,
    .fixed_input_freq = 1000,
};

/* TODO: on EFM32PG1B the voltage range affects:
 *       - flash write/erase only available at scale level 2
 *       - TRNG only available a t scale level 2
 *       - HXFO only supported at scale level 2
 * NOTE: scaling voltage down on this platform should use the following pattern:
 *       - decrease clock freq
 *       - update flash wait states
 *       - issue voltage scaling command
 *       - wait for scaling to finish
 *       scaling voltage up:
 *       - issue voltage scaling cmd
 *       - wait for hardware to finish scaling
 *       - update wait states
 *       - increase clock frequency */
/* @todo: where to put the wait state information? */
const dvfs_conf_t dvfs_confs[] = {
    { .clk = &gclk_efm32_hfcorepresc_scaler.base, .max_freq = 96000000, .vcore_mv = 1800 /* range 1 */},
};

const unsigned int DVFS_CONFS_CNT = ARRAY_SIZE(dvfs_confs);

static void _pre_cahnge_hook_uart(void * ctx)
{
    (void)ctx;
}

void _post_change_hook_uart(void *ctx) {
    (void)ctx;
    stdio_init();
}

void _pre_change_hook_xtimer(void *ctx) {
    (void)ctx;
}

void _post_change_hook_xtimer(void *ctx) {
    (void)ctx;
    xtimer_init();
}

static const gclk_t *_outputtable_clks[]  = { &gclk_efm32_clkout0_mux.base,    &gclk_efm32_clkout1_mux.base };
static const uint8_t _outpin_loc_shifts[] = { _CMU_ROUTELOC0_CLKOUT0LOC_SHIFT, _CMU_ROUTELOC0_CLKOUT1LOC_SHIFT };
static const uint32_t _outpin_loc_masks[] = { _CMU_ROUTELOC0_CLKOUT0LOC_MASK,  _CMU_ROUTELOC0_CLKOUT1LOC_MASK };
static const uint32_t _outpin_pen_masks[] = { CMU_ROUTEPEN_CLKOUT0PEN,         CMU_ROUTEPEN_CLKOUT1PEN };

/* the LOC value is implicitly encoded by the array position */
static const gpio_t _clk_outpins[2][8] = {
// EFM32pg12b pin mappings (taken from device family datasheet
///* CLK0 pin mappings */
//LOC0 PA1
//LOC1 PB15
//LOC2 PC6
//LOC3 PC11 -> EXP HEADER
//LOC4 PD9  -> EXP HEADER
//LOC5 PD14
//LOC6 PF2
//LOC7 PF7
  { GPIO_PIN(0,1), GPIO_PIN(1,15), GPIO_PIN(2,6), GPIO_PIN(2,11), GPIO_PIN(3,9), GPIO_PIN(3,14), GPIO_PIN(5,2), GPIO_PIN(5,7)},
///* CLK1 pin mappings */
//LOC0 PA0
//LOC1 PB14
//LOC2 PC7
//LOC3 PC10 -> EXP HEADER
//LOC4 PD10 -> EXP HEADER
//LOC5 PD15
//LOC6 PF3
//LOC7 PF6
  { GPIO_PIN(0,0), GPIO_PIN(1,14), GPIO_PIN(2,7), GPIO_PIN(2,10), GPIO_PIN(3,10), GPIO_PIN(3,15), GPIO_PIN(5,3), GPIO_PIN(5,6)},
};

int gclk_enable_pin_output(const gclk_t *clk, const gpio_t pin) {
  for (unsigned i = 0; i < ARRAY_SIZE(_outputtable_clks); i++) {
      if (_outputtable_clks[i] == clk) {
          for (unsigned p = 0; p < ARRAY_SIZE(_clk_outpins[i]); p++) {
              if (_clk_outpins[i][p] == pin) {
                  gpio_init(_clk_outpins[i][p], GPIO_OUT);
                  /* enable clock output */
                  CMU->ROUTEPEN |= _outpin_pen_masks[i];
                  /* route clock singal to pin */
                  CMU->ROUTELOC0 = (CMU->ROUTELOC0 &~_outpin_loc_masks[i]) | (p << _outpin_loc_shifts[i]);
                  return ENABLE_PIN_OUTPUT_OK;
              }
          }
          return ENABLE_PIN_OUTPUT_INVALID_PIN;
      }
  }
  return ENABLE_PIN_OUTPUT_INVALID_CLOCK;
}
/* TODO: replace this with a linked list implementation that holds one list of notification methods
 *       per clock. Methods should then get registered dynamically depending on the clock they rely
 *       on */
const reinit_trigger_conf_t reinit_configs[] = {
    { .affected_clock = &gclk_efm32_usart0_gate.base, .post_change_hook_fptr = _post_change_hook_uart,
      .pre_change_hook_fptr = _pre_cahnge_hook_uart,
      .name = "STDIO UART" , .pre_change_freq = 0},
/* the timer on slstk3402a is currently either driven by
 * either timer 0 or timer 2 of the options 0:WTIMER0+WTIMER1 1:TIMER0+WTIMER1 2:LETIMER0
 * where WTIMER  is driven by HFPERCLKEN0
 *       TIMER   is driven by HFPERCLKEN0
 *       LETIMER is driven by LFACLKEN0 */
#if IS_ACTIVE(CONFIG_EFM32_XTIMER_USE_LETIMER)
    { .affected_clock = &gclk_efm32_lfaclk_letimer, .post_change_hook_fptr = _post_change_hook_xtimer,
      .pre_change_hook_fptr = _pre_change_hook_xtimer,
      .name = "xtimer (also periph timer)" , .pre_change_freq = 0},
#else
    { .affected_clock = &gclk_efm32_hfperclk_wtimer, .post_change_hook_fptr = _post_change_hook_xtimer,
      .pre_change_hook_fptr = _pre_change_hook_xtimer,
      .name = "xtimer (also periph timer)" , .pre_change_freq = 0},
#endif
};

/* TODO: once migrated to list based notifications this can be dropped */
const unsigned int GCLK_REINIT_CONFIGS_CNT = ARRAY_SIZE(reinit_configs);

gclk_t const *gclock_handle_for_core_freq = &gclk_efm32_hfcorepresc_scaler.base;

/* making this static leads to copies if included from multiple places, so avoid that */
const gclk_t *gclks[GCLK_NUM_OF_CLOCKS] = {
    &gclk_efm32_hfrcodiv2_scaler.base,
    &gclk_efm32_hfcoreclkcoretex_scaler.base,
    &gclk_efm32_hfclk_scaler.base,
    &gclk_efm32_hfclkle_scaler.base,
    &gclk_efm32_auxclk_scaler.base,
    &gclk_efm32_hfrco_scaler.base,
    &gclk_efm32_hfperpresc_scaler.base,
    &gclk_efm32_hfcorepresc_scaler.base,
    &gclk_efm32_adc0clksel_mux.base,
    &gclk_efm32_adcclkmode_mux.base,
    &gclk_efm32_dbgtraceclk_mux.base,
    &gclk_efm32_hfsrcclk_mux.base,
    &gclk_efm32_clkout0_mux.base,
    &gclk_efm32_clkout1_mux.base,
    &gclk_efm32_lfaclk_mux.base,
    &gclk_efm32_lfbclk_mux.base,
    &gclk_efm32_lfeclk_mux.base,
    &gclk_efm32_wdog_mux.base,
    &gclk_efm32_auxhfrco_base_gate.base,
    &gclk_efm32_hfrco_base_gate.base,
    &gclk_efm32_hfxo_base_gate.base,
    &gclk_efm32_lfrco_base_gate.base,
    &gclk_efm32_clkin0_base_gate.base,
    &gclk_efm32_lfxo_base_gate.base,
    &gclk_efm32_trng0_gate.base,
    &gclk_efm32_csen_gate.base,
    &gclk_efm32_vdac0_gate.base,
    &gclk_efm32_idac0_gate.base,
    &gclk_efm32_adc0_gate.base,
    &gclk_efm32_cryotimer_gate.base,
    &gclk_efm32_acmp1_gate.base,
    &gclk_efm32_acmp0_gate.base,
    &gclk_efm32_i2c1_gate.base,
    &gclk_efm32_i2c0_gate.base,
    &gclk_efm32_usart3_gate.base,
    &gclk_efm32_usart2_gate.base,
    &gclk_efm32_usart1_gate.base,
    &gclk_efm32_usart0_gate.base,
    &gclk_efm32_wtimer1_gate.base,
    &gclk_efm32_wtimer0_gate.base,
    &gclk_efm32_timer1_gate.base,
    &gclk_efm32_timer0_gate.base,
    &gclk_efm32_hfperclk_gate.base,
    &gclk_efm32_gpcrc_gate.base,
    &gclk_efm32_ldma_gate.base,
    &gclk_efm32_prs_gate.base,
    &gclk_efm32_gpio_gate.base,
    &gclk_efm32_le_gate.base,
    &gclk_efm32_crypto0_gate.base,
    &gclk_efm32_crypto1_gate.base,
    &gclk_efm32_ulfrco_src,
    &gclk_efm32_dpllref_mux.base,
    &gclk_efm32_dppl_m_div_scaler.base,
    &gclk_efm32_dppl_n_mul_scaler.base,
    &gclk_efm32_dpll_gate.base,
};
