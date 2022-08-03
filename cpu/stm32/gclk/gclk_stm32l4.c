/*
 * Copyright (C) 2020 HAW Hamburg
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     cpu_stm32_common
 * @{
 *
 * @file
 * @brief       STM32L4-specific declarations for the gclk module
 *
 * @todo        HSI48 (RC48) that is only present on STM32L49x/L4Ax is missing
 *
 * @author      Michel Rottleuthner <michel.rottleuthner@haw-hamburg.de>
 * @}
 */
#include <stdint.h>
#include <stddef.h>

#include "cpu.h" /* pulls in the RCC register definitions etc. from the relevant vendor header */
#include "board.h" /* needed for crystal clock defines */

#include <stdio.h>
#include "gclk.h"
#include "gclk/generic_gate.h"
#include "gclk/generic_mux.h"
#include "gclk/generic_scaler.h"

#define STM32_L476RG_MCO_PIN (GPIO_PIN(0,8))
#define STM32_L476RG_MCO_AF  (GPIO_AF0)

/* the below section needs to be moved out of here once the best method is selected (or it is found that tiny regref
   is not enough for some platforms )*/
#ifdef GCLK_USE_TINY_REG_REF
#if defined(CPU_FAM_STM32L4)
/* The number of regsiters involved in clock configuration is usually pretty small.
   We can exploit that to save some memory by only holding the configuration registers in one place and storing only a
   tiny index to it in each instance
   @todo: evaluate if we can link conditional availability between these values and instances to use the same knob for
          enabling/disabling availability/required state */
typedef enum {
    GCLK_NULL_REG = 0,
    GCLK_STM32_RCC_CR,
    GCLK_STM32_RCC_CSR,
    GCLK_STM32_RCC_BDCR,
    GCLK_STM32_RCC_CFGR,
    GCLK_STM32_RCC_CCIPR,
    GCLK_STM32_RCC_PLLCFGR,
    GCLK_STM32_RCC_PLLSAI1CFGR,
    GCLK_STM32_RCC_PLLSAI2CFGR,
    GCLK_STM32_RCC_APB1ENR1,
    GCLK_STM32_RCC_APB1ENR2,
    GCLK_STM32_RCC_APB2ENR,
    GCLK_STM32_RCC_AHB2ENR,
    /* @todo: add unused entry at the end to derive count? */
} gclk_conf_reg_id_t;
#elif defined(CPU_FAM_STM32L0)
typedef enum {
    GCLK_NULL_REG = 0,
    GCLK_STM32_RCC_CR,
    GCLK_STM32_RCC_CSR,
    GCLK_STM32_RCC_CFGR,
    GCLK_STM32_RCC_CCIPR,
    GCLK_STM32_RCC_CRRCR,
    GCLK_STM32_RCC_ICSCR,
} gclk_conf_reg_id_t;
#else
#error "must specify gclk conf reg ids for this platform!"
#endif /* defined(CPU_FAM_STM32L4) */
#endif /* GCLK_USE_TINY_REG_REF */

/* compatibility taken from stmclock for now
 * since stmclock solves a similar issue this should be a good baseline for testing and comparison.
 * For now this is only tested on STM32L476 */
#if defined(CPU_FAM_STM32L4)
#else
#error gclk not supportet on this STM32 chip!
#endif /* defined(CPU_FAM_STM32L4) */

#ifdef GCLK_USE_TINY_REG_REF
/* @todo: declare this externally (board specific configuration header + single include in here + getters ?)*/
volatile uint32_t* const conf_regs[] = {
    [GCLK_NULL_REG]              = NULL,
    [GCLK_STM32_RCC_CR]          = &RCC->CR,
    [GCLK_STM32_RCC_CSR]         = &RCC->CSR,
    [GCLK_STM32_RCC_BDCR]        = &RCC->BDCR,
    [GCLK_STM32_RCC_CFGR]        = &RCC->CFGR,
    [GCLK_STM32_RCC_CCIPR]       = &RCC->CCIPR,
    [GCLK_STM32_RCC_PLLCFGR]     = &RCC->PLLCFGR,
    [GCLK_STM32_RCC_PLLSAI1CFGR] = &RCC->PLLSAI1CFGR,
    [GCLK_STM32_RCC_PLLSAI2CFGR] = &RCC->PLLSAI2CFGR,
    [GCLK_STM32_RCC_APB1ENR1]    = &RCC->APB1ENR1,
    [GCLK_STM32_RCC_APB1ENR2]    = &RCC->APB1ENR2,
    [GCLK_STM32_RCC_APB2ENR]     = &RCC->APB2ENR,
    [GCLK_STM32_RCC_AHB2ENR]     = &RCC->AHB2ENR,
};
#endif

#if defined(CPU_FAM_STM32L4) || defined(CPU_FAM_STM32F7) || \
    defined(CPU_FAM_STM32WB)
#define REG_PWR_CR          CR1
#define BIT_CR_DBP          PWR_CR1_DBP
#else
#define REG_PWR_CR          CR
#define BIT_CR_DBP          PWR_CR_DBP
#endif

/* compatibility taken from stmclock for now
 * since stmclock solves a similar issue this should be a good baseline for testing and comparison.
 * For now this is only tested on STM32L476 */
 #if defined(CPU_FAM_STM32L4)
 /* implemented agains reference manual RM0367 i.e, STM32L0x3 */
 #elif defined(CPU_LINE_STM32L073xx) || defined(CPU_LINE_STM32L053xx)
 #pragma message("gclk not fully tested on this STM32 chip!")
 #else
 #error gclk not supportet on this STM32 chip!
 #endif /* defined(CPU_FAM_STM32L4) */

/* forward declare clock instances to reference them in other clocks */
const gclk_generic_gate_t gclk_stm32_msi_gate;
const gclk_generic_gate_t gclk_stm32_tim5_gate;
const gclk_t gclk_stm32_sai1_ext;
const gclk_t gclk_stm32_sai2_ext;
const gclk_mux_ll_t gclk_stm32_sysclk_mux;
const gclk_mux_ll_t gclk_stm32_msi_mux;
const gclk_mux_ll_t gclk_stm32_usb_rng_sdmmc_mux;
const gclk_mux_ll_t gclk_stm32_lsco_mux;
const gclk_mux_ll_t gclk_stm32_rtc_lcd_mux;
const gclk_mux_ll_t gclk_stm32_pll_pre_div_mux;
const gclk_mux_ll_t gclk_stm32_sai1_mux;
const gclk_mux_ll_t gclk_stm32_sai2_mux;
const gclk_mux_ll_t gclk_stm32_mco_mux;
const gclk_mux_ll_t gclk_stm32_adc_mux;
const gclk_mux_ll_t gclk_stm32_i2c1_mux;
const gclk_mux_ll_t gclk_stm32_i2c2_mux;
const gclk_mux_ll_t gclk_stm32_i2c3_mux;
const gclk_mux_ll_t gclk_stm32_dfsdm1_mux;
const gclk_mux_ll_t gclk_stm32_lpuart1_mux;
const gclk_mux_ll_t gclk_stm32_uart5_mux;
const gclk_mux_ll_t gclk_stm32_uart4_mux;
const gclk_mux_ll_t gclk_stm32_usart3_mux;
const gclk_mux_ll_t gclk_stm32_usart2_mux;
const gclk_mux_ll_t gclk_stm32_usart1_mux;
const gclk_mux_ll_t gclk_stm32_swpmi1_mux;
const gclk_mux_ll_t gclk_stm32_lptim1_mux;
const gclk_mux_ll_t gclk_stm32_lptim2_mux;
const gclk_clk_scaler_ll_t gclk_stm32_ahb_scaler;
const gclk_clk_scaler_ll_t gclk_stm32_apb1_scaler;
const gclk_clk_scaler_ll_t gclk_stm32_apb2_scaler;
const gclk_clk_scaler_ll_t gclk_stm32_pll_m_scaler;
const gclk_clk_scaler_ll_t gclk_stm32_pll_p_scaler;
const gclk_clk_scaler_ll_t gclk_stm32_pllsai1_p_scaler;
const gclk_clk_scaler_ll_t gclk_stm32_pllsai2_p_scaler;
const gclk_clk_scaler_ll_t gclk_stm32_pll_p_scaler;
const gclk_clk_scaler_ll_t gclk_stm32_pllsai1_p_scaler;
const gclk_clk_scaler_ll_t gclk_stm32_pllsai2_p_scaler;
const gclk_clk_scaler_ll_t gclk_stm32_pll_q_scaler;
const gclk_clk_scaler_ll_t gclk_stm32_pll_r_scaler;
const gclk_clk_scaler_ll_t gclk_stm32_pllsai1_q_scaler;
const gclk_clk_scaler_ll_t gclk_stm32_pllsai1_r_scaler;
const gclk_clk_scaler_ll_t gclk_stm32_pllsai2_r_scaler;
const gclk_clk_scaler_ll_t gclk_stm32_pll_vco_scaler;
const gclk_clk_scaler_ll_t gclk_stm32_pllsai1_vco_scaler;
const gclk_clk_scaler_ll_t gclk_stm32_pllsai2_vco_scaler;
const gclk_clk_scaler_ll_t gclk_apb1_tim_mul_scaler;
const gclk_clk_scaler_ll_t gclk_apb2_tim_mul_scaler;
const gclk_clk_scaler_ll_t gclk_stm32_hse_div32_scaler;
const gclk_clk_scaler_ll_t gclk_stm32_ahb_div8_scaler;
const gclk_clk_scaler_ll_t gclk_stm32_msirange_scaler;
const gclk_clk_scaler_ll_t gclk_stm32_msisrange_scaler;
const gclk_clk_scaler_ll_t gclk_stm32_mco_div_scaler;

//static volatile uint32_t* _gate_enable_reg(gclk_generic_gate_t *gate) {
//#if defined(GCLK_GATE_USE_TINY_REG_REF)
//    return gclk_regref2enable_reg(gate->regref);
//#elif defined(GCLK_GATE_USE_SEPARATE_CONF_REG_VALUES)
//    return gate->ready_reg;
//#endif
//}

///* @todo: this is basically the same implementation as for the scaler -> deduplicate! */
//static bool gclk_stm32_common_xx_gate_enable(const gclk_t *clk, gclk_enable_option_t opt)
//{
//    gclk_generic_gate_t *gate = to_gclk_generic_gate_t(clk);
///* we misuse that as a condition to check if BDCR register is present on this MCU */
//#ifdef RCC_BDCR_LSEON_Pos
//    /* @todo: only on write? */
//    if ((opt != GCLK_READ) && (_gate_enable_reg(gate) == &RCC->BDCR)) {
//        PWR->REG_PWR_CR |= BIT_CR_DBP; /* < disable write protection first */
//    }
//#endif
//
//    bool res = gclk_generic_gate_enable(clk, opt);
//
//#ifdef RCC_BDCR_LSEON_Pos
//    if ((opt != GCLK_READ) && (_gate_enable_reg(gate) == &RCC->BDCR)) {
//        PWR->REG_PWR_CR &= ~(BIT_CR_DBP); /* < enable write protection again */
//    }
//#endif
//
//    return res;
//}

/* wraps the basic gate function with logic to unlock/lock backup domain write protection */
static void _bdcr_gate_enable(const gclk_t *clk, bool on) {
    PWR->REG_PWR_CR |= BIT_CR_DBP; /* < disable write protection first */

    gclk_generic_gate_ed_rdy_reg_enable(clk, on);

    PWR->REG_PWR_CR &= ~(BIT_CR_DBP); /* < enable write protection again */
}

/* A driver that implements just the gate_ops interface.
 * For the is_enabled i.e. (read-only) operation it reuses the generic gate
 * implementation. The enable function that writes to hardware wraps the
 * basic gate driver with code that enables/disables the backup domain write
 * protection.
 **/
static const gclk_op_t _bdcr_gate_ops[] = {
  { .gate_ops = { .is_enabled = gclk_generic_gate_ed_rdy_reg_is_enabled,
                  .enable     = _bdcr_gate_enable,}},
};

/* wraps the basic mux function with logic to unlock/lock backup domain write protection */
static void _bdcr_mux_set_parent(const gclk_t *clk, unsigned int index)
{
    /* TODO: to not interfere with other thigs that access the backup domain
     *       this should be guarded to only disable/enable if it acutally is enabled
     *       to avoid enabling a previously disabled baclup domain */
    PWR->REG_PWR_CR |= BIT_CR_DBP; /* < disable write protection first */

    gclk_plain_mux_set_parent(clk, index);

    PWR->REG_PWR_CR &= ~(BIT_CR_DBP); /* < enable write protection again */
}

/* A driver that implements the mux_ops and gate_ops interface for a gateable mux
 * that requires backup domain lock/unlock.
 * For read-only functions it reuses the functions of the base drivers.
 * For write functions it uses custom implementations that simply wraps the
 * base function with backup domain protection enable/disable.
 **/
static const gclk_op_t _bdcr_basic_muxable_gate_ops[] = {
  { .mux_ops  = { .get_parent = gclk_plain_mux_get_parent,
                  .set_parent = _bdcr_mux_set_parent,}},
  { .gate_ops = { .is_enabled = gclk_generic_gate_ed_rdy_reg_is_enabled,
                  .enable     = _bdcr_gate_enable,}},
};

#define _GATEABLE_BDCR_MUX_STATIC_INIT(NAME,CONF_REG_ID,CONF_REG_MASK,ENABLE_REG_ID,ENABLE_BIT)\
.base.separated_ops  = _bdcr_basic_muxable_gate_ops,\
.base.flags.muxable  = 1,\
.base.flags.gateable = 1,\
.base.name           = #NAME,\
.regref                  = { .conf     = CONF_REG_ID,\
                             .conf_lsb = STATIC_BITMASK_LSB(CONF_REG_MASK),\
                             .conf_msb = STATIC_BITMASK_MSB(CONF_REG_MASK),\
                             .en       = ENABLE_REG_ID,\
                             .en_bit   = ENABLE_BIT,}

//const gclk_ops_t gclk_stm32_common_gate_ops = {
//    .enable = gclk_stm32_common_xx_gate_enable,
//    .set_parent = NULL, /* not applicable to pure gates */
//    .get_parent = NULL, /* a pure gate can not be configured to different parents (otherwise it would be a mux). So it
//                           is never appliccable to have a dynamic parent. Static parents are handled by the higher
//                           user facing API */
//    .set_freq = NULL,   /* not applicable for a gate */
//    .get_freq = gclk_generic_gate_get_freq,   /* returns 0 if off (gated) or the parent freq */
//    .check_freq = gclk_generic_gate_check_freq,
//};

//const gclk_ops_t gclk_stm32_common_src_gate_ops = {
//    .enable = gclk_stm32_common_xx_gate_enable,
//    .set_parent = NULL, /* not applicable to pure gates */
//    .get_parent = NULL, /* a pure gate can not be configured to different parents (otherwise it would be a mux). So it
//                           is never appliccable to have a dynamic parent. Static parents are handled by the higher
//                           user facing API */
//    .set_freq = NULL,   /* not applicable for a gate */
//    .get_freq = gclk_generic_src_gate_get_freq,  /* returns 0 if off (gated) or the fixed source freq */
//    .check_freq = gclk_generic_gate_src_check_freq,
//};

//TODO: move out to generic gate header
/* initializes a basic gate that acts as a fixed frequency source */
#define _BASIC_SOURCE_GATE_STATIC_INIT(NAME,FIXFREQ,EN_REG_ID,RDY_REG_ID,EN_BIT,RDY_BIT)\
.base.separated_ops    = gclk_plain_gate_ops,\
.base.fixed_input_freq = FIXFREQ,\
.base.flags.is_source  = 1,\
.base.flags.gateable   = 1,\
.base.name             = #NAME,\
.regref                = { .en      = EN_REG_ID,\
                           .rdy     = RDY_REG_ID,\
                           .en_bit  = EN_BIT,\
                           .rdy_bit = RDY_BIT, }

/* gates that act as a clock source that always has a fixed frequency that is either enabled or disabled */
const gclk_generic_gate_t gclk_stm32_hsi16_gate = {
_BASIC_SOURCE_GATE_STATIC_INIT(HSI16,
                               16000000,
                               GCLK_STM32_RCC_CR,
                               GCLK_STM32_RCC_CR,
                               RCC_CR_HSION_Pos,
                               RCC_CR_HSIRDY_Pos),
};

/* @todo: HSE has board specific (HW configured) rate */
const gclk_generic_gate_t gclk_stm32_hse_gate = {
#if IS_ACTIVE(CONFIG_BOARD_HAS_HSE)
_BASIC_SOURCE_GATE_STATIC_INIT(HSE,
                               CLOCK_HSE,
                               GCLK_STM32_RCC_CR,
                               GCLK_STM32_RCC_CR,
                               RCC_CR_HSEON_Pos,
                               RCC_CR_HSERDY_Pos),
#else
.base.separated_ops    = NULL,
.base.fixed_input_freq = 0,
.base.flags.is_source  = 1,
.base.flags.gateable   = 0,
.base.name             = "HSE",
#endif
};

const gclk_generic_gate_t gclk_stm32_lsi_gate = {
_BASIC_SOURCE_GATE_STATIC_INIT(LSI,
                               CLOCK_LSI,
                               GCLK_STM32_RCC_CSR,
                               GCLK_STM32_RCC_CSR,
                               RCC_CSR_LSION_Pos,
                               RCC_CSR_LSIRDY_Pos),
};

/* initializes a basic gate that acts as a simple enable disable switch for a fixed-parent clock */
#define _BDCR_SOURCE_GATE_STATIC_INIT(NAME,FIXFREQ,EN_REG_ID,RDY_REG_ID,EN_BIT,RDY_BIT)\
.base.separated_ops    = _bdcr_gate_ops,\
.base.fixed_input_freq = FIXFREQ,\
.base.flags.is_source  = 1,\
.base.flags.gateable   = 1,\
.base.name             = #NAME,\
.regref                = { .en      = EN_REG_ID,\
                           .rdy     = RDY_REG_ID,\
                           .en_bit  = EN_BIT,\
                           .rdy_bit = RDY_BIT, }

const gclk_generic_gate_t gclk_stm32_lse_gate = {
_BDCR_SOURCE_GATE_STATIC_INIT(LSE,
                              IS_ACTIVE(CONFIG_BOARD_HAS_LSE) * 32768,
                              GCLK_STM32_RCC_BDCR,
                              GCLK_STM32_RCC_BDCR,
                              RCC_BDCR_LSEON_Pos,
                              RCC_BDCR_LSERDY_Pos),
};

//TODO: with the new way how the base type handles fixed input (parent or freq)
//      it should now be possible to express this directly within the MSI scaler
//      instance!
/* this is a way to express the internal base clock of MSI separately we use the
 * clock base type directly as we do not need any register access information.
   An alternative would be to add a base clock as internal state to the MSI scaler. */
const gclk_t gclk_stm32_msi_base = {
    /* this is as plain a source as it may get. It only provides a fixed frequency
     * and can not even be switched of. THus, there are not any ops set */
    .separated_ops    = NULL,
    .name             = "MSI_BASE",
    .flags.is_source  = 1,
    /* the exact value of the fixed input freq doesn't really matter. It must only be
     * matched to the options of the scalers connected after this instance (i.e. it's children)
     * to properly represent the actual MSI output */
    .fixed_input_freq = 100000,
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

/* gates that are plugged after some configurable clock source wich might be set to an arbitrary frequency */
const gclk_generic_gate_t gclk_stm32_msi_gate = {
_BASIC_GATE_STATIC_INIT(MSI,
                        &gclk_stm32_msi_mux.base,
                        GCLK_STM32_RCC_CR,
                        GCLK_STM32_RCC_CR,
                        RCC_CR_MSION_Pos,
                        RCC_CR_MSIRDY_Pos),
};

/* ready reg and bit of basic gate are simply left unused for this instance */
const gclk_generic_gate_t gclk_stm32_tim5_gate = {
_BASIC_GATE_STATIC_INIT(TIM5,
                        &gclk_apb1_tim_mul_scaler.base,
                        GCLK_STM32_RCC_APB1ENR1,
                        GCLK_NULL_REG,
                        RCC_APB1ENR1_TIM5EN_Pos,
                        0),
};

/* a "dumb" external clock source does not support any operations, the instance is just
   as a placeholder to reuse the same management functions. There is probably a smarter solution

   @todo: map to dummy as this is an input without any config / gate / whatsoever?.
    it is declared here because considereing the currently provided default nodes an input without control maps closest
    to a gate. Maybe a sepatrate generic type for "dumb" sources should be provided as base type too (?) */
const gclk_t gclk_stm32_sai1_ext = {
    /* this is as plain a source as it may get. It only provides a fixed frequency
     * and can not even be switched of. Thus, there are not any ops set */
    .separated_ops    = NULL,
    .name             = "SAI1_EXT",
    .flags.is_source  = 1,
    /* TODO: This value should be overwriteable via (app) configuration
     *       as an alternative a handler that maps to a custom
     *       frequency lookup funtion might also be valueable if this input
     *       frequency is changeable at runtime */
    .fixed_input_freq = 0,
};

const gclk_t gclk_stm32_sai2_ext = {
    /* this is as plain a source as it may get. It only provides a fixed frequency
     * and can not even be switched of. Thus, there are not any ops set */
    .separated_ops    = NULL,
    .name             = "SAI2_EXT",
    .flags.is_source  = 1,
    /* TODO: This value should be overwriteable via (app) configuration
     *       as an alternative a handler that maps to a custom
     *       frequency lookup funtion might also be valueable if this input
     *       frequency is changeable at runtime */
    .fixed_input_freq = 0,
};

//int gclk_stm32_common_mux_set_parent(const gclk_t *clk, unsigned int index)
//{
///* we misuse that as a condition to check if BDCR register is present on this MCU */
//#ifdef RCC_BDCR_LSEON_Pos
//     gclk_mux_ll_t *stm32clkmux = to_gclk_mux_ll_t(clk);
//    /* if this is a clock that is configured trough the backup domain */
//    if (gclk_regref2conf_reg(stm32clkmux->regref) == &RCC->BDCR) {
//        PWR->REG_PWR_CR |= BIT_CR_DBP; /* < disable write protection first */
//    }
//#endif
//
//    gclk_plain_mux_set_parent(clk, index);
//
//#ifdef RCC_BDCR_LSEON_Pos
//    if (gclk_regref2conf_reg(stm32clkmux->regref) == &RCC->BDCR) {
//        PWR->REG_PWR_CR &= ~(BIT_CR_DBP); /* < enable write protection again */
//    }
//#endif
//
//    (void)index;
//    return 0;
//}

//static const gclk_ops_t gclk_stm32_mux_ops = {
//    .enable     = gclk_generic_mux_enable,        /* not needed as pure mux can not be gated */
//    .set_parent = gclk_stm32_common_mux_set_parent,
//    .get_parent = gclk_generic_mux_get_parent,
//    .set_freq   = gclk_generic_mux_set_freq,    /* not needed for a pure mux */
//    .get_freq   = gclk_generic_mux_get_freq,    /* not needed for a pure mux, can just use parents get_freq */
//    .check_freq = gclk_generic_mux_check_freq,
//};


//extern const gclk_t * const gclk_stm32_msi;

/* @todo the data (or config/relation) part of the implementation should be separated from the pure logic
         because for different MCUs the clock tree can vary a lot, parents can be different, or not there at all.
         Also we don't have access to the static parent in a reliable way. I.e. what stm32_clk_gates[1] exactly is, is
         not guranteed
   @todo there is another part of the register that should mybe be preferred to get the current status (SWS)
         should read/write register be separated? */
static const gclk_t * const _sysclk_clk_mux_configs[] = {
    /* values of SW[1:0] RCC_CFGR[1:0] */
    //{ .parent = gclk_stm32_msi,   .config_reg_val = 0b00 }, /* MSI */
    &gclk_stm32_msi_gate.base,   /* MSI  (maps to reg val 0b00) */
    &gclk_stm32_hsi16_gate.base, /* HSI16  (maps to reg val 0b01) */
    &gclk_stm32_hse_gate.base,   /* HSE  (maps to reg val 0b10) */
    &gclk_stm32_pll_r_scaler.base, /* PLLCLK (PLLR) (maps to reg val 0b11) */
};

/* Initializes all fields needed for a clock that encodes its parent options
 * as a list */
#define GCLK_PARENT_LIST_STATIC_INIT(CONFIG_LIST)\
.base.parent_map_op        = gclk_map_parent_list,\
.base.parent_mapping.plist = &CONFIG_LIST[0],\
.base.flags.conf_cnt       = ARRAY_SIZE(CONFIG_LIST)

const gclk_mux_ll_t gclk_stm32_sysclk_mux = {
GCLK_PLAIN_MUX_STATIC_INIT(SYSCLK,
                           GCLK_STM32_RCC_CFGR,
                           RCC_CFGR_SW_Msk), /* SW[1:0] RCC_CFGR[1:0] */
GCLK_PARENT_LIST_STATIC_INIT(_sysclk_clk_mux_configs),
};

static const gclk_t * const _msi_clk_mux_configs[] = {
    &gclk_stm32_msisrange_scaler.base, /* MSISRANGESEL (startup default) */
    &gclk_stm32_msirange_scaler.base,  /* MSIRANGESEL */
};

/* TODO: this is a special case mux as its setting is only available as startup default.
 *       Therefore reading it works just like any other mux but it is not possible to write it directly.
 *       It would be nice to transport this information to the user/gclk_manager somehow so it is
 *       clear that this setting is read only. */
const gclk_mux_ll_t gclk_stm32_msi_mux = {
GCLK_PLAIN_MUX_STATIC_INIT(MSIMUX,
                           GCLK_STM32_RCC_CR,
                           RCC_CR_MSIRGSEL_Msk), /* MSIRGSEL[0:0] RCC_CR[3:3] */
GCLK_PARENT_LIST_STATIC_INIT(_msi_clk_mux_configs),
};

static const gclk_t * const _clk48_clk_mux_configs[] = {
    /* values of CLK48SEL[1:0] RCC_CCIPR[27:26] */
#if defined(CPU_MODEL_STM32L496ZG) || defined(CPU_MODEL_STM32L496AG)
    &gclk_stm32_hsi48_not_defined,   /* HSI48 (only on STM32L496xx/4A6xx devices) */
#else
    NULL,                            /* NONE i.e. clock to PLLs disabled -> maybe move this to a gate config */
#endif
    &gclk_stm32_pllsai1_q_scaler.base, /* PLLSAI1Q */
    &gclk_stm32_pll_q_scaler.base,     /* PLLQ */
    &gclk_stm32_msi_gate.base,       /* MSI */
};

const gclk_mux_ll_t gclk_stm32_usb_rng_sdmmc_mux = {
GCLK_PLAIN_MUX_STATIC_INIT(CLK48,
                           GCLK_STM32_RCC_CCIPR,
                           RCC_CCIPR_CLK48SEL_Msk), /* CLK48SEL[1:0] RCC_CCIPR[27:26] */
GCLK_PARENT_LIST_STATIC_INIT(_clk48_clk_mux_configs),
};

const gclk_t *_lsco_clk_mux_configs[] = {
    /* values of LSCOSEL[0] RCC_BDCR[25] */
    &gclk_stm32_lsi_gate.base, /* LSI */
    &gclk_stm32_lse_gate.base, /* LSE */
};

/* A driver that implements the mux_ops and gate_ops interface for a gateable mux.
 * It just reuses the basic functions of the generic mux and gate implementations.
 **/
static const gclk_op_t _basic_muxable_gate_ops[] = {
  { .mux_ops  = { .get_parent = gclk_plain_mux_get_parent,
                  .set_parent = gclk_plain_mux_set_parent,}},
  { .gate_ops = { .is_enabled = gclk_generic_gate_ed_rdy_reg_is_enabled,
                  .enable     = gclk_generic_gate_ed_rdy_reg_enable,}},
};

/* This initializes clock instances that are compatible with the generic mux and generic
 * gate interface. I.e., there is a read/write bitfield for control of the parent selection
 * and a single read/write enable bit for clock gating. For now this is not usable for the
 * STM32 clocks that need any special handling before/after access (like with BDCR).
 * @TODO: once the interface gets more mature it could be benefitial to have a standard handling
 *        for such dependencies.
 * */
#define GCLK_STM32_GATEABLE_MUX_STATIC_INIT(NAME,CONF_REG_ID,CONF_REG_MASK,ENABLE_REG_ID,ENABLE_BIT)\
.base.separated_ops  = _basic_muxable_gate_ops,\
.base.flags.muxable  = 1,\
.base.flags.gateable = 1,\
.base.name           = #NAME,\
.regref                  = { .conf     = CONF_REG_ID,\
                             .conf_lsb = STATIC_BITMASK_LSB(CONF_REG_MASK),\
                             .conf_msb = STATIC_BITMASK_MSB(CONF_REG_MASK),\
                             .en       = ENABLE_REG_ID,\
                             .en_bit   = ENABLE_BIT,}

/* TODO: needs custom driver that is muxable and gateable */
const gclk_mux_ll_t gclk_stm32_lsco_mux = {
_GATEABLE_BDCR_MUX_STATIC_INIT(LSCO,
                               GCLK_STM32_RCC_BDCR,
                               RCC_BDCR_LSCOSEL_Msk, /* LSCOSEL[0:0] RCC_BDCR[25:25] */
                               GCLK_STM32_RCC_BDCR,
                               RCC_BDCR_LSCOEN_Pos),
GCLK_PARENT_LIST_STATIC_INIT(_lsco_clk_mux_configs),
};


static const gclk_t * const _rtc_lcd_clk_mux_configs[] = {
    /* values of RTCSEL[1:0] RCC_BDCR[9:8] */
    NULL,                            /* *NONE* i.e. clock to RTC/LCD disabled -> is it better to make this explicit? */
    &gclk_stm32_lse_gate.base,       /* LSE */
    &gclk_stm32_lsi_gate.base,       /* LSI */
    &gclk_stm32_hse_div32_scaler.base, /* HSE /32  TODO: move to separate scaler node and link as parent */
};

const gclk_mux_ll_t gclk_stm32_rtc_lcd_mux = {
_GATEABLE_BDCR_MUX_STATIC_INIT(RTC/LTC,
                               GCLK_STM32_RCC_BDCR,
                               RCC_BDCR_RTCSEL_Msk, /* RTCSEL[1:0] RCC_BDCR[9:8] */
                               GCLK_STM32_RCC_BDCR,
                               RCC_BDCR_RTCEN_Pos),
GCLK_PARENT_LIST_STATIC_INIT(_rtc_lcd_clk_mux_configs),
};

/* this is the clock signal from the mux into the /M predivider, the division itself happens later in a child node */
static const gclk_t * const _pll_pre_div_clk_mux_configs[] = {
    /* values of PLLSRC[1:0] RCC_PLLCFGR[1:0] */
    NULL,                        /* NONE i.e. clock to PLLs disabled -> maybe move this to a gate config */
    &gclk_stm32_msi_gate.base,   /* MSI */
    &gclk_stm32_hsi16_gate.base, /* HSI16 */
    &gclk_stm32_hse_gate.base,   /* HSE */
};

const gclk_mux_ll_t gclk_stm32_pll_pre_div_mux = {
GCLK_PLAIN_MUX_STATIC_INIT(PLL_PREDIV_MUX,
                           GCLK_STM32_RCC_PLLCFGR,
                           RCC_PLLCFGR_PLLSRC_Msk), /* PLLSRC[1:0] RCC_PLLCFGR[1:0] */
GCLK_PARENT_LIST_STATIC_INIT(_pll_pre_div_clk_mux_configs),
    .base.flags.topology_flags = GCLK_STOP_CHILDREN_FOR_UPDATE,
};

static const gclk_t * const _sai1_clk_mux_configs[] = {
    /* values of SAI1SEL[1:0] RCC_CCIPR[23:22] */
    &gclk_stm32_pllsai1_p_scaler.base, /* PLLSAI1P (PLLSAI1CLK)  */
    &gclk_stm32_pllsai2_p_scaler.base, /* PLLSAI2P (PLLSAI2CLK)  */
    &gclk_stm32_pll_p_scaler.base,     /* PLLP     (PLLSAI3CLK)  */
    &gclk_stm32_sai1_ext,              /* External (SAI1_EXTCLK) */
};

/* TODO: needs custom driver that is muxable and gateable */
const gclk_mux_ll_t gclk_stm32_sai1_mux = {
GCLK_STM32_GATEABLE_MUX_STATIC_INIT(SAI1,
                                    GCLK_STM32_RCC_CCIPR,
                                    RCC_CCIPR_SAI1SEL_Msk, /* SAI1SEL[1:0] RCC_CCIPR[23:22] */
                                    GCLK_STM32_RCC_APB2ENR,
                                    RCC_APB2ENR_SAI1EN_Pos),
GCLK_PARENT_LIST_STATIC_INIT(_sai1_clk_mux_configs),
};

static const gclk_t * const _sai2_clk_mux_configs[] = {
    /* values of SAI2SEL[1:0] RCC_CCIPR[25:24] */
    &gclk_stm32_pllsai1_p_scaler.base, /* PLLSAI1P (PLLSAI1CLK)  */
    &gclk_stm32_pllsai2_p_scaler.base, /* PLLSAI2P (PLLSAI2CLK)  */
    &gclk_stm32_pll_p_scaler.base,     /* PLLP     (PLLSAI3CLK)  */
    &gclk_stm32_sai2_ext,              /* External (SAI2_EXTCLK) */
};

/* TODO: needs custom driver that is muxable and gateable */
const gclk_mux_ll_t gclk_stm32_sai2_mux = {
GCLK_STM32_GATEABLE_MUX_STATIC_INIT(SAI2,
                                    GCLK_STM32_RCC_CCIPR,
                                    RCC_CCIPR_SAI2SEL_Msk, /* SAI2SEL[1:0] RCC_CCIPR[25:24] */
                                    GCLK_STM32_RCC_APB2ENR,
                                    RCC_APB2ENR_SAI2EN_Pos),
GCLK_PARENT_LIST_STATIC_INIT(_sai2_clk_mux_configs),
};

static const gclk_t * const _mco_clk_mux_configs[] = {
    /* values of MCOSEL[3:0] RCC_CFGR[27:24] (on STM32L496xx/4A6xx devices) or
                 MCOSEL[2:0] RCC_CFGR[26:24] (on STM32L475xx/476xx/486xx devices) */
    NULL,                         /* *NONE* TODO: implement as separate gate */
    &gclk_stm32_sysclk_mux.base,    /* SYSCLK */
    &gclk_stm32_msi_gate.base,    /* MSI */
    &gclk_stm32_hsi16_gate.base,  /* HSI16 */
    &gclk_stm32_hse_gate.base,    /* HSE */
    &gclk_stm32_pll_r_scaler.base,  /* MAIN PLL (PLLR) */
    &gclk_stm32_lsi_gate.base,    /* LSI */
    &gclk_stm32_lse_gate.base,    /* LSE */
#if defined(CPU_MODEL_STM32L496ZG) || defined(CPU_MODEL_STM32L496AG)
    gclk_stm32_hsi48_not_defined, /* HSI48 (only on STM32L496xx/4A6xx devices) */
#endif
};

const gclk_mux_ll_t gclk_stm32_mco_mux = {
GCLK_PLAIN_MUX_STATIC_INIT(MCOMUX,
                           GCLK_STM32_RCC_CFGR,
                           RCC_CFGR_MCOSEL_Msk), /* MCOSEL[2:0] RCC_CFGR[26:24] */
GCLK_PARENT_LIST_STATIC_INIT(_mco_clk_mux_configs),
};

static const gclk_t * const _adc_clk_mux_configs[] = {
    /* values of ADCSEL[1:0] RCC_CCIPR[29:28] */
    NULL,                            /* NONE i.e. clock to ADC disabled -> maybe move this to a gate config */
    &gclk_stm32_pllsai1_r_scaler.base, /* PLLSAI1R */
    &gclk_stm32_pllsai2_r_scaler.base, /* PLLSAI2R */
    &gclk_stm32_sysclk_mux.base,       /* SYSCLK */
};

const gclk_mux_ll_t gclk_stm32_adc_mux = {
GCLK_STM32_GATEABLE_MUX_STATIC_INIT(ADC,
                                    GCLK_STM32_RCC_CCIPR,
                                    RCC_CCIPR_ADCSEL_Msk, /* ADCSEL[1:0] RCC_CCIPR[29:28] */
                                    GCLK_STM32_RCC_AHB2ENR,
                                    RCC_AHB2ENR_ADCEN_Pos),
GCLK_PARENT_LIST_STATIC_INIT(_adc_clk_mux_configs),
};

static const gclk_t * const _i2c_1_2_3_clk_mux_configs[] = {
    /* values of I2C3SEL[1:0] RCC_CCIPR[17:16] and
                 I2C2SEL[1:0] RCC_CCIPR[15:14] and
                 I2C1SEL[1:0] RCC_CCIPR[13:12] respectively */
    &gclk_stm32_apb1_scaler.base,  /* APB1 (PCLK1)   */
    &gclk_stm32_sysclk_mux.base,   /* SYSCLK */
    &gclk_stm32_hsi16_gate.base, /* HSI16  */
  //{ .parent = ??,   .config_reg_val = 0b11 }, /* Reserved  (remove?)  */
};

const gclk_mux_ll_t gclk_stm32_i2c1_mux = {
GCLK_STM32_GATEABLE_MUX_STATIC_INIT(I2C1,
                                    GCLK_STM32_RCC_CCIPR,
                                    RCC_CCIPR_I2C1SEL_Msk, /*  I2C1SEL[1:0] RCC_CCIPR[13:12] */
                                    GCLK_STM32_RCC_APB1ENR1,
                                    RCC_APB1ENR1_I2C1EN_Pos),
GCLK_PARENT_LIST_STATIC_INIT(_i2c_1_2_3_clk_mux_configs),
};

const gclk_mux_ll_t gclk_stm32_i2c2_mux = {
GCLK_STM32_GATEABLE_MUX_STATIC_INIT(I2C2,
                                    GCLK_STM32_RCC_CCIPR,
                                    RCC_CCIPR_I2C2SEL_Msk, /* I2C2SEL[1:0] RCC_CCIPR[15:14] */
                                    GCLK_STM32_RCC_APB1ENR1,
                                    RCC_APB1ENR1_I2C2EN_Pos),
GCLK_PARENT_LIST_STATIC_INIT(_i2c_1_2_3_clk_mux_configs),
};

const gclk_mux_ll_t gclk_stm32_i2c3_mux = {
GCLK_STM32_GATEABLE_MUX_STATIC_INIT(I2C3,
                                    GCLK_STM32_RCC_CCIPR,
                                    RCC_CCIPR_I2C3SEL_Msk, /* I2C3SEL[1:0] RCC_CCIPR[17:16] */
                                    GCLK_STM32_RCC_APB1ENR1,
                                    RCC_APB1ENR1_I2C3EN_Pos),
GCLK_PARENT_LIST_STATIC_INIT(_i2c_1_2_3_clk_mux_configs),
};

static const gclk_t * const _dfsdm1_clk_mux_configs[] = {
    /* values of DFSDM1SEL[0] RCC_CCIPR[31] */
    &gclk_stm32_apb2_scaler.base, /* APB2 (PCLK2) */
    &gclk_stm32_sysclk_mux.base,  /* SYSCLK */
};

const gclk_mux_ll_t gclk_stm32_dfsdm1_mux = {
GCLK_STM32_GATEABLE_MUX_STATIC_INIT(DFSDM1,
                                    GCLK_STM32_RCC_CCIPR,
                                    RCC_CCIPR_DFSDM1SEL_Msk, /* DFSDM1SEL[0:0] RCC_CCIPR[31:31] */
                                    GCLK_STM32_RCC_APB2ENR,
                                    RCC_APB2ENR_DFSDM1EN_Pos),
GCLK_PARENT_LIST_STATIC_INIT(_dfsdm1_clk_mux_configs),
};

/* valid for all uarts apart from usart1, which uses PCLK2 instead of PCLK1
   The values are actually the same so maybe there is an efficient way to merge it */
static const gclk_t * const _uart_lp1_5_4_3_2_clk_mux_configs[] = {
    /* values of LPUART1SEL[1:0] RCC_CCIPR[11:10] and
                 UART5SEL[1:0] RCC_CCIPR[9:8] and
                 UART4SEL[1:0] RCC_CCIPR[7:6] and
                 USART3SEL[1:0] RCC_CCIPR[5:4] and
                 USART2SEL[1:0] RCC_CCIPR[3:2] respectively */
    &gclk_stm32_apb1_scaler.base,  /* APB1 (PCLK1)   */
    &gclk_stm32_sysclk_mux.base,   /* SYSCLK */
    &gclk_stm32_hsi16_gate.base, /* HSI16  */
    &gclk_stm32_lse_gate.base,   /* LSE */
};

const gclk_mux_ll_t gclk_stm32_lpuart1_mux = {
GCLK_STM32_GATEABLE_MUX_STATIC_INIT(LPUART1,
                                    GCLK_STM32_RCC_CCIPR,
                                    RCC_CCIPR_LPUART1SEL_Msk, /* LPUART1SEL[1:0] RCC_CCIPR[11:10] */
                                    GCLK_STM32_RCC_APB1ENR2,
                                    RCC_APB1ENR2_LPUART1EN_Pos),
GCLK_PARENT_LIST_STATIC_INIT(_uart_lp1_5_4_3_2_clk_mux_configs),
};

const gclk_mux_ll_t gclk_stm32_uart5_mux = {
GCLK_STM32_GATEABLE_MUX_STATIC_INIT(UART5,
                                    GCLK_STM32_RCC_CCIPR,
                                    RCC_CCIPR_UART5SEL_Msk, /* UART5SEL[1:0] RCC_CCIPR[9:8] */
                                    GCLK_STM32_RCC_APB1ENR1,
                                    RCC_APB1ENR1_UART5EN_Pos),
GCLK_PARENT_LIST_STATIC_INIT(_uart_lp1_5_4_3_2_clk_mux_configs),
};

const gclk_mux_ll_t gclk_stm32_uart4_mux = {
GCLK_STM32_GATEABLE_MUX_STATIC_INIT(UART4,
                                    GCLK_STM32_RCC_CCIPR,
                                    RCC_CCIPR_UART4SEL_Msk, /* UART4SEL[1:0] RCC_CCIPR[7:6] */
                                    GCLK_STM32_RCC_APB1ENR1,
                                    RCC_APB1ENR1_UART4EN_Pos),
GCLK_PARENT_LIST_STATIC_INIT(_uart_lp1_5_4_3_2_clk_mux_configs),
};

const gclk_mux_ll_t gclk_stm32_usart3_mux = {
GCLK_STM32_GATEABLE_MUX_STATIC_INIT(USART3,
                                    GCLK_STM32_RCC_CCIPR,
                                    RCC_CCIPR_USART3SEL_Msk, /* USART3SEL[1:0] RCC_CCIPR[5:4] */
                                    GCLK_STM32_RCC_APB1ENR1,
                                    RCC_APB1ENR1_USART3EN_Pos),
GCLK_PARENT_LIST_STATIC_INIT(_uart_lp1_5_4_3_2_clk_mux_configs),
};

const gclk_mux_ll_t gclk_stm32_usart2_mux = {
GCLK_STM32_GATEABLE_MUX_STATIC_INIT(USART2,
                                    GCLK_STM32_RCC_CCIPR,
                                    RCC_CCIPR_USART2SEL_Msk, /* USART2SEL[1:0] RCC_CCIPR[3:2] */
                                    GCLK_STM32_RCC_APB1ENR1,
                                    RCC_APB1ENR1_USART2EN_Pos),
GCLK_PARENT_LIST_STATIC_INIT(_uart_lp1_5_4_3_2_clk_mux_configs),
};


static const gclk_t * const _usart_1_clk_mux_configs[] = {
    /* values of USART1SEL[1:0] RCC_CCIPR[1:0] */
    &gclk_stm32_apb2_scaler.base,  /* APB2 (PCLK2)   */
    &gclk_stm32_sysclk_mux.base,   /* SYSCLK */
    &gclk_stm32_hsi16_gate.base, /* HSI16  */
    &gclk_stm32_lse_gate.base,   /* LSE */
};

const gclk_mux_ll_t gclk_stm32_usart1_mux = {
GCLK_STM32_GATEABLE_MUX_STATIC_INIT(USART1,
                                    GCLK_STM32_RCC_CCIPR,
                                    RCC_CCIPR_USART1SEL_Msk, /* USART1SEL[1:0] RCC_CCIPR[1:0] */
                                    GCLK_STM32_RCC_APB2ENR,
                                    RCC_APB2ENR_USART1EN_Pos),
GCLK_PARENT_LIST_STATIC_INIT(_usart_1_clk_mux_configs),
};

static const gclk_t * const _swpmi1_clk_mux_configs[] = {
    /* values of SWPMI1SEL[0] RCC_CCIPR[30] */
    &gclk_stm32_apb1_scaler.base,  /* APB1 (PCLK1) */
    &gclk_stm32_hsi16_gate.base, /* HSI16 */
};

const gclk_mux_ll_t gclk_stm32_swpmi1_mux = {
GCLK_STM32_GATEABLE_MUX_STATIC_INIT(SWPMI1,
                                    GCLK_STM32_RCC_CCIPR,
                                    RCC_CCIPR_SWPMI1SEL_Msk, /* SWPMI1SEL[0:0] RCC_CCIPR[30:30] */
                                    GCLK_STM32_RCC_APB1ENR2,
                                    RCC_APB1ENR2_SWPMI1EN_Pos),
GCLK_PARENT_LIST_STATIC_INIT(_swpmi1_clk_mux_configs),
};

static const gclk_t * const  _lptim_1_2_clk_mux_configs[] = {
    /* values of LPTIM2SEL[1:0] RCC_CCIPR[21:20] and
                 LPTIM1SEL[1:0] RCC_CCIPR[19:18] respectively*/
    &gclk_stm32_apb1_scaler.base,  /* APB1 (PCLK1)  */
    &gclk_stm32_lsi_gate.base,   /* LSI   */
    &gclk_stm32_hsi16_gate.base, /* HSI16 */
    &gclk_stm32_lse_gate.base,   /* LSE   */
};

const gclk_mux_ll_t gclk_stm32_lptim1_mux = {
GCLK_STM32_GATEABLE_MUX_STATIC_INIT(LPTIM1,
                                    GCLK_STM32_RCC_CCIPR,
                                    RCC_CCIPR_LPTIM1SEL_Msk, /* LPTIM1SEL[1:0] RCC_CCIPR[19:18] */
                                    GCLK_STM32_RCC_APB1ENR1,
                                    RCC_APB1ENR1_LPTIM1EN_Pos),
GCLK_PARENT_LIST_STATIC_INIT(_lptim_1_2_clk_mux_configs),
};

const gclk_mux_ll_t gclk_stm32_lptim2_mux = {
GCLK_STM32_GATEABLE_MUX_STATIC_INIT(LPTIM2,
                                    GCLK_STM32_RCC_CCIPR,
                                    RCC_CCIPR_LPTIM2SEL_Msk, /* LPTIM2SEL[1:0] RCC_CCIPR[21:20] */
                                    GCLK_STM32_RCC_APB1ENR2,
                                    RCC_APB1ENR2_LPTIM2EN_Pos),
GCLK_PARENT_LIST_STATIC_INIT(_lptim_1_2_clk_mux_configs),
};
/* @todo implement custom scaler for APB1/2 conditional x1 x2 multiplier that is fed to timers */
//TODO: all the parents link in private data structures probably have to go to the generic upper layer

//TODO currently not considered
//static const gclk_range32_t _vco_freq_in_constraint = {
//    .min =  4000000,  .max =  16000000,
//};

//TODO currently not considered
//static const gclk_range32_t _vco_freq_out_constraint = {
//    .min = 64000000,  .max = 344000000,
//};

//TODO currently not considered
//static const gclk_range32_t _pll_pqr_freq_out_constraint = {
//    .min = 0,         .max = 80000000,
//};

/* The value from the register directly maps to the respective numeric value for calculation */
static const gclk_range8_t _pll_n_factor_range = {
    .min =  8, .max =  86,
};

/* @todo here we face a problem: the output frequency of the conditional "post APBx"/"pre TIMx" multiplier node requires
         internal state of the APB2 prescaler. Just knowing the frequency of the parent (APBx) is not enough, because
         the condition does not depend on the frequency itself but the internal prescaler value.
         Possible solutions:
         (A) give access to the internals of the parent
         (not only the current state but also the hypothetical i.e. "asked for" state).
         (B) handover the whole topology config that we want to query.
            - That way we can deduce the prescaler value from the parent freq and the parent's parent-freq
              (a.k.a. APBx freq and AHB freq in this case)
         (C) drop support for preliminary clock config evaluation (always require to change all setting up to a child
             before it can be queried for options)
         (D) make explicit exceptions for children that can not be pre-evaluated and signal the "undecidable" state somehow
             -> i.e. - indicate the frequency conditionally depends on the parent topology
                     - indicate options: for a given freq the output may be x or y or whatever */
unsigned long gclk_stm32_common_post_apb_mul_scaler_check_freq(const gclk_t *ll,
                                                                      clk_topology_entry_t *input_topology,
                                                                      uint32_t topology_len, uint32_t hz,
                                                                      uint32_t flags)
{
    assert(topology_len >= 2);
    (void)ll;
    (void)hz;
    (void)flags;

    /* in this particular case we know there is only one possibile topology for the next two parents:
       PCLKx is driven by APBx, APBx is driven by AHB */
    uint32_t apbx_hz = input_topology[0].clk_freq;
    uint32_t ahb_hz =  input_topology[1].clk_freq;

    uint32_t pre_tim_mul = 2;

    /* if APBx and AHB frequencies are equal, APBx prescaler must be 1, forcing the pre-timer multiplier to 1 */
    if (apbx_hz == ahb_hz) {
        pre_tim_mul = 1;
    }

    return apbx_hz * pre_tim_mul;
}

unsigned int gclk_stm32_uptree_dependent_scaler_get_factor(const gclk_t *clk) {
    return clk->cross_ref_factor_map_op(clk, NULL);
}

/* Implementation of only the scale_ops interface for a read only scaler where
 * its scaling value strictly depends on another clock (read-only).
 **/
static const gclk_op_t _stm32_conditional_read_only_scaler_ops[] = {
  { .scale_ops = { .get_factor = gclk_stm32_uptree_dependent_scaler_get_factor,
                   .set_factor = NULL,}},
};

/* _pll_p_div_confs, _pll_qr_div_confs, _msi_range_confs, _stm32_m_div_mul_confs, _stm32_mco_div_confs
   all encode the register value as index */
static const uint8_t _pll_p_div_confs[] = { 7, 17 };
static const uint8_t _pll_qr_div_confs[] = { 2, 4, 6, 8 };
/* The factors assume an MSI-internal base clock of 100000Hz */
static const uint16_t _msi_range_confs[] = { 1, 2, 4, 8, 10, 20, 40, 80, 160, 240, 320, 480 };
static const uint8_t _stm32_m_div_mul_confs[] = { 1, 2, 3, 4, 5, 6, 7 ,8 };
/* an equivalent encoding for the below list would be the following LUT:
 * { .factor = 1,  .reg_val = 0b000 },
 * { .factor = 2,  .reg_val = 0b001 },
 * { .factor = 4,  .reg_val = 0b010 },
 * { .factor = 8,  .reg_val = 0b011 },
 * { .factor = 16, .reg_val = 0b100 }, */
static const uint8_t _stm32_mco_div_confs[] = { 1, 2, 4, 8, 16, };


static const gclk_reg_val_factor_lut_t _msis_range_confs[] = {
    // The factor assumes an MSI-internal base clock of 100000Hz */
    { .factor = 10,  .reg_val = 0b0100 },
    { .factor = 20,  .reg_val = 0b0101 },
    { .factor = 40,  .reg_val = 0b0110 },
    { .factor = 80,  .reg_val = 0b0111 },
};

// Note: setting this correctly needs to consider voltage in the future (DVFS)
static const gclk_reg_val_factor_lut_t _stm32_ahb_div_confs[] = {
    /* actually any 0b0xxx maps to 1 too -> do we need to include that?
       @todo: if the HW doesn't set any of those dont-care values to indicate a factor of 1, we can leave it as is
              because the implementation will always use the only 0b0000 value (check with reset/default values) */
    { .factor = 1,   .reg_val = 0b0000 },
    { .factor = 2,   .reg_val = 0b1000 },
    { .factor = 4,   .reg_val = 0b1001 },
    { .factor = 8,   .reg_val = 0b1010 },
    { .factor = 16,  .reg_val = 0b1011 },
    { .factor = 64,  .reg_val = 0b1100 },
    { .factor = 128, .reg_val = 0b1100 },
    { .factor = 256, .reg_val = 0b1100 },
    { .factor = 512, .reg_val = 0b1100 },
};

static const gclk_reg_val_factor_lut_t _stm32_apb_div_confs[] = {
    { .factor = 1,   .reg_val = 0b000 },
    { .factor = 2,   .reg_val = 0b100 },
    { .factor = 4,   .reg_val = 0b101 },
    { .factor = 8,   .reg_val = 0b110 },
    { .factor = 16,  .reg_val = 0b111 },
};

#ifdef GCLK_USE_SEPARATE_CONF_REG_ARRAYS
/* @todo: evaluate benefit of alternative storage:
         - dynamic array, storing the size/availability via flags?
         - fixed array of all needed config regs with flag/enum based access to the regs */
static const gclk_scaler_regs_t _sr_cfgr = {
    .enable_reg = NULL,
    .ready_reg  = NULL,
    .scaler_reg = &RCC->CFGR,
};

static const gclk_scaler_regs_t _sr_cr = {
    .enable_reg = NULL,
    .ready_reg  = NULL,
    .scaler_reg = &RCC->CR,
};

/* @todo: does this really not need the ready reg? */
static const gclk_scaler_regs_t _sr_pllcfgr = {
    .enable_reg = NULL,
    .ready_reg  = NULL,
    .scaler_reg = &RCC->PLLCFGR,
};

static const gclk_scaler_regs_t _ersr_pllcfgr = {
    .enable_reg = &RCC->PLLCFGR,
    .ready_reg  = NULL,
    .scaler_reg = &RCC->PLLCFGR,
};

static const gclk_scaler_regs_t _ersr_pllsai1cfgr = {
    .enable_reg = &RCC->PLLSAI1CFGR,
    .ready_reg  = NULL,
    .scaler_reg = &RCC->PLLSAI1CFGR,
};

static const gclk_scaler_regs_t _ersr_pllsai2cfgr = {
    .enable_reg = &RCC->PLLSAI2CFGR,
    .ready_reg  = NULL,
    .scaler_reg = &RCC->PLLSAI2CFGR,
};

static const gclk_scaler_regs_t _errrsr_cr_cr_pllcfgr = {
    .enable_reg = &RCC->CR,
    .ready_reg  = &RCC->CR,
    .scaler_reg = &RCC->PLLCFGR,
};

static const gclk_scaler_regs_t _sr_csr = {
    .enable_reg = NULL,
    .ready_reg  = NULL,
    .scaler_reg = &RCC->CSR,
};
#endif

/* Reuses the plain scaler driver that only implements the scale_ops interface
 * for a scaler that is interfaced via a bitfield in a read/write register that
 * exposes the current selection and also allows changing it. */
#define GCLK_PLAIN_SCALER_STATIC_INIT(NAME,PARENT,CONF_REG_ID,CONF_REG_MASK)\
.base.separated_ops      = gclk_plain_scaler_ops,\
.base.flags.scalable     = 1,\
.base.fixed_parent       = PARENT,\
.regref                  = { .conf     = CONF_REG_ID,\
                             .conf_lsb = STATIC_BITMASK_LSB(CONF_REG_MASK),\
                             .conf_msb = STATIC_BITMASK_MSB(CONF_REG_MASK) },\
.base.name               = #NAME

#define GCLK_PLAIN_DIV_SCALER_STATIC_INIT(NAME,PARENT,CONF_REG_ID,CONF_REG_MASK)\
GCLK_PLAIN_SCALER_STATIC_INIT(NAME,PARENT,CONF_REG_ID,CONF_REG_MASK),\
.base.flags.scaler_type  = GCLK_DIV

#define GCLK_PLAIN_MUL_SCALER_STATIC_INIT(NAME,PARENT,CONF_REG_ID,CONF_REG_MASK)\
GCLK_PLAIN_SCALER_STATIC_INIT(NAME,PARENT,CONF_REG_ID,CONF_REG_MASK),\
.base.flags.scaler_type  = GCLK_MUL

//TODO this is probably reusable elsewhere -> move to somewhere for othjers to reuse
/* A driver for a gateable scaler that just reuses the basic imlementations
 * of the scaler and gate */
const gclk_op_t gclk_gateable_scaler_ops[] = {
   { .scale_ops = { .get_factor = gclk_generic_scaler_get_factor,
                    .set_factor = gclk_generic_scaler_set_factor,}},
   { .gate_ops  = { .is_enabled = gclk_generic_gate_ed_rdy_reg_is_enabled,
                    .enable     = gclk_generic_gate_ed_rdy_reg_enable,}},
};

#define GCLK_CONDITONAL_MUL_SCALER_STATIC_INIT(NAME,PARENT,CONF_REG_ID,CONF_REG_MASK)\
.base.separated_ops  = _stm32_conditional_read_only_scaler_ops,\
.base.flags.scalable = 1,\
.base.flags.scaler_type  = GCLK_MUL,\
.base.fixed_parent   = PARENT,\
.regref              = { .conf     = CONF_REG_ID,\
                         .conf_lsb = STATIC_BITMASK_LSB(CONF_REG_MASK),\
                         .conf_msb = STATIC_BITMASK_MSB(CONF_REG_MASK) },\
.base.name           = #NAME

#define GCLK_GATEABLE_SCALER_STATIC_INIT(NAME,PARENT,EN_REG_ID,EN_BIT,CONF_REG_ID,CONF_REG_MASK)\
.base.separated_ops  = gclk_gateable_scaler_ops,\
.base.flags.gateable = 1,\
.base.flags.scalable = 1,\
.base.fixed_parent   = PARENT,\
.regref              = { .en       = EN_REG_ID,\
                         .conf     = CONF_REG_ID,\
                         .en_bit   = EN_BIT,\
                         .conf_lsb = STATIC_BITMASK_LSB(CONF_REG_MASK),\
                         .conf_msb = STATIC_BITMASK_MSB(CONF_REG_MASK) },\
.base.name           = #NAME

#define GCLK_GATEABLE_DIV_SCALER_STATIC_INIT(NAME,PARENT,EN_REG_ID,EN_BIT,CONF_REG_ID,CONF_REG_MASK)\
GCLK_GATEABLE_SCALER_STATIC_INIT(NAME,PARENT,EN_REG_ID,EN_BIT,CONF_REG_ID,CONF_REG_MASK),\
.base.flags.scaler_type  = GCLK_DIV

/* initializes a gateable mul scaler that makes use of the ready reg/bit */
#define GCLK_GATEABLE_MUL_SCALER_STATIC_INIT(NAME,PARENT,EN_REG_ID,EN_BIT,RDY_REG_ID,RDY_BIT,CONF_REG_ID,CONF_REG_MASK)\
GCLK_GATEABLE_SCALER_STATIC_INIT(NAME,PARENT,EN_REG_ID,EN_BIT,CONF_REG_ID,CONF_REG_MASK),\
.regref.rdy = RDY_REG_ID,\
.regref.rdy_bit = RDY_BIT,\
.base.flags.scaler_type  = GCLK_MUL

/* @todo: introduce the concept of a base-scaler?
          I: a base scaler is defined for an instance: reuse all infos from the base scaler instead of own data
          II: no base scaler is definded: every scaler needs to provide it's own information */
//gclk_clk_scaler_ll_t stm32_clk_scalers[]
const gclk_clk_scaler_ll_t gclk_stm32_ahb_scaler = {
GCLK_PLAIN_DIV_SCALER_STATIC_INIT(AHB,
                                  &gclk_stm32_sysclk_mux.base,
                                  GCLK_STM32_RCC_CFGR,
                                  RCC_CFGR_HPRE_Msk), /* HPRE[3:0] (CFGR[7:4]) */
GCLK_FACTOR_LUT_STATIC_INIT(_stm32_ahb_div_confs),
};

const gclk_clk_scaler_ll_t gclk_stm32_apb1_scaler = {
GCLK_PLAIN_DIV_SCALER_STATIC_INIT(APB1,
                                  &gclk_stm32_ahb_scaler.base,
                                  GCLK_STM32_RCC_CFGR,
                                  RCC_CFGR_PPRE1_Msk), /* PPRE1[2:0] (CFGR[10:8]) */
GCLK_FACTOR_LUT_STATIC_INIT(_stm32_apb_div_confs),
};

const gclk_clk_scaler_ll_t gclk_stm32_apb2_scaler = {
GCLK_PLAIN_DIV_SCALER_STATIC_INIT(APB2,
                                  &gclk_stm32_ahb_scaler.base,
                                  GCLK_STM32_RCC_CFGR,
                                  RCC_CFGR_PPRE2_Msk), /* PPRE2[2:0] (CFGR[13:11]) */
GCLK_FACTOR_LUT_STATIC_INIT(_stm32_apb_div_confs),
};

const gclk_clk_scaler_ll_t gclk_stm32_pll_m_scaler = {
GCLK_PLAIN_DIV_SCALER_STATIC_INIT(PLL_M,
                                  &gclk_stm32_pll_pre_div_mux.base,
                                  GCLK_STM32_RCC_PLLCFGR,
                                  RCC_PLLCFGR_PLLM_Msk), /* PLLM[2:0] (PLLCFGR[6:4]) */
GCLK_LIST8_STATIC_INIT(_stm32_m_div_mul_confs),
    .base.flags.topology_flags = GCLK_STOP_CHILDREN_FOR_UPDATE,
//    .out_freq_constraint       = &_vco_freq_in_constraint,
};

const gclk_clk_scaler_ll_t gclk_stm32_msirange_scaler = {
    // TODO use the existing member for internal fixed source freq
    // TODO also the startup frequency is specified via a separate register: RCC_CSR MSISRANGE -> how to model that?
    //      For now: separate msisrange_scaler (below) and then a virtual mux after both
GCLK_PLAIN_MUL_SCALER_STATIC_INIT(MSIRANGE,
                                  &gclk_stm32_msi_base,
                                  GCLK_STM32_RCC_CR,
                                  RCC_CR_MSIRANGE_Msk), /* MSIRANGE[3:0] (RCC_CR[7:4]) */
GCLK_LIST16_STATIC_INIT(_msi_range_confs),
};

const gclk_clk_scaler_ll_t gclk_stm32_msisrange_scaler = {
    //TODO: same as above: use internal fixed freq instead of fixed parent
GCLK_PLAIN_MUL_SCALER_STATIC_INIT(MSISRANGE,
                                  &gclk_stm32_msi_base,
                                  GCLK_STM32_RCC_CSR,
                                  RCC_CSR_MSISRANGE_Msk), /* MSISRANGE[3:1] (RCC_CSR[10:8]) */
/* This STARTUP flag is not used at the moment
 * -> default topology/scaling settings dont employ those settings.
 *  We still include it for now anyway to be able to decode this information from the registers.
 * If this shows to be a feature which is also more commonly available on other platforms, a
 * possible way to work with such settings could be to have an optional startup/default-config interface
 * extension for nodes (similar to the scaler/mux/gate options already available).
 */
 // .base.flags.topology_flags = GCLK_ONLY_FOR_STARTUP,
GCLK_FACTOR_LUT_STATIC_INIT(_msis_range_confs)
};

const gclk_clk_scaler_ll_t gclk_stm32_mco_div_scaler = {
GCLK_PLAIN_DIV_SCALER_STATIC_INIT(MCO,
                                  &gclk_stm32_mco_mux.base,
                                  GCLK_STM32_RCC_CFGR,
                                  RCC_CFGR_MCOPRE_Msk), /* MCOPRE[2:0] (RCC_CFGR[30:28]) */
GCLK_LIST8_STATIC_INIT(_stm32_mco_div_confs),
};

#if defined(CPU_MODEL_STM32L496ZG) || defined(CPU_MODEL_STM32L496AG)
/* the P divider is only available with full range on STM32L496xx/4A6xx
   on other platforms there is only a divtable with two values (either 7 or 17) available*/
static const struct gclk_range8_t _pll_p_factor_range {
    //TODO: 1 is reserved, and 0 maps the factor function to PLLP bit (to a value of 7 or 17)
    //factor value mapped directly to reg value
    .min =  2,
    .max =  31,
};

//TODO: this is a gateable scaler
/* @todo on STM32L496xx/4A6xx devices the alternative setting of P (to 7 or 17) via the single PLLP bit is available too
         -> should be safe to ignore because they are covered by the range employed below
            i.e. the same values can also be set without the bit (everywhere?) */
const gclk_clk_scaler_ll_t gclk_stm32_pll_p_scaler = {
GCLK_GATEABLE_DIV_SCALER_STATIC_INIT(PLL_P,
                                     &gclk_stm32_pll_vco_scaler.base,
                                     GCLK_STM32_RCC_PLLCFGR,
                                     RCC_PLLCFGR_PLLPEN_Pos, /* PLLCFGR[16] (PLLPEN) */
                                     GCLK_STM32_RCC_PLLCFGR,
                                     RCC_PLLCFGR_PLLPDIV_Msk), /* PLLPDIV[4:0] (PLLCFGR[31:27]) */
 GCLK_RANGE8_STATIC_INIT(_pll_p_factor_range),
    .base.flags.topology_flags = GCLK_STOP_PARENT_FOR_UPDATE,
    //.out_freq_constraint       = &_pll_pqr_freq_out_constraint,
};

const gclk_clk_scaler_ll_t gclk_stm32_pllsai1_p_scaler = {
GCLK_GATEABLE_DIV_SCALER_STATIC_INIT(PLLSAI1_P,
                                     &gclk_stm32_pllsai1_vco_scaler.base,
                                     GCLK_STM32_RCC_PLLSAI1CFGR,
                                     RCC_PLLSAI1CFGR_PLLSAI1PEN_Pos, /* PLLSAI1CFGR[16] (PLLSAI1PEN) */
                                     GCLK_STM32_RCC_PLLSAI1CFGR,
                                     RCC_PLLSAI1CFGR_PLLSAI1PDIV_Msk), /* PLLSAI1PDIV[4:0] (PLLSAI1CFGR[31:27]) */
 GCLK_RANGE8_STATIC_INIT(_pll_p_factor_range),
    //.out_freq_constraint       = &_pll_pqr_freq_out_constraint,
};

const gclk_clk_scaler_ll_t gclk_stm32_pllsai2_p_scaler = {
GCLK_GATEABLE_DIV_SCALER_STATIC_INIT(PLLSAI2_P,
                                     &gclk_stm32_pllsai2_vco_scaler.base,
                                     GCLK_STM32_RCC_PLLSAI2CFGR,
                                     RCC_PLLSAI2CFGR_PLLSAI2PEN_Pos, /* PLLSAI2CFGR[16] (PLLSAI2PEN) */
                                     GCLK_STM32_RCC_PLLSAI2CFGR,
                                     RCC_PLLSAI2CFGR_PLLSAI2PDIV_Msk), /* PLLSAI2PDIV[4:0] (PLLSAI2CFGR[31:27]) */
 GCLK_RANGE8_STATIC_INIT(_pll_p_factor_range),
    //.out_freq_constraint       = &_pll_pqr_freq_out_constraint,
};
#else /* everything that is not CPU_MODEL_STM32L496ZG || CPU_MODEL_STM32L496AG */

const gclk_clk_scaler_ll_t gclk_stm32_pll_p_scaler = {
GCLK_GATEABLE_DIV_SCALER_STATIC_INIT(PLL_P,
                                     &gclk_stm32_pll_vco_scaler.base,
                                     GCLK_STM32_RCC_PLLCFGR,
                                     RCC_PLLCFGR_PLLPEN_Pos, /* PLLCFGR[16] (PLLPEN) */
                                     GCLK_STM32_RCC_PLLCFGR,
                                     RCC_PLLCFGR_PLLP_Msk), /* PLLP[1:0] (PLLCFGR[22:21]) */
GCLK_LIST8_STATIC_INIT(_pll_p_div_confs),
    .base.flags.topology_flags = GCLK_STOP_PARENT_FOR_UPDATE,
    //.out_freq_constraint       = &_pll_pqr_freq_out_constraint,
};

const gclk_clk_scaler_ll_t gclk_stm32_pllsai1_p_scaler = {
GCLK_GATEABLE_DIV_SCALER_STATIC_INIT(PLLSAI1_P,
                                     &gclk_stm32_pllsai1_vco_scaler.base,
                                     GCLK_STM32_RCC_PLLSAI1CFGR,
                                     RCC_PLLSAI1CFGR_PLLSAI1PEN_Pos, /* PLLSAI1CFGR[16] (PLLSAI1PEN) */
                                     GCLK_STM32_RCC_PLLSAI1CFGR,
                                     RCC_PLLSAI1CFGR_PLLSAI1P_Msk), /* PLLSAI1P[0:0] (PLLSAI1CFGR[17:17]) */
GCLK_LIST8_STATIC_INIT(_pll_p_div_confs),
    .base.flags.topology_flags =  GCLK_STOP_PARENT_FOR_UPDATE,
    //.out_freq_constraint       = &_pll_pqr_freq_out_constraint,
};

const gclk_clk_scaler_ll_t gclk_stm32_pllsai2_p_scaler = {
GCLK_GATEABLE_DIV_SCALER_STATIC_INIT(PLLSAI2_P,
                                     &gclk_stm32_pllsai2_vco_scaler.base,
                                     GCLK_STM32_RCC_PLLSAI2CFGR,
                                     RCC_PLLSAI2CFGR_PLLSAI2PEN_Pos, /* PLLSAI2CFGR[16] (PLLSAI2PEN) */
                                     GCLK_STM32_RCC_PLLSAI2CFGR,
                                     RCC_PLLSAI2CFGR_PLLSAI2P_Msk), /* PLLSAI2P[0:0] (PLLSAI2CFGR[17:17]) */
GCLK_LIST8_STATIC_INIT(_pll_p_div_confs),
    .base.flags.topology_flags =  GCLK_STOP_PARENT_FOR_UPDATE,
    //.out_freq_constraint       = &_pll_pqr_freq_out_constraint,
};
#endif


const gclk_clk_scaler_ll_t gclk_stm32_pll_q_scaler = {
GCLK_GATEABLE_DIV_SCALER_STATIC_INIT(PLL_Q,
                                     &gclk_stm32_pll_vco_scaler.base,
                                     GCLK_STM32_RCC_PLLCFGR,
                                     RCC_PLLCFGR_PLLQEN_Pos, /* PLLCFGR[20] (PLLQEN) */
                                     GCLK_STM32_RCC_PLLCFGR,
                                     RCC_PLLCFGR_PLLQ_Msk), /* PLLQ[1:0] (PLLCFGR[22:21]) */
GCLK_LIST8_STATIC_INIT(_pll_qr_div_confs),
    .base.flags.topology_flags = GCLK_STOP_PARENT_FOR_UPDATE,
    //.out_freq_constraint       = &_pll_pqr_freq_out_constraint,
};

const gclk_clk_scaler_ll_t gclk_stm32_pll_r_scaler = {
GCLK_GATEABLE_DIV_SCALER_STATIC_INIT(PLL_R,
                                     &gclk_stm32_pll_vco_scaler.base,
                                     GCLK_STM32_RCC_PLLCFGR,
                                     RCC_PLLCFGR_PLLREN_Pos, /* PLLCFGR[24] (PLLREN) */
                                     GCLK_STM32_RCC_PLLCFGR,
                                     RCC_PLLCFGR_PLLR_Msk), /* PLLR[1:0] (PLLCFGR[26:25]) */
GCLK_LIST8_STATIC_INIT(_pll_qr_div_confs),
    .base.flags.topology_flags = GCLK_STOP_PARENT_FOR_UPDATE,
    //.out_freq_constraint       = &_pll_pqr_freq_out_constraint,
};

const gclk_clk_scaler_ll_t gclk_stm32_pllsai1_q_scaler = {
GCLK_GATEABLE_DIV_SCALER_STATIC_INIT(PLLSAI1_Q,
                                     &gclk_stm32_pllsai1_vco_scaler.base,
                                     GCLK_STM32_RCC_PLLSAI1CFGR,
                                     RCC_PLLSAI1CFGR_PLLSAI1QEN_Pos, /* PLLSAI1CFGR[20] (PLLSAI1QEN) */
                                     GCLK_STM32_RCC_PLLSAI1CFGR,
                                     RCC_PLLSAI1CFGR_PLLSAI1Q_Msk), /* PLLSAI1Q[1:0] (PLLSAI1CFGR[22:21]) */
GCLK_LIST8_STATIC_INIT(_pll_qr_div_confs),
    .base.flags.topology_flags = GCLK_STOP_PARENT_FOR_UPDATE,
    //.out_freq_constraint       = &_pll_pqr_freq_out_constraint,
};

const gclk_clk_scaler_ll_t gclk_stm32_pllsai1_r_scaler = {
GCLK_GATEABLE_DIV_SCALER_STATIC_INIT(PLLSAI1_R,
                                     &gclk_stm32_pllsai1_vco_scaler.base,
                                     GCLK_STM32_RCC_PLLSAI1CFGR,
                                     RCC_PLLSAI1CFGR_PLLSAI1REN_Pos, /* PLLSAI1CFGR[24] (PLLSAI1REN) */
                                     GCLK_STM32_RCC_PLLSAI1CFGR,
                                     RCC_PLLSAI1CFGR_PLLSAI1R_Msk), /* PLLSAI1R[1:0] (PLLSAI1CFGR[26:25]) */
GCLK_LIST8_STATIC_INIT(_pll_qr_div_confs),
    .base.flags.topology_flags = GCLK_STOP_PARENT_FOR_UPDATE,
    //.out_freq_constraint       = &_pll_pqr_freq_out_constraint,
};

const gclk_clk_scaler_ll_t gclk_stm32_pllsai2_r_scaler = {
GCLK_GATEABLE_DIV_SCALER_STATIC_INIT(PLLSAI2_R,
                                     &gclk_stm32_pllsai2_vco_scaler.base,
                                     GCLK_STM32_RCC_PLLSAI2CFGR,
                                     RCC_PLLSAI2CFGR_PLLSAI2REN_Pos, /* PLLSAI2CFGR[24] (PLLSAI2REN) */
                                     GCLK_STM32_RCC_PLLSAI2CFGR,
                                     RCC_PLLSAI2CFGR_PLLSAI2R_Msk), /* PLLSAI2R[1:0] (PLLSAI2CFGR[26:25]) */
GCLK_LIST8_STATIC_INIT(_pll_qr_div_confs),
    .base.flags.topology_flags = GCLK_STOP_PARENT_FOR_UPDATE,
    //.out_freq_constraint       = &_pll_pqr_freq_out_constraint,
};

const gclk_clk_scaler_ll_t gclk_stm32_pll_vco_scaler = {
GCLK_GATEABLE_MUL_SCALER_STATIC_INIT(PLL_VCO,
                                     &gclk_stm32_pll_m_scaler.base,
                                     GCLK_STM32_RCC_CR,
                                     RCC_CR_PLLON_Pos, /* RCC_CR[24] PLLON */
                                     GCLK_STM32_RCC_CR,
                                     RCC_CR_PLLRDY_Pos, /*RCC_CR[25] PLLRDY */
                                     GCLK_STM32_RCC_PLLCFGR,
                                     RCC_PLLCFGR_PLLN_Msk), /* PLLN[6:0] (PLLCFGR[14:8]) */
GCLK_REGVAL_AS_NUMVAL_RANGE8_STATIC_INIT(_pll_n_factor_range),
    .base.flags.topology_flags = GCLK_STOP_FOR_UPDATE,
    //.out_freq_constraint       = &_vco_freq_out_constraint,
};

const gclk_clk_scaler_ll_t gclk_stm32_pllsai1_vco_scaler = {
GCLK_GATEABLE_MUL_SCALER_STATIC_INIT(PLLSAI1_VCO,
                                     &gclk_stm32_pll_m_scaler.base,
                                     GCLK_STM32_RCC_CR,
                                     RCC_CR_PLLSAI1ON_Pos,  /* RCC_CR[26] PLLSAI1ON */
                                     GCLK_STM32_RCC_CR,
                                     RCC_CR_PLLSAI1RDY_Pos, /* RCC_CR[27] PLLSAI1RDY */
                                     GCLK_STM32_RCC_PLLSAI1CFGR,
                                     RCC_PLLSAI1CFGR_PLLSAI1N_Msk), /* PLLN[6:0] (PLLSAI1CFGR[14:8]) */
GCLK_REGVAL_AS_NUMVAL_RANGE8_STATIC_INIT(_pll_n_factor_range),
    .base.flags.topology_flags = GCLK_STOP_FOR_UPDATE,
    //.out_freq_constraint       = &_vco_freq_out_constraint,
};

const gclk_clk_scaler_ll_t gclk_stm32_pllsai2_vco_scaler = {
GCLK_GATEABLE_MUL_SCALER_STATIC_INIT(PLLSAI2_VCO,
                                     &gclk_stm32_pll_m_scaler.base,
                                     GCLK_STM32_RCC_CR,
                                     RCC_CR_PLLSAI2ON_Pos, /* RCC_CR[28] PLLSAI2ON */
                                     GCLK_STM32_RCC_CR,
                                     RCC_CR_PLLSAI2RDY_Pos, /* RCC_CR[29] PLLSAI2RDY */
                                     GCLK_STM32_RCC_PLLSAI2CFGR,
                                     RCC_PLLSAI2CFGR_PLLSAI2N_Msk), /* PLLN[6:0] (PLLSAI2CFGR[14:8]) */
GCLK_REGVAL_AS_NUMVAL_RANGE8_STATIC_INIT(_pll_n_factor_range),
    .base.flags.topology_flags = GCLK_STOP_FOR_UPDATE,
    //.out_freq_constraint       = &_vco_freq_out_constraint,
};

uint32_t apb12_post_mul_cross_ref_factor_luf(const gclk_t *clk, const clk_topology_entry_t *other_clock_conf) {
    if (other_clock_conf) {
        return other_clock_conf->factor == 1 ? 1 : 2;
    } else {
        return gclk_get_current_factor(clk->factor_mapping.cross_ref->ref_clk) == 1 ? 1 : 2;
    }
}

const gclk_reg_val_cross_ref_luf_t _apb1_postmul_luf = {
    .ref_clk = &gclk_stm32_apb1_scaler.base,
    .luf = &apb12_post_mul_cross_ref_factor_luf,
};

const gclk_reg_val_cross_ref_luf_t _apb2_postmul_luf = {
    .ref_clk = &gclk_stm32_apb2_scaler.base,
    .luf = &apb12_post_mul_cross_ref_factor_luf,
};

/* NOTE: For clocks like this one, i.e., ones that strictly depend on other clock nodes, there are a few things to consider:
 *       - Exploration that is purely based on numerical factor representation (without a global tree model view) its
 *         configuration could be represented by a symbolic value. This may then be treated in one of the following ways:
 *          - "don't know beforehand what the hardware will do"
 *          - "finitie number of all possible states" (allows to check for possible conflicts, but may gives false positives) 
 *          - "value must be converted from symbolic to specific in a post processing step that uses platform specific
 *             dependency-description-constraints that resolve undefined state based on the respecitve state of the rest of the tree"
 *
 * this scaler multiplies the APB1 (PCLK1) clock by a fixed factor of 1 or 2 depending on the actual APB1 prescaler value.
 * If the APB prescaler is set to 1 this clock has the same frequency as APB1. Otherwise it uses twice the frequency of APB.
 * This is enforced by hardware and can not be changed by the user.
 * See reference manual page 214 section 6.2.15 (Timer Clock) for more details.
 * Since it directly depends on the APB1 config, we use the same configuration register but another GCLK_CONF_LUT */
const gclk_clk_scaler_ll_t gclk_apb1_tim_mul_scaler = {
GCLK_CONDITONAL_MUL_SCALER_STATIC_INIT(APB1{x1|x2},
                                  &gclk_stm32_apb1_scaler.base,
                                  GCLK_STM32_RCC_CFGR,
                                  RCC_CFGR_PPRE1_Msk), /* PPRE1[2:0] (CFGR[10:8]) */
GCLK_FACTOR_CROSSREF_UPTREE_LUF_STATIC_INIT(_apb1_postmul_luf),
};

/* this scaler multiplies the APB2 clock by a fixed factor of 1 or 2 depending on the actual APB2 prescaler value.
   This is enforced by hardware and can not be changed by the user.
   Since it directly depends on the APB2 config, we use the same configuration register but another GCLK_CONF_LUT */
const gclk_clk_scaler_ll_t gclk_apb2_tim_mul_scaler = {
GCLK_CONDITONAL_MUL_SCALER_STATIC_INIT(APB2{x1|x2},
                                       &gclk_stm32_apb2_scaler.base,
                                       GCLK_STM32_RCC_CFGR,
                                       RCC_CFGR_PPRE2_Msk), /* PPRE2[2:0] (CFGR[13:11]) */
GCLK_FACTOR_CROSSREF_UPTREE_LUF_STATIC_INIT(_apb2_postmul_luf),
};

/* TODO a static scaler is just metadata ontop of another clock */
const gclk_clk_scaler_ll_t gclk_stm32_hse_div32_scaler = {
GCLK_PLAIN_FIXED_SCALER_STATIC_INIT(HSE/32, 32),
    .base.fixed_parent       = &gclk_stm32_hse_gate.base,
    .base.flags.scaler_type  = GCLK_DIV,
};

const gclk_clk_scaler_ll_t gclk_stm32_ahb_div8_scaler = {
GCLK_PLAIN_FIXED_SCALER_STATIC_INIT(AHB/8, 8),
    .base.fixed_parent       = &gclk_stm32_ahb_scaler.base,
    .base.flags.scaler_type  = GCLK_DIV,
};

int gclk_enable_pin_output(const gclk_t *clk, const gpio_t pin) {
    if (clk == &gclk_stm32_mco_div_scaler.base) {
        if (pin == STM32_L476RG_MCO_PIN) {
            gpio_init(STM32_L476RG_MCO_PIN, GPIO_OUT);
            gpio_init_af(STM32_L476RG_MCO_PIN, STM32_L476RG_MCO_AF);
            return ENABLE_PIN_OUTPUT_OK;
        }
        return ENABLE_PIN_OUTPUT_INVALID_PIN;
    }

    return ENABLE_PIN_OUTPUT_INVALID_CLOCK;
}


/* @todo rework the access to clock nodes externally defined in other files.
   Currently there is no way to just hand over the generic gclk_t reference without the specific type because the
   compiler wont see the value of the const pointer to a const struct as a *constant value*, thus we cannot use it as
   initializer (e.g. as fixed parent, or parent option).
   Two things to potentially improve here:
   1: use enum IDs to index the various clock instances (also conditionally provide the enum IDs)
   2: provide such an array on a per-file basis with a "next_list" pointer and keep a gobal count in the gclk module */

const gclk_t * const gclks[] = {
#if defined(CPU_FAM_STM32L0)
    &gclk_stm32_ahb_scaler.base,
    &gclk_stm32_apb1_scaler.base,
    &gclk_stm32_apb2_scaler.base,
    &gclk_stm32_pll_div_scaler.base,
    &gclk_stm32_pll_usb_div_scaler.base,
    &gclk_stm32_pll_mul_scaler.base,
    &gclk_apb1_tim_mul_scaler.base,
    &gclk_apb2_tim_mul_scaler.base,
    &gclk_stm32_hse_div_scaler.base,
    &gclk_stm32_ahb_div8_scaler.base,
    &gclk_stm32_msirange_scaler.base,
    &gclk_stm32_mco_div_scaler.base,
    &gclk_stm32_hsi16_div_scaler.base,
    &gclk_stm32_sysclk_mux.base,
    &gclk_stm32_hsi48_mux.base,
    &gclk_stm32_rtc_lcd_mux.base,
    &gclk_stm32_pll_src_mux.base,
    &gclk_stm32_mco_mux.base,
    &gclk_stm32_i2c1_mux.base,
    &gclk_stm32_i2c3_mux.base,
    &gclk_stm32_lpuart1_mux.base,
    &gclk_stm32_usart2_mux.base,
    &gclk_stm32_usart1_mux.base,
    &gclk_stm32_lptim1_mux.base,
    &gclk_stm32_hsi16_gate.base,
    &gclk_stm32_hse_gate.base,
    &gclk_stm32_lse_gate.base,
    &gclk_stm32_lsi_gate.base,
    &gclk_stm32_msi_base,
    &gclk_stm32_msi_gate.base,
    &gclk_stm32_hsi48_gate.base,
#elif defined(CPU_FAM_STM32L4)
    &gclk_stm32_sysclk_mux.base,
    &gclk_stm32_msi_mux.base,
    &gclk_stm32_usb_rng_sdmmc_mux.base,
    &gclk_stm32_lsco_mux.base,
    &gclk_stm32_rtc_lcd_mux.base,
    &gclk_stm32_pll_pre_div_mux.base,
    &gclk_stm32_sai1_mux.base,
    &gclk_stm32_sai2_mux.base,
    &gclk_stm32_mco_mux.base,
    &gclk_stm32_adc_mux.base,
    &gclk_stm32_i2c1_mux.base,
    &gclk_stm32_i2c2_mux.base,
    &gclk_stm32_i2c3_mux.base,
    &gclk_stm32_dfsdm1_mux.base,
    &gclk_stm32_lpuart1_mux.base,
    &gclk_stm32_uart5_mux.base,
    &gclk_stm32_uart4_mux.base,
    &gclk_stm32_usart3_mux.base,
    &gclk_stm32_usart2_mux.base,
    &gclk_stm32_usart1_mux.base,
    &gclk_stm32_swpmi1_mux.base,
    &gclk_stm32_lptim1_mux.base,
    &gclk_stm32_lptim2_mux.base,
    &gclk_stm32_ahb_scaler.base,
    &gclk_stm32_apb1_scaler.base,
    &gclk_stm32_apb2_scaler.base,
    &gclk_stm32_pll_m_scaler.base,
    &gclk_stm32_pll_p_scaler.base,
    &gclk_stm32_pllsai1_p_scaler.base,
    &gclk_stm32_pllsai2_p_scaler.base,
    &gclk_stm32_pll_p_scaler.base,
    &gclk_stm32_pllsai1_q_scaler.base,
    &gclk_stm32_pll_q_scaler.base,
    &gclk_stm32_pll_r_scaler.base,
    &gclk_stm32_pll_q_scaler.base,
    &gclk_stm32_pllsai1_r_scaler.base,
    &gclk_stm32_pllsai2_r_scaler.base,
    &gclk_stm32_pll_vco_scaler.base,
    &gclk_stm32_pllsai1_vco_scaler.base,
    &gclk_stm32_pllsai2_vco_scaler.base,
    &gclk_apb1_tim_mul_scaler.base,
    &gclk_apb2_tim_mul_scaler.base,
    &gclk_stm32_hse_div32_scaler.base,
    &gclk_stm32_ahb_div8_scaler.base,
    &gclk_stm32_msirange_scaler.base,
    &gclk_stm32_msisrange_scaler.base,
    &gclk_stm32_mco_div_scaler.base,
    &gclk_stm32_msi_gate.base,
    &gclk_stm32_sai1_ext,
    &gclk_stm32_sai2_ext,
    &gclk_stm32_hsi16_gate.base,
    &gclk_stm32_hse_gate.base,
    &gclk_stm32_lse_gate.base,
    &gclk_stm32_lsi_gate.base,
    &gclk_stm32_msi_base,
    &gclk_stm32_tim5_gate.base,
#endif
};

//gclk_t const *gclock_core_clock_handle = &gclk_stm32_sysclk_mux.base;
// should actually be this:gclk_t const *gclock_handle_for_core_freq = &gclk_stm32_ahb_scaler.base;
// MSI is probably used instead to expresss "control core clock via MSI
