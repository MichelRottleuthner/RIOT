/*
 * Copyright (C) 2021 HAW Hamburg <michel.rottleuthner@haw-hamburg.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */
/**
 * @ingroup         cpu_efm32_common
 * @{
 *
 * @file
 * @brief           EFM32 specific configuration definitions for the generic clock configuration module
 *
 *
 * @author          Michel Rottleuthner <michel.rottleuthner@haw-hamburg.de>
*/
#ifndef GCLK_CONF_H
#define GCLK_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

#ifdef GCLK_USE_TINY_REG_REF
#if defined(CPU_MODEL_EFM32PG12B500F1024GL125)
/* The number of regsiters involved in clock configuration is usually pretty small.
   We can exploit that to save some memory by only holding the configuration registers in one place and storing only a
   tiny index to it in each instance
   @todo: evaluate if we can link conditional availability between these values and instances to use the same knob for
          enabling/disabling availability/required state */
typedef enum {
    GCLK_NULL_REG = 0,
    GCLK_EFM32_CMU_HFPRESC,
    GCLK_EFM32_CMU_ADCCTRL,
    GCLK_EFM32_ADC0_CTRL,
    GCLK_EFM32_CMU_OSCENCMD,
    GCLK_EFM32_CMU_STATUS,
    GCLK_EFM32_CMU_DBGCLKSEL,
    GCLK_EFM32_CMU_CTRL,
    GCLK_EFM32_CMU_HFPERPRESC,
    GCLK_EFM32_CMU_HFPERCLKEN0,
    GCLK_EFM32_CMU_AUXHFRCOCTRL,
    GCLK_EFM32_CMU_HFRCOCTRL,
    GCLK_EFM32_CMU_HFCLKSTATUS,
    GCLK_EFM32_CMU_HFCLKSEL,
    GCLK_EFM32_CMU_HFEXPPRESC,
    GCLK_EFM32_CMU_HFCOREPRESC,
    GCLK_EFM32_CMU_HFBUSCLKEN0,
    GCLK_EFM32_CMU_LFACLKSEL,
    GCLK_EFM32_CMU_LFBCLKSEL,
    GCLK_EFM32_CMU_LFECLKSEL,
    GCLK_EFM32_WDOG_CTRL,
    GCLK_EFM32_CMU_DPLLCTRL,
    GCLK_EFM32_CMU_DPLLCTRL1,
    GCLK_EFM32__LAST_CONF_REG_IDX,
    /* @todo: add unused entry at the end to derive count? */
} gclk_conf_reg_id_t;

#else
#error "must specify gclk conf reg ids for this platform!"
#endif /* defined(CPU_MODEL_EFM32PG12B500F1024GL125) */
#endif /* GCLK_USE_TINY_REG_REF */

/* Static number ob bits needed to encode a config register index
 * preferrably this should be max 4 to save as much space as possible in the regref field
 * on this platform we have >16 registers so we need at least 5 bits
 * @TODO: derive this from the register ID enum declaration? */
#define GCLK_CONF_REG_IDX_BITWIDTH (5) 

/* Static number of clock instances */
#define GCLK_NUM_OF_CLOCKS         (55)

#ifdef __cplusplus
}
#endif

#endif /* GCLK_CONF_H */

/**
 * @}
 */
