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
 * @brief       EFM32PG1B specific declarations for the gclk module
 *
 * @author      Michel Rottleuthner <michel.rottleuthner@haw-hamburg.de>
 * @}
 */
#include <stdint.h>
#include <stddef.h>

#include "cpu.h" /* pulls in the RCC register definitions etc. from the relevant vendor header */
/* The actual file we need in here is this one: */
// #include "cpu/efm32/families/efm32pg1b/include/vendor/efm32pg1b_cmu.h"

#include "gclk.h"

/* compatibility taken from stmclock for now
 * since stmclock solves a similar issue this should be a good baseline for testing and comparison.
 * For now this is only tested on STM32L476 */
#if defined(CPU_MODEL_EFM32PG12B500F1024GL125)
#else
#error gclk not supportet on this chip!
#endif /* defined(CPU_FAM_EFM32PG1B) */

/* @todo: declare this externally (board specific configuration header + single include in here + getters ?)*/
uint32_t volatile * const conf_regs[] = {
    [GCLK_NULL_REG]               = NULL,
    [GCLK_EFM32_CMU_HFPRESC]      = &CMU->HFPRESC,
    [GCLK_EFM32_CMU_ADCCTRL]      = &CMU->ADCCTRL,
    [GCLK_EFM32_ADC0_CTRL]        = &ADC0->CTRL,
    [GCLK_EFM32_CMU_OSCENCMD]     = &CMU->OSCENCMD,
    [GCLK_EFM32_CMU_STATUS]       = (uint32_t volatile * const)&CMU->STATUS, /* this register is defined as read only */
    [GCLK_EFM32_CMU_DBGCLKSEL]    = &CMU->DBGCLKSEL,
    [GCLK_EFM32_CMU_CTRL]         = &CMU->CTRL,
    [GCLK_EFM32_CMU_HFPERPRESC]   = &CMU->HFPERPRESC,
    [GCLK_EFM32_CMU_HFPERCLKEN0]  = &CMU->HFPERCLKEN0,
    [GCLK_EFM32_CMU_AUXHFRCOCTRL] = &CMU->AUXHFRCOCTRL,
    [GCLK_EFM32_CMU_HFRCOCTRL]    = &CMU->HFRCOCTRL,
    [GCLK_EFM32_CMU_HFCLKSTATUS]  = (uint32_t volatile * const)&CMU->HFCLKSTATUS,
    [GCLK_EFM32_CMU_HFCLKSEL]     = &CMU->HFCLKSEL,
    [GCLK_EFM32_CMU_HFEXPPRESC]   = &CMU->HFEXPPRESC,
    [GCLK_EFM32_CMU_HFCOREPRESC]  = &CMU->HFCOREPRESC,
    [GCLK_EFM32_CMU_HFBUSCLKEN0]  = &CMU->HFBUSCLKEN0,
    [GCLK_EFM32_CMU_LFACLKSEL]    = &CMU->LFACLKSEL,
    [GCLK_EFM32_CMU_LFBCLKSEL]    = &CMU->LFBCLKSEL,
    [GCLK_EFM32_CMU_LFECLKSEL]    = &CMU->LFECLKSEL,
    [GCLK_EFM32_WDOG_CTRL]        = &WDOG0->CTRL,
    [GCLK_EFM32_CMU_DPLLCTRL]     = &CMU->DPLLCTRL,
    [GCLK_EFM32_CMU_DPLLCTRL1]    = &CMU->DPLLCTRL1,
};

#include <stdio.h>
#include "gclk.h"

int gclk_efm32_init_clock_tree(void) {
    /* This is only here to generate a compilation error if the register lookup array
     * is out of sync with the enum based register IDs */
    for (unsigned i = GCLK_NULL_REG; i < GCLK_EFM32__LAST_CONF_REG_IDX; i++) {
        printf("EFM32 gclk connf_regs[%d]: %p\n", i, conf_regs[i]);
    }
    return 0;
}
