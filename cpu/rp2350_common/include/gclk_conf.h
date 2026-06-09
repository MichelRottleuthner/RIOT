/*
 * Copyright (C) 2026 HAW Hamburg <michel.rottleuthner@haw-hamburg.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */
/**
 * @ingroup      cpu_rp2350
 * @{
 *
 * @file
 * @brief        rp2350 configuration definitions for the generic clock configuration module
 *
 *
 * @author       Michel Rottleuthner <michel.rottleuthner@haw-hamburg.de>
*/
#ifndef GCLK_CONF_H
#define GCLK_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

/* Static number ob bits needed to encode a config register index
 * preferrably this should be max 4 to save as much space as possible in the regref field
 * on this platform we have >16 registers so we need at least 5 bits
 * */
#define GCLK_CONF_REG_IDX_BITWIDTH (5)

/* Static number of clock instances */
#define GCLK_NUM_OF_CLOCKS (26)

typedef enum {
    GCLK_NULL_REG = 0,
    GCLK_RP2350_PLL_SYS_CS,
    GCLK_RP2350_PLL_SYS_PWR,
    GCLK_RP2350_PLL_SYS_FBDIV_INT,
    GCLK_RP2350_PLL_SYS_PRIM,
    GCLK_RP2350_CLK_SYS_CTRL,
    GCLK_RP2350_CLK_SYS_DIV,
    GCLK_RP2350_CLK_REF_CTRL,
    GCLK_RP2350_PLL_USB_CS,
    GCLK_RP2350_PLL_USB_FBDIV_INT,
    GCLK_RP2350_PLL_USB_PRIM,
    GCLK_RP2350_PLL_USB_PWR,
    GCLK_RP2350_CLK_PERI_CTRL,
    GCLK_RP2350_CLK_PERI_DIV,
    GCLK_RP2350_CLK_GPOUT0_CTRL,
    GCLK_RP2350_CLK_GPOUT0_DIV,
    GCLK_RP2350_ROSC_CTRL,
    GCLK_RP2350__LAST_CONF_REG_IDX,
    /* @todo: add unused entry at the end to derive count? */
} gclk_conf_reg_id_t;

#ifdef __cplusplus
}
#endif

#endif /* GCLK_CONF_H */

/**
 * @}
 */
