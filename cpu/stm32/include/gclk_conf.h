/*
 * Copyright (C) 2021 HAW Hamburg <michel.rottleuthner@haw-hamburg.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */
/**
 * @ingroup         cpu_stm32_common
 * @{
 *
 * @file
 * @brief           STM32 specific configuration definitions for the generic clock configuration module
 *
 *
 * @author          Michel Rottleuthner <michel.rottleuthner@haw-hamburg.de>
*/
#ifndef GCLK_CONF_H
#define GCLK_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

/* Static number ob bits needed to encode a config register index
 * preferrably this should be max 4 to save as much space as possible in the regref field
 * @TODO: derive this from the register ID enum declaration? */
#define GCLK_CONF_REG_IDX_BITWIDTH (4) 

/* Static number of clock instances */
#define GCLK_NUM_OF_CLOCKS (56)

#ifdef __cplusplus
}
#endif

#endif /* GCLK_CONF_H */

/**
 * @}
 */
