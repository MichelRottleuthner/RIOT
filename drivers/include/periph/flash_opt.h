/*
 * Copyright (C) 2021 HAW Hamburg <michel.rottleuthner@haw-hamburg.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */
/**
 * @defgroup    drivers_periph_flash_opt Flash Option Driver
 * @ingroup     drivers_periph
 * @brief       Interface to basic flash configuration options
 *
 * This module is intended to be used for configuration of low-level
 * flash memory options such as wait states.
 *
 * @{
 *
 * @file
 * @brief       Flash option interface definition
 *
 * @author      Michel Rottleuthner <michel.rottleuthner@haw-hamburg.de>
 */
#ifndef FLASH_OPT_H
#define FLASH_OPT_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Set flash wait states
 *
 * Flash memory on fast MCUs can often not be read fast enough to read data
 * on every clock cycle. In those sutuations wait states must be inserted to
 * effectively slow down the flash access.
 *
 * @param ws  Number of flash wait states.
 */
void flash_opt_set_wait_states(unsigned int ws);

/**
 * @brief get the currently configured number of flash waitstates.
 *
 * @return  Number of flash wait states.
 */
unsigned int flash_opt_get_wait_states(void);

/**
 * @brief get the maximum number of flash wait states.
 *
 * @note  This value represents the worst case flash wait states and thus
 *        can be used as a safe default value for any frequency configuration.
 *
 * @return  Number of flash wait states.
 */
unsigned int flash_opt_get_max_wait_states(void);

#endif /* FLASH_OPT_H */

/**
 * @}
 */
