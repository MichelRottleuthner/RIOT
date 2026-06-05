/*
 * Copyright (C) 2020 HAW Hamburg <michel.rottleuthner@haw-hamburg.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */
/**
 * @defgroup    drivers_periph_core_voltage Core Voltage
 * @ingroup     drivers_periph
 * @brief       Provides a generic interface to control MCU-internal voltage scaling
 *
 * This module is intended to be used for configuration of the MCU's internal core voltage.
 * Most modern MCUs have an option to control the internal core voltage to balance between energy consumption and
 * performance to optimize for specific use cases.
 * Available configurations options can strongly depend on paramters and external conditions like configuration of the
 * core frequency, flash configuration or external supply voltge.
 * Tracking and controling requirements and dependencies between all of these variables can become very complex.
 * Therefore, this module itself does *NOT* check if specific configurations are valid or stay in conflict with any of
 * those requirements.
 * The developer or an external module needs to take care a requested configuration is valid under the given
 * circumstances.
 * Together witht the gclock module it can be used for fully automatic voltage and frequency scaling.
 *
 * @{
 *
 * @file
 * @brief       Generic CPU voltage scaling interface definition
 *
 * @author      Michel Rottleuthner <michel.rottleuthner@haw-hamburg.de>
 */
#ifndef CORE_VOLTAGE_H
#define CORE_VOLTAGE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

int core_voltage_init(void);

/**
 * @brief Set the core voltage
 *
 * @param voltage_idx  Unique index that defines the requested voltage.
                       The value and its semantics may be platform specific but must be normalized
 *                     to a contiguous range of 0 to @core_voltage_get_cnt() - 1. It is recommended
 *                     to use a mapping that is easy to convert to respective register values.
 */
void core_voltage_set(unsigned int voltage_idx);

/**
 * @brief get the current core voltage
 *
 * @return  The configuration index that identifies the  current core voltage
 */
int core_voltage_get(void);

/**
 * @brief Convert a voltage configuration index to a voltage value in mV
 *
 * @param   The configuration index that identifies the core voltage
 * @return  The voltage in mV
 */
unsigned int core_voltage_idx2mv(unsigned voltage_idx);

/**
 * @brief get number of supported voltage ranges
 *
 * @return voltage range count
 */
unsigned int core_voltage_cnt(void);

#endif /* CORE_VOLTAGE_H */

/**
 * @}
 */
