/*
 * Copyright (C) 2020 HAW Hamburg <michel.rottleuthner@haw-hamburg.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */
/**
 * @ingroup     sys_gclk
 * @brief       Interface for a generic mux implementation for the gclk module
 *
 * @{
 *
 * @file
 * @brief       Generic implementation of a clock mux for the gclock module
 *
 * @author      Michel Rottleuthner <michel.rottleuthner@haw-hamburg.de>
 */
#ifndef GCLK_GENERIC_MUX_H
#define GCLK_GENERIC_MUX_H

#include <stdbool.h>
#include <stdint.h>

#include "kernel_defines.h"
#include "gclk.h"
#include "bitarithm.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief The generic mux uses the generic basic clock type directly.
 */
typedef gclk_basic_clock_t gclk_mux_ll_t;

/**
 * @brief returns the pointer to the specific clock instance that holds the given generic clock base
 */
static inline gclk_mux_ll_t *to_gclk_mux_ll_t(const gclk_t *clk)
{
    return gclk_to_basic_clock_t(clk);
}

/**
 * @brief Get the current parent of the clock.
 *
 * @param[in] clk   The clock instance to get the parent of.
 *
 * @return The pointer to the clock instance that is currently the parent of @p clk.
 */
const gclk_t* gclk_plain_mux_get_parent(const gclk_t *clk);

/**
 * @brief Set the parent of a clock via the index.
 *
 * @param[in] clk   The clock instance to set the parent of.
 * @param[in] idx   The index that refers to a parent (specific to @p clk).
 */
void gclk_plain_mux_set_parent(const gclk_t *clk, unsigned int idx);

/**
 * @brief Implementation of only the @ref gclk_mux_ops_t interface.
 *
 * Used for mux instances that are interfaced via a bitfield in a read/write
 * register that exposes the current selection while also allowing to change
 * it via that bitfield.
 *
 * @note The register access MUST be encoded in a way understandable by the
 * generic mux implementation (regref). If that is not possible on a specific platform,
 * custom ops must be used instead.
 */
extern const gclk_op_t gclk_plain_mux_ops[];

/**
 * @brief Static initialization helper for plain mux instances.
 *
 * Meant to be used for muxes that are compatible to the
 * @ref gclk_plain_mux_ops implementation.
 *
 * @param NAME           A unique char string name for the clock instance (without quotes!).
 * @param CONF_REG_ID    LUT ID of the config register that is used for selecting the parent.
 * @param CONF_REG_MASK  Mask of all bits responsible for the parent config setting.
 */
#define GCLK_PLAIN_MUX_STATIC_INIT(NAME,CONF_REG_ID,CONF_REG_MASK)\
.base.separated_ops  = gclk_plain_mux_ops,\
.base.flags.muxable  = 1,\
.base.name           = #NAME,\
.regref              = { .conf     = CONF_REG_ID,\
                         .conf_lsb = STATIC_BITMASK_LSB(CONF_REG_MASK),\
                         .conf_msb = STATIC_BITMASK_MSB(CONF_REG_MASK)}

#ifdef __cplusplus
}
#endif

#endif /* GCLK_GENERIC_MUX_H */

/**
 * @}
 */
