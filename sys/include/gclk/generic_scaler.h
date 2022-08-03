/*
 * Copyright (C) 2020 HAW Hamburg <michel.rottleuthner@haw-hamburg.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */
/**
 * @ingroup     sys_gclk
 * @brief       Interface for a generic scaler implementation for the gclk module
 *
 * @{
 *
 * @file
 * @brief       Generic implementation of a clock scaler for the gclock module
 *
 * @todo        Open considerations for the scaler implementation:
 *              - currently tiny regref can only hold enable, ready and conf reg/mask definitions
 *                where conf is used by muxes for parent config but scaler uses it for scale config
 *                -> that means regref is not enough to hold all registers for scaling AND parent config, so a mux
 *                   with flexible parents is not possible. That either required more overhead to store regref
 *                   or overhead for separate instances.
 *                   An example where a muxable-scaler would be beneficial the PLL-pre divider with its mux
 *                   (/M->{MSI|HSI|HSE}) or the MCO divider.
 *
 * @author      Michel Rottleuthner <michel.rottleuthner@haw-hamburg.de>
 */
#ifndef GCLK_GENERIC_SCALER_H
#define GCLK_GENERIC_SCALER_H

#include <stdbool.h>
#include <stdint.h>

#include "kernel_defines.h"
#include "gclk.h"
#include "bitarithm.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct gclk_scaler_regs {
    volatile uint32_t *enable_reg; /* register responsible to enable/disable this clock */
    volatile uint32_t *ready_reg;
    volatile uint32_t *scaler_reg;
} gclk_scaler_regs_t;

/**
 * @brief   This models a scaler that changes the frequency of a single input clock.
 *
 * @detial  It can be connected behind any other gclk_t instance.
 **/

/* The basic scaler uses the generic basic clock type directly */
typedef gclk_basic_clock_t gclk_clk_scaler_ll_t; 

/**
 * @brief returns the pointer to the specific clock instance that holds the given generic clock base
 */
static inline gclk_clk_scaler_ll_t *to_gclk_clk_scaler_ll_t(const gclk_t *clk)
{
    return gclk_to_basic_clock_t(clk);
}

/**
 * @brief Generic implementation of the gclk_ops_t.enable interface.
 *
 * It is meant to be used for hardware that can be modelled with the generic scaler type gclk_clk_scaler_ll_t.
 * For a scaler, this function is optional because scalers may not incorporate a gate to disable the output.
 * If the scaler has it's own gate (one that only affects it's output clock) it should be preferred to directly include it
 * with the scaler instance declaration to avoid overhead of a separate gate instance.
 *
 * If this function is used, the scaler instance must be initialized with values for the enable register and its bit.
 * Optionally the ready register and mask definitions are used to wait for eabling/disabling to be finished.
 */
bool gclk_generic_scaler_enable(const gclk_t *clk, gclk_enable_option_t opt);

/**
 * @brief Generic implementation of the gclk_ops_t.get_freq interface.
 *
 * It is meant to be used for hardware that can be modelled with the generic scaler type gclk_clk_scaler_ll_t.
 */
unsigned long gclk_generic_scaler_get_freq(const gclk_t *ll);

/**
 * @brief Generic implementation of the gclk_ops_t.set_freq interface.
 *
 * It is meant to be used for hardware that can be modelled with the generic scaler type gclk_clk_scaler_ll_t.
 */
unsigned long gclk_generic_scaler_set_freq(const gclk_t *ll, uint32_t hz, uint32_t parent_hz);

/**
 * @brief Generic implementation of the gclk_ops_t.check_freq interface.
 *
 * It is meant to be used for hardware that can be modelled with the generic scaler type gclk_clk_scaler_ll_t.
 */
unsigned long gclk_generic_scaler_check_freq(const gclk_t *ll,
                                                         clk_topology_entry_t *input_topology,
                                                         uint32_t topology_len, uint32_t hz,
                                                         uint32_t flags);

/**
 * @brief Get currently configured factor from the hardware
 *
 * @param clk  The clock to get the factor of.
 * @return     The currently configured factor as numerical value.
 */
unsigned int gclk_generic_scaler_get_factor(const gclk_t *clk);

/**
 * @brief Configure new factor by writing to hardware
 *
 * @param clk     The clock that the factor will be changed of.
 * @param factor  The new factor as numerical value.
 * @pre   @factor must be a valid value for clk.
 *
 */
void gclk_generic_scaler_set_factor(const gclk_t *clk, unsigned int factor);

unsigned int gclk_generic_fixed_scaler_get_factor(const gclk_t *clk);

/* A driver that only implements the scale_ops interface for a scaler that is interfaced
 * via a bitfield in a read/write register that exposes the current selection and
 * also allows changing it. */
extern const gclk_op_t gclk_plain_scaler_ops[];

/* A driver that only implements the get_factor opration of the scale_ops interface to
 * model a read only fixed factor scaler */
extern const gclk_op_t gclk_plain_fixed_scaler_ops[];

#define GCLK_PLAIN_FIXED_SCALER_STATIC_INIT(NAME,FACTOR)\
.base.separated_ops  = gclk_plain_fixed_scaler_ops,\
.base.flags.scalable = 1,\
.base.name           = #NAME,\
.base.factor_mapping.fixed_factor = FACTOR,\
.base.flags.conf_cnt = 1,\
.base.factor_map_op = gclk_map_func_fixed_factor

#ifdef __cplusplus
}
#endif

#endif /* GCLK_GENERIC_SCALER_H */

/**
 * @}
 */
