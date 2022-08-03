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

/* The basic mux uses the generic basic clock type directly */
typedef gclk_basic_clock_t gclk_mux_ll_t;

/**
 * @brief returns the pointer to the specific clock instance that holds the given generic clock base
 */
static inline gclk_mux_ll_t *to_gclk_mux_ll_t(const gclk_t *clk)
{
    return gclk_to_basic_clock_t(clk);
}

/**
 * @brief Generic implementation of the gclk_ops_t.enable interface.
 *
 * It is meant to be used for hardware that can be modelled with the generic gate type gclk_mux_ll_t.
 * For a mux, this function is optional because many muxes don't incorporate a gate to disable the output.
 * If the mux has it's own gate (one that only affects it's output clock) it should be preferred to directly include it
 * with the mux instance declaration to avoid overhead of a separate gate instance.
 *
 * If this function is used, the mux instance must be initialized with values for the enable register and its bit.
 * Optionally the ready register and mask definitions are used to wait for eabling/disabling to be finished.
 */
bool gclk_generic_mux_enable(const gclk_t *clk, gclk_enable_option_t opt);

/**
 * @brief Generic implementation of the gclk_ops_t.set_parent interface.
 *
 * It is meant to be used for hardware that can be modelled with the generic mux type gclk_mux_ll_t.
 * This function is mandatory for any mux.
 *
 */
int gclk_generic_mux_set_parent(const gclk_t *clk, unsigned int index);

/**
 * @brief Generic implementation of the gclk_ops_t.get_parent interface.
 *
 * It is meant to be used for hardware that can be modelled with the generic mux type gclk_mux_ll_t.
 * This function is mandatory for any mux.
 *
 */
const gclk_t* gclk_generic_mux_get_parent(const gclk_t *clk, unsigned int idx);

/**
 * @brief Generic implementation of the gclk_ops_t.set_freq interface.
 *
 * It is meant to be used for hardware that can be modelled with the generic mux type gclk_mux_ll_t.
 * This function is optional for any mux.
 * @todo: decide if we prefere flexibility or strict semantics here.
 *        A pure mux only has the option to change it's parent to alter it's frequency. But that mixes topological
 *        changes with frequency configuration changes. (Exception: setting the parent to NULL *could* be considered
 *        a pure fequency change as no other topology is touched)
 *        Another possiblity would be to employ a "forward semantics" to the set_freq request so the next node that
 *        actually can change freq would perform an action. But that would probably need somerestirictions to only
 *        forward the request no other clock is involved (this instance is the only child/topology chain of the changed
 *        instance)
 *
 */
unsigned long gclk_generic_mux_set_freq(const gclk_t *clk, uint32_t hz, uint32_t parent_hz);

/**
 * @brief Generic implementation of the gclk_ops_t.get_freq interface.
 *
 * It is meant to be used for hardware that can be modelled with the generic mux type gclk_mux_ll_t.
 * This function is mandatory for any mux.
 * It returns the frequency of the currently selected clock.
 */
unsigned long gclk_generic_mux_get_freq(const gclk_t *clk);

/**
 * @brief Generic implementation of the gclk_ops_t.check_freq interface.
 *
 * It is meant to be used for hardware that can be modelled with the generic mux type gclk_mux_ll_t.
 * This function is mandatory for any mux.
 * This function does'not query or alter HW state and is meant for apriori evaluation of freq configurations.
 */
unsigned long gclk_generic_mux_check_freq(const gclk_t *clk,
                                                      clk_topology_entry_t *input_topology,
                                                      uint32_t topology_len, uint32_t hz,
                                                      uint32_t flags);

const gclk_t* gclk_plain_mux_get_parent(const gclk_t *clk);
void gclk_plain_mux_set_parent(const gclk_t *clk, unsigned int idx);

/* Implementation of only the scale_ops interface for a scaler that is interfaced
 * via a bitfield in a read/write register that exposes the current selection while
 * also allowing to change it via that bitfield.
 * @note The register access MUST be encoded in a way understandable by the
 * generic mux implementation (regref). If that is not possible on a specific platform,
 * custom ops must be used instead */
extern const gclk_op_t gclk_plain_mux_ops[];

/* This helper is meant for static initialization for plain mux instances that are compatible
 * with the backend provided by the generic mux implementation */
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
