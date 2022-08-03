/*
 * Copyright (C) 2020 HAW Hamburg <michel.rottleuthner@haw-hamburg.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */
/**
 * @ingroup     sys_gclk
 * @brief       Interface for a generic gate implementation for the gclk module
 *
 * @{
 *
 * @file
 * @brief       Generic implementation of a clock gate for the gclock module
 *
 * @todo: Open considerations on the gate functionality:
 *        (A): Add the enable reg and mask definitions (i.e. gate data) as members to the controlled (parent) clock node types.
 *             - provide separate "gateable" variants of scaler/mux ?
 *             - add property to indicate "gateable" -> the enable register definition can already go to tiny regref descriptor
 *        (B): Add a single composite struct pointer holding the gate data to the controlled clock node type.
 *             - effectively this would be a simple way to provide a "gateable" variant of whatever clock...
 *        (C): (B) + also move the switch (enable/disable) ptr to the referenced composite struct
 *             Essentially this would move the "gate-feature" to a separete type that can be referenced with a single pointer.
 *             + Probably saves storage (e.g. unused enable/disable ops pointers in parent clock, and all other unused ops
 *               of this type itself)
 *             + The type could still be used for clocks that are not gateable, with only wasting a single ptr.
 *        (D): For now there is no get_parent implementation -> if parent reference is moved from gclk_t base type
 *             to the specific type it would make sense to add a get_parent impl. (more consistent?)
 *             would also make sense because all of the dynamic.parent clock types waste this pointer effectively
 *
 * @author      Michel Rottleuthner <michel.rottleuthner@haw-hamburg.de>
 */
#ifndef GCLK_GENERIC_GATE_H
#define GCLK_GENERIC_GATE_H

#include <stdbool.h>
#include <stdint.h>

#include "kernel_defines.h"
#include "gclk.h"
#include "bitarithm.h"

#ifdef __cplusplus
extern "C" {
#endif

/* The basic gate uses the generic basic clock type directly */
typedef gclk_basic_clock_t gclk_generic_gate_t;

/**
 * @brief returns the pointer to the specific clock instance that holds the given generic clock base
 */
static inline gclk_generic_gate_t *to_gclk_generic_gate_t(const gclk_t *clk)
{
    return gclk_to_basic_clock_t(clk);
}

/**
 * @brief Generic implementation of the gclk_ops_t.enable interface.
 *
 * It is meant to be used for hardware that can be modelled with the generic gate types gclk_generic_gate_t or
 * gclk_generic_src_gate_t.
 */
bool gclk_generic_gate_enable(const gclk_t *clk, gclk_enable_option_t opt) ;

/**
 * @brief Implementation of the is_enabled function for the gate_ops interface.
 *
 * It is meant to be used for hardware that can be modelled with the generic gate types gclk_generic_gate_t or
 * gclk_generic_src_gate_t.
 */
bool gclk_generic_gate_ed_rdy_reg_is_enabled(const gclk_t *clk);

/**
 * @brief Implementation of the enable function for the gate_ops interface.
 *
 * It is meant to be used for hardware that can be modelled with the generic gate types gclk_generic_gate_t or
 * gclk_generic_src_gate_t.
 */
void gclk_generic_gate_ed_rdy_reg_enable(const gclk_t *clk, bool on);

/**
 * @brief Implementation of gclk_ops_t.get_freq interface for pure clock gates.
 *
 * It is meant to be used for hardware that can be modelled with the generic gate type gclk_generic_src_gate_t.
 * It will simply return the fixed frequency or zero depending on the enable state indicated by the HW.
 */
unsigned long gclk_generic_src_gate_get_freq(const gclk_t *clk);

/**
 * @brief Implementation of gclk_ops_t.get_freq interface for gateable sources.
 *
 * It is meant to be used for hardware that can be modelled with the generic gate type gclk_generic_src_gate_t.
 * It will simply return the parent frequency or zero depending on the enable state indicated by the HW.
 */
unsigned long gclk_generic_gate_get_freq(const gclk_t *clk);

/**
 * @brief Implementation of gclk_ops_t.check_freq interface for gateable sources.
 *
 * It is meant to be used for hardware that can be modelled with the generic gate type gclk_generic_src_gate_t.
 * As check_freq is meant for apriori evaluation of different frequency configurations it is not absolutely needed or
 * semantically important for a pure gate (or a gateable source in this case) as the only options available will always
 * be "fixed internal frequency" or "zero". It's only real purpose is to make handling of different clock base-types
 * transparent without special handling in this case.
 */
unsigned long gclk_generic_gate_src_check_freq(const gclk_t *clk,
                                               clk_topology_entry_t *input_topology,
                                               uint32_t topology_len, uint32_t hz,
                                               uint32_t flags);

/**
 * @brief Implementation of gclk_ops_t.check_freq interface for pure clock gates.
 *
 * It is meant to be used for hardware that can be modelled with the generic gate type gclk_generic_src_gate_t.
 * As check_freq is meant for apriori evaluation of different frequency configurations it is not absolutely needed or
 * semantically important for a pure gate (or a gateable source in this case) as the only options available will always
 * be "unmodified parent frequency" or "zero". It's only real purpose is to make handling of different clock base-types
 * transparent without special handling in this case.
 */
unsigned long gclk_generic_gate_check_freq(const gclk_t *clk,
                                           clk_topology_entry_t *input_topology,
                                           uint32_t topology_len, uint32_t hz,
                                           uint32_t flags);

/* Implementation of only the gclk_gate_ops interface for a basic gate that is interfaced
 * via a read/write register that uses a single bit to control enable/disable.
 * Optionaly it also uses a ready register to indicate finishing of clock startup.
 * @note The register access MUST be encoded in a way understandable by the
 * generic mux implementation (regref). If that is not possible on a specific platform,
 * custom ops must be defined and used instead. */
extern const gclk_op_t gclk_plain_gate_ops[];

#ifdef __cplusplus
}
#endif

#endif /* GCLK_GENERIC_GATE_H */

/**
 * @}
 */
