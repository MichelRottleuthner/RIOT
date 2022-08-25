/*
 * Copyright (C) 2020 HAW Hamburg
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     sys_gclk
 * @{
 *
 * @file
 * @brief       Implementation of a generic gate backend for the generic clock configuration module gclk
 *
 * @author      Michel Rottleuthner <michel.rottleuthner@haw-hamburg.de>
 * @}
 */
#include "gclk.h"
#include "gclk/generic_gate.h"
#define LOG_LEVEL LOG_NONE
#include "log.h"

void gclk_generic_gate_ed_rdy_reg_enable(const gclk_t *clk, bool on) {
    gclk_generic_gate_t *gate = to_gclk_generic_gate_t(clk);
    gclk_reg_ref_t regref = gate->regref;

    if (on) {
        LOG_DEBUG("%s: enabling %s...\n", __FUNCTION__, gclk_get_name(clk));
    } else {
        LOG_DEBUG("%s: disabling %s...\n", __FUNCTION__, gclk_get_name(clk));
    }

    /* only access a valid register ( always-on gate could model a fixed source) */
    if (gclk_regref2enable_reg(regref)) {
        if (on) {
            gclk_reg_util_set_mask(gclk_regref2enable_reg(regref), gclk_regref2enable_mask(regref));
        } else {
            gclk_reg_util_clear_mask(gclk_regref2enable_reg(regref), gclk_regref2enable_mask(regref));
        }
    }

    if (gclk_regref2ready_reg(regref)) {/* only wait for a ready flag if this register is defined */
        if (on) {
            while (!(*(gclk_regref2ready_reg(regref)) & gclk_regref2ready_mask(regref))) {}
        } else {
            while (*(gclk_regref2ready_reg(regref)) & gclk_regref2ready_mask(regref)) {}
        }
    }
}

bool gclk_generic_gate_ed_rdy_reg_is_enabled(const gclk_t *clk) {
    gclk_generic_gate_t *gate = to_gclk_generic_gate_t(clk);
    gclk_reg_ref_t regref = gate->regref;
    
    /* if no enable register is defined, always assume the gate was previously enabled */
    if (gclk_regref2enable_reg(regref)) {
        return *gclk_regref2enable_reg(regref) & gclk_regref2enable_mask(regref);
    } else {
        return true;
    }
}

unsigned long gclk_generic_gate_get_freq(const gclk_t *clk)
{
    /* if this clock is enabled (not gated) it just passes on the frequency of its parent */
    if (gclk_is_enabled(clk)) {
        if (gclk_is_source(clk)) {
            return clk->fixed_input_freq;
        }
        return gclk_get_current_freq(clk->fixed_parent);
    }
    return 0;
}

const gclk_op_t gclk_plain_gate_ops[] = {
  { .gate_ops = { .is_enabled = gclk_generic_gate_ed_rdy_reg_is_enabled,
                  .enable     = gclk_generic_gate_ed_rdy_reg_enable,}},
};
