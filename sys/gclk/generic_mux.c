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
 * @brief       Implementation of a generic mux backend for the generic clock configuration module gclk
 *
 * @author      Michel Rottleuthner <michel.rottleuthner@haw-hamburg.de>
 * @}
 */
#include "gclk.h"
#include "gclk/generic_mux.h"
#define LOG_LEVEL LOG_ERROR
#include "log.h"

static inline uint32_t _get_parent_conf_val(gclk_mux_ll_t *clk, uint32_t parent_idx) {
    /* if a generic factor mapping function is defined at the base type */
    if (clk->base.parent_map_op) {
        return gclk_idx2parentregval(&clk->base, parent_idx);
    }

    /* implicitly falls to this condition: if (clk->base.flags.reg_map_type == GCLK_CONF_LIST32)*/
    return parent_idx;
}

const gclk_t* gclk_plain_mux_get_parent(const gclk_t *clk) {
    gclk_mux_ll_t *mux = to_gclk_mux_ll_t(clk);

    uint32_t cur_config_reg_val = gclk_reg_util_read_masked(gclk_regref2conf_reg(mux->regref),
                                                            gclk_regref2conf_mask(mux->regref));

    LOG_DEBUG("%s: %s (config val) 0x%08lx\n", __FUNCTION__, gclk_get_name(clk), cur_config_reg_val);

    /* check could be replaced by a rule that requests this to be present if this get_parent op
     * is used */
    return gclk_regval2parent(clk, cur_config_reg_val);
}

void gclk_plain_mux_set_parent(const gclk_t *clk, unsigned int idx) {
    gclk_mux_ll_t *mux = to_gclk_mux_ll_t(clk);

    uint32_t regval = gclk_idx2parentregval(clk, idx);

    gclk_reg_util_write_masked(gclk_regref2conf_reg(mux->regref), gclk_regref2conf_mask(mux->regref),
                               regval);
}

const gclk_op_t gclk_plain_mux_ops[] = {
  { .mux_ops = { .get_parent = gclk_plain_mux_get_parent,
                 .set_parent = gclk_plain_mux_set_parent,}},
};
