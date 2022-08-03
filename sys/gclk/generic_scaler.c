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
 * @brief       Implementation of a generic scaler backend for the generic clock configuration module gclk
 *
 * @author      Michel Rottleuthner <michel.rottleuthner@haw-hamburg.de>
 * @}
 */
#include "gclk.h"
#include "gclk/generic_scaler.h"
#define LOG_LEVEL LOG_NONE
#include "log.h"

unsigned int gclk_generic_fixed_scaler_get_factor(const gclk_t *clk) {
    return clk->factor_mapping.fixed_factor;
}

unsigned int gclk_generic_scaler_get_factor(const gclk_t *clk) {
    gclk_clk_scaler_ll_t *scaler = to_gclk_clk_scaler_ll_t(clk);

    /* update only read if scaler config register is valid */
    if (gclk_regref2conf_reg(scaler->regref)) {

        uint32_t cur_conf_reg_val = gclk_reg_util_read_masked(gclk_regref2conf_reg(scaler->regref),
                                                              gclk_regref2conf_mask(scaler->regref));

        uint32_t factor = gclk_regval2factor(clk, cur_conf_reg_val);
        //printf("%s conf reg val: %lu (factor: %lu)\n", gclk_get_name(clk), cur_conf_reg_val, factor);
        return factor;
        //return _regval2factor(scaler, cur_conf_reg_val);
    }

    return 0;
}

void gclk_generic_scaler_set_factor(const gclk_t *clk, unsigned int factor) {
    gclk_clk_scaler_ll_t *scaler = to_gclk_clk_scaler_ll_t(clk);

    /* update only if we have a valid scaler config register */
    if (gclk_regref2conf_reg(scaler->regref)) {

        uint32_t scale_reg_val = gclk_factor2regval(clk, factor);
        //uint32_t scale_reg_val = _factor2regval(scaler, factor);

        gclk_reg_util_write_masked(gclk_regref2conf_reg(scaler->regref),
                                   gclk_regref2conf_mask(scaler->regref), scale_reg_val);
    }
}

const gclk_op_t gclk_plain_scaler_ops[] = {
      { .scale_ops = { .get_factor = gclk_generic_scaler_get_factor,
                       .set_factor = gclk_generic_scaler_set_factor,}},
};

const gclk_op_t gclk_plain_fixed_scaler_ops[] = {
  { .scale_ops = { .get_factor = gclk_generic_fixed_scaler_get_factor,
                   .set_factor = NULL,}},
};
