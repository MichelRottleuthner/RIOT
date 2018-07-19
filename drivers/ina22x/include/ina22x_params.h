/*
 * Copyright (C) 2018 HAW-Hamburg
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_isl29020
 *
 * @{
 * @file
 * @brief       Default configuration for INA22x devices
 *
 * @author      Michel Rottleuthner <michel.rottleuthner@haw-hamburg.de>
 */

#ifndef INA22X_PARAMS_H
#define INA22X_PARAMS_H

#include "board.h"
#include "ina22x.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   Set default configuration parameters
 * @{
 */
#ifndef INA22X_PARAM_I2C
#define INA22X_PARAM_I2C            I2C_DEV(0)
#endif
#ifndef INA22X_PARAM_ADDR
#define INA22X_PARAM_ADDR           (0x40)
#endif
#ifndef INA22X_PARAM_MODEL
#define INA22X_PARAM_MODEL          (INA226)
#endif
#ifndef INA22X_PARAM_ALERT_PIN
#define INA22X_PARAM_ALERT_PIN      (GPIO_PIN(0,0))
#endif
#ifndef INA22X_PARAM_CONFIG
#define INA22X_PARAM_CONFIG         (INA22X_MODE_CONTINUOUS_SHUNT_AND_BUS | \
                                     INA226_VBUSCT_588_US | \
                                     INA226_VSHCT_588_US | \
                                     INA226_AVG_1024)
#endif

#ifndef INA22X_PARAM_LSB_NA
#define INA22X_PARAM_LSB_NA         (100000)
#endif

#ifndef INA22X_PARAM_CAL
#define INA22X_PARAM_CAL            (2048)
#endif

#ifndef INA22X_SAUL_INFO
#define INA22X_SAUL_INFO            { .name = "ina22x" }
#endif

#define INA22X_PARAMS               { .i2c       = INA22X_PARAM_I2C, \
                                      .addr      = INA22X_PARAM_ADDR, \
                                      .model     = INA22X_PARAM_MODEL, \
                                      .config    = INA22X_PARAM_CONFIG, \
                                      .alert_pin = INA22X_PARAM_ALERT_PIN, \
                                      .cal       = INA22X_PARAM_CAL, \
                                      .current_lsb_na = INA22X_PARAM_LSB_NA }
/**@}*/

/**
 * @brief   INA22x configuration
 */
static const ina22x_params_t ina22x_params[] =
{
    INA22X_PARAMS
};

/**
 * @brief   INA22x meta information for the SAUL registry
 */
static const saul_reg_info_t ina22x_saul_info[] =
{
    INA22X_SAUL_INFO
};

#ifdef __cplusplus
}
#endif

#endif /* INA22X_PARAMS_H */
/** @} */
