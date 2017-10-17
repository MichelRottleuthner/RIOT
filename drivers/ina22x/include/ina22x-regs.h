/*
 * Copyright (C) 2017 HAW-Hamburg
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     driver_ina22x
 * @{
 *
 * @file
 * @brief       Register definitions for Texas Instruments INA22x High-Side or Low-Side Measurement,
 *              Bi-Directional Current and Power Monitor with I2C Compatible Interface.
 *              Currently this driver supports INA220 and INA226
 *
 * @author      Michel Rottleuthner <michel.rottleuthner@haw-hamburg.de>
 */


#ifndef INA22X_REGS_H
#define INA22X_REGS_H


#ifdef __cplusplus
 extern "C" {
#endif

/**
 * @brief INA22x register addresses (common on INA220 and INA226)
 *
 * All registers in the INA22x are 16 bit wide and transmitted MSB first.
 */
typedef enum ina22x_reg {
    INA22X_REG_CONFIGURATION   = 0x00, /**< Configuration register (read/write) */
    INA22X_REG_SHUNT_VOLTAGE   = 0x01, /**< Shunt voltage register (read only) */
    INA22X_REG_BUS_VOLTAGE     = 0x02, /**< Bus voltage register (read only) */
    INA22X_REG_POWER           = 0x03, /**< Power register (read only) */
    INA22X_REG_CURRENT         = 0x04, /**< Current register (read only) */
    INA22X_REG_CALIBRATION     = 0x05, /**< Calibration register (read/write) */
} ina22x_reg_t;

#ifdef __cplusplus
}
#endif

#endif /* INA22X_REGS_H */
/** @} */
