/*
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_sensors
 * @{
 * @file
 * @brief       Device driver for Texas Instruments INA22x Current and Power
 *              Monitors with I2C Compatible Interface (INA220/INA226).
 *
 * @author      Michel Rottleuthner <michel.rottleuthner@haw-hamburg.de>
 * @}
 */

#include <stdint.h>
#include <assert.h>
#include <string.h>

#include "ina22x.h"
#include "ina22x-regs.h"
#include "periph/gpio.h"
#include "periph/i2c.h"
#include "byteorder.h"

#define ENABLE_DEBUG    (0)
#include "debug.h"

int ina22x_read_bus_voltage(const ina22x_t *dev, uint32_t* uvolt)
{
    int16_t bus_voltage_reg;
    if (ina22x_read_bus_reg(dev, &bus_voltage_reg) == 0) {

        if(dev->model == INA226) {
            *uvolt = (bus_voltage_reg * INA226_BUS_VOLTAGE_LSB_UV);
        } else {
            *uvolt = (bus_voltage_reg * INA220_BUS_VOLTAGE_LSB_UV);
        }

        return INA22X_OK;
    }
    return INA22X_I2C_ERR;
}

int ina22x_read_shunt_voltage(const ina22x_t *dev, int32_t* nvolt)
{
    int16_t sht_voltage_reg;
    if (ina22x_read_shunt_reg(dev, &sht_voltage_reg) == 0) {

        if(dev->model == INA226) {
            *nvolt = (sht_voltage_reg * INA226_SHUNT_VOLTAGE_LSB_NV);
        } else {
            *nvolt = (sht_voltage_reg * INA220_SHUNT_VOLTAGE_LSB_NV);
        }

        return INA22X_OK;
    }
    return INA22X_I2C_ERR;
}

int ina22x_read_current(const ina22x_t *dev, int32_t* ua)
{
    int16_t current_reg;
    if (ina22x_read_current_reg(dev, &current_reg) == 0) {
        *ua = (current_reg * dev->current_lsb_na) / NANOS_PER_MICRO;
        return INA22X_OK;
    }
    return INA22X_I2C_ERR;
}

int ina22x_read_power(const ina22x_t *dev, uint32_t* uw)
{
    int16_t power_reg;
    if (ina22x_read_power_reg(dev, &power_reg) == 0) {

        if(dev->model == INA226) {
            *uw = (uint32_t)(((uint64_t)INA226_POWER_LSB_CURRENT_LSB_RATIO *
                             dev->current_lsb_na * power_reg)
                             / NANOS_PER_MICRO);
        } else {
            *uw = (uint32_t)(((uint64_t)INA220_POWER_LSB_CURRENT_LSB_RATIO *
                             dev->current_lsb_na * power_reg)
                             / NANOS_PER_MICRO);
        }

        return INA22X_OK;
    }
    return INA22X_I2C_ERR;
}

int ina22x_read_reg(const ina22x_t *dev, uint8_t reg, uint16_t *out)
{
    union {
        char c[2];
        uint16_t u16;
    } tmp = { .u16 = 0 };
    int status = 0;

    i2c_acquire(dev->i2c);
    status = i2c_read_regs(dev->i2c, dev->addr, reg, &tmp.c[0], 2);
    i2c_release(dev->i2c);

    if (status != 2) {
        return INA22X_I2C_ERR;
    }

    *out = ntohs(tmp.u16);
    return INA22X_OK;
}

int ina22x_write_reg(const ina22x_t *dev, uint8_t reg, uint16_t in)
{
    union {
        char c[2];
        uint16_t u16;
    } tmp = { .u16 = 0 };
    int status = 0;

    tmp.u16 = htons(in);

    i2c_acquire(dev->i2c);
    status = i2c_write_regs(dev->i2c, dev->addr, reg, &tmp.c[0], 2);
    i2c_release(dev->i2c);

    if (status != 2) {
        return INA22X_I2C_ERR;
    }

    return INA22X_OK;
}

int ina22x_init(ina22x_t *dev, const ina22x_params_t *params)
{
    assert(dev && params);

    dev->model = params->model;
    dev->i2c = params->i2c;
    dev->addr = params->addr;
    dev->config = params->config;
    dev->cal = params->cal;
    dev->current_lsb_na = params->current_lsb_na;

    //memcpy(dev, params, sizeof(ina22x_t));

    if( (ina22x_write_config_reg(dev, params->config) == INA22X_OK) &&
        (ina22x_write_calibration_reg(dev, params->cal) == INA22X_OK) ){
        return INA22X_OK;
    }

    return INA22X_I2C_ERR;
}

int ina226_activate_int(ina22x_t *dev, uint16_t me_config, gpio_t pin, gpio_cb_t callback)
{
    assert(dev->model == INA226);

    uint16_t me_reg_val;
    ina22x_read_reg(dev, INA226_REG_MASK_ENABLE, &me_reg_val);

    if (ina22x_write_reg(dev, INA226_REG_MASK_ENABLE, me_config) == INA22X_OK ) {
        if ( gpio_init_int(pin, GPIO_IN_PU, GPIO_FALLING, callback, dev) == 0) {
            return INA22X_OK;
        }
        return INA22X_PIN_ERR;
    }

    return INA22X_I2C_ERR;
}
