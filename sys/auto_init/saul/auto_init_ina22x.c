/*
 * Copyright (C) 2018 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 *
 */

/*
 * @ingroup     sys_auto_init_saul
 * @{
 *
 * @file
 * @brief       Auto initialization for INA22x bidirectional shunt monitor
 *
 * @author      Michel Rottleuthner <michel.rottleuthner@haw-hamburg.de>
 *
 * @}
 */

#ifdef MODULE_INA22X

#include "log.h"
#include "saul_reg.h"
#include "ina22x_params.h"
#include "ina22x.h"

/**
 * @brief   Define the number of configured sensors
 */
#define INA22X_NUM     (sizeof(ina22x_params) / sizeof(ina22x_params[0]))

/**
 * @brief   Allocate memory for the device descriptors
 */
static ina22x_t ina22x_devs[INA22X_NUM];

/**
 * @brief   Memory for the SAUL registry entries
 */
static saul_reg_t saul_entries[INA22X_NUM * 3];

/**
 * @brief   Define the number of saul info entries
 */
#define INA22X_INFO_NUM (sizeof(ina22x_saul_info) / sizeof(ina22x_saul_info[0]))

/**
 * @name    Import SAUL endpoints
 * @{
 */
extern const saul_driver_t ina22x_voltage_saul_driver;
extern const saul_driver_t ina22x_current_saul_driver;
extern const saul_driver_t ina22x_power_saul_driver;
/** @} */

void auto_init_ina22x(void)
{
    assert(INA22X_INFO_NUM == INA22X_NUM);

    for (unsigned int i = 0; i < INA22X_NUM; i++) {
        LOG_DEBUG("[auto_init_saul] initializing in22x #%u\n", i);

        if (i2c_init_master(ina22x_params[i].i2c, I2C_SPEED_NORMAL) == 0) {
            printf("i2c_init_master [OK]\n");
        }

        if (ina22x_init(&ina22x_devs[i], &ina22x_params[i]) != INA22X_OK) {
            LOG_ERROR("[auto_init_saul] error initializing ina22x #%u\n", i);
            continue;
        }

        saul_entries[(i * 3)].dev = &(ina22x_devs[i]);
        saul_entries[(i * 3)].name = ina22x_saul_info[i].name;
        saul_entries[(i * 3)].driver = &ina22x_voltage_saul_driver;
        saul_entries[(i * 3) + 1].dev = &(ina22x_devs[i]);
        saul_entries[(i * 3) + 1].name = ina22x_saul_info[i].name;
        saul_entries[(i * 3) + 1].driver = &ina22x_current_saul_driver;
        saul_entries[(i * 3) + 2].dev = &(ina22x_devs[i]);
        saul_entries[(i * 3) + 2].name = ina22x_saul_info[i].name;
        saul_entries[(i * 3) + 2].driver = &ina22x_power_saul_driver;
        saul_reg_add(&(saul_entries[(i * 3)]));
        saul_reg_add(&(saul_entries[(i * 3) + 1]));
        saul_reg_add(&(saul_entries[(i * 3) + 2]));
    }
}

#else
typedef int dont_be_pedantic;
#endif /* MODULE_INA22X */
