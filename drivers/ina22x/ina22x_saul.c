/*
 * Copyright (C) 2018 HAW-Hamburg
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_ina22x
 * @{
 *
 * @file
 * @brief       SAUL adaption for INA22X device
 *
 * @author      Michel Rottleuthner <michel.rottleuthner@haw-hamburg.de>
 *
 * @}
 */

#include "saul.h"
#include "ina22x.h"

static int read_bus_voltage(const void *dev, phydat_t *res)
{
    uint32_t uv;
    ina22x_read_bus_voltage((const ina22x_t*)dev, &uv);

    res->val[0] = uv / 1000;
    res->unit = UNIT_V;
    res->scale = -3;
    return 1;
}

static int read_current(const void *dev, phydat_t *res)
{
    int32_t ua;
    ina22x_read_current((const ina22x_t*)dev, &ua);

    printf("current: %ld uA\n", ua);

    res->val[0] = (ua + (ua < 0 ? -50000 : 50000)) / 100000;
    res->unit = UNIT_A;
    res->scale = -5;
    return 1;
}

static int read_power(const void *dev, phydat_t *res)
{
    uint32_t uw;
    ina22x_read_power((const ina22x_t*)dev, &uw);

    printf("power: %lu uW\n", uw);

    res->val[0] = (uw + 500000) / 1000000;
    res->unit = UNIT_W;
    res->scale = -6;
    return 1;
}

const saul_driver_t ina22x_voltage_saul_driver = {
    .read = read_bus_voltage,
    .write = saul_notsup,
    .type = SAUL_SENSE_VOLTAGE,
};

const saul_driver_t ina22x_current_saul_driver = {
    .read = read_current,
    .write = saul_notsup,
    .type = SAUL_SENSE_CURRENT,
};

const saul_driver_t ina22x_power_saul_driver = {
    .read = read_power,
    .write = saul_notsup,
    .type = SAUL_SENSE_POWER,
};
