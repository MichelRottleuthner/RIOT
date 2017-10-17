/*
 * Copyright (C) 2016
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup tests
 * @{
 *
 * @file
 * @brief       Test application for the INA22x sensor driver
 *              All references to the INA226 data sheet are related to the document revision SBOS547A
 *
 * @author      Michel Rottleuthner <michel.rottleuthner@haw-hamburg.de>
 *
 * @}
 */
#include <stdio.h>

#include "xtimer.h"
#include "ina22x.h"
#include "ina22x-regs.h"
#include "ina226-regs.h"
#include "phydat.h"
#include "debug.h"

#define ENABLE_DEBUG (0)

#define PRINT_MSG_QUEUE_SIZE (1)

static msg_t _print_msg_queue[PRINT_MSG_QUEUE_SIZE];
static char print_thread_stack[THREAD_STACKSIZE_DEFAULT];
static kernel_pid_t _print_thread_pid;

//--------------------------------- GENERAL CONFIGURATION PARAMETERS -----vvvvv
#if TEST_INA22X_MODEL == INA226
/* GPIO pin that is connected to the ina226 alert pin */
#define TEST_INA226_ALERT_PIN (GPIO_PIN(0,6)) //PIN PA06

/* This configuration enables continuous measurement of both shunt and bus voltage.
 * The adc conversion time for both is set to be 1.1 ms and the measurements are averaged over 256 individual samples -> so the interrupt will fire ~twice per second*/
#define TEST_INA22X_CONFIG (INA22X_MODE_CONTINUOUS_SHUNT_AND_BUS | INA226_VBUSCT_1_1_MS | INA226_VSHCT_1_1_MS | INA226_AVG_256)
//#define TEST_INA226_CONFIG (INA226_OP_MODE_CONTINUOUS_SHUNT_AND_BUS | INA226_VBUSCT_1_1_MS | INA226_VSHCT_1_1_MS | INA226_AVG_1)
//#define TEST_INA226_CONFIG (INA226_OP_MODE_CONTINUOUS_SHUNT_AND_BUS | INA226_VBUSCT_140_US | INA226_VSHCT_140_US | INA226_AVG_1)
//#define TEST_INA22X_CONFIG (INA22X_MODE_CONTINUOUS_SHUNT_ONLY | INA226_VSHCT_140_US | INA226_AVG_1)
#endif

/* Value of the used shunt resistor in milli ohm */
#define TEST_INA22X_SHUNT_RESISTOR_MILLI_OHM (750)

/* The maximum current that is expected to flow through the shunt resistor in microampere. See section 7.5 in INA226 data sheet */
#define TEST_INA226_MAX_EXPECTED_CURRENT_UA ((INA226_MAX_SHUNT_VOLTAGE_UV * MICROS_PER_MILLI) / TEST_INA22X_SHUNT_RESISTOR_MILLI_OHM)

/* The value of one LSB of the current register in nanoampere. See section 7.5 in INA226 data sheet */
#define TEST_INA226_CURRENT_LSB_NA ((TEST_INA226_MAX_EXPECTED_CURRENT_UA * NANOS_PER_MICRO) / INA22X_MAX_CURRENT_TO_LSB_RATIO)

/* Calculated value for the calibration register for use with a 0.75 Ohm shunt resistor. See section 7.5 in INA226 data sheet */
#define TEST_INA22X_CALIBRATION_VALUE ((uint16_t)(INA226_CAL_SCALE_FACTOR_INT / (TEST_INA226_CURRENT_LSB_NA * TEST_INA22X_SHUNT_RESISTOR_MILLI_OHM)))
//--------------------------------- GENERAL CONFIGURATION PARAMETERS -----^^^^^


//--------------------- REFERENCE MEASUREMENT CALIBRATION PARAMETERS -----vvvvv
/* Current [uA] that was measured with external ampere meter. This value was measured under the same conditions as when the INA226
 * (configured with the original calculated TEST_INA226_CALIBRATION_VALUE) reported TEST_INA226_CAL_REPORTED_CURRENT*/
#define TEST_INA226_CAL_AMM_MEAS_CURRENT (6593)

/* Current [uA] that was reported by INA226 corresponding to the value of TEST_INA226_CAL_AMM_MEAS_CURRENT above. */
#define TEST_INA226_CAL_REPORTED_CURRENT (7002)

/* This value is derived by the equation shown in INA226 datasheet section 7.5.2
 * Write this value to the calibration register when the above reference measurement values ('TEST_INA226_CAL_AMM_MEAS_CURRENT' and
 * 'TEST_INA226_CAL_REPORTED_CURRENT') are set to values that were measured in your specific setup. */
#define TEST_INA226_CORRECTED_FULL_SCALE_CAL ((uint16_t)((TEST_INA226_CALIBRATION_VALUE * TEST_INA226_CAL_AMM_MEAS_CURRENT) / TEST_INA226_CAL_REPORTED_CURRENT))

/* Uncomment this line if after performing reference measurements with your hardware setup
 * and updating 'TEST_INA226_CAL_AMM_MEAS_CURRENT' 'TEST_INA226_CAL_REPORTED_CURRENT' accordingly. */
//#define TEST_INA226_REF_MEASUREMENTS_DONE

#ifndef TEST_INA226_REF_MEASUREMENTS_DONE
#define TEST_INA22X_CALIBRATION_VALUE_TO_USE TEST_INA22X_CALIBRATION_VALUE
#else
#define TEST_INA22X_CALIBRATION_VALUE_TO_USE TEST_INA22X_CORRECTED_FULL_SCALE_CAL
#endif
//--------------------- REFERENCE MEASUREMENT CALIBRATION PARAMETERS -----^^^^^

/** callback that gets fired whenever a new measurement is available (the alert pin of the INA226 gets triggered). */
static void test_ina226_callback(void *arg);
void *measurement_print_thread(void *args);

volatile int measures = 0;

int main(void)
{
    ina22x_t ina22x_dev;
    static const ina22x_params_t ina22x_params = {
        .model  = TEST_INA22X_MODEL,
        .i2c    = TEST_INA22X_I2C,
        .addr   = TEST_INA22X_ADDR,
        .config = TEST_INA22X_CONFIG,
        .cal    = TEST_INA22X_CALIBRATION_VALUE_TO_USE,
    };

    #if TEST_INA22X_MODEL == INA226
    uint16_t ina_226_die_id;
    uint16_t ina_226_manufac_id;
    #endif

    xtimer_init();

    gpio_init(TEST_INA226_ALERT_PIN, GPIO_IN_PU);

    printf("--- RIOT: INA226 test application ---\n");

    _print_thread_pid = thread_create(print_thread_stack, sizeof(print_thread_stack),
                                          THREAD_PRIORITY_MAIN -1,
                                          THREAD_CREATE_STACKTEST, measurement_print_thread,
                                          NULL, "measurement print thread");

    printf("initializing I2C... ");
    if (i2c_init_master(ina22x_params.i2c, TEST_I2C_SPEED) == 0) {
        printf("i2c_init_master [OK]\n");
    }
    else {
        printf("i2c_init_master [FAILED]\n");
        return -1;
    }

    printf("Initializing INA22x sensor at I2C_%" PRIu16 ", address 0x%02" PRIx16 "... ",TEST_INA22X_I2C, TEST_INA22X_ADDR);
    if (ina22x_init(&ina22x_dev, &ina22x_params) == 0) {
        printf("ina226_init [OK]\n");
    }
    else {
        printf("ina22x_init [FAILED]\n");
        return -1;
    }

    #if TEST_INA22X_MODEL == INA226
    printf("enabling interrupt by alert pin... ");
    if (ina226_activate_int(&ina22x_dev, INA226_MASK_ENABLE_CNVR, TEST_INA226_ALERT_PIN, test_ina226_callback ) == 0) {
        printf("ina226_activate_int [OK]\n");
    }
    else {
        printf("ina226_activate_int [FAILED]\n");
        return -1;
    }

    printf("INA226 initialized successfully -> now waiting for interrupts...\n");
    fflush(stdout);

    uint16_t dummy;
    //reading the mask enable register is needed to re-enable interrupt generation by ina226
    if (ina226_read_mask_enable_reg(&ina22x_dev, &dummy) == 0) {
        DEBUG("ina226_read_mask_enable [OK]\n");
    }
    else {
        DEBUG("ina226_read_mask_enable [FAILED]\n");
    }

    #endif

    #if TEST_INA22X_MODEL == INA226
    printf("reading die-id of INA226 sensor... ");
    if (ina226_read_die_id_reg(&ina22x_dev, &ina_226_die_id) == 0) {
        printf("ina226_read_die_id [OK]: ina_226_die_id = 0x%0" PRIx16 "\n", ina_226_die_id);
    }
    else {
        printf("ina226_read_die_id [FAILED]\n");
        return -1;
    }

    printf("reading manufacturer-id of INA226 sensor... ");
    if (ina226_read_manufacturer_id_reg(&ina22x_dev, &ina_226_manufac_id) == 0) {
        printf("ina226_read_manufacturer_id [OK]: ina_226_manufac_id = 0x%0" PRIx16 "\n", ina_226_manufac_id);
    }
    else {
        printf("ina226_read_manufacturer_id [FAILED]\n");
        return -1;
    }
    #endif

    while(true){
        msg_t msg;
        msg.content.ptr = &ina22x_dev;
        msg_send(&msg, _print_thread_pid);
        xtimer_sleep(1);
        printf("measures: %d\n", measures);
    }
}

void *measurement_print_thread(void *args)
{
    msg_init_queue(_print_msg_queue, PRINT_MSG_QUEUE_SIZE);
    msg_t msg;

    //--- raw values of INA226 registers ---vvv
    uint16_t mask_enable_reg;
    int16_t bus_voltage_reg;
    int16_t  shunt_voltage_reg;
    int16_t  current_reg;
    int16_t  power_reg;
    //--- raw values of INA226 registers ---^^^

    //--- derived values for pretty printing afterwards ---vvv
    int32_t current_ua;
    int32_t power_uw;

    uint32_t bus_voltage_int_v;   //integer part of bus voltage [V]
    uint32_t bus_voltage_frac_mv; //fractional part of bus voltage [mV]

    int32_t shunt_voltage_int_mv;  //integer part of shunt voltage [mV]
    int32_t shunt_voltage_frac_uv; //fractional part of shunt voltage [uV]

    int32_t current_int_ma;  //integer part of current [mA]
    int32_t current_frac_ua; //fractional part of current [uA]

    int32_t power_int_mw;  //integer part of power [mW]
    int32_t power_frac_uw; //fractional part of power [uW]
    //--- derived values for pretty printing afterwards ---^^^

    uint32_t bus_voltage_uV;
    int32_t  sht_voltage_nV;


    while(true){
        //printf("waiting...");
        //fflush(stdout);
        msg_receive(&msg);
        //printf("got\n");
        //fflush(stdout);

        //start = xtimer_now();
        ina22x_t *dev = (ina22x_t*)msg.content.ptr;

        if (ina22x_read_bus_reg(dev, &bus_voltage_reg) == 0) {
            DEBUG("ina22x_read_bus [OK]: bus_voltage_reg = %"PRIu16"\n", bus_voltage_reg);
            bus_voltage_int_v  = (bus_voltage_reg * INA226_BUS_VOLTAGE_LSB_UV) / MICROS_PER_INT;
            bus_voltage_frac_mv = ( (bus_voltage_reg * INA226_BUS_VOLTAGE_LSB_UV) / MICROS_PER_MILLI) % MILLIS_PER_INT;
        }
        else {
            DEBUG("ina22x_read_bus [FAILED]\n");
            break;
        }

        if (ina22x_read_bus_voltage(dev, &bus_voltage_uV) == INA22X_OK) {
            printf("ina22x_read_bus_voltage [OK]: bus_voltage = %"PRIu32"\n", bus_voltage_uV);
        }
        else {
            printf("ina22x_read_bus_voltage [FAILED]\n");
            break;
        }

        if (ina22x_read_shunt_voltage(dev, &sht_voltage_nV) == INA22X_OK) {
            printf("ina22x_read_sht_voltage [OK]: sht_voltage = %"PRId32"\n", sht_voltage_nV);
        }
        else {
            printf("ina22x_read_sht_voltage [FAILED]\n");
            break;
        }




        if (ina22x_read_shunt_reg(dev, &shunt_voltage_reg) == 0) {
            DEBUG("ina22x_read_shunt [OK]: shunt_voltage_reg = %"PRIi16"\n", shunt_voltage_reg);
            shunt_voltage_int_mv  = (shunt_voltage_reg * INA226_SHUNT_VOLTAGE_LSB_NV) / NANOS_PER_MILLI;
            if (shunt_voltage_reg < 0) {
                shunt_voltage_frac_uv = ( (-1 * shunt_voltage_reg * INA226_SHUNT_VOLTAGE_LSB_NV) / NANOS_PER_MICRO ) % MICROS_PER_MILLI;
            }
            else {
                shunt_voltage_frac_uv = ( (shunt_voltage_reg* INA226_SHUNT_VOLTAGE_LSB_NV) / NANOS_PER_MICRO ) % MICROS_PER_MILLI;
            }
        }
        else {
            DEBUG("ina22x_read_shunt [FAILED]\n");
            break;
        }

        if (ina22x_read_current_reg(dev, &current_reg) == 0) {
            DEBUG("ina22x_read_current [OK]: current_reg = %"PRIi16"\n", current_reg);
            current_ua = (current_reg * TEST_INA226_CURRENT_LSB_NA) / NANOS_PER_MICRO;
            current_int_ma = current_ua / MICROS_PER_MILLI;

            if (current_ua < 0) {
                current_frac_ua = (-1 * current_ua) % MICROS_PER_MILLI;
            }
            else{
                current_frac_ua = current_ua % MICROS_PER_MILLI;
            }
        }
        else {
            DEBUG("ina22x_read_current [FAILED]\n");
            break;
        }

        if (ina22x_read_power_reg(dev,&power_reg) == 0) {
            DEBUG("ina22x_read_power [OK]: power_reg = %"PRIi16"\n", power_regc);
            power_uw = (INA226_POWER_LSB_CURRENT_LSB_RATIO * TEST_INA226_CURRENT_LSB_NA * power_reg) / NANOS_PER_MICRO;
            power_int_mw = power_uw / MICROS_PER_MILLI;
            power_frac_uw = power_uw % MICROS_PER_MILLI;
        }
        else {
            DEBUG("ina22x_read_power [FAILED]\n");
            break;
        }


        printf("| bus: %c%" PRIu32 ",%03" PRIu32 " V ", ((bus_voltage_int_v == 0 && bus_voltage_reg < 0) ? '-' : ' '), bus_voltage_int_v, bus_voltage_frac_mv);
        printf("| shunt: %c%" PRId32 ",%03" PRId32 " mV ", ((shunt_voltage_int_mv == 0 && shunt_voltage_reg < 0) ? '-' : ' '), shunt_voltage_int_mv, shunt_voltage_frac_uv);
        printf("| current: %c%" PRId32 ",%03" PRId32 " mA ", ((current_int_ma == 0 && current_ua < 0) ? '-' : ' '), current_int_ma, current_frac_ua);
        printf("| power:  %" PRId32 ",%03" PRId32 " mW |\n", power_int_mw, power_frac_uw);
        fflush(stdout);

        //end = xtimer_now();

        //printf("reading costs %lu usecs\n", xtimer_usec_from_ticks(end) - xtimer_usec_from_ticks(start));

        measures++;

        #if TEST_INA22X_MODEL == INA226
        //reading the mask enable register is needed to re-enable interrupt generation by ina226
        if (ina226_read_mask_enable_reg(dev, &mask_enable_reg) == 0) {
            DEBUG("ina226_read_mask_enable [OK]\n");
        }
        else {
            DEBUG("ina226_read_mask_enable [FAILED]\n");
            break;
        }
        #endif

        thread_yield();
    }

    printf("dying...\n");
    return NULL;
}

static void test_ina226_callback(void *arg)
{
    msg_t msg;
    msg.content.ptr = arg;
    //puts("callback");
    //fflush(stdout);
    msg_send(&msg, _print_thread_pid);
}
