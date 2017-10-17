/*
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    drivers_ina22x INA22x current/power monitor
 * @ingroup     drivers_sensors
 * @brief       Device driver for Texas Instruments INA22x High-Side or Low-Side Measurement,
 *              Bi-Directional Current and Power Monitor with I2C Compatible Interface
 * @{
 *
 * @file
 * @brief       Device driver interface for Texas Instruments INA22x High-Side or Low-Side Measurement,
 *              Bi-Directional Current and Power Monitor with I2C Compatible Interface.
 *              All references to the INA226 data sheet are related to the document revision SBOS547A
 *
 * @author      Michel Rottleuthner <michel.rottleuthner@haw-hamburg.de>
 */

#ifndef INA22X_H
#define INA22X_H

#include <stdint.h>

#include "periph/gpio.h"
#include "periph/i2c.h"
#include "ina226-regs.h"
#include "ina22x-regs.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief INA22x device model
 *
 */
typedef enum ina22x_model {
    INA220, /**< INA220 only has registers listed in ina22x_reg_t */
    INA226, /**< INA226 has registers listed in ina22x_reg_t and ina226_reg_t*/
} ina22x_model_t;

/**
 * @brief Device params for INA22x sensors
 */
typedef struct {
    ina22x_model_t model;    /**< the actual type of the device INA220/INA226 */
    i2c_t i2c;               /**< I2C device the sensor is connected to */
    uint8_t addr;            /**< the slave address of the sensor on the I2C bus */
    uint16_t config;         /**< the slave address of the sensor on the I2C bus */
    uint16_t cal;            /**< value that is written to the calibration reg */
    uint16_t current_lsb_na; /**< value of current register-LSB in na */
} ina22x_params_t;

/**
 * @brief Device descriptor for INA22x sensors
 */
typedef struct {
    ina22x_model_t model;    /**< the actual type of the device INA220/INA226 */
    i2c_t i2c;               /**< I2C device the sensor is connected to */
    uint8_t addr;            /**< the slave address of the sensor on the I2C bus */
    uint16_t config;         /**< the slave address of the sensor on the I2C bus */
    uint16_t cal;            /**< value that is written to the calibration reg */
    uint16_t current_lsb_na; /**< value of current register-LSB in na */
} ina22x_t;

/** @brief Shunt voltage measurement range (PGA settings, INA220 only) */
typedef enum ina22x_state {
    INA22X_OK      =  0, /**< all went as expected */
    INA22X_I2C_ERR = -1, /**< error in I2C communication */
    INA22X_PIN_ERR = -2, /**< error with INA22x pin config*/
} ina22x_state_t;

/**
 * @brief INA22x possible mode settings. (Bits 0-2 in configuration register)
 * @see Table 9 in data sheet (INA226)
 */
typedef enum ina22x_mode {
    INA22X_MODE_POWERDOWN                = 0x0000, /**< Power-Down (or Shutdown) */
    INA22X_MODE_TRIGGER_SHUNT_ONLY       = 0x0001, /**< Shunt Voltage, Triggered */
    INA22X_MODE_TRIGGER_BUS_ONLY         = 0x0002, /**< Bus Voltage, Triggered */
    INA22X_MODE_TRIGGER_SHUNT_AND_BUS    = 0x0003, /**< Shunt and Bus, Triggered */
    INA22X_MODE_POWERDOWN_ADC            = 0x0004, /**< ADC off, on INA226 same as POWERDOWN*/
    INA22X_MODE_CONTINUOUS_SHUNT_ONLY    = 0x0005, /**< Shunt Voltage, Continuous */
    INA22X_MODE_CONTINUOUS_BUS_ONLY      = 0x0006, /**< Bus Voltage, Continuous */
    INA22X_MODE_CONTINUOUS_SHUNT_AND_BUS = 0x0007, /**< Shunt and Bus, Continuous, default */
} ina22x_mode_t;

/** @brief Shunt voltage measurement range (PGA settings, INA220 only) */
typedef enum ina220_range {
    INA220_RANGE_40MV  = 0x0000, /**< +/- 40 mV range */
    INA220_RANGE_80MV  = 0x0800, /**< +/- 80 mV range */
    INA220_RANGE_160MV = 0x1000, /**< +/- 160 mV range */
    INA220_RANGE_320MV = 0x1800, /**< +/- 320 mV range, default */
} ina220_range_t;

/** @brief Bus voltage measurement range (INA220 only))*/
typedef enum ina220_brng {
    INA220_BRNG_16V_FSR = 0x0000, /**< 16 V bus voltage full scale range */
    INA220_BRNG_32V_FSR = 0x0200, /**< 32 V bus voltage full scale range, default. */
} ina220_brng_t;

/**
 * @brief Shunt ADC settings (INA220 only)
 *
 * @see Table 5 in INA220 data sheet
 */
typedef enum ina220_sadc {
    /** 9 bit resolution, 84 us conversion time */
    INA220_SADC_9BIT            = 0x0000,
    /** 10 bit resolution, 148 us conversion time */
    INA220_SADC_10BIT           = 0x0008,
    /** 11 bit resolution, 276 us conversion time */
    INA220_SADC_11BIT           = 0x0010,
    /** 12 bit resolution, 532 us conversion time, default */
    INA220_SADC_12BIT           = 0x0018,
    /** 12 bit resolution, 532 us conversion time, same as INA220_SADC_12BIT */
    INA220_SADC_AVG_1_SAMPLE    = 0x0040,
    /** 2 sample average, 1.06 ms conversion time */
    INA220_SADC_AVG_2_SAMPLES   = 0x0048,
    /** 4 sample average, 2.13 ms conversion time */
    INA220_SADC_AVG_4_SAMPLES   = 0x0050,
    /** 8 sample average, 4.26 ms conversion time */
    INA220_SADC_AVG_8_SAMPLES   = 0x0058,
    /** 16 sample average, 8.51 ms conversion time */
    INA220_SADC_AVG_16_SAMPLES  = 0x0060,
    /** 32 sample average, 17.02 ms conversion time */
    INA220_SADC_AVG_32_SAMPLES  = 0x0068,
    /** 64 sample average, 34.05 ms conversion time */
    INA220_SADC_AVG_64_SAMPLES  = 0x0070,
    /** 128 sample average, 68.10 ms conversion time */
    INA220_SADC_AVG_128_SAMPLES = 0x0078,
} ina220_sadc_t;

/**
 * @brief Bus ADC settings (INA220 only)
 *
 * @see Table 5 in INA220 data sheet
 */
typedef enum ina220_badc {
    /** 9 bit resolution, 84 us conversion time */
    INA220_BADC_9BIT            = 0x0000,
    /** 10 bit resolution, 148 us conversion time */
    INA220_BADC_10BIT           = 0x0080,
    /** 11 bit resolution, 276 us conversion time */
    INA220_BADC_11BIT           = 0x0100,
    /** 12 bit resolution, 532 us conversion time, default */
    INA220_BADC_12BIT           = 0x0180,
    /** 12 bit resolution, 532 us conversion time, same as INA220_BADC_12BIT */
    INA220_BADC_AVG_1_SAMPLE    = 0x0400,
    /** 2 sample average, 1.06 ms conversion time */
    INA220_BADC_AVG_2_SAMPLES   = 0x0480,
    /** 4 sample average, 2.13 ms conversion time */
    INA220_BADC_AVG_4_SAMPLES   = 0x0500,
    /** 8 sample average, 4.26 ms conversion time */
    INA220_BADC_AVG_8_SAMPLES   = 0x0580,
    /** 16 sample average, 8.51 ms conversion time */
    INA220_BADC_AVG_16_SAMPLES  = 0x0600,
    /** 32 sample average, 17.02 ms conversion time */
    INA220_BADC_AVG_32_SAMPLES  = 0x0680,
    /** 64 sample average, 34.05 ms conversion time */
    INA220_BADC_AVG_64_SAMPLES  = 0x0700,
    /** 128 sample average, 68.10 ms conversion time */
    INA220_BADC_AVG_128_SAMPLES = 0x0780,
} ina220_badc_t;

/**
 * @brief Determines the number of samples that are collected and averaged.
 *        (Bits 9–11 in configuration register)
 *
 * @see Table 6 in INA226 data sheet
 */
typedef enum ina226_avg {
    INA226_AVG_1     = 0x0000, /**no averaging, default */
    INA226_AVG_4     = 0x0200, /**averaging 4 measurements */
    INA226_AVG_16    = 0x0400, /**averaging 16 measurements */
    INA226_AVG_64    = 0x0600, /**averaging 64 measurements */
    INA226_AVG_128   = 0x0800, /**averaging 128 measurements */
    INA226_AVG_256   = 0x0A00, /**averaging 256 measurements */
    INA226_AVG_512   = 0x0C00, /**averaging 512 measurements */
    INA226_AVG_1024  = 0x0E00, /**averaging 1024 measurements */
} ina226_avg_t;


/** INA22x reset command bit in configuration register (setting this bit to '1' generates a system reset) */
#define INA22X_RESET_BIT (0x8000)

/** Location of the bus voltage in the INA220 bus voltage register */
#define INA220_BUS_VOLTAGE_SHIFT (3)





/** INA226 full scale (max. value) of the bus voltage register. Equivalent to 40.96 V (LSB = 1.25 mV)*/
#define INA226_FULL_VOLTAGE_SCALE (0x7FFF)

/** INA226 bus voltage LSB in microvolts = 1.25 mV*/
#define INA226_BUS_VOLTAGE_LSB_UV (1250)

/** INA220 bus voltage LSB in microvolts = 4.0 mV*/
#define INA220_BUS_VOLTAGE_LSB_UV (4000)

/** INA226 shunt voltage LSB in nanovolts = 2.5 uV*/
#define INA226_SHUNT_VOLTAGE_LSB_NV (2500)

/** INA220 shunt voltage LSB in nanovolts = 20.0 uV*/
#define INA220_SHUNT_VOLTAGE_LSB_NV (10000)

/** INA226 maximum voltage drop over shunt-resistor see "Shunt voltage input range" in data sheet section 6.5*/
#define INA226_MAX_SHUNT_VOLTAGE_UV (81920)

/**
 * INA226 power register value LSB is based on the programmed current register LSB.
 * For the ina226 this fixed ratio is 25. => 25 * CurrentLSB = PowerLSB */
#define INA226_POWER_LSB_CURRENT_LSB_RATIO (25)

/**
 * INA220 power register value LSB is based on the programmed current register LSB.
 * For the ina220 this fixed ratio is 20. => 20 * CurrentLSB = PowerLSB */
#define INA220_POWER_LSB_CURRENT_LSB_RATIO (20)

/** INA22x Current_LSB = MAX_EXPECTED_CURRENT / 2^15
    (see ina226 datasheet section 7.5 or ina220 datasheet section 8.5)*/
#define INA22X_MAX_CURRENT_TO_LSB_RATIO (32768)

/**
 * INA226 CAL = 0.00512 / Current_LSB * R_shunt (see ina226 datasheet section 7.5)
 * this value equals 1,000,000,000,000 * 0.00512. This value is scaled up to ensure
 * maximum accuracy for macro-based calculation of CAL value without using floating point.
 * For proper calculation of CAL value based on this define use [nA] for Current_LSB and [mOhm] for R_shunt*/
#define INA226_CAL_SCALE_FACTOR_INT (5120000000LL)

/**
 * INA220 CAL = 0.00512 / Current_LSB * R_shunt (see ina220 datasheet section 8.5)
 * this value equals 1,000,000,000,000 * 0.04096. This value is scaled up to ensure
 * maximum accuracy for macro-based calculation of CAL value without using floating point.
 * For proper calculation of CAL value based on this define use [nA] for Current_LSB and [mOhm] for R_shunt*/
#define INA220_CAL_SCALE_FACTOR_INT (40960000000LL)


#define MILLIS_PER_INT   (1000)
#define MICROS_PER_INT   (1000 * MILLIS_PER_INT)
#define MICROS_PER_MILLI (1000)
#define NANOS_PER_MICRO  (1000)
#define NANOS_PER_MILLI  (1000 * NANOS_PER_MICRO)



/**
 * @brief Determines the bus voltage measurement conversion time. (Bits 6–8 in configuration register)
 * @see Table 7 in INA226 data sheet
 */
typedef enum ina226_vbusct {
    INA226_VBUSCT_140_US   = 0x0000, /**conversion time 140 us */
    INA226_VBUSCT_204_US   = 0x0040, /**conversion time 204 us */
    INA226_VBUSCT_332_US   = 0x0080, /**conversion time 332 us */
    INA226_VBUSCT_588_US   = 0x00C0, /**conversion time 588 us */
    INA226_VBUSCT_1_1_MS   = 0x0100, /**conversion time 1.1 ms, default */
    INA226_VBUSCT_2_116_MS = 0x0140, /**conversion time 2.116 ms */
    INA226_VBUSCT_4_156_MS = 0x0180, /**conversion time 4.156 ms */
    INA226_VBUSCT_8_244_MS = 0x01C0, /**conversion time 8.244 ms */
} ina226_vbusct_t;

/**
 * @brief Determines the shunt voltage measurement conversion time. (Bits 3–5 in configuration register)
 * @see Table 8 in INA226 data sheet
 */
typedef enum ina226_vshct {
    INA226_VSHCT_140_US   = 0x0000, /**conversion time 140 us */
    INA226_VSHCT_204_US   = 0x0008, /**conversion time 204 us */
    INA226_VSHCT_332_US   = 0x0010, /**conversion time 332 us */
    INA226_VSHCT_588_US   = 0x0018, /**conversion time 588 us */
    INA226_VSHCT_1_1_MS   = 0x0020, /**conversion time 1.1 ms, default */
    INA226_VSHCT_2_116_MS = 0x0028, /**conversion time 2.116 ms */
    INA226_VSHCT_4_156_MS = 0x0030, /**conversion time 4.156 ms */
    INA226_VSHCT_8_244_MS = 0x0038, /**conversion time 8.244 ms */
} ina226_vshct_t;


/**
 * @brief Read bus voltage of INA22x device
 *
 * @param[in]  dev          device descriptor of sensor
 * @param[out] uvolt        pointer to where the value will be stored Unit: [µV]
 *
 * @return                  0 on success
 * @return                  <0 on error
 */
int ina22x_read_bus_voltage(ina22x_t *dev, uint32_t* uvolt);

/**
 * @brief Read bus voltage of INA22x device
 *
 * @param[in]  dev          device descriptor of sensor
 * @param[out] nvolt        pointer to where the value will be stored. Unit: [nV]
 *
 * @return                  0 on success
 * @return                  <0 on error
 */
int ina22x_read_shunt_voltage(ina22x_t *dev, int32_t* nvolt);

/**
 * @brief Read bus voltage of INA22x device
 *
 * @param[in]  dev          device descriptor of sensor
 * @param[out] ua           pointer to where the value will be stored. Unit: [µA]
 *
 * @return                  0 on success
 * @return                  <0 on error
 */
int ina22x_read_current(ina22x_t *dev, int32_t* ua);

/**
 * @brief Read bus voltage of INA22x device
 *
 * @param[in]  dev          device descriptor of sensor
 * @param[out] uw           pointer to where the value will be stored. Unit: [µW]
 *
 * @return                  0 on success
 * @return                  <0 on error
 */
int ina22x_read_power(ina22x_t *dev, uint32_t* uw);

/**
 * @brief Read one 16 bit register of INA22x device
 *
 * @param[in]  dev          device descriptor of sensor
 * @param[in]  reg          register to read
 * @param[out] out          pointer to where the register value will be stored
 *
 * @return                  0 on success
 * @return                  <0 on error
 */
int ina22x_read_reg(ina22x_t *dev, uint8_t reg, uint16_t *out);

/**
 * @brief Write one 16 bit register of INA22x device
 *
 * @param[in] dev           device descriptor of sensor
 * @param[in] reg           register to write
 * @param[in] in            value to write to register
 *
 * @return                  0 on success
 * @return                  <0 on error
 */
int ina22x_write_reg(ina22x_t *dev, uint8_t reg, uint16_t in);

/**
 * @brief Initialize a current sensor
 *
 * @param[out] dev          device descriptor of sensor to initialize
 * @param[in]  params       params for this device
 *
 * @return                  0 on success
 * @return                  <0 on error
 */
int ina22x_init(ina22x_t *dev, const ina22x_params_t *params);

/**
 * @brief enable interrupt for the alert pin
 *
 * @param[in]  dev          device descriptor of sensor
 * @param[in]  me_config    the configuration of the mask/enable register (specifies
                            which condition triggers the interrupt)
 * @param[in]  pin          the gpio pin that is connected to the alert pin
                            of the ina226 device
 * @param[in]  callback     function that is called when an interrupt occurs
 *
 * @return                     0 on success
 *                             <0 on error
 */
int ina226_activate_int(ina22x_t *dev, uint16_t me_config, gpio_t pin, gpio_cb_t callback);

/**
 * @brief Read calibration register
 *
 * @param[in]   dev          device descriptor of sensor to configure
 * @param[out]  calibration  calibration register settings, see data sheet
 *
 * @return                  0 on success
 * @return                  <0 on error
 */
inline int ina22x_read_calibration_reg(ina22x_t *dev, uint16_t *calibration)
{
    return ina22x_read_reg(dev, INA22X_REG_CALIBRATION, calibration);
}

/**
 * @brief Write to calibration register
 *
 * @param[in]  dev          device descriptor of sensor to configure
 * @param[in]  calibration  calibration register settings, see data sheet
 *
 * @return                  0 on success
 * @return                  <0 on error
 */
inline int ina22x_write_calibration_reg(ina22x_t *dev, uint16_t calibration)
{
    return ina22x_write_reg(dev, INA22X_REG_CALIBRATION, calibration);
}

/**
 * @brief Read configuration register
 *
 * @param[in]   dev          device descriptor of sensor to configure
 * @param[out]  config       configuration register settings, see data sheet
 *
 * @return                   0 on success
 * @return                   <0 on error
 */
inline int ina22x_read_config_reg(ina22x_t *dev, uint16_t *config)
{
    return ina22x_read_reg(dev, INA22X_REG_CONFIGURATION, config);
}

/**
 * @brief Write to configuration register
 *
 * @param[in]  dev          device descriptor of sensor to configure
 * @param[in]  config       configuration register settings, see data sheet
 *
 * @return                  0 on success
 * @return                  <0 on error
 */
inline int ina22x_write_config_reg(ina22x_t *dev, uint16_t config)
{
    return ina22x_write_reg(dev, INA22X_REG_CONFIGURATION, config);
}

/**
 * @brief Read shunt voltage
 *
 * The Shunt Voltage Register stores the current shunt voltage reading.
 * Negative numbers are represented in two's complement format.
 * If averaging is enabled, this register displays the averaged value.
 * Full-scale range = 81.92 mV (decimal = 7FFF); LSB: 2.5 uV
 *
 * @param[in]  dev          device descriptor of sensor
 * @param[out] voltage      measured voltage across shunt resistor
 *
 * @return                  0 on success
 * @return                  <0 on error
 */
inline int ina22x_read_shunt_reg(ina22x_t *dev, int16_t *voltage)
{
    return ina22x_read_reg(dev, INA22X_REG_SHUNT_VOLTAGE, (uint16_t *)voltage);
}

/**
 * @brief Read bus voltage register
 *
 * The Bus Voltage Register stores the most recent bus voltage reading, VBUS.
 * If averaging is enabled, this register displays the averaged value.
 * Full-scale range = 40.96 V (decimal = 7FFF); LSB = 1.25 mV.
 *
 * See the device data sheet for details.
 *
 * @param[in]  dev          device descriptor of sensor
 * @param[out] voltage      measured bus voltage
 *
 * @return                  0 on success
 * @return                  <0 on error
 */
inline int ina22x_read_bus_reg(ina22x_t *dev, int16_t *voltage)
{
    return ina22x_read_reg(dev, INA22X_REG_BUS_VOLTAGE, (uint16_t *)voltage);
}

/**
 * @brief Read shunt current
 *
 * @param[in]  dev          device descriptor of sensor
 * @param[out] current      measured current through shunt resistor
 *
 * @return                  0 on success
 * @return                  <0 on error
 */
inline int ina22x_read_current_reg(ina22x_t *dev, int16_t *current)
{
    return ina22x_read_reg(dev, INA22X_REG_CURRENT, (uint16_t *)current);
}

/**
 * @brief Read power consumption
 *
 * @param[in]  dev          device descriptor of sensor
 * @param[out] power        measured power consumption
 *
 * @return                  0 on success
 * @return                  <0 on error
 */
inline int ina22x_read_power_reg(ina22x_t *dev, int16_t *power)
{
    return ina22x_read_reg(dev, INA22X_REG_POWER, (uint16_t *)power);
}

/**
 * @brief read die ID Register
 *
 * @param[in]  dev          device descriptor of sensor
 * @param[out] id           the unique identification number and the revision ID for the die
 *
 * @return                  0 on success
 * @return                  <0 on error
 */
inline int ina226_read_die_id_reg(ina22x_t *dev, uint16_t *id)
{
    return ina22x_read_reg(dev, INA226_REG_DIE_ID, id);
}

/**
 * @brief read manufacturer ID Register
 *
 * @param[in]  dev          device descriptor of sensor
 * @param[out] id           the manufacturer ID (if the INA226 is set up correctly
                            this value should be 0x5449)
 *
 * @return                  0 on success
 * @return                  <0 on error
 */
inline int ina226_read_manufacturer_id_reg(ina22x_t *dev, uint16_t *id)
{
    return ina22x_read_reg(dev, INA226_REG_MANUFACTURER_ID, id);
}

/**
 * @brief read the mask/enable register
 *
 * @param[in]  dev          device descriptor of sensor
 * @param[out] val          the value of the mask/enable register
 *
 * @return                  0 on success
 * @return                  <0 on error
 */
inline int ina226_read_mask_enable_reg(ina22x_t *dev, uint16_t *val)
{
    return ina22x_read_reg(dev, INA226_REG_MASK_ENABLE, val);
}

/**
 * @brief write the mask/enable register
 *
 * @param[in]  dev          device descriptor of sensor
 * @param[in]  val          the value of the mask/enable register
 *
 * @return                  0 on success
 * @return                  <0 on error
 */
inline int ina226_write_mask_enable_reg(ina22x_t *dev, uint16_t val)
{
    return ina22x_write_reg(dev, INA226_REG_MASK_ENABLE, val);
}

/**
 * @brief read alert limit register
 *
 * @param[in]  dev          device descriptor of sensor
 * @param[out] val          the value of the alert limit register
 *
 * @return                  0 on success
 * @return                  <0 on error
 */
inline int ina226_read_alert_limit_reg(ina22x_t *dev, uint16_t *val)
{
    return ina22x_read_reg(dev, INA226_REG_ALERT_LIMIT, val);
}

/**
 * @brief write alert limit register
 *
 * @param[in]  dev          device descriptor of sensor
 * @param[in]  val          the value of the alert limit register
 *
 * @return                  0 on success
 * @return                  <0 on error
 */
inline int ina226_write_alert_limit_reg(ina22x_t *dev, uint16_t val)
{
    return ina22x_write_reg(dev, INA226_REG_ALERT_LIMIT, val);
}

#ifdef __cplusplus
}
#endif

#endif /* INA22X_H */
/** @} */
