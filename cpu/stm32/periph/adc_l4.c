/*
 * Copyright (C) 2014-2016 Freie Universität Berlin
 * Copyright (C) 2018 HAW-Hamburg
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     cpu_stm32
 * @ingroup     drivers_periph_adc
 * @{
 *
 * @file
 * @brief       Low-level ADC driver implementation
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @author      Michel Rottleuthner <michel.rottleuthner@haw-hamburg.de>
 *
 * @}
 */

#include "cpu.h"
#include "mutex.h"
#include "periph/adc.h"
#include "periph_conf.h"
#include "periph/vbat.h"
#include "ztimer.h"

/**
 * @brief Not all STM32 L4 boards have 3 ADC devices
 * for example, L4R5ZI has only one ADC
 */
#if defined ADC_DEVS && ADC_DEVS == 1
#define ADC ADC1_COMMON
#endif
#if defined ADC_DEVS && ADC_DEVS == 3
#define ADC ADC123_COMMON
#endif

/**
 * @brief map CPU specific register/value names
 */
#if defined(CPU_MODEL_STM32L476RG) || defined(CPU_MODEL_STM32L4R5ZI)
#define ADC_CR_REG      CR
#define ADC_ISR_REG     ISR
#define ADC_PERIPH_CLK  AHB2
/* on stm32-l476rg all ADC clocks are are enabled by this bit
   further clock config is possible over CKMODE[1:0] bits in ADC_CCR reg */
#define ADC_CLK_EN_MASK   (RCC_AHB2ENR_ADCEN)
/* referring to Datasheet Section 6.3.18 (ADC characteristics) the minimum
   achievable sampling rate is 4.21 Msps (12 Bit resolution on slow channel)
   we use that worst case for configuring the sampling time to be sure it
   works on all channels.
   TCONV = Sampling time + 12.5 ADC clock cycles.
   At 80MHz this means we need to set SMP to 001 (6.5 ADC clock cycles) to
   stay within specs. (80000000/(6.5+12.5)) = 4210526   */
#define ADC_SMP_MIN_VAL      (0x1)

/* The sampling time can be specified for each channel over SMPR1 and SMPR2.
   This specifies the first channel that goes to SMPR2 instead of SMPR1. */
#define ADC_SMP_BIT_WIDTH    (3)

/* The sampling time can be specified for each channel over SMPR1 and SMPR2.
   This specifies the first channel that goes to SMPR2 instead of SMPR1. */
#define ADC_SMPR2_FIRST_CHAN (10)

/* This is used to define the minimum smapling time required to properly sample the
 * input signal. This will effectively dictated by the characteristics of the measured signal.
 * I.e. the R_AIN external input resistance is defined to be at max 50kOhm.
 * Table 77. of the datasheet shows a table on max R_AIN values for each Sampling time.
 * ~8000 ns will result in the longest sampling time @80MHz.
 */
static uint32_t _min_sampling_time_ns = 1000;

/* used to convert from SMP (sampling time register) value to clock adc cycles*10 */
uint32_t smp2cycle10ths[] = {25, 65, 125, 245, 475, 925, 2475, 6405};

uint32_t _smp_for_min_cycle_time_and_freq(uint32_t min_cycle_time_ns, uint32_t adcfreq){
    uint32_t cycle_ns_10ths = 1000000000 / (adcfreq / 10);
    for (unsigned i = 0; i < ARRAY_SIZE(smp2cycle10ths); i++) {
        uint32_t sample_time_ns = (smp2cycle10ths[i] * cycle_ns_10ths) / 100;
        if (sample_time_ns >= min_cycle_time_ns) {
            return i;
        }
    }
    return 7; /* 0b111*/
}

/* The API currently does not support explicit setting of different sample times.
 * This custom helper is used to inject a minimum required sample time that will
 * be used by the default init function */
void _adc_custom_extension_set_min_sample_time(uint32_t ns) {
    _min_sampling_time_ns = ns;
}
#endif

/**
 * @brief   Default VBAT undefined value
 */
#ifndef VBAT_ADC
#define VBAT_ADC    ADC_UNDEF
#endif

/**
 * @brief   Allocate locks for all three available ADC devices
 */
static mutex_t locks[ADC_DEVS];

static inline ADC_TypeDef *dev(adc_t line)
{
    return (ADC_TypeDef *)(ADC1_BASE + (adc_config[line].dev << 8));
}

static inline void prep(adc_t line)
{
    mutex_lock(&locks[adc_config[line].dev]);
    periph_clk_en(ADC_PERIPH_CLK, ADC_CLK_EN_MASK);
}

static inline void done(adc_t line)
{
/* on STM32L476RG (TODO: maybe true for other L4's? - haven't checked yet)
   all adc devices are controlled by this one bit.
   So don't disable the clock as other devices may still use it */
#if !defined(CPU_MODEL_STM32L476RG)
    periph_clk_dis(ADC_PERIPH_CLK, ADC_CLK_EN_MASK);
#endif
    mutex_unlock(&locks[adc_config[line].dev]);
}

/**
 * @brief   Extract the port base address from the given pin identifier
 */
static inline GPIO_TypeDef *_port(gpio_t pin)
{
    return (GPIO_TypeDef *)(pin & ~(0x0f));
}

/**
 * @brief   Extract the pin number from the last 4 bit of the pin identifier
 */
static inline int _pin_num(gpio_t pin)
{
    return (pin & 0x0f);
}

#include "gclk/generic_mux.h"

/* ADC clock source for async mode (CKMODE == 0)*/
extern const gclk_mux_ll_t gclk_stm32_adc_mux;

#include "gclk/generic_scaler.h"
/* ADC clock for sync mode (CKMODE != 0) */
extern const gclk_clk_scaler_ll_t gclk_stm32_ahb_scaler;

/* as of now the ADC implementation only supports synchronous operation */
const gclk_t *adc_clock_handle = &gclk_stm32_ahb_scaler.base;


int adc_init(adc_t line)
{
    /* check if the line is valid */
    if (line >= ADC_NUMOF) {
        return -1;
    }

    /* lock device and enable its peripheral clock */
    prep(line);

    /* set prescaler to 0 to let the ADC run with maximum speed */
    ADC->CCR &= ~(ADC_CCR_PRESC);

    /* Setting ADC clock to HCLK/1 is only allowed if AHB clock prescaler is 1*/
    if (!(RCC->CFGR & RCC_CFGR_HPRE_3)) {
        /* set ADC clock to HCLK/1 */
        ADC->CCR |= (ADC_CCR_CKMODE_0);
    }
    else {
        /* set ADC clock to HCLK/2 otherwise */
        ADC->CCR |= (ADC_CCR_CKMODE_1);
    }

    /* configure the pin */
    if (adc_config[line].pin != GPIO_UNDEF) {
        gpio_init_analog(adc_config[line].pin);
    }
#if defined(CPU_MODEL_STM32L476RG) || defined(CPU_MODEL_STM32L475VG)
    /* On STM32L475xx/476xx/486xx devices, before any conversion of an input channel coming
       from GPIO pads, it is necessary to configure the corresponding GPIOx_ASCR register in
       the GPIO, in addition to the I/O configuration in analog mode. */
    _port(adc_config[line].pin)->ASCR |= (1 << _pin_num(adc_config[line].pin));
#endif

    /* init ADC line only if it wasn't already initialized */
    if (!(dev(line)->ADC_CR_REG & (ADC_CR_ADEN))) {
        /* reset state of bit DEEPPWD is 1 -> so first leave deep-power down mode */
        dev(line)->ADC_CR_REG &= ~(ADC_CR_DEEPPWD);

        /* enable ADC internal voltage regulator and wait for startup period */
        dev(line)->ADC_CR_REG |= (ADC_CR_ADVREGEN);
#if IS_USED(MODULE_ZTIMER_USEC)
        ztimer_sleep(ZTIMER_USEC, ADC_T_ADCVREG_STUP_US);
#else
        /* to avoid using ZTIMER_USEC unless already included round up the
           internal voltage regulator start up to 1ms */
        ztimer_sleep(ZTIMER_MSEC, 1);
#endif

        /* configure calibration for single ended input */
        dev(line)->ADC_CR_REG &= ~(ADC_CR_ADCALDIF);

        /* ´start automatic calibration and wait for it to complete */
        dev(line)->ADC_CR_REG |= ADC_CR_ADCAL;
        while (dev(line)->ADC_CR_REG  & ADC_CR_ADCAL) {}

        /* clear ADRDY by writing it*/
        dev(line)->ADC_ISR_REG |= (ADC_ISR_ADRDY);

        /* enable ADC and wait for it to be ready */
        dev(line)->ADC_CR_REG |= (ADC_CR_ADEN);
        while ((dev(line)->ADC_ISR_REG & ADC_ISR_ADRDY) == 0) {}

        /* set sequence length to 1 conversion */
        dev(line)->SQR1 |= (0 & ADC_SQR1_L);
    }

    /* adapt the SMP value in oder to achive roughly a fixed sampling time of the sampling step
     * (analog input charging ADC capacitors) across different frequencies. The ADC internal SAR step
     * only depends on the resolution and therefore scales linarly with the frequency.
     * i.e. at a lower frequency we do not need to wait as many ticks to get the same absolute sampling time */
    uint32_t adc_freq = gclk_get_current_freq(adc_clock_handle);
    uint32_t smp_val = _smp_for_min_cycle_time_and_freq(_min_sampling_time_ns, adc_freq);

    /* configure sampling time for the given channel */
    if (adc_config[line].chan < ADC_SMPR2_FIRST_CHAN) {
        dev(line)->SMPR1 =  (smp_val << (adc_config[line].chan *
                                                 ADC_SMP_BIT_WIDTH));
    }
    else {
        dev(line)->SMPR2 =  (smp_val << ((adc_config[line].chan -
                                                  ADC_SMPR2_FIRST_CHAN)
                                                 * ADC_SMP_BIT_WIDTH));
    }

    /* free the device again */
    done(line);
    return 0;
}

int32_t adc_sample(adc_t line, adc_res_t res)
{
    int sample;

    /* check if resolution is applicable */
    if (res & 0x3) {
        return -1;
    }

    /* lock and power on the ADC device  */
    prep(line);

    /* check if this is the VBAT line */
    if (IS_USED(MODULE_PERIPH_VBAT) && line == VBAT_ADC) {
        vbat_enable();
    }

    /* first clear resolution */
    dev(line)->CFGR &= ~(ADC_CFGR_RES);

    /* then set resolution to the required value*/
    dev(line)->CFGR |= res;

    /* specify channel for regular conversion */
    dev(line)->SQR1 = (adc_config[line].chan << ADC_SQR1_SQ1_Pos);

    /* start conversion and wait for it to complete */
    dev(line)->ADC_CR_REG |= ADC_CR_ADSTART;
    while (!(dev(line)->ISR & ADC_ISR_EOC)) {}

    /* read the sample */
    sample = (int)dev(line)->DR;

    /* check if this is the VBAT line */
    if (IS_USED(MODULE_PERIPH_VBAT) && line == VBAT_ADC) {
        vbat_disable();
    }

    /* free the device again */
    done(line);

    return sample;
}
