/*
 * Copyright (C) 2016-2017 Bas Stottelaar <basstottelaar@gmail.com>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     cpu_efm32
 * @ingroup     drivers_periph_adc
 * @{
 *
 * @file
 * @brief       Low-level ADC driver implementation
 *
 * @author      Bas Stottelaar <basstottelaar@gmail.com>
 *
 * @}
 */

#include <assert.h>

#include "cpu.h"
#include "mutex.h"

#include "periph_conf.h"
#include "periph/adc.h"

#include "em_cmu.h"
#include "em_adc.h"

static mutex_t adc_lock[ADC_DEV_NUMOF];

static uint32_t _min_sampling_time_ns = 1000;
/* adcAcqTime8 is the default value of the original RIOT implementation */
static uint32_t _acquisition_val = adcAcqTime8;
#define ADC_DEFAULT_FREQ (400000)
#define NS_PER_SECOND (1000000000)
#define NS_PER_ADC_CYLCE (NS_PER_SECOND / ADC_DEFAULT_FREQ)
/* The API currently does not support explicit setting of different sample times.
 * This custom helper is used to inject a minimum required sample time that will
 * be used by the default init function */
void _adc_custom_extension_set_min_sample_time(uint32_t ns) {
    _min_sampling_time_ns = ns;

/* on other platforms this mapping might not be appliccable (not verivied) */
#if defined(CPU_MODEL_EFM32PG12B500F1024GL125)
    /* min: adcAcqTime1 == 0; max: adcAcqTime256 == 9 */
    _acquisition_val = adcAcqTime256;
    for (unsigned i = 0; i < 9; i++) {
        /* acqisition cycles are powers of two given by 2^i */
        unsigned cycles = 1 << i;
        if (cycles * NS_PER_ADC_CYLCE >= _min_sampling_time_ns) {
            _acquisition_val = i;
            break;
        }
    }
#elif
#error "Custom ADC extension for sample time calculation not verified on this platform"
#endif
}

int adc_init(adc_t line)
{
    /* check if line is valid */
    if (line >= ADC_NUMOF) {
        return -1;
    }

    uint8_t dev = adc_channel_config[line].dev;
    assert(dev < ADC_DEV_NUMOF);

    /* initialize lock */
    mutex_init(&adc_lock[dev]);

    /* enable clock */
    CMU_ClockEnable(cmuClock_HFPER, true);
    CMU_ClockEnable(adc_config[dev].cmu, true);

    /* reset and initialize peripheral */
    ADC_Init_TypeDef init = ADC_INIT_DEFAULT;

    init.timebase = ADC_TimebaseCalc(0);

    init.prescale = ADC_PrescaleCalc(ADC_DEFAULT_FREQ, 0);
    ADC_Reset(adc_config[dev].dev);
    ADC_Init(adc_config[dev].dev, &init);

    return 0;
}

int32_t adc_sample(adc_t line, adc_res_t res)
{
    /* resolutions larger than 12 bits are not supported */
    if (res >= ADC_MODE_UNDEF(0)) {
        return -1;
    }

    uint8_t dev = adc_channel_config[line].dev;

    /* lock device */
    mutex_lock(&adc_lock[dev]);

    /* setup channel */
    ADC_InitSingle_TypeDef init = ADC_INITSINGLE_DEFAULT;

    init.acqTime = _acquisition_val;
    init.reference = adc_channel_config[line].reference;
    init.resolution = (ADC_Res_TypeDef) (res & 0x0F);
#if defined(_SILICON_LABS_32B_SERIES_0)
    init.input = adc_channel_config[line].input;
#elif defined(_SILICON_LABS_32B_SERIES_1)
    init.posSel = adc_channel_config[line].input;
#endif

    ADC_InitSingle(adc_config[dev].dev, &init);

    /* start conversion and block until it completes */
    ADC_Start(adc_config[dev].dev, adcStartSingle);

    while ((adc_config[dev].dev->STATUS & ADC_STATUS_SINGLEDV) == 0);

    int result = ADC_DataSingleGet(adc_config[dev].dev);

    /* for resolutions that are not really supported, shift the result (for
       instance, 10 bit resolution is achieved by shifting a 12 bit sample). */
    result = result >> (res >> 4);

    /* unlock device */
    mutex_unlock(&adc_lock[dev]);

    return result;
}
