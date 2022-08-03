/*
 * Copyright (C) 2022 HAW Hamburg
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */
/**
 * @ingroup  tests_gclk
 * @{
 *
 * @file
 * @brief    Workload definitions for benchmarking the impact of
 *           dynamic clock configuration on various kinds of tasks.
 *
 * @author   Michel Rottleuthner <michel.rottleuthner@haw-hamburg.de>
 *
 * @}
 */
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include "kernel_defines.h"
#include "workloads.h"
#include "periph/spi.h"
#include "periph/adc.h"
#include "gclk_idle_timer.h"
#include "arm_math.h"
#include "random.h"

#define FFT_LEN (4096)

#define ADC_RES_MIN_BITS  (6)
#define ADC_RES_BITS_STEP (2)
#define ADC_RES_BITS_MAX  (12)

void _adc_custom_extension_set_min_sample_time(uint32_t ns);

arm_rfft_fast_instance_f32 fft_instance;
float32_t fft_in[FFT_LEN];
float32_t fft_out[FFT_LEN];

/* we do Not really care for the actual data so we can use the same buffer for in and out */
uint8_t spi_buff_in_out[1024];

adc_res_t _adc_res_vals[] = { ADC_RES_6BIT, ADC_RES_8BIT, ADC_RES_10BIT, ADC_RES_12BIT};

param_def_t spi_params[] = { {.name = "spifreq"}, {.name = "nbytes"}, {.name = "iterations"}};
param_def_t crypt_params[] = {{.name = "nbytes"}};
param_def_t spincrunch_params[]  = {{.name = "poll_us"}, {.name = "work_cnt"}};
param_def_t fft_params[]  = {{.name = "nvalues"}, {.name = "ifft"}};
param_def_t adc_params[]  = {{.name = "adc_line"}, {.name = "min_sample_time_ns"}, {.name = "res_nbits"}, {.name = "nsamples"}};

worker_task_t worker_tasks[] = {
    { .name = "ml",          .work = work_ml,         .ctx = NULL },
    { .name = "compress",    .work = work_compress,   .ctx = NULL },
    { .name = "encrypt",     .work = work_encrypt,    .ctx = NULL, .param_def = crypt_params, .param_cnt = ARRAY_SIZE(crypt_params) },
    { .name = "decrypt",     .work = work_decrypt,    .ctx = NULL, .param_def = crypt_params, .param_cnt = ARRAY_SIZE(crypt_params)},
    { .name = "spi",         .work = work_spi,        .ctx = NULL, .param_def = spi_params,   .param_cnt = ARRAY_SIZE(spi_params)},
    { .name = "spincrunch",  .work = work_spincrunch, .ctx = NULL, .param_def = spincrunch_params, .param_cnt = ARRAY_SIZE(spincrunch_params)},
    { .name = "fft",         .work = work_fft,        .ctx = NULL, .param_def = fft_params, .param_cnt = ARRAY_SIZE(fft_params)},
    { .name = "adc",         .work = work_adc,        .ctx = NULL, .param_def = adc_params, .param_cnt = ARRAY_SIZE(adc_params)},
};

extern int ml_run(void);
extern void compressiontest(void);
extern void aes_test_encrypt(int nbytes);
extern void aes_test_decrypt(int nbytes);

void workloads_init(void) {
    /* static init fft data with random */
    for (unsigned i = 0; i < FFT_LEN; i++) {
        uint32_t rand = random_uint32();
        fft_in[i] = rand;
    }
}

void work_ml(void *ctx) {
   (void)ctx;
   //printf("mlwork\n");
   ml_run();
}

void work_compress(void *ctx) {
    (void)ctx;
    //printf("compress\n");
    compressiontest();
}

void work_encrypt(void *ctx) {
   //printf("aes_test_encrypt\n");
   uint32_t nbytes = *(uint32_t*)ctx;
   aes_test_encrypt(nbytes);
}

void work_decrypt(void *ctx) {
   uint32_t nbytes = *(uint32_t*)ctx;
   //printf("aes_test_decrypt\n");
   aes_test_decrypt(nbytes);
}

static int _spi_read(unsigned spifreq, uint32_t cnt) {
/* it was ony verified on those two platforms that
 * the driver gracefully handles arbitrary clock frequency values
 * instead of only the fixed defines */
#if !(defined(CPU_MODEL_EFM32PG12B500F1024GL125) || \
      defined(CPU_FAM_STM32L4))
#warning "handing arbitrary clock values to the SPI driver might lead to unexpected results"
   switch (spifreq) {
    case   100000: spifreq = SPI_CLK_100KHZ; break;
    case   400000: spifreq = SPI_CLK_400KHZ; break;
    case  1000000: spifreq = SPI_CLK_1MHZ; break;
    case  5000000: spifreq = SPI_CLK_5MHZ; break;
    case 10000000: spifreq = SPI_CLK_10MHZ; break;
    default:
        printf("unsupported SPI speed!\n");
        return -1;
   }
#endif

   spi_t bus = SPI_DEV(0);
   spi_cs_t cs = SPI_HWCS(0);
   spi_init(bus);

   for (unsigned i = 0; i < cnt; i++) {
       spi_acquire(bus, cs, SPI_MODE_0, spifreq);
       spi_transfer_bytes(bus, cs, false, spi_buff_in_out, spi_buff_in_out, ARRAY_SIZE(spi_buff_in_out));
       spi_release(bus);
   }
   return 0;
}

void work_spi(void *ctx) {
   uint32_t *params = (uint32_t*)ctx;
   uint32_t spifreq = params[0];
   //uint32_t buf_len = params[1];
   uint32_t iterations = params[2];
   _spi_read(spifreq, iterations);
}

void work_spincrunch(void *ctx) {
   uint32_t *params = (uint32_t*)ctx;
   uint32_t poll_us = params[0];
   uint32_t work_cnt = params[1];

   uint64_t t_start = idle_timer_ticks_to_usecs(idle_timer_read());
   while (idle_timer_ticks_to_usecs(idle_timer_read()) < (t_start + poll_us)) {}

   /* if no actual work function callback was given emulate freq dependent work via plain iteration */
   volatile uint32_t cntdwn = work_cnt;
   while (cntdwn--) {}
}

void work_fft(void *ctx) {
   uint32_t *params = (uint32_t*)ctx;
   uint32_t len = params[0];
   uint32_t ifft = params[1];
   arm_status as = arm_rfft_fast_init_f32(&fft_instance, len);
   (void)as;
   uint8_t ifftFlag = ifft;	/* value = 0: RFFT  value = 1: RIFFT */
   arm_rfft_fast_f32(&fft_instance, fft_in, fft_out, ifftFlag);
}

void work_adc(void *ctx) {
    uint32_t *params = (uint32_t*)ctx;
    uint32_t adc_line = params[0];
    uint32_t min_sample_time_ns = params[1];
    uint32_t res_n_bits = params[2];
    uint32_t samples = params[3];

    adc_res_t res = _adc_res_vals[(res_n_bits - ADC_RES_MIN_BITS) / ADC_RES_BITS_STEP];
    _adc_custom_extension_set_min_sample_time(min_sample_time_ns);

    adc_init(ADC_LINE(adc_line));
    while (samples--) {
       int sample = adc_sample(ADC_LINE(adc_line), res);
       (void)sample;
    }
}

bool workloads_parse_params(int argc, char **argv, worker_task_t **wt, uint32_t *param_vals, size_t param_cnt) {
    for (unsigned i = 0; i < ARRAY_SIZE(worker_tasks); i++) {
        if (strcmp(worker_tasks[i].name, argv[0]) == 0) {
            *wt = &worker_tasks[i];
            if ((unsigned)argc < (worker_tasks[i].param_cnt + 1)) {
                printf("missing workload params!\n");
                return false;
            } else if (worker_tasks[i].param_cnt > param_cnt) {
                printf("cant store all workload params!\n");
                return false;
            }
            for (unsigned p = 0; p < worker_tasks[i].param_cnt; p++) {
                param_vals[p] = atoi(argv[1+p]);
            }
            return true;
        }
    }
    return false;
}

void workloads_print_param_usage(void) {
    printf("{");
    for (unsigned i = 0; i < ARRAY_SIZE(worker_tasks); i++) {
        if (i > 0) {
            printf(" | ");
        }

        if (worker_tasks[i].param_cnt > 0) {
            printf("{");
        }
        printf("%s", worker_tasks[i].name);
        if (worker_tasks[i].param_cnt > 0) {
            for (unsigned x = 0; x < worker_tasks[i].param_cnt; x++) {
                printf(" <%s>", worker_tasks[i].param_def[x].name);
            }
            printf("}");
        }
    }
    printf("}");
}
