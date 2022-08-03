/*
 * Copyright (C) 2022 Michel Rottleuthner <mihel.rottleuthner@haw-hamburg.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 *
 *
 * @file
 * @brief    Workload definitions for benchmarking the impact of
 *           dynamic clock configuration on various kinds of tasks.
 *
 * @author      Michel Rottleuthner <mihel.rottleuthner@haw-hamburg.de>
 *
 */

#ifndef WORKLOADS_H
#define WORKLOADS_H

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*worker_func_cb_t)(void *ctx);

typedef struct {
    char *name;
} param_def_t;

typedef struct {
    char *name;
    worker_func_cb_t work;
    void *ctx;
    param_def_t *param_def;
    unsigned param_cnt;
} worker_task_t;

void workloads_init(void);
bool workloads_parse_params(int argc, char **argv, worker_task_t **wt, uint32_t *param_vals, size_t param_cnt);
void workloads_print_param_usage(void);

void work_ml(void *ctx);
void work_compress(void *ctx);
void work_encrypt(void *ctx);
void work_decrypt(void *ctx);
void work_spi(void *ctx);
void work_spincrunch(void *ctx);
void work_fft(void *ctx);
void work_adc(void *ctx);

#ifdef __cplusplus
}
#endif

#endif /* WORKLOADS_H */
/** @} */
