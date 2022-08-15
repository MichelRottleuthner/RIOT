/*
 * Copyright (C) 2020 HAW Hamburg
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     tests
 * @{
 *
 * @file
 * @brief       Test application for the generic clock configuration module
 *
 * @author      Michel Rottleuthner <michel.rottleuthner@haw-hamburg.de>
 *
 * @}
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "shell.h"
#include "mutex.h"

#include "gclk.h"
#include "gclk_manager.h"
#include "gclk/generic_gate.h"
#include "gclk/generic_mux.h"
#include "gclk/generic_scaler.h"
#include "gclock_hw_specific.h"
#include "gclk_idle_timer.h"
#include "eval_utils.h"
#include "dbg_control.h"

#include "periph_cpu.h"
#include "periph/gpio.h"
#include "periph/timer.h"
#include "periph/spi.h"
#include "periph/uart.h"
#include "periph/adc.h"
#include "stdio_uart.h"
#include "stdio_base.h"
#include "include/gpio_wakeup.h"
#include "xtimer.h"
#include "ztimer/config.h"
#include "ztimer/periph_timer.h"
#include "workloads.h"

#define LOG_LEVEL LOG_NONE
#include "log.h"

#define MAX_WORKLOAD_PARAM_CNT (4)
typedef void (*worker_func_cb_t)(void *ctx);

#define SC_AUTOSCALE_WORKERS_BASE_PARAM_CNT (8)
#define AUTOSCALE_WORKER_TEST_MAX_THREAD_NUM (2)

#define MAIN_MSG_QUEUE_SIZE (64)

/* defined in spi driver.
 * access used as workaround for missing API to query spi-internal config */
extern int actual_spi_speeds[SPI_NUMOF];

/* udp command is defined in separate file */
extern int udp_cmd(int argc, char **argv);

static msg_t _msg_queue[MAIN_MSG_QUEUE_SIZE];

typedef struct {
    uint32_t thread_iters;
    mutex_t  stop_mutex;
    mutex_t  done_mutex;
    gpio_t   dbg_pin;
    worker_func_cb_t work_cb; /* if this is set to NULL, the given dummy values
                                 for poll_us, idle_us and work_iters are used to
                                 simulate a given worker behavior */
    void *work_params;        /* workload-specific parameter values for use in work_cb */
} worker_thread_context_t;

char worker_thread_stacks[AUTOSCALE_WORKER_TEST_MAX_THREAD_NUM][THREAD_STACKSIZE_MAIN];
char desched_thread_stack[THREAD_STACKSIZE_TINY];

void *desched_thread(void *arg) {
    mutex_t *terminator = (mutex_t*)arg;
    while (true) {
        thread_yield();
        if (mutex_trylock(terminator) != 0) {
            break;
        }
    }
    return NULL;
}

void *worker_thread(void *arg) {
    worker_thread_context_t *ctx = (worker_thread_context_t*)arg;

    if (ctx->dbg_pin != GPIO_UNDEF) {
        gpio_clear(ctx->dbg_pin);
    }

    void *work_ctx = ctx->work_params;

    while(true) {
        /* the mutex will be unlocked to indicate the worker shall be stopped */
        if(mutex_trylock(&ctx->stop_mutex) == 1) {
            break;
        }

        ctx->work_cb(work_ctx);

        /* in order to make use of any thread based utilization at some point the scheduler must be invoked */
        thread_yield();
    }

    if (ctx->dbg_pin != GPIO_UNDEF) {
        gpio_set(ctx->dbg_pin);
    }

    mutex_unlock(&ctx->done_mutex);
    return NULL;
}

/* This command is meant for testing the method used to runtime-assess how much
   a thread benefits (speeds up) by using a higher frequency.
   In best case this should report a low value (close to zero) if the thread
   does not speed up at all and a high value (towards 100) if the performance
   scales linear with frequency.
   To evaluate if this result is useful we want to do three things.
   1) run this for various different kinds of tasks
   2) run the same kind of tasks with all possible frequency configurations
      to identify how the reported performance-utility value is best used to
      adapt frequency at runtime
   3) Test the findings against reality where various different kinds of
      threads are opreating at the same time and adapt their frequency at
      runtime

    NOTE: if UART communication is needed for workers it is recommended to
          configure the UART to a clock that is able to operate
          independent of the system clock (e.g. HSI16 in nucleo-l476rg) before
          calling this command */
int _sc_perf_util_test(int argc, char **argv) {
    if (argc < 5) {
        goto sc_perf_util_test_usage;
    }

    worker_thread_context_t ctx = {
        .thread_iters = 0, /* run forever (till killed) */
        .stop_mutex   = MUTEX_INIT_LOCKED,
        .done_mutex   = MUTEX_INIT_LOCKED,
        .work_cb      = NULL,
    };

    int min_cpu_time_us = atoi(argv[1]);
    int min_schedule_cnt = atoi(argv[2]);
    bool verbose = strcmp(argv[3], "verbose") == 0;

    if (!verbose && strcmp(argv[3], "quiet") != 0) {
        goto sc_perf_util_test_usage;
    }

    uint32_t workload_params[MAX_WORKLOAD_PARAM_CNT];
    worker_task_t *wt;
    if (!workloads_parse_params(argc - 4, &argv[4], &wt, workload_params, MAX_WORKLOAD_PARAM_CNT)) {
        goto sc_perf_util_test_usage;
    }

    ctx.work_cb = wt->work;
    ctx.work_params = workload_params;

    idle_timer_enable();

    gclk_manager_clear_performance_util_data();
    gclk_manager_enable_pu_assessment(true);

    kernel_pid_t wtid = thread_create(
             worker_thread_stacks[0],
             sizeof(worker_thread_stacks[0]),
             THREAD_PRIORITY_MAIN + 2,
             THREAD_CREATE_WOUT_YIELD | THREAD_CREATE_STACKTEST,
             worker_thread,
             &ctx,
             "worker");

    gclk_manager_enable_pu_stat_request_for_thread(wtid);

    /* empty thread that emulates other system threads that grabs the CPU from time to time */
    mutex_t desched_terminator = MUTEX_INIT_LOCKED;
    kernel_pid_t ds_tid = thread_create(
             desched_thread_stack,
             sizeof(desched_thread_stack),
             THREAD_PRIORITY_MAIN + 2,
             THREAD_CREATE_WOUT_YIELD | THREAD_CREATE_STACKTEST,
             desched_thread,
             &desched_terminator,
             "desched_thread");

    LOG_DEBUG("started worker (%d) and desched (%d)\n", wtid, ds_tid);

    gclk_manager_start_freq_cycler(min_cpu_time_us, min_schedule_cnt);

    //printf("killing worker...\n");
    mutex_unlock(&ctx.stop_mutex);
    mutex_unlock(&desched_terminator);

    /* waiting till the worker acutally died */
    mutex_lock(&ctx.done_mutex);

    /* ensure we sleep long enough for the threads to actually be dequeued */
    xtimer_usleep(20000);

    /* calculate the PU metric of the worker and output verbose metadata data if requested */
    int pu = gclk_manager_calculate_pu_factor(wtid, verbose);
    printf("performance util of thread %u: %d\n", wtid, pu);
    gclk_manager_enable_pu_assessment(false);

    return 0;

    sc_perf_util_test_usage:
    printf("Usage: %s <min_cpu_time_us> <min_schedule_cnt> {verbose|quiet} ", argv[0]);
    workloads_print_param_usage();
    printf("\n");
    return -1;
}

int _sc_autoscale_worker_test(int argc, char **argv) {

    if (argc < SC_AUTOSCALE_WORKERS_BASE_PARAM_CNT + 1) {
        goto _sc_autoscale_worker_test_usage;
    }

    /* configure a fast and low frequency and corresponding PU thresholds that tigger using that particular frequency */
    int fboost_pu_threshold = atoi(argv[1]); /* reasonable value: 60 */
    uint32_t fboost = atoi(argv[2]);
    int fthrottle_pu_threshold = atoi(argv[3]); /* reasonable value: 30 */
    uint32_t fthrottle = atoi(argv[4]);

    gclk_manager_set_dvfs_pu_params(fboost, fthrottle, fboost_pu_threshold, fthrottle_pu_threshold);
    int dmm_sync = atoi(argv[5]);
    uint32_t fc_min_us = atoi(argv[6]);
    uint32_t fc_min_scheds = atoi(argv[7]);

    idle_timer_enable();

    kernel_pid_t wtids[AUTOSCALE_WORKER_TEST_MAX_THREAD_NUM];

    worker_thread_context_t ctxs[AUTOSCALE_WORKER_TEST_MAX_THREAD_NUM];
    uint32_t worker_params[AUTOSCALE_WORKER_TEST_MAX_THREAD_NUM][MAX_WORKLOAD_PARAM_CNT];

    if (dmm_sync) {
        NOTIFY_START_TO_DMM;
    }

    unsigned thread_cnt = 0;
    /* start parsing workload params after the last base param */
    for (int pc = SC_AUTOSCALE_WORKERS_BASE_PARAM_CNT; pc < (argc-1);) {
        worker_task_t *wt;
        if (!workloads_parse_params(argc - pc, &argv[pc], &wt,
                                    worker_params[thread_cnt], MAX_WORKLOAD_PARAM_CNT)) {
            goto _sc_autoscale_worker_test_usage;
        }

        pc += wt->param_cnt + 1;

        ctxs[thread_cnt].work_cb = wt->work;
        ctxs[thread_cnt].work_params = worker_params[thread_cnt];
        ctxs[thread_cnt].dbg_pin      = GPIO_UNDEF;
        ctxs[thread_cnt].thread_iters = 0, /* run forever (till killed) */

        /* setup mutexes so the worker will wait till we signal it to stop.
         * and another one that we can wait for to ensure the worker is done */
        mutex_init(&ctxs[thread_cnt].stop_mutex);
        mutex_init(&ctxs[thread_cnt].done_mutex);
        mutex_lock(&ctxs[thread_cnt].stop_mutex);
        mutex_lock(&ctxs[thread_cnt].done_mutex);

        thread_cnt++;
    }

    for (unsigned t = 0; t < thread_cnt; t++) {
        wtids[t] = thread_create(
                 worker_thread_stacks[t],
                 sizeof(worker_thread_stacks[0]),
                 THREAD_PRIORITY_MAIN + 2,
                 THREAD_CREATE_WOUT_YIELD | THREAD_CREATE_STACKTEST,
                 worker_thread,
                 &ctxs[t],
                 "worker");
    }

    /* let the threads work some time before enabling PU assessment */
    idle_timer_wait(1000);

    gclk_manager_clear_performance_util_data();
    gclk_manager_enable_pu_assessment(true);
    for (unsigned i = 0; i < thread_cnt; i++) {
        gclk_manager_enable_pu_stat_request_for_thread(wtids[i]);
    }

    /* perform frequency cycle to evaluate the PU characteristics of the running worker threads */
    gclk_manager_start_freq_cycler(fc_min_us, fc_min_scheds);

    gclk_manager_enable_pu_assessment(false);

    gclk_manager_enable_dynamic_frequency_scaling(true);

    /* wait some time to let the threads work with dynamic frequency scaling enabled */
    idle_timer_wait(1000);

    gclk_manager_enable_dynamic_frequency_scaling(false);

    /* tell all non self-terminating threads to stop */
    for (unsigned i = 0; i < thread_cnt; i++) {
        if (ctxs[i].thread_iters == 0) {
            mutex_unlock(&ctxs[i].stop_mutex);
        }
    }

    /* wait for all threads to be finished. Since we only care for the moment when
     * the last one finishes we can just iterate all of them */
    for (unsigned i = 0; i < thread_cnt; i++) {
        mutex_lock(&ctxs[i].done_mutex);
    }

    if (dmm_sync) {
        NOTIFY_STOP_TO_DMM;
    }

    /* disable automatic scaling again */
    gclk_manager_enable_dynamic_frequency_scaling(false);

    for (unsigned i = 0; i < thread_cnt; i++) {
        bool verbose = false;
        int pu = gclk_manager_calculate_pu_factor(wtids[i], verbose);
        printf("performance util of thread %u: %d\n", wtids[i], pu);
    }

    return 0;

    _sc_autoscale_worker_test_usage:
    printf("Usage: %s boost_th f_boost thr_th f_thr dmm_sync fc_min_us fc_min_scheds {", argv[0]);
    workloads_print_param_usage();
    printf("}+\n");
    return -1;
}

static int _sc_spi_speed(int argc, char **argv) {
    (void)argc;
    (void)argv;
    if (argc != 2) {
        printf("usage: %s <target_spi_speed>\n", argv[0]);
        return -1;
    }
    unsigned target_clk = atoi(argv[1]);
    spi_acquire(SPI_DEV(0), UNUSED_SPI_SC_PIN, SPI_MODE_0, target_clk);
    int current_speed = actual_spi_speeds[SPI_DEV(0)];
    spi_release(SPI_DEV(0));
    printf("current clock configuration results in SPI frequncy of %d Hz (aimed for %d Hz)\n", current_speed, target_clk);
    return 0;
}

int _sc_work(int argc, char **argv)
{
    uint32_t workload_params[MAX_WORKLOAD_PARAM_CNT];
    worker_task_t *wt;
    if (!workloads_parse_params(argc - 1, &argv[1], &wt, workload_params, MAX_WORKLOAD_PARAM_CNT)) {
        printf("Usage: %s ", argv[0]);
        workloads_print_param_usage();
        printf(" [dmmsync]\n");
        return -1;
    }

    bool dmm_sync = false;
   

    if(strcmp("dmmsync", argv[argc-1]) == 0) {
        dmm_sync = true;
        //printf("starting work!\n");
    }

    worker_func_cb_t work_cb = wt->work;

    gpio_init(OSZI_DEBUG_PIN, GPIO_OUT);
    gpio_set(OSZI_DEBUG_PIN);

    if (dmm_sync) {
        NOTIFY_START_TO_DMM;
    }

    /* mark start of work */
    gpio_clear(OSZI_DEBUG_PIN);

    work_cb(workload_params);

    /* mark end of work */
    gpio_set(OSZI_DEBUG_PIN);

    if (dmm_sync) {
        NOTIFY_STOP_TO_DMM;
        //printf("work done!\n");
    }

    return 0;
}

/* Other clock control commands are provided by the respective
 * shell-command submodule (in /sys/shell/commands/sc_gclk.c) */
const shell_command_t shell_commands[] = {
/* utility/test commands to work out quirks and check setup health*/
    { "gpio_test",       "test gpio toggle", _sc_gpio_test },
    { "work",            "perform various kinds of workloads", _sc_work },
    { "blink",           "perform some busy gpio blink work to visualize if CPU is working", _sc_blink },
    { "idle_timer",      "enable/disable idle timer", _sc_ctl_idle_timer },
    { "timer_test",      "show speed of timers", _sc_timer_test },
    { "latest",          "test logic analizer instrumentation", _sc_test_time_trace },
    { "clk_pin",         "output clock via pin", _sc_clk_pin },
    { "dbg",             "configure settings of the debugger (SWD/DWT etc.)", dbg_control_sc },
/* commands for evaluation measurements and microbenchmarks */
    { "perf_util_test",  "test performance utilization assessment", _sc_perf_util_test },
    { "eval_sleep",      "perform an xtimer sleep to go idle", _sc_sleep },
    { "eval_TCI",        "eval task characteristic impact", _sc_TCI },
    { "eval_ACGI",       "eval automatic clock gating impact", _sc_ACGI },
    { "eval_workers",    "start some workers to see how frequency auto-scaling behaves", _sc_autoscale_worker_test },
    { "udp",             "send data over UDP and listen on UDP ports", udp_cmd },
    { "spispeed",        "prints the actually obtainable and aimed for spi speed for the current clock config", _sc_spi_speed },
    { "cmdlist_add",     "add a shell command to a list for later execution", _sc_cmdlist_add },
    { "cmdlist_execute", "execute all commands in the command list in FIFO order", _sc_cmdlist_exe },
    { NULL, NULL, NULL }
};

void wakeup_cb(void *arg) {
    (void)arg;
    /* first thing is to notify the logic analizer that we woke up */
    gpio_clear(LOGIC_ANALYZER_PIN);
#if defined(CPU_FAM_STM32L4)
    /* clear the wakeup flag so the next sleep will not immediately return */
    gpio_wakeup_clear(WAKEUP_GPIO);
#endif
    /* set to high so LA can see the falling flank on next interrupt or main entry */
    gpio_set(LOGIC_ANALYZER_PIN);
    //printf("wakeup!\n");
}

/* setup a wakeup source for low power mode sleep consumption measurement */
static void _low_power_sleep_wakeup_init(void)
{
#if defined(CPU_FAM_STM32L4)
    gpio_wakeup_clear(WAKEUP_GPIO);
    gpio_init_int(WAKEUP_GPIO, GPIO_IN_PU, GPIO_FALLING, wakeup_cb, NULL);
    gpio_wakeup_pull_config(WAKEUP_GPIO, GPIO_IN_PU);
    gpio_wakeup_enable(WAKEUP_GPIO, GPIO_FALLING);
    xtimer_usleep(100 * 1000);
    /* Force DBGMCU configuration into low power mode to enable lowest low-power mode consumption.
     * this still does not solve all the problems - but pm set 0; then hitting the reset button is already better than a cold start by unplugging.*/
    /* disable the debug mode for sleep modes to allow low-power also directly after flashing without re-plugging the board */
    DBGMCU->CR &= ~(DBGMCU_CR_DBG_SLEEP | DBGMCU_CR_DBG_STOP | DBGMCU_CR_DBG_STANDBY | DBGMCU_CR_TRACE_IOEN); // DBGMCU_CR_TRACE_IOEN
#elif defined(CPU_MODEL_EFM32PG12B500F1024GL125)
    gpio_init_int(WAKEUP_GPIO, GPIO_IN, GPIO_FALLING, wakeup_cb, NULL);
#endif
}

int main(void)
{
    /* set to high so LA can see the falling flank on either interrupt or main_trampoline
       This pin is already initialized by main_trampoline */
    gpio_set(LOGIC_ANALYZER_PIN);

    gpio_init(OSZI_DEBUG_PIN, GPIO_OUT);
    gpio_set(OSZI_DEBUG_PIN);

    gpio_init(RADIO_SUPPLY_SWITCH_PIN, GPIO_OUT);
    gpio_set(RADIO_SUPPLY_SWITCH_PIN);
    
    //gpio_init(LED0_PIN, GPIO_OUT);
    //gpio_clear(LED0_PIN);

    printf("ScaleClock Test Application!\n");
    workloads_init();
    msg_init_queue(_msg_queue, MAIN_MSG_QUEUE_SIZE);

    /* TODO: the below stuff should move to an init function */
    int res = gclk_manager_init();

    if (res != 0) {
        printf("ERROR when initializing gclk_manager\n");
    }

    gpio_init(DMM_TRIGGER_PIN_START, GPIO_OUT);
    gpio_init(DMM_TRIGGER_PIN_STOP, GPIO_OUT);

    /* set the pin here .. will be cleared after WFI when leaving LPM */
    gpio_init(DBG_GPIO_WFI, GPIO_OUT);
    gpio_set(DBG_GPIO_WFI);

    gpio_clear(DMM_TRIGGER_PIN_START);
    gpio_clear(DMM_TRIGGER_PIN_STOP);

    gpio_init(DMM_SYNC_PIN, GPIO_IN);

    _low_power_sleep_wakeup_init();

#if defined (CPU_FAM_STM32L4)
#endif

    /* init idle timer but turn it off as it is not used initially */
    idle_timer_init();
    idle_timer_disable();

    char line_buf[SHELL_DEFAULT_BUFSIZE];
    shell_run(shell_commands, line_buf, SHELL_DEFAULT_BUFSIZE);

    return 0;
}
