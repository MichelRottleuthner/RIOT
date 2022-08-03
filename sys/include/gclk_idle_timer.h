/*
 * Copyright (C) 2021 HAW Hamburg
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
 * @brief       Simple wrapper for a timer that is able to measure idle time.
 *              Usually a timer that can run in lowest power mode should be used
 *              for this like an RTT or RTC.
 *
 * @author      Michel Rottleuthner <michel.rottleuthner@haw-hamburg.de>
 *
 * @}
 */
#ifndef IDLE_TIMER_H
#define IDLE_TIMER_H

#include <stdint.h>
#include <stdbool.h>
/* TODO: some guards like below should be added to switch to other backends
 *       on platforms that don't support RTT */
//#if defined(CPU_FAM_STM32L4)
#include "periph/rtt.h"
#define IDLE_TMR_LL_TIMER_FREQ (RTT_FREQUENCY)
#define IDLE_TMR_LL_TIMER_MAX  (RTT_MAX_VALUE)
//#endif

static inline bool _within_this_period(uint64_t target, uint64_t time_base) {
    return (target - time_base) <= IDLE_TMR_LL_TIMER_MAX;
}

static inline uint64_t idle_timer_usecs_to_ticks(uint64_t usecs) {
    return usecs * IDLE_TMR_LL_TIMER_FREQ / 1000000;
}

static inline uint64_t idle_timer_ticks_to_usecs(uint64_t idle_ticks) {
    return idle_ticks * 1000000 / IDLE_TMR_LL_TIMER_FREQ;
}

uint64_t idle_timer_read(void);
void idle_timer_init(void);
void idle_timer_enable(void);
void idle_timer_disable(void);
void idle_timer_set(uint64_t abs_counter);
void idle_timer_set_alarm(uint64_t absolute_target);
uint64_t idle_timer_read_alarm(void);
void idle_timer_clear_alarm(void);
void idle_timer_wait(uint32_t ms);
#endif
