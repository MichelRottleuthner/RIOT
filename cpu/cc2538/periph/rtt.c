/*
 * Copyright (C) 2019 HAW Hamburg
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     cpu_cc2538
 * @ingroup     drivers_periph_rtt
 * @{
 *
 * @file
 * @brief       RTT implementation for the CC2538 CPU using the sleep timer
 *
 * @author      Michel Rottleuthner <michel.rottleuthner@haw-hamburg.de>
 *
 * @}
 */

#include "cpu.h"
#include "irq.h"
#include "periph/rtt.h"

/* allocate memory for overflow and alarm callbacks + args */
static rtt_cb_t ovf_cb = NULL;
static void *ovf_arg;
static rtt_cb_t cmp_cb = NULL;
static void *cmp_arg;

#define SMWDTHROSC_STLOAD_FLAG_LOAD_DONE (0x01)

void rtt_init(void)
{
    /* sleep timer is enabled by defualt so nothing needs to be done here */
}

uint32_t rtt_get_counter(void)
{
    return ((SMWDTHROSC_ST0 & 0xFF) |
           ((SMWDTHROSC_ST1 & 0xFF) << 8) |
           ((SMWDTHROSC_ST2 & 0xFF) << 16) |
           ((SMWDTHROSC_ST3 & 0xFF) << 24) );
}

void rtt_set_overflow_cb(rtt_cb_t cb, void *arg)
{
    assert(cb);

    unsigned is = irq_disable();
    ovf_cb  = cb;
    ovf_arg = arg;
    irq_restore(is);
}

void rtt_clear_overflow_cb(void)
{
    ovf_cb = NULL;
}

void rtt_set_alarm(uint32_t alarm, rtt_cb_t cb, void *arg)
{
    /* assert(cb && !(alarm & ~RTT_MAX_VALUE)); */
    (void)alarm;
    unsigned is = irq_disable();
    cmp_cb  = cb;
    cmp_arg = arg;
    /* TODO check that alarm is at least 5 counts greater than current val */
    while (! (SMWDTHROSC_STLOAD & SMWDTHROSC_STLOAD_FLAG_LOAD_DONE)) {}

    SMWDTHROSC_ST3 = (alarm >> 24) & 0xFF;
    SMWDTHROSC_ST2 = (alarm >> 16) & 0xFF;
    SMWDTHROSC_ST1 = (alarm >>  8) & 0xFF;
    SMWDTHROSC_ST0 = alarm & 0xFF;

    /* enable interrupt */
    NVIC_EnableIRQ(SM_TIMER_ALT_IRQn);
    irq_restore(is);
}

void rtt_clear_alarm(void)
{
    cmp_cb = NULL;
    NVIC_DisableIRQ(SM_TIMER_ALT_IRQn);
}

void rtt_poweron(void)
{
    /* sleep timer is always enabled on this platform */
}

void rtt_poweroff(void)
{
    /* cannot be disabled explicitly */
}

void isr_sleepmode(void)
{
    if (cmp_cb) {
        rtt_cb_t cb = cmp_cb;
        cmp_cb = NULL;
        cb(cmp_arg);
    }

    cortexm_isr_end();
}
