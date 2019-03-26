/*
 * Copyright (C) 2017 Hamburg University of Applied Sciences
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     pkg_openwsn
 * @{
 *
 * @file
 *
 * @author      Tengfei Chang <tengfei.chang@gmail.com>, July 2012
 * @author      Peter Kietzmann <peter.kietzmann@haw-hamburg.de>, July 2017
 *
 * @}
 */

#include "board_info.h"
#include "sctimer.h"

#include "periph/rtt.h"

#define ENABLE_DEBUG    (0)
#include "debug.h"

// ========================== define ==========================================
#define MINIMUM_COMPAREVALE_ADVANCE  10

#if RTT_FREQUENCY != 32768U
    #error "RTT_FREQUENCY not supported"
#endif



// ========================== variable ========================================


typedef struct {
    sctimer_cbt sctimer_cb;
    bool convert;
    bool convertUnlock;
} sctimer_vars_t;

sctimer_vars_t sctimer_vars;

static void sctimer_isr_internal(void *arg);

/**
   \brief Initialization sctimer.
 */
void sctimer_init(void)
{
    DEBUG("sctimer_init\n");
    memset(&sctimer_vars, 0, sizeof(sctimer_vars_t));
    rtt_init();
}

void sctimer_set_callback(sctimer_cbt cb)
{
    DEBUG("sctimer_set_callback\n");
    sctimer_vars.sctimer_cb = cb;
}

void sctimer_setCompare(uint32_t val)
{
    uint32_t cnt = rtt_get_counter();
    if (val <= cnt) {
        rtt_set_alarm(cnt + MINIMUM_COMPAREVALE_ADVANCE, sctimer_isr_internal, NULL);
    }
    else {
        rtt_set_alarm(val - 1, sctimer_isr_internal, NULL);
    }
}

uint32_t sctimer_readCounter(void)
{
    return rtt_get_counter();
}

void sctimer_enable(void)
{
    DEBUG("sctimer_enable\n");
    rtt_poweron();
}

void sctimer_disable(void)
{
    DEBUG("sctimer_disable\n");
    rtt_poweroff();
}

void sctimer_isr_internal(void *arg)
{
    (void)arg;
    DEBUG("sctimer_isr_internal\n");
    if (sctimer_vars.sctimer_cb != NULL) {
        sctimer_vars.sctimer_cb();
    }
}
