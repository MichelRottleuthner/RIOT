/*
 * Copyright (C) 2014 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup tests
 * @{
 *
 * @file
 * @brief       Test for low-level Real Time Timer drivers
 *
 * This test will initialize the real-time timer and trigger an alarm printing
 * 'Hello' every 5 seconds
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 *
 * @}
 */

#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>

#include "cpu.h"
#include "periph_conf.h"
#include "periph/rtt.h"
#include "xtimer.h"

#define TICKS_TO_WAIT       (5 * RTT_FREQUENCY)

static volatile uint32_t last;

void cb(void *arg)
{
    (void)arg;

    last += TICKS_TO_WAIT;
    last &= RTT_MAX_VALUE;
    rtt_set_alarm(last, cb, 0);

    puts("Hello");
}

#define REPEAT 10000000

int main(void)
{
    puts("\nRIOT RTT low-level driver test");
    puts("This test will display 'Hello' every 5 seconds\n");

    puts("Initializing the RTT driver");
    rtt_init();

    volatile uint32_t now = rtt_get_counter();
    xtimer_init();


    printf("start..\n");
    uint32_t start = xtimer_now_usec();
    uint32_t a = 0;
    uint32_t b = 0;
    uint32_t max = 0;
    for (int i = 0; i < REPEAT; i++) {
        a = xtimer_now_usec();
        //xtimer_usleep();
        rtt_set_alarm(0xffffffff, cb, 0);
        b = xtimer_now_usec();
        if ((b-a) > max) {
            max = (b-a);
        }
    }

    uint32_t end = xtimer_now_usec();

    printf("done: %ld (%ld µs per call)\n", end - start, (end - start)/REPEAT );
    printf("max %lu\n", max);


    printf("RTT now: %" PRIu32 "\n", now);

    last = (now + TICKS_TO_WAIT) & RTT_MAX_VALUE;
    printf("Setting initial alarm to now + 5 s (%" PRIu32 ")\n", last);
    rtt_set_alarm(last, cb, 0);

    puts("Done setting up the RTT, wait for many Hellos");
    return 0;
}
