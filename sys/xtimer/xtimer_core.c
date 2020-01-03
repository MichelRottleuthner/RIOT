/**
 * Copyright (C) 2015 Kaspar Schleiser <kaspar@schleiser.de>
 *               2016 Eistec AB
 *               2018 Josua Arndt
 *               2018 UC Berkeley
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup sys_xtimer
 *
 * @{
 * @file
 * @brief xtimer core functionality
 * @author Kaspar Schleiser <kaspar@schleiser.de>
 * @author Joakim Nohlgård <joakim.nohlgard@eistec.se>
 * @author Josua Arndt <jarndt@ias.rwth-aachen.de>
 * @author Hyung-Sin Kim <hs.kim@cs.berkeley.edu>
 * @}
 */

#include <stdint.h>
#include <string.h>
#include "board.h"
#include "periph/timer.h"
#include "periph_conf.h"

#include "xtimer.h"
#include "irq.h"

/* WARNING! enabling this will have side effects and can lead to timer underflows. */
#define ENABLE_DEBUG 0
#include "debug.h"

static volatile int _in_handler = 0;

volatile uint64_t _xtimer_current_time = 0;

static xtimer_t *timer_list_head = NULL;
static xtimer_t *long_list_head = NULL;
static bool _lltimer_ongoing = false;

static void _add_timer_to_list(xtimer_t **list_head, xtimer_t *timer);
static void _shoot(xtimer_t *timer);
static inline void _update_short_timers(uint32_t *now);
static inline void _update_long_timers(uint32_t *now);
static inline void _schedule_earliest_lltimer(uint32_t now);

static void _timer_callback(void);
static void _periph_timer_callback(void *arg, int chan);

void xtimer_init(void)
{
    /* initialize low-level timer */
    timer_init(XTIMER_DEV, XTIMER_HZ, _periph_timer_callback, NULL);

    /* register initial overflow tick */
    _schedule_earliest_lltimer(_xtimer_now());
}

uint32_t _xtimer_now(void)
{
    return (uint32_t) _xtimer_now64();
}

void _xtimer_set64(xtimer_t *timer, uint32_t offset, uint32_t long_offset)
{
    DEBUG(" _xtimer_set64() offset=%" PRIu32 " long_offset=%" PRIu32 "\n", offset, long_offset);

    if (!timer->callback) {
        DEBUG("_xtimer_set64(): timer has no callback.\n");
        return;
    }

    xtimer_remove(timer);

    if (!long_offset && offset < XTIMER_BACKOFF) {
        /* timer fits into the short timer */
        _xtimer_spin(offset);
        _shoot(timer);
        return;
    }

    /* time sensitive */
    unsigned int state = irq_disable();
    uint32_t now = _xtimer_now();
    timer->offset = offset;
    timer->long_offset = long_offset;
    timer->start_time = now;

    if (!long_offset) {
        _add_timer_to_list(&timer_list_head, timer);

        if (timer_list_head == timer) {
            DEBUG("_xtimer_set64(): timer is new list head. updating lltimer.\n");
            _schedule_earliest_lltimer(now);
        }
    }
    else {
        _add_timer_to_list(&long_list_head, timer);
        DEBUG("_xtimer_set64(): added longterm timer (long_offset=%" PRIu32 " offset=%" PRIu32 ")\n",
              timer->long_offset, timer->long_offset);
    }
    irq_restore(state);
}

static void _periph_timer_callback(void *arg, int chan)
{
    (void)arg;
    (void)chan;
    _timer_callback();
}

static void _shoot(xtimer_t *timer)
{
    timer->callback(timer->arg);
}

static inline void _schedule_earliest_lltimer(uint32_t now)
{
    uint32_t target;

    if (_in_handler) {
        return;
    }

    if (timer_list_head && timer_list_head->offset <= _xtimer_lltimer_mask(0xFFFFFFFF)) {
        /* schedule lltimer on next timer target time */
        target = timer_list_head->start_time + timer_list_head->offset;
    }
    else if (!_lltimer_ongoing) {
        /* schedule lltimer after max_low_level_time/2 to detect a cycle */
        target = now + (_xtimer_lltimer_mask(0xFFFFFFFF)>>1);
    }
    else {
        /* lltimer is already running */
        return;
    }

    DEBUG("_schedule_earliest_lltimer(): setting %" PRIu32 "\n", _xtimer_lltimer_mask(target));
    timer_set_absolute(XTIMER_DEV, XTIMER_CHAN, _xtimer_lltimer_mask(target));
    _lltimer_ongoing = true;
}

/**
 * @brief compare two timers. return true if timerA expires earlier than timerB and false otherwise.
 */
static bool _timer_comparison(xtimer_t* timerA, xtimer_t* timerB)
{
    if (timerA->long_offset < timerB->long_offset) {
        return true;
    }
    if (timerA->long_offset == timerB->long_offset
        && timerA->start_time + timerA->offset - timerB->start_time <= timerB->offset) {
        return true;
    }
    return false;
}

/**
 * @brief add a timer to an ordered list of timers
 */
static void _add_timer_to_list(xtimer_t **list_head, xtimer_t *timer)
{
    while (*list_head && _timer_comparison((*list_head), timer)) {
        list_head = &((*list_head)->next);
    }

    timer->next = *list_head;
    *list_head = timer;
}

/**
 * @brief remove a timer from an ordered list of timers
 */
static int _remove_timer_from_list(xtimer_t **list_head, xtimer_t *timer)
{
    while (*list_head) {
        if (*list_head == timer) {
            *list_head = timer->next;
            timer->offset = 0;
            timer->long_offset = 0;
            timer->start_time = 0;
            timer->next = NULL;
            return 1;
        }
        list_head = &((*list_head)->next);
    }

    return 0;
}

void xtimer_remove(xtimer_t *timer)
{
    bool reschedule = false;

    /* time sensitive since the target timer can be fired */
    unsigned int state = irq_disable();
    if (timer == timer_list_head) {
        reschedule = true;
    }

    if (!_remove_timer_from_list(&timer_list_head, timer)) {
        _remove_timer_from_list(&long_list_head, timer);
    }

    /* lltimer should be recheduled when timer_list_head is removed */
    if (reschedule) {
        uint32_t now = _xtimer_now();
        _update_short_timers(&now);
        _schedule_earliest_lltimer(now);
    }
    irq_restore(state);
}

/**
 * @brief update long timers' offsets and switch those that will expire in
 *        one short timer period to the short timer list
 */
static inline void _update_long_timers(uint32_t *now)
{
    if (long_list_head) {
        xtimer_t *curr_timer = long_list_head;

        while (curr_timer) {
            uint32_t elapsed = *now - curr_timer->start_time;
            if (curr_timer->offset <= elapsed) {
                curr_timer->long_offset--;
            }
            curr_timer->offset -= elapsed;
            curr_timer->start_time = *now;

            if (!curr_timer->long_offset) {
                assert(curr_timer == long_list_head);

                xtimer_t *trans_timer = curr_timer;
                curr_timer = curr_timer->next;
                _remove_timer_from_list(&long_list_head, trans_timer);
                _add_timer_to_list(&timer_list_head, trans_timer);
            }
            else {
                curr_timer = curr_timer->next;
            }
        }
    }
}

/**
 * @brief update short timers' offsets and fire those that are close to expiry
 */
static inline void _update_short_timers(uint32_t *now) {
    while (timer_list_head) {
        uint32_t elapsed = *now - timer_list_head->start_time;

        if (timer_list_head->offset <= elapsed ||
            timer_list_head->offset - elapsed < XTIMER_ISR_BACKOFF) {
            /* make sure we don't fire too early */
            while(_xtimer_now() - timer_list_head->start_time < timer_list_head->offset) {}

            /* pick first timer in list */
            xtimer_t *timer = timer_list_head;

            /* advance list */
            timer_list_head = timer->next;

            /* make sure timer is recognized as being already fired */
            timer->offset = 0;
            timer->long_offset = 0;
            timer->next = NULL;

            /* fire timer */
            _shoot(timer);
            /* update current_time */
            *now = _xtimer_now();
        }
        else {
            timer_list_head->offset -= elapsed;
            timer_list_head->start_time = *now;
            return;
        }
    }
}

/**
 * @brief main xtimer callback function (called in an interrupt context)
 */
static void _timer_callback(void)
{
    uint32_t now;
    _in_handler = 1;
    _lltimer_ongoing = false;
    now = _xtimer_now();

overflow:
    /* update short timer offset and fire */
    _update_short_timers(&now);
    /* update long timer offset */
    _update_long_timers(&now);
    /* update current time */
    now = _xtimer_now();

    if (timer_list_head) {
        /* make sure we're not setting a time in the past */
        uint32_t elapsed = now - timer_list_head->start_time;
        if (timer_list_head->offset <= elapsed ||
            timer_list_head->offset - elapsed <= XTIMER_ISR_BACKOFF) {
            goto overflow;
        }
        else {
            timer_list_head->offset -= elapsed;
            timer_list_head->start_time = now;
        }
    }
    _in_handler = 0;

    /* set low level timer */
    _schedule_earliest_lltimer(now);
}
