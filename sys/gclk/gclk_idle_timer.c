#include <stdint.h>
#include "gclk_idle_timer.h"
#include "mutex.h"
#include "periph/rtt.h"
#define LOG_LEVEL LOG_NONE
#include "log.h"
#include "irq.h"

volatile bool idle_timer_scheduled   = false;
volatile bool overflow_already_handled = false;
volatile uint64_t idle_timer_target;

/* now base overlaps with the MSB of the low-level counter to reliably detect overflows even when delayed thru irq_disable() */
volatile uint64_t idle_timer_now_base = 0;

mutex_t idle_timer_mutex;

void idle_timer_alarm_cb(void *arg) {
    (void)arg;
    mutex_unlock(&idle_timer_mutex);
    LOG_DEBUG("idle_timer_alarm_cb\n");
    idle_timer_scheduled = false;
}

uint32_t _timer_mask(uint32_t val) {
    return val &IDLE_TMR_LL_TIMER_MAX;
}

static void _set_scheduled_timer(void) {
    /* if target is now within range of the next period ... */
    if (idle_timer_scheduled && _within_this_period(idle_timer_target, idle_timer_now_base)) {
        rtt_set_alarm(idle_timer_target - idle_timer_now_base, idle_timer_alarm_cb, NULL);
        LOG_DEBUG("scheduled timer\n");
    } else {
        /* wait for next overflow */
        LOG_DEBUG("not yet in the target period.. target: 0x%lx now: 0x%lx\n",  (uint32_t)idle_timer_target, (uint32_t)idle_timer_now_base);
    }
}

void idle_timer_overflow_cb(void *arg) {
    (void)arg;
    LOG_DEBUG("idle_timer_overflow_cb\n");

    if (!overflow_already_handled) {
        uint32_t timer_cnt = rtt_get_counter();
        uint32_t elapsed = _timer_mask(timer_cnt - _timer_mask((uint32_t)idle_timer_now_base));

        /* if the ols time_base was also already calculated on overflow elapsed will wrap to zero, indicating we need to add
           a full period */
        if (!elapsed) {
            elapsed += IDLE_TMR_LL_TIMER_MAX + 1;
        }

        idle_timer_now_base += elapsed;

        LOG_DEBUG("handle Overflow in ISR! (@%lu) elapsed: %lu\n", timer_cnt, elapsed);
    } else {
        LOG_DEBUG("overflow was already handled\n");
    }

    /* in case a future timer is scheduled set it now */
    _set_scheduled_timer();

    overflow_already_handled = false;

    /* we already handled the overflow before ISR could be entered so do nothing */
}

uint64_t idle_timer_read(void) {
    //   0b1011 0000
    //         +0111
    // = 0b1011 0111

    //   0b1011 1000
    //         +0111
    // = 0b1011 1111

    //   0b1011 1111
    //         +1001
    // = 0b1011 1111

    uint32_t state  = irq_disable();
    uint32_t timer_cnt = rtt_get_counter();
    uint32_t ll_base = (idle_timer_now_base & IDLE_TMR_LL_TIMER_MAX);

    if (ll_base > timer_cnt) {
        LOG_DEBUG("Handle overflow in read! %lx > %lx\n", ll_base, timer_cnt);
        /* an overflow event was missed so we increment by a period */
        /* the overflow event will still happen and adjust the time base accordingly */
        //uint32_t ovf_cnt = ll_base - ll_cnt;
        uint32_t elapsed = _timer_mask(timer_cnt - _timer_mask((uint32_t)idle_timer_now_base));
        idle_timer_now_base += elapsed;
        /* don't handle the same overflow via the handler as we already did that here */
        overflow_already_handled = true;
    } else {
        idle_timer_now_base = (idle_timer_now_base - ll_base) + timer_cnt;
    }

    irq_restore(state);
    LOG_DEBUG("timer_cnt: %lu\n", timer_cnt);

    return idle_timer_now_base;
    //return rtt_get_counter();
}

void idle_timer_init(void) {
    /* only init rtt if it was not initialized by the system init
     * This is done to avoid a second call to init which might cause spurious interrupts */
    if (!MODULE_PERIPH_INIT_RTT) {
        rtt_init();
    }
    rtt_set_overflow_cb(idle_timer_overflow_cb, NULL);
    idle_timer_now_base = rtt_get_counter();
}

void idle_timer_disable(void) {
    rtt_poweroff();
}

void idle_timer_enable(void) {
    rtt_poweron();
}

void idle_timer_set(uint64_t abs_counter) {
    idle_timer_now_base = abs_counter;
    rtt_set_counter(abs_counter &IDLE_TMR_LL_TIMER_MAX);
}

void idle_timer_set_alarm(uint64_t absolute_target) {
    uint32_t state = irq_disable();
    uint64_t abs_now_ref = idle_timer_read();
    idle_timer_target = absolute_target;
    /* check if the alarm can already be written to hardware without waiting for an overflow first */
    if (_within_this_period(idle_timer_target, abs_now_ref)) {
        uint32_t rel_diff = idle_timer_target - abs_now_ref;
        uint32_t ll_target = ((abs_now_ref & IDLE_TMR_LL_TIMER_MAX) + rel_diff) & IDLE_TMR_LL_TIMER_MAX ;
        LOG_DEBUG("IDLE_TMR_LL_TIMER_MAX: %08lx\n", (uint32_t)IDLE_TMR_LL_TIMER_MAX);
        LOG_DEBUG("       extended now:   %08lx%08lx\n", (uint32_t)(abs_now_ref >> 32),
                                                         (uint32_t)(abs_now_ref & 0xFFFFFFFF));
        LOG_DEBUG("extended abs. alarm:   %08lx%08lx\n", (uint32_t)(idle_timer_target >> 32),
                                                         (uint32_t)(idle_timer_target & 0xFFFFFFFF));
        LOG_DEBUG("     relative alarm:   %016lx\n", rel_diff);
        LOG_DEBUG("  absolute ll alarm:   %016lx\n", ll_target);
        rtt_set_alarm(ll_target, idle_timer_alarm_cb, NULL);
    } else {
        /* wait for overflows until we can finally set the actual timer */
        idle_timer_scheduled = true;
    }

    irq_restore(state);
}

uint64_t idle_timer_read_alarm(void) {
    return idle_timer_target;
}

void idle_timer_clear_alarm(void) {
    idle_timer_scheduled = false;
    rtt_clear_alarm();
}

/* TODO: Currently this can only be used from a single callee at a time.
 * This should be reworked to a more sophisticated multiplexing if deemed needed */
void idle_timer_wait(uint32_t ms) {
    mutex_init(&idle_timer_mutex);
    mutex_lock(&idle_timer_mutex);
    uint64_t now_val = idle_timer_read();
    LOG_DEBUG("current: %08lx%08lx\n",(uint32_t)(now_val >> 32), (uint32_t)now_val & 0xFFFFFFFF); 
    uint64_t new_val = now_val + ((uint64_t)IDLE_TMR_LL_TIMER_FREQ * ms / 1000);
    LOG_DEBUG("setting: %08lx%08lx\n",(uint32_t)(new_val >> 32), (uint32_t)new_val & 0xFFFFFFFF);
    idle_timer_set_alarm(new_val);

    /* wait for the idle timer to unlock the mutex */
    mutex_lock(&idle_timer_mutex);
}
