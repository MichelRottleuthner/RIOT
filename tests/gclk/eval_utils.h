/*
 * Copyright (C) 2020 Michel Rottleuthner <mihel.rottleuthner@haw-hamburg.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 *
 *
 * @file
 * @brief       Utility definitiions to instrument an application for external
 *              measurements.
 *
 * @author      Michel Rottleuthner <mihel.rottleuthner@haw-hamburg.de>
 */

#ifndef EVAL_UTIL_H
#define EVAL_UTIL_H

#include "xtimer.h"

#ifdef __cplusplus
extern "C" {
#endif

/* tell the synchronous trigger model to start measuring */
#define NOTIFY_START_TO_DMM gpio_set(DMM_TRIGGER_PIN_START)
/* tell the synchronous trigger model to stop measuring (and restore the pin states) */
#define NOTIFY_STOP_TO_DMM  gpio_set(DMM_TRIGGER_PIN_STOP); xtimer_usleep(10 * 1000); gpio_clear(DMM_TRIGGER_PIN_START); gpio_clear(DMM_TRIGGER_PIN_STOP)


/* wait for the DMM to signal it's ready for the next iteration
   ...and also wait till it enabled the barrier again so we can guarantee
   A) the trigger model is up and running again before we continue
   B) we don't skip through very short tasks because the SYNC pin is still signaling "READY" */
#define WAIT_FOR_DMM_READY  while (gpio_read(DMM_SYNC_PIN)!= 0) {}; while (gpio_read(DMM_SYNC_PIN) == 0) {}

int _sc_gpio_test(int argc, char **argv);
int _sc_work(int argc, char **argv);
int _sc_blink(int argc, char **argv);
int _sc_ctl_idle_timer(int argc, char **argv);
int _sc_timer_test(int argc, char **argv);
int _sc_clk_pin(int argc, char **argv);
int _sc_cmdlist_exe(int argc, char **argv);
int _sc_cmdlist_add(int argc, char **argv);
int _sc_TCI(int argc, char **argv);
int _sc_ACGI(int argc, char **argv);
int _sc_sleep(int argc, char **argv);
int _sc_test_time_trace(int argc, char **argv);

#ifdef __cplusplus
}
#endif

#endif /* EVAL_UTIL_H */
/** @} */
