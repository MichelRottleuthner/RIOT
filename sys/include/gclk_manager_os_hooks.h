/*
 * Copyright (C) 2021 HAW Hamburg <michel.rottleuthner@haw-hamburg.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */
/**
 * @ingroup     sys_gclk
 *
 * @{
 *
 * @file
 * @brief       OS hook interface for generic clock config manager
 * @brief       Interface for clock manager OS integration
 *
 * @author      Michel Rottleuthner <michel.rottleuthner@haw-hamburg.de>
 */
#ifndef GCLK_MANAGER_OS_HOOKS_H
#define GCLK_MANAGER_OS_HOOKS_H

#include "gclk.h"
#include "list.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * @brief Hook to be called when the scheduler switches to idle
 * @note  This should be called as soon as the scheduler 'detects' idle condition, i.e., if there
 *        is no other thread ready. It is important that this is called *before* any low power mode
 *        or wait-for-interrupt instruction is executed. Think of it as *idle time starts now* callback */
void gclk_manager_on_idle_hook(void);

/*
 * @brief Hook to be called after the idle
 * @note  This should be called as soon as the idle condition is left, i.e., before the scheduler continues
 *        to run the next thread. Think of it as *idle time ends now* callback */
void gclk_manager_post_idle_hook(void);

/*
 * @brief Hook to be called before the next thread is scheduled
 * @note  This should be called directly before the scheduler continues
 *        to run the next thread. */
void gclk_manager_pre_sched_hook(kernel_pid_t next_thread);

/*
 * @brief Hook to be called before the next thread is scheduled
 * @note  This should be called directly after a thread was descheduled. */
void gclk_manager_post_sched_hook(kernel_pid_t desched_thread);

#ifdef __cplusplus
}
#endif

#endif /* GCLK_MANAGER_OS_HOOKS_H */
/**
 * @}
 */
