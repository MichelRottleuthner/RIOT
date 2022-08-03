/*
 * Copyright (C) 2022 HAW Hamburg
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 *
 *
 * @file
 * @brief       Utility interface to control the MCU debug components.
 *
 * @author      Michel Rottleuthner <mihel.rottleuthner@haw-hamburg.de>
 */

#ifndef DBG_CONTROL_H
#define DBG_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

int dbg_control_sc(int argc, char **argv);

#ifdef __cplusplus
}
#endif

#endif /* DBG_CONTROL_H */
/** @} */

