/*
 * Copyright (C) 2018 HAW-Hamburg
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     tests
 * @{
 *
 * @file
 * @brief       Smoke test for ram sizes defined by the linker
 *
 * @author      Michel Rottleuthner <michel.rottleuthner@haw-hamburg.de>
 *
 * @}
 */

#include <stdio.h>

extern uint32_t _sram;
extern uint32_t _eram;

int main(void)
{
    printf("ram start @0x%p\n", &_sram);
    printf("ram end   @0x%p\n", &_eram);

    printf("last value in ram @0x%p: 0x%08lX\n", &_eram - 1, *(&_eram - 1));

    /* if the ram size definition is incorrect it is highly propable that the
    following line is not reached due to access to invalid adress space above.
    THOUGH, THERE IS NO GUARANTEE THAT A WRONG CONFIGURATION FORCES CRASH
    ON EVERY PLATFORM */
    printf("[SUCCESS]\n");

    return 0;
}
