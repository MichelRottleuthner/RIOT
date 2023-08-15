/*
 * Copyright (C) 2021 HAW Hamburg
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @file
 * @brief       OpenDSME example
 *
 * @author      José I. Álamos <jose.alamos@haw-hamburg.de>
 */

#include <stdio.h>
#include <string.h>
#include "opendsme/opendsme.h"

#include "net/gnrc.h"
#include "net/gnrc/pktdump.h"
#include "net/gnrc/netreg.h"

#include "shell.h"

#define MAIN_QUEUE_SIZE     (8)
static msg_t _main_msg_queue[MAIN_QUEUE_SIZE];

int main(void)
{
#ifdef LA_PIN2
    gpio_init(LA_PIN2, GPIO_OUT);
    gpio_clear(LA_PIN2);
#endif

#ifdef LA_PIN3
    gpio_init(LA_PIN3, GPIO_OUT);
    gpio_clear(LA_PIN3);
#endif

#ifdef LA_PIN6
    gpio_init(LA_PIN6, GPIO_OUT);
    gpio_clear(LA_PIN6);
#endif

#ifdef LA_PIN12
    gpio_init(LA_PIN12, GPIO_OUT);
    gpio_clear(LA_PIN12);
#endif

#ifdef LA_PIN13
    gpio_init(LA_PIN13, GPIO_OUT);
    gpio_clear(LA_PIN13);
#endif

#ifdef LA_PIN14
    gpio_init(LA_PIN14, GPIO_OUT);
    gpio_clear(LA_PIN14);
#endif

#ifdef LA_PIN15
    gpio_init(LA_PIN15, GPIO_OUT);
    gpio_clear(LA_PIN15);
#endif

    msg_init_queue(_main_msg_queue, MAIN_QUEUE_SIZE);

    gnrc_netreg_entry_t dump = GNRC_NETREG_ENTRY_INIT_PID(GNRC_NETREG_DEMUX_CTX_ALL,
                                                          gnrc_pktdump_pid);
    gnrc_netreg_register(GNRC_NETTYPE_UNDEF, &dump);

    char line_buf[SHELL_DEFAULT_BUFSIZE];
    shell_run(NULL, line_buf, SHELL_DEFAULT_BUFSIZE);
    return 0;
}
