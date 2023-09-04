/*
 * Copyright (C) 2015 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     examples
 * @{
 *
 * @file
 * @brief       Example application for demonstrating the RIOT network stack
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 *
 * @}
 */

#include <stdio.h>

#include "shell.h"
#include "msg.h"
#include "periph/gpio.h"
#include "net/gnrc/netapi.h"
#include "net/gnrc/netif.h"
#include "periph/cpuid.h"

#define MAIN_QUEUE_SIZE     (8)
static msg_t _main_msg_queue[MAIN_QUEUE_SIZE];

/* defined in sched.c */
extern bool _is_coordinator;

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

#ifdef LA_PIN4
    gpio_init(LA_PIN4, GPIO_OUT);
    gpio_clear(LA_PIN4);
#endif

#ifdef LA_PIN6
    gpio_init(LA_PIN6, GPIO_OUT);
    gpio_clear(LA_PIN6);
#endif

#ifdef LA_PIN7
    gpio_init(LA_PIN7, GPIO_OUT);
    gpio_clear(LA_PIN7);
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

    /* we need a message queue for the thread running the shell in order to
     * receive potentially fast incoming networking packets */
    msg_init_queue(_main_msg_queue, MAIN_QUEUE_SIZE);
    puts("RIOT network stack example application");

    gnrc_netif_t *netif = gnrc_netif_iter(NULL);
    kernel_pid_t pid = netif->pid;

    netopt_enable_t gts_en = NETOPT_ENABLE;
    int res = gnrc_netapi_set(pid, NETOPT_GTS_TX, 0, &gts_en, sizeof(netopt_enable_t));
    if (res != sizeof(netopt_enable_t)) {
        printf("could not set GTS: %d\n", res);
    }

    netopt_enable_t ack_req_en = NETOPT_ENABLE;

    res = gnrc_netapi_set(pid, NETOPT_ACK_REQ, 0, &ack_req_en, sizeof(netopt_enable_t));
    if (res != sizeof(netopt_enable_t)) {
        printf("could not set ACK_REQ: %d\n", res);
    }

    /* The node with ths CPU ID will be used as cordinator, all other nodes will act as RFD */
    uint8_t coord_cpuid[CPUID_LEN] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC};

    uint8_t cpuid[CPUID_LEN];
    cpuid_get(&cpuid);
    for (unsigned i = 0; i < CPUID_LEN; i++) {
        printf("%02x", cpuid[i]);
    }
    _is_coordinator = memcmp(coord_cpuid, cpuid, CPUID_LEN) == 0;
    printf("%s\n", _is_coordinator ? "--> COORD" : "--> RFD");

    netopt_enable_t pan_coord_en = NETOPT_ENABLE;
    if (_is_coordinator) {
        res = gnrc_netapi_set(netif->pid, NETOPT_PAN_COORD, 0, &pan_coord_en, sizeof(netopt_enable_t));
        if (res != sizeof(netopt_enable_t)) {
            printf("could not set PAN_COORD: %d\n", res);
        }
    }

    netopt_enable_t link_en = NETOPT_ENABLE;
    res = gnrc_netapi_set(netif->pid, NETOPT_LINK, 0, &link_en, sizeof(netopt_enable_t));
    if (res != sizeof(netopt_enable_t)) {
        printf("could not set LINK: %d\n", res);
    }

    /* start shell */
    puts("All up, running the shell now");
    char line_buf[SHELL_DEFAULT_BUFSIZE];
    shell_run(NULL, line_buf, SHELL_DEFAULT_BUFSIZE);

    /* should be never reached */
    return 0;
}
