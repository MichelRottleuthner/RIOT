/*
 * Copyright (C) 2015-17 Freie Universität Berlin
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
 * @brief       Demonstrating the sending and receiving of UDP data
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @author      Martine Lenders <m.lenders@fu-berlin.de>
 *
 * @}
 */
#include <inttypes.h>
#include <stdio.h>

#include <errno.h>
#include "byteorder.h"
#include "thread.h"
#include "msg.h"
#include "net/gnrc/pktdump.h"
#include "net/gnrc.h"
#include "net/icmpv6.h"
#include "net/ipv6/addr.h"
#include "net/ipv6/hdr.h"
#include "net/tcp.h"
#include "net/udp.h"
#include "net/sixlowpan.h"
#include "od.h"

#include "net/gnrc/ipv6.h"
#include "net/gnrc/netif.h"
#include "net/gnrc/netif/hdr.h"
#include "net/gnrc/udp.h"
#include "timex.h"
#include "utlist.h"
#include "xtimer.h"
#include "eval_utils.h"

#define PKTCNT_MSG_QUEUE_SIZE (32)

/**
 * @brief   PID of the pktdump thread
 */
kernel_pid_t gnrc_pktcnt_pid = KERNEL_PID_UNDEF;
/**
 * @brief   Stack for the pktdump thread
 */
static char _stack[THREAD_STACKSIZE_MAIN];

static unsigned pin_sync_cnt = 0;
static unsigned pktcnt = 0; 
static unsigned pktbytecnt = 0; 
static ipv6_addr_t sender_addr;

static void *_pktcnt_eventloop(void *arg)
{
    (void)arg;
    msg_t msg;
    msg_t msg_queue[PKTCNT_MSG_QUEUE_SIZE];
 
    /* setup the message queue */
    msg_init_queue(msg_queue, PKTCNT_MSG_QUEUE_SIZE);

    while (1) {
        
        msg_receive(&msg);
        gnrc_pktsnip_t *snip;
        switch (msg.type) {
            case GNRC_NETAPI_MSG_TYPE_RCV:
                snip = msg.content.ptr;
                size_t size = snip->size;

                gnrc_pktsnip_t *ipv6 = gnrc_pktsnip_search_type(snip, GNRC_NETTYPE_IPV6);
                /* extract header */
                ipv6_hdr_t *hdr = (ipv6_hdr_t *)ipv6->data;
                
                
                //char addr_str[IPV6_ADDR_MAX_STR_LEN];
                //printf("source address: %s\n", ipv6_addr_to_str(addr_str, &hdr->src, sizeof(addr_str)));
                
                /* only count the packets of the correct sender just to be sure */
                if (ipv6_addr_equal(&hdr->src, &sender_addr)){
                    if (pktcnt == 0) {
                        NOTIFY_START_TO_DMM;
                    }
                    pktbytecnt += size;
                    pktcnt++;

                    /* the marker is set *after the packet went through the stack so we wait for the next bigger count */
                    if (pktcnt == (pin_sync_cnt + 1)) {
                        NOTIFY_STOP_TO_DMM;
                        printf("recv complete! (%d pkts %d bytes)\n", pktcnt - 1, pktbytecnt);
                    }
                }

                gnrc_pktbuf_release(snip);
                break;
            default:
                puts("PKTCNT: received something unexpected");
                break;
        }
        
    }

    /* never reached */
    return NULL;
}

static gnrc_netreg_entry_t server = GNRC_NETREG_ENTRY_INIT_PID(GNRC_NETREG_DEMUX_CTX_ALL,
                                                               KERNEL_PID_UNDEF);

static void send(char *addr_str, char *port_str, uint32_t len, unsigned int num,
                 unsigned int delay)
{
    gnrc_netif_t *netif = NULL;
    char *iface;
    uint16_t port;
    ipv6_addr_t addr;
    
    uint8_t buf[len];

    gpio_init(OSZI_DEBUG_PIN, GPIO_OUT);
	  gpio_set(OSZI_DEBUG_PIN);
    printf("sending..\n");
	  gpio_clear(OSZI_DEBUG_PIN);
    NOTIFY_START_TO_DMM;
    
    iface = ipv6_addr_split_iface(addr_str);
    if ((!iface) && (gnrc_netif_numof() == 1)) {
        netif = gnrc_netif_iter(NULL);
    }
    else if (iface) {
        netif = gnrc_netif_get_by_pid(atoi(iface));
    }

    /* parse destination address */
    if (ipv6_addr_from_str(&addr, addr_str) == NULL) {
        puts("Error: unable to parse destination address");
        goto out;
    }
    /* parse port */
    port = atoi(port_str);
    if (port == 0) {
        puts("Error: unable to parse destination port");
        goto out;
    }

    for (unsigned int i = 0; i < num; i++) {
        gnrc_pktsnip_t *payload, *udp, *ip;
        //unsigned payload_size;
        /* allocate payload */
        payload = gnrc_pktbuf_add(NULL, buf, len, GNRC_NETTYPE_UNDEF);

        if (payload == NULL) {
            puts("Error: unable to copy data to packet buffer");
            goto out;
        }
        /* store size for output */
        //payload_size = (unsigned)payload->size;
        /* allocate UDP header, set source port := destination port */
        udp = gnrc_udp_hdr_build(payload, port, port);
        if (udp == NULL) {
            puts("Error: unable to allocate UDP header");
            gnrc_pktbuf_release(payload);
            goto out;
        }
        /* allocate IPv6 header */
        ip = gnrc_ipv6_hdr_build(udp, NULL, &addr);
        if (ip == NULL) {
            puts("Error: unable to allocate IPv6 header");
            gnrc_pktbuf_release(udp);
            goto out;
        }
        /* add netif header, if interface was given */
        if (netif != NULL) {
            gnrc_pktsnip_t *netif_hdr = gnrc_netif_hdr_build(NULL, 0, NULL, 0);

            gnrc_netif_hdr_set_netif(netif_hdr->data, netif);
            ip = gnrc_pkt_prepend(ip, netif_hdr);
        }

        if (i == num-1) {
            /* register for returned packet status */
            if (gnrc_neterr_reg(payload) != 0) {
                puts("Can not register for error reporting");
            }
        }

        /* send packet */
        if (!gnrc_netapi_dispatch_send(GNRC_NETTYPE_UDP, GNRC_NETREG_DEMUX_CTX_ALL, ip)) {
            puts("Error: unable to locate UDP thread");
            gnrc_pktbuf_release(ip);
            goto out;
        }
    
        /* access to `payload` was implicitly given up with the send operation above
         * => use temporary variable for output */
        //printf("Success: sent %u byte(s) to [%s]:%u\n", payload_size, addr_str,port);
        if (delay > 0) {
            xtimer_usleep(delay);
        }
    }

    //while (num--) {
        msg_t msg;
        /* wait for packet status and check */
        msg_receive(&msg);
        if ((msg.type != GNRC_NETERR_MSG_TYPE) ||
                (msg.content.value != GNRC_NETERR_SUCCESS)) {
            puts("Error sending packet");
        }

    //}
out: 
    gpio_set(OSZI_DEBUG_PIN);
    NOTIFY_STOP_TO_DMM;
}

kernel_pid_t gnrc_pktcnt_init(void)
{
    if (gnrc_pktcnt_pid == KERNEL_PID_UNDEF) {
        gnrc_pktcnt_pid = thread_create(_stack, sizeof(_stack), GNRC_PKTDUMP_PRIO,
                             THREAD_CREATE_STACKTEST,
                             _pktcnt_eventloop, NULL, "pktcnt");
    }
    return gnrc_pktcnt_pid;
}

static void start_server(char *port_str)
{
    uint16_t port;

    /* check if server is already running */
    if (server.target.pid != KERNEL_PID_UNDEF) {
        printf("Error: server already running on port %" PRIu32 "\n",
               server.demux_ctx);
        return;
    }

    /* parse port */
    port = atoi(port_str);
    if (port == 0) {
        puts("Error: invalid port specified");
        return;
    }
    
    gnrc_pktcnt_pid = gnrc_pktcnt_init();
    /* start server (which means registering pktdump for the chosen port) */
    server.target.pid = gnrc_pktcnt_pid;
    server.demux_ctx = (uint32_t)port;
    gnrc_netreg_register(GNRC_NETTYPE_UDP, &server);
    printf("Success: started UDP server on port %" PRIu16 "\n", port);
}

static void stop_server(void)
{
    /* check if server is running at all */
    if (server.target.pid == KERNEL_PID_UNDEF) {
        printf("Error: server was not running\n");
        return;
    }
    /* stop server */
    gnrc_netreg_unregister(GNRC_NETTYPE_UDP, &server);
    server.target.pid = KERNEL_PID_UNDEF;
    puts("Success: stopped UDP server");
}

int udp_cmd(int argc, char **argv)
{
    if (argc < 2) {
        printf("usage: %s [send|server]\n", argv[0]);
        return 1;
    }

    if (strcmp(argv[1], "send") == 0) {
        uint32_t len = 0;
        uint32_t num = 1;
        uint32_t delay = 1000000;
        if (argc < 5) {
            printf("usage: %s send <addr> <port> <data_len> [<num> [<delay in us>]]\n",
                   argv[0]);
            return 1;
        }
        if (argc > 4) {
            len = atoi(argv[4]);
        }
        if (argc > 5) {
            num = atoi(argv[5]);
        }
        if (argc > 6) {
            delay = atoi(argv[6]);
        }
        send(argv[2], argv[3], len, num, delay);
    }
    else if (strcmp(argv[1], "server") == 0) {
        if (argc < 3) {
            printf("usage: %s server [start|stop]\n", argv[0]);
            return 1;
        }
        if (strcmp(argv[2], "start") == 0) {
            if (argc < 5) {
                printf("usage %s server start <port> [<pin_sync_cnt> <sender_addr>]\n", argv[0]);
                return 1;
            }

            if (argc == 6) {
                pin_sync_cnt = atoi(argv[4]);
                ipv6_addr_from_str(&sender_addr, argv[5]);
                printf("only listening to pkts from %s\n", argv[5]);
            }
            pktcnt = 0;
            pktbytecnt = 0;
            start_server(argv[3]);
        }
        else if (strcmp(argv[2], "stop") == 0) {
            stop_server();
        }
        else {
            puts("error: invalid command");
        }
    }
    else {
        puts("error: invalid command");
    }
    return 0;
}
