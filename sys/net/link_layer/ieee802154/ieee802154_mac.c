/*
 * Copyright (C) 2022 HAW Hamburg
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @{
 *
 * @file
 * @author  Michel Rottleuthner <m.rottleuthner@haw-hamburg.de>
 */
#include <stdio.h>
#include "net/ieee802154_mac.h"
#include "net/gnrc/pktbuf.h"
#define ENABLE_DEBUG 0
#include "debug.h"
#include "periph/pm.h"
#include "event/thread.h"

//TODO: move function to mac code
extern void _build_mcps_data_request(gnrc_netif_t *netif, gnrc_pktsnip_t *pkt,
                              ieee802154_mcps_data_request_t *request);

//TODO: add aproper interface for this
void _netif_handover_mpdu(gnrc_netif_t *netif, gnrc_pktsnip_t *mpdu, gnrc_pktsnip_t *ieee802154_hdr);


void _handle_mac_cmd(ieee802154_mac_t *mac, gnrc_pktsnip_t *ieee802154_hdr, gnrc_pktsnip_t *mpdu);

extern void _print_pktsnip(gnrc_pktsnip_t *p, const char* prefix_str);

ieee802154_poll_state_t _mlme_poll_state_machine(ieee802154_mac_t *mac,
                                                ieee802154_mac_fsm_state_t state,
                                                ieee802154_event_t event);

ieee802154_data_state_t _mcps_data_state_machine(ieee802154_mac_t *mac,
                                                ieee802154_mac_fsm_state_t state,
                                                ieee802154_event_t event);

ieee802154_rx_idle_state_t _rx_idle_state_machine(ieee802154_mac_t *mac,
                                                  ieee802154_mac_fsm_state_t state,
                                                  ieee802154_event_t event);

ieee802154_norx_idle_state_t _norx_idle_state_machine(ieee802154_mac_t *mac,
                                                      ieee802154_mac_fsm_state_t state,
                                                      ieee802154_event_t event);

void _rx_data_offload_event_handler(event_t *event);
void _tx_done_offload_event_handler(event_t *event);

/* only exposed via driver instance */
void ieee802154_mac_rx_done_cb(ieee802154_mac_t *mac);
void ieee802154_mac_tx_done_cb(ieee802154_mac_t *mac, ieee802154_tx_done_info_t *info);

#define MAC_FSM_DEBUG (false)

ieee802154_driver_t _ieee802154_mac_driver = {
    .rx_done_cb = ieee802154_mac_rx_done_cb,
    .tx_done_cb = ieee802154_mac_tx_done_cb,
};

static const char* _poll_state2str(ieee802154_poll_state_t state)
{
    switch (state) {
       case POLL_INIT: return "POLL_INIT";
       case POLL_WAIT_FOR_ACK: return "POLL_WAIT_FOR_ACK";
       case POLL_WAIT_FOR_DATA: return "POLL_WAIT_FOR_DATA";
       case POLL_DONE: return "POLL_DONE";
    }
    return "POLL_STATE_INVALID";
}

static const char* _event2str(ieee802154_event_t event)
{
    switch (event) {
       case START: return "START";
       case MEDIUM_BUSY: return "MEDIUM_BUSY";
       case ACK_RECEIVED: return "ACK_RECEIVED";
       case ACK_TIMED_OUT: return "ACK_TIMED_OUT";
       case RX_DATA: return "RX_DATA";
       case TX_DONE: return "TX_DONE";
       case DATA_TIMED_OUT: return "DATA_TIMED_OUT";
       case RADIO_TURNED_ON: return "RADIO_TURNED_ON";
       case RADIO_TURNED_OFF: return "RADIO_TURNED_OFF";
    }
    return "STATE_INVALID";
}

static const char* _data_state2str(ieee802154_data_state_t state)
{
    switch (state) {
       case DATA_INIT: return "DATA_INIT";
       case DATA_WAIT_FOR_RADIO_ON: return "DATA_WAIT_FOR_RADIO_ON";
       case DATA_WAIT_FOR_ACK: return "DATA_WAIT_FOR_ACK";
       case DATA_DONE: return "DATA_DONE";
    }
    return "DATA_STATE_INVALID";
}

static const char* _rx_idle_state2str(ieee802154_rx_idle_state_t state)
{
    switch (state) {
       case RX_IDLE_INIT: return "RX_IDLE_INIT";
       case RX_IDLE_WAIT_FOR_RADIO_ON: return "RX_IDLE_WAIT_FOR_RADIO_ON";
       case RX_IDLE_WAIT_FOR_RX: return "RX_IDLE_WAIT_FOR_RX";
       case RX_IDLE_DONE: return "RX_IDLE_DONE";
    }
    return "RX_IDLE_STATE_INVALID";
}

static const char* _norx_idle_state2str(ieee802154_norx_idle_state_t state)
{
    switch (state) {
       case NORX_IDLE_INIT: return "NORX_IDLE_INIT";
       case NORX_IDLE_DONE: return "NORX_IDLE_DONE";
       case NORX_IDLE_WAIT_FOR_RADIO_OFF: return "NORX_IDLE_WAIT_FOR_RADIO_OFF";
    }
    return "NORX_IDLE_STATE_INVALID";
}

uint8_t _get_seq_nr(ieee802154_msdu_t *msdu) {
     return ((uint8_t*)(msdu->pkt->data))[2];
}

uint8_t _get_mhr_seq_nr(gnrc_pktsnip_t *mhr) {
     return ((uint8_t*)(mhr->data))[2];
}

static void _setup_current_mac_request(ieee802154_mac_t *mac, ieee802154_req_meta_t *rm)
{
    if (rm->type == REQ_MLME_POLL) {
        mac->current_req.poll_request = (ieee802154_mlme_poll_request_t*)rm;
        mac->fsm_handler = (ieee802154_fsm_handler_t)((void*)_mlme_poll_state_machine);
        mac->fsm_state.poll_state = POLL_INIT;
    } else if (rm->type == REQ_MCPS_DATA) {
        mac->current_req.data_request = (ieee802154_mcps_data_request_t*)rm;
        mac->fsm_handler = (ieee802154_fsm_handler_t)((void*)_mcps_data_state_machine);
        mac->fsm_state.data_state = DATA_INIT;
    }
}

void _print_data_request(ieee802154_mcps_data_request_t *r)
{
    printf("-------vvvvvv\n");
    printf("MCPS-DATA.request:\n");
    printf("src_addr_mode: %d\n", r->src_addr_mode);
    printf("dst_addr_mode: %d\n", r->dst_addr_mode);
    printf("dst_pan_id: 0x%02x%02x\n", ((uint8_t*)&r->dst_pan_id)[1],
                                     ((uint8_t*)&r->dst_pan_id)[0]);
    printf("dst_address: ");
    for (unsigned i = 0; i < r->dst_address.l2addr_len; i++) {
        printf("%02x ", r->dst_address.l2addr[i]);
    }
    printf("\nmsdu.pkt: %p\n", r->msdu.pkt);
    printf("ack_tx: %d\n", r->ack_tx);
    printf("indirect_tx: %d\n", r->indirect_tx);
    printf("security_level: %d\n", r->security_level);
    printf("-------^^^^^^\n");
}

ieee802154_rx_offload_entry_t* _dequeue_pending_rx_entry(ieee802154_mac_t *mac)
{
    ieee802154_rx_offload_entry_t *roe = NULL;
    mutex_lock(&mac->rx_offload_lock);
    clist_node_t *n = clist_lpop(&mac->pending_rx_buffers);
    n->next = NULL;
    if (n) {
        roe = container_of(n, ieee802154_rx_offload_entry_t, node);
    }
    mutex_unlock(&mac->rx_offload_lock);
    return roe;
}

void _queue_pending_rx_entry(ieee802154_mac_t *mac, ieee802154_rx_offload_entry_t *roe)
{
    mutex_lock(&mac->rx_offload_lock);
    clist_rpush(&mac->pending_rx_buffers, &roe->node);
    mutex_unlock(&mac->rx_offload_lock);
}

void _free_rx_offload_entry(ieee802154_mac_t *mac, ieee802154_rx_offload_entry_t *rxoe)
{
    mutex_lock(&mac->rx_offload_lock);
    rxoe->node.next = NULL;
    rxoe->mpdu = NULL;
    rxoe->ieee802154_hdr = NULL;

    clist_rpush(&mac->free_rx_buffers, &rxoe->node);
    mutex_unlock(&mac->rx_offload_lock);
}

extern void _poll_start_marker(void);
extern void _poll_stop_marker(void);

void _handle_fsm_event(ieee802154_mac_t *mac,
                       ieee802154_event_t event)
{
    if (mac->fsm_handler) {
        mac->fsm_state = mac->fsm_handler(mac, mac->fsm_state, event);
        if ((mac->fsm_handler == (ieee802154_fsm_handler_t)((void*)_mlme_poll_state_machine))
            && (mac->fsm_state.poll_state == POLL_DONE)) {
            _poll_stop_marker();
        }
        if (mac->fsm_state.data_state == DATA_DONE ||
            mac->fsm_state.poll_state == POLL_DONE ||
            mac->fsm_state.rx_idle_state == RX_IDLE_DONE ||
            mac->fsm_state.norx_idle_state == NORX_IDLE_DONE
            ) {
            if (MAC_FSM_DEBUG) {
                printf("SUBFSM.DONE\n");
            }

            mutex_lock(&mac->lock);
            /* do not re-restart the NORX idle state machine (it must only run once to turn of the radio) */
            if (mac->fsm_handler != (ieee802154_fsm_handler_t)(void*)_norx_idle_state_machine) {
                mac->fsm_handler = NULL;
                event_post(mac->event_queue, &mac->fsm_trigger_event);
            }
            mutex_unlock(&mac->lock);
        }
    } else {
        printf("got event (%s), but no FSM is active\n", _event2str(event));
    }
}

ieee802154_mac_request_mode_t _get_mac_current_mac_req_mode(ieee802154_mac_t *mac) {
    if (mac->fsm_handler == (void*)_mcps_data_state_machine) {
        return MAC_MCPS_DATA;
    } else if (mac->fsm_handler == (void*)_mlme_poll_state_machine) {
        return MAC_MLME_POLL;
    } else {
        return MAC_IDLE;
    }
}

const char* _mac_req_mode2str(ieee802154_mac_request_mode_t mode) {
    switch (mode) {
        case MAC_MCPS_DATA: return "MAC_MCPS_DATA";
        case MAC_MLME_POLL: return "MAC_MLME_POLL";
        case MAC_IDLE: return "MAC_IDLE";
    }
    return "MAC_INVALID_MODE";
}

gnrc_netif_t* _mac2netif(ieee802154_mac_t *mac)
{
    return container_of(mac, gnrc_netif_t, ieee802154_mac);
}

netdev_t* _mac2netdev(ieee802154_mac_t *mac)
{
    gnrc_netif_t *netif = _mac2netif(mac);
    return netif->dev;
}

ieee802154_mac_t* _netif2mac(void *netif)
{
    return &(((gnrc_netif_t*)netif)->ieee802154_mac);
}

netdev_ieee802154_t* _mac2netdev802154(ieee802154_mac_t *mac)
{
    netdev_t *d = _mac2netdev(mac);
    netdev_ieee802154_t *netdev_ieee802154 = container_of(d,
                                                          netdev_ieee802154_t,
                                                          netdev);
    return netdev_ieee802154;
}

static ieee802154_l2addr_t _test_node1_l2addr = {
    .l2addr = TEST_ENVIRONMENT_IEEE802154_ADDR1,
    .l2addr_len = 8,
};

static ieee802154_l2addr_t _test_node2_l2addr = {
    .l2addr = TEST_ENVIRONMENT_IEEE802154_ADDR2,
    .l2addr_len = 8,
};

ieee802154_l2addr_t* ieee802154_mac_get_coordinator_l2addr(ieee802154_mac_t *mac)
{
    return mac->coordinator_addr;
}

static void _sched_req(ieee802154_mac_t *mac, ieee802154_req_meta_t *rm)
{
    mutex_lock(&mac->lock); 
    clist_rpush(&mac->pending_mac_requests, &rm->next);
    mutex_unlock(&mac->lock);
    /* Kickstart the FSM in case it is idle. */
    event_post(mac->event_queue, &mac->fsm_trigger_event);
}

static void _fsm_trigger_event_handler(event_t *evp)
{
    ieee802154_mac_t *mac = container_of(evp, ieee802154_mac_t, fsm_trigger_event);

    if (_get_mac_current_mac_req_mode(mac) == MAC_IDLE) {
        /* only kick off FSM if not already active */
        clist_node_t *next_req = clist_lpop(&mac->pending_mac_requests);
        if (next_req) {
            /* if there is a pending request
             * set up the respecitve sub-fsm to serve them */
            ieee802154_req_meta_t *nrm = (ieee802154_req_meta_t*)next_req;
            _setup_current_mac_request(mac, nrm);
        } else if (mac->idle_listen_mode){
            /* Set to handler for RX IDLE state, the FSM will be
             * triggered again by some radio event. */
            mac->fsm_handler = (ieee802154_fsm_handler_t)((void*)_rx_idle_state_machine);
            mac->fsm_state.rx_idle_state = RX_IDLE_INIT;
        } else {
            /* if there is neither a pending request nor configured for
             * idle listening, switch to NORX fsm that turns off the radio.
             * After that the MAC waits for another event before continuing operations. */
            mac->fsm_handler = (ieee802154_fsm_handler_t)((void*)_norx_idle_state_machine);
            mac->fsm_state.norx_idle_state = NORX_IDLE_INIT;
        }
        _handle_fsm_event(mac, START);
    }
}

/* shall only be called by the radio owner (e.g., netif thread). */
void _radio_off_request_event_handler(event_t *event) {
    ieee802154_mac_t *mac = container_of(event, ieee802154_mac_t, radio_off_request_event);
    /* turn off radio */
    _control_radio_enable(mac, false);
    event_post(mac->event_queue, &mac->radio_turned_off_event);
}

void _radio_on_request_event_handler(event_t *event) {
    ieee802154_mac_t *mac = container_of(event, ieee802154_mac_t, radio_on_request_event);
    /* turn on radio */
    _control_radio_enable(mac, true);
    event_post(mac->event_queue, &mac->radio_turned_on_event);
}

void _radio_turned_off_event_handler(event_t *event) {
    ieee802154_mac_t *mac = container_of(event, ieee802154_mac_t, radio_turned_off_event);
    _handle_fsm_event(mac, RADIO_TURNED_OFF);
}

void _radio_turned_on_event_handler(event_t *event) {
    ieee802154_mac_t *mac = container_of(event, ieee802154_mac_t, radio_turned_on_event);
    _handle_fsm_event(mac, RADIO_TURNED_ON);
}

void _data_timeout_event_handler(event_t *event) {
    ieee802154_mac_t *mac = container_of(event, ieee802154_mac_t, data_timeout_event);

    _handle_fsm_event(mac, DATA_TIMED_OUT);
}

/* called from timer interrupt context */
void _mlme_poll_timeout(void *arg)
{
    ieee802154_mac_t *mac = (ieee802154_mac_t*)arg;

    /* check if the poll request is still active. (?)
     * If data was received before this callback was fired, this timeout
     * is obsolete. */

    /* offload the eent to the mac event queue */
    event_post(mac->event_queue, &mac->data_timeout_event);
}

#include "architecture.h"
event_queue_t event_queue;
static char WORD_ALIGNED event_thread_stack[IEEE802154_MAC_EVENT_THREAD_STACKSIZE];

static void *_event_handler_thread(void *arg)
{
    event_queue_t *q = (event_queue_t*)arg;

    event_queue_claim(q);
    /* start event loop */
    event_loop(q);

    /* should be never reached */
    return NULL;
}

void ieee802154_mac_init(ieee802154_mac_t *mac)
{
    memset(mac, 0, sizeof(ieee802154_mac_t));
    mutex_init(&mac->lock);
    mutex_init(&mac->rx_offload_lock);
    mutex_init(&mac->idtx_q_lock);

    mac->fsm_handler = NULL;
    /* Keep the radio on by default, only switch it off when configuring
     * the polling intervall.
     * TODO: maybe replace this with a separate FSM and a configurable
     * idle_fsm */
    mac->idle_listen_mode = true;
    mac->periodic_mlme_poll_enabled = false;
    mac->fsm_trigger_event.handler = _fsm_trigger_event_handler;
    mac->radio_turned_off_event.handler = _radio_turned_off_event_handler;
    mac->radio_turned_on_event.handler = _radio_turned_on_event_handler;
    mac->data_timeout_event.handler = _data_timeout_event_handler;
    mac->rx_data_offload_event.handler = _rx_data_offload_event_handler;
    mac->tx_done_offload_event.handler = _tx_done_offload_event_handler;

    mac->pending_idtx_pkt_cnt = 0;

    /* TODO those are teh defaults on the system used for testing,
     *      replace this with proper initial query code. */
    mac->report_pending_on_ack_enabled = false;
    mac->csma_is_enabled = true;
    mac->radio_is_on = true;

    /* stuff needed to interact with the radio */
    mac->radio_on_request_event.handler = _radio_on_request_event_handler;
    mac->radio_off_request_event.handler = _radio_off_request_event_handler;
    mac->radio_event_queue = &_mac2netif(mac)->evq[GNRC_NETIF_EVQ_INDEX_PRIO_LOW];

    mac->poll_timeout_us = 0;
    // TODO remove this (just here for testing purposes
    netdev_ieee802154_t *netdev_ieee802154 = _mac2netdev802154(mac);
    netdev_ieee802154->mac = mac; /**< TODO: temporary hack to get access to the mac instance from within the driver */
    ieee802154_l2addr_t local_l2_addr;
    memcpy(local_l2_addr.l2addr, netdev_ieee802154->long_addr, IEEE802154_LONG_ADDRESS_LEN);
    local_l2_addr.l2addr_len = IEEE802154_LONG_ADDRESS_LEN;

    /* TODO: just for testing, REMOVE later on */
    if (_l2addr_equals(&local_l2_addr, &_test_node1_l2addr)) {
        printf("this is node 1!\n");
        mac->coordinator_addr = &_test_node2_l2addr;
    } else if (_l2addr_equals(&local_l2_addr, &_test_node2_l2addr)) {
        printf("this is node 2!\n");
        mac->coordinator_addr = &_test_node1_l2addr;
    } else {
        printf("this is some random node!\n");
        mac->coordinator_addr = &_test_node1_l2addr;
    }

    memset(&mac->rx_buffer_pool, 0, sizeof(mac->rx_buffer_pool));
    mac->free_rx_buffers.next = NULL;
    mac->pending_rx_buffers.next = NULL;
    /* append all rx buffers of the pool to the list that holds empty entries */
    for (unsigned i = 0; i < IEEE802154_MAC_RX_BUF_NUMOF; i++) {
        clist_rpush(&mac->free_rx_buffers, &mac->rx_buffer_pool[i].node);
    }

    event_queue_init_detached(&event_queue);
    mac->event_queue = &event_queue;

    thread_create(event_thread_stack, IEEE802154_MAC_EVENT_THREAD_STACKSIZE,
                  IEEE802154_MAC_EVENT_THREAD_PRIO,
                  THREAD_CREATE_STACKTEST,
                  _event_handler_thread, &event_queue, "mac event thread");

    mac->driver = &_ieee802154_mac_driver;
    /* kick off FSM */
    event_post(mac->event_queue, &mac->fsm_trigger_event);
}

ieee802154_idtxq_t *_alloc_idtx_pktq(ieee802154_mac_t *mac, ieee802154_l2addr_t *addr)
{
    mutex_lock(&mac->idtx_q_lock);
    ieee802154_idtxq_t *pool = &mac->idtx_queue_pool[0];
    
    for (int i = 0; i < IEEE802154_MAC_IDTX_QUEUES_NUMOF; i++) {
        if (!pool[i].addr_data) {
            DEBUG("queue[%d] is free, allocate it for %p!\n", i, addr);
            pool[i].addr_data = addr;
            list_add(&mac->idtx_queues_list, &pool[i].node);
            mutex_unlock(&mac->idtx_q_lock);
            return &pool[i];
        }
        DEBUG("queue[%d] is already allocated!\n", i);
    }
    mutex_unlock(&mac->idtx_q_lock);
    return NULL;
}

ieee802154_idtx_req_q_entry_t *_alloc_idtx_req_entry(ieee802154_mac_t *mac, gnrc_pktsnip_t *pkt,
                                                     ieee802154_mcps_data_request_t *request)
{
    mutex_lock(&mac->idtx_q_lock);
    ieee802154_idtx_req_q_entry_t *pool = &mac->idtx_req_q_entry_pool[0];
    ieee802154_idtx_req_q_entry_t *free_entry = NULL;

    for (int i = 0; i < IEEE802154_MAC_IDTX_PKTS_NUMOF; i++) {
        if (pool[i].q_entry.pkt == NULL) {
            pool[i].q_entry.pkt = pkt;
            pool[i].req_ref = request;
            mac->pending_idtx_pkt_cnt++;
            free_entry = &pool[i];
            break;
        }
    }
    mutex_unlock(&mac->idtx_q_lock);
    return free_entry;
}

void _free_idtx_req_entry(ieee802154_mac_t *mac, ieee802154_idtx_req_q_entry_t *req_q_entry)
{
    //TODO as of now protected by mac lock, replace this with more specific locking
    /* setting the pkt to NULL marks the whole entry as free */
    req_q_entry->q_entry.pkt = NULL;
    mac->pending_idtx_pkt_cnt--;
}

/* returns the indirect transmission queue for a particular destination address.
 * @p addr can not be an arbitrary dynamic variable, more specifically it must
 * point to an entry of @ref ieee802154_l2addr_t.idtx_l2addrs */
gnrc_pktqueue_t **_get_idtxq(ieee802154_mac_t *mac,
                             ieee802154_l2addr_t *addr)
{
    list_node_t *list = &mac->idtx_queues_list;
    gnrc_pktqueue_t **qptr = NULL;
    DEBUG("get queue for addr @%p\n", addr);
    while (list->next) {
        ieee802154_idtxq_t *txq = container_of(list->next, ieee802154_idtxq_t, node);
        if (txq->addr_data == addr) {
            DEBUG("found queue @%p\n", txq);
            qptr = &txq->queue;
            break;
        } else {
            DEBUG("queue @%p is for dst %p not %p\n", txq, txq->addr_data, addr);
        }
        list = list->next;
    }
    return qptr;
}

bool _l2addr_equals(const ieee802154_l2addr_t *a, const ieee802154_l2addr_t *b)
{
    if ((a->l2addr_len == b->l2addr_len) &&
        (memcmp(a->l2addr, b->l2addr, a->l2addr_len) == 0)) {
       return true; 
    }

    return false;
}
                            

ieee802154_l2addr_t *_save_l2addr_for_idtx(ieee802154_mac_t *mac,
                                                  const uint8_t *dst_l2addr,
                                                  uint8_t dst_l2addr_len)
{
    ieee802154_l2addr_t *l2addrs = mac->idtx_l2addrs;
    ieee802154_l2addr_t *freel2addr = NULL; 
    
    ieee802154_l2addr_t dstl2addr = { .l2addr_len = dst_l2addr_len }; 
    memcpy(&dstl2addr, dst_l2addr, dst_l2addr_len);

    for (int i = 0; i < IEEE802154_MAC_IDTX_QUEUES_NUMOF; i++) {
        if (!freel2addr && !l2addrs[i].l2addr_len) {
            freel2addr = &l2addrs[i];
        } else if (_l2addr_equals(&l2addrs[i], &dstl2addr)) {
            return &l2addrs[i];
        }
    }
    
    if (freel2addr) {
        memcpy(freel2addr, dst_l2addr, dst_l2addr_len);
        freel2addr->l2addr_len = dst_l2addr_len;
        return freel2addr;
    }

    return NULL;
}

extern gnrc_netif_t* _mac2netif(ieee802154_mac_t *mac);

//TODO fix synchronization handling...
static void _config_ack_pending_bit(ieee802154_mac_t *mac, bool enable)
{
    //NOTE: can not call netif_set_opt from netif thread
    //netopt_enable_t netopt_en = enable ? NETOPT_ENABLE : NETOPT_DISABLE;

    //int r = netif_set_opt(&_mac2netif(mac)->netif, NETOPT_ACK_PENDING, 0, &netopt_en, sizeof(netopt_enable_t));
    //if (r != sizeof(netopt_enable_t)) {
    //    printf("ERROR setting NETOPT_ACK_PENDING\n");
    //}

    if (mac->report_pending_on_ack_enabled != enable) {
        netdev_t *dev = _mac2netdev(mac);
        netopt_enable_t netopt_en = enable ? NETOPT_ENABLE : NETOPT_DISABLE;
        int len = dev->driver->set(dev, NETOPT_ACK_PENDING, &netopt_en, sizeof(netopt_enable_t));
        if (len != sizeof(netopt_enable_t)) {
            printf("ERROR: couldn't set pending bit!\n");
        } else {
            /* save state so it is only changed if really neded */
            mac->report_pending_on_ack_enabled = enable;
        }
        //printf("%s pending\n", enable ? "enabled" : "disabled");
    }
}

/* returns the next packet that was intended to be sent to a node
 * that expects indirect transmissions.
 * @p addr can not be an arbitrary dynamic variable, more specifically it must
 * point to an entry of @ref ieee802154_l2addr_t.idtx_l2addrs */
bool _get_next_idtx_entry(ieee802154_mac_t *mac,
                          ieee802154_l2addr_t *addr,
                          gnrc_pktsnip_t **next_pkt_out,
                          ieee802154_mcps_data_request_t **req_ref)
{
    ieee802154_idtx_req_q_entry_t *re = NULL;
    mutex_lock(&mac->idtx_q_lock);
    
    gnrc_pktqueue_t **idtxq = _get_idtxq(mac, addr);

    if (idtxq) {
        DEBUG("there is a queue at %p for the requesting node\n", idtxq);
        gnrc_pktqueue_t *qelem = gnrc_pktqueue_remove_head(idtxq);

        if (qelem) {
            re = container_of(qelem, ieee802154_idtx_req_q_entry_t, q_entry);
            *next_pkt_out = qelem->pkt;
            *req_ref = re->req_ref;
            /* mark request entry as free */
            _free_idtx_req_entry(mac, re);
            /* check if there is another packet */
            if (*idtxq) {
                ((uint8_t*)(*next_pkt_out)->data)[0] |= IEEE802154_FCF_FRAME_PEND;
            }
            //TODO: possibly update seq number
            mutex_unlock(&mac->idtx_q_lock);
            return true;
        }
    }
    mutex_unlock(&mac->idtx_q_lock);
    return false;
}

int _build_data_request(netdev_ieee802154_t *state,
                       ieee802154_mlme_poll_request_t *request,
                       uint8_t *psdu)
{
    //le_uint16_t bcast = {.u16 = 0xFFFF};
    //le_uint16_t dst_addr = bcast;
    uint8_t seq = state->seq++;

    size_t src_len = IEEE802154_LONG_ADDRESS_LEN;
    uint8_t *src = state->long_addr;
    //uint8_t *src = NULL; /* src addr*/
    //uint8_t *dst = dst_addr.u8; /* dst addr*/
    //le_uint16_t src_pan = bcast;
    //le_uint16_t dst_pan = bcast;
    le_uint16_t src_pan = byteorder_htols(state->pan);
    le_uint16_t dst_pan = request->coord_pan_id;
    uint8_t flags = IEEE802154_FCF_TYPE_MACCMD | IEEE802154_FCF_ACK_REQ; /* equivalent to first byte of FCF */

    int res = ieee802154_set_frame_hdr(psdu, src, src_len,
                            request->coord_address.l2addr,
                            request->coord_address.l2addr_len,
                            src_pan, dst_pan, flags, seq);
    ///* above function doesn't allow ACK_REQ for bcast , force it anyway for testing */
    //psdu[0] |= IEEE802154_FCF_ACK_REQ;
    //assert(res == 15);
    if (res > 0) {
        psdu[res] = IEEE802154_MAC_CMD_DATA_REQUEST; 
        return res + 1;
    }

    return res;
}

typedef struct {
    ieee802154_mac_t *mac;
    iolist_t *iol;
    mutex_t sync;
    event_t evt;
    int res;
} send_offload_ctx_t;

/* to be called from radio context only! */
void _send_offload_event_handler(event_t *event) {
    //printf("offload evt..\n");
    send_offload_ctx_t *ctx = container_of(event, send_offload_ctx_t, evt);
    netdev_ieee802154_t *state = _mac2netdev802154(ctx->mac);
    netdev_t *dev = &state->netdev;
    ctx->res = dev->driver->send(dev, ctx->iol);
    mutex_unlock(&ctx->sync);
}

int _offload_send_blocking(ieee802154_mac_t *mac, iolist_t *iol)
{
    send_offload_ctx_t ctx;
    ctx.mac = mac;
    ctx.evt.list_node.next = NULL;
    ctx.evt.handler = _send_offload_event_handler;
    mutex_init(&ctx.sync);
    mutex_lock(&ctx.sync);
    ctx.iol = iol;
    event_post(mac->radio_event_queue, &ctx.evt);
    mutex_lock(&ctx.sync);
    return ctx.res;
}

int _send_data_request(ieee802154_mac_t *mac,
                       ieee802154_mlme_poll_request_t *request)
{
    /* Buffer to store the data request frame */
    uint8_t dr_psdu[32];

    netdev_ieee802154_t *state = _mac2netdev802154(mac);
    int len = _build_data_request(state, request, dr_psdu);
    //printf("DR %d\n", dr_psdu[2]);
    iolist_t iol = {.iol_base = dr_psdu, .iol_len = len};

    return _offload_send_blocking(mac, &iol);
}

void _control_radio_enable(ieee802154_mac_t *mac, bool on) {
    if (mac->radio_is_on != on) {
        netdev_t *dev = _mac2netdev(mac);
        netopt_state_t state = on ? NETOPT_STATE_IDLE : NETOPT_STATE_SLEEP;
        int len = dev->driver->set(dev, NETOPT_STATE, &state, sizeof(state));
        if (len != sizeof(state)) {
            printf("ERROR: couldn't set radio to sleep!\n");
        }
        mac->radio_is_on = on;
    }
}

void _control_radio_csma(ieee802154_mac_t *mac, bool on) {
    if (mac->csma_is_enabled != on) {
        netdev_t *dev = _mac2netdev(mac);
        netopt_enable_t netopt_en = on ? NETOPT_ENABLE : NETOPT_DISABLE;

        int len = dev->driver->set(dev, NETOPT_CSMA, &netopt_en, sizeof(netopt_en));
        if (len != sizeof(netopt_en)) {
            printf("ERROR: couldn't set CSMA!\n");
        }
        mac->csma_is_enabled = on;
    }
}

extern void _custom_event_cb(netdev_t *dev, netdev_event_t event);

void _trigger_single_data_request(ieee802154_mac_t *mac, unsigned timeout_us) {
    mac->poll_timeout_us = timeout_us;
    ieee802154_perform_poll_request(mac);
}

void _data_request_event_handler(event_t *event) {
    _poll_start_marker();
    ieee802154_mac_t *mac = container_of(event, ieee802154_mac_t, data_request_event);
    ieee802154_perform_poll_request(mac);
}

static void _ieee802154_mlme_poll_confirm_cb(ieee802154_mac_t *mac,
                                             ieee802154_mlme_poll_confirm_t *confirm)
{
    (void)mac;
    switch (confirm->status) {
        case MLME_SUCCESS:
            DEBUG("MLME-POLL.confirm(SUCCESS)\n");
            break;
        case MLME_CHANNEL_ACCESS_FAILURE:
            DEBUG("MLME-POLL.confirm(CHANNEL_ACCESS_FAILURE)\n");
            break;
        case MLME_NO_DATA:
            DEBUG("MLME-POLL.confirm(NO_DATA)\n");
            break;
        case MLME_NO_ACK:
            DEBUG("MLME-POLL.confirm(NO_ACK)\n");
            break;
        default:
            DEBUG("MLME-POLL.confirm(OTHER)");
    }
}

void ieee802154_enable_periodic_data_request(ieee802154_mac_t *mac, unsigned poll_ms, unsigned timeout_us) {
    mac->data_request_event.handler = _data_request_event_handler;

    mac->poll_timeout_us = timeout_us;

    /* don't go to listen mode when there is nothing to do */
    mac->idle_listen_mode = false;
    mac->periodic_mlme_poll_enabled = true;

    if (poll_ms > 0) {
        event_periodic_init(&mac->periodic_data_request_event, ZTIMER_MSEC,
                            mac->event_queue,
                            &mac->data_request_event);
        event_periodic_start(&mac->periodic_data_request_event, poll_ms);
        /* post the first event immediately to go to sleep ASAP */
        event_post(mac->event_queue, &mac->data_request_event);
    } else {
        event_periodic_stop(&mac->periodic_data_request_event);
    }
}

void ieee802154_set_idle_listening(ieee802154_mac_t *mac, bool enabled) {
    /* don't go to listen mode when there is nothing to do */
    mac->idle_listen_mode = enabled;
    /* kick off FSM in case it is inactive */
    event_post(mac->event_queue, &mac->fsm_trigger_event);
}

static inline size_t _addr_mode_to_addr_len(ieee802154_addr_mode_t addr_mode)
{
    if (addr_mode == IEEE802154_ADDR_MODE_SHORT) {
        return IEEE802154_SHORT_ADDRESS_LEN;
    }

    return IEEE802154_LONG_ADDRESS_LEN;
}

#define BUILD_HEADER_FAILED (-1)

int ieee802154_build_mac_data_frame(ieee802154_mac_t *mac,
                                     ieee802154_mcps_data_request_t *request,
                                     gnrc_pktsnip_t **out_frame)
{
#if IS_USED(MODULE_IEEE802154_SECURITY)
    size_t mac_header_size = IEEE802154_MAX_HDR_LEN + IEEE802154_SEC_MAX_AUX_HDR_LEN;
#else
    size_t mac_header_size = IEEE802154_MAX_HDR_LEN;
#endif

    gnrc_pktsnip_t *mac_header_snip = gnrc_pktbuf_add(NULL, NULL, mac_header_size, GNRC_NETTYPE_UNDEF);
    if (mac_header_snip == NULL) {
        DEBUG("ieee802154_build_mac_data_frame: cannot allocate pktsnip.\n");
        return -IEEE802154_NOMEM;
    }

    // TODO: set flags according to request values
    // TODO: the calling function must ensure translation from netdev_ieee802154_t->flags to request values
    // TODO: decide how to set IEEE802154_FCF_FRAME_PEND, as the pure request structure does not have a member for it
    uint8_t flags = IEEE802154_FCF_TYPE_DATA;

    gnrc_netif_t *netif = container_of(mac, gnrc_netif_t, ieee802154_mac);
    netdev_t *netdev = netif->dev;
    netdev_ieee802154_t *netdev_ieee802154 = (netdev_ieee802154_t*)netdev;
         
    size_t src_len, dst_len, mhr_len;
    const uint8_t *src = NULL;
    int res = 0;

    if (request->ack_tx) {
        flags |= IEEE802154_FCF_ACK_REQ;
    }

    dst_len = _addr_mode_to_addr_len(request->dst_addr_mode);

    // TODO: get proper (maybe different) pan id from the mac handle
    le_uint16_t src_pan = request->dst_pan_id;
    le_uint16_t dst_pan = request->dst_pan_id;
    
    if(request->security_level > IEEE802154_SEC_SCF_SECLEVEL_NONE) {
        src_len = IEEE802154_LONG_ADDRESS_LEN;
        src = netdev_ieee802154->long_addr;
    } else {
        src_len = _addr_mode_to_addr_len(request->src_addr_mode);
        if (request->src_addr_mode == IEEE802154_ADDR_MODE_SHORT) {
            src = netdev_ieee802154->short_addr;
        } else {
            src = netdev_ieee802154->long_addr;
        }
    }

    /* fill MAC header, seq should be set by device */
    if ((res = ieee802154_set_frame_hdr(mac_header_snip->data, src, src_len,
                                        request->dst_address.l2addr,
                                        dst_len, src_pan,
                                        dst_pan, flags, netdev_ieee802154->seq++)) == 0) {
        DEBUG("_send_ieee802154: Error preperaring frame\n");
        //TODO ensure this gets called after return gnrc_pktbuf_release(pkt)
        return BUILD_HEADER_FAILED;
    }
    mhr_len = res;
    
    //TODO: check if it is more elegant to predetermine the size
    //      and avoid realloc
    /* shrink pktsnip to actually required size */
    gnrc_pktbuf_realloc_data(mac_header_snip, mhr_len);
        
    /* populate MAC header metadata in the caller-allocated iolist */
    //out_frame_iolist->iol_next = &request->msdu.iolist;
    //out_frame_iolist->iol_len = mhr_len;

    *out_frame = gnrc_pkt_append(mac_header_snip, request->msdu.pkt);
    

#if IS_USED(MODULE_IEEE802154_SECURITY)
    {
        /* write protect `pkt` to set `pkt->next` */
        gnrc_pktsnip_t *tmp = gnrc_pktbuf_start_write(pkt);
        if (!tmp) {
            DEBUG("_send_ieee802154: no write access to pkt");
            gnrc_pktbuf_release(pkt);
            return -ENOMEM;
        }
        pkt = tmp;
        tmp = gnrc_pktbuf_start_write(pkt->next);
        if (!tmp) {
            DEBUG("_send_ieee802154: no write access to pkt->next");
            gnrc_pktbuf_release(pkt);
            return -ENOMEM;
        }
        pkt->next = tmp;
        /* merge snippets to store the L2 payload uniformly in one buffer */
        res = gnrc_pktbuf_merge(pkt->next);
        if (res < 0) {
            DEBUG("_send_ieee802154: failed to merge pktbuf\n");
            gnrc_pktbuf_release(pkt);
            return res;
        }

        iolist_header.iol_next = (iolist_t *)pkt->next;

        uint8_t mic[IEEE802154_SEC_MAX_MAC_SIZE];
        uint8_t mic_size = 0;

        if (flags & NETDEV_IEEE802154_SECURITY_EN) {
            res = ieee802154_sec_encrypt_frame(&netdev_ieee802154->sec_ctx,
                                               mhr, &mhr_len,
                                               pkt->next->data, pkt->next->size,
                                               mic, &mic_size,
                                               netdev_ieee802154->long_addr);
            if (res != 0) {
                DEBUG("_send_ieee802154: encryption failedf\n");
                gnrc_pktbuf_release(pkt);
                return res;
            }
        }
        if (mic_size) {
            gnrc_pktsnip_t *pktmic = gnrc_pktbuf_add(pkt->next->next,
                                                     mic, mic_size,
                                                     GNRC_NETTYPE_UNDEF);
            if (!pktmic) {
                DEBUG("_send_ieee802154: no space left in pktbuf to allocate MIC\n");
                gnrc_pktbuf_release(pkt);
                return -ENOMEM;
            }
            pkt->next->next = pktmic;
        }
        iolist_header.iol_len = mhr_len;
    }
#endif

    return res;
}

void _print_poll_request(ieee802154_mlme_poll_request_t *r)
{
    printf("coord_addr_mode: %d\n",     r->coord_addr_mode);
    printf("coord_pan_id: 0x%02x%02x\n", ((uint8_t*)&r->coord_pan_id)[1],
                                         ((uint8_t*)&r->coord_pan_id)[0]);
    printf("coord_address:  0x");
    for (unsigned i = 0; i < r->coord_address.l2addr_len; i++) {
        printf("%02x", r->coord_address.l2addr[i]);
    }
    printf("\n");
}

ieee802154_mlme_poll_request_t poll_request;
void ieee802154_perform_poll_request(ieee802154_mac_t *mac)
{
    //TODO: move to util function
    //TODO: allocate "dynamically" or copy on the request call
    //      this only works as is if it is guaranteed that only one
    //      request will be done at a time !!
    poll_request.coord_addr_mode = IEEE802154_ADDR_MODE_EXTENDED;
    poll_request.coord_pan_id = byteorder_htols(_mac2netdev802154(mac)->pan);

    ieee802154_l2addr_t *coord_addr = ieee802154_mac_get_coordinator_l2addr(mac);
    memcpy(poll_request.coord_address.l2addr,
            &coord_addr->l2addr,
            coord_addr->l2addr_len);
    poll_request.coord_address.l2addr_len = coord_addr->l2addr_len;

    ieee802154_mlme_poll_request(mac,
            &poll_request,
            &_ieee802154_mlme_poll_confirm_cb);
}

void _print_iolist(iolist_t *iol)
{
    while (iol) {
        printf("iol has %d bytes\n", iol->iol_len);
        for (unsigned i = 0; i < iol->iol_len; i++) {
            printf("0x%02x ", ((uint8_t*)iol->iol_base)[i]);
        }
        printf("\n");
        iol = iol->iol_next;
    }
}

bool ieee802154_dst_addr_uses_idtx(const ieee802154_mac_t *mac, const ieee802154_l2addr_t *dst)
{
    //TODO check if dst is an idtx node.. and add pkt to queue 
    const ieee802154_l2addr_t *l2addrs = mac->idtx_l2addrs;
    //bool queued_idtx = false;

    DEBUG("check if node uses IDTX\n");
    for (int i = 0; i < IEEE802154_MAC_IDTX_QUEUES_NUMOF; i++) {
        if (l2addrs[i].l2addr_len &&  _l2addr_equals(&l2addrs[i], dst)) {
            DEBUG("target addr uses IDTX\n");
            return true;
        }
    }
    return false;
}

int ieee802154_mcps_data_request(ieee802154_mac_t *mac,
                                 ieee802154_mcps_data_request_t *request,
                                 ieee802154_mcps_data_confirm_cb_t confirm_cb)
{
    DEBUG("ieee802154_mcps_data_request\n");

    request->meta.type = REQ_MCPS_DATA;
    /* just store the confirm cb in the request */
    request->confirm_cb = confirm_cb;

    //_print_data_request(request);
    /* Frame to be sent via the radio (MPDU without FCS) */
    gnrc_pktsnip_t *frame;
    int res = ieee802154_build_mac_data_frame(mac,
                                              request,
                                              &frame);
     
    DEBUG("ieee802154_build_mac_data_frame res = %d\n", res);
    if (res > 0) {
        //_print_iolist(&out_frame_iolist);
        /* store reference to the allocated packet in the request so it
         * can be freed after the confirm. */
        request->msdu.pkt = frame;
        
        if (request->indirect_tx) {
            DEBUG("ieee802154_mcps_data_request : IDTX!\n");
            //TODO save all relevant data from the request (as its only valid for the call)
            ieee802154_l2addr_t *l2addrs = mac->idtx_l2addrs;
           
            for (int i = 0; i < IEEE802154_MAC_IDTX_QUEUES_NUMOF; i++) {
                if (l2addrs[i].l2addr_len &&  _l2addr_equals(&l2addrs[i], &request->dst_address)) {
                    DEBUG("target addr uses IDTX\n");
                    
                    ieee802154_idtx_req_q_entry_t *rqe = _alloc_idtx_req_entry(mac, frame, request);
                    if (rqe) {
                        mutex_lock(&mac->idtx_q_lock);
                        gnrc_pktqueue_t **queue = _get_idtxq(mac, &l2addrs[i]);
                        mutex_unlock(&mac->idtx_q_lock);

                        if (!queue) {
                            DEBUG("no queue set up for the destination addr. alloc one..\n");
                            ieee802154_idtxq_t *idtxd = _alloc_idtx_pktq(mac, &l2addrs[i]);
                            if (idtxd) {
                                queue = &idtxd->queue;
                                DEBUG("allocated a queue @%p for the IDTX destination @%p\n", queue, &l2addrs[i]);
                            }
                        } else {
                            DEBUG("there is already a queue\n");
                        }
                        if (queue) {
                            mutex_lock(&mac->idtx_q_lock);
                            gnrc_pktqueue_add(queue, &rqe->q_entry);
                            mutex_unlock(&mac->idtx_q_lock);
                            DEBUG("pushed pkt to IDTX queue\n");
                            _config_ack_pending_bit(mac, true);
                        } else {
                            DEBUG("no queue for dstaddr available\n");
                        }
                    } else {
                        DEBUG("could not allocate packet queue element\n");
                    }
                }
            }
        } else {
            DEBUG("ieee802154_mcps_data_request : send direct!\n");
            /* replace by blocking queue */
            _sched_req(mac, &request->meta);
        }
        //TODO: set this confirm data properly
        //confirm->msdu_handle = &request->msdu;
        //confirm->timestamp = 0;
        //confirm->num_backoffs = 0;
        //confirm->status = MCPS_INVALID_PARAMETER;
        return IEEE802154_OK;
    }

    return IEEE802154_ERROR;
}

void _radio_async_on_off_request(ieee802154_mac_t *mac, bool enable)
{
    if (enable) {
        event_post(mac->radio_event_queue, &mac->radio_on_request_event);
    } else {
        event_post(mac->radio_event_queue, &mac->radio_off_request_event);
    }
}


ieee802154_poll_state_t _mlme_poll_state_machine(ieee802154_mac_t *mac,
                                                ieee802154_mac_fsm_state_t state,
                                                ieee802154_event_t event) {
    if (MAC_FSM_DEBUG) {
        printf("%s.%s\n", _poll_state2str(state.poll_state), _event2str(event));
    }
    ieee802154_mlme_poll_confirm_t c;

    switch (state.poll_state) {
       case POLL_INIT:
           if (event == START) {
               _send_data_request(mac, mac->current_req.poll_request);
               return POLL_WAIT_FOR_ACK;
           } else {
               break;
           }
       case POLL_WAIT_FOR_ACK:
           if (event == MEDIUM_BUSY) {
               c.status = MLME_CHANNEL_ACCESS_FAILURE;
               mac->current_req.poll_request->confirm_cb(mac, &c);
               _radio_async_on_off_request(mac, false);
               return POLL_WAIT_FOR_ACK;
           } else if (event == ACK_RECEIVED) {
              if (mac->current_req.poll_request->tx_info.data_pending) {
                  /* received a reply that indicated more data.
                   * -> setup timeout for actually receiving the data. */
                  mac->data_request_timeout_timer.callback= _mlme_poll_timeout;
                  mac->data_request_timeout_timer.arg = mac; 
                  if (!mac->poll_timeout_us) {
                      mac->poll_timeout_us = IEEE802154_MAC_MLME_POLL_PENDING_DEFAULT_TIMEOUT_US;
                  }
                  ztimer_set(ZTIMER_USEC, &mac->data_request_timeout_timer,
                             mac->poll_timeout_us);

                  /* NOTE: instead of an explicit on-request, an alternative could be to tell
                   *       the driver to set the radio to RX-mode (once) after TX is done */
                  _radio_async_on_off_request(mac, true);

                  return POLL_WAIT_FOR_DATA;
              } else {
                  c.status = MLME_NO_DATA;
                  mac->current_req.poll_request->confirm_cb(mac, &c);
                  _radio_async_on_off_request(mac, false);
                  return POLL_WAIT_FOR_ACK;
              }
           } else if (event == ACK_TIMED_OUT) {
               c.status = MLME_NO_ACK;
               mac->current_req.poll_request->confirm_cb(mac, &c);
               _radio_async_on_off_request(mac, false);
               return POLL_WAIT_FOR_ACK;
           } else if (event == RADIO_TURNED_OFF) {
               return POLL_DONE;
           }
           break;
       case POLL_WAIT_FOR_DATA:
           if (event == RADIO_TURNED_ON) {
               return POLL_WAIT_FOR_DATA;
           } else if (event == DATA_TIMED_OUT) {
               _radio_async_on_off_request(mac, false);
               /* as this request is asyncronous, it is still possible that
                * data arrives before the radio is turned off.. */
               return POLL_WAIT_FOR_DATA;
           } else if (event == RX_DATA) {
                /* cancel the poll timeout */
                bool canceled_timeout = ztimer_remove(ZTIMER_USEC, &mac->data_request_timeout_timer);
                ztimer_remove(ZTIMER_USEC, &mac->data_request_timeout_timer);
                if (!canceled_timeout) {
                    /* The timer already fired *and* the event wasn't processed yet,
                     * (since RX_DATA came first)
                     * --> ignore the timeout event! */
                    event_cancel(mac->event_queue, &mac->data_timeout_event);
                }

                //TODO: check if the received data acutally matches the poll(?)
                //      otherwise, treat this as normal RX.
                c.status = MLME_SUCCESS;
                mac->current_req.poll_request->confirm_cb(mac, &c);
                mac->current_req_confirmed = true;

                ieee802154_rx_offload_entry_t *roe = _dequeue_pending_rx_entry(mac);

                bool another_frame_pending = ((uint8_t*)roe->ieee802154_hdr->data)[0] & IEEE802154_FCF_FRAME_PEND;

                if (((uint8_t*)roe->ieee802154_hdr->data)[0] & IEEE802154_FCF_FRAME_PEND) {
                    /* TODO this case has some optimization potential:
                     *     -Another delay before turning off the radio would allow the node
                     *      to keep receiving data. The sender has to use the information of the
                     *      "ACK received after sending data with pending bit" just the same as receiving a
                     *      POLL command, i.e. just immidiately send out another frame.
                     * For now, just the basic case of sending a single packet after getting
                     * a poll command is implemented. */
                }

                //printf("RXD: %d\n", _get_mhr_seq_nr(roe->ieee802154_hdr));
                /* read received data before requesting radio to turn off. */
                _netif_handover_mpdu(_mac2netif(mac), roe->mpdu, roe->ieee802154_hdr);
                _free_rx_offload_entry(mac, roe);

                if (another_frame_pending) {
                  ieee802154_perform_poll_request(mac);
                  return POLL_DONE;
                } else {
                  _radio_async_on_off_request(mac, false);
                  /* stay in this state till it is guaranteed
                   * that no data can be received anymore (radio is off), */
                  return POLL_WAIT_FOR_DATA;
                }

           } else if (event == RADIO_TURNED_OFF) {
               /* Only confirm with NO_DATA if no RX hapened before the radio turned off. */
               if (!mac->current_req_confirmed) {
                   c.status = MLME_NO_DATA;
                   mac->current_req.poll_request->confirm_cb(mac, &c);
               }
               return POLL_DONE;
           }
           break;
       case POLL_DONE:
           return POLL_DONE;
    }
    printf("################ POTENTIALLY invalid transition! %s.%s\n", _poll_state2str(state.poll_state), _event2str(event));
    return state.poll_state;
}

ieee802154_data_state_t _mcps_data_state_machine(ieee802154_mac_t *mac,
                                                ieee802154_mac_fsm_state_t state,
                                                ieee802154_event_t event) {
    if (MAC_FSM_DEBUG) {
        printf("%s.%s\n", _data_state2str(state.data_state), _event2str(event));
    }
    ieee802154_mcps_data_confirm_t c;

    switch (state.data_state) {
       case DATA_INIT:
           if (event == START) {
               iolist_t *frame = (iolist_t*)mac->current_req.data_request->msdu.pkt;
               //printf("TX %d\n", _get_seq_nr(&mac->current_req.data_request->msdu));

               /* if the request is an indirect transmission (i.e., a reply to POLL MAC cmd),
                * --> send it out ASAP.
                *     Therefore CSMA must be (temporarily) disabled.
                * If it is just a regular transmission, just leave the radio setting. */
               if (mac->current_req.data_request->indirect_tx) {
                  _control_radio_csma(mac, false);
               }

               if (mac->current_req.data_request->indirect_tx) {
                   /* Delay the response to an indirect transmission enough
                    * so that the requesting device has at least enough time
                    * to switch from TX back to RX mode.
                    * This could be optimized to consider the actual arrival time
                    * of the CMD packet. */
                   ztimer_sleep(ZTIMER_USEC, 3000);
               }
               int res = _offload_send_blocking(mac, (iolist_t*)frame);
               //TODO add error handling (properly evaluate res)
               (void)res;
               return DATA_WAIT_FOR_ACK;
           }
           break;
       // TODO: Remove this, as the TX must ensure to turn on the radio anyway
       case DATA_WAIT_FOR_RADIO_ON:
           if (event == RADIO_TURNED_ON) {
           }
           break;
       case DATA_WAIT_FOR_ACK:
           if (event == MEDIUM_BUSY) {
               c.status = MCPS_CHANNEL_ACCESS_FAILURE;
           } else if (event == ACK_RECEIVED ||
                      event == TX_DONE) {
               //TODO populate
               ////c->timestamp =
               //int8_t retries = -1;
               //dev->driver->get(dev, NETOPT_TX_RETRIES_NEEDED, &retries, sizeof(retries));
               //c.num_backoffs = retries > 0 ? retries : 0;
               c.status = MCPS_SUCCESS;
           } else if (event == ACK_TIMED_OUT) {
               c.status = MCPS_NO_ACK;
           } else if (event == RADIO_TURNED_OFF) {
               return DATA_DONE;
           } else {
               /* indicate potentially unexpected state. */
               break;
           }
           c.request = mac->current_req.data_request;
           c.msdu_handle = c.request->msdu;
           c.request->confirm_cb(mac, &c);
           /* intentionally fall thru as there is no furhter event to wait for. */
       case DATA_DONE:
           return DATA_DONE;
            
           break;
    }
    printf("######### potentially INVALID transition! (%s.%s)\n", _data_state2str(state.data_state), _event2str(event));
    return state.data_state;
}

ieee802154_norx_idle_state_t _norx_idle_state_machine(ieee802154_mac_t *mac,
                                                      ieee802154_mac_fsm_state_t state,
                                                      ieee802154_event_t event) {
    if (MAC_FSM_DEBUG) {
        printf("%s.%s\n", _norx_idle_state2str(state.norx_idle_state), _event2str(event));
    }

    switch (state.norx_idle_state) {
        case NORX_IDLE_INIT:
            if (event == START) {
               _radio_async_on_off_request(mac, false);
               return NORX_IDLE_WAIT_FOR_RADIO_OFF;
           }
           break;
        case NORX_IDLE_WAIT_FOR_RADIO_OFF:
           if (event == RADIO_TURNED_OFF) {
               return NORX_IDLE_DONE;
           }
           break;
        case NORX_IDLE_DONE:
           return NORX_IDLE_DONE;
           break;
    }
    printf("######### potentially INVALID transition! (%s.%s)\n", _norx_idle_state2str(state.norx_idle_state), _event2str(event));
    return state.norx_idle_state;
}

ieee802154_rx_idle_state_t _rx_idle_state_machine(ieee802154_mac_t *mac,
                                                  ieee802154_mac_fsm_state_t state,
                                                  ieee802154_event_t event) {
    if (MAC_FSM_DEBUG) {
        printf("%s.%s\n", _rx_idle_state2str(state.rx_idle_state), _event2str(event));
    }

    switch (state.rx_idle_state) {
        case RX_IDLE_INIT:
            if (event == START) {
               //TODO: once there is a separate FSM for sleep idle,
               //      explicitly start the radio in the rx idle FSM
               //      (but NOT in the sleep idle fsm)
               //_radio_async_on_off_request(mac, true);
               //return RX_IDLE_WAIT_FOR_RADIO_ON;
               ////TODO dont skip wait for on once there is a sleep fsm
               return RX_IDLE_WAIT_FOR_RX;
            }
            break;
       case RX_IDLE_WAIT_FOR_RADIO_ON:
           if (event == RADIO_TURNED_ON) {
               return RX_IDLE_WAIT_FOR_RX;
           }
           break;
       case RX_IDLE_WAIT_FOR_RX:
            if (event == START) {
                return RX_IDLE_INIT;
            } else if (event == RX_DATA) {
               ieee802154_rx_offload_entry_t *roe = _dequeue_pending_rx_entry(mac);
               /* only data can already be handled by the mac. Other dataa is handed to netif directly. */
               /* check if the received frame is a MAC comand */
               if ((((uint8_t*)roe->ieee802154_hdr->data)[0] & IEEE802154_FCF_TYPE_MACCMD) == IEEE802154_FCF_TYPE_MACCMD) {
                   _handle_mac_cmd(mac, roe->ieee802154_hdr, roe->mpdu);
                   gnrc_pktbuf_release(roe->mpdu);
               } else {
                   /* if this is no MAC CMD, it must be data.. so hand it over to netif
                    * so it is forwarded to respective upper layer. */
                   _netif_handover_mpdu(_mac2netif(mac), roe->mpdu, roe->ieee802154_hdr);
               }
               _free_rx_offload_entry(mac, roe);
               return RX_IDLE_DONE;
           }
           break;
        case RX_IDLE_DONE:
           return RX_IDLE_DONE;
           break;
        default:
           printf("other event...\n");
           return RX_IDLE_DONE;
    }
    printf("######### potentially INVALID transition! (%s.%s)\n", _rx_idle_state2str(state.rx_idle_state), _event2str(event));
    return state.rx_idle_state;
}

void ieee802154_mlme_poll_request(ieee802154_mac_t *mac,
                                  ieee802154_mlme_poll_request_t *request,
                                  ieee802154_mlme_poll_confirm_cb_t confirm_cb)
{
    DEBUG("ieee802154_mlme_poll_request\n");
    request->meta.type = REQ_MLME_POLL;
    request->confirm_cb = confirm_cb;
    _sched_req(mac, &request->meta);
}

void _tx_done_offload_event_handler(event_t *event) {
    ieee802154_mac_t *mac = container_of(event, ieee802154_mac_t, tx_done_offload_event);
    ieee802154_event_t oe;
    mutex_lock(&mac->lock);
    if (_get_mac_current_mac_req_mode(mac) == MAC_MCPS_DATA) {
        oe = mac->current_req.data_request->txd_offloaded_event;
    } else if (_get_mac_current_mac_req_mode(mac) == MAC_MLME_POLL) {
        oe = mac->current_req.poll_request->txd_offloaded_event;
    } else {
        printf("INVALID offload event!\n");
        assert(false);
        return;
    }
    mutex_unlock(&mac->lock);
    _handle_fsm_event(mac, oe);
}

bool _ieee802154_ensure_ifs = false;
static inline size_t _framesize2ifs(size_t framesize) {
   if (framesize <= IEEE802154_SIFS_MAX_FRAME_SIZE) {
       return IEEE802154_SIFS_SYMS * 16;
    } else {
       return IEEE802154_LIFS_SYMS * 16;
    }
}

void ieee802154_mac_tx_done_cb(ieee802154_mac_t *mac, ieee802154_tx_done_info_t *info)
{
   mutex_lock(&mac->lock);
   mac->t_last_on_air = ztimer_now(ZTIMER_USEC);
   ieee802154_event_t event;
   if (info->medium_busy) {
       event = MEDIUM_BUSY;
   } else if (info->recvd_ack) {
       event = ACK_RECEIVED;
   } else {
       event = ACK_TIMED_OUT;
   }

   ieee802154_tx_done_info_t *req_txinfo = NULL;
   if (_get_mac_current_mac_req_mode(mac) == MAC_MCPS_DATA) {
       req_txinfo = &mac->current_req.data_request->tx_info;
       mac->current_req.data_request->txd_offloaded_event = event;
       if (_ieee802154_ensure_ifs) {
         iolist_t *frame = (iolist_t*)mac->current_req.data_request->msdu.pkt;
         size_t flen = iolist_size(frame);
         mac->t_ifs = _framesize2ifs(flen);
       }
   } else if (_get_mac_current_mac_req_mode(mac) == MAC_MLME_POLL) {
       req_txinfo = &mac->current_req.poll_request->tx_info;
       mac->current_req.poll_request->txd_offloaded_event = event;
   }
   mutex_unlock(&mac->lock);

   /* copy the info so it can be offloaded */
   memcpy(req_txinfo, info, sizeof(ieee802154_tx_done_info_t));
   event_post(mac->event_queue, &mac->tx_done_offload_event);
}

void _handle_mac_cmd(ieee802154_mac_t *mac, gnrc_pktsnip_t *ieee802154_hdr, gnrc_pktsnip_t *mpdu)
{
    /* check if it is a data request... */
    if (((uint8_t*)mpdu->data)[0] == IEEE802154_MAC_CMD_DATA_REQUEST) {
        /* check if there is pending data for that address */
        uint8_t srcaddr[IEEE802154_LONG_ADDRESS_LEN];
        le_uint16_t pan;
        int srcaddr_len = ieee802154_get_src(ieee802154_hdr->data, srcaddr, &pan);
        //TODO: guard data with lock
        ieee802154_l2addr_t *svdaddr = _save_l2addr_for_idtx(mac, srcaddr, srcaddr_len);
        if (svdaddr) {
            gnrc_pktsnip_t *idtx_pkt;
            ieee802154_mcps_data_request_t *req_ref;
            bool have_pkt = _get_next_idtx_entry(mac, svdaddr, &idtx_pkt, &req_ref);

            if (have_pkt) {
                DEBUG("found pkt (@%p) in IDTXQ\n", idtx_pkt);
                /* For now the IDTX frame is just queued like any other request.
                 * TODO: check if some kind of ASAP-flag is a useful extension for better
                 *       performance under other network load that would delay the IDTX reply.
                 *       (answering poll CMDs ASAP allows for stricter timeouts on the RFD side). */
                _sched_req(mac, &req_ref->meta);
            } else {
                mutex_lock(&mac->idtx_q_lock);
                if (mac->pending_idtx_pkt_cnt == 0) {
                    /* after removing the last IDTX element disable the pending bit.
                     * That way RFDs don't need to wait for the full timeout
                     * and can already stop after receiving the ACK without the pending bit.
                     * TODO: add a counter to easily check if there are pending
                     *       requests (for *ALL* queues) */
                    _config_ack_pending_bit(mac, false);
                }
                mutex_unlock(&mac->idtx_q_lock);
            }
        }
    }
}

ieee802154_rx_offload_entry_t* _alloc_rx_offload_entry(ieee802154_mac_t *mac)
{
    mutex_lock(&mac->rx_offload_lock);
    clist_node_t *n = clist_lpop(&mac->free_rx_buffers);
    ieee802154_rx_offload_entry_t *e = NULL;
    if (n) {
        e = container_of(n, ieee802154_rx_offload_entry_t, node);
        n->next = NULL;
        if (e->mpdu || e->ieee802154_hdr || e->node.next) {
            printf("error not NULL!\n");
        }
    }
    mutex_unlock(&mac->rx_offload_lock);
    return e;
}

void _rx_data_offload_event_handler(event_t *event) {
    ieee802154_mac_t *mac = container_of(event, ieee802154_mac_t, rx_data_offload_event);

    DEBUG("_rx_data_offload_event_handler\n");

    _handle_fsm_event(mac, RX_DATA);
    return;

    ieee802154_rx_offload_entry_t *roe = _dequeue_pending_rx_entry(mac);

    if (!roe) {
        printf("FATAL! ---> called offload handler, but there is no pending data!\n");
        assert(false);
    }

    /* only data can already be handled by the mac. Other dataa is handed to netif directly. */
    /* check if the received frame is a MAC comand */
    if ((((uint8_t*)roe->ieee802154_hdr->data)[0] & IEEE802154_FCF_TYPE_MACCMD) == IEEE802154_FCF_TYPE_MACCMD) {
        _handle_mac_cmd(mac, roe->ieee802154_hdr, roe->mpdu);
        gnrc_pktbuf_release(roe->mpdu);
        _free_rx_offload_entry(mac, roe);
    } else {
        mutex_lock(&mac->lock);
        if (_get_mac_current_mac_req_mode(mac) == MAC_MLME_POLL) {
            mutex_unlock(&mac->lock);
            _handle_fsm_event(mac, RX_DATA);
            _free_rx_offload_entry(mac, roe);
        } else {
            mutex_unlock(&mac->lock);
            /* TODO: add a distinct RX_IDLE FSM that is able to
             *       take over all packets, to remove the fallback method. */
            _netif_handover_mpdu(_mac2netif(mac), roe->mpdu, roe->ieee802154_hdr);
            _free_rx_offload_entry(mac, roe);
        }
    }

    /* in case there is more pending data, schedule another offload event to trigger processing */
    if (!clist_is_empty(&mac->pending_rx_buffers)) {
        event_post(mac->event_queue, &mac->rx_data_offload_event);
    }
}

/* @brief hand off a received packet to the mac context.
 *
 * @return whether the mac took over (handled) the packet. */
bool _mac_offload_mpdu(ieee802154_mac_t *mac, gnrc_pktsnip_t *mpdu,
                       gnrc_pktsnip_t *ieee802154_hdr)
{
    ieee802154_rx_offload_entry_t *roe = _alloc_rx_offload_entry(mac);

    /* if there is no space left for unprocessed rx data,
     * the radio is receiving data faster than the MAc can process it.
     * A locking mechanism for the radio is needed in that case to
     * guarantee there is no other RX event before the last one was handled. */
    if (!roe) {
        printf("FATAL: can not keep up with the RX data speed!\n");
    } else {
        roe->ieee802154_hdr = ieee802154_hdr;
        roe->mpdu = mpdu;
        _queue_pending_rx_entry(mac, roe);
        event_post(mac->event_queue, &mac->rx_data_offload_event);
        return true;
    }
    return false;
}

void ieee802154_mac_rx_done_cb(ieee802154_mac_t *mac)
{
    DEBUG("ieee802154_mac_rx_done_cb\n");
    mac->t_last_on_air = ztimer_now(ZTIMER_USEC);

    netdev_t *dev = _mac2netdev(mac);
    /* received data will be a MAC Protocol Data Unit (MPDU).
     * Always allocate max Frame size */
    gnrc_pktsnip_t *mpdu = gnrc_pktbuf_add(NULL, NULL, IEEE802154_FRAME_LEN_MAX, GNRC_NETTYPE_UNDEF);
    if (mpdu == NULL) {
        DEBUG("ieee802154_mac_rx_done_cb: cannot allocate pktsnip.\n");
        /* Discard packet on netdev device */
        dev->driver->recv(dev, NULL, IEEE802154_FRAME_LEN_MAX, NULL);
        return;
    }

    netdev_ieee802154_rx_info_t rx_info;
    int nread = dev->driver->recv(dev, mpdu->data, IEEE802154_FRAME_LEN_MAX, &rx_info);
    if ((nread <= 0) || ((unsigned)nread < IEEE802154_MIN_FRAME_LEN)) {
        gnrc_pktbuf_release(mpdu);
        return;
    } else if ((unsigned)nread < IEEE802154_FRAME_LEN_MAX) {
       /* cut off unused part of the pktsnip */
       int rares = gnrc_pktbuf_realloc_data(mpdu, nread);
       assert(rares == 0);
    }

    /* Normal mode, try to parse the frame according to IEEE 802.15.4 */
    gnrc_pktsnip_t *ieee802154_hdr;
    size_t mhr_len = ieee802154_get_frame_hdr_len(mpdu->data);

    /* nread was checked for <= 0 before so we can safely cast it to
     * unsigned */
    if ((mhr_len == 0) || ((size_t)nread < mhr_len)) {
       DEBUG("_recv_ieee802154: illegally formatted frame received\n");
       gnrc_pktbuf_release(mpdu);
       return;
    }

    /* mark IEEE 802.15.4 header */
    ieee802154_hdr = gnrc_pktbuf_mark(mpdu, mhr_len, GNRC_NETTYPE_UNDEF);
    if (ieee802154_hdr == NULL) {
        DEBUG("_recv_ieee802154: no space left in packet buffer\n");
        gnrc_pktbuf_release(mpdu);
        return;
    }

    /* offload to the mac (for now fallback to offloading to netif
     * if the mac doesn't handle the data) */
    if (!_mac_offload_mpdu(mac, mpdu, ieee802154_hdr)) {
        printf("could not offload to mac..\n");
    }

    return;
}

// TODO rework this to take the information either from
//      each request or from a global setting.
bool ieee802154_mac_requested_ack(ieee802154_mac_t *mac)
{
    netdev_ieee802154_t *nd154 = _mac2netdev802154(mac);
    return nd154->flags & NETDEV_IEEE802154_ACK_REQ;
}


