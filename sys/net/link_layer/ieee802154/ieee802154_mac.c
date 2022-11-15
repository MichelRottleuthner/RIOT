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

//TODO: move function to mac code
extern void _build_mcps_data_request(gnrc_netif_t *netif, gnrc_pktsnip_t *pkt,
                              ieee802154_mcps_data_request_t *request);

extern void _print_pktsnip(gnrc_pktsnip_t *p, const char* prefix_str);

/* only exposed via driver instance */
void ieee802154_mac_rx_done_cb(ieee802154_mac_t *mac);
void ieee802154_mac_tx_done_cb(ieee802154_mac_t *mac, ieee802154_tx_done_info_t *info);

ieee802154_driver_t _ieee802154_mac_driver = {
    .rx_done_cb = ieee802154_mac_rx_done_cb,
    .tx_done_cb = ieee802154_mac_tx_done_cb,
};

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

/* This event handler is called after a MLME-POLL CMD (request for pending data)
 * was received and there is pending data. This data must be transmitted now. */ 
static void _request_offload_event_handler(event_t *evp)
{
    ieee802154_mac_t *mac = container_of(evp, ieee802154_mac_t, request_offload_event);
    
    gnrc_pktsnip_t *idtx_pkt = mac->pending_idtx_pkt;

    printf("_request_offload_event_handler (idtx_pkt@%p)\n", idtx_pkt);

    if (idtx_pkt) {
        //TODO: replace request with "phy_send_thingy"
        /* send directly */
        mac->mcps_state.mcps_req = MCPS_DATA;
        mac->mcps_state.data.msdu_handle.pkt = idtx_pkt;
        //TODO fix handling of this
        mac->mlme_mcps_confirm.data_confirm_cb = mac->idtx_data_confirm_cb;
        //mac->mlme_mcps_confirm.data_confirm_cb = confirm_cb;
        gnrc_netif_t *netif = container_of(mac, gnrc_netif_t, ieee802154_mac);
        netdev_t *dev = netif->dev;
        int res = dev->driver->send(dev, (iolist_t*)idtx_pkt);
        assert(gnrc_netif_netdev_legacy_api(_mac2netif(mac)));
        if (res < 0 || ((unsigned)res) != gnrc_pkt_len(idtx_pkt)) {
            printf("_request_offload_event_handler: netdev send indicated error (%d)\n", res);
        }
        mac->pending_idtx_pkt = NULL;
    } else {
        printf("_request_offload_event_handler: NO pending packet IDTX\n");
    }
}

void _mlme_poll_timeout(void *arg)
{
    ieee802154_mac_t *mac = (ieee802154_mac_t*)arg;
    netdev_t *netdev = _mac2netdev(mac);
    /* check if the poll request is still active.
     * If data was received before this callback was fired, this timeout
     * is obsolete. */
    if (mac->mlme_req == MLME_POLL) {
        /* timed out while wainting for data after data pending indication
         * -> put radio back to sleep */
        _control_radio_sleep(netdev, true);
        mac->mlme_req = MLME_UNDEF;
        ieee802154_mlme_poll_confirm_t c = { .status = MLME_NO_DATA };
        printf("calling poll_confirm_cb (timeout)\n");
        mac->mlme_mcps_confirm.poll_confirm_cb(mac, &c);
    }
}

void _init_mac_data(ieee802154_mac_t *mac)
{
    memset(mac, 0, sizeof(ieee802154_mac_t));
    mutex_init(&mac->lock); 
    mac->mlme_req = MLME_UNDEF;
    mac->driver = &_ieee802154_mac_driver;
    mac->request_offload_event.handler = _request_offload_event_handler;
    
    // TODO remove this (just here for testing purposes
    netdev_ieee802154_t *netdev_ieee802154 = _mac2netdev802154(mac);
    ieee802154_l2addr_t local_l2_addr;
    memcpy(local_l2_addr.l2addr, netdev_ieee802154->long_addr, IEEE802154_LONG_ADDRESS_LEN);
    local_l2_addr.l2addr_len = IEEE802154_LONG_ADDRESS_LEN;

    if (_l2addr_equals(&local_l2_addr, &_test_node1_l2addr)) {
        printf("this is node 1!\n");
        mac->coordinator_addr = &_test_node2_l2addr;
    } else if (_l2addr_equals(&local_l2_addr, &_test_node2_l2addr)) {
        printf("this is node 2!\n");
        mac->coordinator_addr = &_test_node1_l2addr;
    }
    //macdata->mac_idtx_queues_list = &macdata->idtx_queue_pool[0].next;
}

ieee802154_idtxq_t *_alloc_idtx_pktq(ieee802154_mac_t *mac, ieee802154_l2addr_t *addr)
{
    ieee802154_idtxq_t *pool = &mac->idtx_queue_pool[0];
    
    for (int i = 0; i < IEEE802154_MAC_IDTX_QUEUES_NUMOF; i++) {
        if (!pool[i].addr_data) {
            DEBUG("queue[%d] is free, allocate it for %p!\n", i, addr);
            pool[i].addr_data = addr;
            list_add(&mac->idtx_queues_list, &pool[i].node);
            return &pool[i];
        }
        DEBUG("queue[%d] is already allocated!\n", i);
    }
    return NULL;
}

gnrc_pktqueue_t *_alloc_idtx_pktq_entry(ieee802154_mac_t *mac, gnrc_pktsnip_t *pkt)
{
    gnrc_pktqueue_t *pool = &mac->idtx_pktq_entry_pool[0];
    
    for (int i = 0; i < IEEE802154_MAC_IDTX_PKTS_NUMOF; i++) {
        if (pool[i].pkt == NULL) {
            pool[i].pkt = pkt;
            return &pool[i];
        }
    }
    return NULL;
}

/* returns the indirect transmission queue for a particular destination address.
 * @p addr can not be an arbitrary dynamic variable, more specifically it must
 * point to an entry of @ref ieee802154_l2addr_t.idtx_l2addrs */
gnrc_pktqueue_t **_get_idtxq(ieee802154_mac_t *mac,
                             ieee802154_l2addr_t *addr)
{
    list_node_t *list = &mac->idtx_queues_list;
    //ieee802154_idtxq_t *free_idtxq = NULL;
    
    DEBUG("get queue for addr @%p\n", addr);
    while (list->next) {
        ieee802154_idtxq_t *txq = container_of(list->next, ieee802154_idtxq_t, node);
        //if (!txq->addr_data) {
        //    free_idtxq = txq;
        //} else 
        if (txq->addr_data == addr) {
            DEBUG("found queue @%p\n", txq);
            return &txq->queue;
        } else {
            DEBUG("queue @%p is for dst %p not %p\n", txq, txq->addr_data, addr);
        }
        list = list->next;
    }

    ///* if an unused queue descriptor was available */
    //if (free_idtxq) {
    //    free_idtxq->addr_data = addr;
    //}

    return NULL;
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


/* returns the next packet that was intended to be sent to a node
 * that expects indirect transmissions.
 * @p addr can not be an arbitrary dynamic variable, more specifically it must
 * point to an entry of @ref ieee802154_l2addr_t.idtx_l2addrs */
gnrc_pktsnip_t *_get_next_indirect_pkt(ieee802154_mac_t *mac,
                                             ieee802154_l2addr_t *addr)
{
    gnrc_pktsnip_t *nextpkt = NULL;
    
    gnrc_pktqueue_t **idtxq = _get_idtxq(mac, addr);

    if (idtxq) {
        DEBUG("there is a queue at %p for the requesting node\n", idtxq);
        gnrc_pktqueue_t *entry = gnrc_pktqueue_remove_head(idtxq);

        if (entry != NULL) {
            nextpkt = entry->pkt;
            entry->pkt = NULL;
        }
    }
    
    return nextpkt;
}

/* Sends a packet to @p addr that was previously queued for indirect transmission
 * if there is any.
 * @p addr can not be an arbitrary dynamic variable, more specifically it must
 * point to an entry of @ref ieee802154_l2addr_t.idtx_l2addrs */
void _send_indirect_tx_queued_pkt(ieee802154_mac_t *mac,
                                  ieee802154_l2addr_t *addr)
{
    gnrc_pktsnip_t *pkt;

    if ((pkt = _get_next_indirect_pkt(mac, addr)) != NULL) {
        //_send(netif, pkt);
        
        //gnrc_netif_pktq_sched_get(netif);
    }
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

int _send_data_request(netdev_ieee802154_t *state,
                       ieee802154_mlme_poll_request_t *request)
{
    /* Send data request */
    uint8_t dr_psdu[32];

    int len = _build_data_request(state, request, dr_psdu);

    iolist_t iol = {.iol_base = dr_psdu, .iol_len = len};

    //ieee802154_send(&mac->submac, &iol);
    netdev_t *dev = &state->netdev;
    int res = 0;
    res = dev->driver->send(dev, &iol);
    return res;
}

void _control_radio_sleep(netdev_t *dev, bool sleep) {
    netopt_state_t state = sleep ? NETOPT_STATE_SLEEP : NETOPT_STATE_IDLE;
    int len = dev->driver->set(dev, NETOPT_STATE, &state, sizeof(state));
    if (len != sizeof(state)) {
        printf("ERROR: couldn't set radio to sleep!\n");
    }
}

extern void _custom_event_cb(netdev_t *dev, netdev_event_t event);

void _trigger_data_request(ieee802154_mac_t *mac) {
    ieee802154_perform_poll_request(mac);
}

void _data_request_event_handler(event_t *event) {
    ieee802154_mac_t *mac = container_of(event, ieee802154_mac_t, data_request_event);
    ieee802154_perform_poll_request(mac);
}

static void _ieee802154_mlme_poll_confirm_cb(ieee802154_mac_t *mac,
                                             ieee802154_mlme_poll_confirm_t *confirm)
{
    (void)mac;
    switch (confirm->status) {
        case MLME_SUCCESS:
            printf("MLME-POLL.confirm(SUCCESS)\n");
            break;
        case MLME_CHANNEL_ACCESS_FAILURE:
            printf("MLME-POLL.confirm(CHANNEL_ACCESS_FAILURE)\n");
            break;
        case MLME_NO_DATA:
            printf("MLME-POLL.confirm(NO_DATA)\n");
            break;
        case MLME_NO_ACK:
            printf("MLME-POLL.confirm(NO_ACK)\n");
            break;
        default:
            printf("MLME-POLL.confirm(OTHER)");
    }
}

void _enable_periodic_data_request(ieee802154_mac_t *mac, unsigned int poll_ms) {
    gnrc_netif_t *netif = container_of(mac, gnrc_netif_t, ieee802154_mac);
    mac->data_request_event.handler = _data_request_event_handler;

    event_periodic_init(&mac->periodic_data_request_event, ZTIMER_MSEC,
                        &netif->evq[GNRC_NETIF_EVQ_INDEX_PRIO_LOW],
                        &mac->data_request_event);
    event_periodic_start(&mac->periodic_data_request_event, poll_ms);
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

void ieee802154_mlme_poll_request(ieee802154_mac_t *mac,
                                  ieee802154_mlme_poll_request_t *request,
                                  ieee802154_mlme_poll_confirm_cb_t confirm_cb)
{
    DEBUG("ieee802154_mlme_poll_request\n");
    gnrc_netif_t *netif = container_of(mac, gnrc_netif_t, ieee802154_mac);
    netdev_t *dev = netif->dev;
    //turn on radio first
    _control_radio_sleep(dev, false);
    netdev_ieee802154_t *netdev_ieee802154 = container_of(dev, netdev_ieee802154_t, netdev);

    mac->mlme_req = MLME_POLL;
    mac->mlme_mcps_confirm.poll_confirm_cb = confirm_cb;
    //setup timeout for the recive that turns off the radio again
    _send_data_request(netdev_ieee802154, request);
}

void ieee802154_perform_poll_request(ieee802154_mac_t *mac)
{
    ieee802154_mlme_poll_request_t request;
    //TODO: move to util function
    request.coord_addr_mode = IEEE802154_ADDR_MODE_EXTENDED;
    request.coord_pan_id = byteorder_htols(_mac2netdev802154(mac)->pan);

    ieee802154_l2addr_t *coord_addr = ieee802154_mac_get_coordinator_l2addr(mac);
    memcpy(request.coord_address.l2addr,
            &coord_addr->l2addr,
            coord_addr->l2addr_len);
    request.coord_address.l2addr_len = coord_addr->l2addr_len;

    ieee802154_mlme_poll_request(mac,
            &request,
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
    printf("ieee802154_mcps_data_request\n");

    //_print_data_request(request);
    /* Frame to be sent via the radio (MPDU without FCS) */
    gnrc_pktsnip_t *frame;
    int res = ieee802154_build_mac_data_frame(mac,
                                              request,
                                              &frame);

    //_print_pktsnip(frame, "ieee802154_mcps_data_request:");
     
    DEBUG("ieee802154_build_mac_data_frame res = %d\n", res);
    if (res > 0) {
        //_print_iolist(&out_frame_iolist);
        
        if (request->indirect_tx) {
            DEBUG("ieee802154_mcps_data_request : IDTX!\n");
            //TODO save all relevant data from the request (as its only valid for the call)
            ieee802154_l2addr_t *l2addrs = mac->idtx_l2addrs;
           
            for (int i = 0; i < IEEE802154_MAC_IDTX_QUEUES_NUMOF; i++) {
                if (l2addrs[i].l2addr_len &&  _l2addr_equals(&l2addrs[i], &request->dst_address)) {
                    DEBUG("target addr uses IDTX\n");
                    
                    gnrc_pktqueue_t *pktqe = _alloc_idtx_pktq_entry(mac, frame);
                    if (pktqe) {
                        gnrc_pktqueue_t **queue = _get_idtxq(mac, &l2addrs[i]);

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
                            gnrc_pktqueue_add(queue, pktqe);
                            DEBUG("pushed pkt to IDTX queue\n");
                            mac->idtx_data_confirm_cb = confirm_cb;
                        } else {
                            DEBUG("no queue for dstaddr available\n");
                        }
                    } else {
                        DEBUG("could not allocate packet queue element\n");
                    }
                }
            }
        } else {
            printf("ieee802154_mcps_data_request : send direct!\n");
            /* send directly */
            mac->mcps_state.mcps_req = MCPS_DATA;
            mac->mcps_state.data.msdu_handle.pkt = frame;
            mac->mlme_mcps_confirm.data_confirm_cb = confirm_cb;
            gnrc_netif_t *netif = container_of(mac, gnrc_netif_t, ieee802154_mac);
            netdev_t *dev = netif->dev;
            res = dev->driver->send(dev, (iolist_t*)frame);
        }

        ////TODO: set confirm data properly
        //confirm->msdu_handle = &request->msdu;
        //confirm->timestamp = 0;
        //confirm->num_backoffs = 0;
        //confirm->status = MCPS_INVALID_PARAMETER;
        return IEEE802154_OK;
    }

    return IEEE802154_ERROR;
}

void ieee802154_mac_tx_done_cb(ieee802154_mac_t *mac, ieee802154_tx_done_info_t *info)
{
   (void)mac;
   (void)info;
   netdev_t *dev = _mac2netdev(mac);

   if (mac->mlme_req == MLME_POLL) {
       DEBUG("ieee802154_mac_tx_done_cb: MLME_POLL\n");
       //ieee802154_mlme_poll_request_t *r = mac->mlme_mcps_request.poll_request;
       //ieee802154_mlme_poll_confirm_t *c = mac->mlme_mcps_confirm.poll_confirm;
       if (info->data_pending) {
           DEBUG("MLME-POLL *pending data*...\n");
           //TODO: only indicate success if actual data was received
           //c->status = MLME_SUCCESS;
           /* received a reply that indicated more data.
            * -> setup timeout for actually receiving the data. */
           mac->data_request_timeout_timer.callback= _mlme_poll_timeout;
           mac->data_request_timeout_timer.arg = mac; 
           ztimer_set(ZTIMER_MSEC, &mac->data_request_timeout_timer,
                      IEEE802154_MAC_MLME_POLL_PENDING_TIMEOUT_US / US_PER_MS);
       } else {
           ztimer_remove(ZTIMER_MSEC, &mac->data_request_timeout_timer);

           ieee802154_mlme_poll_confirm_t c;
           /* clear request state as MLME request is finished now */
           mac->mlme_req = MLME_UNDEF;
           if (info->medium_busy) {
               c.status = MLME_CHANNEL_ACCESS_FAILURE;
           } else if (info->recvd_ack) {
               c.status = MLME_NO_DATA;
           } else {
               c.status = MLME_NO_ACK;
           }
           /* turn off radio cause no data is expected */ 
           _control_radio_sleep(dev, true);
           DEBUG("calling poll confirm_cb\n");
           mac->mlme_mcps_confirm.poll_confirm_cb(mac, &c);
       }
   } else if (mac->mcps_state.mcps_req == MCPS_DATA) {
       DEBUG("ieee802154_mac_tx_done_cb: MCPS_DATA\n");
       ieee802154_mcps_data_confirm_t c;
       ieee802154_mcps_data_request_t *r = mac->mlme_mcps_request.data_request;
       //TODO populate
       //c->timestamp =
       int8_t retries = -1;
       dev->driver->get(dev, NETOPT_TX_RETRIES_NEEDED, &retries, sizeof(retries));
       c.num_backoffs = retries > 0 ? retries : 0;

       if (info->medium_busy) {
           DEBUG("MCPS-DATA.confirm(MCPS_CHANNEL_ACCESS_FAILURE)\n");
           c.status = MCPS_CHANNEL_ACCESS_FAILURE;
       } else if (info->recvd_ack || !r->ack_tx) {
           DEBUG("MCPS-DATA.confirm(SUCCESS) (GOT%sACK,%sACKREQ)\n",
                  (info->recvd_ack ? " " : " NO "),
                  (r->ack_tx ? " " : " NO "));
           //TODO use info->data_pending info
           c.status = MCPS_SUCCESS;
       } else {
           DEBUG("MCPS-DATA.confirm(MCPS_NO_ACK)\n");
           c.status = MCPS_NO_ACK;
       }
       c.msdu_handle = mac->mcps_state.data.msdu_handle;
       mac->mcps_state.mcps_req = MCPS_UNDEF;
       DEBUG("calling data confirm_cb\n");
       mac->mlme_mcps_confirm.data_confirm_cb(mac, &c);
   }
}

void _netif_handover_mpdu(gnrc_netif_t *netif, gnrc_pktsnip_t *mpdu, gnrc_pktsnip_t *ieee802154_hdr);

void _handle_mac_cmd(ieee802154_mac_t *mac, gnrc_pktsnip_t *ieee802154_hdr, gnrc_pktsnip_t *mpdu)
{
    DEBUG("received a MAC CMD\n");
    /* check if it is a data request... */
    if (((uint8_t*)mpdu->data)[0] == IEEE802154_MAC_CMD_DATA_REQUEST) {
        /* check if there is pending data for that address */
        DEBUG("got a Data Request!\n");
        uint8_t srcaddr[IEEE802154_LONG_ADDRESS_LEN];
        le_uint16_t pan;
        int srcaddr_len = ieee802154_get_src(ieee802154_hdr->data, srcaddr, &pan);
        DEBUG("srcaddr_len: %d\n", srcaddr_len);
        ieee802154_l2addr_t *svdaddr = _save_l2addr_for_idtx(mac, srcaddr, srcaddr_len);
        DEBUG("saved at %p\n", svdaddr);
        if (svdaddr) {
            /* only serve mac command if not already busy with serving another one */
            if (!mac->pending_idtx_pkt) {
                DEBUG("checking if there is any pending TX for the requester...\n");
                gnrc_pktsnip_t *idtx_pkt = _get_next_indirect_pkt(mac, svdaddr);
                if (idtx_pkt) {
                    DEBUG("found pkt (@%p) in IDTXQ\n", idtx_pkt);
                    mac->pending_idtx_pkt = idtx_pkt;
                    /* post event to handle IDTX */
                    event_post(&_mac2netif(mac)->evq[GNRC_NETIF_EVQ_INDEX_PRIO_LOW], &mac->request_offload_event);
                }
            }
        }
    }
}

void ieee802154_mac_rx_done_cb(ieee802154_mac_t *mac)
{
    (void)mac;
    DEBUG("ieee802154_mac_rx_done_cb\n");

    netdev_t *dev = _mac2netdev(mac);
    /* received data will be a MAC Protocol Data Unit (MPDU) */
    gnrc_pktsnip_t *mpdu = NULL;
    int bytes_expected = dev->driver->recv(dev, NULL, 0, NULL);
    netdev_ieee802154_rx_info_t rx_info;

    if (bytes_expected >= (int)IEEE802154_MIN_FRAME_LEN) {
        int nread;

        mpdu = gnrc_pktbuf_add(NULL, NULL, bytes_expected, GNRC_NETTYPE_UNDEF);
        if (mpdu == NULL) {
            DEBUG("ieee802154_mac_rx_done_cb: cannot allocate pktsnip.\n");
            /* Discard packet on netdev device */
            dev->driver->recv(dev, NULL, bytes_expected, NULL);
            //return NULL;
            return;
        }
        nread = dev->driver->recv(dev, mpdu->data, bytes_expected, &rx_info);
        if (nread <= 0) {
            gnrc_pktbuf_release(mpdu);
            //return NULL;
            return;
        }

        /* Normal mode, try to parse the frame according to IEEE 802.15.4 */
        gnrc_pktsnip_t *ieee802154_hdr;
        size_t mhr_len = ieee802154_get_frame_hdr_len(mpdu->data);
        //uint8_t *mhr = mpdu->data;
        /* nread was checked for <= 0 before so we can safely cast it to
         * unsigned */
        if ((mhr_len == 0) || ((size_t)nread < mhr_len)) {
            DEBUG("_recv_ieee802154: illegally formatted frame received\n");
            gnrc_pktbuf_release(mpdu);
            //return NULL;
            return;
        }
#if IS_USED(MODULE_IEEE802154_SECURITY)
        {
            uint8_t *payload = NULL;
            uint16_t payload_size = 0;
            uint8_t *mic = NULL;
            uint8_t mic_size = 0;
            netdev_ieee802154_t *netdev_ieee802154 = container_of(dev,
                    netdev_ieee802154_t,
                    netdev);
            if (mhr[0] & NETDEV_IEEE802154_SECURITY_EN) {
                if (ieee802154_sec_decrypt_frame(&netdev_ieee802154->sec_ctx,
                            nread,
                            mhr, (uint8_t *)&mhr_len,
                            &payload, &payload_size,
                            &mic, &mic_size,
                            gnrc_netif_hdr_get_src_addr(hdr)) != 0) {
                    DEBUG("_recv_ieee802154: packet dropped by security check\n");
                    gnrc_pktbuf_release(mpdu);
                    //gnrc_pktbuf_release(netif_hdr);
                    //return NULL;
                    return;
                }
            }
            nread -= mic_size;
        }
#endif
        /* data available via rx info */
        //rx_info.lqi;
        //rx_info.rssi;
        //rx_info.timestamp;

        // why not assign this value based from known driver/device metadata!?
        //dev->driver->get(dev, NETOPT_PROTO, &pkt->type, sizeof(pkt->type));




        /* mark IEEE 802.15.4 header */
        ieee802154_hdr = gnrc_pktbuf_mark(mpdu, mhr_len, GNRC_NETTYPE_UNDEF);
        if (ieee802154_hdr == NULL) {
            DEBUG("_recv_ieee802154: no space left in packet buffer\n");
            gnrc_pktbuf_release(mpdu);
            //gnrc_pktbuf_release(netif_hdr);
            //return NULL;
            return;
        }

        //printf("ieee802154_hdr: %d bytes: ", ieee802154_hdr->size);
        //for (unsigned i = 0; i < ieee802154_hdr->size; i++) {
        //    printf("%02X ", ((uint8_t*)ieee802154_hdr->data)[i]);
        //}
        //printf("\n");

        /* check if the received frame is a MAC comand */
        if ((((uint8_t*)ieee802154_hdr->data)[0] & IEEE802154_FCF_TYPE_MACCMD) == IEEE802154_FCF_TYPE_MACCMD) {
            _handle_mac_cmd(mac, ieee802154_hdr, mpdu);
            gnrc_pktbuf_release(mpdu);
        } else {
            DEBUG("received DATA\n");
            // TODO: rework to indication callback
            _netif_handover_mpdu(_mac2netif(mac), mpdu, ieee802154_hdr);
        }

        //nread -= ieee802154_hdr->size;
        //gnrc_pktbuf_remove_snip(pkt, ieee802154_hdr);
        //DEBUG("_recv_ieee802154: reallocating MAC payload for upper layer.\n");
        //gnrc_pktbuf_realloc_data(pkt, nread);
    } else if (bytes_expected > 0) {
        DEBUG("_recv_ieee802154: received frame is too short\n");
        dev->driver->recv(dev, NULL, bytes_expected, NULL);
    }

    //return pkt;
}

bool ieee802154_mac_requested_ack(ieee802154_mac_t *mac)
{
    netdev_ieee802154_t *nd154 = _mac2netdev802154(mac);
    return nd154->flags & NETDEV_IEEE802154_ACK_REQ;
//   if (mac->mlme_req == MLME_POLL) {
//       printf("ieee802154_mac_requested_ack MLME_POLL\n");
//       /* poll always expects an ack */
//       return true;
//   } else if (mac->mcps_state.mcps_req == MCPS_DATA) {
//       ieee802154_mcps_data_request_t *r = mac->mlme_mcps_request.data_request;
//       printf("ieee802154_mac_requested_ack MCPS_DATA %s\n", r->ack_tx ? "true" : "false");
//       return r->ack_tx;
//   } else {
//       printf("ieee802154_mac_requested_ack check netdev\n");
//       netdev_ieee802154_t *nd154 = _mac2netdev802154(mac);
//       return nd154->flags & NETDEV_IEEE802154_ACK_REQ;
//   }
}


