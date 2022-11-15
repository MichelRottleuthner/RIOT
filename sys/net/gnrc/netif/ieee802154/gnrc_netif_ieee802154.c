/*
 * Copyright (C) 2017 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @{
 *
 * @file
 * @author  Martine Lenders <m.lenders@fu-berlin.de>
 */

#include "net/gnrc.h"
#include "net/gnrc/netif/ieee802154.h"
#include "net/netdev/ieee802154.h"

#ifdef MODULE_GNRC_IPV6
#include "net/ipv6/hdr.h"
#endif

#define ENABLE_DEBUG 0
#include "debug.h"

#include "od.h"


#include "net/netstats.h"
#include "net/netstats/neighbor.h"
#include "net/gnrc/ipv6.h"
#include "net/gnrc/ipv6/nib.h"
#include "event/periodic.h"
#include "net/gnrc/netif/pktq.h"
#include "net/ieee802154_mac.h"

//TODO utils to be removed
gnrc_netif_t* _mac2netif(ieee802154_mac_t *mac);
netdev_t* _mac2netdev(ieee802154_mac_t *mac);
ieee802154_mac_t* _netif2mac(void *netif);
netdev_ieee802154_t* _mac2netdev802154(ieee802154_mac_t *mac);

static int _send(gnrc_netif_t *netif, gnrc_pktsnip_t *pkt);
static gnrc_pktsnip_t *_recv(gnrc_netif_t *netif);

static const gnrc_netif_ops_t ieee802154_ops = {
    .init = gnrc_netif_default_init,
    .send = _send,
    .recv = _recv,
    .get = gnrc_netif_get_from_netdev,
    .set = gnrc_netif_set_from_netdev,
};

//static void _process_receive_stats(gnrc_netif_t *netdev, gnrc_pktsnip_t *pkt)
//{
//    if (!IS_USED(MODULE_NETSTATS_NEIGHBOR)) {
//        return;
//    }
//
//    gnrc_netif_hdr_t *hdr;
//    const uint8_t *src = NULL;
//    gnrc_pktsnip_t *netif = gnrc_pktsnip_search_type(pkt, GNRC_NETTYPE_NETIF);
//
//    if (netif == NULL) {
//        return;
//    }
//
//    size_t src_len;
//    hdr = netif->data;
//    src = gnrc_netif_hdr_get_src_addr(hdr);
//    src_len = hdr->src_l2addr_len;
//    netstats_nb_update_rx(&netdev->netif, src, src_len, hdr->rssi, hdr->lqi);
//}

//TODO check if we still need to manually invoke this on this layer
//static void _send_queued_pkt(gnrc_netif_t *netif)
//{
//    (void)netif;
//    gnrc_pktsnip_t *pkt;
//    if ((pkt = gnrc_netif_pktq_get(netif)) != NULL) {
//        _send(netif, pkt);
//        gnrc_netif_pktq_sched_get(netif);
//    }
//}

static void _pass_on_packet(gnrc_pktsnip_t *pkt)
{
    /* throw away packet if no one is interested */
    if (!gnrc_netapi_dispatch_receive(pkt->type, GNRC_NETREG_DEMUX_CTX_ALL,
                                      pkt)) {
        DEBUG("gnrc_netif: unable to forward packet of type %i\n", pkt->type);
        gnrc_pktbuf_release(pkt);
        return;
    }
}


static void _ieee802154_mcps_data_confirm_cb(ieee802154_mac_t *mac,
                                             ieee802154_mcps_data_confirm_t *confirm)
{
    (void)mac;

    switch (confirm->status) {
        case MCPS_CHANNEL_ACCESS_FAILURE:
           printf("MCPS-DATA.confirm(MCPS_CHANNEL_ACCESS_FAILURE)\n");
           break;
        case MCPS_SUCCESS:
           printf("MCPS-DATA.confirm(SUCCESS)\n");
           break;
        case MCPS_NO_ACK:
           printf("MCPS-DATA.confirm(MCPS_NO_ACK)\n");
           break;
        default:
           printf("_ieee802154_mcps_data_confirm_cb [OTHER]\n");
           break;

    }
    if (gnrc_netif_netdev_legacy_api(_mac2netif(mac))) {
        printf("pktbuf release pkt@%p\n", confirm->msdu_handle.pkt);
        /* only for legacy drivers we need to release pkt here */
        gnrc_pktbuf_release(confirm->msdu_handle.pkt);
    }
}

//#define DEBUG_NETDEV_EVENT(X) printf(X)
#define DEBUG_NETDEV_EVENT(X) ({})

void _custom_event_cb(netdev_t *dev, netdev_event_t event)
{
    gnrc_netif_t *netif = (gnrc_netif_t *)dev->context;
    ieee802154_mac_t *mac = _netif2mac(netif);
    ieee802154_tx_done_info_t info;

    if (event == NETDEV_EVENT_ISR) {
        event_post(&netif->evq[GNRC_NETIF_EVQ_INDEX_PRIO_LOW], &netif->event_isr);
    }
#if IS_USED(MODULE_NETDEV_NEW_API)
    else if (gnrc_netif_netdev_new_api(netif)
             && (event == NETDEV_EVENT_TX_COMPLETE)) {
        event_post(&netif->evq, &netif->event_tx_done);
    }
#endif
    else {
        DEBUG("gnrc_netif: event triggered -> %i\n", event);
        //gnrc_pktsnip_t *pkt = NULL;
        switch (event) {
            case NETDEV_EVENT_LINK_UP:
                if (IS_USED(MODULE_GNRC_IPV6)) {
                    msg_t msg = { .type = GNRC_IPV6_NIB_IFACE_UP, .content = { .ptr = netif } };

                    msg_send(&msg, gnrc_ipv6_pid);
                }
                break;
            case NETDEV_EVENT_LINK_DOWN:
                if (IS_USED(MODULE_GNRC_IPV6)) {
                    msg_t msg = { .type = GNRC_IPV6_NIB_IFACE_DOWN, .content = { .ptr = netif } };

                    msg_send(&msg, gnrc_ipv6_pid);
                }
                break;
            case NETDEV_EVENT_RX_COMPLETE:
                DEBUG_NETDEV_EVENT("NETDEV_EVENT_RX_COMPLETE\n");
                mac->driver->rx_done_cb(mac);
                break;
#if IS_USED(MODULE_NETDEV_LEGACY_API)
#  if IS_USED(MODULE_NETSTATS_L2) || IS_USED(MODULE_GNRC_NETIF_PKTQ)
            case NETDEV_EVENT_TX_COMPLETE:
                DEBUG_NETDEV_EVENT("NETDEV_EVENT_TX_COMPLETE\n");
                info.recvd_ack = ieee802154_mac_requested_ack(mac);
                info.data_pending = false;
                info.medium_busy = false;
                mac->driver->tx_done_cb(mac, &info);
                break;

            case NETDEV_EVENT_TX_COMPLETE_DATA_PENDING:
                DEBUG_NETDEV_EVENT("NETDEV_EVENT_TX_COMPLETE_DATA_PENDING\n");
                /* pending data after TX may only be indicated by an ack,
                 * so ack reveiced state is always true in this case. */ 
                info.recvd_ack = true;
                info.data_pending = true;
                info.medium_busy = false;
                mac->driver->tx_done_cb(mac, &info);
#    if IS_USED(MODULE_NETSTATS_L2)
                /* we are the only ones supposed to touch this variable,
                 * so no acquire necessary */
                netif->stats.tx_success++;
#    endif  /* IS_USED(MODULE_NETSTATS_L2) */
                if (IS_USED(MODULE_NETSTATS_NEIGHBOR)) {
                    int8_t retries = -1;
                    dev->driver->get(dev, NETOPT_TX_RETRIES_NEEDED, &retries, sizeof(retries));
                    netstats_nb_update_tx(&netif->netif, NETSTATS_NB_SUCCESS, retries + 1);
                }
                break;
#  endif  /* IS_USED(MODULE_NETSTATS_L2) || IS_USED(MODULE_GNRC_NETIF_PKTQ) */
#  if IS_USED(MODULE_NETSTATS_L2) || IS_USED(MODULE_GNRC_NETIF_PKTQ) || \
      IS_USED(MODULE_NETSTATS_NEIGHBOR)
            case NETDEV_EVENT_TX_MEDIUM_BUSY:
                DEBUG_NETDEV_EVENT("NETDEV_EVENT_TX_MEDIUM_BUSY\n");
                info.recvd_ack = false;
                info.data_pending = false;
                info.medium_busy = true;
                mac->driver->tx_done_cb(mac, &info);

                /* update neighbor statistics */
                if (IS_USED(MODULE_NETSTATS_NEIGHBOR)) {
                    int8_t retries = -1;
                    netstats_nb_result_t result = NETSTATS_NB_BUSY;
                    netstats_nb_update_tx(&netif->netif, result, retries + 1);
                }
#    if IS_USED(MODULE_NETSTATS_L2)
                /* we are the only ones supposed to touch this variable,
                 * so no acquire necessary */
                netif->stats.tx_failed++;
#    endif  /* IS_USED(MODULE_NETSTATS_L2) */
                break;
                
            case NETDEV_EVENT_TX_NOACK:
                DEBUG_NETDEV_EVENT("NETDEV_EVENT_TX_NOACK\n");
                info.recvd_ack = false;
                info.data_pending = false;
                info.medium_busy = false;
                mac->driver->tx_done_cb(mac, &info);

                /* update neighbor statistics */
                if (IS_USED(MODULE_NETSTATS_NEIGHBOR)) {
                    int8_t retries = -1;
                    netstats_nb_result_t result = NETSTATS_NB_NOACK;
                    dev->driver->get(dev, NETOPT_TX_RETRIES_NEEDED, &retries, sizeof(retries));
                    netstats_nb_update_tx(&netif->netif, result, retries + 1);
                }
#    if IS_USED(MODULE_NETSTATS_L2)
                /* we are the only ones supposed to touch this variable,
                 * so no acquire necessary */
                netif->stats.tx_failed++;
#    endif  /* IS_USED(MODULE_NETSTATS_L2) */
                break;
#  endif  /* IS_USED(MODULE_NETSTATS_L2) || IS_USED(MODULE_GNRC_NETIF_PKTQ) */
#endif /* IS_USED(MODULE_NETDEV_LEGACY_API) */
            default:
                DEBUG("gnrc_netif: warning: unhandled event %u.\n", event);
        }
    }
}

int gnrc_netif_ieee802154_create(gnrc_netif_t *netif, char *stack, int stacksize,
                                 char priority, const char *name, netdev_t *dev)
{
    int res = gnrc_netif_create(netif, stack, stacksize, priority, name, dev,
                                &ieee802154_ops);
    _init_mac_data(_netif2mac(netif));
    return res;
}

static gnrc_pktsnip_t *_make_netif_hdr(uint8_t *mhr)
{
    gnrc_netif_hdr_t *hdr;
    gnrc_pktsnip_t *snip;
    uint8_t src[IEEE802154_LONG_ADDRESS_LEN], dst[IEEE802154_LONG_ADDRESS_LEN];
    int src_len, dst_len;
    le_uint16_t _pan_tmp;   /* TODO: hand-up PAN IDs to GNRC? */

    dst_len = ieee802154_get_dst(mhr, dst, &_pan_tmp);
    src_len = ieee802154_get_src(mhr, src, &_pan_tmp);
    if ((dst_len < 0) || (src_len < 0)) {
        DEBUG("_make_netif_hdr: unable to get addresses\n");
        return NULL;
    }
    /* allocate space for header */
    snip = gnrc_netif_hdr_build(src, (size_t)src_len, dst, (size_t)dst_len);
    if (snip == NULL) {
        DEBUG("_make_netif_hdr: no space left in packet buffer\n");
        return NULL;
    }
    hdr = snip->data;
    /* set broadcast flag for broadcast destination */
    if ((dst_len == 2) && (dst[0] == 0xff) && (dst[1] == 0xff)) {
        hdr->flags |= GNRC_NETIF_HDR_FLAGS_BROADCAST;
    }
    /* set flags for pending frames */
    if (mhr[0] & IEEE802154_FCF_FRAME_PEND) {
        hdr->flags |= GNRC_NETIF_HDR_FLAGS_MORE_DATA;
    }
    return snip;
}

void _netif_handover_mpdu(gnrc_netif_t *netif, gnrc_pktsnip_t *mpdu, gnrc_pktsnip_t *ieee802154_hdr)
{
    netdev_t *dev = netif->dev;
    gnrc_pktsnip_t *netif_hdr;
    gnrc_netif_hdr_t *hdr;
    //uint8_t *mhr = pkt->data;
    uint8_t *mhr = ieee802154_hdr->data;
    netif_hdr = _make_netif_hdr(mhr);
    if (netif_hdr == NULL) {
        DEBUG("_recv_ieee802154: no space left in packet buffer\n");
        gnrc_pktbuf_release(mpdu);
        return;
    } else {
        hdr = netif_hdr->data;
        gnrc_netif_hdr_set_netif(hdr, netif);
        dev->driver->get(dev, NETOPT_PROTO, &mpdu->type, sizeof(mpdu->type));
        
        /* drop 802.15.4 header.. (mpdu contains raw MSDU afterwards) */ 
        gnrc_pktbuf_remove_snip(mpdu, ieee802154_hdr);
        /* ..and append netif header instead. */ 
        mpdu = gnrc_pkt_append(mpdu, netif_hdr);
        /* pass on packet with netif header. */ 
        _pass_on_packet(mpdu);
    }
}


#if MODULE_GNRC_NETIF_DEDUP
static inline bool _already_received(gnrc_netif_t *netif,
                                     gnrc_netif_hdr_t *netif_hdr,
                                     uint8_t *mhr)
{
    const uint8_t seq = ieee802154_get_seq(mhr);

    return  (netif->last_pkt.seq == seq) &&
            (netif->last_pkt.src_len == netif_hdr->src_l2addr_len) &&
            (memcmp(netif->last_pkt.src, gnrc_netif_hdr_get_src_addr(netif_hdr),
                    netif_hdr->src_l2addr_len) == 0);
}
#endif /* MODULE_GNRC_NETIF_DEDUP */

static gnrc_pktsnip_t *_recv(gnrc_netif_t *netif)
{
    netdev_t *dev = netif->dev;
    netdev_ieee802154_rx_info_t rx_info;
    gnrc_pktsnip_t *pkt = NULL;
    int bytes_expected = dev->driver->recv(dev, NULL, 0, NULL);

    if (bytes_expected >= (int)IEEE802154_MIN_FRAME_LEN) {
        int nread;

        pkt = gnrc_pktbuf_add(NULL, NULL, bytes_expected, GNRC_NETTYPE_UNDEF);
        if (pkt == NULL) {
            DEBUG("_recv_ieee802154: cannot allocate pktsnip.\n");
            /* Discard packet on netdev device */
            dev->driver->recv(dev, NULL, bytes_expected, NULL);
            return NULL;
        }
        nread = dev->driver->recv(dev, pkt->data, bytes_expected, &rx_info);
        if (nread <= 0) {
            gnrc_pktbuf_release(pkt);
            return NULL;
        }
#ifdef MODULE_NETSTATS_L2
        netif->stats.rx_count++;
        netif->stats.rx_bytes += nread;
#endif

        if (netif->flags & GNRC_NETIF_FLAGS_RAWMODE) {
            /* Raw mode, skip packet processing, but provide rx_info via
             * GNRC_NETTYPE_NETIF */
            gnrc_pktsnip_t *netif_snip = gnrc_netif_hdr_build(NULL, 0, NULL, 0);
            if (netif_snip == NULL) {
                DEBUG("_recv_ieee802154: no space left in packet buffer\n");
                gnrc_pktbuf_release(pkt);
                return NULL;
            }
            gnrc_netif_hdr_t *hdr = netif_snip->data;
            hdr->lqi = rx_info.lqi;
            hdr->rssi = rx_info.rssi;
#if IS_USED(MODULE_GNRC_NETIF_TIMESTAMP)
            if (rx_info.flags & NETDEV_RX_IEEE802154_INFO_FLAG_TIMESTAMP) {
                gnrc_netif_hdr_set_timestamp(hdr, rx_info.timestamp);
            }
#endif
            gnrc_netif_hdr_set_netif(hdr, netif);
            pkt = gnrc_pkt_append(pkt, netif_snip);
        }
        else {
            /* Normal mode, try to parse the frame according to IEEE 802.15.4 */
            gnrc_pktsnip_t *ieee802154_hdr, *netif_hdr;
            gnrc_netif_hdr_t *hdr;
            size_t mhr_len = ieee802154_get_frame_hdr_len(pkt->data);
            uint8_t *mhr = pkt->data;
            /* nread was checked for <= 0 before so we can safely cast it to
             * unsigned */
            if ((mhr_len == 0) || ((size_t)nread < mhr_len)) {
                DEBUG("_recv_ieee802154: illegally formatted frame received\n");
                gnrc_pktbuf_release(pkt);
                return NULL;
            }
            netif_hdr = _make_netif_hdr(mhr);
            if (netif_hdr == NULL) {
                DEBUG("_recv_ieee802154: no space left in packet buffer\n");
                gnrc_pktbuf_release(pkt);
                return NULL;
            }
            hdr = netif_hdr->data;

#ifdef MODULE_L2FILTER
            if (!l2filter_pass(dev->filter, gnrc_netif_hdr_get_src_addr(hdr),
                               hdr->src_l2addr_len)) {
                gnrc_pktbuf_release(pkt);
                gnrc_pktbuf_release(netif_hdr);
                DEBUG("_recv_ieee802154: packet dropped by l2filter\n");
                return NULL;
            }
#endif
#ifdef MODULE_GNRC_NETIF_DEDUP
            if (_already_received(netif, hdr, mhr)) {
                gnrc_pktbuf_release(pkt);
                gnrc_pktbuf_release(netif_hdr);
                DEBUG("_recv_ieee802154: packet dropped by deduplication\n");
                return NULL;
            }
            memcpy(netif->last_pkt.src, gnrc_netif_hdr_get_src_addr(hdr),
                   hdr->src_l2addr_len);
            netif->last_pkt.src_len = hdr->src_l2addr_len;
            netif->last_pkt.seq = ieee802154_get_seq(mhr);
#endif /* MODULE_GNRC_NETIF_DEDUP */
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
                        gnrc_pktbuf_release(pkt);
                        gnrc_pktbuf_release(netif_hdr);
                        return NULL;
                    }
                }
                nread -= mic_size;
            }
#endif
            hdr->lqi = rx_info.lqi;
            hdr->rssi = rx_info.rssi;
#if IS_USED(MODULE_GNRC_NETIF_TIMESTAMP)
            if (rx_info.flags & NETDEV_RX_IEEE802154_INFO_FLAG_TIMESTAMP) {
                gnrc_netif_hdr_set_timestamp(hdr, rx_info.timestamp);
            }
#endif
            gnrc_netif_hdr_set_netif(hdr, netif);
            dev->driver->get(dev, NETOPT_PROTO, &pkt->type, sizeof(pkt->type));
            if (IS_ACTIVE(ENABLE_DEBUG)) {
                char src_str[GNRC_NETIF_HDR_L2ADDR_PRINT_LEN];

                DEBUG("_recv_ieee802154: received packet from %s of length %u\n",
                    gnrc_netif_addr_to_str(gnrc_netif_hdr_get_src_addr(hdr),
                                            hdr->src_l2addr_len,
                                            src_str),
                    nread);
                if (IS_USED(MODULE_OD)) {
                    od_hex_dump(pkt->data, nread, OD_WIDTH_DEFAULT);
                }
            }
            /* mark IEEE 802.15.4 header */
            ieee802154_hdr = gnrc_pktbuf_mark(pkt, mhr_len, GNRC_NETTYPE_UNDEF);
            if (ieee802154_hdr == NULL) {
                DEBUG("_recv_ieee802154: no space left in packet buffer\n");
                gnrc_pktbuf_release(pkt);
                gnrc_pktbuf_release(netif_hdr);
                return NULL;
            }


            printf("ieee802154_hdr: %d bytes: ", ieee802154_hdr->size);
            for (unsigned i = 0; i < ieee802154_hdr->size; i++) {
                printf("%02X ", ((uint8_t*)ieee802154_hdr->data)[i]);
            }
            printf("\n");
            if (((uint8_t*)ieee802154_hdr->data)[0] & IEEE802154_FCF_TYPE_MACCMD) {
                printf("received a MAC CMD\n");
                /* check if it is a data request... */
                if (((uint8_t*)pkt->data)[0] == IEEE802154_MAC_CMD_DATA_REQUEST) {
                    /* check if there is pending data for that address */
                    printf("got a Data Request!\n"); 
                    ieee802154_l2addr_t *svdaddr = _save_l2addr_for_idtx(&netif->ieee802154_mac,
                                                   gnrc_netif_hdr_get_src_addr(hdr),
                                                   hdr->src_l2addr_len);
                    printf("saved at %p\n", svdaddr);
                    if (svdaddr) {
                        printf("checking if there is any pending TX for the requester...\n");
                        //_send_indirect_tx_queued_pkt(netif, svdaddr);
                        // TODO: post event to handle IDTX
                        
                        gnrc_pktsnip_t *idtx_pkt = _get_next_indirect_pkt(&netif->ieee802154_mac, svdaddr);
                        if (idtx_pkt) {
                            printf("found pkt in IDTXQ\n");
                            //gnrc_netif_pktq_put(netif, gnrc_pktsnip_t *pkt);
                            
                        }
                        //gnrc_pktqueue_t *qe = gnrc_pktqueue_remove(gnrc_pktqueue_t **queue, gnrc_pktqueue_t *node)
                        
                    }

                }
            }

            nread -= ieee802154_hdr->size;
            gnrc_pktbuf_remove_snip(pkt, ieee802154_hdr);
            pkt = gnrc_pkt_append(pkt, netif_hdr);
        }

        DEBUG("_recv_ieee802154: reallocating MAC payload for upper layer.\n");
        gnrc_pktbuf_realloc_data(pkt, nread);
    } else if (bytes_expected > 0) {
        DEBUG("_recv_ieee802154: received frame is too short\n");
        dev->driver->recv(dev, NULL, bytes_expected, NULL);
    }

    return pkt;
}

char* _pkttype2str(gnrc_nettype_t type)
{
    switch(type) {
        case GNRC_NETTYPE_TX_SYNC: return "GNRC_NETTYPE_TX_SYNC";
        case GNRC_NETTYPE_NETIF: return "GNRC_NETTYPE_NETIF";
        case GNRC_NETTYPE_UNDEF: return "GNRC_NETTYPE_UNDEF";
        case GNRC_NETTYPE_SIXLOWPAN: return "GNRC_NETTYPE_SIXLOWPAN";
        //case GNRC_NETTYPE_GOMACH: return "GNRC_NETTYPE_GOMACH";
        //case GNRC_NETTYPE_LWMAC: return "GNRC_NETTYPE_LWMAC";
        //case GNRC_NETTYPE_CUSTOM: return "GNRC_NETTYPE_CUSTOM";
        case GNRC_NETTYPE_IPV6: return "GNRC_NETTYPE_IPV6";
        //case GNRC_NETTYPE_IPV6_EXT: return "GNRC_NETTYPE_IPV6_EXT";
        case GNRC_NETTYPE_ICMPV6: return "GNRC_NETTYPE_ICMPV6";
        //jcase GNRC_NETTYPE_TCP: return "GNRC_NETTYPE_TCP";
        case GNRC_NETTYPE_UDP: return "GNRC_NETTYPE_UDP";
        //case GNRC_NETTYPE_CCN: return "GNRC_NETTYPE_CCN";
        //case GNRC_NETTYPE_CCN_CHUNK: return "GNRC_NETTYPE_CCN_CHUNK";
        //case GNRC_NETTYPE_NDN: return "GNRC_NETTYPE_NDN";
        //case GNRC_NETTYPE_LORAWAN: return "GNRC_NETTYPE_LORAWAN";
        //case GNRC_NETTYPE_TEST: return "GNRC_NETTYPE_TEST";
        case GNRC_NETTYPE_NUMOF: return "GNRC_NETTYPE_NUMOF";
        default: return "NONE";
    }
}

void _print_pktsnip(gnrc_pktsnip_t *p, const char* prefix_str) {
    while (p) {
        printf("%s pktsnip (%s) has %d bytes\n", prefix_str, _pkttype2str(p->type), p->size);
        for (unsigned i = 0; i < p->size; i++) {
            printf("0x%02x ", ((uint8_t*)p->data)[i]);
        }
        printf("\n");
        p = p->next;
    }
}

void _print_pktsnip_metadata(gnrc_pktsnip_t *snip)
{
    while (snip) {
        printf("size: %u users: %u\n", snip->size, snip->users);
        snip = snip->next;
    }
}

void _build_mcps_data_request(gnrc_netif_t *netif, gnrc_pktsnip_t *pkt,
                              ieee802154_mcps_data_request_t *request)
{
    //_print_pktsnip(pkt, "BUILD MCPS-DR: ");

    netdev_t *dev = netif->dev;
    netdev_ieee802154_t *netdev_ieee802154 = container_of(dev, netdev_ieee802154_t, netdev);
    
    gnrc_netif_hdr_t *netif_hdr = pkt->data;
 
    request->ack_tx = netdev_ieee802154->flags & NETDEV_IEEE802154_ACK_REQ;
    DEBUG("REQ ACK: %s\n", request->ack_tx ? "true" : "false");
    // TODO: dst pan might be different from device pan
    request->dst_pan_id = byteorder_htols(netdev_ieee802154->pan);
    request->src_addr_mode = (netdev_ieee802154->flags & NETDEV_IEEE802154_SRC_MODE_LONG) ?
                             IEEE802154_ADDR_MODE_EXTENDED : IEEE802154_ADDR_MODE_SHORT;

#if IS_USED(MODULE_IEEE802154_SECURITY)
    if (netdev_ieee802154->flags & NETDEV_IEEE802154_SECURITY_EN) {
        request->security_level = netdev_ieee802154->sec_ctx.security_level;
    } else {
        request->security_level = IEEE802154_SEC_SCF_SECLEVEL_NONE;
    }
#else
    request->security_level = IEEE802154_SEC_SCF_SECLEVEL_NONE;
#endif

    /* prepare destination address */
    if (netif_hdr->flags & /* If any of these flags is set assume broadcast */
        (GNRC_NETIF_HDR_FLAGS_BROADCAST | GNRC_NETIF_HDR_FLAGS_MULTICAST)) {
        memcpy(request->dst_address.l2addr, ieee802154_addr_bcast, IEEE802154_ADDR_BCAST_LEN);
        request->dst_address.l2addr_len = IEEE802154_ADDR_BCAST_LEN;
        //TODO this info is implicitly contained in l2_addr_len
        request->dst_addr_mode = IEEE802154_ADDR_MODE_SHORT;
    } else {
        memcpy(request->dst_address.l2addr, gnrc_netif_hdr_get_dst_addr(netif_hdr), netif_hdr->dst_l2addr_len);
        request->dst_address.l2addr_len = netif_hdr->dst_l2addr_len;
        if (request->dst_address.l2addr_len == IEEE802154_SHORT_ADDRESS_LEN) {
            request->dst_addr_mode = IEEE802154_ADDR_MODE_SHORT;
        } else {
            request->dst_addr_mode = IEEE802154_ADDR_MODE_EXTENDED;
        }
    }

    if (netdev_ieee802154->flags & NETDEV_IEEE802154_SECURITY_EN) {
        /* need to include long source address because the recipient
           will need it to decrypt the frame */
        request->src_addr_mode = IEEE802154_ADDR_MODE_EXTENDED;
    }

    request->indirect_tx = ieee802154_dst_addr_uses_idtx(_netif2mac(netif), &request->dst_address);

    /* remove netif header as we already retreived all needed data from it */
    gnrc_pktsnip_t *netif_hdr_snip = pkt;
    pkt = gnrc_pktbuf_remove_snip(pkt, netif_hdr_snip);
    /* set the MSDU to the payload given in the netif pkt */
    request->msdu.pkt = pkt;
}

extern void _print_data_request(ieee802154_mcps_data_request_t *r);

static int _send(gnrc_netif_t *netif, gnrc_pktsnip_t *pkt)
{
    gnrc_netif_hdr_t *netif_hdr;
    int res = 0;

    if (pkt == NULL) {
        DEBUG("_send_ieee802154: pkt was NULL\n");
        return -EINVAL;
    }
    if (pkt->type != GNRC_NETTYPE_NETIF) {
        DEBUG("_send_ieee802154: first header is not generic netif header\n");
        return -EBADMSG;
    }
    netif_hdr = pkt->data;
    
    //_print_pktsnip(pkt, "NETIF _send: ");

#ifdef MODULE_NETSTATS_L2
    if (netif_hdr->flags &
            (GNRC_NETIF_HDR_FLAGS_BROADCAST | GNRC_NETIF_HDR_FLAGS_MULTICAST)) {
        netif->stats.tx_mcast_count++;
    }
    else {
        netif->stats.tx_unicast_count++;
    }
#endif
    
    ieee802154_mcps_data_request_t request;
    
    _build_mcps_data_request(netif, pkt, &request);
    
    //_print_data_request(&request);

    ieee802154_mcps_data_request(&netif->ieee802154_mac, &request, _ieee802154_mcps_data_confirm_cb);

    return res;
}
/** @} */
