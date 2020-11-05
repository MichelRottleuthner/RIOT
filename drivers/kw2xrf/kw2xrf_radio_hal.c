/*
 * Copyright (C) 2020 HAW Hamburg
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 *
 */

/**
 * @ingroup     drivers_kw2xrf
 * @{
 *
 * @file
 * @brief       IEEE 802.15.4 Radio HAL implementation for the KW2x RF driver
 *
 * @author      Michel Rottleuthner <michel.rottleuthner@haw-hamburg.de>
 *
 * @}
 */

#include <assert.h>
#include <errno.h>
#include <stdio.h>

#include "net/gnrc.h"

#define LOG_LEVEL LOG_DEBUG
#include "log.h"
#include "kw2xrf.h"
#include "kw2xrf_spi.h"
#include "kw2xrf_getset.h"
#include "kw2xrf_intern.h"
#include "net/ieee802154/radio.h"
#include "kw2xrf_params.h"
#include "event/thread.h"

#define _MACACKWAITDURATION         (864 / 16) /* 864us * 62500Hz */

static const ieee802154_radio_ops_t kw2xrf_ops;

//ieee802154_dev_t kw2xrf_dev = {
//    .driver = &kw2xrf_ops,
//};

#define KW2XRF_NUM                    ARRAY_SIZE(kw2xrf_params)

kw2xrf_dev_evt_ctx_t kw2xrf_dev_ctxs[KW2XRF_NUM];

void kw2xrf_radio_hal_irq_handler(ieee802154_dev_t *dev)
{
    kw2xrf_t *kw_dev = container_of(dev, kw2xrf_t, hal);
    uint8_t dregs[MKW2XDM_PHY_CTRL4 + 1];

    kw2xrf_read_dregs(kw_dev, MKW2XDM_IRQSTS1, dregs, MKW2XDM_PHY_CTRL4 + 1);
    kw2xrf_mask_irq_b(kw_dev);

    printf("kw2xrf_radio_hal_irq_handler\n");

    LOG_DEBUG("[kw2xrf] CTRL1 %0x, IRQSTS1 %0x, IRQSTS2 %0x\n",
          dregs[MKW2XDM_PHY_CTRL1], dregs[MKW2XDM_IRQSTS1], dregs[MKW2XDM_IRQSTS2]);
    uint8_t irqsts1 = 0;

    switch (dregs[MKW2XDM_PHY_CTRL1] & MKW2XDM_PHY_CTRL1_XCVSEQ_MASK) {
        case XCVSEQ_RECEIVE:
            LOG_DEBUG("IRQ handler: [XCVSEQ_RECEIVE]\n");
            if (dregs[MKW2XDM_IRQSTS1] & MKW2XDM_IRQSTS1_RXIRQ) {
                LOG_DEBUG("        finished RXSEQ\n");
                irqsts1 |= MKW2XDM_IRQSTS1_RXIRQ;
            }

            if (dregs[MKW2XDM_IRQSTS1] & MKW2XDM_IRQSTS1_SEQIRQ) {
                LOG_DEBUG("        finished SEQIRQ\n");
                irqsts1 |= MKW2XDM_IRQSTS1_SEQIRQ;
            }

            kw2xrf_write_dreg(kw_dev, MKW2XDM_IRQSTS1, irqsts1);
            break;

        case XCVSEQ_TRANSMIT:
            LOG_DEBUG("IRQ handler: [XCVSEQ_TRANSMIT]\n");
            if (dregs[MKW2XDM_IRQSTS1] & MKW2XDM_IRQSTS1_TXIRQ) {
                LOG_DEBUG("        finished TXSEQ\n");
                irqsts1 |= MKW2XDM_IRQSTS1_TXIRQ;
            }

            if (dregs[MKW2XDM_IRQSTS1] & MKW2XDM_IRQSTS1_SEQIRQ) {
                LOG_DEBUG("        finished SEQIRQ\n");
                irqsts1 |= MKW2XDM_IRQSTS1_SEQIRQ;

                //if (dregs[MKW2XDM_IRQSTS1] & MKW2XDM_IRQSTS1_CCAIRQ) {
                //    irqsts1 |= MKW2XDM_IRQSTS1_CCAIRQ;
                //    if (dregs[MKW2XDM_IRQSTS2] & MKW2XDM_IRQSTS2_CCA) {
                //        DEBUG("[kw2xrf] CCA CH busy\n");
                //        netdev->event_callback(netdev, NETDEV_EVENT_TX_MEDIUM_BUSY);
                //    }
                //    else {
                //        netdev->event_callback(netdev, NETDEV_EVENT_TX_COMPLETE);
                //    }
                //}
                //assert(dev->pending_tx != 0);
                //dev->pending_tx--;
                //kw2xrf_set_idle_sequence(dev);
            }

            kw2xrf_write_dreg(kw_dev, MKW2XDM_IRQSTS1, irqsts1);
            break;

        case XCVSEQ_CCA:
            LOG_DEBUG("IRQ handler: [XCVSEQ_CCA]\n");
            ////_isr_event_seq_cca(netdev, dregs);
            /* onle handle IRQ if CCA *and* sequence (warmdown) finished */
            if ((dregs[MKW2XDM_IRQSTS1] & MKW2XDM_IRQSTS1_CCAIRQ) &&
                (dregs[MKW2XDM_IRQSTS1] & MKW2XDM_IRQSTS1_SEQIRQ)) {
                /* clear CCA IRQ and SEQIRQ wich gets asserted after warmdown */
                kw2xrf_write_dreg(kw_dev, MKW2XDM_IRQSTS1, MKW2XDM_IRQSTS1_CCAIRQ | MKW2XDM_IRQSTS1_SEQIRQ);
                kw2xrf_set_idle_sequence(kw_dev);

                kw_dev->channel_free = !(dregs[MKW2XDM_IRQSTS2] & MKW2XDM_IRQSTS2_CCA);
                kw_dev->waiting_for_cca = false;
                dev->cb(dev, IEEE802154_RADIO_CONFIRM_CCA);
            }
            break;

        case XCVSEQ_TX_RX:
            LOG_DEBUG("IRQ handler: [XCVSEQ_TX_RX]\n");
            if (dregs[MKW2XDM_IRQSTS1] & MKW2XDM_IRQSTS1_TXIRQ) {
                LOG_DEBUG("        finished TXSEQ\n");
                irqsts1 |= MKW2XDM_IRQSTS1_TXIRQ;

                //if (dregs[MKW2XDM_PHY_CTRL1] & MKW2XDM_PHY_CTRL1_RXACKRQD) {
                //    LOG_DEBUG("    enable ACK RX timeout\n");
                //    /* Allow TMR3IRQ to cancel RX operation */
                //    kw2xrf_timer3_seq_abort_on(kw_dev);
                //    /* Enable interrupt for TMR3 and set timer */
                //    kw2xrf_abort_rx_ops_enable(kw_dev, _MACACKWAITDURATION);
                //}
            }

            if (dregs[MKW2XDM_IRQSTS1] & MKW2XDM_IRQSTS1_SEQIRQ) {
                LOG_DEBUG("        finished SEQIRQ\n");
                irqsts1 |= MKW2XDM_IRQSTS1_SEQIRQ;

                if (dregs[MKW2XDM_IRQSTS1] & MKW2XDM_IRQSTS1_CCAIRQ) {
                    irqsts1 |= MKW2XDM_IRQSTS1_CCAIRQ;
                    if (dregs[MKW2XDM_IRQSTS2] & MKW2XDM_IRQSTS2_CCA) {
                        LOG_DEBUG("    CCA CH busy\n");
                        //netdev->event_callback(netdev, NETDEV_EVENT_TX_MEDIUM_BUSY);
                    }
                }

                if (dregs[MKW2XDM_IRQSTS3] & MKW2XDM_IRQSTS3_TMR3IRQ) {
                    kw_dev->ack_rcvd = false; /* ACK timed out */
                } else {
                    kw_dev->ack_rcvd = true;
                }

                /* Disallow TMR3IRQ to cancel RX operation */
                kw2xrf_timer3_seq_abort_off(kw_dev);
                /* Disable interrupt for TMR3 and reset TMR3IRQ */
                kw2xrf_abort_rx_ops_disable(kw_dev);
                ///* Go back to idle state */
                //kw2xrf_set_idle_sequence(kw_dev);
                kw_dev->tx_done = true;
                dev->cb(dev, IEEE802154_RADIO_CONFIRM_TX_DONE);
            }

            kw2xrf_write_dreg(kw_dev, MKW2XDM_IRQSTS1, irqsts1);

            //_isr_event_seq_tr(netdev, dregs);
            break;

        case XCVSEQ_CONTINUOUS_CCA:
            LOG_DEBUG("IRQ handler: [XCVSEQ_CONTINUOUS_CCA]\n");
            //_isr_event_seq_ccca(netdev, dregs);
            break;

        case XCVSEQ_IDLE:
            LOG_DEBUG("IRQ handler: [XCVSEQ_IDLE]\n");
            break;
        default:
            LOG_DEBUG("[IRQ handler: undefined seq state in isr\n");
            break;
    }

    //uint8_t irqsts2 = 0;
    //if (dregs[MKW2XDM_IRQSTS2] & MKW2XDM_IRQSTS2_PB_ERR_IRQ) {
    //    DEBUG("[kw2xrf] untreated PB_ERR_IRQ\n");
    //    irqsts2 |= MKW2XDM_IRQSTS2_PB_ERR_IRQ;
    //}
    //if (dregs[MKW2XDM_IRQSTS2] & MKW2XDM_IRQSTS2_WAKE_IRQ) {
    //    DEBUG("[kw2xrf] untreated WAKE_IRQ\n");
    //    irqsts2 |= MKW2XDM_IRQSTS2_WAKE_IRQ;
    //}
    //kw2xrf_write_dreg(kw_dev, MKW2XDM_IRQSTS2, irqsts2);
    //if (ENABLE_DEBUG) {
    //    /* for debugging only */
    //    kw2xrf_read_dregs(dev, MKW2XDM_IRQSTS1, dregs, MKW2XDM_IRQSTS1 + 3);
    //    if (dregs[MKW2XDM_IRQSTS1] & 0x7f) {
    //        DEBUG("[kw2xrf] IRQSTS1 contains untreated IRQs: 0x%02x\n",
    //            dregs[MKW2XDM_IRQSTS1]);
    //    }
    //    if (dregs[MKW2XDM_IRQSTS2] & 0x02) {
    //        DEBUG("[kw2xrf] IRQSTS2 contains untreated IRQs: 0x%02x\n",
    //            dregs[MKW2XDM_IRQSTS2]);
    //    }
    //    if (dregs[MKW2XDM_IRQSTS3] & 0x0f) {
    //        DEBUG("[kw2xrf] IRQSTS3 contains untreated IRQs: 0x%02x\n",
    //            dregs[MKW2XDM_IRQSTS3]);
    //    }
    //}

    kw2xrf_enable_irq_b(kw_dev);
}

static void kw2xrf_irq_cb(void *ctx) {
    //printf("kw2xrf_irq_cb\n");
    kw2xrf_dev_evt_ctx_t *c = (kw2xrf_dev_evt_ctx_t*)ctx;
    /* calls the below kw2xrf_irq_event_handler from the the event thread */
    //event_post(EVENT_PRIO_HIGHEST, &c->event);
    event_post(EVENT_PRIO_MEDIUM, &c->event);
}

static void kw2xrf_irq_event_handler(event_t *evt) {
    kw2xrf_dev_evt_ctx_t *ctx = container_of(evt, kw2xrf_dev_evt_ctx_t, event);
    kw2xrf_radio_hal_irq_handler((ieee802154_dev_t*)&ctx->dev.hal);
}

//static event_queue_t offload_queues[KW2XRF_NUM];
//void *offload_thread(void *arg) {
//    event_queue_t *queue = (event_queue_t*)arg;
//    event_queue_init(queue);
//    event_loop(queue);
//}

void kw2xrf_auto_init_offloaded(void) {
    for (unsigned i = 0; i < KW2XRF_NUM; i++) {
        const kw2xrf_params_t *p = &kw2xrf_params[i];

        LOG_DEBUG("[auto_init_netif] initializing kw2xrf #%u\n", i);
        if (IS_USED(MODULE_IEEE802154_RADIO_HAL)) {
            kw2xrf_dev_ctxs[i].dev.hal.driver = &kw2xrf_ops;
            kw2xrf_dev_ctxs[i].event.handler = kw2xrf_irq_event_handler;
            kw2xrf_new_init(&kw2xrf_dev_ctxs[i].dev, (kw2xrf_params_t*)p,
                            kw2xrf_irq_cb, &kw2xrf_dev_ctxs[i]);
        }
    }
}

static int _write(ieee802154_dev_t *dev, const iolist_t *iolist)
{
    printf("_write\n");
    kw2xrf_t *kw_dev = container_of(dev, kw2xrf_t, hal);
    size_t len = 0;
    /* TODO: replace with address-offset spi transfer */
    uint8_t pkt_buf[KW2XRF_MAX_PKT_LENGTH];
    /* load packet data into buffer */
    for (const iolist_t *iol = iolist; iol; iol = iol->iol_next) {
        /* current packet data + FCS too long */
        if ((len + iol->iol_len + IEEE802154_FCS_LEN) > KW2XRF_MAX_PKT_LENGTH) {
            LOG_ERROR("[kw2xrf] packet too large (%u byte) to be send\n",
                  (unsigned)len + IEEE802154_FCS_LEN);
            return -EOVERFLOW;
        }

        /* start after pkt len byte */
        memcpy(&pkt_buf[len + 1], iol->iol_base, iol->iol_len);
        len += iol->iol_len;
    }

    printf("len: %u\n", len);
    pkt_buf[0] = len + IEEE802154_FCS_LEN;
    for (unsigned i = 0; i < len; i++) {
        printf("%02X ", pkt_buf[i+1]);
    }
    printf("\n");
    for (unsigned i = 0; i < len; i++) {
        printf(" %c ", ((pkt_buf[i+1] >= 0x20) && (pkt_buf[i +1] <= 0x7E)) ? (char)pkt_buf[i +1] : '?');
    }
    printf("\n");

    /* check if ack req bit is set to decide which transmition sequence is best
       to send the frame */
    kw_dev->ack_requested = pkt_buf[1] & IEEE802154_FCF_ACK_REQ;

    kw2xrf_write_fifo(kw_dev, pkt_buf, pkt_buf[0]);
    return 0;
}

static int _request_transmit(ieee802154_dev_t *dev)
{
    kw2xrf_t *kw_dev = container_of(dev, kw2xrf_t, hal);
    printf("_request_transmit %s\n", kw_dev->ack_requested ? "TR" : "T");
    kw_dev->tx_done = false;

    if (kw_dev->ack_requested) {
        /* expect an ACK after TX */
        kw2xrf_set_dreg_bit(kw_dev, MKW2XDM_PHY_CTRL1,
                            MKW2XDM_PHY_CTRL1_RXACKRQD);
    } else {
        /* don't expect an ACK after TX */
        kw2xrf_clear_dreg_bit(kw_dev, MKW2XDM_PHY_CTRL1,
                              MKW2XDM_PHY_CTRL1_RXACKRQD);
    }

    /* A T sequence performs a simple transmit and returns to idle, a TR
    sequence waits for the requested ACK response after the transmition */
    kw2xrf_set_sequence(kw_dev, kw_dev->ack_requested ? XCVSEQ_TX_RX :
                                                        XCVSEQ_TRANSMIT);

    return 0;
}

static int _confirm_transmit(ieee802154_dev_t *dev, ieee802154_tx_info_t *info)
{
    kw2xrf_t *kw_dev = container_of(dev, kw2xrf_t, hal);

    if (!kw_dev->tx_done) {
        printf("_confirm_transmit [BUSY]\n");
        return -EAGAIN;
    }

    printf("_confirm_transmit [DONE] (%sACK)\n", kw_dev->ack_rcvd ? "" : "No ");

    if (info) {
        if (kw_dev->ack_rcvd) {
            info->status = TX_STATUS_SUCCESS;
        } else {
            info->status = TX_STATUS_NO_ACK;
        }
    }
    return 0;
}

static int _len(ieee802154_dev_t *dev)
{
    kw2xrf_t *kw_dev = container_of(dev, kw2xrf_t, hal);
    printf("_len\n");

    size_t pkt_len = kw2xrf_read_dreg(kw_dev, MKW2XDM_RX_FRM_LEN);
    return pkt_len;
}

static int _read(ieee802154_dev_t *dev, void *buf, size_t size, ieee802154_rx_info_t *info)
{
    kw2xrf_t *kw_dev = container_of(dev, kw2xrf_t, hal);
    printf("_read\n");

    kw2xrf_read_fifo(kw_dev, (uint8_t *)buf, size);

    if (info != NULL) {
        info->lqi = kw2xrf_read_dreg(kw_dev, MKW2XDM_LQI_VALUE);
        info->rssi = kw2xrf_get_rssi(info->lqi);
    }

    return 0;
}

static int _confirm_cca(ieee802154_dev_t *dev)
{
    kw2xrf_t *kw_dev = container_of(dev, kw2xrf_t, hal);
    printf("_confirm_cca\n");
    if (kw_dev->waiting_for_cca) {
        return -EAGAIN;
    }
    return kw_dev->channel_free;
}

static int _request_cca(ieee802154_dev_t *dev)
{
    kw2xrf_t *kw_dev = container_of(dev, kw2xrf_t, hal);
    printf("_request_cca\n");
    kw_dev->waiting_for_cca = true;
    kw2xrf_set_sequence(kw_dev, XCVSEQ_CCA);
    return 0;
}

static int _set_cca_threshold(ieee802154_dev_t *dev, int8_t threshold)
{
    kw2xrf_t *kw_dev = container_of(dev, kw2xrf_t, hal);
    printf("_set_cca_threshold to %d\n", threshold);
    /* normalize to absolute value */
    if (threshold < 0) {
        threshold = -threshold;
    }

    kw2xrf_write_iregs(kw_dev, MKW2XDMI_CCA1_THRESH, (uint8_t*)&threshold, 1);
    kw2xrf_write_iregs(kw_dev, MKW2XDMI_CCA2_THRESH, (uint8_t*)&threshold, 1);
    return 0;
}

static int _config_phy(ieee802154_dev_t *dev, const ieee802154_phy_conf_t *conf)
{
    printf("_config_phy\n");
    kw2xrf_t *kw_dev = container_of(dev, kw2xrf_t, hal);
    kw2xrf_set_tx_power(kw_dev, conf->pow);
    return kw2xrf_set_channel(kw_dev, conf->channel) == 0 ? 0 : -EINVAL;
}

static int _request_set_trx_state(ieee802154_dev_t *dev, ieee802154_trx_state_t state)
{
    kw2xrf_t *kw_dev = container_of(dev, kw2xrf_t, hal);
    switch (state) {
        case IEEE802154_TRX_STATE_TRX_OFF:
            printf("_request_set_trx_state [IEEE802154_TRX_STATE_TRX_OFF]\n");
            kw2xrf_set_power_mode(kw_dev, KW2XRF_DOZE);
            break;
        case IEEE802154_TRX_STATE_RX_ON:
            printf("_request_set_trx_state [IEEE802154_TRX_STATE_RX_ON]\n");
            /* TODO: ensure FIFO is flushed (maybe below already does) */
            kw2xrf_set_sequence(kw_dev, XCVSEQ_RECEIVE);
            break;
        case IEEE802154_TRX_STATE_TX_ON:
            printf("_request_set_trx_state [IEEE802154_TRX_STATE_TX_ON]\n");
            /* no-op on this radio as it can always perform transmit if in ON mode */
            //kw2xrf_set_power_mode(kw_dev, KW2XRF_IDLE);
            break;
    }
    return 0;
}

static int _confirm_set_trx_state(ieee802154_dev_t *dev)
{
    printf("_confirm_set_trx_state\n");
    (void) dev;
    return 0;
}

static int _off(ieee802154_dev_t *dev)
{
    printf("_off\n");
    kw2xrf_t *kw_dev = container_of(dev, kw2xrf_t, hal);
    /* TODO: do COMPLETE powerdown (PIN?) */
    kw2xrf_set_power_mode(kw_dev, KW2XRF_HIBERNATE);
    return 0;
}

static bool _get_cap(ieee802154_dev_t *dev, ieee802154_rf_caps_t cap)
{
    (void) dev;
    printf("_get_cap\n");
    switch (cap) {
        case IEEE802154_CAP_24_GHZ:
        case IEEE802154_CAP_IRQ_TX_DONE:
        case IEEE802154_CAP_IRQ_CCA_DONE:
        case IEEE802154_CAP_IRQ_ACK_TIMEOUT:
        //case IEEE802154_CAP_IRQ_RX_START: // TODO: not supported directly but possible via watermark register (fire interrupt n bytes after SFD)
        //case IEEE802154_CAP_IRQ_TX_START:
        //case IEEE802154_CAP_AUTO_CSMA:
            return true;
        default:
            return false;
    }
}

static int _set_hw_addr_filter(ieee802154_dev_t *dev, const network_uint16_t *short_addr,
                               const eui64_t *ext_addr, const uint16_t *pan_id)
{
    printf("_set_hw_addr_filter\n");
    kw2xrf_t *kw_dev = container_of(dev, kw2xrf_t, hal);
    kw2xrf_set_pan(kw_dev, *pan_id);
    kw2xrf_set_addr_short(kw_dev, byteorder_ntohs(*short_addr));
    kw2xrf_set_addr_long(kw_dev, byteorder_ntohll(ext_addr->uint64));
    return 0;
}

static int _request_on(ieee802154_dev_t *dev)
{
    printf("_request_on\n");
    kw2xrf_t *kw_dev = container_of(dev, kw2xrf_t, hal);
    /* enables xtal and puts power managment controller to high power mode */
    kw2xrf_set_power_mode(kw_dev, KW2XRF_IDLE);
    return 0;
}

static int _confirm_on(ieee802154_dev_t *dev)
{
    printf("_confirm_on\n");
    kw2xrf_t *kw_dev = container_of(dev, kw2xrf_t, hal);
    size_t pwr_modes = kw2xrf_read_dreg(kw_dev, MKW2XDM_PWR_MODES);
    return (pwr_modes & MKW2XDM_PWR_MODES_XTAL_READY) ? 0 : -EAGAIN;
}

static int _set_cca_mode(ieee802154_dev_t *dev, ieee802154_cca_mode_t mode)
{
    kw2xrf_t *kw_dev = container_of(dev, kw2xrf_t, hal);
    printf("_set_cca_mode\n");

    uint8_t dev_mode = 0;
    switch (mode) {
        case IEEE802154_CCA_MODE_ED_THRESHOLD:
            dev_mode = 1;
            break;
        case IEEE802154_CCA_MODE_CARRIER_SENSING:
            dev_mode = 2;
            break;
        case IEEE802154_CCA_MODE_ED_THRESH_AND_CS:
        case IEEE802154_CCA_MODE_ED_THRESH_OR_CS:
            dev_mode = 3;
            break;
    }

    kw2xrf_set_cca_mode(kw_dev, dev_mode);
    return 0;
}

static int _set_rx_mode(ieee802154_dev_t *dev, ieee802154_rx_mode_t mode)
{
    kw2xrf_t *kw_dev = container_of(dev, kw2xrf_t, hal);
    //bool promisc = false;
    //bool ack_filter = true;
    switch (mode) {
        case IEEE802154_RX_AACK_DISABLED:
            printf("_set_rx_mode [IEEE802154_RX_AACK_DISABLED]\n");
            kw2xrf_clear_dreg_bit(kw_dev, MKW2XDM_PHY_CTRL1,
                                  MKW2XDM_PHY_CTRL1_AUTOACK);
            break;
        case IEEE802154_RX_AACK_ENABLED:
            printf("_set_rx_mode [IEEE802154_RX_AACK_ENABLED]\n");
            kw2xrf_set_dreg_bit(kw_dev, MKW2XDM_PHY_CTRL1,
                                MKW2XDM_PHY_CTRL1_AUTOACK);
            break;
        case IEEE802154_RX_AACK_FRAME_PENDING:
            printf("_set_rx_mode [IEEE802154_RX_AACK_FRAME_PENDING]\n");
            break;
        case IEEE802154_RX_PROMISC:
            printf("_set_rx_mode [IEEE802154_RX_PROMISC]\n");
            /* disable auto ACKs in promiscuous mode */
            kw2xrf_clear_dreg_bit(kw_dev, MKW2XDM_PHY_CTRL1,
                                  MKW2XDM_PHY_CTRL1_AUTOACK | MKW2XDM_PHY_CTRL1_RXACKRQD);
            /* enable promiscuous mode */
            kw2xrf_set_dreg_bit(kw_dev, MKW2XDM_PHY_CTRL4,
                                MKW2XDM_PHY_CTRL4_PROMISCUOUS);
            break;
        case IEEE802154_RX_WAIT_FOR_ACK:
            printf("_set_rx_mode [IEEE802154_RX_WAIT_FOR_ACK]\n");
            //ack_filter = false;
            break;
    }

    return 0;
}

static int _set_csma_params(ieee802154_dev_t *dev, const ieee802154_csma_be_t *bd, int8_t retries)
{
    printf("_set_csma_params %d retries\n", retries);
    kw2xrf_t *kw_dev = container_of(dev, kw2xrf_t, hal);
    (void) bd;

    if (retries < 0) {
        kw2xrf_clear_dreg_bit(kw_dev, MKW2XDM_PHY_CTRL1,
                             MKW2XDM_PHY_CTRL1_CCABFRTX);
        return 0;
    } else if (retries == 0) {
        kw2xrf_set_dreg_bit(kw_dev, MKW2XDM_PHY_CTRL1,
                           MKW2XDM_PHY_CTRL1_CCABFRTX);
        return 0;
    }

    return -1;
}

static const ieee802154_radio_ops_t kw2xrf_ops = {
    .write = _write,
    .read = _read,
    .request_transmit = _request_transmit,
    .confirm_transmit = _confirm_transmit,
    .len = _len,
    .off = _off,
    .request_on = _request_on,
    .confirm_on = _confirm_on,
    .request_set_trx_state = _request_set_trx_state,
    .confirm_set_trx_state = _confirm_set_trx_state,
    .request_cca = _request_cca,
    .confirm_cca = _confirm_cca,
    .get_cap = _get_cap,
    .set_cca_threshold = _set_cca_threshold,
    .set_cca_mode = _set_cca_mode,
    .config_phy = _config_phy,
    .set_hw_addr_filter = _set_hw_addr_filter,
    .set_csma_params = _set_csma_params,
    .set_rx_mode = _set_rx_mode,
};
