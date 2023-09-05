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
 * @author  José I. Álamos <jose.alamos@haw-hamburg.de>
 */

#include "opendsme/opendsme.h"
#include "opendsme/DSMEPlatform.h"
#include "ztimer.h"
#include "iolist.h"
#include "event.h"
#include "event/thread.h"
#include "luid.h"
#include "dsmeAdaptionLayer/scheduling/TPS.h"
#include "dsmeAdaptionLayer/scheduling/StaticScheduling.h"
#include "board.h"

#define ENABLE_DEBUG 0
#include "debug.h"

/* Use default openDSME value for TPS scheduler alpha */
#define OPENDSME_TPS_ALPHA (0.1f)

/* one symbol is 16 µs so the effective frequency is 62.5 kHz */
#define SYMBOL_TIMER_FREQUENCY (62500)

/* The low power timer integration uses ztimer which
 * handles extension to 32 bits. */
#define HW_COUNTER_MAX (0xFFFFFFFF)

#if DSME_USE_LOW_POWER_TIMER == 1
/* The low power timer integration uses the RTT via ztimer. */
#define HWTIMER_FREQUENCY RTT_FREQUENCY
#define ZTIMER_INSTANCE   ZTIMER_MSEC_BASE
#else
#define HWTIMER_FREQUENCY (1000000)
#define ZTIMER_INSTANCE   ZTIMER_USEC
#endif

namespace dsme {

uint32_t _hw_counter_base = 0x0;
uint32_t _rescaled_symbol_counter_offs = 0x0;

/* The 64 bit timer extension uses this value to maintain its state
 * and gracefully handle overflows. */
uint64_t extended64_bit_timer_base = 0;
uint64_t _counter_read_hw_ticks_64(void);

static uint32_t _symbols_to_hw_ticks(uint32_t symbols);
uint32_t _hw_counter_read(void);

/* The one and only openDSME instance */
DSMEPlatform *DSMEPlatform::instance = nullptr;

/********************* C functions *********************/

static void _cca_ev_handler(event_t *ev);
static void _acktimer_ev_handler(event_t *ev);
static void _acktimer_cb(void *arg);
static void _timer_ev_handler(event_t *ev);
static void _tx_done_handler(event_t *ev);
static void _rx_done_handler(event_t *ev);
static void _handle_rx_offload(event_t *ev);
static void _start_of_cfp_handler(event_t *ev);

/* Event used for ACK Timeout */
static event_t acktimer_ev = {{}, _acktimer_ev_handler};

/* Event used for CCA Done */
static event_t cca_ev = {{}, _cca_ev_handler};

/* Event used for timer events */
static event_t timer_event = {{}, _timer_ev_handler};

/* Event used for TX Done */
static event_t tx_done_event = {{}, _tx_done_handler};

/* Event used for RX Done */
static event_t rx_done_event = {{}, _rx_done_handler};

/* Event used for offloading the receive procedure */
static event_t rx_offload_ev = {{}, _handle_rx_offload};

/* Event used for offloading the start of a CFP */
static event_t start_of_cfp_ev = {{}, _start_of_cfp_handler};

void _handle_rx_offload(event_t *ev)
{
    dsme::DSMEPlatform::instance->processRxOffload();
}

void _start_of_cfp_handler(event_t *ev)
{
    dsme::DSMEPlatform::instance->getDSME().handleStartOfCFP();
    dsme::DSMEPlatform::instance->updateVisual();
}

static void _cca_ev_handler(event_t *ev)
{
    dsme::DSMEPlatform::instance->processCCAEvent();
}

static void _acktimer_ev_handler(event_t *ev)
{
    dsme::DSMEPlatform::instance->sendNow();
}

static void _acktimer_cb(void *arg)
{
    dsme::DSMEPlatform::instance->offloadACKTimer();
}

static void _timer_ev_handler(event_t *ev)
{
    dsme::DSMEPlatform::instance->getDSME().getEventDispatcher().timerInterrupt();
}

static void _tx_done_handler(event_t *ev)
{
    dsme::DSMEPlatform::instance->processTXDoneEvent();
}

static void _rx_done_handler(event_t *ev)
{
    /* offloading done, remove busy flag */
    dsme::DSMEPlatform::instance->rxd_offload_pending = false;
    dsme::DSMEPlatform::instance->processRxDone();
}

static void _hal_radio_cb(ieee802154_dev_t *dev, ieee802154_trx_ev_t status)
{
    switch (status) {
    case IEEE802154_RADIO_CONFIRM_TX_DONE:
        dsme::DSMEPlatform::instance->offloadTXDoneEvent();
        break;
    case IEEE802154_RADIO_INDICATION_RX_START:
        dsme::DSMEPlatform::instance->indicateRxStart();
        break;
    case IEEE802154_RADIO_INDICATION_CRC_ERROR:
        if(dsme::DSMEPlatform::instance->getDSME().getMAC_PIB().macIsPANCoord) {
            DBG_PIN_CLEAR(LA_PIN_COORD_SET_RX_RXD);
            DBG_PIN_SET(LA_PIN_COORD_SET_RX_RXD);
        } else {
            DBG_PIN_CLEAR(LA_PIN_RFD_SET_RX_RXD);
            DBG_PIN_SET(LA_PIN_RFD_SET_RX_RXD);
        }
        break;
    case IEEE802154_RADIO_INDICATION_TX_START:
        break;
    case IEEE802154_RADIO_INDICATION_RX_DONE:
        dsme::DSMEPlatform::instance->offloadRXDoneEvent();
        break;
    case IEEE802154_RADIO_CONFIRM_CCA:
        dsme::DSMEPlatform::instance->offloadCCAEvent();
        break;
    default:
        DSME_ASSERT(false);
    }
}

/********************* C++ functions *********************/

void DSMEPlatform::processRxOffload()
{
    IDSMEMessage *message = this->message;

    this->message = nullptr;
    receiveFromAckLayerDelegate(message);
}

void DSMEPlatform::processCCAEvent()
{
    bool clear = ieee802154_radio_confirm_cca(this->radio);

    this->setPlatformState(DSMEPlatform::STATE_READY);
    this->getDSME().dispatchCCAResult(clear);
}

void DSMEPlatform::processTXDoneEvent()
{
    int res;
    int cnt = 0;
    /* TODO: remove the loop and assert again, once the fix for the wrong
     * radio state transition is resolved/confirmed */
    do {
        res = ieee802154_radio_confirm_transmit(this->radio, NULL);
        if (res == -EAGAIN) {
            puts("RBS");
        }
        cnt++;
    } while (res == -EAGAIN);

    if (cnt > 1) {
        printf("EA %d\n", cnt);
    }
    this->pending_tx = false;
    DSME_ASSERT(res >= 0);

    if (this->state != DSMEPlatform::STATE_TX_ACK) {
        if (this->wait_for_ack) {
            this->wait_for_ack = false;
            ieee802154_radio_set_frame_filter_mode(this->radio, IEEE802154_FILTER_ACK_ONLY);
        }
        else {
            ieee802154_radio_set_frame_filter_mode(this->radio, IEEE802154_FILTER_ACCEPT);
        }
    }

    this->frame_preloaded = false;
    this->setPlatformState(DSMEPlatform::STATE_READY);
    this->txEndCallback(true);
}

void DSMEPlatform::processRxDone()
{
    /* From the standard '6.2.5.1 CSMA-CA algorithm':
     * "Although the receiver of the device is enabled during the CCA analysis portion of this algorithm,
     * the device may discard any frames received during this time." */
    if (this->state == STATE_CCA_WAIT) {
        return;
    }
    if (this->state != STATE_READY) {
        /* in case the radio is currently doing something other than waiting for an RX,
         * ignore this event.
         * This may be triggered if the code path to send a frame does the following:
         * prepare_next_tx() {
         *   // <- RX_DONE Interrupt happens here and gets offloaded
         *   prepareSendingCopy();
         *   copy_frame_to_radio();
         * }
         * */
        //TODO: check if the assert can be enabled again. The aboce CCA case should be the only
        //      remaining case werhe this is expected
        return;
        assert(false);
    }

    DSMEMessage *message = getEmptyMessage();

    message->netif = this->netif;
    message->setStartOfFrameDelimiterSymbolCounter(rx_sfd);

    int res;

    res = ieee802154_radio_set_idle(this->radio, true);
    DSME_ASSERT(res == 0);
    int len = ieee802154_radio_len(this->radio);

    if (len > 127 || len < 0) {
        ieee802154_radio_read(this->radio, NULL, 127, NULL);
        res = ieee802154_radio_set_rx(this->radio);
        DSME_ASSERT(res == 0);
        return;
    }
    res = message->loadBuffer(len);
    DSME_ASSERT(res >= 0);
    ieee802154_rx_info_t info;

    res = ieee802154_radio_read(this->radio, message->getPayload(), 127, &info);
    if (res < 0) {
        message->releaseMessage();
        res = ieee802154_radio_set_rx(this->radio);
        DSME_ASSERT(res == 0);
        return;
    }

    message->messageLQI = info.lqi;
    message->messageRSSI = info.rssi;
    const uint8_t *buf = message->getPayload();

    bool success = message->getHeader().deserializeFrom(buf, len);

    if (!success) {
        message->releaseMessage();
        //TODO: enable again?
        // This should not interfere with the explicit radio control
        // in upper layers, as the state without an indication is still expected
        // to be RX.
        //res = ieee802154_radio_set_rx(this->radio);
        DSME_ASSERT(res == 0);
        return;
    }

    message->dropHdr(message->getHeader().getSerializationLength());

    getDSME().getAckLayer().receive(message);
}

void DSMEPlatform::offloadCCAEvent()
{
    event_post(this->getEventQueue(), &this->cca_ev);
}

void DSMEPlatform::offloadTXDoneEvent()
{
    if(this->dsme.getMAC_PIB().macIsPANCoord) {
        DBG_PIN_CLEAR(LA_PIN_COORD_TXNOW_TXD);
    } else {
        DBG_PIN_CLEAR(LA_PIN_RFD_TXNOW_TXD);
    }

    if (this->state == DSMEPlatform::STATE_TX_ACK) {
        mutex_unlock(&this->sda_lock);
    }
    event_post(this->getEventQueue(), &this->tx_done_event);
}

void DSMEPlatform::indicateRxStart()
{
    //TODO: add calibration routine to determine the correct offset
    this->rx_sfd = this->getSymbolCounter() - 11;
}

void DSMEPlatform::offloadRXDoneEvent()
{
    if(this->dsme.getMAC_PIB().macIsPANCoord) {
        DBG_PIN_CLEAR(LA_PIN_COORD_SET_RX_RXD);
    } else {
        DBG_PIN_CLEAR(LA_PIN_RFD_SET_RX_RXD);
    }
    this->rxd_offload_pending = true;
    event_post(this->getEventQueue(), &this->rx_done_event);
}

void DSMEPlatform::offloadTimerEvent()
{
    event_post(this->getEventQueue(), &this->timer_event);
}

void DSMEPlatform::offloadACKTimer()
{
    event_post(this->getEventQueue(), &this->acktimer_ev);
}

static void _timer_cb(void *arg)
{
    dsme::DSMEPlatform::instance->offloadTimerEvent();
}

void DSMEPlatform::sendFrame(uint16_t addr, iolist_t *pkt)
{
    /* First 2 bytes are the ID */
    if (!this->mac_pib.macAssociatedPANCoord) {
        return;
    }

    DSMEMessage *message = getEmptyMessage();

    if (message->loadBuffer(pkt) < 0) {
        return;
    }

    IEEE802154MacAddress dst;

    dst.setShortAddress(addr);
    mcps_sap::DATA::request_parameters params;

    message->getHeader().setSrcAddrMode(SHORT_ADDRESS);
    message->getHeader().setDstAddrMode(SHORT_ADDRESS);
    message->getHeader().setDstAddr(dst);

    message->getHeader().setSrcPANId(this->mac_pib.macPANId);
    message->getHeader().setDstPANId(this->mac_pib.macPANId);

    this->dsmeAdaptionLayer.sendMessage(message);
}

DSMEPlatform::DSMEPlatform() :
    phy_pib(),
    mac_pib(phy_pib),

    mcps_sap(dsme),
    mlme_sap(dsme),
    dsmeAdaptionLayer(dsme),
    initialized(false),
    state(STATE_READY)
{
    instance = this;
    this->timer.callback = _timer_cb;
    this->timer.arg = this;

    this->acktimer.callback = _acktimer_cb;
    this->acktimer.arg = this;
    this->acktimer_ev.handler = _acktimer_ev_handler;
    this->cca_ev.handler = _cca_ev_handler;
    this->timer_event.handler = _timer_ev_handler;
    this->tx_done_event.handler = _tx_done_handler;
    this->rx_done_event.handler = _rx_done_handler;
    this->rx_offload_ev.handler = _handle_rx_offload;
    this->start_of_cfp_ev.handler = _start_of_cfp_handler;
    mutex_init(&this->sda_lock);
    mutex_lock(&this->sda_lock);
}

DSMEPlatform::~DSMEPlatform()
{}

/**
 * Creates an IEEE802154MacAddress out of an uint16_t short address
 */
void DSMEPlatform::translateMacAddress(uint16_t& from, IEEE802154MacAddress& to)
{
    if (from == 0xFFFF) {
        to = IEEE802154MacAddress(IEEE802154MacAddress::SHORT_BROADCAST_ADDRESS);
    }
    else {
        to.setShortAddress(from);
    }
}

void DSMEPlatform::initialize(bool pan_coord, uint8_t sfo, uint8_t msfo, uint8_t bo)
{
    this->instance = this;
    this->dsme.setPHY_PIB(&(this->phy_pib));
    this->dsme.setMAC_PIB(&(this->mac_pib));
    this->dsme.setMCPS(&(this->mcps_sap));
    this->dsme.setMLME(&(this->mlme_sap));

    /* init wrapping counter base value */
    _hw_counter_base = _hw_counter_read();

    /* Use all channels of the channel page 0 (O-QPSK) */
    constexpr uint8_t MAX_CHANNELS = 16;
    uint8_t channels[MAX_CHANNELS];

    uint8_t num = MAX_CHANNELS;

    channelList_t DSSS2450_channels(num);

    for (uint8_t i = 0; i < num; i++) {
        DSSS2450_channels[i] = 11 + i;
    }

    phy_pib.setDSSS2450ChannelPage(DSSS2450_channels);

    /* Initialize Address */
    IEEE802154MacAddress address;

    uint8_t ext_addr[IEEE802154_LONG_ADDRESS_LEN];
    network_uint16_t short_addr;

    luid_base(ext_addr, sizeof(ext_addr));
    address.setA1((ext_addr[0] << 8) | ext_addr[1]);
    address.setA2((ext_addr[2] << 8) | ext_addr[3]);
    address.setA3((ext_addr[4] << 8) | ext_addr[5]);
    address.setA4((ext_addr[6] << 8) | ext_addr[7]);
    this->mac_pib.macExtendedAddress = address;

    /* TODO: UGLY HACK! To be removed when gnrc_netif<->netdev dependency is
     * not granted */
    this->radio = (ieee802154_dev_t *)this->netif->dev;

    this->radio->cb = _hal_radio_cb;

    ieee802154_radio_request_on(this->radio);
    while (ieee802154_radio_confirm_on(this->radio) == -EAGAIN) {}

    this->radio_on = true;

    /* Disable Auto CSMA-CA */
    ieee802154_radio_set_csma_params(this->radio, NULL, -1);

    /* Accept all frames except ACK */
    ieee802154_radio_set_frame_filter_mode(this->radio, IEEE802154_FILTER_ACCEPT);

    /* Call more radio configurations here if needed... */

    this->mac_pib.macShortAddress = this->mac_pib.macExtendedAddress.getShortAddress();

    short_addr.u8[0] = this->mac_pib.macExtendedAddress.getShortAddress() >> 8;
    short_addr.u8[1] = this->mac_pib.macExtendedAddress.getShortAddress() & 0xFF;

    this->mac_pib.macIsPANCoord = pan_coord;
    this->mac_pib.macIsCoord = pan_coord;
    if (this->mac_pib.macIsPANCoord) {
        DEBUG("This node is PAN coordinator\n");
        this->mac_pib.macPANId = CONFIG_IEEE802154_DEFAULT_PANID;
    }
    ieee802154_radio_config_addr_filter(this->radio, IEEE802154_AF_PANID, &this->mac_pib.macPANId);
    ieee802154_radio_config_addr_filter(this->radio, IEEE802154_AF_SHORT_ADDR, &short_addr);
    ieee802154_radio_config_addr_filter(this->radio, IEEE802154_AF_EXT_ADDR, &ext_addr);

    this->mac_pib.macCapReduction = CONFIG_IEEE802154_DSME_CAP_REDUCTION;

    this->mac_pib.macAssociatedPANCoord = this->mac_pib.macIsPANCoord;

    this->mac_pib.macSuperframeOrder = sfo;
    this->mac_pib.macMultiSuperframeOrder = msfo;
    this->mac_pib.macBeaconOrder = bo;

    this->mac_pib.macMinBE = CONFIG_IEEE802154_DEFAULT_CSMA_CA_MIN_BE;
    this->mac_pib.macMaxBE = CONFIG_IEEE802154_DEFAULT_CSMA_CA_MAX_BE;
    this->mac_pib.macMaxCSMABackoffs = CONFIG_IEEE802154_DEFAULT_CSMA_CA_RETRIES;
    this->mac_pib.macMaxFrameRetries = CONFIG_IEEE802154_DEFAULT_MAX_FRAME_RETRANS;

    this->mac_pib.macDSMEGTSExpirationTime = CONFIG_IEEE802154_DSME_GTS_EXPIRATION;

    /* hacky way to let the coord wait a bit longer than an RFD before triggering the expiration.
     * This is dine to prevent triggering at exactly the same time which gives higher likelyhood
     * of collisions during CAP. */
    if (pan_coord) {
        this->mac_pib.macDSMEGTSExpirationTime = CONFIG_IEEE802154_DSME_GTS_EXPIRATION
                                               + CONFIG_IEEE802154_DSME_GTS_EXPIRATION / 10;
    }

    this->mac_pib.macResponseWaitTime = CONFIG_IEEE802154_DSME_MAC_RESPONSE_WAIT_TIME;
    this->mac_pib.macChannelDiversityMode = Channel_Diversity_Mode::CHANNEL_HOPPING;

    this->phy_pib.phyCurrentChannel = CONFIG_IEEE802154_DEFAULT_CHANNEL;

    this->dsmeAdaptionLayer.setIndicationCallback(DELEGATE(&DSMEPlatform::
                                                           handleDataMessageFromMCPSWrapper,
                                                           *this));
    this->dsmeAdaptionLayer.setConfirmCallback(DELEGATE(&DSMEPlatform::handleConfirmFromMCPSWrapper,
                                                        *this));

    this->dsme.initialize(this);

    channelList_t scanChannels;

    scanChannels.add(CONFIG_IEEE802154_DEFAULT_CHANNEL);
    if (IS_ACTIVE(CONFIG_IEEE802154_DSME_STATIC_GTS)) {
        StaticScheduling *staticScheduling = new StaticScheduling(this->dsmeAdaptionLayer);
        staticScheduling->setNegotiateChannels(false);
        scheduling = staticScheduling;
    }
    else {
        TPS *tps = new TPS(this->dsmeAdaptionLayer);
        tps->setAlpha(OPENDSME_TPS_ALPHA);
        tps->setMinFreshness(this->mac_pib.macDSMEGTSExpirationTime);
        scheduling = tps;
    }

    this->dsmeAdaptionLayer.initialize(scanChannels, CONFIG_IEEE802154_DSME_SCAN_DURATION,
                                       scheduling);
    this->initialized = true;

}

void DSMEPlatform::initialize(bool pan_coord)
{
    initialize(pan_coord, CONFIG_IEEE802154_DSME_SUPERFRAME_ORDER,
                          CONFIG_IEEE802154_DSME_MULTISUPERFRAME_ORDER,
                          CONFIG_IEEE802154_DSME_BEACON_ORDER);
}

#if IS_ACTIVE(CONFIG_IEEE802154_DSME_STATIC_GTS)
void DSMEPlatform::allocateGTS(uint8_t superframeID, uint8_t slotID, uint8_t channelID,
                               Direction direction, uint16_t address)
{
    static_cast<StaticScheduling *>(scheduling)->allocateGTS(superframeID, slotID, channelID,
                                                             direction, address);
}
#endif

void DSMEPlatform::setGTSTransmission(bool gts)
{
    this->dsmeAdaptionLayer.getMessageHelper().setGTSTransmission(gts);
}

void DSMEPlatform::setAckReq(bool ackReq)
{
    this->dsmeAdaptionLayer.getMessageHelper().setAckReq(ackReq);
}

void DSMEPlatform::start()
{
    DSME_ASSERT(this->initialized);
    this->dsme.start();
    this->dsmeAdaptionLayer.startAssociation();
}

void DSMEPlatform::getShortAddress(network_uint16_t *addr)
{
    addr->u8[0] = this->mac_pib.macExtendedAddress.getShortAddress() >> 8;
    addr->u8[1] = this->mac_pib.macExtendedAddress.getShortAddress() & 0xFF;
}

bool DSMEPlatform::isAssociated()
{
    return this->mac_pib.macAssociatedPANCoord;
}

void DSMEPlatform::handleDataMessageFromMCPSWrapper(IDSMEMessage *msg)
{
    this->handleDataMessageFromMCPS(static_cast<DSMEMessage *>(msg));
}

void DSMEPlatform::handleConfirmFromMCPSWrapper(IDSMEMessage *msg,
                                                DataStatus::Data_Status dataStatus)
{
    this->handleConfirmFromMCPS(static_cast<DSMEMessage *>(msg), dataStatus);
}

void DSMEPlatform::handleConfirmFromMCPS(DSMEMessage *msg, DataStatus::Data_Status dataStatus)
{
    if (dataStatus == DataStatus::Data_Status::SUCCESS) {
        /* TODO: Add to statistics */
    }

    IDSMEMessage *m = static_cast<IDSMEMessage*>(msg);
    releaseMessage(m);
}

void DSMEPlatform::handleDataMessageFromMCPS(DSMEMessage *msg)
{
    msg->dispatchMessage();
}

bool DSMEPlatform::isReceptionFromAckLayerPossible()
{
    return this->state == STATE_READY;
}

void DSMEPlatform::handleReceivedMessageFromAckLayer(IDSMEMessage *message)
{
    DSME_ASSERT(receiveFromAckLayerDelegate);
    DSME_ASSERT(!this->message);
    this->message = message;
    event_post(this->getEventQueue(), &rx_offload_ev);
}

DSMEMessage *DSMEPlatform::getEmptyMessage()
{
    DSMEMessage *msg = new DSMEMessage();

    DSME_ASSERT(msg);
    msg->clearMessage();
    signalNewMsg(msg);
    return msg;
}

void DSMEPlatform::signalNewMsg(DSMEMessage *msg)
{
    /* Not used */
}

void DSMEPlatform::releaseMessage(IDSMEMessage *msg)
{
    DSMEMessage *m = static_cast<DSMEMessage *>(msg);

    m->releaseMessage();
}

uint32_t _hw_counter_mask(uint32_t val) {
    return val & HW_COUNTER_MAX;
}

uint32_t _hw_counter_read(void) {
    return ztimer_now(ZTIMER_INSTANCE);
}

/* This behaves like a continuously running 32-bit wraparound counter with
 * one symbol (16 µs) resolution.  Instead of a timer backend that actually
 * runs on 1/16 MHz it uses a slower low power timer.
 * Returned values are therefore subject to quantization. Even though the
 * required resolution of 16 µs can not be guaranteed with a typical 32 kHz
 * timer, the jitter related to that is still well within an acceptable range.
 * NOTE: this function *MUST* be called at least once within each period of
 * the below timer to guarantee correct behavior. For normal DSME configurations
 * this is guaranteed by slot / (multi-)superframe events. For very slow DSME
 * configurations, as well as for very fast or very short timers this may need
 * explicit handling (e.g., performing additional reads between events). */
uint32_t symbol_counter_read32(void) {
    uint32_t timer_cnt = _hw_counter_read();

    /* use the difference since the last wrap around of the arbitrary frequency timer as base counter value */
    uint32_t offset_timer_cnt = timer_cnt - _hw_counter_base;

    /* rescale to symbol counter frequency */
    uint64_t scaled = ((uint64_t)offset_timer_cnt * SYMBOL_TIMER_FREQUENCY + (HWTIMER_FREQUENCY >> 1))
                      / HWTIMER_FREQUENCY + _rescaled_symbol_counter_offs;
    uint32_t res;

    if (scaled > UINT32_MAX) {
        /* how much the counter overflowed into the next cycle. */
        uint64_t diff = scaled - UINT32_MAX;

        /* save the hardware counter as new base value.
         * The base value is later used to calculate how many ticks the
         * hw counter advanced. since the last marker. */
        _hw_counter_base = timer_cnt;

        /* use 32 bit arithmetic to wraparound the symbol counter */
        res = (scaled & 0xFFFFFFFF);
        res += diff;
        /* offset that will be applied to wraparound the 32 bit value */
        _rescaled_symbol_counter_offs = diff;
    } else {
        res = (uint32_t)(scaled & 0xFFFFFFFF);
    }
    return res;
}

/* Returns 64 bit timestamps as a continuous counter to extend shorter timer.
 * Must be called at least once in each period of lower layer timer.
 * Currently only used for testing where long absolute timestamps are helpful. */
uint64_t _counter_read_hw_ticks_64(void) {
    uint32_t timer_cnt = _hw_counter_read();
    uint32_t ll_base = (extended64_bit_timer_base & HW_COUNTER_MAX);

    if (ll_base > timer_cnt) {
        uint32_t elapsed = _hw_counter_mask(timer_cnt - _hw_counter_mask((uint32_t)extended64_bit_timer_base));
        extended64_bit_timer_base += elapsed;
    } else {
        extended64_bit_timer_base = (extended64_bit_timer_base - ll_base) + timer_cnt;
    }
    return extended64_bit_timer_base;
}

static uint32_t _symbols_to_hw_ticks(uint32_t symbols)
{
    return (uint32_t)((uint64_t)symbols * HWTIMER_FREQUENCY / SYMBOL_TIMER_FREQUENCY);
}

static uint32_t _usecs_to_lptticks(uint32_t usecs)
{
    return (uint32_t)((uint64_t)usecs * HWTIMER_FREQUENCY / 1000000);
}

void DSMEPlatform::startTimer(uint32_t symbolCounterValue)
{
    uint32_t now = symbol_counter_read32();

    int32_t delta = symbolCounterValue - now;
    /* scheduling a timer in the past is not possible, so ensure the minimum delay is 0 */
    if (delta < 0) {
        delta = 0;
    }

    uint32_t hw_delta = _symbols_to_hw_ticks(delta);

    /* compensate for offloading overhead by setting smaller delay */
    if (hw_delta > DSME_LOW_POWER_TIMER_COMPENSATION_TICKS) {
        hw_delta -= DSME_LOW_POWER_TIMER_COMPENSATION_TICKS;
    } else {
        hw_delta = 0;
    }

    /* TODO: when using a low speed timer, additionally using the USEC instance for
     *       very short timeouts or even the otherwise rounded remainder of longer
     *       timeouts could improve accuracy.
     *       This is especially relevant in case the minimum delay of the low-power
     *       timer is relatively long as that would artificially increase the delay
     *       of very short timeouts.
     */
    ztimer_set(ZTIMER_INSTANCE, &timer, hw_delta);
}

uint32_t DSMEPlatform::getSymbolCounter()
{
    return symbol_counter_read32();
}

void DSMEPlatform::scheduleStartOfCFP()
{
    event_post(this->getEventQueue(), &start_of_cfp_ev);
}

void DSMEPlatform::signalAckedTransmissionResult(bool success, uint8_t transmissionAttempts,
                                                 IEEE802154MacAddress receiver)
{
    /* Not used */
}

/*
 * Signal GTS allocation or deallocation
 */
void DSMEPlatform::signalGTSChange(bool deallocation, IEEE802154MacAddress counterpart,
                                   uint16_t superframeID, uint8_t gtSlotID, uint8_t channel,
                                   Direction direction)
{}

void DSMEPlatform::signalQueueLength(uint32_t length)
{}

/*
 * Number of packets sent per CAP
 */
void DSMEPlatform::signalPacketsPerCAP(uint32_t packets)
{}

/*
 * Number of failed packets per CAP
 */
void DSMEPlatform::signalFailedPacketsPerCAP(uint32_t packets)
{
    /* Not used */
}

void DSMEPlatform::updateVisual()
{
    /* Not used */
}

bool DSMEPlatform::setChannelNumber(uint8_t channel)
{
    ieee802154_phy_conf_t conf = {
        .phy_mode = IEEE802154_PHY_OQPSK,
        .channel = channel,
        .page = 0,
        .pow = CONFIG_IEEE802154_DEFAULT_TXPOWER,
    };
    int res;

    res = ieee802154_radio_config_phy(this->radio, &conf);
    DSME_ASSERT(res == 0);

    /* TODO: Find a better solution */
    ieee802154_radio_config_addr_filter(this->radio, IEEE802154_AF_PANID, &this->mac_pib.macPANId);

    return true;
}

uint8_t DSMEPlatform::getChannelNumber()
{
    /* Apparently not used by OpenDSME */
    DSME_ASSERT(false);
    return 0;
}

bool DSMEPlatform::prepareSendingCopy(IDSMEMessage *msg, Delegate<void(bool)> txEndCallback)
{
    if(this->rxd_offload_pending) {
        this->rxd_offload_pending = false;
        event_cancel(this->getEventQueue(), &this->rx_done_event);
        puts("busy");
        return false;
    }

    this->frame_preloaded = true;

    if(this->dsme.getMAC_PIB().macIsPANCoord) {
        DBG_PIN_SET(LA_PIN_COORD_TXNOW_TXD);
        DBG_PIN_CLEAR(LA_PIN_COORD_TXNOW_TXD);
    } else {
        DBG_PIN_SET(LA_PIN_RFD_TXNOW_TXD);
        DBG_PIN_CLEAR(LA_PIN_RFD_TXNOW_TXD);
    }

    DSMEMessage *m = (DSMEMessage *)msg;

    this->state = DSMEPlatform::STATE_SEND;
    this->txEndCallback = txEndCallback;
    uint8_t mhr[IEEE802154_MAX_HDR_LEN];
    uint8_t mhr_len = msg->getHeader().getSerializationLength();
    uint8_t *p = mhr;

    msg->getHeader().serializeTo(p);
    iolist_t iol = {
        .iol_next = (iolist_t *)m->getIolPayload(),
        .iol_base = mhr,
        .iol_len = mhr_len,
    };

    if (mhr[0] & IEEE802154_FCF_ACK_REQ) {
        this->wait_for_ack = true;
    }
    else {
        this->wait_for_ack = false;
    }

    int res = ieee802154_radio_write(this->radio, &iol);
    DSME_ASSERT(res == 0);

    return true;
}

bool DSMEPlatform::sendNow()
{
    if(this->dsme.getMAC_PIB().macIsPANCoord) {
        DBG_PIN_SET(LA_PIN_COORD_TXNOW_TXD);
    } else {
        DBG_PIN_SET(LA_PIN_RFD_TXNOW_TXD);
    }

    int res = ieee802154_radio_request_transmit(this->radio);
    DSME_ASSERT(res == 0);
    this->pending_tx = true;
    if(this->dsme.getMAC_PIB().macIsPANCoord) {
        DBG_PIN_CLEAR(LA_PIN_COORD_TXNOW_TXD);
        DBG_PIN_SET(LA_PIN_COORD_TXNOW_TXD);
    } else {
        DBG_PIN_CLEAR(LA_PIN_RFD_TXNOW_TXD);
        DBG_PIN_SET(LA_PIN_RFD_TXNOW_TXD);
    }
    return true;
}

void DSMEPlatform::abortPreparedTransmission()
{
    puts("A");

    /* clear the preloaded state here as the transmission was explicitly aborted */
    this->frame_preloaded = false;
    /* Nothing to do here, since the Radio HAL will drop the frame if
     * the write function is called again */
    this->setPlatformState(DSMEPlatform::STATE_READY);
}

bool DSMEPlatform::sendDelayedAck(IDSMEMessage *ackMsg, IDSMEMessage *receivedMsg,
                                  Delegate<void(bool)> txEndCallback)
{
    /* if the radio received data, and already offloaded the event,
     * it is canceled here and the layer indicates busy to avoid inconsistent radio state.
     * TODO: check if this is still needed now that the radio properly separates TX/RX IRQs*/
    if(this->rxd_offload_pending) {
        this->rxd_offload_pending = false;
        event_cancel(this->getEventQueue(), &this->rx_done_event);
        puts("SDAbusy");
        return false;
    }

    this->state = DSMEPlatform::STATE_TX_ACK;
    DSMEMessage *m = (DSMEMessage *)ackMsg;

    this->frame_preloaded = true;
    DSME_ASSERT(m != nullptr);

    uint8_t ack[IEEE802154_ACK_FRAME_LEN - IEEE802154_FCS_LEN];
    uint8_t mhr_len = ackMsg->getHeader().getSerializationLength();

    DSME_ASSERT(mhr_len == sizeof(ack));

    uint8_t *p = ack;

    ackMsg->getHeader().serializeTo(p);

    this->txEndCallback = txEndCallback;

    iolist_t iol = {
        .iol_next = NULL,
        .iol_base = ack,
        .iol_len = mhr_len,
    };

    int res = ieee802154_radio_write(this->radio, &iol);
    DSME_ASSERT(res == 0);

    /* Hardcoded to O-QPSK
     * Preamble (4) | SFD (1) | PHY Hdr (1) | MAC Payload | FCS (2)
     */
    uint32_t endOfReception = receivedMsg->getStartOfFrameDelimiterSymbolCounter()
                              + receivedMsg->getTotalSymbols()
                              - 2 * 4   /* Preamble */
                              - 2 * 1;  /* SFD */
    uint32_t ackTime = endOfReception + aTurnaroundTime;
    uint32_t now = getSymbolCounter();
    int32_t diff = ackTime - now;

    /* blocking wait for remaining time before sending the ACK.
     * This must be done because upper layers expect the ACK transmission to be finished
     * when this function returns. */
    if (diff > 0) {
        ztimer_sleep(ZTIMER_INSTANCE, _symbols_to_hw_ticks(diff));
    }

    this->sendNow();

    mutex_lock(&this->sda_lock);

    return true;
}

void DSMEPlatform::setReceiveDelegate(receive_delegate_t receiveDelegate)
{
    this->receiveFromAckLayerDelegate = receiveDelegate;
}

bool DSMEPlatform::startCCA()
{
    if (this->pending_tx) {
        puts("*");
        return false;
    }
    if(this->dsme.getMAC_PIB().macIsPANCoord) {
        DBG_PIN_SET(LA_PIN_COORD_CCA);
        DBG_PIN_CLEAR(LA_PIN_COORD_SET_RX_RXD);
    } else {
        DBG_PIN_SET(LA_PIN_RFD_CCA);
        DBG_PIN_CLEAR(LA_PIN_RFD_SET_RX_RXD);
    }
    ieee802154_radio_request_cca(this->radio);
    if(this->dsme.getMAC_PIB().macIsPANCoord) {
        DBG_PIN_CLEAR(LA_PIN_COORD_CCA);
        DBG_PIN_SET(LA_PIN_COORD_CCA);
    } else {
        DBG_PIN_CLEAR(LA_PIN_RFD_CCA);
        DBG_PIN_SET(LA_PIN_RFD_CCA);
    }
    this->state = DSMEPlatform::STATE_CCA_WAIT;
    return true;
}

void DSMEPlatform::turnTransceiverOn()
{
    if (!this->radio_on) {
        if(this->dsme.getMAC_PIB().macIsPANCoord) {
            DBG_PIN_SET(LA_PIN_COORD_ON_IDLE);
        } else {
            DBG_PIN_SET(LA_PIN_RFD_ON_IDLE);
        }
        int res = ieee802154_radio_request_on(this->radio);

        DSME_ASSERT(res == 0);

        do {
            res = ieee802154_radio_confirm_on(this->radio);
        } while (res == -EAGAIN);

        DSME_ASSERT(res == 0);
        ieee802154_radio_set_cca_threshold(this->radio, CONFIG_IEEE802154_CCA_THRESH_DEFAULT);
        this->radio_on = true;
    }
}
void DSMEPlatform::turnTransceiverToIdle()
{
    if(this->dsme.getMAC_PIB().macIsPANCoord) {
        DBG_PIN_CLEAR(LA_PIN_COORD_SET_RX_RXD);
        DBG_PIN_CLEAR(LA_PIN_COORD_TXNOW_TXD);
    } else {
        DBG_PIN_CLEAR(LA_PIN_RFD_SET_RX_RXD);
        DBG_PIN_CLEAR(LA_PIN_RFD_TXNOW_TXD);
    }
    int res = ieee802154_radio_set_idle(this->radio, true);
    DSME_ASSERT(res == 0);
}

void DSMEPlatform::turnTransceiverToRX()
{
    if(this->dsme.getMAC_PIB().macIsPANCoord) {
        DBG_PIN_SET(LA_PIN_COORD_SET_RX_RXD);
    } else {
        DBG_PIN_SET(LA_PIN_RFD_SET_RX_RXD);
    }
    int res = ieee802154_radio_set_rx(this->radio);
    DSME_ASSERT(res == 0);
}

void DSMEPlatform::turnTransceiverOff()
{
    if (this->radio_on) {
        /* the transceiver shall not be turned off before
         * the preloaded frame is explicitly discarded by aborting the prepared
         * transmission. */
        assert(!this->frame_preloaded);
        if(this->dsme.getMAC_PIB().macIsPANCoord) {
            DBG_PIN_CLEAR(LA_PIN_COORD_SET_RX_RXD);
            DBG_PIN_CLEAR(LA_PIN_COORD_ON_IDLE);
            DBG_PIN_CLEAR(LA_PIN_COORD_TXNOW_TXD);
        } else {
            DBG_PIN_CLEAR(LA_PIN_RFD_SET_RX_RXD);
            DBG_PIN_CLEAR(LA_PIN_RFD_ON_IDLE);
            DBG_PIN_CLEAR(LA_PIN_RFD_TXNOW_TXD);
        }
        int res = ieee802154_radio_off(this->radio);

        DSME_ASSERT(res == 0);
        this->radio_on = false;
    }
}

bool DSMEPlatform::isRxEnabledOnCap()
{
    /* TODO: This feature is experimental. Enable RX on CAP for now... */
    return true;
}

}

/** @} */
