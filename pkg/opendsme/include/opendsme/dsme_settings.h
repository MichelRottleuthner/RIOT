/*
 * Copyright (C) 2022 HAW Hamburg
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     pkg_opendsme
 *
 * @{
 *
 * @file
 *
 * @author      José I. Álamos <jose.alamos@haw-hamburg.de>
 */
#ifndef OPENDSME_DSME_SETTINGS_H
#define OPENDSME_DSME_SETTINGS_H

#include <stdint.h>

#include <stdio.h>
#include "kernel_defines.h"
#include "net/ieee802154.h"
#include "opendsme.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief maximum number of lost beacons before leaving the network
 */
#define DSME_MAX_LOST_BEACONS CONFIG_OPENDSME_MAX_LOST_BEACONS

/**
 * @brief controls whether also irrelevant slot events are scheduled.
 *
 * This controls a custom extension that skips slot events if
 * no GTS is allocated to reduce system wakeups compared to the
 * default (strictly tick based) implementation.
 */
#ifndef DSME_MINIMIZE_CFP_SLOT_EVENTS
#define DSME_MINIMIZE_CFP_SLOT_EVENTS (0)
#endif

/**
 * @brief Set to 1 to use a low power compatible timer for DSME.
 *
 * This is EXPERIMENTAL and WILL not work as expected on all platforms.
 * Currently It assumes that ZTIMER_MSEC_BASE (the timer backend that usually
 * feeds ZTIMER_MSEC) is available (explicitly pulled in) and configured to
 * use the RTT at a fast enough frequency of at least 32kHz.
 * If this is the case you may be lucky and it might work as is.
 * Using ZTIMER_MSEC is *way* to coarse for DSME timing, and ZTIMER_USEC
 * prevents low-power modes.
 * As some backends have pretty high overhead for accessing the timer hardware
 * you may need to add compensation for the slow timer set operation
 * in DSME code and the PRE_EVENT_SHIFT value in below DSME settings.
 */
#ifndef DSME_USE_LOW_POWER_TIMER
#define DSME_USE_LOW_POWER_TIMER (0)
#endif

/**
 * @brief Compensation value for the low power timer.
 *
 * All timer values to be set smaller than this will map to zero.
 * Greater timer values will be reduced by this number.
 */
#ifndef DSME_LOW_POWER_TIMER_COMPENSATION_TICKS
#define DSME_LOW_POWER_TIMER_COMPENSATION_TICKS (3)
#endif

namespace dsme {

namespace const_redefines {
/**
 * @brief fixed value, see Table 8-80 of IEEE 802.15.4-2015, this includes beacon, CAP and GTS
*/
constexpr uint8_t aNumSuperframeSlots = 16;
/**
 * @brief fixed value, see 8.1.3 of IEEE 802.15.4-2011 (assuming no UWB PHY)
 */
constexpr uint8_t macSIFSPeriod = 12;

/**
 * @brief fixed value, see 8.1.3 of IEEE 802.15.4-2011 (assuming no UWB PHY)
 */
constexpr uint8_t macLIFSPeriod = 40;
}

/**
 * @brief guard time before a DSME event
 */
constexpr uint8_t PRE_EVENT_SHIFT = const_redefines::macLIFSPeriod;

/**
 * @brief Minimum number of CSMA slots
 */
constexpr uint8_t MIN_CSMA_SLOTS = 0;

/**
 * @brief maximum number of GTS slots
 */
constexpr uint8_t MAX_GTSLOTS = const_redefines::aNumSuperframeSlots - MIN_CSMA_SLOTS - 1;

/**
 * @brief maximum number of IEEE 802.15.4 channels
 */
constexpr uint8_t MAX_CHANNELS = 16; /* For O-QPSK */

/**
 * @brief minimum number of IEEE 802.15.4 channels
 */
constexpr uint8_t MIN_CHANNEL = 11;  /* For O-QPSK */

/**
 * @brief default MAC channel
 */
constexpr uint8_t MAC_DEFAULT_CHANNEL = CONFIG_IEEE802154_DEFAULT_CHANNEL; /* For O-QPSK */

/**
 * @brief maximum number of neighbors
 */
constexpr uint8_t MAX_NEIGHBORS = CONFIG_OPENDSME_MAX_NEIGHBOURS;

/**
 * @brief default PAN ID
 */
constexpr uint16_t MAC_DEFAULT_NWK_ID = CONFIG_IEEE802154_DEFAULT_PANID;

/**
 * @brief Minimum superframe order
 */
constexpr uint8_t MIN_SO = CONFIG_IEEE802154_DSME_SUPERFRAME_ORDER;

/**
 * @brief Maximum beacon order
 */
constexpr uint8_t MAX_BO = CONFIG_IEEE802154_DSME_BEACON_ORDER;

/**
 * @brief Maximum multi-superframe order
 */
constexpr uint8_t MAX_MO = CONFIG_IEEE802154_DSME_MULTISUPERFRAME_ORDER;

/**
 * @brief Maximum number of slots per superframe
 */
constexpr uint16_t MAX_SLOTS_PER_SUPERFRAMES = 1 << (uint16_t)(MAX_BO - MIN_SO);

/**
 * @brief Maximum number of superframes per beacon interval
 */
constexpr uint16_t MAX_TOTAL_SUPERFRAMES = 1 << (uint16_t)(MAX_BO - MIN_SO);

/**
 * @brief Maximum number of superframes per multi-superframe
 */
constexpr uint16_t MAX_SUPERFRAMES_PER_MULTI_SUPERFRAME = 1 << (uint16_t)(MAX_MO - MIN_SO);

/**
 * @brief Maximum number of GTS slots per superframe
 */
constexpr uint16_t MAX_OCCUPIED_SLOTS = MAX_SUPERFRAMES_PER_MULTI_SUPERFRAME*MAX_GTSLOTS*MAX_CHANNELS;

/**
 * @brief Maximum number of SAB units
 */
constexpr uint8_t MAX_SAB_UNITS = 1;

/**
 * @brief Size of the CAP queue
 */
constexpr uint16_t CAP_QUEUE_SIZE = CONFIG_OPENDSME_CAP_QUEUE_SIZE;

/**
 * @brief Size of the CFP queue
 */
constexpr uint16_t TOTAL_GTS_QUEUE_SIZE = CONFIG_OPENDSME_CFP_QUEUE_SIZE;

/**
 * @brief Size of the CAP queue
 */
constexpr uint16_t UPPER_LAYER_QUEUE_SIZE = 4;

/**
 * @brief Broadcast PAN ID
 */
constexpr uint16_t DSME_BROADCAST_PAN_ID = 0xffff;

/**
 * @brief Additional time to wait for an ACK
 */
constexpr uint8_t ADDITIONAL_ACK_WAIT_DURATION = 63;
}

#ifdef __cplusplus
}
#endif

#endif /* OPENDSME_DSME_SETTINGS_H */
/** @} */
