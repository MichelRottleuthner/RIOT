/*
 * Copyright (C) 2022 HAW Hamburg
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    net_ieee802154_mac_header IEEE 802.15.4 MAC definitions
 * @ingroup     net_ieee802154_mac
 * @brief       IEEE 802.15.4 MAC types and interface
 * @{
 *
 * @file
 * @brief       IEEE 802.15.4 MAC interface
 * @note        This is WIP and used as a placeholder for missing MAC features
 *              of 802154 in gnrc netif.
 * @author      Michel Rottleuthner <michel.rottleuthner@haw-hamburg.de>
 */
#include "mutex.h"
#include "event/periodic.h"
#include "net/ieee802154.h"
#include "net/gnrc/pktqueue.h"
#include "net/gnrc/netif.h"
#include "net/netdev/ieee802154.h"
#include "net/ieee802154_security.h"

#ifndef NET_IEEE802154_MAC_H
#define NET_IEEE802154_MAC_H

/**
 * @brief IEEE802.15.4 MAC command definitions
 *
 * @{
 */
#define IEEE802154_MAC_CMD_DATA_REQUEST (0x04)
/** @} */

/* helper define to exclude features/fields that are not yet implemented
 * in order to still keep track of them. */
#define IEEE802154_MAC_TO_BE_IMPLEMENTED (0)

/* number of distinct queues (one queue per indirect tx destination) */
#define IEEE802154_MAC_IDTX_QUEUES_NUMOF (8)

/* number of max overall indirect transmission packets to be queued per interface.
 * Will be shared across all tx destination queues.  */
#define IEEE802154_MAC_IDTX_PKTS_NUMOF (16)

/* Timeout for an MLME-POLL. After a data pending indication (indicated by the
 * pending bit in the ACK responding to a data request MAC command), the radio
 * will continue to listen for this time. If no data is received the readio will
 * stop listening again. */
#define IEEE802154_MAC_MLME_POLL_PENDING_TIMEOUT_US (50000)

typedef struct {
    uint8_t l2addr[IEEE802154_LONG_ADDRESS_LEN];
    uint8_t l2addr_len;
} ieee802154_l2addr_t; 

typedef enum {
    IEEE802154_OK = 0,
    IEEE802154_NOMEM,
    IEEE802154_ERROR,
} ieee802154_return_code_t;

typedef enum {
    MLME_UNDEF = 0,
    MLME_POLL,
} ieee802154_mlme_req_t;

typedef enum {
    MCPS_UNDEF = 0,
    MCPS_DATA,
} ieee802154_mcps_req_t;

typedef struct {
    list_node_t node;
    ieee802154_l2addr_t *addr_data;
    gnrc_pktqueue_t *queue;
} ieee802154_idtxq_t;

typedef enum {
    IEEE802154_ADDR_MODE_SHORT,
    IEEE802154_ADDR_MODE_EXTENDED,
} ieee802154_addr_mode_t;

typedef struct {
    uint8_t key_source[IEEE802154_LONG_ADDRESS_LEN];
} ieee802154_key_source_t;

typedef struct {
    ieee802154_addr_mode_t        coord_addr_mode;
    le_uint16_t                   coord_pan_id;
    ieee802154_l2addr_t           coord_address;
#ifdef IEEE802154_MAC_TO_BE_IMPLEMENTED
    ieee802154_sec_scf_seclevel_t security_level;
    ieee802154_sec_scf_keymode_t  key_id_mode;
    ieee802154_key_source_t       key_source;
    uint8_t                       key_index;
#endif
} ieee802154_mlme_poll_request_t;

typedef struct { 
    /* The packet snip that contains the payload (MSDU) that is transferred
     * within a MAC frame. The frame will be built by the MAC layer
     * based on infromation given via the request primitive.
     * This pointer is used to identify the packet with the confirmation. */
    gnrc_pktsnip_t *pkt;
    //iolist_t *iolist;
    ///* The packet handle that must be released when the confirmation.
    // * was received by the upper layer. */ 
    //gnrc_pktsnip_t *pkt;
} ieee802154_msdu_t;

/* placeholder type for not yet defined members */
typedef void* ieee802154_undef_t;

typedef struct {
    ieee802154_addr_mode_t        src_addr_mode;
    ieee802154_addr_mode_t        dst_addr_mode;
    le_uint16_t                   dst_pan_id;
    ieee802154_l2addr_t           dst_address;
    ieee802154_msdu_t             msdu;
    /* not needed (implicit via buffer) */
    /*uint8_t                     msdu_handle; */
    bool                          ack_tx;
    bool                          indirect_tx;
    ieee802154_sec_scf_seclevel_t security_level;
#ifdef IEEE802154_MAC_TO_BE_IMPLEMENTED
    ieee802154_undef_t           header_ie_list;
    ieee802154_undef_t           payload_ie_list;
    ieee802154_undef_t           header_ie_id_list;
    ieee802154_undef_t           nested_ie_subid_list;
    bool                         gts_tx;
    ieee802154_sec_scf_keymode_t key_id_mode;
    ieee802154_key_source_t      key_source;
    uint8_t                      key_index;
    ieee802154_undef_t           uwbprf;
    ieee802154_undef_t           ranging;
    ieee802154_undef_t           uwb_preamble_symbol_repetitions;
    ieee802154_undef_t           data_rate;
    ieee802154_undef_t           loc_enh_info_post_len;
    bool                         panid_suppressed;
    bool                         seq_num_suppressed;
    bool                         send_multipurpose;
    ieee802154_undef_t           frak_policy;
    bool                         critical_event_message;
#endif
} ieee802154_mcps_data_request_t;

typedef uint32_t ieee802154_timestamp_t;

typedef enum {
    MCPS_SUCCESS,
    MCPS_TRANSACTION_OVERFLOW,
    MCPS_TRANSACTION_EXPIRED,
    MCPS_CHANNEL_ACCESS_FAILURE,
    MCPS_INVALID_ADDRESS,
    MCPS_INVALID_GTS,
    MCPS_NO_ACK,
    MCPS_COUNTER_ERROR,
    MCPS_FRAME_TOO_LONG,
    MCPS_UNAVAILABLE_KEY,
    MCPS_UNSUPPORTED_SECURITY,
    MCPS_UNSUPPORTED_FEATURE,
    MCPS_INVALID_PARAMETER,
    MCPS_UNSUPPORTED_PRF,
    MCPS_UNSUPPORTED_RANGING,
    MCPS_UNSUPPORTED_PSR,
    MCPS_UNSUPPORTED_DATARATE,
    MCPS_UNSUPPORTED_LEIP,
    MCPS_ACK_RCVD_NODSN_NOSA
} ieee802154_mcps_data_confirm_status_t;

typedef struct {
    //ieee802154_msdu_t                     *msdu_handle;
    ieee802154_msdu_t                     msdu_handle;
    ieee802154_timestamp_t                timestamp;
    uint8_t                               num_backoffs;
    ieee802154_mcps_data_confirm_status_t status;
#ifdef IEEE802154_MAC_TO_BE_IMPLEMENTED
    bool                         ranging_received;
    uint32_t                     ranging_counter_start;
    uint32_t                     ranging_counter_stop;
    ieee802154_undef_t           ranging_tracking_interval;
    ieee802154_undef_t           ranging_offset;
    uint8_t                      ranging_fom;
    uint8_t                      *ack_payload;
#endif
} ieee802154_mcps_data_confirm_t;

typedef enum {
    MLME_SUCCESS,
    MLME_CHANNEL_ACCESS_FAILURE,
    MLME_NO_ACK,
    MLME_NO_DATA,
    MLME_COUNTER_ERROR,
    MLME_FRAME_TOO_LONG,
    MLME_UNAVAILABLE_KEY,
    MLME_UNSUPPORTED_SECURITY,
    MLME_INVALID_PARAMETER,
} ieee802154_mlme_status_t;

typedef struct {
    ieee802154_mlme_status_t status;
} ieee802154_mlme_poll_confirm_t;

/** @brief iolist forward declaration */
typedef struct ieee802154_mac ieee802154_mac_t;

typedef void (*ieee802154_mlme_poll_confirm_cb_t)(ieee802154_mac_t *mac,
                                                  ieee802154_mlme_poll_confirm_t *confirm);

typedef void (*ieee802154_mcps_data_confirm_cb_t)(ieee802154_mac_t *mac,
                                                  ieee802154_mcps_data_confirm_t *confirm);

typedef struct {
    union {
       //ieee802154_mlme_poll_confirm_t *poll_confirm;
       /* for async request-confirm calls a callback is
        * used to return the confirmation instead of returning
        * the data via the pointer to the caller-allocated
        * confirmation structure. */
       ieee802154_mlme_poll_confirm_cb_t poll_confirm_cb;
       ieee802154_mcps_data_confirm_cb_t data_confirm_cb;
    };
} ieee802154_confirm_ptr_t;

typedef struct {
    union {
       ieee802154_mlme_poll_request_t *poll_request;
       ieee802154_mcps_data_request_t *data_request;
    };
} ieee802154_request_ptr_t;

typedef struct {
    bool recvd_ack;
    bool data_pending;
    bool medium_busy;
} ieee802154_tx_done_info_t;

typedef struct {
    void (*rx_done_cb)(ieee802154_mac_t *mac);
    void (*tx_done_cb)(ieee802154_mac_t *mac, ieee802154_tx_done_info_t *info);
} ieee802154_driver_t;

/* type that stores state relatd to an MCPS-DATA request */
typedef struct {
    /* handle of the transmitted data */
    ieee802154_msdu_t   msdu_handle;
} ieee802154_mcps_data_request_state_t;
    
typedef struct {
    /* the type of the currently active MCPS request */
    ieee802154_mcps_req_t mcps_req;
    union {
        ieee802154_mcps_data_request_state_t data;
    };
} ieee802154_mcps_request_state_t;

struct ieee802154_mac {
    ieee802154_driver_t *driver;
    /* holds the current MLME request state. */
    ieee802154_mlme_req_t mlme_req;
    ieee802154_mcps_request_state_t mcps_state;
      /* structure that holds a pointer to the request params */
    ieee802154_request_ptr_t mlme_mcps_request;
    /* structure that holds a pointer to the confirm params */
    ieee802154_confirm_ptr_t mlme_mcps_confirm;
    /* mutex for mac state */
    mutex_t            lock;
    /* buffer for queue entries for indirect transmission packets. */
    gnrc_pktqueue_t    idtx_pktq_entry_pool[IEEE802154_MAC_IDTX_PKTS_NUMOF];
    /* memory to be used for all indirect tx queues. */
    ieee802154_idtxq_t idtx_queue_pool[IEEE802154_MAC_IDTX_QUEUES_NUMOF];
    /* The list of queues for indirect transmssions, where each
     * list entry is ne queue per destination address. */
    list_node_t        idtx_queues_list;
    /* Buffer to store all L2 addrs that are expecting indirect transmissions.
     * For now this buffer will be populated when receiving a data request from
     * another node.
     * The length of an unused entry will be set to 0. */
    ieee802154_l2addr_t idtx_l2addrs[IEEE802154_MAC_IDTX_QUEUES_NUMOF];
    event_periodic_t    periodic_data_request_event;
    ztimer_t            data_request_timeout_timer;
    event_t             request_offload_event;
    /* pointer to next pkt that shall be sent via offloaded mcps data request */
    gnrc_pktsnip_t      *pending_idtx_pkt;
    /* pointer to confirm_cb that shall be used for IDTX.
     * TODO: handle this properly (global confirm registration or per-tx state handling)*/
    ieee802154_mcps_data_confirm_cb_t idtx_data_confirm_cb;
    /* for testing purposes a fixed coordinator addr is stored in the mac instance */
    ieee802154_l2addr_t *coordinator_addr;
    /**
     * Event used to trigger data requests via event queue and callback.
     */
    event_t data_request_event;

};

void ieee802154_mlme_poll_request(ieee802154_mac_t *mac,
                                  ieee802154_mlme_poll_request_t *request,
                                  ieee802154_mlme_poll_confirm_cb_t confirm_cb);

void ieee802154_perform_poll_request(ieee802154_mac_t *mac);

/* @brief issue an MCPS-DATA request to the MAC layer.
 *
 * @param mac         Reference to the mac lyer instance.
 * @param request     Structure holding all necessary data for the request.
 * @param confirm_cb  Callback that will be issued once the request finished.
 *
 * @note The confirmation callback will only be issued if the request was
 *       sucessfully created. This might happen during execution of this
 *       function.
 *
 * @return 0 if the request could be created.
 * @return negative if there was an immediate error when creating the request.
 */
int ieee802154_mcps_data_request(ieee802154_mac_t *mac,
                                 ieee802154_mcps_data_request_t *request,
                                 ieee802154_mcps_data_confirm_cb_t confirm_cb);

/* check if a destination address uses indirect transimssion via MLME-POLL */
bool ieee802154_dst_addr_uses_idtx(const ieee802154_mac_t *mac, const ieee802154_l2addr_t *dst);

ieee802154_l2addr_t* ieee802154_mac_get_coordinator_l2addr(ieee802154_mac_t *mac);

//TODO remove/refactor below functions
void _init_mac_data(ieee802154_mac_t *macdata);
ieee802154_mac_t* _netif2mac(void *netif);
ieee802154_idtxq_t *_alloc_idtx_pktq(ieee802154_mac_t *mac, ieee802154_l2addr_t *addr);
gnrc_pktqueue_t *_alloc_idtx_pktq_entry(ieee802154_mac_t *mac, gnrc_pktsnip_t *pkt);
gnrc_pktqueue_t **_get_idtxq(ieee802154_mac_t *mac, ieee802154_l2addr_t *addr);
bool _l2addr_equals(const ieee802154_l2addr_t *a,  const ieee802154_l2addr_t *b);
ieee802154_l2addr_t *_save_l2addr_for_idtx(ieee802154_mac_t *mac, const uint8_t *dst_l2addr, uint8_t dst_l2addr_len);
gnrc_pktsnip_t *_get_next_indirect_pkt(ieee802154_mac_t *mac, ieee802154_l2addr_t *addr);
void _send_indirect_tx_queued_pkt(ieee802154_mac_t *mac, ieee802154_l2addr_t *addr);
int _build_data_request(netdev_ieee802154_t *state, ieee802154_mlme_poll_request_t *request, uint8_t *psdu);
void _trigger_data_request(ieee802154_mac_t *mac);
void _enable_periodic_data_request(ieee802154_mac_t *mac, unsigned int poll_ms);
int _send_data_request(netdev_ieee802154_t *state,
                       ieee802154_mlme_poll_request_t *request);
void _control_radio_sleep(netdev_t *dev, bool sleep);
void _data_request_timeout(void *arg);

bool ieee802154_mac_requested_ack(ieee802154_mac_t *mac);

#endif /* NET_IEEE802154_MAC_H */

