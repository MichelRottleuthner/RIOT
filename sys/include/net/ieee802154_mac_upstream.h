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

/* number of raw RX frames that can be buffered */
#define IEEE802154_MAC_RX_BUF_NUMOF (8)

/* Timeout for an MLME-POLL. After a data pending indication (indicated by the
 * pending bit in the ACK responding to a data request MAC command), the radio
 * will continue to listen for this time. If no data is received the readio will
 * stop listening again. */
#define IEEE802154_MAC_MLME_POLL_PENDING_DEFAULT_TIMEOUT_US (4000)

#define IEEE802154_MAC_EVENT_THREAD_STACKSIZE THREAD_STACKSIZE_DEFAULT

#ifndef IEEE802154_MAC_EVENT_THREAD_PRIO
/* If separate threads for MAC and radio are used, the MAC
 * should have a lower priority than the thread that serves
 * the radio to ensure interrupts of the radio are served ASAP. */
#define IEEE802154_MAC_EVENT_THREAD_PRIO      (3)
#endif

/** @brief ieee802154_mac_t forward declaration */
typedef struct ieee802154_mac ieee802154_mac_t;

/** @brief ieee802154_mcps_data_request_t forward declaration */
typedef struct ieee802154_mcps_data_request_struct ieee802154_mcps_data_request_t;

/** @brief ieee802154_mlme_poll_request_t forward declaration */
typedef struct ieee802154_mlme_poll_request_struct ieee802154_mlme_poll_request_t;

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

typedef enum {
    START,
    MEDIUM_BUSY,
    ACK_RECEIVED,
    ACK_TIMED_OUT,
    DATA_TIMED_OUT,
    RX_DATA,
    TX_DONE,
    RADIO_TURNED_ON,
    RADIO_TURNED_OFF,
} ieee802154_event_t;

typedef void (*ieee802154_mlme_poll_confirm_cb_t)(ieee802154_mac_t *mac,
                                                  ieee802154_mlme_poll_confirm_t *confirm);
typedef struct {
    bool recvd_ack;
    bool data_pending;
    bool medium_busy;
} ieee802154_tx_done_info_t;

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

typedef enum {
    REQ_MCPS_DATA,
    REQ_MLME_POLL,
} ieee802154_request_type_t;

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

/* Common metadata structure for each request type.
 * This must ne the first entry of every request type
 * that can be queued.*/
typedef struct {
    list_node_t next;
    ieee802154_request_type_t type;
} ieee802154_req_meta_t;

struct ieee802154_mlme_poll_request_struct {
    ieee802154_req_meta_t         meta;
    ieee802154_addr_mode_t        coord_addr_mode;
    le_uint16_t                   coord_pan_id;
    ieee802154_l2addr_t           coord_address;
    ieee802154_mlme_poll_confirm_cb_t confirm_cb;
    ieee802154_tx_done_info_t     tx_info;
    ieee802154_event_t            txd_offloaded_event;
#ifdef IEEE802154_MAC_TO_BE_IMPLEMENTED
    ieee802154_sec_scf_seclevel_t security_level;
    ieee802154_sec_scf_keymode_t  key_id_mode;
    ieee802154_key_source_t       key_source;
    uint8_t                       key_index;
#endif
};

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
    //ieee802154_msdu_t                   *msdu_handle;
    ieee802154_msdu_t                     msdu_handle;
    ieee802154_timestamp_t                timestamp;
    uint8_t                               num_backoffs;
    ieee802154_mcps_data_confirm_status_t status;
    ieee802154_mcps_data_request_t        *request;
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

typedef void (*ieee802154_mcps_data_confirm_cb_t)(ieee802154_mac_t *mac,
                                                  ieee802154_mcps_data_confirm_t *confirm);

struct ieee802154_mcps_data_request_struct {
    ieee802154_req_meta_t         meta;
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
    /* the confirm cb to call for confirming this request */
    ieee802154_mcps_data_confirm_cb_t confirm_cb;
    ieee802154_tx_done_info_t     tx_info;
    ieee802154_event_t            txd_offloaded_event;
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
};

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

typedef enum {
    SUB_FSM_INIT = 0,
    SUB_FSM_DONE = 1,
} ieee802154_mac_fsm_res_t;

typedef enum {
    POLL_INIT = SUB_FSM_INIT, /*< Initial state, request radio to be turned on. */
    POLL_DONE = SUB_FSM_DONE, /*< state machine finished. */
    POLL_WAIT_FOR_ACK,        /*< After sending the cmd, wait for an ACK. */
    POLL_WAIT_FOR_DATA,       /*< After receiving an ACK with PENDING bit,
                                  wait for data. */
} ieee802154_poll_state_t;

typedef enum {
    DATA_INIT = SUB_FSM_INIT, /*< Initial state, ensure radio is on. */
    DATA_DONE = SUB_FSM_DONE, /*< state machine finished. */
    DATA_WAIT_FOR_RADIO_ON,   /*< Waiting till radio is on. */
    DATA_WAIT_FOR_ACK,        /*< After sending data, wait for an ACK. */
} ieee802154_data_state_t;

typedef enum {
    RX_IDLE_INIT = SUB_FSM_INIT, /*< Initial state, ensure radio is on. */
    RX_IDLE_DONE = SUB_FSM_DONE, /*< state machine finished. */
    RX_IDLE_WAIT_FOR_RADIO_ON,   /*< Waiting till radio is on. */
    RX_IDLE_WAIT_FOR_RX,         /*< Wait for receiving data. */
} ieee802154_rx_idle_state_t;

typedef enum {
    NORX_IDLE_INIT = SUB_FSM_INIT, /*< Initial state, ensure radio is on. */
    NORX_IDLE_DONE = SUB_FSM_DONE, /*< state machine finished. */
    NORX_IDLE_WAIT_FOR_RADIO_OFF   /*< Waiting till radio is off. */
} ieee802154_norx_idle_state_t;

typedef enum {
    MAC_IDLE,
    MAC_MLME_POLL,
    MAC_MCPS_DATA,
} ieee802154_mac_request_mode_t;

typedef struct {
    union {
        ieee802154_data_state_t      data_state;
        ieee802154_poll_state_t      poll_state;
        ieee802154_rx_idle_state_t   rx_idle_state;
        ieee802154_norx_idle_state_t norx_idle_state;
    };
} ieee802154_mac_fsm_state_t;

typedef ieee802154_mac_fsm_state_t (*ieee802154_fsm_handler_t)(ieee802154_mac_t *mac,
                                                           ieee802154_mac_fsm_state_t state,
                                                           ieee802154_event_t event);

/* @brief structure to store a queued MCPS-DATA request.
 **/
typedef struct {
    gnrc_pktqueue_t    q_entry;
    ieee802154_mcps_data_request_t *req_ref;
} ieee802154_idtx_req_q_entry_t;

/* @brief structure to buffer incoming frames.
 *
 * NOTE: Currently there is no nice way to throttle the netdev interface
 * gracefully. The radio HAL will fix that once all radios are suported.
 * For now, we just add a buffering mechanism so we don't need to
 * throttle the radio in most cases.
 */
typedef struct {
    clist_node_t node;               /* list/queue entry */
    gnrc_pktsnip_t *mpdu;            /* the raw MPDU received */
    gnrc_pktsnip_t *ieee802154_hdr;  /* the marked header in the received MPDU */
} ieee802154_rx_offload_entry_t;

struct ieee802154_mac {
    event_queue_t *event_queue;
    event_queue_t *radio_event_queue;
    clist_node_t pending_mac_requests;
    ieee802154_driver_t *driver;
    ieee802154_tx_done_info_t *tx_done_info;
    ieee802154_data_state_t mcps_data_state;
    uint32_t t_last_on_air;
    uint32_t t_ifs;
    /* reference to the request currently being served. */
    ieee802154_request_ptr_t current_req;
    /* stores whether the current request was already confirmed.
     * This is needed to syncronize between timeout-, rx-, and off-events
     * during state machine operation.
     * Currently it is not possible to perform these operations atomically
     * in a way that guarantees to not interfere with ongoing radio transactions
     * (if the radio is served by another thread). */
    bool current_req_confirmed;
    event_t fsm_trigger_event;
    ieee802154_fsm_handler_t fsm_handler;
    ieee802154_mac_fsm_state_t fsm_state;

    /* some buffered state of the radio to only perform switching
     * operations when needed. */
    bool report_pending_on_ack_enabled;
    bool csma_is_enabled;
    bool radio_is_on;

    /* mutex for mac state */
    mutex_t            lock;

    /* buffer for queued request entries for indirect transmission packets.
     * NOTE: As of now the packet is built directly when the request is placed.
     * This has more overhead (memory for buffering) as just generating
     * the packet from the request when a poll request is received.
     * However, that way the request can be served faster. This saves
     * energy on the sleepy node (RFD), and allows better response times
     * for the "coordinator" devices with slower CPUs.
     *
     * Entries are marked as empty by setting q_entry.pkt to NULL.  */
    ieee802154_idtx_req_q_entry_t idtx_req_q_entry_pool[IEEE802154_MAC_IDTX_PKTS_NUMOF];
    /* Number of all pending indirect tx packets.
     * Used to for control of the pending bit to be sent in ACKs. */
    unsigned pending_idtx_pkt_cnt;
    /* Memory to be used for all indirect tx queues.
     * NOTE: This defines the maximum number of nodes that an indirect
     *       transmission may be queued for.
     *       The overall number of packets that may be enqueued simultaneously
     *       is independently limited by the above @ref idtx_req_q_entry_pool
     *       which is shared across all queues. */
    ieee802154_idtxq_t idtx_queue_pool[IEEE802154_MAC_IDTX_QUEUES_NUMOF];
    /* The list of active queues for indirect transmssions.
     * Each element (ieee802154_idtxq_t) holds one queue for one partiucular
     * destination address. */
    list_node_t        idtx_queues_list;
    /* Buffer to store all L2 addrs that are expecting indirect transmissions.
     * For now this buffer will be populated when receiving a data request from
     * another node.
     * The length of an unused entry will be set to 0. */
    ieee802154_l2addr_t idtx_l2addrs[IEEE802154_MAC_IDTX_QUEUES_NUMOF];
    /* lock for protecting idtx data above */
    mutex_t idtx_q_lock;

    ieee802154_rx_offload_entry_t rx_buffer_pool[IEEE802154_MAC_RX_BUF_NUMOF];
    clist_node_t free_rx_buffers;
    clist_node_t pending_rx_buffers;
    mutex_t rx_offload_lock;

    event_periodic_t    periodic_data_request_event;
    ztimer_t            data_request_timeout_timer;

    /* The time to wait for data after sending an MLME-POLL request
     * and receiving a corresponding ACK with the pending flag set.
     * Set to 0 to use the default value. */
    unsigned            poll_timeout_us;

    bool idle_listen_mode;
    bool periodic_mlme_poll_enabled;

    /* for testing purposes a fixed coordinator addr is stored in the mac instance */
    ieee802154_l2addr_t *coordinator_addr;

    /**
     * Events used to trigger data requests via the mac
     * event queue.
     */
    event_t data_request_event;
    event_t radio_turned_on_event;
    event_t radio_turned_off_event;
    event_t data_timeout_event;
    event_t rx_data_offload_event;
    event_t tx_done_offload_event;

    /**
     * Event used to request the radio to turn off
     * via its own event queue.
     */
    event_t radio_off_request_event;
    event_t radio_on_request_event;
};
    /**
     * Events used to trigger data requests via the mac
     * event queue.
     */

void ieee802154_mlme_poll_request(ieee802154_mac_t *mac,
                                  ieee802154_mlme_poll_request_t *request,
                                  ieee802154_mlme_poll_confirm_cb_t confirm_cb);

void ieee802154_perform_poll_request(ieee802154_mac_t *mac);

/* @brief issue an MCPS-DATA request to the MAC layer.
 *
 * @param mac         Reference to the mac layer instance.
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

/* @brief Enable periodic data request (MLME-POLL).
 *
 * @param mac         Reference to the mac layer instance.
 * @param poll_ms     Interval to send the request MAC command.
 * @param timeout_us  Waiting period to keep the radio on RX after an ACK
 *                    with pending flag was received.
 */
void ieee802154_enable_periodic_data_request(ieee802154_mac_t *mac,
                                             unsigned poll_ms,
                                             unsigned timeout_us);

/* @brief Configure idle listening mode.
 *
 * @param mac         Reference to the mac layer instance.
 * @param enable      true    enable idle listening (go to RX on IDLE).
 *                    false   radio will go to sleep if no operation is pending.
 */
void ieee802154_set_idle_listening(ieee802154_mac_t *mac, bool enabled);

/* check if a destination address uses indirect transimssion via MLME-POLL */
bool ieee802154_dst_addr_uses_idtx(const ieee802154_mac_t *mac, const ieee802154_l2addr_t *dst);

ieee802154_l2addr_t* ieee802154_mac_get_coordinator_l2addr(ieee802154_mac_t *mac);

void ieee802154_mac_init(ieee802154_mac_t *mac);

//TODO remove/refactor below functions
ieee802154_mac_t* _netif2mac(void *netif);
ieee802154_idtxq_t *_alloc_idtx_pktq(ieee802154_mac_t *mac, ieee802154_l2addr_t *addr);
gnrc_pktqueue_t *_alloc_idtx_pktq_entry(ieee802154_mac_t *mac, gnrc_pktsnip_t *pkt);
gnrc_pktqueue_t **_get_idtxq(ieee802154_mac_t *mac, ieee802154_l2addr_t *addr);
bool _l2addr_equals(const ieee802154_l2addr_t *a,  const ieee802154_l2addr_t *b);
ieee802154_l2addr_t *_save_l2addr_for_idtx(ieee802154_mac_t *mac, const uint8_t *dst_l2addr, uint8_t dst_l2addr_len);
void _send_indirect_tx_queued_pkt(ieee802154_mac_t *mac, ieee802154_l2addr_t *addr);
int _build_data_request(netdev_ieee802154_t *state, ieee802154_mlme_poll_request_t *request, uint8_t *psdu);
void _trigger_single_data_request(ieee802154_mac_t *mac, unsigned timeout_us);
void _enable_periodic_data_request(ieee802154_mac_t *mac, unsigned int poll_ms);
int _send_data_request(ieee802154_mac_t *mac,
                       ieee802154_mlme_poll_request_t *request);
void _control_radio_enable(ieee802154_mac_t *mac, bool on);
void _data_request_timeout(void *arg);

bool ieee802154_mac_requested_ack(ieee802154_mac_t *mac);

#endif /* NET_IEEE802154_MAC_H */

