/*
 * Copyright (C) 2015 Kaspar Schleiser <kaspar@schleiser.de>
 * Copyright (C) 2016 PHYTEC Messtechnik GmbH
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 *
 */

/*
 * @ingroup sys_auto_init_gnrc_netif
 * @{
 *
 * @file
 * @brief   Auto initialization for kw2xrf network interfaces
 *
 * @author  Kaspar Schleiser <kaspar@schleiser.de>
 * @author  Jonas Remmert <j.remmert@phytec.de>
 * @author  Sebastian Meiling <s@mlng.net>
 */
#define LOG_LEVEL LOG_DEBUG
#include "log.h"
#include "board.h"
#include "net/gnrc/netif/ieee802154.h"
#include "net/gnrc.h"

#include "kw2xrf.h"
#include "kw2xrf_params.h"

/**
 * @brief   Define stack parameters for the MAC layer thread
 * @{
 */
#define KW2XRF_MAC_STACKSIZE     (THREAD_STACKSIZE_DEFAULT)
#ifndef KW2XRF_MAC_PRIO
#define KW2XRF_MAC_PRIO          (GNRC_NETIF_PRIO)
#endif

#define KW2XRF_NUM ARRAY_SIZE(kw2xrf_params)

#if IS_USED(MODULE_IEEE802154_RADIO_HAL)
extern void kw2xrf_hal_init(kw2xrf_t *dev,
                            gpio_cb_t isr_cb, void *cb_ctx);
static kw2xrf_dev_evt_ctx_t kw2xrf_dev_ctxs[KW2XRF_NUM];

static void kw2xrf_irq_cb(void *ctx) {
    kw2xrf_dev_evt_ctx_t *c = (kw2xrf_dev_evt_ctx_t*)ctx;
    if (c->event_queue) {
        /* calls the below kw2xrf_irq_event_handler from the the netif thread */
        event_post(c->event_queue, &c->event);
    }
}

extern void kw2xrf_radio_hal_irq_handler(ieee802154_dev_t *dev);

static void kw2xrf_irq_event_handler(event_t *evt){
    kw2xrf_dev_evt_ctx_t *ctx = container_of(evt, kw2xrf_dev_evt_ctx_t, event);
    kw2xrf_radio_hal_irq_handler(&ctx->dev.hal);
}
#else
static kw2xrf_t kw2xrf_devs[KW2XRF_NUM];
#endif

static gnrc_netif_t _netif[KW2XRF_NUM];
static char _kw2xrf_stacks[KW2XRF_NUM][KW2XRF_MAC_STACKSIZE];

void auto_init_kw2xrf(void)
{
    for (unsigned i = 0; i < KW2XRF_NUM; i++) {
        const kw2xrf_params_t *p = &kw2xrf_params[i];

        LOG_DEBUG("[auto_init_netif] initializing kw2xrf #%u\n", i);
#if (IS_USED(MODULE_IEEE802154_RADIO_HAL))

            //kw2xrf_dev_ctxs[i].event_queue = NULL;
            kw2xrf_dev_ctxs[i].event_queue = EVENT_PRIO_HIGHEST;
            kw2xrf_dev_ctxs[i].event.handler = kw2xrf_irq_event_handler;

            printf("setup...\n");
            kw2xrf_setup(&kw2xrf_dev_ctxs[i].dev, (kw2xrf_params_t*)p);

            //kw2xrf_init(&kw2xrf_dev_ctxs[i].dev, NULL);

            printf("hal_init\n");
            kw2xrf_hal_init(&kw2xrf_dev_ctxs[i].dev,
                            kw2xrf_irq_cb, &kw2xrf_dev_ctxs[i]);

            printf("register netdev..\n");
            netdev_register((netdev_t* )&kw2xrf_dev_ctxs[i].dev, NETDEV_KW2XRF, 0);
            printf("init submac..\n");
            netdev_ieee802154_submac_init(&kw2xrf_dev_ctxs[i].dev.netdev, &kw2xrf_dev_ctxs[i].dev.hal);
            printf("init submac done\n");

            gnrc_netif_ieee802154_create(&_netif[i], _kw2xrf_stacks[i], KW2XRF_MAC_STACKSIZE,
                                         KW2XRF_MAC_PRIO, "kw2xrf",
                                         &kw2xrf_dev_ctxs[i].dev.netdev.dev.netdev);
            LOG_DEBUG("create netif...\n");

            //ieee802154_radio_set_cca_mode(&kw2xrf_dev_ctxs[i].dev.hal, IEEE802154_CCA_MODE_ED_THRESH_AND_CS);
#else
            kw2xrf_setup(&kw2xrf_devs[i], (kw2xrf_params_t*) p);
            gnrc_netif_ieee802154_create(&_netif[i], _kw2xrf_stacks[i], KW2XRF_MAC_STACKSIZE,
                                         KW2XRF_MAC_PRIO, "kw2xrf",
                                         (netdev_t *)&kw2xrf_devs[i]);
#endif
    }
}
/** @} */
