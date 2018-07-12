/*
 * Copyright 2015 Ludwig Knüpfer
 *           2015 Christian Mehlis
 *           2016-2017 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_dht
 * @{
 *
 * @file
 * @brief       Device driver implementation for the DHT 11 and 22
 *              temperature and humidity sensor
 *
 * @author      Ludwig Knüpfer <ludwig.knuepfer@fu-berlin.de>
 * @author      Christian Mehlis <mehlis@inf.fu-berlin.de>
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 *
 * @}
 */

#include <stdint.h>
#include <string.h>

#include "log.h"
#include "assert.h"
#include "xtimer.h"
#include "timex.h"
#include "periph/gpio.h"
#include "mutex.h"

#include "dht.h"
#include "dht_params.h"

#define ENABLE_DEBUG    (0)
#include "debug.h"

#define PULSE_WIDTH_THRESHOLD       (40U)

typedef struct{
    uint32_t start;
    uint32_t read_val;
    mutex_t mutex;
    gpio_t pin;
    uint8_t bitcnt;
} dht_isr_ctx_t;

static void cb(void *arg)
{
    dht_isr_ctx_t *ctx = (dht_isr_ctx_t*)arg;
    uint32_t now = xtimer_now_usec();

    if (gpio_read(ctx->pin)) {
        ctx->start = now;
    }else{
        ctx->read_val<<=1;
        if ((now - ctx->start) > PULSE_WIDTH_THRESHOLD) {
            ctx->read_val |= 0x0001;
        }
        ctx->bitcnt--;
    }

    if (ctx->bitcnt == 0) {
      gpio_irq_disable(ctx->pin);
      mutex_unlock(&ctx->mutex);
      return;
    }
}

static uint16_t read(const dht_t *dev, int bits)
{
    dht_isr_ctx_t ctx;
    mutex_lock(&ctx.mutex);
    ctx.bitcnt = bits;
    ctx.pin = dev->pin;
    gpio_init_int(dev->pin, dev->in_mode, GPIO_BOTH, cb, &ctx);

    /* wait for the callback to give back the mutex */
    mutex_lock(&ctx.mutex);

    /* reset mutex */
    mutex_unlock(&ctx.mutex);

    return ctx.read_val;
}

int dht_init(dht_t *dev, const dht_params_t *params)
{
    DEBUG("dht_init\n");

    /* check parameters and configuration */
    assert(dev && params &&
           ((dev->type == DHT11) || (dev->type == DHT22) || (dev->type == DHT21)));

    memcpy(dev, params, sizeof(dht_t));

    gpio_init(dev->pin, GPIO_OUT);
    gpio_set(dev->pin);

    xtimer_usleep(2000 * US_PER_MS);

    DEBUG("dht_init: success\n");
    return DHT_OK;
}

int dht_read(const dht_t *dev, int16_t *temp, int16_t *hum)
{
    uint8_t csum, sum;
    uint16_t raw_hum, raw_temp;

    assert(dev && temp && hum);

    /* send init signal to device */
    gpio_clear(dev->pin);
    xtimer_usleep(20 * US_PER_MS);
    gpio_set(dev->pin);
    xtimer_usleep(40);

    /* sync on device */
    gpio_init(dev->pin, dev->in_mode);
    while (!gpio_read(dev->pin)) {}
    while (gpio_read(dev->pin)) {}

    /*
     * data is read in sequentially, highest bit first:
     *  40 .. 24  23   ..   8  7  ..  0
     * [humidity][temperature][checksum]
     */

    /* read the humidity, temperature, and checksum bits */
    raw_hum = read(dev, 16);
    raw_temp = read(dev, 16);
    csum = (uint8_t)read(dev, 8);

    /* set pin high again - so we can trigger the next reading by pulling it low
     * again */
    gpio_init(dev->pin, GPIO_OUT);
    gpio_set(dev->pin);

    /* validate the checksum */
    sum = (raw_temp >> 8) + (raw_temp & 0xff) + (raw_hum >> 8) + (raw_hum & 0xff);
    if ((sum != csum) || (csum == 0)) {
        DEBUG("error: checksum invalid\n");
        return DHT_NOCSUM;
    }

    /* parse the RAW values */
    DEBUG("RAW values: temp: %7i hum: %7i\n", (int)raw_temp, (int)raw_hum);
    switch (dev->type) {
        case DHT11:
            *temp = (int16_t)((raw_temp >> 8) * 10);
            *hum = (int16_t)((raw_hum >> 8) * 10);
            break;
        case DHT22:
            *hum = (int16_t)raw_hum;
            /* if the high-bit is set, the value is negative */
            if (raw_temp & 0x8000) {
                *temp = (int16_t)((raw_temp & ~0x8000) * -1);
            }
            else {
                *temp = (int16_t)raw_temp;
            }
            break;
        default:
            return DHT_NODEV;      /* this should never be reached */
    }

    return DHT_OK;
}
