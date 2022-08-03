/*
 * Copyright (C) 2014-2016 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     cpu_efm32
 * @ingroup     drivers_periph_spi
 * @{
 *
 * @file
 * @brief       Low-level SPI driver implementation
 *
 * @author      Ryan Kurte <ryankurte@gmail.com>
 * @author      Bas Stottelaar <basstottelaar@gmail.com>
 * @author      Christian Amsüss <c@amsuess.com>
 * @}
 */

#include <assert.h>

#include "cpu.h"
#include "sched.h"
#include "thread.h"
#include "mutex.h"

#include "periph_conf.h"
#include "periph/gpio.h"
#include "periph/spi.h"

#include "em_device.h"
#include "em_cmu.h"
#include "em_usart.h"

static mutex_t spi_lock[SPI_NUMOF];

void spi_init(spi_t bus)
{
    assert(bus < SPI_NUMOF);

    /* initialize lock */
    mutex_init(&spi_lock[bus]);

    /* initialize pins */
    spi_init_pins(bus);
}

void spi_init_pins(spi_t bus)
{
    /* configure the pins */
    gpio_init(spi_config[bus].clk_pin, GPIO_OUT);
    gpio_init(spi_config[bus].mosi_pin, GPIO_OUT);
    gpio_init(spi_config[bus].miso_pin, GPIO_IN_PD);
}


int actual_spi_speeds[SPI_NUMOF];
void spi_acquire(spi_t bus, spi_cs_t cs, spi_mode_t mode, spi_clk_t clk)
{
    (void)cs;
    assert((unsigned)bus < SPI_NUMOF);

    mutex_lock(&spi_lock[bus]);

    /* power on spi bus */
    CMU_ClockEnable(cmuClock_HFPER, true);
    CMU_ClockEnable(spi_config[bus].cmu, true);

    USART_InitSync_TypeDef init = USART_INITSYNC_DEFAULT;

    init.baudrate = (uint32_t)clk;
    init.clockMode = (USART_ClockMode_TypeDef)mode;
    init.msbf = true;

    USART_InitSync(spi_config[bus].dev, &init);

    /* get the actually set up frequency */
    /* HFPERCLK/HFPERBCLK used to clock all USART/UART peripheral modules. */
    uint32_t refFreq;
#if defined(_SILICON_LABS_32B_SERIES_2)
    refFreq = CMU_ClockFreqGet(cmuClock_PCLK);
#else
#if defined(_CMU_HFPERPRESCB_MASK)
    if (spi_config[bus].dev == USART2) {
      refFreq = CMU_ClockFreqGet(cmuClock_HFPERB);
    } else {
      refFreq = CMU_ClockFreqGet(cmuClock_HFPER);
    }
#else
    refFreq = CMU_ClockFreqGet(cmuClock_HFPER);
#endif
#endif

    /* access the internal div member to calculate back to the baudrate that is ctually setup */
    uint32_t div = spi_config[bus].dev->CLKDIV;

    /*
     * CLKDIV in synchronous mode is given by:
     *
     * CLKDIV = 256 * (fHFPERCLK/(2 * br) - 1)
     *  so we calculate back like this:
     *
     * CLKDIV / 256 + 1 = fHFPERCLK/(2 * br)
     * br * (CLKDIV / 256 + 1) =  0.5*fHFPERCLK
     * br  =  fHFPERCLK / 2 * (CLKDIV / 256 + 1)
     */
    uint32_t br = refFreq / (2 * ((div >> 8) + 1));
    actual_spi_speeds[bus] = br;

    /* configure pin functions */
#if defined(_SILICON_LABS_32B_SERIES_0)
    spi_config[bus].dev->ROUTE = (spi_config[bus].loc |
                                  USART_ROUTE_RXPEN |
                                  USART_ROUTE_TXPEN |
                                  USART_ROUTE_CLKPEN);
#elif defined(_SILICON_LABS_32B_SERIES_1)
    spi_config[bus].dev->ROUTELOC0 = spi_config[bus].loc;
    spi_config[bus].dev->ROUTEPEN = (USART_ROUTEPEN_RXPEN |
                                     USART_ROUTEPEN_TXPEN |
                                     USART_ROUTEPEN_CLKPEN);
#endif
}

void spi_release(spi_t bus)
{
    /* power off spi bus */
    CMU_ClockEnable(spi_config[bus].cmu, false);

    mutex_unlock(&spi_lock[bus]);
}

void spi_transfer_bytes(spi_t bus, spi_cs_t cs, bool cont,
                        const void *out, void *in, size_t len)
{
    uint8_t *out_buf = (uint8_t *)out;
    uint8_t *in_buf = (uint8_t *)in;

    if (cs != SPI_CS_UNDEF) {
        gpio_clear((gpio_t)cs);
    }

    for (size_t i = 0; i < len; i++) {
        uint8_t ret = USART_SpiTransfer(spi_config[bus].dev,
                                        out != NULL ? out_buf[i] : 0);

        if (in != NULL) {
            in_buf[i] = ret;
        }
    }

    if ((!cont) && (cs != SPI_CS_UNDEF)) {
        gpio_set((gpio_t)cs);
    }
}
