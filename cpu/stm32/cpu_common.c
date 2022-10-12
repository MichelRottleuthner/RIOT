/*
 * Copyright (C) 2016 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     cpu_stm32
 * @{
 *
 * @file
 * @brief       Shared CPU specific function for the STM32 CPU family
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 *
 * @}
 */

#include "periph_conf.h"
#include "periph_cpu.h"

#define ENABLE_DEBUG 0
#include "debug.h"

#if IS_USED(MODULE_GCLK)
#include "gclk.h"
#include "gclk_stm32_common_conf.h" /* pulls in definitions of the clock instances */

bool apb_clk_cached[3] = { false };
uint32_t clk_cache[3];
uint32_t periph_apb_clk(uint8_t bus)
{
#ifdef CLOCK_APB2
    if (bus == APB2) {
        if (apb_clk_cached[APB2]) {
            return clk_cache[APB2];
        } else {
            clk_cache[APB2] = gclk_get_current_freq(&gclk_stm32_apb2_scaler.base);
            apb_clk_cached[APB2] = true;
            return clk_cache[APB2];
        }
    }
#endif
#if defined (CPU_FAM_STM32L4) || defined(CPU_FAM_STM32WB)
    else if (bus == APB12) {
        if (apb_clk_cached[APB12]) {
            return clk_cache[APB12];
        } else {
            clk_cache[APB12] = gclk_get_current_freq(&gclk_stm32_apb1_scaler.base);
            apb_clk_cached[APB12] = true;
            return clk_cache[APB12];
        }
    }
#endif
    if (apb_clk_cached[APB1]) {
        return clk_cache[APB1];
    } else {
        clk_cache[APB1] = gclk_get_current_freq(&gclk_stm32_apb1_scaler.base);
        apb_clk_cached[APB1] = true;
        return clk_cache[APB1];
    }
}

uint32_t _get_current_timer_input_clock(uint8_t bus) {
#ifdef CLOCK_APB2
    if (bus == APB2) {
        return gclk_get_current_freq(&gclk_apb2_tim_mul_scaler.base);
    }
#endif
#if defined (CPU_FAM_STM32L4) || defined(CPU_FAM_STM32WB)
    else if (bus == APB12) {
        return gclk_get_current_freq(&gclk_apb1_tim_mul_scaler.base);
    }
#endif
    return gclk_get_current_freq(&gclk_apb1_tim_mul_scaler.base);
}

#else /* IS_USED(MODULE_GCLK) */
/**
 * @brief   Timer specific additional bus clock prescaler
 *
 * This prescale factor is dependent on the actual APBx bus clock divider, if
 * the APBx presacler is != 1, it is set to 2, if the APBx prescaler is == 1, it
 * is set to 1.
 *
 * See reference manuals section 'reset and clock control'.
 */
static const uint8_t apbmul[] = {
#if (CLOCK_APB1 < CLOCK_CORECLOCK)
    [APB1] = 2,
#if defined(CPU_FAM_STM32L4) || defined(CPU_FAM_STM32WB) || \
    defined(CPU_FAM_STM32G0) || defined(CPU_FAM_STM32G4) || \
    defined(CPU_FAM_STM32L5) || defined(CPU_FAM_STM32U5) || \
    defined(CPU_FAM_STM32WL)
    [APB12] = 2,
#endif
#else
    [APB1] = 1,
#if defined(CPU_FAM_STM32L4) || defined(CPU_FAM_STM32WB) || \
    defined(CPU_FAM_STM32G0) || defined(CPU_FAM_STM32G4) || \
    defined(CPU_FAM_STM32L5) || defined(CPU_FAM_STM32U5) || \
    defined(CPU_FAM_STM32WL)
    [APB12] = 1,
#endif
#endif
#if (CLOCK_APB2 < CLOCK_CORECLOCK)
    [APB2] = 2
#else
    [APB2] = 1
#endif
};

uint32_t periph_apb_clk(uint8_t bus)
{
    if (bus == APB1) {
        return CLOCK_APB1;
    }
#if defined (CPU_FAM_STM32L4) || defined(CPU_FAM_STM32WB)
    else if (bus == APB12) {
        return CLOCK_APB1;
    }
#endif
    else {
        return CLOCK_APB2;
    }
}
#endif /* IS_USED(MODULE_GCLK) */

uint32_t periph_timer_clk(uint8_t bus)
{
#if IS_USED(MODULE_GCLK)
    return _get_current_timer_input_clock(bus);
#else
    return periph_apb_clk(bus) * apbmul[bus];
#endif /* IS_USED(MODULE_GCLK) */
}

void periph_clk_en(bus_t bus, uint32_t mask)
{
    switch (bus) {
        case APB1:
#if defined(CPU_FAM_STM32L4) || defined(CPU_FAM_STM32WB) || \
    defined(CPU_FAM_STM32G4) || defined(CPU_FAM_STM32L5) || \
    defined(CPU_FAM_STM32U5) || defined(CPU_FAM_STM32WL)
            RCC->APB1ENR1 |= mask;
#elif defined(CPU_FAM_STM32G0)
            RCC->APBENR1 |= mask;
#elif defined(CPU_FAM_STM32MP1)
            RCC->MC_APB1ENSETR |= mask;
#else
            RCC->APB1ENR |= mask;
#endif
            break;
#if !defined(CPU_FAM_STM32G0)
        case APB2:
#if defined(CPU_FAM_STM32MP1)
            RCC->MC_APB2ENSETR |= mask;
#else
            RCC->APB2ENR |= mask;
#endif
            break;
#endif
#if defined(CPU_FAM_STM32WL) || defined(CPU_FAM_STM32U5)
        case APB3:
            RCC->APB3ENR |= mask;
            break;
#endif
#if defined(CPU_FAM_STM32L4) || defined(CPU_FAM_STM32WB) || \
    defined(CPU_FAM_STM32G4) || defined(CPU_FAM_STM32L5) || \
    defined(CPU_FAM_STM32U5) || defined(CPU_FAM_STM32WL)
        case APB12:
            RCC->APB1ENR2 |= mask;
            break;
#elif defined(CPU_FAM_STM32G0)
        case APB12:
            RCC->APBENR2 |= mask;
            break;
#endif
#if defined(CPU_FAM_STM32L0) || defined(CPU_FAM_STM32G0)
        case AHB:
            RCC->AHBENR |= mask;
            break;
        case IOP:
            RCC->IOPENR |= mask;
            break;
#elif defined(CPU_FAM_STM32L1) || defined(CPU_FAM_STM32F1) || \
      defined(CPU_FAM_STM32F0) || defined(CPU_FAM_STM32F3)
        case AHB:
            RCC->AHBENR |= mask;
            break;
#elif defined(CPU_FAM_STM32F2) || defined(CPU_FAM_STM32F4) || \
      defined(CPU_FAM_STM32L4) || defined(CPU_FAM_STM32F7) || \
      defined(CPU_FAM_STM32WB) || defined(CPU_FAM_STM32G4) || \
      defined(CPU_FAM_STM32L5) || defined(CPU_FAM_STM32U5) || \
      defined(CPU_FAM_STM32WL)
        case AHB1:
            RCC->AHB1ENR |= mask;
            break;
/* STM32F410 RCC doesn't provide AHB2 and AHB3 */
#if !defined(CPU_LINE_STM32F410Rx)
        case AHB2:
#if defined(CPU_FAM_STM32U5)
            RCC->AHB2ENR1 |= mask;
#else
            RCC->AHB2ENR |= mask;
#endif
            break;
#if defined(CPU_FAM_STM32U5)
        case AHB22:
            RCC->AHB2ENR2 |= mask;
            break;
#endif
        case AHB3:
            RCC->AHB3ENR |= mask;
            break;
#endif
#endif
#if defined(CPU_FAM_STM32MP1)
        case AHB4:
            RCC->MC_AHB4ENSETR |= mask;
            break;
#endif
        default:
            DEBUG("unsupported bus %d\n", (int)bus);
            break;
    }
    /* stm32xx-errata: Delay after a RCC peripheral clock enable */
    __DSB();
}

void periph_clk_dis(bus_t bus, uint32_t mask)
{
    switch (bus) {
        case APB1:
#if defined(CPU_FAM_STM32L4) || defined(CPU_FAM_STM32WB) || \
    defined(CPU_FAM_STM32G4) || defined(CPU_FAM_STM32L5) || \
    defined(CPU_FAM_STM32U5) || defined(CPU_FAM_STM32WL)
            RCC->APB1ENR1 &= ~(mask);
#elif defined(CPU_FAM_STM32G0)
            RCC->APBENR1 &= ~(mask);
#elif defined(CPU_FAM_STM32MP1)
            /* Write 1 to clear */
            RCC->MC_APB1ENCLRR |= mask;
#else
            RCC->APB1ENR &= ~(mask);
#endif
            break;
#if !defined(CPU_FAM_STM32G0)
        case APB2:
#if defined(CPU_FAM_STM32MP1)
            RCC->MC_APB2ENCLRR |= mask;
#else
            RCC->APB2ENR &= ~(mask);
#endif
            break;
#endif
#if defined(CPU_FAM_STM32WL) || defined(CPU_FAM_STM32U5)
        case APB3:
            RCC->APB3ENR &= ~(mask);
            break;
#endif
#if defined(CPU_FAM_STM32L4) || defined(CPU_FAM_STM32WB) || \
    defined(CPU_FAM_STM32G4) || defined(CPU_FAM_STM32L5) || \
    defined(CPU_FAM_STM32U5) || defined(CPU_FAM_STM32WL)
        case APB12:
            RCC->APB1ENR2 &= ~(mask);
            break;
#elif defined(CPU_FAM_STM32G0)
        case APB12:
            RCC->APBENR2 &= ~(mask);
            break;
#endif
#if defined(CPU_FAM_STM32L0)
        case AHB:
            RCC->AHBENR &= ~(mask);
            break;
        case IOP:
            RCC->IOPENR &= ~(mask);
            break;
#elif defined(CPU_FAM_STM32L1) || defined(CPU_FAM_STM32F1) || \
      defined(CPU_FAM_STM32F0) || defined(CPU_FAM_STM32F3)
        case AHB:
            RCC->AHBENR &= ~(mask);
            break;
#elif defined(CPU_FAM_STM32F2) || defined(CPU_FAM_STM32F4) || \
      defined(CPU_FAM_STM32L4) || defined(CPU_FAM_STM32F7) || \
      defined(CPU_FAM_STM32WB) || defined(CPU_FAM_STM32G4) || \
      defined(CPU_FAM_STM32L5) || defined(CPU_FAM_STM32U5) || \
      defined(CPU_FAM_STM32WL)
        case AHB1:
            RCC->AHB1ENR &= ~(mask);
            break;
/* STM32F410 RCC doesn't provide AHB2 and AHB3 */
#if !defined(CPU_LINE_STM32F410Rx)
        case AHB2:
#if defined(CPU_FAM_STM32U5)
            RCC->AHB2ENR1 &= ~(mask);
#else
            RCC->AHB2ENR &= ~(mask);
#endif
            break;
#if defined(CPU_FAM_STM32U5)
        case AHB22:
            RCC->AHB2ENR2 &= ~(mask);
            break;
#endif
        case AHB3:
            RCC->AHB3ENR &= ~(mask);
            break;
#endif
#if defined(CPU_FAM_STM32WB)
        case AHB4:
            RCC->AHB3ENR &= ~(mask);
            break;
#endif
#endif
        default:
            DEBUG("unsupported bus %d\n", (int)bus);
            break;
    }
}

#if defined(CPU_FAM_STM32L4) || defined(CPU_FAM_STM32G4) || \
    defined(CPU_FAM_STM32L5) || defined(CPU_FAM_STM32U5)
void periph_lpclk_en(bus_t bus, uint32_t mask)
{
    switch (bus) {
        case APB1:
            RCC->APB1SMENR1 |= mask;
            break;
        case APB12:
            RCC->APB1SMENR2 |= mask;
            break;
        case APB2:
            RCC->APB2SMENR |= mask;
            break;
        case AHB1:
            RCC->AHB1SMENR |= mask;
            break;
        case AHB2:
#if defined(CPU_FAM_STM32U5)
            RCC->AHB2SMENR1 |= mask;
#else
            RCC->AHB2SMENR |= mask;
#endif
            break;
#if defined(CPU_FAM_STM32U5)
        case AHB22:
            RCC->AHB2SMENR2 |= mask;
            break;
#endif
        case AHB3:
            RCC->AHB3SMENR |= mask;
            break;
        default:
            DEBUG("unsupported bus %d\n", (int)bus);
            break;
    }
}

void periph_lpclk_dis(bus_t bus, uint32_t mask)
{
    switch (bus) {
        case APB1:
            RCC->APB1SMENR1 &= ~(mask);
            break;
        case APB12:
            RCC->APB1SMENR2 &= ~(mask);
            break;
        case APB2:
            RCC->APB2SMENR &= ~(mask);
            break;
        case AHB1:
            RCC->AHB1SMENR &= ~(mask);
            break;
        case AHB2:
#if defined(CPU_FAM_STM32U5)
            RCC->AHB2SMENR1 &= ~(mask);
#else
            RCC->AHB2SMENR &= ~(mask);
#endif
            break;
#if defined(CPU_FAM_STM32U5)
        case AHB22:
            RCC->AHB2SMENR2 &= ~(mask);
            break;
#endif
        case AHB3:
            RCC->AHB3SMENR &= ~(mask);
            break;
        default:
            DEBUG("unsupported bus %d\n", (int)bus);
            break;
    }
}
#elif defined(CPU_FAM_STM32F2) || \
      defined(CPU_FAM_STM32F4) || \
      defined(CPU_FAM_STM32F7)
void periph_lpclk_en(bus_t bus, uint32_t mask)
{
    switch (bus) {
        case APB1:
            RCC->APB1LPENR |= mask;
            break;
        case APB2:
            RCC->APB2LPENR |= mask;
            break;
        case AHB1:
            RCC->AHB1LPENR |= mask;
            break;
/* STM32F410 RCC doesn't provide AHB2 and AHB3 */
#if !defined(CPU_LINE_STM32F410Rx)
        case AHB2:
            RCC->AHB2LPENR |= mask;
            break;
        case AHB3:
            RCC->AHB3LPENR |= mask;
            break;
#endif
        default:
            DEBUG("unsupported bus %d\n", (int)bus);
            break;
    }
}

void periph_lpclk_dis(bus_t bus, uint32_t mask)
{
    switch (bus) {
        case APB1:
            RCC->APB1LPENR &= ~(mask);
            break;
        case APB2:
            RCC->APB2LPENR &= ~(mask);
            break;
        case AHB1:
            RCC->AHB1LPENR &= ~(mask);
            break;
/* STM32F410 RCC doesn't provide AHB2 and AHB3 */
#if !defined(CPU_LINE_STM32F410Rx)
        case AHB2:
            RCC->AHB2LPENR &= ~(mask);
            break;
        case AHB3:
            RCC->AHB3LPENR &= ~(mask);
            break;
#endif
        default:
            DEBUG("unsupported bus %d\n", (int)bus);
            break;
    }
}
#endif
