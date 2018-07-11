/*
 * Copyright (C) 2017 HAW-Hamburg
                 2016 Kaspar Schleiser <kaspar@schleiser.de>
 *               2015 Freie Universität Berlin
 *               2015 Engineering-Spirit
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     cpu_stm32_common
 * @ingroup     drivers_periph_pm
 * @{
 *
 * @file
 * @brief       Implementation of the kernels power management interface
 *
 * @author      Nick v. IJzendoorn <nijzndoorn@engineering-spirit.nl>
 * @author      Kaspar Schleiser <kaspar@schleiser.de>
 * @author      Fabian Nack <nack@inf.fu-berlin.de>
 * @author      Michel Rottleuthner <michel.rottleuthner@haw-hamburg.de>
 *
 * @}
 */

#include "irq.h"
#include "periph/pm.h"
#if defined(CPU_FAM_STM32F1) || defined(CPU_FAM_STM32F2) || \
    defined(CPU_FAM_STM32F4) || defined(CPU_FAM_STM32L0) || \
    defined(CPU_FAM_STM32L4)
#include "stmclk.h"
#include "cpu.h"
#endif

#define ENABLE_DEBUG (0)
#include "debug.h"

#ifndef PM_STOP_CONFIG
/**
 * @brief Define config flags for stop mode
 *
 * Available values can be found in reference manual, PWR section, register CR.
 */
#define PM_STOP_CONFIG (PWR_CR_LPDS | PWR_CR_FPDS)
#endif

#if defined(CPU_FAM_STM32L4)
/**
 *  Power modes available on STM32L4x5 and STM32L4x6
 *  For details see ST reference manual RM0351 (DocID024597 Rev 5)
 */
enum stm32l4_system_sleepmode {
    /**
     * The Shutdown mode allows to achieve the lowest power
     * consumption. It is based on the deepsleep mode, with the voltage
     * regulator disabled. The VCORE domain is consequently powered
     * off. The PLL, the HSI16, the MSI, the LSI and the HSE
     * oscillators are also switched off.
     * SRAM1, SRAM2 and register contents are lost except for registers
     * in the Backup domain. The BOR is not available in Shutdown mode.
     * No power voltage monitoring is possible in this mode, therefore
     * the switch to Backup domain is not supported.
     *
     * In the Shutdown mode, the I/Os can be configured either with a
     * pull-up (refer to PWR_PUCRx registers (x=A,B,C,D,E,F,G,H),
     * or with a pull-down (refer to PWR_PDCRx registers
     * (x=A,B,C,D,E,F,G,H)), or can be kept in analog state. However
     * this configuration is lost when exiting the Shutdown mode due
     * to the power-on reset. The RTC outputs on PC13 are functional
     * in Shutdown mode. PC14 and PC15 used for LSE are also
     * functional. 5 wakeup pins (WKUPx, x=1,2...5) and the 3 RTC
     * tampers are available.
     */
    SYSTEM_SLEEPMODE_SHUTDOWN,

    /**
     * The PLL, the HSI16, the MSI and the HSE oscillators are also
     * switched off.
     *
     * SRAM1 and register contents are lost except for registers in
     * the Backup domain and Standby circuitry (see Figure 9). SRAM2
     * content can be preserved if the bit RRS is set in the PWR_CR3
     * register. In this case the Low-power regulator is ON and
     * provides the supply to SRAM2 only.
     *
     * In the Standby mode, the I/Os can be configured either with a
     * pull-up (refer to PWR_PUCRx registers (x=A,B,C,D,E,F,G,H)), or
     * with a pull-down (refer to PWR_PDCRx registers
     * (x=A,B,C,D,E,F,G,H)), or can be kept in analog state.
     * The RTC outputs on PC13 are functional in Standby mode. PC14 and
     * PC15 used for LSE are also functional. 5 wakeup pins (WKUPx,
     * x=1,2...5) and the 3 RTC tampers are available.
     */
    SYSTEM_SLEEPMODE_STANDBY,

    /**
     * The Stop 2 mode is based on the Cortex®-M4 deepsleep mode
     * combined with peripheral clock gating. In Stop 2 mode, all
     * clocks in the VCORE domain are stopped, the PLL, the MSI, the
     * HSI16 and the HSE oscillators are disabled. Some peripherals
     * with wakeup capability (I2C3 and LPUART) can switch on the
     * HSI16 to receive a frame, and switch off the HSI16 after
     * receiving the frame if it is not a wakeup frame. In this case
     * the HSI16 clock is propagated only to the peripheral requesting
     * it. SRAM1, SRAM2 and register contents are preserved.
     * The BOR is always available in Stop 2 mode. The consumption is
     * increased when thresholds higher than VBOR0 are used.
     *
     * Note: To enter Stop 2 mode, all EXTI Line pending bits (in
     * Pending register 1 (EXTI_PR1)), and the peripheral flags
     * generating wakeup interrupts must be cleared. Otherwise, the
     * Stop mode entry procedure is ignored and program execution
     * continues.
     */
    SYSTEM_SLEEPMODE_STOP2,

    /**
     * All I/O pins keep the same state as in Run mode.
     *
     * The Sleep mode is entered according Section : Entering low
     * power mode, when the SLEEPDEEP bit in the Cortex®-M4 System
     * Control register is clear.
     */
    SYSTEM_SLEEPMODE_SLEEP,
};
#endif

void pm_set(unsigned mode)
{
    int deep = 0;

/* I just copied it from stm32f1/2/4, but I suppose it would work for the
 * others... /KS */
#if defined(CPU_FAM_STM32F1) || defined(CPU_FAM_STM32F2) || \
    defined(CPU_FAM_STM32F4) || defined(CPU_FAM_STM32L0)
    switch (mode) {
        case STM32_PM_STANDBY:
            /* Set PDDS to enter standby mode on deepsleep and clear flags */
            PWR->CR |= (PWR_CR_PDDS | PWR_CR_CWUF | PWR_CR_CSBF);
            /* Enable WKUP pin to use for wakeup from standby mode */
#if defined(CPU_FAM_STM32L0)
            PWR->CSR |= (PWR_CSR_EWUP1 | PWR_CSR_EWUP2);
#if !defined(CPU_MODEL_STM32L053R8)
            /* STM32L053 only have 2 wake pins */
            PWR->CSR |= PWR_CSR_EWUP3;
#endif
#else
            PWR->CSR |= PWR_CSR_EWUP;
#endif
            /* Set SLEEPDEEP bit of system control block */
            deep = 1;
            break;
        case STM32_PM_STOP:
#if defined(CPU_FAM_STM32L0)
            /* Clear PDDS to enter stop mode on */
            /*
             * Regarding LPSDSR, it's up to the user to configure it :
             * 0: Voltage regulator on during Deepsleep/Sleep/Low-power run mode
             * 1: Voltage regulator in low-power mode during
             *    Deepsleep/Sleep/Low-power run mode
             */
            PWR->CR &= ~(PWR_CR_PDDS);
#else
            /* Clear PDDS and LPDS bits to enter stop mode on */
            /* deepsleep with voltage regulator on */
            PWR->CR &= ~(PWR_CR_PDDS | PWR_CR_LPDS);
            PWR->CR |= PM_STOP_CONFIG;
#endif
            /* Set SLEEPDEEP bit of system control block */
            deep = 1;
            break;
    }
#elif defined(CPU_FAM_STM32L4)
    periph_clk_en(APB1, RCC_APB1ENR1_PWREN);
    switch (mode) {

        case SYSTEM_SLEEPMODE_SHUTDOWN:
            /* optionally enable RTC & LSE */
            RCC->BDCR |= RCC_BDCR_RTCEN;
            RCC->BDCR |= RCC_BDCR_LSEON;
            /* LPMS = “1XX” in PWR_CR1 */
            PWR->CR1 |= PWR_CR1_LPMS_SHUTDOWN;
            deep = 1;
            break;

        case SYSTEM_SLEEPMODE_STANDBY:
            /* optionally enable RTC, LSI & LSE */
            RCC->BDCR |= RCC_BDCR_RTCEN;
            RCC->CSR |= RCC_CSR_LSION;
            RCC->BDCR |= RCC_BDCR_LSEON;
            /* LPMS = “011” in PWR_CR1 */
            PWR->CR1 &= ~(PWR_CR1_LPMS_SHUTDOWN);
            PWR->CR1 |= PWR_CR1_LPMS_STANDBY;

            PWR->CR3 |= PWR_CR3_RRS;
            deep = 1;
            break;

        case SYSTEM_SLEEPMODE_STOP2:
            /* optionally enable RTC, LSI & LSE */
            RCC->BDCR |= RCC_BDCR_RTCEN;
            RCC->CSR |= RCC_CSR_LSION;
            RCC->BDCR |= RCC_BDCR_LSEON;

            /* moreover an indepedent watchdog (IWDG) could be enabled */
            /* LPMS = “010” in PWR_CR1 */
            PWR->CR1 &= ~(PWR_CR1_LPMS_SHUTDOWN | PWR_CR1_LPMS_STOP1);
            PWR->CR1 |= PWR_CR1_LPMS_STOP2;
            deep = 1;
            break;

        case SYSTEM_SLEEPMODE_SLEEP:
            deep = 0;
            break;

        default:
            /* return immediately to prevent calling cortexm_sleep() */
            return;
    }
    periph_clk_dis(APB1, RCC_APB1ENR1_PWREN);
#else
    (void) mode;
#endif

    cortexm_sleep(deep);

#if defined(CPU_FAM_STM32F1) || defined(CPU_FAM_STM32F2) || \
    defined(CPU_FAM_STM32F4) || defined(CPU_FAM_STM32L0) || \
    defined(CPU_FAM_STM32L4)
    if (deep) {
        /* Re-init clock after STOP */
        stmclk_init_sysclk();
    }
#endif
}

#if defined(CPU_FAM_STM32F1) || defined(CPU_FAM_STM32F2) || \
    defined(CPU_FAM_STM32F4) || defined(CPU_FAM_STM32L0) || \
    defined(CPU_FAM_STM32L4)
void pm_off(void)
{
    irq_disable();
    pm_set(0);
}
#endif
