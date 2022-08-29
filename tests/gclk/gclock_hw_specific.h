/*
 * Copyright (C) 2020 HAW Hamburg
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     tests
 * @{
 *
 * @file
 * @brief       Hardware specific config and utils for the clock config test app
 *
 * @author      Michel Rottleuthner <michel.rottleuthner@haw-hamburg.de>
 *
 * @}
 */

#if defined(CPU_FAM_STM32L4) || defined(CPU_FAM_STM32L0)
/* on nucleo-l476rg and nucleo-l073rz the MCO (microcontroller clock output) pin is available as alternate function 0
   on PA8 */
#define MCO_PIN (GPIO_PIN(0,8))
#endif

#if defined(CPU_FAM_STM32L0) || defined(CPU_FAM_STM32L4)
#endif

#if defined(CPU_MODEL_EFM32PG12B500F1024GL125)
#elif defined(CPU_FAM_STM32L0)
/* this is the PLL_VCO scaler @todo rename!? */
extern const gclk_clk_scaler_ll_t gclk_stm32_pll_mul_scaler;
/* has the restriction of not being available at all (off) on range 3 (1.2V) */
extern const gclk_generic_src_gate_t gclk_stm32_hsi16_gate;
#elif defined(CPU_FAM_STM32L4)
#endif

#if defined(CPU_FAM_STM32L0)
const gclk_t *gclk_handle_flash_freq = &gclk_stm32_ahb_scaler.base;
#endif

