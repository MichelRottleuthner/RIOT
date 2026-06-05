/*
 * Copyright (C) 2020 HAW Hamburg <michel.rottleuthner@haw-hamburg.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */
/**
 * @ingroup         cpu_stm32_common
 * @{
 *
 * @file
 * @brief           STM32 specific definitions for the generic clock configuration module
 *
 *
 * @author          Michel Rottleuthner <michel.rottleuthner@haw-hamburg.de>
*/
#ifndef GCLK_STM32_COMMON_CONF_H
#define GCLK_STM32_COMMON_CONF_H

#include "gclk.h"
#include "gclk/generic_gate.h"    /* @todo: remove/replace (only needed for accessing the gate instances */
#include "gclk/generic_mux.h"     /* @todo: remove/replace (only needed for accessing the mux instances */
#include "gclk/generic_scaler.h"  /* @todo: remove/replace (only needed for accessing the mux instances */

#ifdef __cplusplus
extern "C" {
#endif

//// ####################### L0/L4 COMMON  vvv
///* scaler */
//extern const gclk_clk_scaler_ll_t gclk_stm32_ahb_scaler;
extern const gclk_clk_scaler_ll_t gclk_stm32_apb1_scaler;
extern const gclk_clk_scaler_ll_t gclk_stm32_apb2_scaler;
extern const gclk_clk_scaler_ll_t gclk_apb1_tim_mul_scaler;
extern const gclk_clk_scaler_ll_t gclk_apb2_tim_mul_scaler;
//extern const gclk_clk_scaler_ll_t gclk_stm32_ahb_div8_scaler;
//extern const gclk_clk_scaler_ll_t gclk_stm32_msirange_scaler;
//extern const gclk_clk_scaler_ll_t gclk_stm32_mco_div_scaler;
//
///* muxes */
//extern const gclk_mux_ll_t gclk_stm32_sysclk_mux;
//extern const gclk_mux_ll_t gclk_stm32_rtc_lcd_mux;
//extern const gclk_mux_ll_t gclk_stm32_mco_mux;
//extern const gclk_mux_ll_t gclk_stm32_i2c1_mux;
//extern const gclk_mux_ll_t gclk_stm32_i2c3_mux;
//extern const gclk_mux_ll_t gclk_stm32_lpuart1_mux;
extern const gclk_mux_ll_t gclk_stm32_usart2_mux;
extern const gclk_mux_ll_t gclk_stm32_usart1_mux;
//extern const gclk_mux_ll_t gclk_stm32_lptim1_mux;
//
///* gates */
//extern const gclk_generic_gate_t gclk_stm32_msi_gate;
//extern const gclk_generic_src_gate_t gclk_stm32_hsi16_gate;
//extern const gclk_generic_src_gate_t gclk_stm32_hse_gate;
//extern const gclk_generic_src_gate_t gclk_stm32_lse_gate;
//extern const gclk_generic_src_gate_t gclk_stm32_lsi_gate;
//extern const gclk_generic_src_gate_t gclk_stm32_msi_base_gate; /* is actually a virtual source, not really a gate */
//
//// ####################### L0/L4 COMMON  ^^^
//
//#if defined(CPU_FAM_STM32L0)
//extern const gclk_clk_scaler_ll_t gclk_stm32_pll_mul_scaler; /* effectively/logically the same as PLL_VCO scalers */
//extern const gclk_clk_scaler_ll_t gclk_stm32_pll_div_scaler; /* effectively/logically the same as PLL_P scalers */
//extern const gclk_clk_scaler_ll_t gclk_stm32_pll_usb_div_scaler;
//extern const gclk_generic_src_gate_t gclk_stm32_hsi48_gate;
//extern const gclk_clk_scaler_ll_t gclk_stm32_hse_div_scaler; /* could be merged with gclk_stm32_hse_div32_scaler on L4 */
//extern const gclk_mux_ll_t gclk_stm32_hsi48_mux;
//extern const gclk_mux_ll_t gclk_stm32_pll_src_mux;
//extern const gclk_clk_scaler_ll_t gclk_stm32_hsi16_div_scaler;
//#endif
//
//#if defined(CPU_FAM_STM32L4)
//extern const gclk_clk_scaler_ll_t gclk_stm32_pll_m_scaler;
//extern const gclk_clk_scaler_ll_t gclk_stm32_pll_p_scaler;
//extern const gclk_clk_scaler_ll_t gclk_stm32_pllsai1_p_scaler;
//extern const gclk_clk_scaler_ll_t gclk_stm32_pllsai2_p_scaler;
//extern const gclk_clk_scaler_ll_t gclk_stm32_pll_p_scaler;
//extern const gclk_clk_scaler_ll_t gclk_stm32_pllsai1_q_scaler;
//extern const gclk_clk_scaler_ll_t gclk_stm32_pll_q_scaler;
//extern const gclk_clk_scaler_ll_t gclk_stm32_pll_r_scaler;
//extern const gclk_clk_scaler_ll_t gclk_stm32_pll_q_scaler;
//extern const gclk_clk_scaler_ll_t gclk_stm32_pllsai1_r_scaler;
//extern const gclk_clk_scaler_ll_t gclk_stm32_pllsai2_r_scaler;
//extern const gclk_clk_scaler_ll_t gclk_stm32_pll_vco_scaler;
//extern const gclk_clk_scaler_ll_t gclk_stm32_pllsai1_vco_scaler;
//extern const gclk_clk_scaler_ll_t gclk_stm32_pllsai2_vco_scaler;
//extern const gclk_clk_scaler_ll_t gclk_stm32_hse_div32_scaler;
//extern const gclk_clk_scaler_ll_t gclk_stm32_msisrange_scaler;
//
///* muxes */
//extern const gclk_mux_ll_t gclk_stm32_msi_mux;
//extern const gclk_mux_ll_t gclk_stm32_usb_rng_sdmmc_mux;
//extern const gclk_mux_ll_t gclk_stm32_lsco_mux;
//extern const gclk_mux_ll_t gclk_stm32_pll_pre_div_mux; /* could be renamed to make it common */
//extern const gclk_mux_ll_t gclk_stm32_sai1_mux;
//extern const gclk_mux_ll_t gclk_stm32_sai2_mux;
//extern const gclk_mux_ll_t gclk_stm32_adc_mux;
//extern const gclk_mux_ll_t gclk_stm32_i2c2_mux;
//extern const gclk_mux_ll_t gclk_stm32_dfsdm1_mux;
//extern const gclk_mux_ll_t gclk_stm32_uart5_mux;
//extern const gclk_mux_ll_t gclk_stm32_uart4_mux;
extern const gclk_mux_ll_t gclk_stm32_usart3_mux;
//extern const gclk_mux_ll_t gclk_stm32_swpmi1_mux;
//extern const gclk_mux_ll_t gclk_stm32_lptim2_mux;
//
///* gates */
//extern const gclk_generic_gate_t gclk_stm32_sai1_ext_gate;
//extern const gclk_generic_gate_t gclk_stm32_sai2_ext_gate;
//extern const gclk_generic_gate_t gclk_stm32_tim5_gate;
//#endif /* defined(CPU_FAM_STM32L4) */

#ifdef __cplusplus
}
#endif

#endif /* GCLK_STM32_COMMON_CONF_H */

/**
 * @}
 */
