/*
 * Copyright (C) 2021 HAW Hamburg <michel.rottleuthner@haw-hamburg.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */
/**
 * @ingroup     sys_gclk
 *
 * @{
 *
 * @file
 * @brief       Configuration for the clock manager module
 *
 * @author      Michel Rottleuthner <michel.rottleuthner@haw-hamburg.de>
 */
#ifndef GCLK_MANAGER_CONF_H
#define GCLK_MANAGER_CONF_H

#include "gclk.h"
#include "gclk_manager.h"
#include "gclk/generic_gate.h"
#include "gclk/generic_mux.h"
#include "gclk/generic_scaler.h"

#ifdef __cplusplus
extern "C" {
#endif

/* These topology ids are obtained via the test application command 'topologies SYSCLK'.
 * The IDs therefore need to be updated in case the toppologies are updated or represented differently at some point. */
enum {
    /* 1 | [SYSCLK]-->[MSI]-->[MSIMUX]-->[MSIRANGE]-->[MSI_BASE] */
    STM32_L476RG_SYSCLK_TOPO_ID_MSI = 1,
    /* 6 | [SYSCLK]-->[PLL_R]-->[PLL_VCO]-->[PLL_M]-->[PLL_PREDIV_MUX]-->[MSI]-->[MSIMUX]-->[MSIRANGE]-->[MSI_BASE] */
    STM32_L476RG_SYSCLK_TOPO_ID_PLL_MSI = 6,
    /* 7 | [SYSCLK]-->[PLL_R]-->[PLL_VCO]-->[PLL_M]-->[PLL_PREDIV_MUX]-->[HSI16] */
    STM32_L476RG_SYSCLK_TOPO_ID_PLL_HSI = 7,
};

//
//    if (vscale_id == 2) {
//        /* lookup lowest WS value for vcore = 1.0 V (range2) */
//        if (ahb_freq <= 6000000) {
//            waitstates = FLASH_ACR_LATENCY_0WS;
//        } else if (ahb_freq <= 12000000) {
//            waitstates = FLASH_ACR_LATENCY_1WS;
//        } else if (ahb_freq <= 18000000) {
//            waitstates = FLASH_ACR_LATENCY_2WS;
//        } else if (ahb_freq <= 26000000) {
//            waitstates = FLASH_ACR_LATENCY_3WS;
//        }
//    } else if (vscale_id == 1) {
//        /* lookup lowest WS value for vcore = 1.2 V (range1) */
//        if (ahb_freq <= 16000000) {
//            waitstates = FLASH_ACR_LATENCY_0WS;
//        } else if (ahb_freq <= 32000000) {
//            waitstates = FLASH_ACR_LATENCY_1WS;
//        } else if (ahb_freq <= 48000000) {
//            waitstates = FLASH_ACR_LATENCY_2WS;
//        } else if (ahb_freq <= 64000000) {
//            waitstates = FLASH_ACR_LATENCY_3WS;
//        }
//    }

extern const unsigned int GCLK_PREFERRED_FREQ_CONF_CNT;
//extern gclk_t const *gclock_handle_for_core_freq;

/* TIM5 is used for xtimer on nucleo-l476rg, which uses conditionally multiplied APB1 clock */
extern const gclk_clk_scaler_ll_t gclk_apb1_tim_mul_scaler;
extern const gclk_clk_scaler_ll_t gclk_stm32_apb1_scaler;
extern const gclk_clk_scaler_ll_t gclk_stm32_usart2_mux;
extern const gclk_clk_scaler_ll_t gclk_stm32_ahb_scaler;
const gclk_t * const scaler_connected_to_xtimer = &gclk_apb1_tim_mul_scaler.base;
const gclk_t * const clock_connected_to_stdio_uart = &gclk_stm32_usart2_mux.base;
const gclk_t * const core_clock_instance = &gclk_stm32_ahb_scaler.base;

extern const gclk_generic_gate_t gclk_stm32_msi_gate;
extern const gclk_mux_ll_t gclk_stm32_sysclk_mux;
extern const gclk_t gclk_stm32_msi_base;
extern const gclk_clk_scaler_ll_t gclk_stm32_msirange_scaler;
extern const gclk_generic_gate_t gclk_stm32_hsi16_gate;
extern const gclk_generic_gate_t gclk_stm32_hse_gate;
extern const gclk_clk_scaler_ll_t gclk_stm32_pll_p_scaler;
extern const gclk_clk_scaler_ll_t gclk_stm32_pll_q_scaler;
extern const gclk_clk_scaler_ll_t gclk_stm32_pll_r_scaler;
extern const gclk_clk_scaler_ll_t gclk_stm32_pll_vco_scaler;
extern const gclk_clk_scaler_ll_t gclk_stm32_pll_m_scaler;
extern const gclk_mux_ll_t gclk_stm32_pll_pre_div_mux;
extern const gclk_clk_scaler_ll_t gclk_stm32_pllsai1_vco_scaler;
extern const gclk_clk_scaler_ll_t gclk_stm32_pllsai2_vco_scaler;
extern const gclk_clk_scaler_ll_t gclk_stm32_pllsai1_p_scaler;
extern const gclk_clk_scaler_ll_t gclk_stm32_pllsai1_r_scaler;
extern const gclk_clk_scaler_ll_t gclk_stm32_pllsai1_q_scaler;
extern const gclk_clk_scaler_ll_t gclk_stm32_pllsai2_p_scaler;
extern const gclk_clk_scaler_ll_t gclk_stm32_pllsai2_r_scaler;
extern const gclk_clk_scaler_ll_t gclk_stm32_ahb_scaler;
extern const gclk_clk_scaler_ll_t gclk_stm32_apb1_scaler;
extern const gclk_clk_scaler_ll_t gclk_stm32_msirange_scaler;
extern const gclk_mux_ll_t gclk_stm32_pll_pre_div_mux;

/**
 * @name    Frequency limit configuration
 *
 * All of these limits apply to gclk_stm32_ahb_scaler.
 *
 * @note See RM0351 Section 3.3.3 (read access latency) on page 97 (DocID024597 Rev 5)
 *       Possible alternative encoding as LUF for WS (integer calculation!):
 *       range1:  WS = (HCLK_MHz - 1) / 16
 *       range2:  WS = (HCLK_MHz - 1) / 7
 */
static const freq_conf_limit_t ahb_freq_vc_ws_limits[] = {
    { .freq_max =  6000000, .vc_idx_min = 0, .ws_min = 0 },
    { .freq_max = 12000000, .vc_idx_min = 0, .ws_min = 1 },
    { .freq_max = 16000000, .vc_idx_min = 1, .ws_min = 0 },
    { .freq_max = 18000000, .vc_idx_min = 0, .ws_min = 2 },
    { .freq_max = 26000000, .vc_idx_min = 0, .ws_min = 3 },
    { .freq_max = 32000000, .vc_idx_min = 1, .ws_min = 1 },
    { .freq_max = 48000000, .vc_idx_min = 1, .ws_min = 2 },
    { .freq_max = 64000000, .vc_idx_min = 1, .ws_min = 3 },
    { .freq_max = 80000000, .vc_idx_min = 1, .ws_min = 4 },
};

/* a list of frequencies that sit at preferrable values (good ratio between WS, vcore, and frequency).
 * - higher frequency minimizes static losses due to faster execution
 * - faster flash access minimizes static losses due to wasting less cycles for flash access
 * - lower voltage reduces dynamic as well as static losses
 * -> a good ratio therefore sits at point with maximum frequency for a particular flash wait state value.
 *
 *  Most values are just based on the constraint steps set up by above AHB limits.
 *  Exceptions to this rule:
 *   - 6MHz is dropped as PLL constraints make that infeasible (at 1:1 core freq to AHB ratio at least)
 *   - 8Mhz is added as slowest possible Frequency for PLL topologies
 *   - 4Mhz is added as highest possible freq that allows 0 WS and 0 VC operation (only with MSI topology)
 *  */
//static const uint32_t gclk_manager_preferred_freqs[] = { 4000000, 8000000, 12000000, 16000000, 18000000, 26000000, 32000000, 48000000, 64000000, 80000000};
static const uint32_t _prefered_freqs_with_pll[] = { 8000000, 12000000, 16000000, 18000000, 26000000, 32000000, 48000000, 64000000, 80000000};
//static const uint32_t _prefered_direct_scale_freqs[] = { 13333333, 26666666, 40000000, 53333333, 80000000 };
/* would be an alternative to the above list but is not able to reach max freq */
//static const uint32_t _prefered_direct_scale_freqs_even[] = { 6000000, 12000000, 24000000, 36000000, 48000000, 72000000 };

#define MAX_PREFERRED_FREQS_NUM (ARRAY_SIZE(_prefered_freqs_with_pll))

//gclk_stm32_pllsai1_p_scaler.base,
//gclk_stm32_pllsai2_p_scaler.base,
//gclk_stm32_pll_p_scaler.base,
static const freq_conf_limit_t pllx_p_freq_vc_ws_limits[] = {
    { .freq_max = 80000000, .vc_idx_min = 1, .ws_min = 0 }, //GCLK_WS_NOSPEC
    { .freq_max = 26000000, .vc_idx_min = 0, .ws_min = 0 }, //GCLK_WS_NOSPEC
};

//gclk_stm32_pll_vco_scaler.base,
//gclk_stm32_pllsai1_vco_scaler.base,
//gclk_stm32_pllsai2_vco_scaler.base,
static const freq_conf_limit_t pllx_vco_freq_vc_ws_limits[] = {
    { .freq_max = 344000000, .vc_idx_min = 1, .ws_min = 0 }, //GCLK_WS_NOSPEC
    { .freq_max = 128000000, .vc_idx_min = 0, .ws_min = 0 }, //GCLK_WS_NOSPEC
};

//gclk_stm32_hse_gate.base
static const freq_conf_limit_t hse_freq_vc_ws_limits[] = {
    { .freq_max = 48000000, .vc_idx_min = 1, .ws_min = 0 }, //GCLK_WS_NOSPEC
    { .freq_max = 26000000, .vc_idx_min = 0, .ws_min = 0 }, //GCLK_WS_NOSPEC
};

//gclk_stm32_msi_gate.base instead of gclk_stm32_msirange_scaler.base,
// This way the model correctly differentiates between the scale factor being configured and the clock being gated.
// I.e., if the clock is gated it does not put up a limit as long as it is gated.
static const freq_conf_limit_t msi_freq_vc_ws_limits[] = {
    { .freq_max = 48000000, .vc_idx_min = 1, .ws_min = 0 }, //GCLK_WS_NOSPEC
    { .freq_max = 24000000, .vc_idx_min = 0, .ws_min = 0 }, //GCLK_WS_NOSPEC
};

extern const gclk_clk_scaler_ll_t gclk_stm32_ahb_scaler;

/* for each unique clock instance that has associated frequency configuration limits,
 * this points to the limit list */
static const clock_freq_conf_limits_t gclk_freq_conf_limits[] = {
    { .clk = &gclk_stm32_ahb_scaler.base,
      .limits = ahb_freq_vc_ws_limits,
      .len = ARRAY_SIZE(ahb_freq_vc_ws_limits)
    },
    { .clk = &gclk_stm32_pllsai1_p_scaler.base,
      .limits = pllx_p_freq_vc_ws_limits,
      .len = ARRAY_SIZE(pllx_p_freq_vc_ws_limits)
    },
    { .clk = &gclk_stm32_pllsai2_p_scaler.base,
      .limits = pllx_p_freq_vc_ws_limits,
      .len = ARRAY_SIZE(pllx_p_freq_vc_ws_limits)
    },
    { .clk = &gclk_stm32_pll_p_scaler.base,
      .limits = pllx_p_freq_vc_ws_limits,
      .len = ARRAY_SIZE(pllx_p_freq_vc_ws_limits)
    },
    { .clk = &gclk_stm32_pll_vco_scaler.base,
      .limits = pllx_vco_freq_vc_ws_limits,
      .len = ARRAY_SIZE(pllx_vco_freq_vc_ws_limits)
    },
    { .clk = &gclk_stm32_pllsai1_vco_scaler.base,
      .limits = pllx_vco_freq_vc_ws_limits,
      .len = ARRAY_SIZE(pllx_vco_freq_vc_ws_limits)
    },
    { .clk = &gclk_stm32_pllsai2_vco_scaler.base,
      .limits = pllx_vco_freq_vc_ws_limits,
      .len = ARRAY_SIZE(pllx_vco_freq_vc_ws_limits)
    },
    { .clk = &gclk_stm32_hse_gate.base,
      .limits = hse_freq_vc_ws_limits,
      .len = ARRAY_SIZE(hse_freq_vc_ws_limits)
    },
    { .clk = &gclk_stm32_msi_gate.base,
      .limits = msi_freq_vc_ws_limits,
      .len = ARRAY_SIZE(msi_freq_vc_ws_limits)
    },
};
#define GCLK_FREQ_LIMIT_CLKS_NUMOF   ARRAY_SIZE(gclk_freq_conf_limits)

extern const gclk_mux_ll_t gclk_stm32_sysclk_mux;
const gclk_t* gclk_core_clock_handle = &gclk_stm32_sysclk_mux.base;

gclk_scale_setting_t scale_settings[] = {
    {
      .output_clk = &gclk_stm32_sysclk_mux.base,
      .scale_clk = &gclk_stm32_msirange_scaler.base,
      .topology_id = STM32_L476RG_SYSCLK_TOPO_ID_MSI,
      .default_freqs = NULL,
      .default_freqs_cnt = 0,
      /* since there is no additional scaling happening inbetween the scaler and the core handle (only muxing)
       * the approach that applies here is SCALE_DIRECT instead of SCALE_UPTREE_RELATIVE */
      .approach = SCALE_DIRECT, },
    {
      .output_clk = &gclk_stm32_sysclk_mux.base,
      .scale_clk = &gclk_stm32_msirange_scaler.base,
      .topology_id = STM32_L476RG_SYSCLK_TOPO_ID_PLL_MSI,
      .default_freqs = NULL,
      .default_freqs_cnt = 0,
      //.default_freqs = _prefered_direct_scale_freqs,
      //.default_freqs_cnt = ARRAY_SIZE(_prefered_direct_scale_freqs),
      .approach = SCALE_UPTREE_RELATIVE,
    },
    /* in below cases scaling is not done via a single scaler but instead updating multiple clock instances by automatically
     * finding a valid intermediate topology transition */
    {
      .output_clk = &gclk_stm32_sysclk_mux.base,
      .topology_id = STM32_L476RG_SYSCLK_TOPO_ID_PLL_MSI,
      .default_freqs = _prefered_freqs_with_pll,
      .default_freqs_cnt = ARRAY_SIZE(_prefered_freqs_with_pll),
      .approach = SCALE_INTERMEDIATE_TOPO_AUTO,
    },
    {
      .output_clk = &gclk_stm32_sysclk_mux.base,
      .topology_id = STM32_L476RG_SYSCLK_TOPO_ID_PLL_HSI,
      .default_freqs = _prefered_freqs_with_pll,
      .default_freqs_cnt = ARRAY_SIZE(_prefered_freqs_with_pll),
      .approach = SCALE_INTERMEDIATE_TOPO_AUTO,
    },
};

#define SCALE_SETTINGS_NUMOF       (ARRAY_SIZE(scale_settings))

/* Defines a list of clock sources that are allowed to be used as the originating source to drive the systems core clock.
 * A clock listed here is not neccessarily able to drive the core clock directly. Instead, this list just defines the roots
 * of possible topologies that may drive the core clock but without listing all possible topologies explicitly.
 * If a given topology and clock configuration is appliccable is of course still subject to certain runtime conditions like
 * configuration constraints and peripheral requirements */
static const gclk_t* core_clock_sources[] = {
&gclk_stm32_hsi16_gate.base,
&gclk_stm32_msi_base,
};

/* Topology transitions can be described as a state machine.
 *
 * the individual steps to setup a specific topoB from topoA could then be loaded by calling
 * steps = setup_steps(clock, topoA_id, topoB_id);
 * for step in steps:
 *  s.execute();
 *
 * # ID | Topology
 * # ---|-------------------
 * #  0 | [SYSCLK]-->[MSI]-->[MSIMUX]-->[MSISRANGE]-->[MSI_BASE]
 * #  1 | [SYSCLK]-->[MSI]-->[MSIMUX]-->[MSIRANGE]-->[MSI_BASE]
 * #  2 | [SYSCLK]-->[HSI16]
 * #  3 | [SYSCLK]-->[HSE]
 * #  4 | [SYSCLK]-->[PLL_R]-->[PLL_VCO]-->[PLL_M]-->[PLL_PREDIV_MUX]-->[NULL]
 * #  5 | [SYSCLK]-->[PLL_R]-->[PLL_VCO]-->[PLL_M]-->[PLL_PREDIV_MUX]-->[MSI]-->[MSIMUX]-->[MSISRANGE]-->[MSI_BASE]
 * #  6 | [SYSCLK]-->[PLL_R]-->[PLL_VCO]-->[PLL_M]-->[PLL_PREDIV_MUX]-->[MSI]-->[MSIMUX]-->[MSIRANGE]-->[MSI_BASE]
 * #  7 | [SYSCLK]-->[PLL_R]-->[PLL_VCO]-->[PLL_M]-->[PLL_PREDIV_MUX]-->[HSI16]
 * #  8 | [SYSCLK]-->[PLL_R]-->[PLL_VCO]-->[PLL_M]-->[PLL_PREDIV_MUX]-->[HSE]
 *
 * Assume the above topologies are available for SYSCLK.
 * Topologies that can actually be used for operation would be 1, 2, 6, 7
 * (3 in case a crystal is present).
 *
 * A naive modelling would result in a table (1,2,6,7) x (1,2,6,7) where each cell holds required steps to do the transition.
 * In this particular case we could apply some simplifications:
 *  Transitions that are always possible (in terms of the transition not requiring intermediate steps apart from enabling clocks):
 *  ? -> 1; ? -> 2; ? -> 3
 *
 *  The following transitions require the target topology to be setup before switching the core frequency over.
 *  ? -> 6; ? -> 7; ? -> 8
 *
 * Abstract sequences:
 * A: update leaf parent (leaf should be same in target and start topology)
 * B: set target-topo-config downtree (may be factor or parent)
 * C: disable partial target-topo-config downtree; (params: from)
 *
 * 1 -> 2: enable HSI16, set_parent SYSCLK HSI16 (abstract B,A)
 * 1 -> 6: disable PLL_VCO (all of them); config PLL nodes; config MSIRANGE; enable PLL_VCO; set_parent SYSCLK PLL_R;
 *   could be benefitial to provide an alternative for the above in cases where MSIRANGE is very low (avoid temporary slowdown)
 * 1 -> 7: disable PLL_VCO (all of them); config PLL nodes; enable HSI16; enable PLL_VCO; set_parent SYSCLK PLL_R;
 * 2 -> 1: configure MSIRANGE; enable root downtree; set_parent SYSCLK MSI;
 * 2 -> 6: disable PLL_VCO (all of them); config root downtree PLL_R; enable PLL_VCO; set_parent SYSCLK PLL_R;
 * 2 -> 7: disable PLL_VCO (all of them); config root downtree PLL_R; enable PLL_VCO; set_parent SYSCLK PLL_R;
 * 6 -> 1: set_parent SYSCLK MSI; disable PLL_VCO; config MSIRANGE;
 * 6 -> 2: same as (1 -> 2)
 * 6 -> 7: 6 -> 1 -> 7; or 6 -> 2 -> 7;
 * 7 -> 1: config MSIRANGE; enable MSI; set_parent SYSCLK MSI; or 7 -> 2 -> 1;  (abstract B,A)
 * 7 -> 2: same as (1 -> 2)
 * 7 -> 6: 7 -> 2 -> 6; or 7 -> 1 -> 6;
 *
 * *General note*
 * There are different abstract cases how a new topology config can be set up:
 * NOT_USE_DURING_SETUP/INTERMEDIATE_TOPOLOGY_STEP: the target config can only be setup while not used
 *
 * Types how scaling to a specific frequency can be achieved:
 * ONTHEFLY_SINGLE: the core freq only depends on a single scaler that can be changed on the fly
 * ONTHEFLY_MULTI: the core freq depends on multiple clock nodes where all can be changed on the fly
 * PREPAREDSETUP: at least one clock in the topology can not be changed on the fly so the whole topology must be configured before switching to it
 * */


const gclk_manager_sequence_step_t sysclk_1_2[] = {
    /* this step is very simple and can always be executed */
    { .op = CLK_ENABLE,     .clk = &gclk_stm32_hsi16_gate.base },
    { .op = CLK_SET_PARENT, .clk = &gclk_stm32_sysclk_mux.base,   .clk_arg = &gclk_stm32_hsi16_gate.base },
    { .op = CLK_DISABLE,    .clk = &gclk_stm32_msi_gate.base },
    /* TODO: optional: disable old topo (if not needed) */
};

const gclk_manager_sequence_step_t sysclk_1_6[] = {
    { .op = CLK_DISABLE,       .clk = &gclk_stm32_pll_vco_scaler.base },
    { .op = CLK_CONFIG_TARGET, .clk = &gclk_stm32_pll_r_scaler.base },
    { .op = CLK_CONFIG_TARGET, .clk = &gclk_stm32_pll_vco_scaler.base },
    { .op = CLK_CONFIG_TARGET, .clk = &gclk_stm32_pll_m_scaler.base },
    { .op = CLK_CONFIG_TARGET, .clk = &gclk_stm32_pll_pre_div_mux.base },
    { .op = CLK_CONFIG_TARGET, .clk = &gclk_stm32_msirange_scaler.base },
    { .op = CLK_ENABLE,        .clk = &gclk_stm32_pll_vco_scaler.base },
    { .op = CLK_SET_PARENT,    .clk = &gclk_stm32_sysclk_mux.base,   .clk_arg = &gclk_stm32_pll_r_scaler.base },
    /* old topo is part of new topo so there is nothing to disable */
};

const gclk_manager_sequence_step_t sysclk_1_7[] = {
    { .op = CLK_DISABLE,       .clk = &gclk_stm32_pll_vco_scaler.base },
    { .op = CLK_CONFIG_TARGET, .clk = &gclk_stm32_pll_r_scaler.base },
    { .op = CLK_CONFIG_TARGET, .clk = &gclk_stm32_pll_vco_scaler.base },
    { .op = CLK_CONFIG_TARGET, .clk = &gclk_stm32_pll_m_scaler.base },
    { .op = CLK_CONFIG_TARGET, .clk = &gclk_stm32_pll_pre_div_mux.base },
    { .op = CLK_ENABLE,        .clk = &gclk_stm32_hsi16_gate.base },
    { .op = CLK_ENABLE,        .clk = &gclk_stm32_pll_vco_scaler.base },
    { .op = CLK_SET_PARENT,    .clk = &gclk_stm32_sysclk_mux.base,   .clk_arg = &gclk_stm32_pll_r_scaler.base },
    { .op = CLK_DISABLE,       .clk = &gclk_stm32_msi_gate.base },
};

/* TODO: for operations like the below last disable op it would make sense to differenciate between optional and mandatory
 *       operations to give a reasonable default on what to do, avoid doing it if not feasible, but still be able to tell
 *       if the overall transition really is conflicting (infeasible( or just some optional steps of it.
 *       This would then also interact with the envisioned mechanism to allocate/free clock resources to only perform optional
 *       disables if the clock is otherwise not needed */
const gclk_manager_sequence_step_t sysclk_2_1[] = {
    { .op = CLK_CONFIG_TARGET, .clk = &gclk_stm32_msirange_scaler.base },
    { .op = CLK_ENABLE,        .clk = &gclk_stm32_msi_gate.base },
    { .op = CLK_SET_PARENT,    .clk = &gclk_stm32_sysclk_mux.base,   .clk_arg = &gclk_stm32_msi_gate.base },
    /* TODO: runtime check if disable is in line with other constraints */
    { .op = CLK_DISABLE,       .clk = &gclk_stm32_hsi16_gate.base },
};

/* special about this topology is that the target topology is a subset of the current toology so we know it is already running
 * and it therefore must only be reconfigured not enabled */
const gclk_manager_sequence_step_t sysclk_6_1[] = {
    { .op = CLK_SET_PARENT,    .clk = &gclk_stm32_sysclk_mux.base,   .clk_arg = &gclk_stm32_msi_gate.base },
    { .op = CLK_DISABLE,       .clk = &gclk_stm32_pll_vco_scaler.base },
    { .op = CLK_SET_PARENT,    .clk = &gclk_stm32_pll_pre_div_mux.base, .clk_arg = NULL },
    { .op = CLK_CONFIG_TARGET, .clk = &gclk_stm32_msirange_scaler.base },
};

const gclk_manager_sequence_step_t sysclk_7_1[] = {
    { .op = CLK_CONFIG_TARGET, .clk = &gclk_stm32_msirange_scaler.base },
    { .op = CLK_ENABLE,        .clk = &gclk_stm32_msi_gate.base },
    { .op = CLK_SET_PARENT,    .clk = &gclk_stm32_sysclk_mux.base,   .clk_arg = &gclk_stm32_msi_gate.base },
    { .op = CLK_DISABLE,       .clk = &gclk_stm32_pll_vco_scaler.base },
    /* TODO: runtime check if disable is in line with other constraints */
    { .op = CLK_DISABLE,       .clk = &gclk_stm32_hsi16_gate.base },
};

/* TODO: also add explicit transition descriptors for same-to-same-topo transitions? */
const gclk_manager_topo_switch_desc_t core_clk_topo_switch_descs[] = {
    { .src_topo_id = 1, .dst_topo_id = 2, .steps = sysclk_1_2, .step_cnt= ARRAY_SIZE(sysclk_1_2) },
    { .src_topo_id = 1, .dst_topo_id = 6, .steps = sysclk_1_6, .step_cnt= ARRAY_SIZE(sysclk_1_6) },
    { .src_topo_id = 1, .dst_topo_id = 7, .steps = sysclk_1_7, .step_cnt= ARRAY_SIZE(sysclk_1_7) },
    { .src_topo_id = 2, .dst_topo_id = 1, .steps = sysclk_2_1, .step_cnt= ARRAY_SIZE(sysclk_2_1) },
    { .src_topo_id = 6, .dst_topo_id = 1, .steps = sysclk_6_1, .step_cnt= ARRAY_SIZE(sysclk_6_1) },
    { .src_topo_id = 7, .dst_topo_id = 1, .steps = sysclk_7_1, .step_cnt= ARRAY_SIZE(sysclk_7_1) },
};

#define CORE_CLOCK_TOPO_SWITCH_DESC_NUMOF   (ARRAY_SIZE(core_clk_topo_switch_descs))

/* these constraints universally apply for this platform at all times */
const gclk_freq_constraint_t global_clock_constraints[] = {
    { .type = GCLK_ENSURE_MIN_FREQ, .clk = &gclk_stm32_pll_m_scaler.base, .freq = 4000000 },
    { .type = GCLK_ENSURE_MAX_FREQ, .clk = &gclk_stm32_pll_m_scaler.base, .freq = 16000000 },
    { .type = GCLK_ENSURE_MIN_FREQ, .clk = &gclk_stm32_pll_vco_scaler.base, .freq = 64000000 },
    { .type = GCLK_ENSURE_MAX_FREQ, .clk = &gclk_stm32_pll_vco_scaler.base, .freq = 344000000 },

    { .type = GCLK_ENSURE_MIN_FREQ, .clk = &gclk_stm32_pllsai1_vco_scaler.base, .freq = 64000000 },
    { .type = GCLK_ENSURE_MAX_FREQ, .clk = &gclk_stm32_pllsai1_vco_scaler.base, .freq = 344000000 },

    { .type = GCLK_ENSURE_MIN_FREQ, .clk = &gclk_stm32_pllsai2_vco_scaler.base, .freq = 64000000 },
    { .type = GCLK_ENSURE_MAX_FREQ, .clk = &gclk_stm32_pllsai2_vco_scaler.base, .freq = 344000000 },

    { .type = GCLK_ENSURE_MAX_FREQ, .clk = &gclk_stm32_pll_p_scaler.base, .freq = 80000000 },
    { .type = GCLK_ENSURE_MAX_FREQ, .clk = &gclk_stm32_pll_q_scaler.base, .freq = 80000000 },
    { .type = GCLK_ENSURE_MAX_FREQ, .clk = &gclk_stm32_pll_r_scaler.base, .freq = 80000000 },

    { .type = GCLK_ENSURE_MAX_FREQ, .clk = &gclk_stm32_pllsai1_p_scaler.base, .freq = 80000000 },
    { .type = GCLK_ENSURE_MAX_FREQ, .clk = &gclk_stm32_pllsai1_q_scaler.base, .freq = 80000000 },
    { .type = GCLK_ENSURE_MAX_FREQ, .clk = &gclk_stm32_pllsai1_r_scaler.base, .freq = 80000000 },

    { .type = GCLK_ENSURE_MAX_FREQ, .clk = &gclk_stm32_pllsai2_p_scaler.base, .freq = 80000000 },
    { .type = GCLK_ENSURE_MAX_FREQ, .clk = &gclk_stm32_pllsai2_r_scaler.base, .freq = 80000000 },
};

/* Power model data calculated from FCAE / FCAM experiment measurements.   
 * The FCAE/FCAM experiments collect power consumption data for all frequency
 * configurations for each topology and a big set of possible frequencies.
 * 'C_SYSCLK' is expected to be highly dependent on the reference workload of the
 * experiment. I.e. another workloads (with different switching activity) will result
 * in a different value. But since the model is used to calculate and compare the
 * power consumption between different configurations with the same, yet arbitrary,
 * workload, its absolute value does not matter. Its value will simply offset the
 * value of each power calculation in the same way. Yet, incorporating this value
 * has the benefit that the model can be easily checked for correctness by comparing
 * against absolute power values obtained from physical measurmements.
 *
 * 'VDD' and 'P_static' are properties of the whole system (and its configuration)
 *  instead of a property of a single particular clock node (and its config).
 * model = {
 *      'VDD' : 1.2,
 *      'P_MSIEN' : 0.001629912161,
 *      'P_HSI16EN' : 0.002356888648,
 *      'P_PLLEN' : 0.0001811811504,
 *      'C_HSI16' : 0,
 *      'C_MSI' : 1.175873657e-11,
 *      'C_PLL_PREDIV_MUX' : 1.004693958e-12,
 *      'C_PLL_M' : 4.183176681e-12,
 *      'C_PLL_VCO' : 4.880479284e-12,
 *      'C_PLL_R' : 1.395348936e-12,
 *      'C_SYSCLK' : 4.298355483e-10,
 *      'P_static' : 0.001060020757 }
 **/
const gclk_manager_power_properties_t clock_power_model[] = {
    // 'P_HSI16EN' : 0.002356888648, 'C_HSI16' : 0,
    { .clk = &gclk_stm32_hsi16_gate.base, .P_en_nW = 2356889, .C_fF = 0 },
    // 'P_MSIEN' : 0.001629912161, 'C_MSI' : 1.175873657e-11,
    { .clk = &gclk_stm32_msi_gate.base, .P_en_nW = 1629912, .C_fF = 11759 },
    // 'P_PLLEN' : 0.0001811811504, 'C_PLL_VCO' : 4.880479284e-12,
    { .clk = &gclk_stm32_pll_vco_scaler.base, .P_en_nW = 181181, .C_fF = 4880 },
    // 'C_PLL_PREDIV_MUX' : 1.004693958e-12,
    { .clk = &gclk_stm32_pll_pre_div_mux.base, .P_en_nW = 0, .C_fF = 1005 },
    // 'C_PLL_M' : 4.183176681e-12,
    { .clk = &gclk_stm32_pll_m_scaler.base, .P_en_nW = 0, .C_fF = 4183 },
    // 'C_PLL_R' : 1.395348936e-12,
    { .clk = &gclk_stm32_pll_r_scaler.base, .P_en_nW = 0, .C_fF = 1395 },
    // Its value can in fact be ignored when considering the problem
    // of finding the most efficient factors for a given frequency and workload.
    // 'C_SYSCLK' : 4.298355483e-10,
    { .clk = &gclk_stm32_sysclk_mux.base, .P_en_nW = 0, .C_fF = 429836 },
};

#define GCLK_MANAGER_CONF_SYS_PSTATIC_NW (1060021)

#define GLOBAL_CLOCK_POWER_MODEL_PROPERTIES_NUMOF (ARRAY_SIZE(clock_power_model))

#define GLOBAL_CLOCK_POWER_MODEL_AVAILABLE (1)

#define GLOBAL_CLOCK_CONSTRAINTS_NUMOF (ARRAY_SIZE(global_clock_constraints))

#define CORE_CLOCK_SOURCES_NUMOF   (ARRAY_SIZE(core_clock_sources))

/* Defines the maximum number of discrete frequency steps that are used for dynamic frequency scaling
 * and PU metric assessment */
#define MAX_DFS_FREQ_VALUES_NUM (MAX_PREFERRED_FREQS_NUM)


#define DFS_CYCLER_MIN_FREQ (8000000)
//#define DFS_CYCLER_MIN_FREQ (100000)
#define DFS_CYCLER_MAX_FREQ (80000000)

gclk_clock_change_notify_list_t stdio_nl;
gclk_clock_change_notify_list_t timer_reinit_nl;
gclk_clock_change_notify_list_t ahb_nl;

extern bool apb_clk_cached[3];

void _apb_cache_invalidate_cb(const gclk_t* altered_clk, const gclk_t* affected_clk, uint32_t f_old, uint32_t f_new, bool post_change) {
    (void)altered_clk;
    (void)affected_clk;
    (void)f_old;
    (void)f_new;
    if (post_change) {
       for (unsigned i = 0; i < ARRAY_SIZE(apb_clk_cached); i++) {
           apb_clk_cached[i] = false;
       }
    }
}

static inline int gclk_manager_platform_init(void) {
    gclk_manager_register_clk_change_cb(&gclk_apb1_tim_mul_scaler.base, &timer_reinit_nl, gclk_manager_default_timer_reinit_cb);
    gclk_manager_register_clk_change_cb(&gclk_stm32_usart2_mux.base, &stdio_nl, gclk_manager_default_stdio_reinit_cb);
    gclk_manager_register_clk_change_cb(&gclk_stm32_ahb_scaler.base, &ahb_nl, _apb_cache_invalidate_cb);

    /* configure APB1 and APB2 to highest possible freq to allow slower core frequencies at same bus frequency.
     * This avoids feeding peripherals (such as high speed timers) with very low frequencies
     * when scaling down the core clock to single digit MHz or even sub-MHz range.
     * A future improvement would just allow handling flexibly clocked timers seamlessly,
     * which RIOT does not currently.
     * NOTE: this MUST be called after setting up the clk change callbacks (see above)
     * to ensure the necessary reconfigurations in the peripheral drivers are triggered */
    gclk_manager_set_factor(gclk_get_clk_by_name("APB1"), 1);
    gclk_manager_set_factor(gclk_get_clk_by_name("APB2"), 1);

    return 0;
}

#ifdef __cplusplus
}
#endif

#endif /* GCLK_MANAGER_CONF_H */
/** @} */
