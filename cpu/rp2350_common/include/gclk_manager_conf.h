/*
 * Copyright (C) 2026 HAW Hamburg <michel.rottleuthner@haw-hamburg.de>
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
//#include "gclk_efm32_types.h"
#include "gclk/generic_gate.h"
#include "gclk/generic_scaler.h"
#include "gclk/generic_mux.h"
#include "periph/core_voltage.h"

#ifdef __cplusplus
extern "C" {
#endif

extern const gclk_t * const core_clock_instance;
extern const unsigned int GCLK_PREFERRED_FREQ_CONF_CNT;
extern gclk_t const *gclock_handle_for_core_freq;
extern const unsigned int DVFS_CONFS_CNT;

/**
 * @name    Frequency limit configuration
 * All of these limits apply to the system clock.
 */
static const freq_conf_limit_t sys_freq_vc_ws_limits[] = {
    { .freq_max =  150000000, .vc_idx_min = 11, .ws_min = 0 },
};

enum {
    RP2350_XX_TOPO_ID_HFXO_DPLL = 0,
};

/* TODO: there are transitions that actually works the same way for multiple source-topologies
 * Therefore topology should actually be some kind of "topologies list" or we need a separate setting entry
 * for each applicable source+topology */
gclk_scale_setting_t scale_settings[] = {
};

#define SCALE_SETTINGS_NUMOF       (ARRAY_SIZE(scale_settings))

extern const gclk_clk_scaler_ll_t gclk_rp2350_pll_sys_scaler;
extern const gclk_clk_scaler_ll_t gclk_rp2350_rosc_scaler;
extern const gclk_generic_gate_t gclk_rp2350_rosc_gate;
extern const gclk_generic_gate_t gclk_rp2350_xosc_gate;
extern const gclk_t gclk_rp2350_gpin0_src;
extern const gclk_t gclk_rp2350_gpin1_src;

extern const gclk_generic_gate_t gclk_rp2350_clk_sys_gate;
extern const gclk_mux_ll_t gclk_rp2350_clk_sys_auxsrc_mux;
extern const gclk_mux_ll_t gclk_rp2350_pll_sys_bypass_mux;
extern const gclk_mux_ll_t gclk_rp2350_clk_peri_auxsrc_mux;

extern const gclk_generic_gate_t gclk_rp2350_pll_sys_vco_gate;
extern const gclk_clk_scaler_ll_t gclk_rp2350_pll_sys_postdiv1_scaler;
extern const gclk_clk_scaler_ll_t gclk_rp2350_pll_sys_postdiv2_scaler;

extern const gclk_clk_scaler_ll_t gclk_rp2350_pll_sys_refdiv_scaler;
extern const gclk_clk_scaler_ll_t gclk_rp2350_pll_usb_refdiv_scaler;
extern const gclk_clk_scaler_ll_t gclk_rp2350_pll_usb_fbdiv_scaler;
extern const gclk_clk_scaler_ll_t gclk_rp2350_pll_sys_fbdiv_scaler;
extern const gclk_mux_ll_t gclk_rp2350_clk_sys_src_mux;
extern const gclk_clk_scaler_ll_t gclk_rp2350_clk_sys_div_scaler;
extern const gclk_generic_gate_t gclk_rp2350_clk_peri_gate;

const gclk_t* gclk_core_clock_handle = &gclk_rp2350_clk_sys_gate.base;

/*
 *
 * ID | Topology
 * ---|-------------------
 *  0 | [CLK_SYS]-->[CLK_SYS_SRC]-->[CLK_REF_SRC]-->[CLK_REF_AUXSRC]-->[PLL_USB_BYPASS]-->[PLL_USB_POSTDIV2]-->[PLL_USB_POSTDIV1]-->[PLL_USB_VCO]-->[PLL_USB_FBDIV]-->[PLL_USB_REFDIV]-->[XOSC]
 *  1 | [CLK_SYS]-->[CLK_SYS_SRC]-->[CLK_REF_SRC]-->[CLK_REF_AUXSRC]-->[PLL_USB_BYPASS]-->[XOSC]
 *  ### default topology in RIOT vvv
 *  2 | [CLK_SYS]-->[CLK_SYS_SRC]-->[CLK_SYS_AUXSRC]-->[PLL_SYS_BYPASS]-->[PLL_SYS_POSTDIV2]-->[PLL_SYS_POSTDIV1]-->[PLL_SYS_VCO]-->[PLL_SYS_FBDIV]-->[PLL_SYS_REFDIV]-->[XOSC]
 *  ### default topology in RIOT ^^^
 *  3 | [CLK_SYS]-->[CLK_SYS_SRC]-->[CLK_SYS_AUXSRC]-->[PLL_SYS_BYPASS]-->[XOSC]
 *  4 | [CLK_SYS]-->[CLK_SYS_SRC]-->[CLK_SYS_AUXSRC]-->[PLL_USB_BYPASS]-->[PLL_USB_POSTDIV2]-->[PLL_USB_POSTDIV1]-->[PLL_USB_VCO]-->[PLL_USB_FBDIV]-->[PLL_USB_REFDIV]-->[XOSC]
 *  5 | [CLK_SYS]-->[CLK_SYS_SRC]-->[CLK_SYS_AUXSRC]-->[PLL_USB_BYPASS]-->[XOSC]
 *  6 | [CLK_SYS]-->[CLK_SYS_SRC]-->[CLK_SYS_AUXSRC]-->[ROSC]-->[ROSC_BASE]
 *  7 | [CLK_SYS]-->[CLK_SYS_SRC]-->[CLK_SYS_AUXSRC]-->[XOSC]
 *  8 | [CLK_SYS]-->[CLK_SYS_SRC]-->[CLK_SYS_AUXSRC]-->[GPIN0]
 *  9 | [CLK_SYS]-->[CLK_SYS_SRC]-->[CLK_SYS_AUXSRC]-->[GPIN0]
 */
enum {
    RP2350_TOPOID_CLK_SYS_REF_XOSC_PLL_USB   = 0,
    RP2350_TOPOID_CLK_SYS_REF_PLLUSBBYP_XOSC = 1,
    RP2350_TOPOID_CLK_SYS_AUX_PLLSYS_XOSC    = 2,
    RP2350_TOPOID_CLK_SYS_AUX_PLLSYSBYP_XOSC = 3,
    RP2350_TOPOID_CLK_SYS_AUX_PLLUSB_XOSC    = 4,
    RP2350_TOPOID_CLK_SYS_AUX_PLLUSBBYP_XOSC = 5,
    RP2350_TOPOID_CLK_SYS_AUX_ROSC           = 6,
    RP2350_TOPOID_CLK_SYS_AUX_XOSC           = 7,
    RP2350_TOPOID_CLK_SYS_AUX_GPIN0          = 8,
    RP2350_TOPOID_CLK_SYS_AUX_GPIN1          = 9,
};

const gclk_manager_sequence_step_t clk_sys__aux_pllsys__aux_rosc[] = {
    { .op = CLK_CONFIG_TARGET, .clk = &gclk_rp2350_rosc_scaler.base },
    { .op = CLK_ENABLE,        .clk = &gclk_rp2350_rosc_gate.base },
    { .op = CLK_SET_PARENT,    .clk = &gclk_rp2350_clk_sys_auxsrc_mux.base,   .clk_arg = &gclk_rp2350_rosc_scaler.base },
    /* TODO: optional: disable old topo (if not needed) */
};

const gclk_manager_sequence_step_t clk_sys__aux_pllsys__aux_xosc[] = {
    { .op = CLK_SET_PARENT,    .clk = &gclk_rp2350_clk_sys_auxsrc_mux.base,   .clk_arg = &gclk_rp2350_xosc_gate.base },
    /* TODO: optional: disable old topo (if not needed) */
};

const gclk_manager_sequence_step_t clk_sys__aux_xosc__aux_pllsys[] = {
    /* make sure the PLL is off before adjusting internal values */
    { .op = CLK_DISABLE,        .clk = &gclk_rp2350_pll_sys_vco_gate.base },
    { .op = CLK_CONFIG_TARGET,  .clk = &gclk_rp2350_pll_sys_postdiv2_scaler.base },
    { .op = CLK_CONFIG_TARGET,  .clk = &gclk_rp2350_pll_sys_postdiv1_scaler.base },
    { .op = CLK_CONFIG_TARGET,  .clk = &gclk_rp2350_pll_sys_fbdiv_scaler.base },
    { .op = CLK_CONFIG_TARGET,  .clk = &gclk_rp2350_pll_sys_refdiv_scaler.base },
    /* re-enable PLL before using it as source */
    { .op = CLK_ENABLE,         .clk = &gclk_rp2350_pll_sys_vco_gate.base },

    { .op = CLK_SET_PARENT,     .clk = &gclk_rp2350_pll_sys_bypass_mux.base,   .clk_arg = &gclk_rp2350_pll_sys_postdiv2_scaler.base },
    // this may in some cases slow down the clock unnecessarily, but it it is always safe to do
    // because even at the lowest div (== 1) the sys clock is at max exposed to the crystal freq
    // An optimization could take into account if the scaler adaptation would still be allowed
    // after switching over to the new clock
    { .op = CLK_CONFIG_TARGET,  .clk = &gclk_rp2350_clk_sys_div_scaler.base },
    { .op = CLK_SET_PARENT,     .clk = &gclk_rp2350_clk_sys_auxsrc_mux.base,   .clk_arg = &gclk_rp2350_pll_sys_bypass_mux.base },
    { .op = CLK_SET_PARENT,     .clk = &gclk_rp2350_clk_sys_src_mux.base,   .clk_arg = &gclk_rp2350_clk_sys_auxsrc_mux.base },
};

const gclk_manager_topo_switch_desc_t core_clk_topo_switch_descs[] = {
    { .src_topo_id = RP2350_TOPOID_CLK_SYS_AUX_PLLSYS_XOSC,
      .dst_topo_id = RP2350_TOPOID_CLK_SYS_AUX_ROSC,
      .steps = clk_sys__aux_pllsys__aux_rosc,
      .step_cnt= ARRAY_SIZE(clk_sys__aux_pllsys__aux_rosc) },
    { .src_topo_id = RP2350_TOPOID_CLK_SYS_AUX_PLLSYS_XOSC,
      .dst_topo_id = RP2350_TOPOID_CLK_SYS_AUX_XOSC,
      .steps = clk_sys__aux_pllsys__aux_xosc,
      .step_cnt= ARRAY_SIZE(clk_sys__aux_pllsys__aux_xosc) },
    { .src_topo_id = RP2350_TOPOID_CLK_SYS_AUX_XOSC,
      .dst_topo_id = RP2350_TOPOID_CLK_SYS_AUX_PLLSYS_XOSC,
      .steps = clk_sys__aux_xosc__aux_pllsys,
      .step_cnt= ARRAY_SIZE(clk_sys__aux_xosc__aux_pllsys) },
};

#define CORE_CLOCK_TOPO_SWITCH_DESC_NUMOF   (ARRAY_SIZE(core_clk_topo_switch_descs))

/* Defines a list of clock sources that are allowed to be used as the originating source to drive the systems core clock.
 * A clock listed here is not neccessarily able to drive the core clock directly. Instead, this list just defines the roots
 * of possible topologies that may drive the core clock but without listing all possible topologies explicitly.
 * If a given topology and clock configuration is appliccable is of course still subject to certain runtime conditions like
 * configuration constraints and püeripheral requirements */
static const gclk_t* core_clock_sources[] = {
&gclk_rp2350_pll_sys_scaler.base,
&gclk_rp2350_rosc_scaler.base,
&gclk_rp2350_xosc_gate.base,
&gclk_rp2350_gpin0_src,
&gclk_rp2350_gpin1_src,
};

#define CORE_CLOCK_SOURCES_NUMOF   (ARRAY_SIZE(core_clock_sources))

//TODO: those do not add actual constraints (max value same as possible config)
// - just here for testing for now
/* these constraints universally apply for this platform at all times */
const gclk_freq_constraint_t global_clock_constraints[] = {
    { .type = GCLK_ENSURE_MAX_FREQ, .clk = &gclk_rp2350_clk_sys_gate.base, .freq = 150000000 },
    { .type = GCLK_ENSURE_MIN_FREQ, .clk = &gclk_rp2350_pll_usb_refdiv_scaler.base, .freq =   5000000 },
    { .type = GCLK_ENSURE_MIN_FREQ, .clk = &gclk_rp2350_pll_sys_refdiv_scaler.base, .freq =   5000000 },
    { .type = GCLK_ENSURE_MIN_FREQ, .clk = &gclk_rp2350_pll_usb_fbdiv_scaler.base, .freq =  750000000 }, // VCO
    { .type = GCLK_ENSURE_MIN_FREQ, .clk = &gclk_rp2350_pll_sys_fbdiv_scaler.base, .freq =  750000000 }, // VCO
    { .type = GCLK_ENSURE_MAX_FREQ, .clk = &gclk_rp2350_pll_usb_fbdiv_scaler.base, .freq = 1600000000 }, // VCO
    { .type = GCLK_ENSURE_MAX_FREQ, .clk = &gclk_rp2350_pll_sys_fbdiv_scaler.base, .freq = 1600000000 }, // VCO
};

#define GLOBAL_CLOCK_CONSTRAINTS_NUMOF (ARRAY_SIZE(global_clock_constraints))

#define CORE_CLOCK_SOURCES_NUMOF   (ARRAY_SIZE(core_clock_sources))

/* Defines the maximum number of discrete frequency steps that are used for dynamic frequency scaling
 * and PU metric assessment */
#define MAX_DFS_FREQ_VALUES_NUM (6)

#define DFS_CYCLER_MIN_FREQ (2000000)
#define DFS_CYCLER_MAX_FREQ (40000000)

/* for each unique clock instance that has associated frequency configuration limits,
 * this points to the limit list */
static const clock_freq_conf_limits_t gclk_freq_conf_limits[] = {
    { .clk = &gclk_rp2350_clk_sys_gate.base,
      .limits = sys_freq_vc_ws_limits,
      .len = ARRAY_SIZE(sys_freq_vc_ws_limits)
    },
};
#define GCLK_FREQ_LIMIT_CLKS_NUMOF   ARRAY_SIZE(gclk_freq_conf_limits)

/* TODO: replace these placeholders with data from the model */
const gclk_manager_power_properties_t clock_power_model[] = {
    //{ .clk = NULL, .P_en_nW = 0, .C_fF = 0 },
};
#define GCLK_MANAGER_CONF_SYS_PSTATIC_NW (1000000)
#define GLOBAL_CLOCK_POWER_MODEL_PROPERTIES_NUMOF (0)
#define GLOBAL_CLOCK_POWER_MODEL_AVAILABLE (0)


gclk_clock_change_notify_list_t stdio_nl;
gclk_clock_change_notify_list_t timer_reinit_nl;
//extern const gclk_clk_scaler_ll_t gclk_efm32_hfperpresc_scaler;

//TODO: should just be taken from resets.h (currently there are dependency issues with the includes ) */
#define RESET_PLL_USB (1u << 15u)

extern bool _custom_print_enable;

static inline int gclk_manager_platform_init(void) {
    /* the reset must be performed before the regsiters can be written */
    reset_component(RESET_PLL_USB, RESET_PLL_USB);
    //gclk_manager_register_clk_change_cb(&gclk_efm32_hfperpresc_scaler.base, &timer_reinit_nl, gclk_manager_default_timer_reinit_cb);
    //gclk_manager_register_clk_change_cb(&gclk_efm32_hfperpresc_scaler.base, &stdio_nl, gclk_manager_default_stdio_reinit_cb);

    core_voltage_init();

    bool set_uart_to_xosc = true;

    if (set_uart_to_xosc) {
        _custom_print_enable = false;
        const gclk_t* altered_clk = NULL;
        const gclk_t* affected_clk = NULL;
        uint32_t f_old = 0;
        uint32_t f_new = 0;
        bool post_change = false;

        gclk_manager_default_stdio_reinit_cb(altered_clk, affected_clk, f_old, f_new, post_change);
        gclk_disable(&gclk_rp2350_clk_peri_gate.base);

        const gclk_t *mux = &gclk_rp2350_clk_peri_auxsrc_mux.base;
        const gclk_t *parent = &gclk_rp2350_xosc_gate.base;

        int idx = gclk_parent2idx(mux, parent);

        if (idx < 0) {
            printf("[%s] does not have a parent named [%s]\n", gclk_get_name(mux), gclk_get_name(parent));
            assert(false);
        }

        gclk_set_parent(mux, idx);
        gclk_enable(&gclk_rp2350_clk_peri_gate.base);
        post_change = true;
        gclk_manager_default_stdio_reinit_cb(altered_clk, affected_clk, f_old, f_new, post_change);
        _custom_print_enable = true;
    }
    return 0;
}

#ifdef __cplusplus
}
#endif

#endif /* GCLK_MANAGER_CONF_H */
/** @} */
