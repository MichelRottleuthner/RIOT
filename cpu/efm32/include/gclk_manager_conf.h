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
#include "gclk_efm32_types.h"
#include "gclk/generic_scaler.h"

#ifdef __cplusplus
extern "C" {
#endif

extern const unsigned int GCLK_REINIT_CONFIGS_CNT;
extern reinit_trigger_conf_t reinit_configs[];
extern const gclk_t * const core_clock_instance;
extern const unsigned int GCLK_PREFERRED_FREQ_CONF_CNT;
extern gclk_t const *gclock_handle_for_core_freq;
extern const unsigned int DVFS_CONFS_CNT;
extern const gclk_clk_scaler_ll_t gclk_efm32_hfclk_scaler;

/* TODO: forward declare or include? */
/* clocks needed to specify complex transition sequences */
extern const gclk_efm32_gate_t gclk_efm32_hfrco_base_gate;
extern const gclk_clk_scaler_ll_t gclk_efm32_hfrco_scaler;
extern const gclk_efm32_mux_t gclk_efm32_hfsrcclk_mux;
extern const gclk_efm32_gate_t gclk_efm32_hfxo_base_gate;
extern const gclk_clk_scaler_ll_t gclk_efm32_hfrcodiv2_scaler;
extern const gclk_clk_scaler_ll_t gclk_efm32_hfcorepresc_scaler;
extern const gclk_efm32_gate_t gclk_efm32_lfrco_base_gate;
extern const gclk_efm32_gate_t gclk_efm32_lfxo_base_gate;

/* TODO: add a list of different topologies/approaches/entrypoint-clocks that are reasonable on this particular
 *       platform to apply dynamic frequency scaling.
 *       In particular on this platform mainly the following topology options seem reasonable:
 *       (as read from topologies cmd of tests/gclk)
 *       0 | [HFCORECLK]-->[HFCLK]-->[HFSRCCLK]-->[HFRCO]-->[HFRCO_BASE]
 *       1 | [HFCORECLK]-->[HFCLK]-->[HFSRCCLK]-->[HFXO_BASE]
 *
 *       whereas sevaral options are available on *how* to perform the scaling:
 *       A) For both frequency can be scaled using HFCORECLK directly
 *          -only affects CPU freq (i.e., leaves many parts of the tree unscaled, at potentially higher clock than needed)
 *          -wont make use of voltage scaling (voltage scaling levels depend on HFCLK which wont be changed)
 *          -is expected to have the lowest maintenance overhead (no clock change callbacks to other affected entities)
 *       B) For both topologies it is also possible to scale via HFCLK which affects flash, voltage scale, and other peripherals
 *          -i.e. bigger energy saving potential but also higher maintenance overhead
 *       C) For (0) there is the additional option to a more involved scaling pattern available via changing HFRCO frequency directly
 *          -For that it is needed to first switch the topology to (4)
 *           4 | [HFCORECLK]-->[HFCLK]-->[HFSRCCLK]-->[HFRCODIV2]-->[HFRCO]-->[HFRCO_BASE]
 *          -Then scale HFRCO to the target frequency
 *          -And then switch back to HFRCO directly
 **/

/* this transition is meant to be used when the core clock is already driven by HFRCO but it should be rescaled to another frequency */
gclk_manager_sequence_step_t hfrco_rescale_steps[] = {
    /* before adapting HFRCO the system must be temporarily switched over to a divider to ensure the frequency is not overshot
     * while the clock is adapting to the new frequency */
    { .op = CLK_SET_PARENT, .clk = &gclk_efm32_hfsrcclk_mux.base, .clk_arg = &gclk_efm32_hfrcodiv2_scaler.base },
    { .op = CLK_CONFIG_TARGET, .clk = &gclk_efm32_hfrco_scaler.base },
    { .op = CLK_SET_PARENT, .clk = &gclk_efm32_hfsrcclk_mux.base, .clk_arg = &gclk_efm32_hfrco_scaler.base },
};

/* the two below transitions are used to switch from one source to another */
gclk_manager_sequence_step_t hfxo_to_hfrco_transition_steps[] = {
    { .op = CLK_ENABLE,     .clk = &gclk_efm32_hfrco_base_gate.base },
    { .op = CLK_CONFIG_TARGET, .clk = &gclk_efm32_hfrco_scaler.base },
    { .op = CLK_SET_PARENT, .clk = &gclk_efm32_hfsrcclk_mux.base, .clk_arg = &gclk_efm32_hfrco_scaler.base },
    { .op = CLK_DISABLE,    .clk = &gclk_efm32_hfxo_base_gate.base },
};

gclk_manager_sequence_step_t hfrco_to_hfxo_transition_steps[] = {
    { .op = CLK_ENABLE,     .clk = &gclk_efm32_hfxo_base_gate.base },
    { .op = CLK_SET_PARENT, .clk = &gclk_efm32_hfsrcclk_mux.base, .clk_arg = &gclk_efm32_hfxo_base_gate.base },
    { .op = CLK_DISABLE,    .clk = &gclk_efm32_hfrco_base_gate.base },
};


/**
 * @name    Frequency limit configuration
 * 
 * All of these limits apply to gclk_efm32_hfclk_scaler.base.
 *
 * @note EFM32PG1B has the following additional requirements regarding voltage range:
 *  - flash write/erase only available at scale level 2 (vc idx 1)
 *  - TRNG only available a t scale level 2 (vc idx 1)
 *  - HXFO only supported at scale level 2 (vc idx 1)
 */
static const freq_conf_limit_t hfclk_freq_vc_ws_limits[] = {
    { .freq_max =  7000000, .vc_idx_min = 0, .ws_min = 0 },
    { .freq_max = 14000000, .vc_idx_min = 0, .ws_min = 1 },
    { .freq_max = 20000000, .vc_idx_min = 0, .ws_min = 2 },
    { .freq_max = 25000000, .vc_idx_min = 1, .ws_min = 0 },
    { .freq_max = 40000000, .vc_idx_min = 1, .ws_min = 1 },
};

/* These topology ids are obtained via the test application command 'topologies HFCORECLK'.
 * The IDs therefore need to be updated in case the toppologies are updated or represented differently at some point.
 * They are based on the following output:
 * # All possible topologies to drive [HFCORECLK] (max 6 clock nodes)
 * # ID | Topology
 * # ---|-------------------
 * #  0 | [HFCORECLK]-->[HFCLK]-->[HFSRCCLK]-->[HFRCO]-->[HFRCO_BASE]
 * #  1 | [HFCORECLK]-->[HFCLK]-->[HFSRCCLK]-->[HFXO_BASE]
 * #  2 | [HFCORECLK]-->[HFCLK]-->[HFSRCCLK]-->[LFRCO_BASE]
 * #  3 | [HFCORECLK]-->[HFCLK]-->[HFSRCCLK]-->[LFXO_BASE]
 * #  4 | [HFCORECLK]-->[HFCLK]-->[HFSRCCLK]-->[HFRCODIV2]-->[HFRCO]-->[HFRCO_BASE]
 * #  5 | [HFCORECLK]-->[HFCLK]-->[HFSRCCLK]-->[CLKIN0_BASE]
 *
 * */
enum {
    EFM32_HFCORECLK_TOPO_ID_HFRCO = 0,
    EFM32_HFCORECLK_TOPO_ID_HFXO = 1,
    EFM32_HFCORECLK_TOPO_ID_LFRCO = 2,
    EFM32_HFCORECLK_TOPO_ID_LFXO = 3,
};

/* TODO: there are transitions that actually works the same way for multiple source-topologies
 * Therefore topology should actually be some kind of "topologies list" or we need a separate setting entry
 * for each applicable source+topology */
gclk_scale_setting_t scale_settings[] = {
    /* Sclaing the clock via HFRCO is benefitial in terms of potential energy savings because
     * 1) running the system from HFRCO is overall more energy efficient
     * 2) scaling down HFRCO reduces the frequency at the source an thereby reduces the frequency
     *    of a bigger part of the clock tree compared to just reducing the frequency of the CPU core.
     * 3) Also reducing flash/bus frequencies allows for voltage scaling which may otherwise be
     *    permitted due to higher voltage required the clocks that are not slowed down.
     *
     * But this also comes with some downsides:
     * 1) The overhead of the switching operation itself is higher for the scaling operation because
     *    the system must temporarily be switched to an intermediate clock source.
     * 2) The associated maintenance overhead of changing the frequency may be higher. I.e. if users of
     *    HFRCO (or any HFRCO-dependent clocks) must be notified about a frequency change this may induce
     *    additional overhead. Whether this is of concern depends on the application. E.g., for pure computing
     *    tasks that only use the CPU and no peripherals this may be completely irellevant while for applications
     *    that use many peripheral drivers this may have a significant performance impact.
     * 3) This option is not viable for configurations that require better clock accuracy that what is provided
     *    by HFRCO. E.g. for reliable UART operation this can become a problem.
     **/
    {
      .output_clk = &gclk_efm32_hfcorepresc_scaler.base,
      .topology_id = EFM32_HFCORECLK_TOPO_ID_HFRCO,
      .approach = SCALE_SEQUENCE,
      .sequence = hfrco_rescale_steps,
      .sequence_len = ARRAY_SIZE(hfrco_rescale_steps)},

    /* Scaling the core frequency via the HFCLK instance is the next best option regarding potential energy savings
     * compared to directly changing the source frequency.
     * The benefits are similar to scaling HFRCO directly:
     * 1) Same as with scaling HFRCO this still affects a big part of the clock tree and allows for voltage scaling.
     * 2) Additionally, this option is available independent of the actual oscillator source.
     *    I.e. opposed to HFXO not being scalable directly (but HFRCO being scaleable), HFCLK can be scaled
     *    no matter if HFXO or HFRCO is used.
     * 3) The scaling operation itself is a bit faster because no intermediate topology change is needed.
     *
     * Downsides are:
     * 1) The source oscillator will still run at its normal (high) frequency even if HFCLK is scaled down.
     * 2) Same as with HFRCO, the benefit of affecting a bigger part of the clock tree also induces potentially higher
     *    maintenance overhead for resources that depend on this clock.
     **/
    {
      .output_clk = &gclk_efm32_hfcorepresc_scaler.base,
      .scale_clk = &gclk_efm32_hfclk_scaler.base,
      .topology_id = EFM32_HFCORECLK_TOPO_ID_HFXO,
      .approach = SCALE_UPTREE_RELATIVE,},
    /* Scaling the core frequency via this distinct core prescaler is expeced to give the lowset potential energy
     * savings because it only reduces the frequency of the core clock leaving big parts of the clock tree running at
     * its high frequency.
     * The benefits are:
     * 1) The scaling operation itself is fast to execute
     * 2) side effects to other clocks and thereby induced maintenance overhead for other dependent clocks is
     *    expected to be very low to non-existent.
     * Downsides are:
     * 1) Much lower energy savings are expected.
     */
    {
      .output_clk = &gclk_efm32_hfcorepresc_scaler.base,
      .scale_clk = &gclk_efm32_hfcorepresc_scaler.base,
      .topology_id = EFM32_HFCORECLK_TOPO_ID_HFXO,
      .approach = SCALE_DIRECT,},
};

#define SCALE_SETTINGS_NUMOF       (ARRAY_SIZE(scale_settings))

/* Defines a list of clock sources that are allowed to be used as the originating source to drive the systems core clock.
 * A clock listed here is not neccessarily able to drive the core clock directly. Instead, this list just defines the roots
 * of possible topologies that may drive the core clock but without listing all possible topologies explicitly.
 * If a given topology and clock configuration is appliccable is of course still subject to certain runtime conditions like
 * configuration constraints and püeripheral requirements */
static const gclk_t* core_clock_sources[] = {
&gclk_efm32_hfrco_base_gate.base,
&gclk_efm32_hfxo_base_gate.base,
&gclk_efm32_lfrco_base_gate.base,
&gclk_efm32_lfxo_base_gate.base,
};

#define CORE_CLOCK_SOURCES_NUMOF   (ARRAY_SIZE(core_clock_sources))

const gclk_t* gclk_core_clock_handle = &gclk_efm32_hfcorepresc_scaler.base;

//TODO: those do not add actual constraints (max value same as possible config)
// - just here for testing for now
/* these constraints universally apply for this platform at all times */
const gclk_freq_constraint_t global_clock_constraints[] = {
    { .type = GCLK_ENSURE_MAX_FREQ, .clk = &gclk_efm32_hfrco_scaler.base, .freq = 38000000 },
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
    { .clk = &gclk_efm32_hfclk_scaler.base,
      .limits = hfclk_freq_vc_ws_limits,
      .len = ARRAY_SIZE(hfclk_freq_vc_ws_limits)
    },
};
#define GCLK_FREQ_LIMIT_CLKS_NUMOF   ARRAY_SIZE(gclk_freq_conf_limits)
/** @} */

/* TODO: also add explicit transition descriptors for same-to-same-topo transitions? */
gclk_manager_topo_switch_desc_t core_clk_topo_switch_descs[] = {
    { .src_topo_id = 0, .dst_topo_id = 1, .steps = hfrco_to_hfxo_transition_steps, .step_cnt= ARRAY_SIZE(hfrco_to_hfxo_transition_steps) },
    { .src_topo_id = 1, .dst_topo_id = 0, .steps = hfxo_to_hfrco_transition_steps, .step_cnt= ARRAY_SIZE(hfxo_to_hfrco_transition_steps) },
};

#define CORE_CLOCK_TOPO_SWITCH_DESC_NUMOF   (ARRAY_SIZE(core_clk_topo_switch_descs))

/* TODO: replace these placeholders with data from the model */ 
const gclk_manager_power_properties_t clock_power_model[] = {
    //{ .clk = NULL, .P_en_nW = 0, .C_fF = 0 },
};
#define GCLK_MANAGER_CONF_SYS_PSTATIC_NW (1000000)
#define GLOBAL_CLOCK_POWER_MODEL_PROPERTIES_NUMOF (0)
#define GLOBAL_CLOCK_POWER_MODEL_AVAILABLE (0)


gclk_clock_change_notify_list_t stdio_nl;
gclk_clock_change_notify_list_t timer_reinit_nl;
extern const gclk_clk_scaler_ll_t gclk_efm32_hfperpresc_scaler;

static inline int gclk_manager_platform_init(void) {
    gclk_manager_register_clk_change_cb(&gclk_efm32_hfperpresc_scaler.base, &timer_reinit_nl, gclk_manager_default_timer_reinit_cb);
    gclk_manager_register_clk_change_cb(&gclk_efm32_hfperpresc_scaler.base, &stdio_nl, gclk_manager_default_stdio_reinit_cb);
    return 0;
}

#ifdef __cplusplus
}
#endif

#endif /* GCLK_MANAGER_CONF_H */
/**
 * @}
 */
