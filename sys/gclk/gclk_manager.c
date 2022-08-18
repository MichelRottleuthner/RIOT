/*
 * Copyright (C) 2021 HAW Hamburg
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */
/**
 * @ingroup  sys_gclk
 * @{
 *
 * @file
 * @brief    High-level clock manager implementation
 *
 * TODO make use of min/max utility functions in several functions that set sched stat values
 * TODO unify naming fo task/thread
 *
 * @author   Michel Rottleuthner <michel.rottleuthner@haw-hamburg.de>
 *
 * @}
 */
#include <string.h>

#include "gclk_manager.h"
#include "gclk_manager_conf.h"
#include "periph/core_voltage.h"
#include "periph/flash_opt.h"
#include "gclk_idle_timer.h"
#include "mutex.h"
#include "bitfield.h"
#include "stdio_uart.h"
#include "stdio_base.h"
#include "periph/uart.h"
#include "xtimer.h"
#include "ztimer/config.h"
#include "ztimer/periph_timer.h"
#define LOG_LEVEL LOG_NONE
#include "log.h"


/* custom RIOT-specific hooks required to tap into peripheral re-init code for clock config changes */
extern ztimer_periph_timer_t *___ztimer_periph_timer_instance;
extern timer_cb_t __ztimer_perph_timer_cb;
extern void timer_write(tim_t tim, unsigned int cnt);
static unsigned int _timer_cnt_backup = 0;

/**
 * @brief  Get the subset of clock constraints which are applicable to the given topology.
 *
 * @pre (ARRAY_SIZE(acc) >= GLOBAL_CLOCK_CONSTRAINTS_NUMOF)
 *
 * @param[out] acc       location where all clock constraints that apply will be stored.
 * @param[in]  topo      topology of which all clocks will be checked for constraints.
 * @param[in]  topo_len  number of clock instances in @topo.
 *
 * @return     The number of found constraints that apply to the given topology.
 */
static unsigned _populate_applicable_clock_constraints(gclk_freq_constraint_t *acc, clk_topology_entry_t *topo, uint32_t topo_len);

/**
 * @brief propagates a clock config change down the clock tree model
 *
 * Can be used to evaluate how a specific change will affect (the properties of) other clocks.
 * This operation does not affect the active clock configuration ans operates purely on the virtual model representation of the clock tree.
 *
 * @param[in]     changed_conf      pointer to the changed configuration entry.
 * @param[in,out] tree_model        pointer to the topology entries holding (the relevant part of) the clock tree model which represents the state
 *                                  of the clock tree before the change is applied. This state will be updated according to the indicated change
 *                                  so that the @p tree_model reflects the state of the tree after performing the change.
 * @param[in]     tree_model_size   number of clock entries the @p tree_model consists of.
 */
static void _model_propagate_conf_change_downtree(clk_topology_entry_t *changed_conf, clk_topology_entry_t *tree_model, size_t tree_model_size);

/**
 * @brief calculates the down-tree output freq with given parameters.
 *
 * @param[in]  clk     the clock scaler @p factor and @p f_in are applied to.
 * @param[in]  f_in    the input frequency of @p clk.
 * @param[in]  dtf     the fractional scaling factor that is equivalent to all down-tree scaling factors
 *                     after @p clk till the output clock. (Not including the factor of @p clk).
 * @param[in]  factor  the scaling factor of @p clk.
 */
static inline uint32_t _get_freq_for_factors(const gclk_t *clk, uint32_t f_in, gclk_fraction_t *dtf, uint32_t factor);

/**
 * @brief calculates the output freq of a clock for given input freq and factor.
 *
 * @param[in]  clk     the clock scaler @p factor and @p f_in are applied to.
 * @param[in]  factor  the scaling factor of @p clk.
 * @param[in]  f_in    the input frequency of @p clk.
 */
static inline uint32_t _apply_scale_factor(const gclk_t *scaler, uint32_t factor, uint32_t f_in);

/**
 * @brief Voltage scaling enabled state.
 *
 * Stores whether the automatic voltage scaling feature is currently enabled.
 * Never change directly! Use @gclk_manager_enable_voltage_auto_scale() to update this at runtime instead.*/
static bool auto_vscale_enabled = false;

/**
 * @brief Flash wait-state adaptation enabled state.
 *
 * Stores whether the automatic wait state adaptation feature is currently enabled.
 * Never change directly! Use @gclk_manager_enable_flashws_auto_update() to update this at runtime instead.*/
static bool auto_wsadapt_enabled = false;

/**
 * @brief Clock change notification list.
 *
 * This list holds one list of notification callbacks per clock that callbacks were registered for.
 **/
static list_node_t clock_change_notify_list;

/**
 * @brief Number of registered clock change notifications.
 *
 * Registrations are simply counted on reg/unreg operations. This allows a fast check on whether
 * ther are no active registrations in the \ref clock_change_notify_list. */
static unsigned int registered_clk_change_cb_cnt = 0;

/**
 * @brief The last core frequency value set up by the clock manager
 */
volatile uint32_t current_core_freq;

/**
 * @brief The core frequency before DFS got enabled.
 *
 * This old frequency setting is restored when DFS is disabled again.
 */
uint32_t pre_dfs_enable_freq = 0;

/**
 * @brief Clock Manager mutex.
 *
 * mutex used by the manager to guard critical sections like complex topology switch operations.
 * NOTE: As of now the manager should only be used by a single controller entity.
 *       Multiple simultaneous operators are not tested. */
mutex_t clock_conf_mutex = MUTEX_INIT;

/**
 * @brief Frequency cycler context.
 *
 * Holds state of the frequency cycler which is used for automatic performance utilization
 * assemssment (PUA). This process changes the core frequency to a number of different
 * speeds and collects performance metrics for each setting per running thread (if requested).
 * The collected data is used to determine the workload-specific PU metric to rate its scalability. */
typedef struct {
    uint32_t *freqs; /*< pointer to frequency values used for PU assessment freq-cycle. */
    unsigned freq_cnt; /*< number of elements in \ref freqs. */
    uint32_t cpu_time_threshold_ticks; /*< minimum duration of cpu time to collect data for in tick.
                                           This value applies per thread, not for the whole PUA-cycle. */
    uint32_t thread_schedule_threshold; /*< minimum number of thread shedules (per thread) that must
                                            happen before the collected data is considered enough. */
    uint32_t pu_stats_requested; /*< bit field that marks if PU statistics were requested for a thread.
                                     Bit N refers to thread pid N. */
    uint32_t pu_stats_pending_cur_freq; /*< similar to \ref pu_stats_requested but holds the state for
                                            pending pu stats for each freq step of the cycle.
                                            Is set to \ref pu_stats_requested before collecting data at
                                            each cycle freq. */
    unsigned cur_freq_idx; /*< index of the currently assessed frequency setting. Starts at 0 for a new
                               PUA cycle and is incremented for each frequency step till all frequencies
                               were tested. */
    mutex_t  done_mutex; /*< mutex used to synchronize/wait for the PUA-cycle to finish. Will be locked
                             when starting the frequency cycle and unlocked once all data for all
                             frequencies was collected. */
    gclk_manager_core_freq_reconf_cb_t freq_change_cb; /* callback that executes the core frequency update */
    bool freq_cycle_enabled; /*< flag that indicates whether the PUA frequency cycle is currently active.
                                 Is set to true on cycle start and cleared when all data was collected */
} freq_cycle_thread_context_t;

/** @brief Frequency-cycle thread context. */
freq_cycle_thread_context_t fc_ctx;

/** @brief Struct for storing task-specific scheduling metrics */
typedef struct {
    uint32_t cpu_time_ticks; /*< sum of cpu time the task was scheduled in ticks */
    uint32_t schedules; /*< number of times the task was scheduled */
} task_util_metrics_t;

/**
 * @brief Scheduler statistics used by the clock manager.
 *
 * These variables are used to collect metadata on scheduling and busy/idle time to calculate
 * overall CPU utilization and the performance utilization metric for running threads */
typedef struct {
    uint64_t t_went_idle; /*< timestamp when the scheduler went to idle state last */
    uint64_t t_left_idle; /*< timestamp when the schduler left the idle state last */
    uint64_t t_cur_thread_start; /*< timestamp when the current thread got scheduled last */
    volatile uint32_t enter_idle_cnt; /*< number of times the scheduler went to idle state */
    volatile uint32_t idle_ticks; /*< number of ticks the scheduler was idle last */
    volatile uint32_t busy_ticks; /*< number of ticks the scheduler was busy last */
    volatile uint32_t utilization; /*< last utilization calculated via ratio of busy and idle time */
    /* basic averages of the above to reduce feedback speed and improve stability against outliers */
    volatile uint32_t busy_ticks_avg; /*< (moving) average number of ticks the scheduler was busy */
    volatile uint32_t idle_ticks_avg; /*< (moving) average number of ticks the scheduler was idle */
    volatile uint32_t utilization_avg; /*< (moving) average utilization based on busy/idle ratio */
    int task_performance_util[GCLK_MANAGER_PU_STATS_TASK_NUM]; /*< PU value for each thread */
    /* metadata for debugging/testing purposes (e.g. to provide insight of the value
     * ranges, dvfs usage, and to evaluate how tick resolution affects accuracy) */
    volatile uint32_t idle_ticks_min; /*< lowest number of idle ticks observed */
    volatile uint32_t idle_ticks_max; /*< highest number of idle ticks observed */
    volatile uint32_t busy_ticks_min; /*< lowest number of busy ticks observed */
    volatile uint32_t busy_ticks_max; /*< highest number of busy ticks observed */
    uint32_t freq_sched_cnt[MAX_DFS_FREQ_VALUES_NUM]; /*< number of schedules at each freq. idx */

    /* scheduling and timing metrics per thread at each frequency */
    volatile task_util_metrics_t task_perf_util_data[GCLK_MANAGER_PU_STATS_TASK_NUM][MAX_DFS_FREQ_VALUES_NUM];
} gclk_manager_sched_stats_t;

/** @brief Global scheduler statistics data. */
static gclk_manager_sched_stats_t _sched_stats;

/** @brief Stores the state of the clock manager. */
typedef struct {
    /**
     * @brief Currently active (DFS) frequency scale idx.
     *
     * This value refers to the frequency value in the prepopulated \ref dfs_frequencies array, which
     * holds frequency values that are applicable to the current frequency scaling settings defined
     * by \ref active_core_scale_setting.
     * This variable is only set by \ref _dvfs(). The explicit freq. scaler used for PUA
     * tracks its freq scale idx separately as it may use another (PUA-specific) set of frequencies. */
    int current_dfs_freq_idx;

    /**
     * @brief Currently active D(V)FS setting.
     *
     * The scale setting that is applied for scaling the core clock via D(V)FS.
     * Different options for this setting should be defined in the gclk_manager_conf file
     * according to hardware capabilities. The \ref gclk_manager_init() function sets this up to
     * the first applicable setting for the active topology (if there is any). */
    const gclk_scale_setting_t *active_core_scale_setting;

    clk_topology_entry_t topology_conf_cache[GCLK_MANAGER_PREP_CONFS_MAX_NUMOF][GCLK_MANAGER_PREP_CONFS_TOPO_MAX_LEN];
    gclk_manager_sequence_step_t prepared_rescale_sequences[MAX_DFS_FREQ_VALUES_NUM][GCLK_MANAGER_MAX_PREPARED_SEQUENCE_LEN];
    int prepared_rescale_sequence_lengths[MAX_DFS_FREQ_VALUES_NUM];

    /* This gets updated with a list of possible frequencies when setting the clock handle that is used
     * for dynamic frequency scaling. If the handle that is set up supports more values than this can hold
     * a subset of possible values is stored instead */
    uint32_t dfs_frequencies[MAX_DFS_FREQ_VALUES_NUM];

    /* Holds the number of valid frequency settings contained in dfs_frequencies */
    unsigned int dfs_frequencies_cnt;

    /* For the simple case where a PU threshold is used for D(V)FS control, this defines the min. level
     * on when to use a higher frequency for execution.
     * TODO: instead of basic threshold control this should be replaced with a (parameterized) lookup
     *       function which returns the optimal configuration for a given PU value. Additionally,
     *       information on the flash/voltage scaling behavior can be added here e.g. to affect wether
     *       low voltage or fast flash is preferrable for a specific task. More global considereations
     *       (how is this decision affected by other therads, and overhead of reconfiguration steps)
     *       should also be evaluated in more detail */
    volatile int pre_sched_freq_boost_threshold;

    /* Same as \ref pre_sched_freq_boost_threshold but marks the limit for lower frequency operation.
     * I.e., defines if a lower frequency should be set up for execution. */
    volatile int pre_sched_freq_throttle_threshold;

    /* fast boost frequency that is applied if a thread has a 'high' PU value as indicated
     * by \ref pre_sched_freq_boost_threshold. */
    volatile uint32_t pre_sched_boost_freq;

    /* slow throttle frequency that is applied if a thread has a 'low' PU value
     * as indicated by \ref pre_sched_freq_boost_threshold */
    volatile uint32_t pre_sched_throttle_freq;

    /* controls whether dynamic frequency scaling is applied before scheduling a thread */
    volatile bool pre_sched_pu_dfs_enabled;

    /* controls whether scheduling/thread/timing metadata is collected to assess the PU metric during
     * execution. Usually this should be enabled together with with the frequency scaler-thread that
     * proactively cycles between frequencies. */
    bool pu_metadata_collection_enabled;

    /* controls if DVFS should be applied based on global CPU utilization (not thread PU)
     * TODO: this should be merged with pre_sched_pu_dfs etc. to define the adaptation method just at one place */
    bool cpu_util_based_dvfs_enabled;

    /* This policy defines which optimization goal to prefer in cases where different optimizations are
     * possible but some parameters are mutually exlusive */
    gclk_manager_dvs_policy_t dvs_policy;

    /* absolute maximum number of clocks involved in a (sub-)topology of any clock.
     * This is determined once at \ref gclk_manager_init(). */
    unsigned int max_clocks_in_topology;

    /* absolute maximum number of clocks involved in a topology that drives the core clock.
     * This is determined once at \ref gclk_manager_init(). */
    unsigned int max_clocks_in_core_topology;

    /* NOTE: below values are cached for faster operation an therefore need
     * to be updated at relevant changes. This is usually needed when the manager
     * reconfigures the clock tree (e.g., via the switch_topology function). */

    /* length of current topology that drives the core clock. */
    unsigned int current_core_topolen;

    /* holds the currently active topology id for the core clock handle. */
    int current_core_topo_id;

    /* holds the topology that was previously set up to drive the core clock.
     * NOTE: the size is currently based on an absolute worst case assumption. */
    clk_topology_entry_t current_core_topology[GCLK_MANAGER_PREP_CONFS_TOPO_MAX_LEN];

    /* Core-clock change notification list.
     * Allocates space for one clock change notification entry for each clock that
     * has constraints on core voltage and/or wait state configuration. */
    gclk_clock_change_notify_list_t ccnl[GCLK_FREQ_LIMIT_CLKS_NUMOF];

    /* TODO: implement functions to switch this to other implementations at runtime */
    /* Depending on the clock configuration, topology, dependencies and the DVFS-scheme at use, the
     * mathod to switch the frequency may be changed at runtime.
     * If, for example, it is known beforehand (still at runtime) that no other clock instance is
     * affected by changing the core clock (because no dependent peripharal is in use or because the
     * core is clocked by a completely independent instance), it may be possible to completely avoid
     * checks of whether a clock is affected and also not callbacks are required.
     * The same applies to clocks where it is known that no complex transition mechanism (temporary
     * swithcing to another clock) is needed. */
    gclk_manager_core_freq_reconf_cb_t freq_change_cb;
} gclk_manager_ctx_t;

/** @brief Global clock manager context. */
static gclk_manager_ctx_t _mgr_ctx;

/**
 * @brief Core frequency scaling callback.
 *
 * A freq change implementation that automatically uses the scaling approach that applies to the current
 * topology (as defined by the active scale setting of the manager).
 * This function is not meant to be used to setup arbitrary frequency settings but to regularly switch
 * between predetermined frequency settings (e.g., for D(V)FS).
 *
 * @param[in] new_freq      The new core frequency to set up. Must be a valid value. Can be ensured
 *                          e.g. by calling this function only with values of the DFS cache.
 */
static void _freq_change_scale_auto(uint32_t new_freq) {
    gclk_manager_scale_core_freq(new_freq);
}

/**
 * @brief Add DFS config entry to the config cache.
 *
 * @note In case of a single scaled clock there is no need the save the whole topology config.
 *       The topology conf cache storage is reused regardless, but relevant data is just stored
 *       in the first element of the data structure. For that reason, the topology conf cache
 *       does not represent a full config and shall not be used by other methods that operate
 *       on topology structures.
 *
 * @param[in] cidx     The configuration index (DFS frequency index) to store the data at.
 * @param[in] freq     The resulting core frequency of the appended setting.
 * @param[in] factor   The scaling factor used to obtain the core frequency @freq.
 *
 */
static void _append_dfs_cache_entry(unsigned cidx, uint32_t freq, uint32_t factor) {
    /* TODO for some use cases it could be benefitial to also precalculate/store the equivalent
     *      downtree factors or the scale factors per instance.
     * TODO predetermined configurations can also cache VS/WS configs instead of using the
     *      notification callback for that. */
    _mgr_ctx.topology_conf_cache[cidx][0].clk_freq = freq;
    _mgr_ctx.topology_conf_cache[cidx][0].factor = factor;
    _mgr_ctx.dfs_frequencies[cidx] = freq;
}

/** @brief Helper to update cached state. */
static void _update_cached_state_vars(void) {
    _mgr_ctx.current_core_topolen = gclk_get_current_topology_len(gclk_core_clock_handle);
    _mgr_ctx.current_core_topology[0].clk = gclk_core_clock_handle;
    gclk_get_current_topology_config(_mgr_ctx.current_core_topology, _mgr_ctx.current_core_topolen);
    _mgr_ctx.current_core_topo_id = gclk_topology2id(_mgr_ctx.current_core_topology, _mgr_ctx.current_core_topolen);

    /* always assume there is no applicable scale setting in case none can be found */
    _mgr_ctx.active_core_scale_setting = NULL;
    /* select first appliccable scale setting for the current core topology as the active scale setting
     * that will be used by automatic scale operations (e.g., via gclk_manager_scale_core_freq()) */
    for (unsigned i = 0; i < SCALE_SETTINGS_NUMOF; i++) {
        if (scale_settings[i].output_clk == gclk_core_clock_handle &&
            scale_settings[i].topology_id == _mgr_ctx.current_core_topo_id) {
            _mgr_ctx.active_core_scale_setting = &scale_settings[i];
            break;
        }
    }
}

/**
 * @brief Contrained clock configs cache.
 *
 * Holds the state of clocks which put up limiting constaints on core voltage or flash access.
 **/
clk_topology_entry_t constrained_clocks_conf_cache[GCLK_FREQ_LIMIT_CLKS_NUMOF];

/**
 * @brief Initialize the cache that handles DVS/WS config constraints.
 *
 * This function saves a config copy of all clocks that put up constraints on DVS/WS settings.
 * Based on the current configuration active DVS policy the best applicable settings for
 * core voltage and wait states are determined and set up.
 */
void _init_dvs_wsa_constraint_cache(void) {

    /* for all clock instances with assinged vcore/flash-waitstate constraints, load the current config into the cache */
    for (unsigned i = 0; i < GCLK_FREQ_LIMIT_CLKS_NUMOF; i++) {
        constrained_clocks_conf_cache[i].clk = gclk_freq_conf_limits[i].clk;
        gclk_get_current_topology_config(&constrained_clocks_conf_cache[i], 1);
    }

    unsigned min_ws;
    unsigned min_vc_idx;

    /* determine the best configuration that fulfilll all constraints and set it up */
    gclk_get_min_required_ws_vc_from_tree_config(constrained_clocks_conf_cache, GCLK_NUM_OF_CLOCKS, &min_ws,  &min_vc_idx, _mgr_ctx.dvs_policy);

    /* For testing purposes check if the initial config was optimal */
    unsigned cur_ws = flash_opt_get_wait_states();
    unsigned cur_vc = core_voltage_get();

    if ((cur_ws != min_ws) || (cur_vc != min_vc_idx)) {
        printf("WARNING! vcore/flash-waitstate settings werent set up to the best determined config!\n"
               "(was %u WS and %u VC, instead of %u WS and %u VC)\n", cur_ws, min_ws, cur_vc, min_vc_idx);
    }

    flash_opt_set_wait_states(min_ws);
    core_voltage_set(min_vc_idx);
}

/**
 * @brief DVS/WSA update callback.
 *
 * Uses the gneric clock change callback mechanism to update the cache of constrained clock
 * configurations. This state is used by \ref _post_notify_commit() later on in the
 * recofiguration process to update the DVS/WSA config with the best applicable settings
 * considering all constraints.
 *
 * @see \ref clock_change_cb_t for the interface documentation.
 */
void _dvs_wsa_freq_constraint_change_cb(const gclk_t* altered_clk, const gclk_t* affected_clk,
                                        uint32_t f_old, uint32_t f_new, bool post_change) {
    (void)altered_clk;
    (void)f_old;
    (void)post_change;

    /* optimize storage lookup via context variable that gets initialized once on manager init */
    for (unsigned i = 0; i < GCLK_FREQ_LIMIT_CLKS_NUMOF; i++) {
        if (gclk_freq_conf_limits[i].clk == affected_clk) {
            /* save the changed state to the cache to determine the tree-wide limit at the end of a multi-clock change */
            constrained_clocks_conf_cache[i].clk = affected_clk;
            constrained_clocks_conf_cache[i].clk_freq = f_new;
        }
    }

}

/** @brief Helper to print a configuration change. */
static void _print_conf_change(clk_topology_entry_t *old, clk_topology_entry_t *new) {
    printf("%s changed from %8lu Hz (%s) to %8lu Hz (%s)\n", gclk_get_name(old->clk),
                                                             old->clk_freq, old->enabled ? "enabled" : "disabled",
                                                             new->clk_freq, new->enabled ? "enabled" : "disabled");
}

static uint32_t _min(uint32_t a, uint32_t b) {
    return (a <= b) ? a : b;
}

static uint32_t _max(uint32_t a, uint32_t b) {
    return (a >= b) ? a : b;
}

static uint32_t _abs_diff(uint32_t a, uint32_t b) {
    return (a > b) ? (a - b) : (b - a);
}

static void _get_minmax_equivalent_factors_of_topology(clk_topology_entry_t *topo, size_t topo_len,
                                                       gclk_fraction_t *min, gclk_fraction_t *max) {
    uint32_t minfm = 1;
    uint32_t minfd = 1;

    uint32_t maxfm = 1;
    uint32_t maxfd = 1;

    for (unsigned i = 0; i < topo_len; i++) {
        gclk_factor_limit_t limits;
        gclk_get_factor_minmax(topo[i].clk, &limits);

        if (gclk_is_divider(topo[i].clk)) {
            minfd *= limits.max;
            maxfd *= limits.min;
        } else if (gclk_is_multiplier(topo[i].clk)) {
            maxfm *= limits.max;
            minfm *= limits.min;
        }
    }

    min->n = minfm;
    min->d = minfd;
    max->n = maxfm;
    max->d = maxfd;
}

static int _clk_to_entry_idx(clk_topology_entry_t *topo, size_t len, const gclk_t *clk) {
    for (unsigned i = 0; i < len; i++) {
        if (topo[i].clk == clk) {
            return i;
        }
    }
    return -1;
}

/* returns the equivalent factors */
static void _get_equivalent_dt_factors(clk_topology_entry_t *topo, size_t len, const gclk_t *src, gclk_fraction_t *dtf, bool incl_src) {
    int idx = _clk_to_entry_idx(topo, len, src);
    if (idx > 0) {
        gclk_manager_get_combined_topology_fraction(topo, len - (len - idx) + (incl_src ? 1 : 0), dtf);
    } else {
        printf("given src is not in topology!\n");
    }
}

static bool _get_minmax_constraint(const gclk_t *clk, gclk_freq_limit_t *limit) {
    uint32_t fmin = 0;
    uint32_t fmax = 0xFFFFFFFF;
    bool is_constrained = false;

    for (unsigned c = 0; c < GLOBAL_CLOCK_CONSTRAINTS_NUMOF; c++) {
        if (global_clock_constraints[c].clk == clk) {
            is_constrained = true;
            if (global_clock_constraints[c].type == GCLK_ENSURE_MIN_FREQ) {
                fmin = _max(global_clock_constraints[c].freq, fmin);
            }
            if (global_clock_constraints[c].type == GCLK_ENSURE_MAX_FREQ) {
                fmax = _min(global_clock_constraints[c].freq, fmax);
            }
        }
    }

    limit->min = fmin;
    limit->max = fmax;

    return is_constrained;
}

/* combines all constraints put up by clocks of the given topology into an absolute min/max frequency requirement
 * for the input to the topology. Returned limits indicate that it is not allowed to feed the topology with
 * a frequency that is lower than f_min or higher than f_max */
static void _get_minmax_applicable_topo_input_freq(clk_topology_entry_t *topo, size_t len, gclk_freq_limit_t *freq_limits) {

    uint32_t fmin = 0;
    uint32_t fmax = 0xFFFFFFFF;

    for (unsigned i = 0; i < len; i++) {

        /* will hold the most strict output constraint for this i-th clock (if any applies) */
        gclk_freq_limit_t fo_lim;

        /* if this clock puts up a constraint check if it is more severe than the
         * constraint put up by any children before */
        if (_get_minmax_constraint(topo[i].clk, &fo_lim)) {
            /* in case this clock requires a higher minimum freq than its children,
             * update the absolute constraint accordingly */
            fmin = _max(fo_lim.min, fmin);
            /* in case this clock enforces a lower maximim freq than its children,
             * update the absolute constraint accordingly */
            fmax = _min(fo_lim.max,fmax);
        }

        /* calculate the constraints for the *input* side of this clock.
         * This alos implicitly propagates the frequency requirement from
         * the input side of the current clock to the output side of the parent (by iterating uptree) */
        gclk_factor_limit_t fact_lim;
        if (gclk_is_scalable(topo[i].clk)) {
            gclk_get_factor_minmax(topo[i].clk, &fact_lim);
            // TODO: in case there are configurations where !(freq>>factor), rounding could become relevant
            if (gclk_is_multiplier(topo[i].clk)) {
                /* minimum frequency that must be fed into this clock so that it is still able to fulfill constraints.
                 * The minimum input freq. is defined by the max multiplier@min allowed output freq */
                fmin = fmin / fact_lim.max;
                /* max input freq. := min multiplier@max allowed output freq */
                fmax = fmax / fact_lim.min;
            } else { /* divider */
                /* min input freq. := min divider@min output freq. */
                fmin = fmin * fact_lim.min;
                /* max input freq := max divider@max output freq. */
                fmax = fmax * fact_lim.max;
            }
        }
    }

    freq_limits->min = fmin;
    freq_limits->max = fmax;
}

static bool _within_dfs_range(uint32_t freq) {
    if ((freq < DFS_CYCLER_MIN_FREQ) || (freq > DFS_CYCLER_MAX_FREQ)) {
        return false;
    }
    return true;
}

static inline uint32_t _get_freq_for_factors(const gclk_t *clk, uint32_t f_in, gclk_fraction_t *dtf, uint32_t factor) {
    if (gclk_is_multiplier(clk)) {
        return (uint64_t)f_in * (uint64_t)dtf->n * (uint64_t)factor / (uint64_t)dtf->d;
    } else {
        return (uint64_t)f_in * (uint64_t)dtf->n / ((uint64_t)factor * (uint64_t)dtf->d);
    }
}

/* Determines the best factor of available opptions to setup a given target frequency.
 * The function assumes the following szenario:
 * [input clock@f_in Hz]---drives-->[clk]---drives-->[down-tree-topology(x dt_mul, / dt_div)]---outputs-->[target_freq]
 *
 * @param[in] clk          a scalable clock instance (multiplier or divider)
 * @param[in] f_in         the fixed input frequency of @clk
 * @param[in] target_freq  the wanted output frequency value to aim for. This frequency will apply behind both, the
 *                         scaled @clk and the downtree-topology config after it, where the topology is represented by
 *                         a combined fraction.
 * @param[in] dt_factor    a combined fractional factor that represents the whole topology after @clk and shall generate
 *                         a frequency @target_freq at its output.
 *
 * @return    The scaling factor of @clk which generates the closest frequency to @target_freq for @clk and given configs.
 */
static uint32_t _get_best_factor(const gclk_t *clk, uint32_t f_in, uint32_t target_freq, gclk_fraction_t *dt_factor) {
    uint32_t min_diff = 0xFFFFFFFF;
    uint32_t factor_cnt = gclk_factor_cnt(clk);
    uint32_t best_factor = 0;
    for (unsigned i = 0; i < factor_cnt; i++) {
        uint32_t factor = gclk_idx2factor(clk, i);
        uint32_t f = _get_freq_for_factors(clk, f_in, dt_factor, factor);
        uint32_t diff = _abs_diff(f, target_freq);
        if (diff < min_diff) {
            min_diff = diff;
            best_factor = factor;
        }
    }
    return best_factor;
}

static clk_topology_entry_t *_clear_core_topology_cache(clk_topology_entry_t *cacheloc) {
    clk_topology_entry_t *topology = cacheloc;
    memset(topology, 0, sizeof(clk_topology_entry_t) * _mgr_ctx.max_clocks_in_core_topology);
    topology[0].clk = gclk_core_clock_handle;
    topology[0].clk_freq = GCLK_INVALID_FREQ;
    return topology;
}

static const char* _approach2_str(gclk_scale_approach_t approach) {
    switch (approach) {
        case SCALE_DIRECT: return "SCALE_DIRECT";
        case SCALE_UPTREE_RELATIVE: return "SCALE_UPTREE_RELATIVE";
        case SCALE_INTERMEDIATE_TOPO_AUTO: return "SCALE_INTERMEDIATE_TOPO_AUTO";
        default: return "INVALID_APPROACH";
    }
}

static inline uint32_t _apply_scale_factor(const gclk_t *scaler, uint32_t factor, uint32_t f_in) {
    if (gclk_is_multiplier(scaler)) {
        return f_in * factor;
    }

    return f_in / factor;
}

static void _get_scaler_min_max_freq(const gclk_t *scaler, uint32_t factor, gclk_freq_limit_t *fi_limits, gclk_freq_limit_t *fo_limits) {
    fo_limits->min = _apply_scale_factor(scaler, fi_limits->min, factor);
    fo_limits->max = _apply_scale_factor(scaler, fi_limits->max, factor);
}

static bool _limits_are_disjunct(gclk_freq_limit_t *a, gclk_freq_limit_t *b) {
    return (a->max < b->min) || (a->min > b->max);
}

static void _get_scale_factor_limits(const gclk_t *scaler, gclk_freq_limit_t *f_in, gclk_freq_limit_t *f_out, gclk_factor_limit_t *factor_limits) {
    /* initialize to opposite extremes before searching for actual min max */
    factor_limits->min = 0xFFFFFFFF;
    factor_limits->max = 1;

    size_t fact_cnt = gclk_factor_cnt(scaler);
    /* for each factor: check if it can be ignored completely (i.e. if it is out of range for the given limits) */
    for (unsigned i = 0; i < fact_cnt; i++) {
        uint32_t factor = gclk_idx2factor(scaler, i);
        gclk_freq_limit_t limits_at_this_factor;
        _get_scaler_min_max_freq(scaler, factor, f_in, &limits_at_this_factor);

        if (!_limits_are_disjunct(&limits_at_this_factor, f_out)) {
            factor_limits->min = _min(factor, factor_limits->min);
            factor_limits->max = _max(factor, factor_limits->max);
        }
    }
}

static int _populate_dfs_freqs_bf(const gclk_scale_setting_t *scs, const uint32_t *freqs, size_t cnt) {
    size_t match_freq_cnt = 0;
    unsigned matched = 0;

    printf("populate %s freqs for %s approach\n", cnt > 0 ? "the following" : "default", _approach2_str(scs->approach));
    if (cnt > 0) {
        for (unsigned i = 0; i < cnt; i++) {
            printf("%u: %lu\n", i, freqs[i]);
        }
    }

    _mgr_ctx.current_core_topolen = gclk_get_current_topology_len(gclk_core_clock_handle);
    gclk_get_current_topology_config(_mgr_ctx.current_core_topology, _mgr_ctx.current_core_topolen);
    _mgr_ctx.current_core_topo_id = gclk_topology2id(_mgr_ctx.current_core_topology, _mgr_ctx.current_core_topolen);

    if (scs->approach == SCALE_DIRECT ||
        scs->approach == SCALE_UPTREE_RELATIVE) {
        /* Currently there are two different approaches that adapt the frequency via only a single clock instance (scaler)
         * I.e., SCALE_DIRECT and SCALE_UPTREE_RELATIVE. To come up with configurations that are feasible for both configs
         * an exploration must be performed that matches the given set of target frequencies only via the single scaled clock.
         * One way to do this is to just assume the current (default) config of the topology and select the factors
         * that match the given freqs as close as possible.
         * A more comprehensive (and complex) approach derives the best combination of other involved factors that then
         * match the given frequencies best via the single adapted scaler.
         * */
        uint32_t max_involved_clks = _mgr_ctx.max_clocks_in_core_topology;
        //clk_topology_entry_t *topology = _clear_core_topology_cache(&_mgr_ctx.topology_conf_cache[0][0]);

        /* once for each topology, check if there are any global constraints that must be considered */
        gclk_freq_constraint_t relevant_clock_constraints[GLOBAL_CLOCK_CONSTRAINTS_NUMOF];
        unsigned rel_constr_cnt = _populate_applicable_clock_constraints(relevant_clock_constraints, _mgr_ctx.current_core_topology, _mgr_ctx.current_core_topolen);

        size_t possible_freq_cnt = gclk_factor_cnt(scs->scale_clk);

        /* if no freq values are provided explicitly, derive the target frequencies from the
         * highest possible frequency at its minimal power configuration and using the available factors
         * of the scaled clock */
        if (freqs == NULL || cnt == 0) {
            /* only match as many frequencies as we can store or are possible via the scaler, whatever is smaller */
            match_freq_cnt = MAX_DFS_FREQ_VALUES_NUM <= possible_freq_cnt ? MAX_DFS_FREQ_VALUES_NUM : possible_freq_cnt;
        } else {
            match_freq_cnt = cnt <= possible_freq_cnt ? cnt : possible_freq_cnt;
            match_freq_cnt = match_freq_cnt <= MAX_DFS_FREQ_VALUES_NUM ? match_freq_cnt : MAX_DFS_FREQ_VALUES_NUM;
        }

        gclk_fraction_t dtf;
        _get_equivalent_dt_factors(_mgr_ctx.current_core_topology, _mgr_ctx.current_core_topolen, scs->scale_clk, &dtf, false);

        int srcidx = _clk_to_entry_idx(_mgr_ctx.current_core_topology, max_involved_clks, scs->scale_clk);
        /* TODO: replace this with a utility function that returns the the input freq
         * (for cases where the source itself is scalable) */
        uint32_t input_freq = _mgr_ctx.current_core_topology[srcidx+1].clk_freq;

        uint32_t prev_freq = 0;

        for (unsigned i = 0; (i < possible_freq_cnt) && (matched < match_freq_cnt); i++) {
            uint32_t factor;
            /* if a set of specific target freqs was provided match them.
             * if not, just scale down the max freq via available factors */
            if (cnt > 0) {
                factor = _get_best_factor(scs->scale_clk, input_freq, freqs[i], &dtf);
            } else {
                factor = gclk_idx2factor(scs->scale_clk, i);
            }
            uint32_t possible_freq = _get_freq_for_factors(scs->scale_clk, input_freq, &dtf, factor);

            /* filter out duplicates and invalids on the fly */
            if (_within_dfs_range(possible_freq) && ((matched == 0) || (prev_freq != possible_freq))) {

                /* only one clock is touched. So to check for constraint violations we just copy the whole core topology config
                 * and only overwrite the setting specific to the single adapted scaler.
                 * Before constraint violations can be checked the downtree effects of the scaler adaptaion must be applied
                 * to a model of the current config */
                memcpy(&_mgr_ctx.topology_conf_cache[matched][0], _mgr_ctx.current_core_topology, sizeof(clk_topology_entry_t) * _mgr_ctx.current_core_topolen);
                _mgr_ctx.topology_conf_cache[matched][srcidx].clk_freq = possible_freq;
                _mgr_ctx.topology_conf_cache[matched][srcidx].factor = factor;
                _model_propagate_conf_change_downtree(&_mgr_ctx.topology_conf_cache[matched][srcidx], &_mgr_ctx.topology_conf_cache[matched][0], _mgr_ctx.current_core_topolen);

                if (LOG_LEVEL >= LOG_DEBUG) {
                    gclk_manager_print_topology_conf(&_mgr_ctx.topology_conf_cache[matched][0], _mgr_ctx.current_core_topolen, false, true);
                }

                const gclk_freq_constraint_t *constraint = gclk_manager_conf_breaks_constraint(relevant_clock_constraints, rel_constr_cnt, &_mgr_ctx.topology_conf_cache[matched][0], _mgr_ctx.current_core_topolen);
                if (!constraint) {
                    _append_dfs_cache_entry(matched++, possible_freq, factor);
                    prev_freq = possible_freq;
                    LOG_DEBUG("%lu Hz via %lu does not violate any constraint!\n", possible_freq, factor);
                } else {
                    LOG_DEBUG("%lu Hz via %lu violates a constraint!\n", possible_freq, factor);
                    LOG_DEBUG("constraint: %s %s %lu\n", gclk_get_name(constraint->clk), constraint->type == GCLK_ENSURE_MIN_FREQ ? "MIN" : "MAX", constraint->freq);
                }
            }
        }
        match_freq_cnt = matched;
    } else if (scs->approach == SCALE_INTERMEDIATE_TOPO_AUTO) {
        gclk_cmp_func_t cmp_func = gclk_manager_cmp_topology_exact_leaf_freq_pmin;

        uint32_t freq_step = (DFS_CYCLER_MAX_FREQ - DFS_CYCLER_MIN_FREQ) / (cnt - 1);

        for (unsigned i = 0; i < cnt; i++) {
            uint32_t max_involved_clks = _mgr_ctx.max_clocks_in_core_topology;
            clk_topology_entry_t *topology = _clear_core_topology_cache(&_mgr_ctx.topology_conf_cache[matched][0]);

            uint32_t target_freq;
            /* either use provided freq values or spread them across allowed range */
            if (freqs == NULL) {
                /* This uses a simplified method to get a somewhat linear distribution within the allowed DFS frequency range.
                 * In case the topology only supports a very limited number of frequencies or unequally distributed frequencies it
                 * is possilbe that this approach returns overlapping frequencies (i.e. if the brute force gives the same matched
                 * frequency for differend aimed-for-frequencies). Therefore we simply drop duplicates.*/
                target_freq = DFS_CYCLER_MIN_FREQ + i * freq_step;
            } else {
                target_freq = freqs[i];
            }

            int tid = _mgr_ctx.active_core_scale_setting->topology_id;
            size_t valid_cnt = 0;
            int force_nth = -1;
            uint32_t leaf_freq = gclk_manager_brute_force_freq_conf(gclk_core_clock_handle, topology, &max_involved_clks,
                                                                    &tid, cmp_func, (void*)&target_freq, &valid_cnt, force_nth, NULL);
            if (leaf_freq != GCLK_INVALID_FREQ) {
                /* ignore duplicates on the fly */
                if ((!matched) || (_mgr_ctx.topology_conf_cache[matched-1][0].clk_freq != leaf_freq)) {
                    _mgr_ctx.dfs_frequencies[matched] = leaf_freq;
                    gclk_manager_sequence_step_t *seq = &_mgr_ctx.prepared_rescale_sequences[matched][0];
                    int seq_size = gclk_manager_derive_sequence(_mgr_ctx.current_core_topology, _mgr_ctx.current_core_topolen,
                                                                topology, max_involved_clks, seq,
                                                                GCLK_MANAGER_MAX_PREPARED_SEQUENCE_LEN);
                    if (seq_size > 0) {
                        _mgr_ctx.prepared_rescale_sequence_lengths[matched] = seq_size;
                        matched++;
                    } else {
                        LOG_DEBUG("%s: transition from [%s] topology from %d to %d infeasible!\n", __FUNCTION__, gclk_get_name(gclk_core_clock_handle), _mgr_ctx.current_core_topo_id, _mgr_ctx.current_core_topo_id);
                    }
                }
            }
        }
        match_freq_cnt = matched;
    }

    return match_freq_cnt;
}

static uint32_t _append_performance_util_data(uint32_t task_id, uint32_t freq, uint32_t busy_ticks) {
    (void)freq; /* currently, performance util data is only collected when the frequency cycler is
                   is used, and in that case the frequency (or actually its index) is known out-of-band,
                   (via the freq cycler context), therfore, translation from freq to index can be skipped */
    uint32_t freq_idx = fc_ctx.cur_freq_idx;
    _sched_stats.task_perf_util_data[task_id][freq_idx].cpu_time_ticks += busy_ticks;
    _sched_stats.task_perf_util_data[task_id][freq_idx].schedules++;

    if (_sched_stats.task_perf_util_data[task_id][freq_idx].cpu_time_ticks >= fc_ctx.cpu_time_threshold_ticks &&
        _sched_stats.task_perf_util_data[task_id][freq_idx].schedules >= fc_ctx.thread_schedule_threshold) {
        /* mark that enough stats were collected for this thread */
        fc_ctx.pu_stats_pending_cur_freq &= ~(1 << task_id);
    }
    return 0;
}

static void _dvfs(uint32_t utilization) {

    unsigned state = irq_disable();
    /* dfs can only be applied if there are multiple freq settings available */
    if (_mgr_ctx.dfs_frequencies_cnt > 0) {
        int old_scale_idx = _mgr_ctx.current_dfs_freq_idx;

        if (utilization > 80) {
            _mgr_ctx.current_dfs_freq_idx++;
        } else if (utilization < 60){
            _mgr_ctx.current_dfs_freq_idx--;
        }

        if (_mgr_ctx.current_dfs_freq_idx < 0) {
            _mgr_ctx.current_dfs_freq_idx = 0;
        } else if ((uint32_t)_mgr_ctx.current_dfs_freq_idx >= _mgr_ctx.dfs_frequencies_cnt){
            _mgr_ctx.current_dfs_freq_idx =  _mgr_ctx.dfs_frequencies_cnt - 1;
        }

        if(_mgr_ctx.current_dfs_freq_idx != old_scale_idx) {
            gclk_manager_scale_core_freq(_mgr_ctx.dfs_frequencies[_mgr_ctx.current_dfs_freq_idx]);
        }

        _sched_stats.freq_sched_cnt[_mgr_ctx.current_dfs_freq_idx]++;
    }

    irq_restore(state);
}

static bool _is_clk_modification_step(gclk_manager_sequence_step_t *step) {
    switch (step->op) {
        case CLK_SET_FREQ:
        case CLK_SET_FACTOR:
        case CLK_SET_PARENT:
        case CLK_SET_PARENT_IDX:
        case CLK_ENABLE:
        case CLK_DISABLE:
            return true;
        case CLK_CONFIG_TARGET:
        case BUSY_SPIN:
        case SET_LED:
            return false;
        default:
            printf("Illegal sequence step type!\n");
            return false;
        }
}

static clk_topology_entry_t *_get_clock_conf_from_tree_conf(const gclk_t *clk, clk_topology_entry_t *tree_conf, size_t tree_clock_cnt) {
    for (unsigned i = 0; i < tree_clock_cnt; i++) {
        if (tree_conf[i].clk == clk) {
            return &tree_conf[i];
        }
    }
    return NULL;
}

/* if either uptree_parent or chid are NULL, this returns false. */
static bool _gclk_manager_is_derived_from_clock(const gclk_t *uptree_parent, const gclk_t *child, clk_topology_entry_t *tree_conf, size_t tree_clock_cnt) {

    if ((uptree_parent == NULL) || (child == NULL)) {
        return false;
    } else if (uptree_parent == child) {
       return true;
    }

    clk_topology_entry_t *tmp_conf = _get_clock_conf_from_tree_conf(child, tree_conf, tree_clock_cnt);

    while (true) {
        if (gclk_is_source(tmp_conf->clk)) {
            return false;
        }
        const gclk_t *parent = gclk_idx2parent(tmp_conf->clk, tmp_conf->par_idx);
        if (parent == NULL) {
            return false;
        }
        if (parent == uptree_parent) {
            return true;
        }
        tmp_conf = _get_clock_conf_from_tree_conf(parent, tree_conf, tree_clock_cnt);
    }
}

/* returns the root source conf along the quivalent factors for the path from cc to its source */
static clk_topology_entry_t *_model_get_equivalent_uptree_factors(clk_topology_entry_t *tree_model, size_t tree_size, clk_topology_entry_t *cc, uint32_t *mul, uint32_t *div) {
    uint32_t m = 1;
    uint32_t d = 1;
    while (true) {
        /* handle the special case where a clock config strictly depends on an uptree node.
         * Since we are not considering the current state of config registers but the virtual memory instead , the
         * method to get the respective factor for the clock needs to have access to te clock tree state that is investigated */
        if (cc->clk->flags.topology_flags & GCLK_STRICT_UPTREE_DEPENDENT) {
            /* update the dependent clock config state based on the tree state */
            cc->factor = gclk_get_uptree_dependent_factor(cc->clk, tree_model, tree_size);
        }
        if (gclk_is_divider(cc->clk)) {
            LOG_DEBUG("%s: div of %s: %u\n", __FUNCTION__, gclk_get_name(cc->clk), cc->factor);
            d *= cc->factor;
        } else if (gclk_is_multiplier(cc->clk)) {
            LOG_DEBUG("%s: mul of %s: %u\n", __FUNCTION__, gclk_get_name(cc->clk), cc->factor);
            m *= cc->factor;
        } else {
            LOG_DEBUG("%s: %s is no scaler!\n", __FUNCTION__, gclk_get_name(cc->clk));
        }

        if (gclk_is_source(cc->clk)) {
            break;
        }
        const gclk_t *pclk = gclk_idx2parent(cc->clk, cc->par_idx);
        /* if a clock points to NULL as parent that means this subtree is completely disconnected.
         * E.e., the equivalent mul factor becomes zero for all clocks downtree */
        if (pclk != NULL) {
            cc = _get_clock_conf_from_tree_conf(pclk, tree_model, tree_size);
        } else {
            m = 0;
            break;
        }
    };

    *mul = m;
    *div = d;
    return cc;
}

static uint32_t _get_minmax_freq_at_fixed_uptree_conf(clk_topology_entry_t *topology, uint32_t len, bool max) {

    if (!len || !topology[0].clk) {
        LOG_DEBUG("%s: max freq of an empty topology or NULL is always 0\n", __FUNCTION__);
        return 0;
    }

    uint32_t m = 1;
    uint32_t d = 1;

    clk_topology_entry_t *root_conf;
    const gclk_t *clk = topology[0].clk;

    if (len > 1) {
        root_conf = _model_get_equivalent_uptree_factors(&topology[1], len - 1, &topology[0], &m, &d);
    } else {
        root_conf = &topology[0];
    }

    uint32_t root_input_freq = gclk_get_input_freq(root_conf->clk);

    if (!root_input_freq) {
        return 0;
    }

    if ((gclk_is_divider(clk) && max) ||
        (gclk_is_multiplier(clk) && !max)) {
        d *= gclk_factor_min(clk);
    } else if ((gclk_is_multiplier(clk) && max) ||
               ((gclk_is_divider(clk) && !max))) {
        m *= gclk_factor_max(clk);
    }

    return root_input_freq * m / d;
}

static void _model_update_to_parent_changes(clk_topology_entry_t *tree_model, size_t tree_size, clk_topology_entry_t *child_conf) {
    LOG_DEBUG("%s: update model to parent changes...\n", __FUNCTION__);
    uint32_t mul = 1;
    uint32_t div = 1;
    clk_topology_entry_t *root_conf = _model_get_equivalent_uptree_factors(tree_model, tree_size, child_conf, &mul, &div);
    LOG_DEBUG("%s: root_conf of %s is %s\n", __FUNCTION__, gclk_get_name(child_conf->clk), gclk_get_name(root_conf->clk));
    LOG_DEBUG("%s: equiv mul is %lu\n", __FUNCTION__, mul);
    LOG_DEBUG("%s: equiv div is %lu\n", __FUNCTION__, div);
    uint32_t new_freq;
    //TODO: optimize calculation based on actual values (e.g. very high mul/div vals..)
    if (mul == 0) {
        new_freq = 0;
    } else {
        new_freq = root_conf->clk_freq * mul / div;
    }
    LOG_DEBUG("%s: updating freq of %s from %lu to %lu\n", __FUNCTION__, gclk_get_name(child_conf->clk), child_conf->clk_freq, new_freq);
    child_conf->clk_freq = new_freq;
}

/* returns true if new propagations were marked */
static clk_topology_entry_t *_mark_pending_propagation_on_children(clk_topology_entry_t *changed_conf, clk_topology_entry_t *tree_model, size_t tree_model_size) {
    clk_topology_entry_t *next = NULL;
    for (unsigned i = 0; i < tree_model_size; i++) {
        /* mark all children of the changed clock. (which is never a source clock) */
        if (!gclk_is_source(tree_model[i].clk)) {
            if (gclk_idx2parent(tree_model[i].clk, tree_model[i].par_idx) == changed_conf->clk) {
                LOG_DEBUG("%s: mark %s for propagation update\n", __FUNCTION__, gclk_get_name(tree_model[i].clk));
                tree_model[i].propagation_pending = true;
                /* save the first encountered children to return it for further propagation */
                if (!next) {
                    next = &tree_model[i];
                }
            }
        }
    }
    return next;
}

static clk_topology_entry_t *_find_next_pending_propagation(clk_topology_entry_t *tree_model, size_t tree_model_size) {
    for (unsigned i = 0; i < tree_model_size; i++) {
        clk_topology_entry_t *c = &tree_model[i];
        if (c->propagation_pending) {
            return c;
        }
    }
    return NULL;
}

static void _model_propagate_conf_change_downtree(clk_topology_entry_t *changed_conf, clk_topology_entry_t *tree_model, size_t tree_model_size) {
    clk_topology_entry_t *pc = changed_conf;
    pc->propagation_pending = true;
    while (pc) {
        /* update the currently selected pending clock */
        _model_update_to_parent_changes(tree_model, tree_model_size, pc);

        /* check if there are further pending propagations */
        clk_topology_entry_t *next = _mark_pending_propagation_on_children(pc, tree_model, tree_model_size);

        /* clear pending propagation after updating its state and marking all its children for propagation */
        pc->propagation_pending = false;

        /* if we found another downtree node affected by this change continue propagation there */
        if (next != NULL) {
            pc = next;
        } else {
            /* otherwise check if there is another pending change somewhere in the tree */
            pc = _find_next_pending_propagation(tree_model, tree_model_size);
            if (pc == NULL) {
                LOG_DEBUG("%s: no further pending config found! -> finished propagation!\n", __FUNCTION__);
            }
        }
    }
}

/* Some kind of transaction mechanism is needed for when multiple callbacks modify the requirements for WS/Vcore
 * in a contradicting way.
 * Either :
 * - some state must be handed to the callbacks so they can determine if this is the final change_cb
 * - the number of pending (relevant) cbs must be evaluated on the global view so that a flag
 *   like "commit changes" can be handed to the last relevant cb
 * - a concept of multi-level cbs could be used to first notify changes to distribute required information
 *   and then a commit cb is called were the final consistent state (updated by previous level notify cbs)
 *   can be considered to perform required updates.
 * - For the special case of flash WS/vcore updates a separate callback like this one may be explicitly called
 *   after all callbacks were issued.
 *
 * Note: with step-wise notifications this would not be needed but additional update steps would likely introduce
 *       back-and-forth updates during complex sequences.
 *
 * Note: as a performance improvement measure a parameter could indicate whether any cbs were executed before.
 *       Alternatively it could be made convention to only call if cbs were executed in a reconfiguration.
 */
static void _post_notify_commit(bool post_change) {
    if (auto_vscale_enabled || auto_wsadapt_enabled) {
        unsigned min_ws;
        unsigned min_vc_idx;

        gclk_get_min_required_ws_vc_from_tree_config(constrained_clocks_conf_cache, GCLK_FREQ_LIMIT_CLKS_NUMOF, &min_ws,  &min_vc_idx, _mgr_ctx.dvs_policy);

        if (auto_wsadapt_enabled &&
            ((!post_change &&  (flash_opt_get_wait_states() < min_ws)) ||
             (post_change && (flash_opt_get_wait_states() > min_ws)))) {
            flash_opt_set_wait_states(min_ws);
        }

        if (auto_vscale_enabled &&
            ((!post_change &&  ((unsigned)core_voltage_get() < min_vc_idx)) ||
             (post_change && ((unsigned)core_voltage_get() > min_vc_idx)))) {
            core_voltage_set(min_vc_idx);
        }
    }
}

/* the vcore value is fed into the calculation as mV*V value which in worst case becomes
 * 1200 * 1.2 = 1440.
 * shifting down by 11 (division by 2048) therefore provides enough space to accomodate the voltage value */
#define CLOCK_MANAGER_PM_PRE_VCORE_MUL_SHIFT (11)

static uint32_t _get_system_consumption_nW_from_power_model(clk_topology_entry_t *topo_conf, size_t len) {
    uint32_t p_sum_nW = 0;

    unsigned min_ws;
    unsigned min_vc;
    /* we are interested in p_min so always opt for low voltage policy */
    gclk_get_min_required_ws_vc_from_tree_config(topo_conf, len, &min_ws, &min_vc, DVS_PREFER_LOW_VOLTAGE);

    uint32_t vcore_mv = core_voltage_idx2mv(min_vc);

    /* the voltage value is squared and held in unit "mV * V" to keep the precision but still reduce the bit-width */
    uint32_t vcore_mVV = vcore_mv * vcore_mv / 1000;

    for (unsigned i = 0; i < len; i++) {
#if GLOBAL_CLOCK_POWER_MODEL_AVAILABLE > 0
        for (unsigned x = 0; x < GLOBAL_CLOCK_POWER_MODEL_PROPERTIES_NUMOF; x++) {
            if (topo_conf[i].clk == clock_power_model[x].clk) {
                /* scaled freq unit is 10kHz */
                uint32_t f_clk_scaled = topo_conf[i].clk_freq / 10000;
                //TODO: ensure the enabled state is also initialized on topology brute force settings that do not
                //      use the derive sequence method, then the enabed flag can be used directly instead of a freq check
                //uint32_t p_clk_nW = topo_conf[i].clk_freq > 0 ? clock_power_model[x].P_en_nW : 0;
                /* in this context (a topo_cmp function) we only compare active topologies. I.e. topologies where every clock is enabled.
                 * Therefore we can always add the clock specific static consumption if a clock is present in the given topology */
                uint32_t p_clk_nW = clock_power_model[x].P_en_nW;
                uint32_t C_fF = clock_power_model[x].C_fF;
                uint32_t p_clk_dyn_shifted =  ((f_clk_scaled * C_fF) >> CLOCK_MANAGER_PM_PRE_VCORE_MUL_SHIFT) * vcore_mVV;

                // ------ calculation step --------|------------------- operation ----------|-- effective combined operation ---|
                // scale mV*V to V*V               | / 1000;                                |  / 1000
                // scale fF to nF                  | / 1000000;                             |  / 1000000000
                // undo freq scaling, 10 kHz -> Hz | * 10000;                               |  / 100000
                // undo pre-vcore mul shift        | * (2^11); i.e. * 2048; i.e. * 32 * 64; |  / 3125 * 64
                // The unit after this operation is Hz * nF * V * V.  i.e., nW
                p_clk_dyn_shifted = p_clk_dyn_shifted / 3125 * 64;

                p_sum_nW += (p_clk_dyn_shifted + p_clk_nW);

                // found the power property for this clock -> stop iteration
                break;
            }
        }
#else
        (void)topo_conf;
        (void)vcore_mVV;
#endif
    }
    return p_sum_nW + GCLK_MANAGER_CONF_SYS_PSTATIC_NW;
}

static bool _contains(unsigned *list, size_t len, unsigned elem) {
    for (unsigned i = 0; i < len; i++) {
        if (list[i] == elem) {
            return true;
        }
    }
    return false;
}

static size_t _append_if_not_contained(unsigned *list, size_t len, unsigned val) {
    if (!_contains(list, len, val)) {
        list[len] = val;
        return 1;
    }
    return 0;
}

static unsigned _get_unique_topo_cnt(const gclk_manager_topo_switch_desc_t *sds, unsigned cnt) {
    /* as data is encoded only in transitions (edges between topologies),
     * we assume the worst case number of possible topologies first */
    unsigned topos[cnt * 2];
    unsigned topo_cnt = 0;

    for (unsigned i = 0; i < cnt; i++) {
        topo_cnt += _append_if_not_contained(topos, topo_cnt, sds[i].src_topo_id);
        topo_cnt += _append_if_not_contained(topos, topo_cnt, sds[i].dst_topo_id);
    }

    return topo_cnt;
}

static int _get_elem_idx(unsigned *list, size_t len, unsigned elem) {
    for (unsigned i = 0; i < len; i++) {
        if (list[i] == elem) {
            return i;
        }
    }

    return -1;
}

/* TODO: this can still be heavily simplified and improved (splitting code, using bitmasks to avoid iterations etc.) */
/* topo_cnt must be the number of possible topologies the leaf clock can be set to. This value also forms an upper bound
 * on how many sequences any transition may take in the worst case. seq_chain *must* be have enough capacity to take
 * up to topo_cnt-1 elements. */
int _derive_sequence_chain(int stid, int ttid, unsigned *seq_chain, size_t topo_cnt) {
    unsigned unique_tids = _get_unique_topo_cnt(core_clk_topo_switch_descs, CORE_CLOCK_TOPO_SWITCH_DESC_NUMOF);

    /* stores the mapping from unique topology numbers (0 to n) for every topology id (e.g. 2,6,8) */
    unsigned tidx_tid[unique_tids];
    unsigned added = 0;
    for (unsigned i = 0; i < CORE_CLOCK_TOPO_SWITCH_DESC_NUMOF; i++) {
        added += _append_if_not_contained(tidx_tid, added, core_clk_topo_switch_descs[i].src_topo_id);
        added += _append_if_not_contained(tidx_tid, added, core_clk_topo_switch_descs[i].dst_topo_id);
        if (added == unique_tids) {
            break;
        }
    }

    int tid_graph[unique_tids][unique_tids];
    for (unsigned i = 0; i < unique_tids; i++) {
        for (unsigned j = 0; j < unique_tids; j++) {
            tid_graph[i][j] = -1;
        }
    }

    /* transform the transition-based encoding into a graph of topology ids */
    for (unsigned i = 0; i < CORE_CLOCK_TOPO_SWITCH_DESC_NUMOF; i++) {
        unsigned sid = core_clk_topo_switch_descs[i].src_topo_id;
        unsigned sidx = _get_elem_idx(tidx_tid, unique_tids, sid);

        for (unsigned x = 0; x < CORE_CLOCK_TOPO_SWITCH_DESC_NUMOF; x++) {
            /* for all edges coming from the source topology populate all topologies that are reachable */
            if (sid == core_clk_topo_switch_descs[x].src_topo_id) {
                unsigned did = core_clk_topo_switch_descs[x].dst_topo_id;
                unsigned didx = _get_elem_idx(tidx_tid, unique_tids, did);
                /* for now we use the same wheight everywhere, more complex approaches could consider different weights
                 * depending on how complex the transitions are and how this is affected by the current configuration/constraints */
                tid_graph[sidx][didx] = 1;
            }
        }
    }

    /* distance form start topology to each other topology */
    unsigned dist[unique_tids];

    /* will hold the shortest path to reach ttid from stid */
    int path[unique_tids];

    /* markers to store which node was already visited */
    int visited[unique_tids];

    for (unsigned i = 0; i < unique_tids; i++) {
        dist[i] = 0xFFFFFFFE;
        visited[i] = 0;
        path[i] = -1;
    }

    /* get graph table idx of start topology idx */
    unsigned sidx = _get_elem_idx(tidx_tid, unique_tids, stid);

    /* distance from start to itself is always 0 */
    dist[sidx] = 0;

    int current = sidx;

    /* list of nodes to be visited */
    int node_queue[unique_tids];
    unsigned queuelen = 0;

    while (true) {

        visited[current] = 1;

        for (unsigned i = 0; i < unique_tids; i++) {
            /* for all reachable children */
            if (tid_graph[current][i] != -1) {
                if (visited[i]) {
                    continue;
                } else {
                    node_queue[queuelen++] = i;

                    /* for now we assume same weight of 1 everywhere */
                    unsigned cur_i_dist = 1;

                    /* calculate distance from 'start' to 'i' via 'current' */
                    unsigned s_dist = dist[current] + cur_i_dist;

                    /* check if path via 'current' is shorter than previous best path */
                    if (s_dist < dist[i]) {
                        dist[i] = s_dist;
                        /* shortest path to 'i' is via 'current' */
                        path[i] = current;
                    }
                }
            }
        }

        /* remove 'current' from queue */
        for (unsigned i = 0; i < queuelen; i++) {
            if (node_queue[i] == current) {
                /* move entries down if there are any after current */
                for (unsigned x = i; x < (queuelen - 1); x++) {
                    node_queue[x] = node_queue[x+1];
                }
                queuelen--;
                break;
            }
        }

        if (queuelen == 0) {
            /* no nodes left to be visited */
            break;
        }

        unsigned min_dist = 0xFFFFFFFE;
        unsigned index = 0;

        for (unsigned i = 0; i < queuelen; i++) {
            if (dist[node_queue[i]] < min_dist) {
                index = node_queue[i];
            }
        }
        current = index;
    }

    unsigned didx = _get_elem_idx(tidx_tid, unique_tids, ttid);

    if (dist[didx] != 0xFFFFFFFE) {
        unsigned seq_steps = dist[didx];

        if (seq_steps > (topo_cnt - 1)) {
            /* can not store sequence chain in destination buffer */
            return -1;
        }

        unsigned tidxs[seq_steps +1];
        unsigned pos = seq_steps -1;

        for (unsigned i = path[didx];; i = path[i]) {
            tidxs[pos--] = i;
            if (i == sidx) {
                break;
            }
        }
        tidxs[seq_steps] = didx;

        for (unsigned i = 0; i < seq_steps; i++) {
            unsigned src_tid = tidx_tid[tidxs[i]];
            unsigned dst_tid = tidx_tid[tidxs[i+1]];
            for (unsigned x = 0; x < CORE_CLOCK_TOPO_SWITCH_DESC_NUMOF; x++) {
                if ((core_clk_topo_switch_descs[x].src_topo_id == src_tid) &&
                    (core_clk_topo_switch_descs[x].dst_topo_id == dst_tid)) {
                    seq_chain[i] = x;
                }
            }
        }

        return seq_steps;
    }

    /* no feasible sequence chain found */
    return -2;
}

static bool _can_be_changed_otf(const clk_topology_entry_t *src_topo, uint32_t src_len) {
    for (unsigned i = 0; i < src_len; i++) {
        uint32_t flags = src_topo[i].clk->flags.topology_flags;
        /* TODO this is a rather simplified check that is pretty conservative as these flags do not nessesarily
         * impose restirictions on clocks far down or up the clock (where the actual change takes place).
         * A more reasonable approach would be to check those flags only against a "topology diff".
         * NOTE: if this is addressed the below method to derive a same-to-same topology config sequence
         * must be updated to not run into temporary invalid configs see the other TODO down there */
        if ((flags & GCLK_STOP_CHILDREN_FOR_UPDATE) ||
                (flags & GCLK_STOP_PARENT_FOR_UPDATE) ||
                (flags & GCLK_STOP_FOR_UPDATE)) {
            return false;
            LOG_DEBUG("%s: [%s] prohibits OTF update\n", __FUNCTION__, gclk_get_name(src_topo[i].clk));
        }
    }
    return true;
}

static const gclk_manager_topo_switch_desc_t *_get_intermediate_topo_switch_desc(unsigned int src_tid) {
    const gclk_manager_topo_switch_desc_t *desc = NULL;
    for (unsigned i = 0; i < CORE_CLOCK_TOPO_SWITCH_DESC_NUMOF; i++) {
        /* direct transition available to some other topology */
        if ((core_clk_topo_switch_descs[i].src_topo_id == src_tid) &&
            (core_clk_topo_switch_descs[i].dst_topo_id != src_tid)) {
            desc = &core_clk_topo_switch_descs[i];
            //return desc; // should be valid - but must be tested as it changes behavior
        }
    }
    return desc;
}

void gclk_manager_get_combined_topology_fraction(clk_topology_entry_t *topo, size_t topo_len, gclk_fraction_t *f) {
    uint32_t m = 1;
    uint32_t d = 1;

    for (unsigned i = 0; i < topo_len; i++) {
        if (gclk_is_divider(topo[i].clk)) {
            d *= topo[i].factor;
        } else if (gclk_is_multiplier(topo[i].clk)) {
            m *= topo[i].factor;
        }
    }

    f->n = m;
    f->d = d;
}

int gclk_mananger_set_dfs_frequencies(const uint32_t *freqs, size_t cnt) {
    /* reset dfs count before setting new values */
    _mgr_ctx.dfs_frequencies_cnt = 0;

    if (!_mgr_ctx.active_core_scale_setting) {
        printf("no active core scale setting defined\n");
        return -1;
    }

    if (cnt > MAX_DFS_FREQ_VALUES_NUM) {
        printf("Warning: can not cache %u frequency configs, will limit to at most %u entries\n",
                cnt, MAX_DFS_FREQ_VALUES_NUM);
    }

    uint64_t t_1 = xtimer_now_usec64();
    int res = _populate_dfs_freqs_bf(_mgr_ctx.active_core_scale_setting, freqs, cnt);
    uint64_t t_2 = xtimer_now_usec64();

    uint32_t t_populate_dfs = (uint32_t)((t_2 - t_1)/1000);

    printf("took %lu ms for populate\n", t_populate_dfs);

    _mgr_ctx.dfs_frequencies_cnt = res > 0 ? res : 0;

    return res;
}

int gclk_mananger_set_default_dfs_frequencies(void) {
    if (!_mgr_ctx.active_core_scale_setting) {
        printf("no active core scale setting defined\n");
        return 0;
    }
    uint64_t t_1 = xtimer_now_usec64();
    gclk_manager_setup_default_dfs_topo_conf(_mgr_ctx.active_core_scale_setting);
    uint64_t t_2 = xtimer_now_usec64();

    uint32_t t_default_topo_setup = (uint32_t)((t_2 - t_1)/1000);

    printf("took %lu ms for default topo setup\n", t_default_topo_setup);

    int res = gclk_mananger_set_dfs_frequencies(_mgr_ctx.active_core_scale_setting->default_freqs, _mgr_ctx.active_core_scale_setting->default_freqs_cnt);
    return res;
}

const gclk_t* gclk_manager_get_core_clock_handle(void) {
    return gclk_core_clock_handle;
}

void gclk_manager_default_stdio_reinit_cb(const gclk_t* altered_clk, const gclk_t* affected_clk, uint32_t f_old, uint32_t f_new, bool post_change) {
    (void)altered_clk;
    (void)affected_clk;
    (void)f_old;
    (void)f_new;
    if (post_change) {
        stdio_init();
    } else {
        /* Must be done (at least) on nucleo-l476rg because stdio_init powers on the uart again, which blocks a PM mode which in turn
         * increments the PM-block registrations, which overflows if there is no balancing number of PM-unblocks by shutting down the uart */
        uart_poweroff(STDIO_UART_DEV);
    }
}

void gclk_manager_default_timer_reinit_cb(const gclk_t* altered_clk, const gclk_t* affected_clk, uint32_t f_old, uint32_t f_new, bool post_change) {
    (void)altered_clk;
    (void)affected_clk;
    (void)f_old;
    (void)f_new;
    if (post_change) {
        timer_init(CONFIG_ZTIMER_USEC_DEV, CONFIG_ZTIMER_USEC_BASE_FREQ, __ztimer_perph_timer_cb, ___ztimer_periph_timer_instance);
        /* TODO: add compensation value based on independently clocked timer */
        timer_write(CONFIG_ZTIMER_USEC_DEV, _timer_cnt_backup);
    } else {
        /* backup current timer value for restoration after re-init */
        _timer_cnt_backup  = timer_read(CONFIG_ZTIMER_USEC_DEV);
        /* stop the timer during frequency updates */
        timer_stop(CONFIG_ZTIMER_USEC_DEV);
    }
}

void gclk_update_ws_vc_limits(const gclk_t *clk, uint32_t freq,
                              unsigned *min_ws_ff,  unsigned *min_vc_ff, unsigned *min_ws_lv, unsigned *min_vc_lv) {
    for (unsigned x = 0; x < GCLK_FREQ_LIMIT_CLKS_NUMOF; x++) {
        if (gclk_freq_conf_limits[x].clk == clk) {
            unsigned clk_min_ws_ff = 255;
            unsigned clk_min_vc_ff = 255;
            unsigned clk_min_ws_lv = 255;
            unsigned clk_min_vc_lv = 255;

            bool ws_found = false;
            bool vc_found = false;
            /* iterate over all limits for this clock */
            for (unsigned li = 0; li < gclk_freq_conf_limits[x].len; li++) {
                const freq_conf_limit_t *lim = &gclk_freq_conf_limits[x].limits[li];
                /* if the limit applies to the given frequency */
                if (lim->freq_max >= freq) {
                    if (!ws_found || (lim->ws_min < clk_min_ws_ff) || ((lim->ws_min == clk_min_ws_ff) && (lim->vc_idx_min < clk_min_vc_ff)) ) {
                        clk_min_ws_ff = lim->ws_min;
                        clk_min_vc_ff = lim->vc_idx_min;
                        ws_found = true;
                    }
                    if (!vc_found || (lim->vc_idx_min < clk_min_vc_lv) || ((lim->vc_idx_min == clk_min_vc_lv) && (lim->ws_min < clk_min_ws_lv)) ) {
                        clk_min_ws_lv = lim->ws_min;
                        clk_min_vc_lv = lim->vc_idx_min;
                        vc_found = true;
                    }
                }
            }

            if ((clk_min_ws_ff > *min_ws_ff) || (clk_min_vc_ff > *min_vc_ff)) {
                *min_ws_ff = clk_min_ws_ff;
                *min_vc_ff = clk_min_vc_ff;
            }

            if ((clk_min_ws_lv > *min_ws_lv) || (clk_min_vc_lv > *min_vc_lv)) {
                *min_ws_lv = clk_min_ws_lv;
                *min_vc_lv = clk_min_vc_lv;
            }

            /* each limit entry carries all limits for one clock ao this can termiate early */
            break;
        }
    }
}

void gclk_get_min_required_ws_vc_from_tree_config(clk_topology_entry_t *tree_conf, size_t tree_size, unsigned *min_ws,
                                                  unsigned *min_vc_idx, gclk_manager_dvs_policy_t dvspolicy) {
    unsigned abs_req_min_ws_ff = 0;
    unsigned abs_req_min_vc_ff = 0;
    unsigned abs_req_min_ws_lv = 0;
    unsigned abs_req_min_vc_lv = 0;

    for (size_t i = 0; i < tree_size; i++) {
        clk_topology_entry_t *tc = &tree_conf[i];
        gclk_update_ws_vc_limits(tc->clk, tc->clk_freq,
                &abs_req_min_ws_ff,  &abs_req_min_vc_ff, &abs_req_min_ws_lv, &abs_req_min_vc_lv);
    }

    bool optimize_ws = dvspolicy == DVS_PREFER_FAST_FLASH ? true : false;

    if (optimize_ws) {
        /* if the lv variant results in better or equal values use that one */
        if ((abs_req_min_ws_lv <= abs_req_min_ws_ff) &&
                (abs_req_min_vc_lv <= abs_req_min_vc_ff)) {
            *min_ws = abs_req_min_ws_lv;
            *min_vc_idx = abs_req_min_vc_lv;
        } else {
            *min_ws = abs_req_min_ws_ff;
            *min_vc_idx = abs_req_min_vc_ff;
        }
    } else {
        /* if the ff variant results in better or equal values use that one */
        if ((abs_req_min_ws_ff <= abs_req_min_ws_lv) &&
                (abs_req_min_vc_ff <= abs_req_min_vc_lv)) {
            *min_ws = abs_req_min_ws_ff;
            *min_vc_idx = abs_req_min_vc_ff;
        } else {
            *min_ws = abs_req_min_ws_lv;
            *min_vc_idx = abs_req_min_vc_lv;
        }
    }
}

int gclk_manager_init(void) {

    /* TODO: check for all of these if the initialization can be dropped
     * (e.g. the first one should be set up on DFS enable anyway) */
    _mgr_ctx.current_dfs_freq_idx = MAX_DFS_FREQ_VALUES_NUM - 1;
    _mgr_ctx.active_core_scale_setting = NULL;
    _mgr_ctx.dfs_frequencies_cnt = 0;
    _mgr_ctx.pre_sched_freq_boost_threshold = 60;
    _mgr_ctx.pre_sched_freq_throttle_threshold = 30;
    _mgr_ctx.pre_sched_pu_dfs_enabled = false;
    _mgr_ctx.pu_metadata_collection_enabled = false;
    _mgr_ctx.cpu_util_based_dvfs_enabled = false;
    _mgr_ctx.dvs_policy = DVS_PREFER_LOW_VOLTAGE;

    _sched_stats.idle_ticks_min = 0xFFFFFFFF;
    _sched_stats.idle_ticks_max = 0;
    _sched_stats.busy_ticks_min = 0xFFFFFFFF;
    _sched_stats.busy_ticks_max = 0;

    _mgr_ctx.max_clocks_in_topology = gclk_get_max_topology_depth();
    _mgr_ctx.max_clocks_in_core_topology = gclk_get_clk_subtree_max_depth(gclk_core_clock_handle, 0) + 1;


    _init_dvs_wsa_constraint_cache();

    _update_cached_state_vars();

    //gclk_manager_setup_default_dfs_topo_conf(active_core_scale_setting);

    gclk_mananger_set_default_dfs_frequencies();

    gclk_manager_platform_init();

    return 0;
}

int gclk_manager_get_scale_settings(const gclk_scale_setting_t **s) {
  *s = &scale_settings[0];
  return SCALE_SETTINGS_NUMOF;
}

bool gclk_mananger_set_active_scale_setting(unsigned i) {
    if ((SCALE_SETTINGS_NUMOF > i) &&
        scale_settings[i].topology_id == _mgr_ctx.current_core_topo_id) {
        _mgr_ctx.active_core_scale_setting = &scale_settings[i];
        gclk_mananger_set_default_dfs_frequencies();
        return true;
    }
    return false;
}

const gclk_scale_setting_t* gclk_mananger_get_active_scale_setting(void) {
    return _mgr_ctx.active_core_scale_setting;
}

int gclk_manager_get_allowed_core_clock_sources(const gclk_t ***clks) {
  *clks = &core_clock_sources[0];
  return CORE_CLOCK_SOURCES_NUMOF;
}

void gclk_manager_set_dvfs_pu_params(uint32_t fboost, uint32_t fthrottle, int fboost_pu_th, int fthrottle_pu_thresh) {
    _mgr_ctx.pre_sched_boost_freq = fboost;
    _mgr_ctx.pre_sched_throttle_freq = fthrottle;
    _mgr_ctx.pre_sched_freq_boost_threshold = fboost_pu_th;
    _mgr_ctx.pre_sched_freq_throttle_threshold = fthrottle_pu_thresh;
}

unsigned int gclk_manager_get_dfs_freqs(uint32_t **freqs) {
    *freqs = &_mgr_ctx.dfs_frequencies[0];
    return _mgr_ctx.dfs_frequencies_cnt;
}

gclk_cmp_result_t gclk_manager_cmp_single_scaler_range_limited(clk_topology_entry_t *topo_best,
                            size_t len1, clk_topology_entry_t *topo_cmp, size_t len2, void *arg) {
    range_limit_cmp_fun_ctx_t *ctx = (range_limit_cmp_fun_ctx_t*)arg;

    (void)topo_best;
    (void)len1;

    /* the particular factor of the scaled clock is not really relevant in this context since the search
     * is not for a single factor (but a range of factors which should all apply to the given config).
     * Further down there is a check that considers all appliccable factors so skip all but one factor
     * to perform the more complex check only on a subset of the proposed configs. The other factors will
     * still be implicitly considered by the check below. */
    if (topo_cmp[ctx->scale_clk_topo_idx].factor != ctx->scaler_factor_target) {
        return GCLK_CONF_INVALID;
    }
    if (!gclk_freq_within_limit(topo_cmp[ctx->scale_clk_topo_idx].clk_freq, &ctx->scaler_fo_limits)) {
        return GCLK_CONF_INVALID;
    }

    uint32_t err = _abs_diff(ctx->target_freq, topo_cmp[0].clk_freq);

    /* prefer configs that are closer to the targeted frequency */
    if(err <= ctx->min_error) {
        ctx->min_error = err;

        uint32_t infeasible = 0;
        uint32_t sc_backup_freq = topo_cmp[ctx->scale_clk_topo_idx].clk_freq;
        uint32_t sc_backup_factor = topo_cmp[ctx->scale_clk_topo_idx].factor;

        /* to evaluate how good this config is suitable for dfs with a single scaler instance, some
         * more complex checks are performed so configs that enable more frequency options are preferred */
        for (unsigned i = 0; i < gclk_factor_cnt(ctx->scale_clk); i++) {
            uint32_t factor = gclk_idx2factor(ctx->scale_clk,i);
            if ((factor >= ctx->scaler_factor_limits.min) &&
                (factor <= ctx->scaler_factor_limits.max)) {

                uint32_t f_sclr = _apply_scale_factor(ctx->scale_clk, factor, topo_cmp[ctx->scale_clk_topo_idx + 1].clk_freq);

                /* update the proposed topology config at the scaler clock position to determine the scaling effects
                 * NOTE: this must be undone before returning! */
                topo_cmp[ctx->scale_clk_topo_idx].clk_freq = f_sclr;
                topo_cmp[ctx->scale_clk_topo_idx].factor = factor;
                _model_propagate_conf_change_downtree(&topo_cmp[ctx->scale_clk_topo_idx], topo_cmp, len2);

                if (gclk_manager_conf_breaks_constraint(global_clock_constraints, GLOBAL_CLOCK_CONSTRAINTS_NUMOF, topo_cmp, len2)) {
                    infeasible++;
                }
            }
        }

        /* restore state before feasibility check */
        topo_cmp[ctx->scale_clk_topo_idx].clk_freq = sc_backup_freq;
        topo_cmp[ctx->scale_clk_topo_idx].factor = sc_backup_factor;
        _model_propagate_conf_change_downtree(&topo_cmp[ctx->scale_clk_topo_idx], topo_cmp, len2);

        if (infeasible < ctx->min_infeasible_cnt) {
            ctx->min_infeasible_cnt = infeasible;
            return GCLK_CONF_BETTER;
        } else if (infeasible == ctx->min_infeasible_cnt) {
            gclk_cmp_result_t pmin_res = gclk_manager_cmp_topology_closest_leaf_freq_pmin(topo_best, len1,
                                                                                          topo_cmp, len2, &ctx->target_freq);
            if (pmin_res == GCLK_CONF_BETTER) {
                return GCLK_CONF_BETTER;
            }
       }
    }

    return GCLK_CONF_WORSE;
}

gclk_cmp_result_t gclk_manager_cmp_lowest_freq_list_abs_err(clk_topology_entry_t *topo_best, size_t len1,
                                                            clk_topology_entry_t *topo_cmp, size_t len2,
                                                            void *arg) {
    lflae_cmp_fun_ctx_t *ctx = (lflae_cmp_fun_ctx_t*)arg;

    (void)topo_best;
    (void)len1;

    gclk_fraction_t dtf;
    _get_equivalent_dt_factors(topo_cmp, len2, ctx->scale_clk, &dtf, false);

    uint32_t abs_err = 0;
    size_t possible_freq_cnt = gclk_factor_cnt(ctx->scale_clk);

    int srcidx = _clk_to_entry_idx(topo_cmp, len2, ctx->scale_clk);

    uint32_t input_freq = topo_cmp[srcidx+1].clk_freq;
    uint32_t possible_freqs[possible_freq_cnt];

    for (unsigned i = 0; i < possible_freq_cnt; i++) {
        uint32_t factor = gclk_idx2factor(ctx->scale_clk, i);
        possible_freqs[i] = _get_freq_for_factors(ctx->scale_clk, input_freq, &dtf, factor);
    }

    //uint32_t prev_freq = 0;
    for (unsigned i = 0; i < ctx->match_freqs_cnt; i++) {
        uint32_t min_diff = 0xFFFFFFFF;
        //uint32_t best_freq = 0;
        for (unsigned x = 0; x < possible_freq_cnt; x++) {
            uint32_t diff = (ctx->freqs[i] >= possible_freqs[x]) ? (ctx->freqs[i] - possible_freqs[x]) : (possible_freqs[x] - ctx->freqs[i]);
            if (diff < min_diff) {
                min_diff = diff;
                //best_freq = possible_freqs[x];
                //printf("matched freq for %lu: %lu\n", ctx->freqs[i], possible_freqs[x]);
                //ctx->matched_freqs[i] = possible_freqs[x];
            }
        }

        abs_err += min_diff;
        /* only consider errors of frequencies that are not filtered out as duplicate anyway */
        //if (i > 0) {
        //    if (prev_freq != best_freq) {
        //        abs_err += min_diff;
        //    }
        //}  else {
        //    abs_err += min_diff;
        //}
        //prev_freq = best_freq;
    }

    if (abs_err < ctx->lowest_err) {
        ctx->lowest_err = abs_err;
        return GCLK_CONF_BETTER;
    } else if (abs_err == ctx->lowest_err) {
        return GCLK_CONF_EQUAL;
    } else {
        return GCLK_CONF_WORSE;
    }
}

bool gclk_manager_setup_default_dfs_topo_conf(const gclk_scale_setting_t *scs) {
    /* params needed to run config exploration */
    uint32_t max_involved_clks = _mgr_ctx.max_clocks_in_core_topology;
    clk_topology_entry_t ttopo[max_involved_clks];
    int tid = scs->topology_id;
    size_t valid_cnt = 0;
    int force_nth = -1;

    size_t possible_freq_cnt = gclk_factor_cnt(scs->scale_clk);

    void *cmpctx;
    gclk_cmp_func_t cmp_func;
    uint32_t target_freq;

    size_t freqs_to_match_cnt = scs->default_freqs_cnt <= possible_freq_cnt ? scs->default_freqs_cnt : possible_freq_cnt;

    /* only required in case a DIRECT or UPTREE_RELATIVE approach is used and a specific set of default freqs is defined */
    lflae_cmp_fun_ctx_t lflaectx;
    lflaectx.freqs = scs->default_freqs;
    lflaectx.target_freqs_cnt = scs->default_freqs_cnt;
    lflaectx.match_freqs_cnt = freqs_to_match_cnt;
    lflaectx.lowest_err = 0xFFFFFFFF;
    lflaectx.scale_clk = scs->scale_clk;

    /* used if no default freqs are specified and available factors must be matched
     * to a set of actually feasible configs */
    range_limit_cmp_fun_ctx_t range_limit_ctx;
    range_limit_ctx.scale_clk = scs->scale_clk;
    range_limit_ctx.target_freq = DFS_CYCLER_MAX_FREQ;
    range_limit_ctx.min_error = 0xFFFFFFFF;

    if (scs->approach == SCALE_DIRECT ||
        scs->approach == SCALE_UPTREE_RELATIVE) {
        /* if no freq values are provided explicitly, derive a target config from the highest allowed DFS frequency and
         * all available factors of the single scaled clock. */
        if (scs->default_freqs == NULL || scs->default_freqs_cnt == 0) {
            int srcidx = _clk_to_entry_idx(_mgr_ctx.current_core_topology, _mgr_ctx.current_core_topolen, scs->scale_clk);

            /* get absolute input requirements for the topology fed by the scaled clock instance
             * This data is then used to rule out any configs for the input side (the scaler) that wont be able
             * to operate within these limits. */
            _get_minmax_applicable_topo_input_freq(_mgr_ctx.current_core_topology,
                                                   _mgr_ctx.current_core_topolen - (_mgr_ctx.current_core_topolen - srcidx),
                                                   &range_limit_ctx.scaler_fo_limits);

            /* factor limits of the uptree topology */
            gclk_fraction_t utf_min;
            gclk_fraction_t utf_max;

            /* get min max factors of the topology that feeds the scaler instance. In case any additional scalers
             * sit before the scaler which is used for dfs, the full operational range of the input side must be considered too,
             * before ruling out factors of the dfs scaler as infeasible */
            _get_minmax_equivalent_factors_of_topology(&_mgr_ctx.current_core_topology[srcidx + 1], _mgr_ctx.current_core_topolen - (srcidx + 1),
                                                       &utf_min, &utf_max);

            /* determine frequency boundaries of the uptree topology */
            uint32_t root_freq = _mgr_ctx.current_core_topology[_mgr_ctx.current_core_topolen-1].clk_freq;
            gclk_freq_limit_t scaler_f_in_limits = {
                .min = root_freq * utf_min.n / utf_min.d,
                .max = root_freq * utf_max.n / utf_max.d,
            };

            /* rule out factors that will never yield a feasible configuration based on the up- and down-tree boundaries*/
            _get_scale_factor_limits(scs->scale_clk, &scaler_f_in_limits, &range_limit_ctx.scaler_fo_limits, &range_limit_ctx.scaler_factor_limits);

            /* the start configuration of the scale-via-single-clock approach is selected
             * by maxing out the frequency range (so that the remaining factors are still able to scale the frequency) */
            if (gclk_is_multiplier(scs->scale_clk)) {
                range_limit_ctx.scaler_factor_target = range_limit_ctx.scaler_factor_limits.max;
            } else {
                range_limit_ctx.scaler_factor_target = range_limit_ctx.scaler_factor_limits.min;
            }

            /* NOTE: another check could use similar 'symbolic calculation' steps as above to pre-determine
             *       additional downtree limits for each involved clock instance based on the determined limits
             *       at the input side. */

            range_limit_ctx.scale_clk_topo_idx = srcidx;
            range_limit_ctx.min_infeasible_cnt = 0xFFFFFFFF;

            cmp_func = gclk_manager_cmp_single_scaler_range_limited;
            cmpctx = &range_limit_ctx;
        } else {
            cmp_func = gclk_manager_cmp_lowest_freq_list_abs_err;
            cmpctx = &lflaectx;
        }

    } else if (scs->approach == SCALE_INTERMEDIATE_TOPO_AUTO) {
        target_freq = DFS_CYCLER_MAX_FREQ;
        cmp_func = gclk_manager_cmp_topology_closest_leaf_freq_pmin;
        cmpctx = &target_freq;
    } else {
        printf("ERROR: default config setup for this approach not implemented!\n");
        return false;
    }

    uint32_t leaf_freq = gclk_manager_brute_force_freq_conf(gclk_core_clock_handle, ttopo, &max_involved_clks,
                                                            &tid, cmp_func, cmpctx, &valid_cnt, force_nth, NULL);

    if (leaf_freq == GCLK_INVALID_FREQ) {
        printf("ERROR: couldn't derive config for %lu Hz\n", leaf_freq);
        return false;
    } else {
        printf("derived %lu topo conf entries for running at %lu Hz with approach %s\n", max_involved_clks, leaf_freq, _approach2_str(scs->approach));
    }

    int seq_len = gclk_manager_derive_sequence(_mgr_ctx.current_core_topology, _mgr_ctx.current_core_topolen,
                                               ttopo, max_involved_clks, &_mgr_ctx.prepared_rescale_sequences[0][0],
                                               ARRAY_SIZE(_mgr_ctx.prepared_rescale_sequences[0]));
    if (seq_len > 0) {
        printf("derived sequence to switch to %lu Hz with %u steps\n", leaf_freq, seq_len);
        gclk_manager_run_sequence_with_notify(&_mgr_ctx.prepared_rescale_sequences[0][0], seq_len, false);
    } else {
        printf("derive seq res: %d\n", seq_len);
        return false;
    }

    return true;
}

void gclk_manager_enable_pu_assessment(bool enable) {
    _mgr_ctx.pu_metadata_collection_enabled = enable;
}

void gclk_manager_start_freq_cycler(unsigned int cycle_us, uint32_t min_schedules) {
    uint32_t initial_freq = gclk_get_current_freq(gclk_manager_get_core_clock_handle());

    fc_ctx.cpu_time_threshold_ticks = idle_timer_usecs_to_ticks(cycle_us);
    fc_ctx.thread_schedule_threshold = min_schedules;
    fc_ctx.freqs = _mgr_ctx.dfs_frequencies;
    fc_ctx.freq_cnt = _mgr_ctx.dfs_frequencies_cnt;
    fc_ctx.cur_freq_idx = 0;
    fc_ctx.freq_change_cb = _freq_change_scale_auto;
    mutex_init(&fc_ctx.done_mutex);

    /* indicate that all PU stats for the first frequency are still pending for each requested thread */
    fc_ctx.pu_stats_pending_cur_freq = fc_ctx.pu_stats_requested;

    /* lock the mutex so that the second lock call below will block till the freq cycle is finished
     * (gets unlocked by the cycler thread when done) */
    mutex_lock(&fc_ctx.done_mutex);

    /* set_up first frequency of the cycle */
    current_core_freq = fc_ctx.freqs[fc_ctx.cur_freq_idx];
    fc_ctx.freq_change_cb(current_core_freq);

    LOG_DEBUG("starting freq_cycle and waiting for it to finish...\n");
    fc_ctx.freq_cycle_enabled = true;

    mutex_lock(&fc_ctx.done_mutex);
    mutex_unlock(&fc_ctx.done_mutex);
    /* go back to default frequency via the manager freq change method instead of the freq cycler variant */
    _mgr_ctx.freq_change_cb(initial_freq);
    LOG_DEBUG("freq cycler done\n");
    current_core_freq = initial_freq;
}
uint32_t _append_performance_util_data(uint32_t task_id, uint32_t freq, uint32_t time_us);

static inline uint32_t _freq_interval_mean(uint32_t slot) {
    return _mgr_ctx.dfs_frequencies[slot];
}

int gclk_manager_calculate_pu_factor(uint32_t task_id, bool debug_print) {
    int32_t pu_sum = 0;
    int32_t pu_cnt = 0;

    /* For multiple frequency pairs the pu value is averaged across all pairs */
    for (unsigned a = 0; a < MAX_DFS_FREQ_VALUES_NUM; a++) {
        for (unsigned b = a + 1; b < MAX_DFS_FREQ_VALUES_NUM; b++) {
            /* only use valid data points */
            if ((_sched_stats.task_perf_util_data[task_id][a].schedules != 0) &&
                (_sched_stats.task_perf_util_data[task_id][b].schedules != 0)) {

                /* use the middle fo the frequency slot as value for computation */
                int32_t freq_a = _freq_interval_mean(a) / 1000;
                int32_t freq_b = _freq_interval_mean(b) / 1000;
                /* relative change in frequency */
                int32_t freq_inc_fact = freq_b * 100 / freq_a;

                int32_t ta = _sched_stats.task_perf_util_data[task_id][a].cpu_time_ticks;
                ta /= _sched_stats.task_perf_util_data[task_id][a].schedules;
                int32_t tb = _sched_stats.task_perf_util_data[task_id][b].cpu_time_ticks;
                tb /= _sched_stats.task_perf_util_data[task_id][b].schedules;
                int32_t t_dec_fact = ta * 100 / tb;
                int32_t t_diff = tb - ta;

                int32_t task_speedup = ta * 100 / tb - 100;
                int32_t f_speedup = freq_b * 100 / freq_a - 100;
                int32_t pu = task_speedup * 100 / f_speedup;

                if (debug_print) {
                    printf("\nf: %ld %ld %ld %ld %%\n", freq_a * 1000, freq_b * 1000, (freq_b - freq_a) * 1000, freq_inc_fact);
                    printf("t: %ld %ld %ld %ld %%\n", ta, tb, t_diff, t_dec_fact);
                    printf("cpu time: %ld %ld\n", _sched_stats.task_perf_util_data[task_id][a].cpu_time_ticks, _sched_stats.task_perf_util_data[task_id][b].cpu_time_ticks);
                    printf("schedules: %ld %ld\n", _sched_stats.task_perf_util_data[task_id][a].schedules, _sched_stats.task_perf_util_data[task_id][b].schedules);
                    printf("PU: %ld\n", pu);
                }
                pu_sum += pu;
                pu_cnt++;
            }
        }
    }

    /* save the PU value per task */
    _sched_stats.task_performance_util[task_id] = pu_sum / pu_cnt;
    return _sched_stats.task_performance_util[task_id];
}

void gclk_manager_enable_pu_stat_request_for_thread(kernel_pid_t tid) {
    fc_ctx.pu_stats_requested |= (1 << tid);
}

void gclk_manager_clear_performance_util_data(void) {
    for (unsigned t = 0; t < GCLK_MANAGER_PU_STATS_TASK_NUM; t++) {
        for (unsigned f = 0; f < MAX_DFS_FREQ_VALUES_NUM; f++) {
            _sched_stats.task_perf_util_data[t][f].cpu_time_ticks = 0;
            _sched_stats.task_perf_util_data[t][f].schedules = 0;
        }
    }

    fc_ctx.pu_stats_requested = 0;
    fc_ctx.pu_stats_pending_cur_freq = 0;
    _sched_stats.enter_idle_cnt = 0;
}

void gclk_manager_pre_sched_hook(kernel_pid_t next_thread) {
    if (_mgr_ctx.pu_metadata_collection_enabled) {
        _sched_stats.t_cur_thread_start = idle_timer_read();
    }
    if (_mgr_ctx.pre_sched_pu_dfs_enabled) {
        if (_sched_stats.task_performance_util[next_thread] >= _mgr_ctx.pre_sched_freq_boost_threshold &&
            _mgr_ctx.pre_sched_boost_freq != current_core_freq) {
            _mgr_ctx.freq_change_cb(_mgr_ctx.pre_sched_boost_freq);
            current_core_freq = _mgr_ctx.pre_sched_boost_freq;
        } else if (_sched_stats.task_performance_util[next_thread] <= _mgr_ctx.pre_sched_freq_throttle_threshold &&
            _mgr_ctx.pre_sched_throttle_freq != current_core_freq) {
            _mgr_ctx.freq_change_cb(_mgr_ctx.pre_sched_throttle_freq);
            current_core_freq = _mgr_ctx.pre_sched_throttle_freq;
        }
    }
}

void gclk_manager_post_sched_hook(kernel_pid_t desched_thread) {
    if (_mgr_ctx.pu_metadata_collection_enabled) {
        uint32_t busy_ticks = idle_timer_read() - _sched_stats.t_cur_thread_start;
        _append_performance_util_data(desched_thread, current_core_freq, busy_ticks);
        if (fc_ctx.freq_cycle_enabled) {
            /* only try to advance to next freq if enough stats were collected for each thread of interest */
            if (!fc_ctx.pu_stats_pending_cur_freq) {
                /* if there are more frequencies advance to next, otherwise indicate end of cycle */
                if (fc_ctx.cur_freq_idx < (fc_ctx.freq_cnt - 1)) {
                    fc_ctx.cur_freq_idx++;
                    uint32_t new_freq = fc_ctx.freqs[fc_ctx.cur_freq_idx];
                    fc_ctx.freq_change_cb(new_freq);
                    current_core_freq = new_freq;
                    /* set all previously requested thread pu stats to pending for the new freq */
                    fc_ctx.pu_stats_pending_cur_freq = fc_ctx.pu_stats_requested;
                } else {
                    /* disable freq cycle after all freqs were measured */
                    fc_ctx.freq_cycle_enabled = false;
                    mutex_unlock(&fc_ctx.done_mutex);
                }
            }
        }
    }
}

/* cases to consider for this hook:
   The CPU might be sleeping completely after this task and there might be no generic way to measure idle time
   The CPU might still have a clock signal, so we could use either a separate timer or a cycle counter to measure idle time

   @todo: issue an upscaling callback when idle is not reached within a dynamically set boundry */
void gclk_manager_on_idle_hook(void) {
    //CYCCNT_pre = DWT->CYCCNT;
    _sched_stats.t_went_idle = idle_timer_read();

    /* cancel the critical busy time if not already happened busy-callback */
    if (_sched_stats.enter_idle_cnt) {
        _sched_stats.busy_ticks = _sched_stats.t_went_idle - _sched_stats.t_left_idle;

        if (_sched_stats.busy_ticks > _sched_stats.busy_ticks_max) {
            _sched_stats.busy_ticks_max = _sched_stats.busy_ticks;
        }

        if (_sched_stats.busy_ticks < _sched_stats.busy_ticks_min) {
            _sched_stats.busy_ticks_min = _sched_stats.busy_ticks;
        }

        _sched_stats.busy_ticks_avg = (3 * _sched_stats.busy_ticks_avg + _sched_stats.busy_ticks) >> 2;
    }
    _sched_stats.enter_idle_cnt++;
}

void gclk_manager_enable_dynamic_frequency_scaling(bool enable) {
    _mgr_ctx.pre_sched_pu_dfs_enabled = enable;
    if (enable) {
        pre_dfs_enable_freq = gclk_get_current_freq(gclk_manager_get_core_clock_handle());
    } else {
        _mgr_ctx.freq_change_cb(pre_dfs_enable_freq);
    }
}

void gclk_manager_post_idle_hook(void) {
    /* @todo: either we woke up because ll_timer triggered (as xtimer_replacement)..
               -> advance xtimer accordingly and take care it triggers */
    /* @todo: alternatively we woke up because of some other unrelated event (but xtimer was indeed stopped for a while)
               -> measure sleep_duration with help of ll_timer and just advance xtimer so it will trigger sooner (keep in mind backoff and so on) */
    /* @todo: compensate currently running xtimer */
    //CYCCNT_post = DWT->CYCCNT;
    //CYCCNT_diff = CYCCNT_post - CYCCNT_pre;
    //CYCCNT_diff_avg = ((CYCCNT_diff_avg * 9) + CYCCNT_diff) / 10;

    _sched_stats.t_left_idle = idle_timer_read();
    _sched_stats.idle_ticks = _sched_stats.t_left_idle - _sched_stats.t_went_idle;
    if (_sched_stats.idle_ticks > _sched_stats.idle_ticks_max) {
        _sched_stats.idle_ticks_max = _sched_stats.idle_ticks;
    }

    if (_sched_stats.idle_ticks < _sched_stats.idle_ticks_min) {
        _sched_stats.idle_ticks_min = _sched_stats.idle_ticks;
    }

    _sched_stats.idle_ticks_avg = (3 * _sched_stats.idle_ticks_avg + _sched_stats.idle_ticks) >> 2;

    /* @todo: schedule some kind of callback to notify when reaching a "critical high" busy time */
    _sched_stats.utilization = (201 * _sched_stats.busy_ticks + _sched_stats.idle_ticks) / ((_sched_stats.idle_ticks + _sched_stats.busy_ticks) * 2);
    _sched_stats.utilization_avg = (3 * _sched_stats.utilization_avg + _sched_stats.utilization) >> 2;
    //uint32_t utilization_avg = busy_ticks_avg / ((idle_ticks_avg + busy_ticks_avg) / 100);
    if (_mgr_ctx.cpu_util_based_dvfs_enabled) {
        //printf("busy: %lu idle %lu util: %lu avg_util: %lu\n", busy_ticks, idle_ticks, utilization, utilization_avg);
        //printf("performing DVFS for %lu %% utilization\n", utilization_avg);
        if (_sched_stats.idle_ticks == 0) {
            _dvfs(_sched_stats.utilization_avg);
        } else {
            _dvfs(_sched_stats.utilization);
        }
    }
}

void gclk_manager_print_util_metrics(void) {
    printf("idle_cycles:    %lu\n", _sched_stats.idle_ticks);
    printf("working_cycles: %lu\n", _sched_stats.busy_ticks);
    printf("idle_ticks_min: %lu\n", _sched_stats.idle_ticks_min);
    printf("idle_ticks_max: %lu\n", _sched_stats.idle_ticks_max);
    printf("idle_ticks_avg: %lu\n", _sched_stats.idle_ticks_avg);
    printf("busy_ticks_min: %lu\n", _sched_stats.busy_ticks_min);
    printf("busy_ticks_max: %lu\n", _sched_stats.busy_ticks_max);
    printf("busy_ticks_avg: %lu\n", _sched_stats.busy_ticks_avg);
    printf("utilization:    %lu\n", _sched_stats.utilization);
    printf("util_avg:       %lu\n", _sched_stats.utilization_avg);

    for (unsigned i = 0; i < _mgr_ctx.dfs_frequencies_cnt; i++) {
        printf("used %lu Hz for %lu schedules\n", _mgr_ctx.dfs_frequencies[i], _sched_stats.freq_sched_cnt[i]);
        _sched_stats.freq_sched_cnt[i] = 0;
    }
}

void gclk_manager_register_clk_change_cb(const gclk_t *clk, gclk_clock_change_notify_list_t *nle,
                                         clock_change_cb_t cb) {
    /* always store given callback and clock in the given notify list entry */
    nle->change_cb_list.change_cb = cb;
    nle->change_cb_list.node.next = NULL;
    nle->clk = clk;

    list_node_t *clk_list = clock_change_notify_list.next;

    /* If there already exists a notification list for this clock
     * the new callback is added to that list */
    while (clk_list) {
        gclk_clock_change_notify_list_t *ccnl = container_of(clk_list,
                                                             gclk_clock_change_notify_list_t,
                                                             node);
        if (ccnl->clk == clk) {
            list_add(&ccnl->change_cb_list.node, &nle->change_cb_list.node);
            registered_clk_change_cb_cnt++;
            return;
        }
        clk_list = clk_list->next;
    }

    /* if no one registered a change notification for this clock before, the given
     * notify list entry becomes the sub-list for the given clock */
    list_add(&clock_change_notify_list, &nle->node);
    registered_clk_change_cb_cnt++;
}

void gclk_manager_unregister_clk_change_cb(gclk_clock_change_notify_list_t *nle) {
    list_node_t *clk_list = &clock_change_notify_list;

    /* Get the sublist of the given notification entries clock */
    while (clk_list->next) {
        gclk_clock_change_notify_list_t *ccnl = container_of(clk_list->next,
                                                             gclk_clock_change_notify_list_t,
                                                             node);
        if (ccnl->clk == nle->clk) {
            /* If this entry is the root list for the given clock */
            if (ccnl == nle) {

                /* if it also holds other callbacks they must be relocated to the new
                 * sublist-root for this clock */
                if (nle->change_cb_list.node.next) {
                    /* get the new root via the callback property of the callback registration
                     * that sits after the removed entry */
                    gclk_clock_change_notify_list_t *nre = container_of(ccnl->change_cb_list.node.next,
                                                                        gclk_clock_change_notify_list_t,
                                                                        change_cb_list.node);
                    /* relocate the cb list of the removed entry to the new root.
                     * nle->change_cb_list.node.next points to the callback of the root itself so
                     * the root now must point to the next list node after its own registration */
                    nre->change_cb_list.node.next = nle->change_cb_list.node.next->next;

                    /* move other clock entries from after the removed clock to the new root */
                    nre->node.next = nle->node.next;

                    /* reinsert the new root to the clock lists */
                    clk_list->next = &nre->node;
                } else {
                    /* simply remove the list entry at the current position */
                    clk_list->next = clk_list->next->next;
                }
            } else {
                /* the nle to remove is not part of the top level clock list
                 * itself, so only remove its callback from the clocks sublist */
                list_remove(&ccnl->change_cb_list.node, &nle->change_cb_list.node);
            }
            registered_clk_change_cb_cnt--;
            break;
        }
        clk_list = clk_list->next;
    }
}

uint32_t gclk_manager_get_min_freq_at_fixed_uptree_conf(clk_topology_entry_t *topology, uint32_t len) {
   return _get_minmax_freq_at_fixed_uptree_conf(topology, len, false);
}

uint32_t gclk_manager_get_max_freq_at_fixed_uptree_conf(clk_topology_entry_t *topology, uint32_t len) {
   return _get_minmax_freq_at_fixed_uptree_conf(topology, len, true);
}

void gclk_manager_print_topology_conf(clk_topology_entry_t *topology, uint32_t size, bool min_max, bool factors) {
    for (uint32_t i = 0; i < size; i++) {
        printf("[%s@%lu", gclk_get_name(topology[i].clk), topology[i].clk_freq);
        if (min_max) {
            /* Note: this output refer to theoretical values obtainable with the uptree configuration being fixed
             *       to the settings given via the 'topology' parameter and *only adapting the factor of this clock*.
             *       This does not check for any constraints that may limit the frequency for any other reason. */
            printf("(%lu - %lu)", gclk_manager_get_min_freq_at_fixed_uptree_conf(&topology[i], size - i),
                                  gclk_manager_get_max_freq_at_fixed_uptree_conf(&topology[i], size - i));
        }
        if (factors) {
            printf("|%s", gclk_is_divider(topology[i].clk) ? "/" : (gclk_is_multiplier(topology[i].clk) ? "*" : ""));
            if (gclk_is_scalable(topology[i].clk)) {
                printf("%u", topology[i].factor);
            }
            printf("-");
        }

        printf("]");

        if (i < size -1) {
            printf("-->");
        }
    }

    printf("\n");
}

//TODO: rename params to unifiead scheme e.g., "tree_model"
void gclk_manager_simulate_seq_step_on_tree_conf(gclk_manager_sequence_step_t *step, clk_topology_entry_t *tree_conf, size_t tree_size) {

    clk_topology_entry_t *conf = NULL;

    if (_is_clk_modification_step(step)) {
        LOG_DEBUG("%s: is mod step -> get corresponding conf entry...\n", __FUNCTION__);
        conf = _get_clock_conf_from_tree_conf(step->clk, tree_conf, tree_size);
        LOG_DEBUG("%s: got entry -> simulate step on tree model\n", __FUNCTION__);
    }

    switch (step->op) {
        //TODO: sahll not used at the moment (set freq steps are converted to set factor steps before)
        case CLK_SET_FREQ:
            assert(false); /* should not be used in this context */
            conf->clk_freq = step->num_arg;
            //TODO: propagate change downtree!
            //TODO: update other conf members
            break;
        case CLK_SET_FACTOR:
            conf->factor = step->num_arg;
            /* parent did not actually change here, but we reuse the function to consider all uptree clocks for the frequency re-calculation */
            _model_propagate_conf_change_downtree(conf, tree_conf, tree_size);
            break;
        case CLK_SET_PARENT:
            assert(false); /* should not be used in this context */
            conf->par_idx = gclk_parent2idx(step->clk, step->clk_arg);
            //TODO: propagate change downtree!
            break;
        case CLK_SET_PARENT_IDX:
            conf->par_idx = step->num_arg;
            LOG_DEBUG("%s: PROPAGATE changes downtree...\n", __FUNCTION__);
            _model_propagate_conf_change_downtree(conf, tree_conf, tree_size);
            break;
        // no factor recalculation needed here as the only thing that was changed during this step was the enable state of THIS clock
        // we only need to propagate the own frequency downtree as it now is (or now isn't anymore) in effect.
        case CLK_ENABLE:
            conf->enabled = true;
            _model_propagate_conf_change_downtree(conf, tree_conf, tree_size);
            break;
        case CLK_DISABLE:
            conf->enabled = false;
            _model_propagate_conf_change_downtree(conf, tree_conf, tree_size);
            break;
        case CLK_CONFIG_TARGET:
            /* Does not make sense here at all (no target topo given, only single well-defined step) */
            printf("CLK_CONFIG_TARGET NOT IMPLEMENTED\n");
            assert(false);
            break;
        case BUSY_SPIN:
        case SET_LED:
            assert(false);
    }
}

/* based on a specific tree configruation and a reconfiguration sequence, determine the resulting tree configuration.
 * NOTE: the topology nodes here are not used to describe a specific chain (subtopology) but contain all (possibly involved)
 *       nodes of the tree in no particular order. The wording 'possibly involved' means that future optimizations
 *       may try to reduce the the number of clock nodes handed to this function. E.g., it is known that only a subset of
 *       clocks will be affected this would reduce the overhead to determine the configuration diff because less clocks need
 *       to be traversed.
 *       This function does not perform any changes to the hardware configuration but only to the state of the model in memory.
 *       This functionality can be used to predetermine/evaluate effects of reconfiguration steps to decide if they are valid
 *       and applicable acoording to static HW constrains or dynamic limits currently active. */
void gclk_manager_apply_sequence_to_tree_model(gclk_manager_sequence_step_t *seq, size_t seq_len, clk_topology_entry_t *tree_before,
                                               clk_topology_entry_t *tree_after, size_t tree_size) {

    /* save a working copy of the clock configuration to hold the intermediate state
     * so we can evaluate changes on it (an their effects) as we move along the sequence */
    memcpy(tree_after, tree_before, sizeof(clk_topology_entry_t) * tree_size);

    /* each change affects the clock tree configuration in one of the following ways:
     * - clocks that are part of the initial topology may be modified
     *  - transiently, clocks that depend on one of the initial clocks may be affected
     * - the topology may be transitioned to a new one, effectively changing parts of the old topology
     * - a step may change something that is not even related to the original toplogy
     */
    for (size_t si = 0; si < seq_len; si++) {
        const gclk_t *altered_clk = seq[si].clk;

        bool clk_mod_step = _is_clk_modification_step(&seq[si]);
        if (clk_mod_step) {
            list_node_t *n = clock_change_notify_list.next;
            while(n) {
                gclk_clock_change_notify_list_t *ccnl = container_of(n, gclk_clock_change_notify_list_t, node);
                if (_gclk_manager_is_derived_from_clock(altered_clk, ccnl->clk, tree_after, tree_size)) {
                    LOG_DEBUG("%s: %s is affected by this step!\n", __FUNCTION__, gclk_get_name(ccnl->clk));
                } else {
                    LOG_DEBUG("%s: %s is NOT affected by this step!\n", __FUNCTION__, gclk_get_name(ccnl->clk));
                }
                n = n->next;
            }
            LOG_DEBUG("%s: simulate step on tree model...\n", __FUNCTION__);
            /* update the tree model state with the step */
            gclk_manager_simulate_seq_step_on_tree_conf(&seq[si], tree_after, tree_size);
            LOG_DEBUG("%s: simulation step done!\n", __FUNCTION__);
         } else {
            LOG_DEBUG("%s: step does not modify clock\n", __FUNCTION__);
         }
    }

    //NOTE: there are different ways on how to incorporate reconfiguration feedback
    //      -> gernerate the before-after diff and execute reconfiguration callbacks before and after.
    //      -> call the reconfiguration callbacks before/after each individual modification that changes
    //         a relevant setting
}

void gclk_manager_notify_diff_changes(clk_topology_entry_t *tree_before, clk_topology_entry_t *tree_after, size_t tree_size,
                                      bool post_change, bool print_only) {
    for (size_t i = 0; i< tree_size; i++) {
        clk_topology_entry_t *oc = &tree_before[i];
        clk_topology_entry_t *nc = &tree_after[i];

        if (memcmp(nc, oc, sizeof(clk_topology_entry_t)) != 0) {
            bool one_enabled = oc->enabled || nc->enabled;
            if (one_enabled) {
                list_node_t *n = clock_change_notify_list.next;

                /* for each clock check if there is a list of callbacks that must be notified */
                while(n) {
                    gclk_clock_change_notify_list_t *ccnl = container_of(n, gclk_clock_change_notify_list_t, node);
                    const gclk_t *affected_clk = ccnl->clk;
                    if (affected_clk == oc->clk) {
                        list_node_t *cbn = &ccnl->change_cb_list.node;
                        /* call all registered callbacks per clock that was changed */
                        do {
                            gclk_change_cb_list_t *cbl = container_of(cbn, gclk_change_cb_list_t, node);
                            uint32_t orig_freq_effective = oc->enabled ? oc->clk_freq : 0;
                            uint32_t new_freq_effective = nc->enabled ? nc->clk_freq : 0;
                            if (print_only) {
                                printf("would call cbl->change_cb(%s, %s, %lu, %lu, %s);\n", gclk_get_name(affected_clk), gclk_get_name(affected_clk), orig_freq_effective, new_freq_effective, post_change ? "true" : "false");
                            } else {
                                cbl->change_cb(affected_clk, ccnl->clk, orig_freq_effective, new_freq_effective, post_change);
                            }
                            cbn = cbn->next;
                        } while (cbn);
                    }
                    n = n->next;
                }
            }
        }
    }

    _post_notify_commit(post_change);
}

/* for now allocate this statically till we replace it with a more concise (i.e. only a single clock specific config index)
 * or a reduced subset of clocks (i.e. only "relevant clocks") to also allow on demand stack allocation */
clk_topology_entry_t current_tree_conf[GCLK_NUM_OF_CLOCKS];
clk_topology_entry_t target_tree_conf[GCLK_NUM_OF_CLOCKS];

bool last_cb_was_post_change = true;

void gclk_manager_get_abs_min_ws_min_vc(const gclk_t *clk, uint32_t freq, unsigned *ws, unsigned *vc) {
    unsigned min_ws = 0;
    unsigned min_vc = 0;
    for (unsigned i = 0; i < GCLK_FREQ_LIMIT_CLKS_NUMOF; i++) {
        if (gclk_freq_conf_limits[i].clk == clk) {
            bool ws_found = false;
            bool vc_found = false;
            for (unsigned li = 0; li < gclk_freq_conf_limits[i].len; li++) {
                const freq_conf_limit_t *lim = &gclk_freq_conf_limits[i].limits[li];
                /* if the limit applies to the given frequency */
                if (lim->freq_max >= freq) {
                    if (!ws_found || (lim->ws_min < min_ws)) {
                        min_ws = lim->ws_min;
                        ws_found = true;
                    }
                    if (!vc_found || (lim->vc_idx_min < min_vc)) {
                        min_vc = lim->vc_idx_min;
                        vc_found = true;
                    }
                }
            }
        }
    }
    *ws = min_ws;
    *vc = min_vc;
}

/* TODO: There might be a use for a configurable (or clock-/tree-specific) notification policy.
 *       E.g., a way to explicitly express 'notify pre-/post- running the whole sequence' or
 *       'notify pre-/post- each individual step. This depends on how dynamic constraints/blocks and notifications
 *       are treated semantically and whether the sequence is guaranteed to run without intermediate notifications.
 *
 *       An illustration of the thought: the pre-cb may set a peripheral into a mode where any change is allowed
 *       till its post-call or it may require an update after each change. The waitstate adaptation cb for example
 *       can be implemented in a way that sets up the highest (safest) waitstate configuration in the pre-cb.
 *       Then various steps in the sequence can setup whatever frequency without breaking any constraint or limit.
 *       If the pre- cb however only prepares for the state of the *final* configuration (reached after after all
 *       sequence steps), intermediate steps that cause a higher configuration should not be allowed. */
void gclk_manager_run_sequence_with_notify(gclk_manager_sequence_step_t *seq, size_t seq_len, bool print_only)
{
    //TODO: for now we use a static allocation for this (effectively this breaks reentrant usage for now)
    /* get the current state of all clocks to have a full view on the clock model
     * TODO: consider caching this in the future.
     * TODO: investigating the sequence beforehand (e.g., on a purely topological basis) could be used
     *       to filter the number of involved clocks which may significantly reduce the set of clocks
     *       that must be looked at.
     * TODO: another representation of the tree (e.g., with linked lists, ordered sets, or the like)
     *       would GREATLY improve performance here and should be targeted if this method proves to be
     *       useful for regular runtime-use. (the functional benefits can be evaluated as is nonetheless).
     * For now we leave it as is to first evaluate functional operation */
    for (unsigned i = 0; i < GCLK_NUM_OF_CLOCKS; i++) {
        current_tree_conf[i].clk = gclks[i];
        gclk_get_current_topology_config(&current_tree_conf[i], 1);
    }

    size_t tree_size = GCLK_NUM_OF_CLOCKS;
    gclk_manager_apply_sequence_to_tree_model(seq, seq_len, current_tree_conf, target_tree_conf, tree_size);

    if (print_only) {
        unsigned affected_crit = 0;
        unsigned affected_noncrit = 0;
        //TODO: actually calculate the effective diff
        printf("\nRelevant changes of active clocks:\n");
        for (size_t i = 0; i< tree_size; i++) {
            clk_topology_entry_t *oc = &current_tree_conf[i];
            clk_topology_entry_t *nc = &target_tree_conf[i];

            if (memcmp(nc, oc, sizeof(clk_topology_entry_t)) != 0) {
                bool one_enabled = oc->enabled || nc->enabled;
                if (one_enabled) {
                    affected_crit++;
                    _print_conf_change(oc, nc);
                }
            }
        }

        printf("\nNon-critical (inactive subtree) changes:\n");
        for (size_t i = 0; i< tree_size; i++) {
            clk_topology_entry_t *oc = &current_tree_conf[i];
            clk_topology_entry_t *nc = &target_tree_conf[i];

            if (memcmp(nc, oc, sizeof(clk_topology_entry_t)) != 0) {
                bool one_enabled = oc->enabled || nc->enabled;
                if (!one_enabled) {
                    affected_noncrit++;
                    _print_conf_change(oc, nc);
                }
            }
        }

        printf("\noverall, %u clocks of %u were affected by this sequence (%u critical, %u non-critical)\n",
                affected_crit + affected_noncrit, tree_size, affected_crit, affected_noncrit);
    }

    //TODO: an optional (parameterized) check for constraint/limit-violations can be added here
    //      for cases where the validity of the sequence was not validated alongside the sequence generation
    gclk_manager_notify_diff_changes(current_tree_conf, target_tree_conf, tree_size, false, print_only);
    if (!print_only) {
        gclk_manager_run_sequence(seq, seq_len);
    }
    gclk_manager_notify_diff_changes(current_tree_conf, target_tree_conf, tree_size, true, print_only);

    if (print_only) {
        unsigned int ws = 0;
        unsigned int vc_idx = 0;
        /* NOTE: for now this only works as expected when called with a non-reduced set of clocks
         *       (i.e. the full global tree view), because constraints that became active earlier
         *       and are *not* changed with this step-sequence may not be part of the (reduced)
         *       tree view (only contained in the cache). The below function would need to be
         *       updated to also consider the cached constraints */
        gclk_get_min_required_ws_vc_from_tree_config(target_tree_conf, tree_size, &ws, &vc_idx, _mgr_ctx.dvs_policy);
        printf("Applicable Voltage/Flash configs\n");
        printf("New flash waitstate setting: %u\n", ws);
        printf("New core voltage setting: %u (%u mV)\n", vc_idx, core_voltage_idx2mv(vc_idx));
    }
}

void gclk_manager_notify_multi_clk_change(gclk_manager_sequence_step_t *seq, size_t seq_len,
                                          clk_topology_entry_t *old_topo, size_t old_topo_len,
                                          clk_topology_entry_t *new_topo, size_t new_topo_len,
                                          bool post_change) {
    (void)seq;
    (void)seq_len;
    unsigned affected_cnt = 0;
    /* registered_clk_change_cb_cnt is actually a pessimistic value (the number of distinct affected clocks micht be lower) */
    gclk_clock_change_notify_list_t *affected[registered_clk_change_cb_cnt];

    const gclk_t *topmost_altered_clk = NULL;
    uint32_t tmc_f_old = 0;
    uint32_t tmc_f_new = 0;
    for (unsigned oi = old_topo_len -1; oi > 0; oi--) {
        for (unsigned ni = new_topo_len -1; ni > 0; ni--) {
            if (old_topo[oi].clk == new_topo[ni].clk) {
                topmost_altered_clk = old_topo[oi].clk;
                tmc_f_old = old_topo[oi].clk_freq;
                tmc_f_new = new_topo[ni].clk_freq;
                break;
            }
        }

        if (topmost_altered_clk) {
            break;
        }
    }

    /* notify all clocks affected by that change */
    /* @TODO: store flag on pre-call to leverage that on post call ?*/
    list_node_t *n = clock_change_notify_list.next;
    while(n) {
        gclk_clock_change_notify_list_t *ccnl = container_of(n, gclk_clock_change_notify_list_t, node);
        //list_node_t *cbn = &ccnl->change_cb_list.node;
        //gclk_change_cb_list_t *cbl = container_of(cbn, gclk_change_cb_list_t, node);

        bool already_added = false;
        for (unsigned i = 0; i < affected_cnt; i++) {
            if (affected[i] == ccnl) {
                already_added = true;
                break;
            }
        }

        if (!already_added &&
            gclk_affected_by_change(topmost_altered_clk, ccnl->clk)) {
            affected[affected_cnt++] = ccnl;
        }
        n = n->next;
    }

    for (unsigned i = 0; i < affected_cnt; i++) {
        gclk_clock_change_notify_list_t *ccnl = affected[i];
        list_node_t *cbn = &ccnl->change_cb_list.node;
        do {
            gclk_change_cb_list_t *cbl = container_of(cbn, gclk_change_cb_list_t, node);
            cbl->change_cb(topmost_altered_clk, ccnl->clk, tmc_f_old, tmc_f_new, post_change);
            cbn = cbn->next;
        } while (cbn);
    }
}

void gclk_manager_notify_clk_change(const gclk_t *clk, uint32_t f_old, uint32_t f_new,
                                    bool post_change) {
    /* notify all clocks affected by that change */
    /* @TODO: store flag on pre-call to leverage that on post call ?*/
    list_node_t *n = clock_change_notify_list.next;
    while(n) {
        gclk_clock_change_notify_list_t *ccnl = container_of(n, gclk_clock_change_notify_list_t, node);
        //TODO: the way *how* the clock down the tree is affected should be determined here because we want to avoid doing
        //      that in each callback.
        if (gclk_affected_by_change(clk, ccnl->clk)) {
            LOG_DEBUG("%s is affected by change of %s\n", gclk_get_name(ccnl->clk), gclk_get_name(clk));
            list_node_t *cbn = &ccnl->change_cb_list.node;
            do {
                gclk_change_cb_list_t *cbl = container_of(cbn, gclk_change_cb_list_t, node);
                cbl->change_cb(clk, ccnl->clk, f_old, f_new, post_change);
                cbn = cbn->next;
            } while (cbn);
        }
        n = n->next;
    }
}

const freq_conf_limit_t *gclk_manager_get_freq_conf_limit(const gclk_t *clk, uint32_t freq, bool optimize_ws) {
    const clock_freq_conf_limits_t *limitspec = NULL;

    for (unsigned i = 0; i < GCLK_FREQ_LIMIT_CLKS_NUMOF; i++) {
        if (gclk_freq_conf_limits[i].clk == clk) {
            limitspec = &gclk_freq_conf_limits[i];
            break;
        }
    }

    if (!limitspec) {
        return NULL;
    }

    int ws_optimal_idx = -1;
    int vc_optimal_idx = -1;

    for (unsigned i = 0; i < limitspec->len; i++) {
        /* if config applies to this frequency */
        if (limitspec->limits[i].freq_max >= freq) {
            if ((ws_optimal_idx < 0) ||
                (limitspec->limits[ws_optimal_idx].ws_min == GCLK_WS_NOSPEC) ||
                (limitspec->limits[ws_optimal_idx].ws_min > limitspec->limits[i].ws_min) ||
                ((limitspec->limits[ws_optimal_idx].ws_min == limitspec->limits[i].ws_min) &&
                 (limitspec->limits[ws_optimal_idx].vc_idx_min > limitspec->limits[i].vc_idx_min)) ) {
                ws_optimal_idx = i;
            }
            if ((vc_optimal_idx < 0) ||
                (limitspec->limits[vc_optimal_idx].vc_idx_min > limitspec->limits[i].vc_idx_min) ||
                ((limitspec->limits[ws_optimal_idx].vc_idx_min == limitspec->limits[i].vc_idx_min) &&
                 (limitspec->limits[ws_optimal_idx].ws_min > limitspec->limits[i].ws_min)) ) {
                vc_optimal_idx = i;
            }
        }
    }

    if (optimize_ws) {
        return &limitspec->limits[ws_optimal_idx];
    }

    return &limitspec->limits[vc_optimal_idx];
}

/* NOTE: the flags on 'whether to prioritize flash or voltage optimizations' (optimize_flash) and
 *       'whether to actually perform voltage updates' (enabe_vscale) are separated to allow
 *       to investigate the effects of each aspect in an isolated way.
 * This updates the vcore/ws settings according to the current state of the clock, configured policy and
 * the flag that controls whether the vscale should actually be applied */
void _update_vcore_and_ws_config(const gclk_t *altered_clk, uint32_t f_new, gclk_manager_dvs_policy_t dvs_policy, bool fup) {
    /* TODO separate DVS and flash WS adaptation enable/disable -> leverage the policy
     *      decision to prefer fast flash if DVS is disabled and the other way around */
    /* @TODO: this may need more advanced handling on platforms that have multiple
     *        clock instances that induce different dependencies on voltage or waitstates.
     *        In such a case the most stringent limit of all involved clocks must be selected
     *        and applied. For that, a more global view on the change that is about to happen
     *        is required. (could be done either by handing over all changed clocks or storing
     *        limit candidates across cb executions and only applying the most stringent
     *        limit on the last call). */
    const freq_conf_limit_t *limit = gclk_manager_get_freq_conf_limit(altered_clk, f_new,
            dvs_policy == DVS_PREFER_FAST_FLASH ? true : false);
    if (fup) {
        /* when increasing frequency the order is:
         * update voltage -> update wait states -> increase freq */
        if (auto_vscale_enabled) {
            core_voltage_set(limit->vc_idx_min);
        }
        if (auto_wsadapt_enabled) {
            flash_opt_set_wait_states(limit->ws_min);
        }
    } else {
        /* when decreasing frequency the order is:
         * reduce freq -> update wait states -> update voltage */
        if (auto_wsadapt_enabled) {
            flash_opt_set_wait_states(limit->ws_min);
        }
        if (auto_vscale_enabled) {
            core_voltage_set(limit->vc_idx_min);
        }
    }
}

static void _lazy_reg_freq_limit_clk_change_cbs(void) {
    /* only register new callback if no automatic adaption is enabled yet */
    if (!(auto_vscale_enabled || auto_wsadapt_enabled)) {
        for (unsigned i = 0; i < GCLK_FREQ_LIMIT_CLKS_NUMOF; i++) {
            /* DVS just re-uses the notification mechanism to change the
             * voltage to an appropriate value before/after the frequency is adapted */
            gclk_manager_register_clk_change_cb(gclk_freq_conf_limits[i].clk, &_mgr_ctx.ccnl[i],
                    _dvs_wsa_freq_constraint_change_cb);
        }
    }
}

static void _lazy_unreg_freq_limit_clk_change_cbs(void) {
    /* unregister callback if neither automatic adaption shall be enabled anymore */
    if (!(auto_vscale_enabled || auto_wsadapt_enabled)) {
        /* only disable if enabled */
        for (unsigned i = 0; i < GCLK_FREQ_LIMIT_CLKS_NUMOF; i++) {
            gclk_manager_unregister_clk_change_cb(&_mgr_ctx.ccnl[i]);
        }
    }
}

void _gclk_manager_run_sequence__dyn_freq(gclk_manager_sequence_step_t *steps, size_t step_cnt, uint32_t freq) {
    /* TODO: instead of a soingle freq variable this should contain a target topology conf */
    (void)freq;

    for (unsigned i = 0; i < step_cnt; i++) {
        gclk_manager_sequence_step_t *step = &steps[i];
        gclk_manager_execute_sequence_step(step);
    }
}

static unsigned _populate_applicable_clock_constraints(gclk_freq_constraint_t *acc, clk_topology_entry_t *topo, uint32_t topo_len) {
    unsigned rccnt = 0;
    for (unsigned i = 0; i < topo_len; i++) {
        for (unsigned c = 0; c < GLOBAL_CLOCK_CONSTRAINTS_NUMOF; c++) {
            if (global_clock_constraints[c].clk == topo[i].clk) {
                acc[rccnt] = global_clock_constraints[c];
                rccnt++;
            }
        }
    }
    return rccnt;
}

void gclk_manager_enable_voltage_auto_scale(bool on) {
    if (on) {
        _lazy_reg_freq_limit_clk_change_cbs();
        auto_vscale_enabled = true;
    } else {
        auto_vscale_enabled = false;
        _lazy_unreg_freq_limit_clk_change_cbs();
        /* TODO: setup worst case value here */
        core_voltage_set(core_voltage_cnt() - 1);
    }
}

void gclk_manager_enable_flashws_auto_update(bool on) {
    if (on) {
        _lazy_reg_freq_limit_clk_change_cbs();
        auto_wsadapt_enabled = true;
    } else {
        auto_wsadapt_enabled = false;
        _lazy_unreg_freq_limit_clk_change_cbs();
        /* TODO: setup worst case value here */
        flash_opt_set_wait_states(flash_opt_get_max_wait_states());
    }
}

void gclk_manager_set_dvs_policy(gclk_manager_dvs_policy_t policy) {
    _mgr_ctx.dvs_policy = policy;
}

gclk_manager_dvs_policy_t gclk_manager_get_dvs_policy(void) {
    return _mgr_ctx.dvs_policy;
}

void gclk_manager_disable_unused(void) {
    for (unsigned i = 0; i < gclk_get_cnt(); i++) {
        const gclk_t *clk = gclk_get(i);
        //printf("%s\n", gclk_get_name(clk));
        LOG_DEBUG("%s leaf: %s\n", gclk_get_name(clk), gclk_is_leaf(clk) ? "true" : "false");
        LOG_DEBUG("%s used: %s\n", gclk_get_name(clk), gclk_is_used(clk) ? "true" : "false");

        /* TODO: the fact that an intermediate clock is not used by a leaf doesn't necessarily guarantee it is unused..
         *       i.e. there might be an internal dependency why the clock is needed */
        if (gclk_is_enabled(clk) && (!gclk_is_used(clk)) && (!gclk_is_leaf(clk))) {
            /* only disable clocks that are gateable itself
             * TODO: alternatively the disable can be forwarded if this is the only user... */
            if (gclk_is_gateable(clk)) {
                LOG_DEBUG("disabling [%s]..\n", gclk_get_name(clk));
            } else {
                LOG_DEBUG("cant disable [%s] directly (not gateable)\n", gclk_get_name(clk));
                /* check if we can forward the unused state to the parent */
                const gclk_t *parent = gclk_get_current_parent(clk);
                const gclk_t *child;
                unsigned child_idx = 0;
                bool otherwise_used = false;
                do {
                    child = gclk_get_child(parent, child_idx);
                    /* another clock also usews the parent the disable can not be forwarded */
                    if ((child != NULL) && (child != clk)) {
                        otherwise_used = true;
                        break;
                    }
                    child_idx++;
                } while (child != NULL);

                if (!otherwise_used && gclk_is_gateable(parent)) {
                    LOG_DEBUG("disabling otherwise unused parent [%s] instead..\n", gclk_get_name(parent));
                    gclk_disable(parent);
                }
            }
            //gclk_disable(clk);
        }
    }
}

bool gclk_manager_set_factor(const gclk_t *clk, uint32_t factor) {
    uint32_t f_old = gclk_get_current_freq(clk);
    uint32_t f_in = gclk_get_input_freq(clk);
    uint32_t f_new = gclk_is_divider(clk) ? (f_in / factor) :
                    (gclk_is_multiplier(clk) ? (f_in * factor) : f_in);
    /* only check for applicable callbacks if there are registrations */
    if (registered_clk_change_cb_cnt) {
        gclk_manager_notify_clk_change(clk, f_old, f_new, false);
    }
    int res = gclk_set_factor(clk, factor);

    if (registered_clk_change_cb_cnt) {
        gclk_manager_notify_clk_change(clk, f_old, f_new, true);
    }

    return res == 0;
}

bool gclk_manager_set_freq(const gclk_t *clk, uint32_t freq) {
    uint32_t f_old = gclk_get_current_freq(clk);

    /* only check for applicable callbacks if there are registrations */
    if (registered_clk_change_cb_cnt) {
        gclk_manager_notify_clk_change(clk, f_old, freq, false);
    }

    uint32_t new_freq = gclk_set_freq(clk, freq);

    if (registered_clk_change_cb_cnt) {
        gclk_manager_notify_clk_change(clk, f_old, freq, true);
    }

    if (new_freq != freq) {
        LOG_DEBUG("most appliccable frequency was %lu Hz\n", new_freq);
    }

    if (new_freq == f_old) {
        return false;
    }

    return true;
}

//TODO: just make above function paramterized?
/* clone of the above function with instrumentation for performance measurements */
bool _gclk_manager_set_freq_instrumented(const gclk_t *clk, uint32_t freq) {

    gpio_clear(LOGIC_ANALYZER_PIN);
    uint32_t f_old = gclk_get_current_freq(clk);
    gpio_set(LOGIC_ANALYZER_PIN);

    /* only check for applicable callbacks if there are registrations */
    if (registered_clk_change_cb_cnt) {
        gclk_manager_notify_clk_change(clk, f_old, freq, false);
    }

    gpio_clear(LOGIC_ANALYZER_PIN);
    uint32_t new_freq = gclk_set_freq(clk, freq);
    gpio_set(LOGIC_ANALYZER_PIN);

    if (registered_clk_change_cb_cnt) {
        gclk_manager_notify_clk_change(clk, f_old, freq, true);
    }

    gpio_clear(LOGIC_ANALYZER_PIN);

    gpio_set(LOGIC_ANALYZER_PIN); /* return to idle state */

    if (new_freq != freq) {
        LOG_DEBUG("most appliccable frequency was %lu Hz\n", new_freq);
    }

    if (new_freq == f_old) {
        return false;
    }

    return true;
}

void gclk_manager_execute_sequence_step(gclk_manager_sequence_step_t *step) {
    switch (step->op) {
        case CLK_SET_FREQ:   gclk_manager_set_freq(step->clk, step->num_arg); break;
        case CLK_SET_FACTOR: gclk_set_factor(step->clk, step->num_arg);       break;
        case CLK_SET_PARENT:
            gclk_set_parent(step->clk, gclk_parent2idx(step->clk, step->clk_arg));
            break;
        case CLK_SET_PARENT_IDX:
            gclk_set_parent(step->clk, step->num_arg);
            break;
        case CLK_ENABLE:     gclk_enable(step->clk);                          break;
        case CLK_DISABLE:    gclk_disable(step->clk);                         break;
        case CLK_CONFIG_TARGET:
            /* TODO: implement */
            printf("CLK_CONFIG_TARGET NOT IMPLEMENTED\n");
            break;
        case BUSY_SPIN: {volatile int i = step->num_arg; while (i--) {};} break;
        case SET_LED:
            if (step->num_arg) {
                gpio_set(LED0_PIN);
            } else {
                gpio_clear(LED0_PIN);
            }
    }
}

const gclk_freq_constraint_t* gclk_manager_conf_breaks_constraint(const gclk_freq_constraint_t *constraints, unsigned constr_cnt, clk_topology_entry_t *topo_conf, uint32_t topo_len) {
    for (unsigned c = 0; c < constr_cnt; c++) {
        for (unsigned t = 0; t < topo_len; t++) {
            if (constraints[c].clk == topo_conf[t].clk) {
                if ((constraints[c].type == GCLK_ENSURE_MIN_FREQ) &&
                    (topo_conf[t].clk_freq < constraints[c].freq)) {
                    return &constraints[c];
                }
                if ((constraints[c].type == GCLK_ENSURE_MAX_FREQ) &&
                    (topo_conf[t].clk_freq > constraints[c].freq)) {
                    return &constraints[c];
                }
            }
        }
    }
    return NULL;
}

uint32_t gclk_manager_brute_force_freq_conf(const gclk_t *clk, clk_topology_entry_t *best_topology, uint32_t *topo_len,
                                            int *topo_idx, gclk_cmp_func_t cmp_func, void *cmp_func_ctx, size_t *ret_n_valid, int force_nth,
                                            gclk_exploration_result_cb_conf_t *valid_conf_found_cb_conf) {

    clk_topology_entry_t ct[*topo_len];
    memset(ct, 0, sizeof(clk_topology_entry_t) * *topo_len);

    /* this should be the only field set to describe where we want to start,
       everything else must be zero at the beginning */
    ct[0].clk = clk;
    uint32_t best_top_size = 0;
    size_t valids = 0;
    uint32_t best_topo_idx = 0;
    uint32_t best_val_conf_idx = 0;

    for (unsigned ti = 0; ti < gclk_get_topology_config_cnt(clk); ti++) {
        size_t ct_len = gclk_get_nth_topology(ct, *topo_len, ti);

        /* once for each topology, check if there are any global constraints that must be considered */
        gclk_freq_constraint_t relevant_clock_constraints[GLOBAL_CLOCK_CONSTRAINTS_NUMOF];
        unsigned rel_constr_cnt = _populate_applicable_clock_constraints(relevant_clock_constraints, ct, ct_len);

        if (ct_len > 0) {
            if ((*topo_idx == GCLK_UNDEFINED_TOPOLOGY) || (ti == (unsigned)*topo_idx)) {
                LOG_DEBUG("checking topology with size: %d\n", ct_len);
                LOG_DEBUG("topology after init: \n");
                if (LOG_LEVEL >= LOG_DEBUG) {
                    gclk_print_topology_metadata(ct, ct_len);
                    gclk_manager_print_topology_conf(ct, ct_len, true, false);
                }
                uint32_t f_min = GCLK_HIGHEST_VALID_FREQ;
                uint32_t f_max = 0;


                size_t configs = gclk_get_factors_config_cnt_from_topology(ct, ct_len);

                for (unsigned i = 0; i < configs; i++) {
                    gclk_advance_topology_to_next_frequency_setting(ct, ct_len, i);

                    gclk_calculate_topology_config_freqs(ct, ct_len);

                    if (ct[0].clk_freq < f_min) {
                        f_min = ct[0].clk_freq;
                    }

                    if (ct[0].clk_freq > f_max) {
                        f_max = ct[0].clk_freq;
                    }

                    if (!gclk_manager_conf_breaks_constraint(relevant_clock_constraints, rel_constr_cnt, ct, ct_len)) {
                        gclk_cmp_result_t cmp_res = cmp_func(best_topology, best_top_size, ct, ct_len, cmp_func_ctx);
                        if (cmp_res != GCLK_CONF_INVALID) {
                            bool requested_this_config = (force_nth >= 0) && (valids == (unsigned)force_nth);
                            uint32_t valid_conf_idx = valids;

                            if (valid_conf_found_cb_conf && ((force_nth < 0) || requested_this_config)) {
                                if ((valid_conf_found_cb_conf->cb_mode == CB_ON_BETTER && cmp_res == GCLK_CONF_BETTER) ||
                                        (valid_conf_found_cb_conf->cb_mode == CB_ON_VALID && cmp_res != GCLK_CONF_INVALID)) {
                                    valid_conf_found_cb_conf->valid_conf_found_cb(ct, ct_len, cmp_res, valid_conf_idx,
                                            valid_conf_found_cb_conf->ctx);
                                }
                            }

                            valids++;

                            /* is true if ct is "better than" best_topology or a specific valid config was requested */
                            if (cmp_res == GCLK_CONF_BETTER || requested_this_config) {
                                memcpy(best_topology, ct, sizeof(clk_topology_entry_t) * ct_len);
                                best_top_size = ct_len;
                                best_topo_idx = ti;
                                best_val_conf_idx = valids - 1;
                                if (requested_this_config) {
                                    goto end_search;
                                }
                            }
                        }
                    } else {
                        LOG_DEBUG("breaks constraint\n");
                    }
                }

                LOG_DEBUG("[%s] f_min: %lu   f_max: %lu\n", gclk_get_name(clk), f_min, f_max);
                LOG_DEBUG("checked %u possible frequency configurations for topology %u\n", configs, ti);

                if (LOG_LEVEL >= LOG_DEBUG) {
                    gclk_print_topology(ct, ct_len);
                }

                LOG_DEBUG("\n\n");
            }
        }
        else {
            break;
        }
    }
end_search:

    if (ret_n_valid != NULL) {
        *ret_n_valid = valids;
    }

    if (valids == 0) {
        *topo_len = 0;
        *topo_idx = GCLK_UNDEFINED_TOPOLOGY;
        return GCLK_INVALID_FREQ;
    }

    if (valid_conf_found_cb_conf && valid_conf_found_cb_conf->cb_mode == CB_ON_BEST_ONLY) {
        valid_conf_found_cb_conf->valid_conf_found_cb(best_topology, best_top_size, GCLK_CONF_BEST,
                                                      best_val_conf_idx,
                                                      valid_conf_found_cb_conf->ctx);
    }

    *topo_idx = best_topo_idx;
    *topo_len = best_top_size;
    return best_topology[0].clk_freq;
}

gclk_cmp_result_t gclk_manager_cmp_topology_exact_leaf_freq_pmin(clk_topology_entry_t *topo_best, size_t len1,
                                                                 clk_topology_entry_t *topo_cmp, size_t len2,
                                                                 void *arg) {
    uint32_t target_freq = *(uint32_t*)arg;

    /* only works with in-order topology list (not with arbitrary order tree config) */
    if (topo_cmp[0].clk_freq == target_freq) {
        /* if the best topology is still invalid max out the value to select the first valid one */
        uint32_t best_nW = topo_best[0].clk_freq != GCLK_INVALID_FREQ ? _get_system_consumption_nW_from_power_model(topo_best, len1) : 0xFFFFFFFF;
        uint32_t cmp_nW = _get_system_consumption_nW_from_power_model(topo_cmp, len2);
        if (cmp_nW < best_nW) {
            return GCLK_CONF_BETTER;
        } else if (cmp_nW == best_nW) {
            return GCLK_CONF_EQUAL;
        }
        return GCLK_CONF_WORSE;
    } else {
        return GCLK_CONF_INVALID;
    }
}

gclk_cmp_result_t gclk_manager_cmp_topology_closest_leaf_freq_pmin(clk_topology_entry_t *topo_best, size_t len1,
                                                                   clk_topology_entry_t *topo_cmp, size_t len2,
                                                                   void *arg) {
    gclk_cmp_result_t closest_res = gclk_cmp_topology_for_closest_leaf_freq(topo_best, len1, topo_cmp, len2, arg);

    if (closest_res == GCLK_CONF_BETTER) {
        return GCLK_CONF_BETTER;
    } else if (closest_res == GCLK_CONF_EQUAL) {
        /* if the best topology is still invalid max out the value to select the first valid one */
        uint32_t best_nW = topo_best[0].clk_freq != GCLK_INVALID_FREQ ? _get_system_consumption_nW_from_power_model(topo_best, len1) : 0xFFFFFFFF;
        uint32_t cmp_nW = _get_system_consumption_nW_from_power_model(topo_cmp, len2);
        if (cmp_nW < best_nW) {
            return GCLK_CONF_BETTER;
        } else if (cmp_nW == best_nW) {
            return GCLK_CONF_EQUAL;
        }
    }
    return closest_res;
}


uint32_t gclk_manager_switch_topology(const gclk_t *clk, int target_topology, uint32_t target_freq, gclk_cmp_func_t cmp_func) {

    if (clk) {
        mutex_lock(&clock_conf_mutex);
        uint32_t max_involved_clks = gclk_get_clk_subtree_max_depth(clk, 0);
        LOG_DEBUG("%s: max topology for driving %s employs %lu clock nodes\n", __FUNCTION__, gclk_get_name(clk), max_involved_clks);

        clk_topology_entry_t topology[max_involved_clks];
        memset(topology, 0, sizeof(clk_topology_entry_t) * max_involved_clks);
        topology[0].clk = clk;
        topology[0].clk_freq = GCLK_INVALID_FREQ;
        size_t valid_cnt = 0;
        int force_nth = -1;

        unsigned long leaf_freq = gclk_manager_brute_force_freq_conf(clk, topology, &max_involved_clks, &target_topology,
                                                                     cmp_func, (void*)&target_freq, &valid_cnt, force_nth, NULL);

        LOG_DEBUG("%s: Matched frequency for %s: %lu Hz\n", __FUNCTION__, gclk_get_name(clk), leaf_freq);
        LOG_DEBUG("%s: Obtainable by using the following topology:\n", __FUNCTION__);
        if (LOG_LEVEL >= LOG_DEBUG) {
            gclk_manager_print_topology_conf(topology, max_involved_clks, true, false);
        }

        unsigned int cur_topolen = gclk_get_current_topology_len(clk);
        LOG_DEBUG("%s: topolen:%u\n", __FUNCTION__, cur_topolen);
        clk_topology_entry_t cur_topology[cur_topolen];
        LOG_DEBUG("%s: get topo\n", __FUNCTION__);
        cur_topology[0].clk = clk;
        gclk_get_current_topology_config(cur_topology, cur_topolen);
        int cur_topo_idx = gclk_topology2id(cur_topology, cur_topolen);

        printf("%s: Topo switch from %d to %d was requested -> derive transition sequence...\n", __FUNCTION__, cur_topo_idx, target_topology);
        gclk_manager_sequence_step_t out_seq[20];
        int seq_size = gclk_manager_derive_sequence(cur_topology, cur_topolen,
                                                    topology, max_involved_clks, out_seq, ARRAY_SIZE(out_seq));

        if (seq_size > 0) {
            if (LOG_LEVEL >= LOG_DEBUG) {
                gclk_manager_print_step_sequence(out_seq, seq_size);
            }

            LOG_DEBUG("%s: execute sequence...\n", __FUNCTION__);
            gclk_manager_notify_multi_clk_change(out_seq, seq_size,
                                                 cur_topology, cur_topolen,
                                                 topology, max_involved_clks,
                                                 false);
            //gclk_manager_notify_clk_change(cur_topology[0].clk, cur_topology[0].clk_freq, topology[0].clk_freq, false);
            //mutex_lock(&clock_conf_mutex);
            //gpio_irq_disable(AT86RF2XX_PARAM_INT);
            for (int i = 0; i < seq_size; i++) {
                gclk_manager_execute_sequence_step(&out_seq[i]);
            }

            ////TODO: replace that by registering a change notify callback.
            //gpio_irq_enable(AT86RF2XX_PARAM_INT);
            //mutex_unlock(&clock_conf_mutex);
            //gclk_manager_notify_clk_change(cur_topology[0].clk, cur_topology[0].clk_freq, topology[0].clk_freq, true);
            gclk_manager_notify_multi_clk_change(out_seq, seq_size,
                                                 cur_topology, cur_topolen,
                                                 topology, max_involved_clks,
                                                 true);

            // if it can be guaranteed that the target setting is effective and setup correctly
            // it could be more efficient to just update the values based on the known changes instead of queriying the whole state again.
            //_mgr_ctx.current_core_topolen = max_involved_clks;
            //_mgr_ctx.current_core_topo_id = target_topology;
            _update_cached_state_vars();
            /* a topology change in most cases invalidates configured DFS settings.
             * Therefore they are updated to valid default values */
            gclk_mananger_set_default_dfs_frequencies();

            mutex_unlock(&clock_conf_mutex);
            return leaf_freq;
        } else {
            LOG_DEBUG("%s: transition from [%s] topology from %d to %d infeasible!\n", __FUNCTION__, gclk_get_name(clk), cur_topo_idx, target_topology);
        }
    } else {
        LOG_DEBUG("%s: can not change topology of NULL clock\n", __FUNCTION__);
    }

    mutex_unlock(&clock_conf_mutex);

    return GCLK_INVALID_FREQ;
}

bool gclk_manager_scale_core_freq(uint32_t freq) {
    const gclk_scale_setting_t *s = _mgr_ctx.active_core_scale_setting;
    if (!s) {
        printf("%s: no scale setting appliccable at the moment!\n", __FUNCTION__);
        return false;
    }

    LOG_DEBUG("%s: scale freq of [%s](topo %d) to %luHz\n", __FUNCTION__, gclk_get_name(gclk_core_clock_handle), _mgr_ctx.current_core_topo_id, freq);

    const gclk_t *adapted_clk = s->scale_clk;

    uint32_t f_old = gclk_get_current_freq(adapted_clk);
    uint32_t adapted_clk_new_freq = freq;
    uint32_t new_freq = GCLK_INVALID_FREQ;
    uint32_t new_uptree_factor = 0;

    /* in case relative uptree-scaling is used, the notification mechanism has to notify via the topmost clock.
     * For that, the relative factor between the core handle and the clock used for scaling must be considered in order
     * to calculate the old and new frequency of the scaled clock */
    if (s->approach == SCALE_UPTREE_RELATIVE) {
        /* for cases where any nodes between the output clock instance and the scale_clock instance do intermediate scaling,
         * the factor applied to the scaled clock needs to adapted accordingly by calculating the effective scaling factor
         * relative to the intermediate nodes and the output. */
        for (unsigned i = 0; i < _mgr_ctx.dfs_frequencies_cnt; i++) {
            if (_mgr_ctx.topology_conf_cache[i][0].clk_freq == freq) {
                new_uptree_factor = _mgr_ctx.topology_conf_cache[i][0].factor;
                adapted_clk_new_freq = _mgr_ctx.topology_conf_cache[i][0].factor * gclk_get_input_freq(adapted_clk);
                new_freq = freq;
                break;
            }
        }
    }

    //TODO: update Notification handling (this variant is only appliccable to scale modes where
    //      a single clock is changed and all other changes are propagations thereof.
    //      a complex update (multiple factors adapted) would be handled better by calculating the
    //      (minimized) tree diff and then running callbacks based on that.
    //      For predetermined (fast DVFS) reconfigurations the notifications could be cached
    //      (maybe with some automated way to invalidate the cache on other (non-DVFS) tree changes.
    /* only check for applicable callbacks if there are registrations */
    if (registered_clk_change_cb_cnt) {
        gclk_manager_notify_clk_change(adapted_clk, f_old, adapted_clk_new_freq, false);
    }

    switch (s->approach) {
        case SCALE_DIRECT:
            new_freq = gclk_set_freq(s->scale_clk, freq);
            break;
        case SCALE_UPTREE_RELATIVE:
            gclk_set_factor(s->scale_clk, new_uptree_factor);
            break;
        case SCALE_SEQUENCE:
            _gclk_manager_run_sequence__dyn_freq(s->sequence, s->sequence_len, freq);
            new_freq = gclk_get_current_freq(gclk_core_clock_handle);
            break;
        case SCALE_INTERMEDIATE_TOPO_AUTO:
            {
            //TODO: unify with derive sequence function, update notification handling
            gclk_manager_sequence_step_t adhoc_seq[20];
            int seq_len = 0;
            gclk_manager_sequence_step_t *seq = NULL;
            for (unsigned i = 0; i < _mgr_ctx.dfs_frequencies_cnt; i++) {
                if (_mgr_ctx.topology_conf_cache[i][0].clk_freq == freq) {
                    seq = &_mgr_ctx.prepared_rescale_sequences[i][0];
                    seq_len = _mgr_ctx.prepared_rescale_sequence_lengths[i];
                    new_freq = freq;
                    break;
                }
            }

            /* if there is no applicable prepared sequence, derive it on the fly
             * (expensive, but okay for manual testing purposes) */
            if (!seq) {
                uint32_t max_topo_len = _mgr_ctx.max_clocks_in_core_topology;
                clk_topology_entry_t target_topology[max_topo_len];
                memset(target_topology, 0, sizeof(clk_topology_entry_t) * max_topo_len);
                target_topology[0].clk = gclk_core_clock_handle;
                target_topology[0].clk_freq = GCLK_INVALID_FREQ;
                gclk_cmp_func_t cmp_func = gclk_cmp_topology_for_closest_leaf_freq;
                uint32_t target_freq = freq;
                size_t valid_cnt = 0;
                int force_nth = -1;
                /* brute force a configuration for the current topology */
                new_freq = gclk_manager_brute_force_freq_conf(gclk_core_clock_handle, target_topology, &max_topo_len, &_mgr_ctx.current_core_topo_id,
                        cmp_func, (void*)&target_freq, &valid_cnt, force_nth, NULL);

                seq_len = gclk_manager_derive_sequence(_mgr_ctx.current_core_topology, _mgr_ctx.current_core_topolen,
                                                       target_topology, max_topo_len, adhoc_seq, ARRAY_SIZE(adhoc_seq));
                seq = adhoc_seq;
            }

            if (seq_len > 0) {
                //gclk_manager_print_step_sequence(seq, seq_len);
                _gclk_manager_run_sequence__dyn_freq(seq, seq_len, freq);
                new_freq = gclk_get_current_freq(gclk_core_clock_handle);
            } else {
                printf("transition from [%s] topology from %d to %d infeasible!\n", gclk_get_name(gclk_core_clock_handle), _mgr_ctx.current_core_topo_id, _mgr_ctx.current_core_topo_id);
            }

            }
            break;
        default:
            return false;
    }

    if (registered_clk_change_cb_cnt) {
        gclk_manager_notify_clk_change(adapted_clk, f_old, adapted_clk_new_freq, true);
    }

    if (new_freq != freq) {
        LOG_DEBUG("most appliccable frequency was %lu Hz\n", new_freq);
    }

    if (new_freq == f_old) {
        return false;
    }

    return true;
}

void gclk_manager_print_step_sequence(gclk_manager_sequence_step_t *seq, size_t len) {
    for (unsigned i = 0; i < len; i++) {
        gclk_manager_sequence_step_t *step = &seq[i];

        switch (step->op) {
            case CLK_SET_FREQ:
                printf("CLK_SET_FREQ: [%s] %lu\n", gclk_get_name(step->clk), step->num_arg);
                //gclk_set_freq(step->clk, step->num_arg);
                break;
            case CLK_SET_FACTOR:
                printf("CLK_SET_FACTOR: [%s] %lu\n", gclk_get_name(step->clk), step->num_arg);
                //gclk_set_factor(step->clk, step->num_arg);
                break;
            case CLK_SET_PARENT:
                printf("CLK_SET_PARENT of [%s] to [%s] (%d)\n",
                        gclk_get_name(step->clk),
                        gclk_get_name(step->clk_arg),
                        gclk_parent2idx(step->clk, step->clk_arg));
                //gclk_set_parent(step->clk, gclk_parent2idx(step->clk, step->clk_arg));
                break;
            case CLK_SET_PARENT_IDX:
                printf("CLK_SET_PARENT_IDX of [%s] to [%lu] (%s)\n",
                        gclk_get_name(step->clk),
                        step->num_arg,
                        gclk_get_name(gclk_idx2parent(step->clk, step->num_arg)));
                //gclk_set_parent(step->clk, gclk_parent2idx(step->clk, step->clk_arg));
                break;
            case CLK_ENABLE:
                printf("CLK_ENABLE: [%s]\n", gclk_get_name(step->clk));
                //gclk_enable(step->clk);
                break;
            case CLK_DISABLE:
                printf("CLK_DISABLE: [%s]\n", gclk_get_name(step->clk));
                //gclk_disable(step->clk);
                break;
            case CLK_CONFIG_TARGET:
                printf("CLK_CONFIG_TARGET ERROR (only expect explicit steps)\n");
                break;
            case BUSY_SPIN:
                printf("BUSY_SPIN: [%lu]\n", step->num_arg);
                break;
            case SET_LED:
                printf("SET_LED: [%lu]\n", step->num_arg);
                break;
            default: printf("Illegal sequence step type!\n");
        }
    }
}

void gclk_manager_run_sequence(gclk_manager_sequence_step_t *steps, size_t step_cnt) {
    for (unsigned i = 0; i < step_cnt; i++) {
        gclk_manager_execute_sequence_step(&steps[i]);
    }
}

int gclk_manager_derive_sequence(const clk_topology_entry_t *src_topo, uint32_t src_len,
                                 const clk_topology_entry_t *target_topo, uint32_t target_len,
                                 gclk_manager_sequence_step_t *out_seq, unsigned max_seq_steps) {
    unsigned int src_tid = gclk_topology2id(src_topo, src_len);
    unsigned int target_tid = gclk_topology2id(target_topo, target_len);
    LOG_DEBUG("%s: derive a sequence to transition from topology %d to %d\n", __FUNCTION__, src_tid, target_tid);

    unsigned out_seq_idx = 0;
    size_t numof_topologies = gclk_get_topology_config_cnt(src_topo[0].clk);
    unsigned seq_ids[numof_topologies];
    int move_to_tmp_topo_seq_len = 0;

    if (src_tid == target_tid) {
        bool otf_update_possible = _can_be_changed_otf(src_topo, src_len);
        if (!otf_update_possible) {
            const gclk_manager_topo_switch_desc_t *direct = _get_intermediate_topo_switch_desc(src_tid);
            if (direct) {
                LOG_DEBUG("%s: derive sequence to temporarily change topo from %d to %d\n", __FUNCTION__, src_tid, direct->dst_topo_id);
                //TODO: find a valid frequency config for the intermediate topology so that we can use it to derive the sequence for
                //      transitioning there (the config is needed to translate target-config-specific CONFIG_TARGET-steps into fixed
                //      SET_FACTOR/SET_PARENT steps)
                move_to_tmp_topo_seq_len = _derive_sequence_chain(src_tid, direct->dst_topo_id, seq_ids, numof_topologies);
                if (move_to_tmp_topo_seq_len > 0) {
                    LOG_DEBUG("%s: intermediate step feasible..\n", __FUNCTION__);
                    /* update the starting point for remaining steps */
                    src_tid = direct->dst_topo_id;
                }
            } else {
                LOG_DEBUG("%s: transition infeasible (can neither change freqs on the fly nor switch topology)\n", __FUNCTION__);
                return -1;
            }
        } else { /* the transition should be possible by just updating frequency settings */
            /* TODO here some order flags or common-sense ordering might be needed at some point
             *      e.g. to avoid scaling up frequencies too high. E.g. one common pattern could be to first update
             *      the settings that reduce the effective clock an the update the settings that increase it only later on. */
            for (unsigned i = 0; i < target_len; i++) {
                if (out_seq_idx >= max_seq_steps) {
                    return -1;
                }
                if (gclk_is_scalable(target_topo[i].clk)) {
                    out_seq[out_seq_idx].clk = target_topo[i].clk;
                    out_seq[out_seq_idx].op = CLK_SET_FACTOR;
                    /* convert parent to numeric parent idx */
                    out_seq[out_seq_idx].num_arg = target_topo[i].factor;
                    out_seq_idx++;

                }
                if (gclk_is_muxable(target_topo[i].clk)) {
                    out_seq[out_seq_idx].clk = target_topo[i].clk;
                    out_seq[out_seq_idx].op = CLK_SET_PARENT_IDX;
                    /* convert parent to numeric parent idx */
                    out_seq[out_seq_idx].num_arg = target_topo[i].par_idx;
                    out_seq_idx++;
                }
            }
            /* no further steps required */
            return out_seq_idx;
        }
    }

    int res = _derive_sequence_chain(src_tid, target_tid, &seq_ids[move_to_tmp_topo_seq_len], numof_topologies - move_to_tmp_topo_seq_len);
    if (res > 0) {
        res += move_to_tmp_topo_seq_len;
        LOG_DEBUG("%s: got valid sequence chain that consists of %d sub sequences: ", __FUNCTION__, res);
        for (int i = 0; i < res; i++) {
           LOG_DEBUG(" %u ", seq_ids[i]);
        }
        LOG_DEBUG("\n");

        for (int i = 0; i < res; i++) {
            const gclk_manager_topo_switch_desc_t *sd = &core_clk_topo_switch_descs[seq_ids[i]];
            const gclk_manager_sequence_step_t *steps = sd->steps;

            LOG_DEBUG("%s: append %u steps to transition [%s] from topology %u to %u\n", __FUNCTION__, sd->step_cnt,
                      gclk_get_name(src_topo[0].clk), sd->src_topo_id, sd->dst_topo_id);


            uint32_t max_tmp_topo_len = gclk_get_clk_subtree_max_depth(src_topo[0].clk, 0);
            clk_topology_entry_t tmp_topology[max_tmp_topo_len];
            const clk_topology_entry_t *next_topo_conf = target_topo;
            uint32_t next_topo_len = target_len;

            /* for intermediate topologies valid frequency configurations need to be found */
            if (i < (res - 1)) {
                /* get a valid topology configuration for the intermediate topology,
                 * using the frequency of the final topology frequency also as the temporary frequency goal */
                memset(tmp_topology, 0, sizeof(clk_topology_entry_t) * max_tmp_topo_len);
                tmp_topology[0].clk = src_topo[0].clk;
                tmp_topology[0].clk_freq = GCLK_INVALID_FREQ;
                gclk_cmp_func_t cmp_func = gclk_cmp_topology_for_closest_leaf_freq;
                int tmp_tid = sd->dst_topo_id;
                uint32_t tmp_target_freq = target_topo[0].clk_freq;
                size_t valid_cnt = 0;
                int force_nth = -1;
                /* brute force a configuration for the temporary topology */
                uint32_t matched_tmp_freq = gclk_manager_brute_force_freq_conf(src_topo[0].clk, tmp_topology, &max_tmp_topo_len,
                                                                               &tmp_tid, cmp_func, (void*)&tmp_target_freq,
                                                                               &valid_cnt, force_nth, NULL);

                if (matched_tmp_freq == GCLK_INVALID_FREQ) {
                    LOG_DEBUG("%s: no valid frequency config for temporary topology!\n", __FUNCTION__);
                    return 0;
                }

                next_topo_conf = tmp_topology;
                next_topo_len = max_tmp_topo_len;
            }

            /* TODO insert notify pre/post steps here? */
            for (unsigned n = 0; n < sd->step_cnt; n++) {
                const gclk_manager_sequence_step_t *step = &steps[n];

                const clk_topology_entry_t *target_clk_conf = NULL;

                for (unsigned x = 0; x < next_topo_len; x++) {
                    if (next_topo_conf[x].clk == step->clk) {
                        target_clk_conf = &next_topo_conf[x];
                        break;
                    }
                }

                if (out_seq_idx >= max_seq_steps) {
                    return -1;
                }
                /* always copy the step from predefined sequence (and only overwrite where variable values need to be taken from target config) */
                memcpy(&out_seq[out_seq_idx], step, sizeof(gclk_manager_sequence_step_t));
                switch (step->op) {
                    case CLK_SET_FREQ:
                        LOG_DEBUG("%s: CLK_SET_FREQ: [%s] %lu\n", __FUNCTION__, gclk_get_name(step->clk), step->num_arg);
                        break;
                    case CLK_SET_FACTOR:
                        LOG_DEBUG("%s: CLK_SET_FACTOR: [%s] %lu\n", __FUNCTION__, gclk_get_name(step->clk), step->num_arg);
                        break;
                    case CLK_SET_PARENT:
                        LOG_DEBUG("%s: CLK_SET_PARENT of [%s] to [%s] (%d)\n", __FUNCTION__,
                                gclk_get_name(step->clk),
                                gclk_get_name(step->clk_arg),
                                gclk_parent2idx(step->clk, step->clk_arg));
                        /* replace parent instance with numeriacal idx */
                        out_seq[out_seq_idx].op = CLK_SET_PARENT_IDX;
                        out_seq[out_seq_idx].num_arg = gclk_parent2idx(step->clk, step->clk_arg);
                        break;
                    case CLK_SET_PARENT_IDX:
                        break;
                    case CLK_ENABLE:
                        LOG_DEBUG("%s: CLK_ENABLE: [%s]\n", __FUNCTION__, gclk_get_name(step->clk));
                        break;
                    case CLK_DISABLE:
                        LOG_DEBUG("%s: CLK_DISABLE: [%s]\n", __FUNCTION__, gclk_get_name(step->clk));
                        break;
                    case CLK_CONFIG_TARGET:
                        LOG_DEBUG("%s: CLK_CONFIG_TARGET: [%s]\n", __FUNCTION__, gclk_get_name(step->clk));
                        if (gclk_is_scalable(step->clk)) {
                            if (out_seq_idx >= max_seq_steps) {
                                return -1;
                            }
                            out_seq[out_seq_idx].clk = step->clk;
                            out_seq[out_seq_idx].op = CLK_SET_FACTOR;
                            /* convert parent to numeric parent idx */
                            out_seq[out_seq_idx].num_arg = target_clk_conf->factor;
                        }
                        if (gclk_is_muxable(step->clk)) {
                            if (gclk_is_scalable(step->clk)) {
                                out_seq_idx++;
                            }
                            out_seq[out_seq_idx].clk = step->clk;
                            out_seq[out_seq_idx].op = CLK_SET_PARENT_IDX;
                            /* convert parent to numeric parent idx */
                            out_seq[out_seq_idx].num_arg = target_clk_conf->par_idx;
                        }
                        if (! (gclk_is_scalable(step->clk) || gclk_is_muxable(step->clk))) {
                            /* drop the previously memcopied step as it effectively is a no-op */
                            out_seq_idx--;
                        }

                        break;
                    default: LOG_ERROR("%s: Illegal sequence step type!\n", __FUNCTION__);
                }
                out_seq_idx++;
            }
        }
    }

    return out_seq_idx;
}
