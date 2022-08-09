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

/* this callback type is used to issue core clock changes. Depending on which implementation sits
 * behind it, that may be a very efficient operation (only changing a prescaler), a slightly more
 * expensive variant that also (pre- and post-) notifies registered clients that are affected by
 * this change, or even a very complex one that temporarily adapts the source topology of the clock
 * to be sourced by a different clock in order to actually be able to change its value.
 * The latter case is required for clocks that can not be directly scaled during operation but must
 * be swisched off and on again when changing their config.
 * Things that use this interface are e.g. the frequency-cycler-thread that changes the core
 * frequency while collecting metadata to calculate the PU metric for the different running threads.
 * Another use for this is when actually applying DVFS to switch to the most appropriate frequency
 * of the thread being executed */
typedef void (*freq_reconf_cb_t)(uint32_t new_freq);

/* custom hooks required to tap into peripheral re-init code for clock config changes */
extern ztimer_periph_timer_t *___ztimer_periph_timer_instance;
extern timer_cb_t __ztimer_perph_timer_cb;
static unsigned int _timer_cnt_backup = 0;
extern void timer_write(tim_t tim, unsigned int cnt);

extern const gclk_t *gclks[GCLK_NUM_OF_CLOCKS];

/* a function to set up a configuration that works well with the given scale setting */
static bool _setup_default_dfs_topology_config(const gclk_scale_setting_t *scs);

static unsigned _populate_applicable_clock_constraints(gclk_freq_constraint_t *acc, clk_topology_entry_t *topo, uint32_t topo_len);
static bool _breaks_constraint(const gclk_freq_constraint_t *constraints, unsigned constr_cnt, clk_topology_entry_t *topo_conf, uint32_t topo_len);
static void _model_propagate_conf_change_downtree(clk_topology_entry_t *changed_conf, clk_topology_entry_t *tree_model, size_t tree_model_size);

uint8_t active_freq_constraints[GCLK_FREQ_LIMIT_CLKS_NUMOF];

static bool auto_vscale_enabled = false;
static bool auto_wsadapt_enabled = false;

mutex_t clock_conf_mutex = MUTEX_INIT;

typedef struct {
    uint32_t *freqs;
    uint32_t cpu_time_threshold_ticks;
    uint32_t thread_schedule_threshold;
    /* bit cache that holds a bit for each thread that PU statistics were requested for */
    uint32_t pu_stats_requested;
    /* bit cache for pending pu stats for the currently active freq of the cycle.
     * Is set to pu_stats_requested_tbc before measuring at the next cycle freq */
    uint32_t pu_stats_pending_cur_freq;
    unsigned freq_cnt;
    unsigned cur_freq_idx;
    mutex_t  done_mutex;
    freq_reconf_cb_t freq_change_cb;
    bool freq_cycle_enabled;
} freq_cycle_thread_context_t;

typedef struct {
    uint32_t cpu_time_ticks;
    uint32_t schedules;
} task_util_metrics_t;

/* This list holds one list of notification callbacks per clock that callbacks were registered for.
 **/
static list_node_t clock_change_notify_list;

/* registrations are simply counted on reg/unreg so the above list must not be iterated
 * to get the registration count */
static unsigned int registered_clk_change_cb_cnt = 0;

/* holds the last frequency set up by the clock manager */
volatile uint32_t current_core_freq;

/* holds the core frequency that was active when DFS was enabled to restore that setting later */
uint32_t pre_dfs_enable_freq = 0;

/* variables used to collect metadata on scheduling and busy/idle time to calculate
 * overall CPU utilization and the performance utilization metric for running threads */
uint64_t t_went_idle;
uint64_t t_left_idle;
uint64_t t_cur_thread_start;
volatile uint32_t enter_idle_cnt;
volatile uint32_t idle_ticks;
volatile uint32_t busy_ticks;
volatile uint32_t utilization;

/* basic averages of the above to reduce feedback speed and improve stability against outliers */
volatile uint32_t busy_ticks_avg;
volatile uint32_t utilization_avg;
volatile uint32_t idle_ticks_avg;

/* min max values just for debugging/testing purposes to provide insight of the value ranges and
 * help deciding if tick resolution is limiting accuracy */
volatile uint32_t idle_ticks_min = 0xFFFFFFFF;
volatile uint32_t idle_ticks_max = 0;
volatile uint32_t busy_ticks_min = 0xFFFFFFFF;
volatile uint32_t busy_ticks_max = 0;

/* Max number of threads to reserve memory for, that stores task util metrics */
#define TASK_UTIL_TASK_NUM       (10)

/* performance utilization for each thread */
int task_performance_util[TASK_UTIL_TASK_NUM];

/* below parameters are used to control how the thread-based dynamic frequency scaling is applied
 * and in general how the colck manager is able to control the clocks (e.g. which frequencies are
 * allowed/intended to be set up) */

uint32_t freq_sched_cnt[MAX_DFS_FREQ_VALUES_NUM];
int current_scale_idx = MAX_DFS_FREQ_VALUES_NUM - 1;

/* the currently active scale setting that is applied for scaling the core clock (setup by manager init) */
static const gclk_scale_setting_t *active_core_scale_setting = NULL;

/* TODO: the sizes for both of those cached/prepared configs are based on s pessimistic worst case and
 *       should be replaced by a more efficient representation */
static clk_topology_entry_t topology_conf_cache[MAX_DFS_FREQ_VALUES_NUM][GCLK_NUM_OF_CLOCKS];
#define MAX_PREPARED_SEQUENCE_LEN (GCLK_NUM_OF_CLOCKS)
static gclk_manager_sequence_step_t prepared_rescale_sequences[MAX_DFS_FREQ_VALUES_NUM][MAX_PREPARED_SEQUENCE_LEN];
static int prepared_rescale_sequence_lengths[MAX_DFS_FREQ_VALUES_NUM];

/* this gets updated with a list of possible frequencies when setting the clock handle that is used
 * for dynamic frequency scaling. If the handle that is set up supports more values than this can hold
 * a subset of possible values is stored instead */
static uint32_t dfs_frequencies[MAX_DFS_FREQ_VALUES_NUM];

/* the holds the number of valid frequency settings contained in dfs_frequencies */
static unsigned int dfs_frequencies_cnt = 0;

/* This PU threshold defines if a higher frequency should be used for a specific thread
 * TODO: replace this value with a lookup function that can be parameterized and returns an
 *       optimal configuration for the actuall PU. Additionally, information on the flash/voltage
 *       scaling behavior can be added here e.g. to affect wether low voltage or fast flash is
 *       preferrable for a specific task. More global considereations (how is this decision
 *       affected by other therads, and overhead of reconfiguration steps) should also be
 *       evaluated */
volatile int pre_sched_freq_boost_threshold = 60;
/* the threshold that defines if a lower frequency should be set up for the next thread */
volatile int pre_sched_freq_throttle_threshold = 30;

/* fast boost frequency that is applied if a thread has a 'high' PU value
 * (as indicated by pre_sched_freq_boost_threshold) */
volatile uint32_t pre_sched_boost_freq;

/* slow throttle frequency that is applied if a thread has a 'low' PU value
 * (as indicated by pre_sched_freq_boost_threshold) */
volatile uint32_t pre_sched_throttle_freq;

/* below values are used to control features of the clock manager at runtime */

/* controls whether dynamic frequency scaling is applied before scheduling a thread */
volatile bool pre_sched_pu_dfs_enabled = false;

/* controls whether scheduling/thread/timing metadata is collected to assess the PU metric during
 * execution. Usually this should be enabled together with with the frequency scaler-thread that
 * proactively cycles between frequencies.
 * TODO: It would also be possible to use an opportinustic approach that changes the frequency onlya thread is executed at. */
bool pu_metadata_collection_enabled = false;

/* controls if DVFS should be applied based on overal CU utilization (not thread PU)
 * TODO: this should be merged with pre_sched_pu_dfs etc. to define the adaptation method just at one place */
bool cpu_util_based_dvfs_enabled = false;

/* This policy defines which optimization goal to prefer in cases where different optimizations are
 * possible but some parameters are mutually exlusive */
static gclk_manager_dvs_policy_t dvs_policy = DVS_PREFER_LOW_VOLTAGE;

/* absolute maximum number of clocks involved in a topology of any clock */
static unsigned int max_clocks_in_topology;

/* absolute maximum number of clocks involved in a topology of the core clock */
static unsigned int max_clocks_in_core_topology;

/* below values are cached for faster operation an therefore need to be updated at relevant changes */
/* length of current topology that drives the core handle */
static unsigned int current_core_topolen;

/* holds the currently active topology id for the core clock handle */
static int current_core_topo_id;

/* meant to hold the topology that is currently driving the core clock
 * the size is an absolute worst case for now */
static clk_topology_entry_t current_core_topology[GCLK_NUM_OF_CLOCKS];

/* is used decide in which group a specific frequency should be put */
/* defines how the history is weighted for moving average calculation of
   time frequency products */
#define TASK_UTIL_MAVG_STEPS     (10)

/* Core-clock change notification list */
static gclk_clock_change_notify_list_t ccnl[GCLK_FREQ_LIMIT_CLKS_NUMOF];

/* schedule and timing metadata for each threads execution at different frequencies */
volatile task_util_metrics_t task_perf_util_data[TASK_UTIL_TASK_NUM][MAX_DFS_FREQ_VALUES_NUM];

static void _freq_change_scale_auto(uint32_t new_freq);

/* TODO: implement functions to switch this to other implementations at runtime */
/* Depending on the clock configuration, topology, dependencies and the DVFS-scheme at use, the
 * mathod to switch the frequency may be changed at runtime.
 * If, for example, it is known beforehand (still at runtime) that no other clock instance is
 * affected by changing the core clock (because no dependent peripharal is in use or because the
 * core is clocked by a completely independent instance), it may be possible to completely avoid
 * checks of whether a clock is affected and also not callbacks are required.
 * The same applies to clocks where it is known that no complex transition mechanism (temporary
 * swithcing to another clock) is needed. */
freq_reconf_cb_t freq_change_cb = _freq_change_scale_auto;

freq_cycle_thread_context_t fc_ctx = {
    .freqs = dfs_frequencies,
    //.freq_cnt = dfs_frequencies_cnt,
    //.cycle_us = atoi(argv[3]),
    .done_mutex = MUTEX_INIT,
    .freq_change_cb = _freq_change_scale_auto,
};

/* A freq change implementation that just maps to the core scale funtion that uses scale settings that apply
 * to the current topology */
static void _freq_change_scale_auto(uint32_t new_freq) {
    gclk_manager_scale_core_freq(new_freq);
}

///* returns number of appliccable factors that are >= min && <= max; and the idexes of those (via min/maxidx params)*/
//static unsigned _get_closest_min_max_factors(const gclk_t *clk, uint32_t minf, uint32_t *minidx, uint32_t maxf, uint32_t *maxidx){
//    /* init to invalid mul/div factors */
//    uint32_t cmin = 0;
//    uint32_t cmax = 0;
//    unsigned valid_cnt = 0;
//
//    for (unsigned i = 0; i < gclk_factor_cnt(clk); i++) {
//        uint32_t tmpf = gclk_idx2factor(clk, i);
//        if (tmpf >= minf && tmpf <= maxf) {
//            valid_cnt++;
//            if (tmpf >= minf && (cmin == 0 || cmin > tmpf)) {
//                cmin = tmpf;
//                *minidx = i;
//            }
//            if (tmpf <= maxf && (cmax == 0 || cmax < tmpf)) {
//                cmax = tmpf;
//                *maxidx = i;
//            }
//        }
//    }
//
//    return valid_cnt;
//}

static void _append_dfs_cache_entry(unsigned cidx, uint32_t freq, uint32_t factor) {
    //TODO it could be benefitial to store either the equivalent downtree factors or the scale factors that correspond to the
    //     frequencies that are being set up (to avoid translating between freq and factor ad hoc)
    topology_conf_cache[cidx][0].clk_freq = freq;
    topology_conf_cache[cidx][0].factor = factor;
    dfs_frequencies[cidx] = freq;
}

int gclk_mananger_set_default_dfs_frequencies(void) {
    if (!active_core_scale_setting) {
        printf("no active core scale setting defined\n");
        return 0;
    }
    int res = gclk_mananger_set_dfs_frequencies(active_core_scale_setting->default_freqs, active_core_scale_setting->default_freqs_cnt);
    return res;
}

const gclk_t* gclk_manager_get_core_clock_handle(void) {
    return gclock_core_clock_handle;
}

/* crude helper to force update of cached state */
static void _update_cached_state_vars(void) {
    current_core_topolen = gclk_get_current_topology_len(gclock_core_clock_handle);
    current_core_topology[0].clk = gclock_core_clock_handle;
    gclk_get_current_topology_config(current_core_topology, current_core_topolen);
    current_core_topo_id = gclk_topology2id(current_core_topology, current_core_topolen);

    /* always assume there is no applicable scale setting in case none can be found */
    active_core_scale_setting = NULL;
    /* select first appliccable scale setting for the current core topology as the active scale setting
     * that will be used by automatic scale operations (e.g., via gclk_manager_scale_core_freq()) */
    for (unsigned i = 0; i < SCALE_SETTINGS_NUMOF; i++) {
        if (scale_settings[i].output_clk == gclock_core_clock_handle &&
            scale_settings[i].topology_id == current_core_topo_id) {
            active_core_scale_setting = &scale_settings[i];
            break;
        }
    }
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

    //printf("abs_req_min_ws_ff: %u\n", abs_req_min_ws_ff);
    //printf("abs_req_min_vc_ff: %u\n", abs_req_min_vc_ff);
    //printf("abs_req_min_ws_lv: %u\n", abs_req_min_ws_lv);
    //printf("abs_req_min_vc_lv: %u\n", abs_req_min_vc_lv);

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

/* A cache that holds the state of clocks which put up limiting constaints on core voltage or flash access.
 **/
clk_topology_entry_t constrained_clocks_conf_cache[GCLK_FREQ_LIMIT_CLKS_NUMOF];

void _init_dvs_wsa_constraint_cache(void) {

    /* for all clock instances with assinged vcore/flash-waitstate constraints, load the current config into the cache */
    for (unsigned i = 0; i < GCLK_FREQ_LIMIT_CLKS_NUMOF; i++) {
        constrained_clocks_conf_cache[i].clk = gclk_freq_conf_limits[i].clk;
        gclk_get_current_topology_config(&constrained_clocks_conf_cache[i], 1);
    }

    unsigned min_ws;
    unsigned min_vc_idx;

    /* determine the best configuration that fulfilll all constraints and set it up */
    gclk_get_min_required_ws_vc_from_tree_config(constrained_clocks_conf_cache, GCLK_NUM_OF_CLOCKS, &min_ws,  &min_vc_idx, dvs_policy);

    unsigned cur_ws = flash_opt_get_wait_states();
    unsigned cur_vc = core_voltage_get();

    if ((cur_ws != min_ws) || (cur_vc != min_vc_idx)) {
        printf("WARNING! vcore/flash-waitstate settings werent set up to the best determined config was %u WS and %u VC (instead of %u WS and %u VC)\n",
                cur_ws, min_ws, cur_vc, min_vc_idx);
    }

    flash_opt_set_wait_states(min_ws);
    core_voltage_set(min_vc_idx);
}

int gclk_manager_init(void) {
    //TODO: this should be updated with code that checks the initial clock config (active topology), and saves the
    //      most appliccable scale_setting instead of the dfs_clock_handle. The actual DFS clock handle may not even
    //      be a single instance (e.g. for a multi-instance PLL configuration) and this info should not be needed from
    //      outside of the manager anyway.
    /* set default clock handle for dynamic scaling from static clock manager configuration. */
    //gclk_manager_set_dfs_clock_handle(scale_settings[0].clk);
    max_clocks_in_topology = gclk_get_max_topology_depth();
    max_clocks_in_core_topology = gclk_get_clk_subtree_max_depth(gclock_core_clock_handle, 0) + 1;

    _init_dvs_wsa_constraint_cache();
    _update_cached_state_vars();
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
        scale_settings[i].topology_id == current_core_topo_id) {
        active_core_scale_setting = &scale_settings[i];
        gclk_mananger_set_default_dfs_frequencies();
        return true;
    }
    return false;
}

const gclk_scale_setting_t* gclk_mananger_get_active_scale_setting(void) {
    return active_core_scale_setting;
}

int gclk_manager_get_allowed_core_clock_sources(const gclk_t ***clks) {
  *clks = &core_clock_sources[0];
  return CORE_CLOCK_SOURCES_NUMOF;
}

void gclk_manager_set_dvfs_pu_params(uint32_t fboost, uint32_t fthrottle, int fboost_pu_th, int fthrottle_pu_thesh) {
    pre_sched_boost_freq = fboost;
    pre_sched_throttle_freq = fthrottle;
    pre_sched_freq_boost_threshold = fboost_pu_th;
    pre_sched_freq_throttle_threshold = fthrottle_pu_thesh;
}

static void _do_freq_cycle_step_if_ready(void) {
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
                //idle_timer_wait((ctx->cycle_us + 500) / 1000);
                //xtimer_usleep(ctx->cycle_us);
            } else {
                /* disable freq cycle after all freqs were measured */
                fc_ctx.freq_cycle_enabled = false;
                mutex_unlock(&fc_ctx.done_mutex);
            }
        }
    }
}

unsigned int gclk_manager_get_dfs_freqs(uint32_t **freqs) {
    *freqs = &dfs_frequencies[0];
    return dfs_frequencies_cnt;
}

void _get_equivalent_factors_after_source(const gclk_t *source, clk_topology_entry_t *topo,
                                          size_t topo_len, uint32_t *mul, uint32_t *div) {
    uint32_t m = 1;
    uint32_t d = 1;

    for (unsigned i = 0; i < topo_len; i++) {
        /* only consider factors after the scaled clock till the output clock */
        if (topo[i].clk == source) {
            break;
        }
        if (gclk_is_divider(topo[i].clk)) {
            d *= gclk_get_current_factor(topo[i].clk);
        } else if (gclk_is_multiplier(topo[i].clk)) {
            m *= gclk_get_current_factor(topo[i].clk);
        }
    }

    *mul = m;
    *div = d;
}

void _get_equivalent_factors_of_topology(clk_topology_entry_t *topo, size_t topo_len,
                                         uint32_t *mul, uint32_t *div) {
    uint32_t m = 1;
    uint32_t d = 1;

    for (unsigned i = 0; i < topo_len; i++) {
        if (gclk_is_divider(topo[i].clk)) {
            d *= topo[i].factor;
        } else if (gclk_is_multiplier(topo[i].clk)) {
            m *= topo[i].factor;
        }
    }

    *mul = m;
    *div = d;
}

typedef struct {
    const uint32_t *freqs;
    size_t target_freqs_cnt;
    size_t match_freqs_cnt;
    uint32_t lowest_err;
    const gclk_t *scale_clk;
} lflae_cmp_fun_ctx_t;

int _clk_to_entry_idx(clk_topology_entry_t *topo, size_t len, const gclk_t *clk) {
    for (unsigned i = 0; i < len; i++) {
        if (topo[i].clk == clk) {
            return i; 
        }
    }
    return -1;
}

/* returns the equivalent factors */
void _get_equivalent_dt_factors(clk_topology_entry_t *topo, size_t len, const gclk_t *src, uint32_t *mul, uint32_t *div, bool incl_src) {
    int idx = _clk_to_entry_idx(topo, len, src);
    if (idx > 0) {
        _get_equivalent_factors_of_topology(topo, len - (len - idx) + (incl_src ? 1 : 0), mul, div);
    } else {
        printf("given src is not in topology!\n");
    }
}

gclk_cmp_result_t gclk_manager_cmp_lowest_freq_list_abs_err(clk_topology_entry_t *topo_best, size_t len1,
                                                            clk_topology_entry_t *topo_cmp, size_t len2,
                                                            void *arg) {
    lflae_cmp_fun_ctx_t *ctx = (lflae_cmp_fun_ctx_t*)arg;

    //uint32_t bmul;
    //uint32_t bdiv;
    //_get_equivalent_dt_factors(topo_best, len1, ctx->scale_clk, &bmul, &bdiv, false);
    (void)topo_best;
    (void)len1;

    uint32_t cmul;
    uint32_t cdiv;
    _get_equivalent_dt_factors(topo_cmp, len2, ctx->scale_clk, &cmul, &cdiv, false);

    uint32_t abs_err = 0; 
    size_t possible_freq_cnt = gclk_factor_cnt(ctx->scale_clk);

    int srcidx = _clk_to_entry_idx(topo_cmp, len2, ctx->scale_clk);

    uint32_t input_freq = topo_cmp[srcidx+1].clk_freq;
    uint32_t possible_freqs[possible_freq_cnt];

    for (unsigned i = 0; i < possible_freq_cnt; i++) {
        uint32_t factor = gclk_idx2factor(ctx->scale_clk, i);
        if (gclk_is_multiplier(ctx->scale_clk)) {
            possible_freqs[i] =  input_freq * cmul * factor / cdiv;
        } else {
            possible_freqs[i] =  input_freq * cmul / (factor * cdiv);
        }
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

bool _within_dfs_range(uint32_t freq) {
    if ((freq < DFS_CYCLER_MIN_FREQ) || (freq > DFS_CYCLER_MAX_FREQ)) {
        return false;
    }
    return true;
}

uint32_t _get_freq_for_factors(const gclk_t *clk, uint32_t f_in, uint32_t dt_mul, uint32_t dt_div, uint32_t fact) {
    if (gclk_is_multiplier(clk)) {
        return (uint64_t)f_in * (uint64_t)dt_mul * (uint64_t)fact / (uint64_t)dt_div; 
    } else {
        return (uint64_t)f_in * (uint64_t)dt_mul / ((uint64_t)fact * (uint64_t)dt_div); 
    }
}

static uint32_t _get_best_factor(const gclk_t *clk, uint32_t f_in, uint32_t target_freq, uint32_t dt_mul, uint32_t dt_div) {
    uint32_t min_diff = 0xFFFFFFFF;
    uint32_t factor_cnt = gclk_factor_cnt(clk);
    uint32_t best_factor = 0;
    for (unsigned i = 0; i < factor_cnt; i++) {
        uint32_t factor = gclk_idx2factor(clk, i);
        uint32_t f = _get_freq_for_factors(clk, f_in, dt_mul, dt_div, factor);
        uint32_t diff = f >= target_freq ? (f - target_freq) : (target_freq - f);
        if (diff < min_diff) {
            min_diff = diff;
            best_factor = factor;
        }
    }
    return best_factor;
}

clk_topology_entry_t *_clear_core_topology_cache(clk_topology_entry_t *cacheloc) {
    clk_topology_entry_t *topology = cacheloc;
    memset(topology, 0, sizeof(clk_topology_entry_t) * max_clocks_in_core_topology);
    topology[0].clk = gclock_core_clock_handle;
    topology[0].clk_freq = GCLK_INVALID_FREQ;
    return topology;
}


int _populate_dfs_freqs_bf(const gclk_scale_setting_t *scs, const uint32_t *freqs, size_t cnt) {
    size_t match_freq_cnt = 0;
    unsigned matched = 0; 
         
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
        uint32_t max_involved_clks = max_clocks_in_core_topology;
        clk_topology_entry_t *topology = _clear_core_topology_cache(&topology_conf_cache[0][0]);
        
        int tid = scs->topology_id;
        size_t valid_cnt = 0;
        int force_nth = -1;
        
        size_t possible_freq_cnt = gclk_factor_cnt(scs->scale_clk);

        /* if no freq values are provided explicitly, derive the target frequencies from the 
         * highest possible frequency at its minimal power configuration and using the available factors
         * of the scaled clock */
        if (freqs == NULL || cnt == 0) {
            match_freq_cnt = MAX_DFS_FREQ_VALUES_NUM <= possible_freq_cnt ? MAX_DFS_FREQ_VALUES_NUM : possible_freq_cnt;
            /* determine target frequencies by running bf for pmin of fmax config and then just use the list of possible factors
             * with the derived conf */
            uint32_t target_freq = DFS_CYCLER_MAX_FREQ;
            uint32_t leaf_freq = gclk_manager_brute_force_freq_conf(gclock_core_clock_handle, topology, &max_involved_clks,
                                                                    &tid, gclk_cmp_topology_for_closest_leaf_freq, (void*)&target_freq, &valid_cnt, force_nth, NULL);

            if (leaf_freq != GCLK_INVALID_FREQ) {
                target_freq = leaf_freq;
                uint32_t max_involved_clks = max_clocks_in_core_topology;
                clk_topology_entry_t *topology = _clear_core_topology_cache(&topology_conf_cache[0][0]);

                uint32_t pmin_freq = gclk_manager_brute_force_freq_conf(gclock_core_clock_handle, topology, &max_involved_clks,
                                                                        &tid, gclk_manager_cmp_topology_exact_leaf_freq_pmin, (void*)&target_freq, &valid_cnt, force_nth, NULL); 
                if (pmin_freq == GCLK_INVALID_FREQ) {
                    printf("ERROR: couldn't derive pmin config for %lu Hz\n", leaf_freq); 
                    return -2;
                }
                gclk_manager_print_topology_conf(topology, max_involved_clks, false, true);
            } else {
                return -1;
            }
            
        } else {
            match_freq_cnt = cnt <= possible_freq_cnt ? cnt : possible_freq_cnt;

            lflae_cmp_fun_ctx_t ctx = {
                .freqs = freqs,
                .target_freqs_cnt = cnt,
                .match_freqs_cnt = match_freq_cnt,
                .lowest_err = 0xFFFFFFFF,
                .scale_clk = scs->scale_clk,
            };

            uint32_t leaf_freq = gclk_manager_brute_force_freq_conf(gclock_core_clock_handle, topology, &max_involved_clks,
                                                                    &tid, gclk_manager_cmp_lowest_freq_list_abs_err, (void*)&ctx, &valid_cnt, force_nth, NULL);
            if (leaf_freq == GCLK_INVALID_FREQ) {
               return -1;
            }
        }

        uint32_t mul;
        uint32_t div;
        _get_equivalent_dt_factors(topology, max_involved_clks, scs->scale_clk, &mul, &div, false);

        int srcidx = _clk_to_entry_idx(topology, max_involved_clks, scs->scale_clk);
        /* TODO: replace this with a utility function that returns the the input freq
         * (for cases where the source itself is scalable) */
        uint32_t input_freq = topology[srcidx+1].clk_freq;
        
        uint32_t prev_freq = 0;

        for (unsigned i = 0; (i < possible_freq_cnt) && (matched < match_freq_cnt); i++) {
            uint32_t factor;
            /* if a set of specific target freqs was provided match them.
             * if not, just scale down the max freq via available factors */
            if (cnt > 0) {
                factor = _get_best_factor(scs->scale_clk, input_freq, freqs[i], mul, div);
            } else {
                factor = gclk_idx2factor(scs->scale_clk, i);
            }
            uint32_t possible_freq = _get_freq_for_factors(scs->scale_clk, input_freq, mul, div, factor);

            /* filter out duplicates and invalids on the fly */
            if (_within_dfs_range(possible_freq) && ((matched == 0) || (prev_freq != possible_freq))) {
                _append_dfs_cache_entry(matched++, possible_freq, factor);
                prev_freq = possible_freq;
            }
        }
        match_freq_cnt = matched;
    } else if (scs->approach == SCALE_INTERMEDIATE_TOPO_AUTO) {
        gclk_cmp_func_t cmp_func = gclk_manager_cmp_topology_exact_leaf_freq_pmin;

        uint32_t freq_step = (DFS_CYCLER_MAX_FREQ - DFS_CYCLER_MIN_FREQ) / (cnt - 1);
         
        for (unsigned i = 0; i < cnt; i++) {
            uint32_t max_involved_clks = max_clocks_in_core_topology;
            clk_topology_entry_t *topology = _clear_core_topology_cache(&topology_conf_cache[matched][0]);

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

            int tid = active_core_scale_setting->topology_id;
            size_t valid_cnt = 0;
            int force_nth = -1;
            uint32_t leaf_freq = gclk_manager_brute_force_freq_conf(gclock_core_clock_handle, topology, &max_involved_clks,
                                                                    &tid, cmp_func, (void*)&target_freq, &valid_cnt, force_nth, NULL);
            if (leaf_freq != GCLK_INVALID_FREQ) {
                /* ignore duplicates on the fly */
                if ((!matched) || (topology_conf_cache[matched-1][0].clk_freq != leaf_freq)) {
                    dfs_frequencies[matched] = leaf_freq;
                    gclk_manager_sequence_step_t *seq = &prepared_rescale_sequences[matched][0];
                    int seq_size = gclk_manager_derive_sequence(current_core_topology, current_core_topolen,
                                                                topology, max_involved_clks, seq, MAX_PREPARED_SEQUENCE_LEN);
                    if (seq_size > 0) {
                        prepared_rescale_sequence_lengths[matched] = seq_size;
                        matched++;
                    } else {
                        LOG_DEBUG("%s: transition from [%s] topology from %d to %d infeasible!\n", __FUNCTION__, gclk_get_name(gclock_core_clock_handle), current_core_topo_id, current_core_topo_id);
                    }
                }
            }
        }
        match_freq_cnt = matched;
    }

    return match_freq_cnt;
}

int gclk_mananger_set_dfs_frequencies(const uint32_t *freqs, size_t cnt) {
    /* reset dfs count before setting new values */
    dfs_frequencies_cnt = 0;

    if (!active_core_scale_setting) {
        printf("no active core scale setting defined\n");
        return -1;
    }

    if (cnt > MAX_DFS_FREQ_VALUES_NUM) {
        printf("Warning: can not cache %u frequency configs, will limit to at most %u entries\n",
                cnt, MAX_DFS_FREQ_VALUES_NUM);
    }
    
    int res = _populate_dfs_freqs_bf(active_core_scale_setting, freqs, cnt);

    if (res > 0) {
        dfs_frequencies_cnt = res;
    }

    return 0;
}

void gclk_manager_enable_pu_assessment(bool enable) {
    pu_metadata_collection_enabled = enable;
}

void gclk_manager_start_freq_cycler(unsigned int cycle_us, uint32_t min_schedules) {
    uint32_t initial_freq = gclk_get_current_freq(gclk_manager_get_core_clock_handle());

    fc_ctx.cpu_time_threshold_ticks = idle_timer_usecs_to_ticks(cycle_us);
    fc_ctx.thread_schedule_threshold = min_schedules;
    fc_ctx.freqs = dfs_frequencies;
    fc_ctx.freq_cnt = dfs_frequencies_cnt;
    fc_ctx.cur_freq_idx = 0;

    /* indicate that PU stats for the first frequency are pending for each requested thread */
    fc_ctx.pu_stats_pending_cur_freq = fc_ctx.pu_stats_requested;
    mutex_lock(&fc_ctx.done_mutex);

    /* set_up first frequency of the cycle */
    current_core_freq = fc_ctx.freqs[fc_ctx.cur_freq_idx];
    fc_ctx.freq_change_cb(current_core_freq);

    LOG_DEBUG("starting freq_cycle and waiting for it to finish...\n");
    fc_ctx.freq_cycle_enabled = true;

    mutex_lock(&fc_ctx.done_mutex);
    mutex_unlock(&fc_ctx.done_mutex);
    /* go back to default frequency */
    freq_change_cb(initial_freq);
    LOG_DEBUG("freq cycler done\n");
    current_core_freq = initial_freq;
}
uint32_t _append_performance_util_data(uint32_t task_id, uint32_t freq, uint32_t time_us);

static inline uint32_t _freq_interval_mean(uint32_t slot) {
    return dfs_frequencies[slot];
}

static inline uint32_t _freq_to_util_idx(uint32_t freq) {
    for (unsigned i = 0; i < ARRAY_SIZE(dfs_frequencies); i++) {
        if (dfs_frequencies[i] == freq) {
            return i;
        }
    }

    return -1;
}

int gclk_manager_calculate_pu_factor(uint32_t task_id, bool debug_print) {
    // TODO: calculate how much a taskbenefits from higher frequency
    int32_t pu_sum = 0;
    int32_t pu_cnt = 0;
    /* Theil-Sen like estimation */

    for (unsigned a = 0; a < MAX_DFS_FREQ_VALUES_NUM; a++) {
        for (unsigned b = a + 1; b < MAX_DFS_FREQ_VALUES_NUM; b++) {
            /* only use valid data points */
            if ((task_perf_util_data[task_id][a].schedules != 0) &&
                (task_perf_util_data[task_id][b].schedules != 0)) {

                /* use the middle fo the frequency slot as value for computation */
                int32_t freq_a = _freq_interval_mean(a) / 1000;
                int32_t freq_b = _freq_interval_mean(b) / 1000;
                /* must be normalized! */
                //int32_t freq_diff = freq_b - freq_a;
                int32_t freq_inc_fact = freq_b * 100 / freq_a;

                int32_t ta = task_perf_util_data[task_id][a].cpu_time_ticks;
                ta /= task_perf_util_data[task_id][a].schedules;
                int32_t tb = task_perf_util_data[task_id][b].cpu_time_ticks;
                tb /= task_perf_util_data[task_id][b].schedules;
                int32_t t_dec_fact = ta * 100 / tb;
                int32_t t_diff = tb - ta;

                int32_t task_speedup = ta * 100 / tb - 100;
                int32_t f_speedup = freq_b * 100 / freq_a - 100;
                int32_t pu = task_speedup * 100 / f_speedup;

                if (debug_print) {
                    printf("\nf: %ld %ld %ld %ld %%\n", freq_a * 1000, freq_b * 1000, (freq_b - freq_a) * 1000, freq_inc_fact);
                    printf("t: %ld %ld %ld %ld %%\n", ta, tb, t_diff, t_dec_fact);
                    printf("cpu time: %ld %ld\n", task_perf_util_data[task_id][a].cpu_time_ticks, task_perf_util_data[task_id][b].cpu_time_ticks);
                    printf("schedules: %ld %ld\n", task_perf_util_data[task_id][a].schedules, task_perf_util_data[task_id][b].schedules);
                    printf("PU: %ld\n", pu);
                }
                pu_sum += pu;
                pu_cnt++;
            }
        }
    }
    /* TODO: for now this depends on another instance actually triggering this calculation */
    task_performance_util[task_id] = pu_sum / pu_cnt;
    return task_performance_util[task_id];
}

uint32_t _append_performance_util_data(uint32_t task_id, uint32_t freq, uint32_t busy_ticks) {
    uint32_t freq_idx = _freq_to_util_idx(freq);
    task_perf_util_data[task_id][freq_idx].cpu_time_ticks += busy_ticks;
    task_perf_util_data[task_id][freq_idx].schedules++;

    if (task_perf_util_data[task_id][freq_idx].cpu_time_ticks >= fc_ctx.cpu_time_threshold_ticks &&
        task_perf_util_data[task_id][freq_idx].schedules >= fc_ctx.thread_schedule_threshold) {
        /* mark that enough stats were collected for this thread */
        fc_ctx.pu_stats_pending_cur_freq &= ~(1 << task_id);
    }
    return 0;
}

void gclk_manager_enable_pu_stat_request_for_thread(kernel_pid_t tid) {
    fc_ctx.pu_stats_requested |= (1 << tid);
}

void gclk_manager_clear_performance_util_data(void) {
    for (unsigned t = 0; t < TASK_UTIL_TASK_NUM; t++) {
        for (unsigned f = 0; f < MAX_DFS_FREQ_VALUES_NUM; f++) {
            task_perf_util_data[t][f].cpu_time_ticks = 0;
            task_perf_util_data[t][f].schedules = 0;
        }
    }

    fc_ctx.pu_stats_requested = 0;
    fc_ctx.pu_stats_pending_cur_freq = 0;
    enter_idle_cnt = 0;
}

void gclk_manager_pre_sched_hook(kernel_pid_t next_thread) {
    if (pu_metadata_collection_enabled) {
        t_cur_thread_start = idle_timer_read();
    }
    if (pre_sched_pu_dfs_enabled) {
        if (task_performance_util[next_thread] >= pre_sched_freq_boost_threshold &&
            pre_sched_boost_freq != current_core_freq) {
            freq_change_cb(pre_sched_boost_freq);
            current_core_freq = pre_sched_boost_freq;
        } else if (task_performance_util[next_thread] <= pre_sched_freq_throttle_threshold &&
            pre_sched_throttle_freq != current_core_freq) {
            freq_change_cb(pre_sched_throttle_freq);
            current_core_freq = pre_sched_throttle_freq;
        }
    }
}

void gclk_manager_post_sched_hook(kernel_pid_t desched_thread) {
    if (pu_metadata_collection_enabled) {
        uint32_t busy_ticks = idle_timer_read() - t_cur_thread_start;
        _append_performance_util_data(desched_thread, current_core_freq, busy_ticks);
        _do_freq_cycle_step_if_ready();
    }
}

/* cases to consider for this hook:
   The CPU might be sleeping completely after this task and there might be no generic way to measure idle time
   The CPU might still have a clock signal, so we could use either a separate timer or a cycle counter to measure idle time

   @todo: issue an upscaling callback when idle is not reached within a dynamically set boundry */
void gclk_manager_on_idle_hook(void) {
    //CYCCNT_pre = DWT->CYCCNT;
    t_went_idle = idle_timer_read();

    /* cancel the critical busy time if not already happened busy-callback */
    if (enter_idle_cnt) {
        busy_ticks = t_went_idle - t_left_idle;

        if (busy_ticks > busy_ticks_max) {
            busy_ticks_max = busy_ticks;
        }

        if (busy_ticks < busy_ticks_min) {
            busy_ticks_min = busy_ticks;
        }

        busy_ticks_avg = (3 * busy_ticks_avg + busy_ticks) >> 2;
    }
    enter_idle_cnt++;
    /* @todo: start ll_timer, set callback for *next_xtimer*-equivalent timeout */
}

void gclk_manager_enable_dynamic_frequency_scaling(bool enable) {
    pre_sched_pu_dfs_enabled = enable;
    if (enable) {
        pre_dfs_enable_freq = gclk_get_current_freq(gclk_manager_get_core_clock_handle());
    } else {
        freq_change_cb(pre_dfs_enable_freq);
    }
}

static void _dvfs(uint32_t utilization) {

    unsigned state = irq_disable();
    /* dfs can only be applied if there are multiple freq settings available */
    if (dfs_frequencies_cnt > 0) {
        int old_scale_idx = current_scale_idx;

        if (utilization > 80) {
            current_scale_idx++;
        } else if (utilization < 60){
            current_scale_idx--;
        }

        if (current_scale_idx < 0) {
            current_scale_idx = 0;
        } else if ((uint32_t)current_scale_idx >= dfs_frequencies_cnt){
            current_scale_idx =  dfs_frequencies_cnt - 1;
        }

        if(current_scale_idx != old_scale_idx) {
            gclk_manager_scale_core_freq(dfs_frequencies[current_scale_idx]);
        }

        freq_sched_cnt[current_scale_idx]++;
    }

    irq_restore(state);
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

    t_left_idle = idle_timer_read();
    idle_ticks = t_left_idle - t_went_idle;
    if (idle_ticks > idle_ticks_max) {
        idle_ticks_max = idle_ticks;
    }

    if (idle_ticks < idle_ticks_min) {
        idle_ticks_min = idle_ticks;
    }

    idle_ticks_avg = (3 * idle_ticks_avg + idle_ticks) >> 2;

    /* @todo: schedule some kind of callback to notify when reaching a "critical high" busy time */
    utilization = (201 * busy_ticks + idle_ticks) / ((idle_ticks + busy_ticks) * 2);
    utilization_avg = (3 * utilization_avg + utilization) >> 2;
    //uint32_t utilization_avg = busy_ticks_avg / ((idle_ticks_avg + busy_ticks_avg) / 100);
    if (cpu_util_based_dvfs_enabled) {
        //printf("busy: %lu idle %lu util: %lu avg_util: %lu\n", busy_ticks, idle_ticks, utilization, utilization_avg);
        //printf("performing DVFS for %lu %% utilization\n", utilization_avg);
        if (idle_ticks == 0) {
            _dvfs(utilization_avg);
        } else {
            _dvfs(utilization);
        }
    }
}

void gclk_manager_print_util_metrics(void) {
    //uint32_t utilization = (busy_ticks * 100) / (idle_ticks + busy_ticks);
    //uint32_t utilization_avg = (busy_ticks_avg * 100) / (idle_ticks_avg + busy_ticks_avg);
    printf("done working @ %lu MHz\n", gclk_get_current_freq(gclk_manager_get_core_clock_handle()) / 1000000);
    printf("idle_cycles:    %lu\n", idle_ticks);
    printf("working_cycles: %lu\n", busy_ticks);
    printf("idle_ticks_min: %lu\n", idle_ticks_min);
    printf("idle_ticks_max: %lu\n", idle_ticks_max);
    printf("idle_ticks_avg: %lu\n", idle_ticks_avg);
    printf("busy_ticks_min: %lu\n", busy_ticks_min);
    printf("busy_ticks_max: %lu\n", busy_ticks_max);
    printf("busy_ticks_avg: %lu\n", busy_ticks_avg);
    printf("utilization:    %lu\n", utilization);
    printf("util_avg:       %lu\n", utilization_avg);

    for (unsigned i = 0; i < dfs_frequencies_cnt; i++) {
        printf("used %lu Hz for %lu schedules\n", dfs_frequencies[i], freq_sched_cnt[i]);
        freq_sched_cnt[i] = 0;
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

bool _is_clk_modification_step(gclk_manager_sequence_step_t *step) {
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
bool _gclk_manager_is_derived_from_clock(const gclk_t *uptree_parent, const gclk_t *child, clk_topology_entry_t *tree_conf, size_t tree_clock_cnt) {

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

/* Some kind of transaction mechanism is needed for when multiple callbacks modify the requirements for WS/Vcore
 * in a contradicting way.
 * Either :
 * - some state must be handed to the callback so it can determine if this is the final change_cb
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
void _post_notify_commit(bool post_change) {
    if (auto_vscale_enabled || auto_wsadapt_enabled) {
        unsigned min_ws;
        unsigned min_vc_idx;

        gclk_get_min_required_ws_vc_from_tree_config(constrained_clocks_conf_cache, GCLK_NUM_OF_CLOCKS, &min_ws,  &min_vc_idx, dvs_policy);

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
            //gclk_get_current_topology_config(&constrained_clocks_conf_cache[i], 1);
        }
    }

}

static void _print_conf_change(clk_topology_entry_t *old, clk_topology_entry_t *new) {
    printf("%s changed from %8lu Hz (%s) to %8lu Hz (%s)\n", gclk_get_name(old->clk),
                                                             old->clk_freq, old->enabled ? "enabled" : "disabled",
                                                             new->clk_freq, new->enabled ? "enabled" : "disabled");
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
        gclk_get_min_required_ws_vc_from_tree_config(target_tree_conf, tree_size, &ws, &vc_idx, dvs_policy);
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

//static void _get_equivalent_uptree_factors(const gclk_t *dtc, const gclk_t *utc, uint32_t *mul, uint32_t *div) {
//    uint32_t m = 1;
//    uint32_t d = 1;
//    do {
//        if (gclk_is_divider(dtc)) {
//            d *= gclk_get_current_factor(dtc);
//        } else if (gclk_is_multiplier(dtc)) {
//            m *= gclk_get_current_factor(dtc);
//        }
//        dtc = gclk_get_current_parent(dtc);
//    } while (dtc != utc);
//
//    *mul = m;
//    *div = d;
//}

//static void _core_clock_change_cb(const gclk_t* altered_clk, const gclk_t* affected_clk,
//                                  uint32_t f_old, uint32_t f_new, bool post_change) {
//
//    /* if the changed clock is affected indirectly.. */
//    if (altered_clk != affected_clk) {
//        /* ..find out how the change to the altered clock affects the affected clock */
//        uint32_t mul = 1;
//        uint32_t div = 1;
//        _get_equivalent_uptree_factors(affected_clk, altered_clk, &mul, &div);
//
//        altered_clk = affected_clk;
//        f_old = f_old * mul / div;
//        f_new = f_new * mul / div;
//    }
//
//    if ((post_change && (f_new < f_old)) ||  /* the voltage can potentially be scaled down */
//        (!post_change && (f_new > f_old))) { /* the voltage must potentially be scaled up */
//        _update_vcore_and_ws_config(altered_clk, f_new, dvs_policy, (f_new > f_old) ? true : false);
//    }
//}

static void _lazy_reg_freq_limit_clk_change_cbs(void) {
    /* only register new callback if no automatic adaption is enabled yet */
    if (!(auto_vscale_enabled || auto_wsadapt_enabled)) {
        for (unsigned i = 0; i < GCLK_FREQ_LIMIT_CLKS_NUMOF; i++) {
            /* DVS just re-uses the notification mechanism to change the
             * voltage to an appropriate value before/after the frequency is adapted */
            gclk_manager_register_clk_change_cb(gclk_freq_conf_limits[i].clk, &ccnl[i],
                    _dvs_wsa_freq_constraint_change_cb);
        }
    }
}

static void _lazy_unreg_freq_limit_clk_change_cbs(void) {
    /* unregister callback if neither automatic adaption shall be enabled anymore */
    if (!(auto_vscale_enabled || auto_wsadapt_enabled)) {
        /* only disable if enabled */
        for (unsigned i = 0; i < GCLK_FREQ_LIMIT_CLKS_NUMOF; i++) {
            gclk_manager_unregister_clk_change_cb(&ccnl[i]);
        }
    }
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
    dvs_policy = policy;
}

gclk_manager_dvs_policy_t gclk_manager_get_dvs_policy(void) {
    return dvs_policy;
}

/* TODO: relocate from test app to gclk */
extern bool _set_freq(const gclk_t *clk, uint32_t freq);
void gclk_manager_setup_preferred_freqs(void) {
    for (uint32_t i = 0; i < GCLK_PREFERRED_FREQ_CONF_CNT; i++) {
        gclk_manager_set_freq(preferred_freqs[i].clk, preferred_freqs[i].preferred_freq);
    }
}

static void _cut_off_users(const gclk_t *clk, uint8_t *bitfield) {
    uint32_t child_idx = 0;
    const gclk_t *child = gclk_get_child(clk, child_idx);
    while (child != NULL) {
        if (gclk_is_enabled(child)) {
            /* @todo: critical for topological disable because enable cannot revert the change */
            gclk_disable(child);
            /* if it is still enabled after disable, we can not disable it so we must walk down the tree */
            if (gclk_is_enabled(child)) {
                _cut_off_users(child, bitfield);
            } else {
                /* we mark this one as disabled and do not disable any further clocks down this branch
                   because they shouldn't have a clock signal now anyway */
                bf_set(bitfield, gclk_get_index(child));
            }
        }
        child_idx++;
        child = gclk_get_child(clk, child_idx);
    }
}

/* @todo: needs to be protected agains parents that are currently NULL and have only one option that is a depended parent
   @todo: how to handle dependence of parents that are not directly sourced by the same clock but after some hops
          consider two classes of dependencies (provide different functions for that, if needed)?
          Let C be a clock that we want to change, S be its parent and I be a clock that is "independent" of S
          -up to source independence: completely different source i.e., I is not allowed to be sourced by the same clock that supplies S
          -intermediate independence: I is not allowed to be sourced by S (even if a chain of other nodes sits in between)
          -partial intermediate independence: I can be adjusted independent of S (even if I is sourced via S)*/
static const gclk_t* _another_independent_parent(const gclk_t *clk) {
    unsigned int parent_idx = 1;
    const gclk_t *current_parent = gclk_get_current_parent(clk);
    const gclk_t *other_parent = gclk_get_parent(clk, parent_idx);

    while (other_parent != clk) {
        if (other_parent != current_parent) {
            bool sourced_by = gclk_is_sourced_by(other_parent, current_parent);
            if ((other_parent != NULL) && !sourced_by) {
                return other_parent;
            }
            LOG_DEBUG("%s no valid option because it is sourced by %s\n", gclk_get_name(other_parent), gclk_get_name(current_parent));
        }

        parent_idx++;
        other_parent = gclk_get_parent(clk, parent_idx);
    }

    return clk;
}

/* @todo: this could be extended with more functionality:
    - constraints on when an action is tried to be executed
        -only on exact match, only on not disable xy, only on not altering freq xy etc..
    - explicit topology change: input target topology, dumb performing all required settings only for the give topo
    - explicit sequences: dumb execution of a predefined set of actions
        - disable clkA; set_parent clkA to clkB; set_freq clkA to xHz; enable clkA
        - would be very helpful generate (rather slim) but very expressive sequences without needing the code to deduce
          a required sequence at runtime
        - interactive test-app mode could be used to "record" such sequences for for "store" and "replay"
        - the base actions would be a a very limited set of operations that can be encoded in small action&data arrays
*/
//uint32_t call_cnt = 0;
void gclk_manager_transition(const gclk_t *clk, uint32_t freq) {
    (void)freq;
    //uint32_t state  = irq_disable();

    /* try to switch frequency directly */
    if (!(gclk_must_be_stopped_for_change(clk) || gclk_parent_must_be_stopped_for_change(clk))) {
        if (gclk_manager_set_freq(clk, freq)) {
            LOG_DEBUG("%s: _set_freq sucessful!\n", __FUNCTION__);
            return;
        }
    } else {
        LOG_DEBUG("%s: neither clock nor parent needs to be stopped\n", __FUNCTION__);
    }

    /* if direct scaling is not possible, find out why:
       -because flags indicate a clock can only be changed when it is stopped or its parent/children are stopped/disconnected (can be queried beforehand)
       -because this clock can actually not influence the frequency directly, in that case we can:
            - move up the tree till we find a parent that *can* influence the frequency and perform the change there
              - this information can actually be fixed in some cases where nodes above the clock have no other branches
                or, in other words, where the clock we want to change is an "exclusive" child, and changing the parent freq has no side-effects
                - Model as Flag of the parent? */
    const gclk_t *old_parent = gclk_get_current_parent(clk);
    /* find parent that doesn't depend on the clock we want to adjust */
    const gclk_t *another_parent = _another_independent_parent(clk);
    uint32_t another_parent_idx = gclk_parent2idx(clk, another_parent);
    uint32_t old_parent_idx = gclk_parent2idx(clk, old_parent);
    //clk_topology_entry_t temp_topology[topology_len_max];
    //clk_topology_entry_t *topo_bkup = &temp_topology[0];

    if (another_parent != clk) {
        LOG_DEBUG("will transition %s using %s (idx %lu)...\n", gclk_get_name(clk), gclk_get_name(another_parent), another_parent_idx);
        //reinit_trigger_conf_t reinit_confs[GCLK_REINIT_CONFIGS_CNT];
        //uint32_t old_topolen = _backup_current_topology(clk, reinit_confs, ARRAY_SIZE(reinit_confs), topo_bkup, topology_len_max);
        /* disconnect "possible all branches driven by this clock"
           (i.e. *not* flash access etc. check if we need blocks for that); save which */
        BITFIELD(disabled_clks, gclk_get_cnt());
        memset(disabled_clks, 0, sizeof(disabled_clks));

        //_prepare_potentially_affected_peripherals(reinit_confs, ARRAY_SIZE(reinit_confs));
        _cut_off_users(clk, disabled_clks);

        /* switch clk to tmp parent */
        gclk_set_parent(clk, another_parent_idx - 1);

        //_re_init_affected_peripherals(clk, reinit_confs, ARRAY_SIZE(reinit_confs), topo_bkup, old_topolen);

        /* disable old parent(s) if needed (recursively) */
        if (old_parent->flags.topology_flags & GCLK_STOP_FOR_UPDATE) {
            LOG_DEBUG("disabling %s for transition..\n", gclk_get_name(clk));
            gclk_disable(old_parent);
        };

        if (old_parent->flags.topology_flags & GCLK_STOP_PARENT_FOR_UPDATE) {
            const gclk_t *pp = gclk_get_current_parent(old_parent);
            LOG_DEBUG("disabling parent %s for transition..\n", gclk_get_name(pp));
            gclk_disable(pp);
        };

        /* adjust old parent */
        gclk_manager_set_freq(old_parent, freq);

        /* enable old parent(s) if needed (recursively) */
        if (old_parent->flags.topology_flags & GCLK_STOP_PARENT_FOR_UPDATE) {
            const gclk_t *pp = gclk_get_current_parent(old_parent);
            LOG_DEBUG("enabling %s for transition..\n", gclk_get_name(pp));
            gclk_enable(pp);
        }

        if (old_parent->flags.topology_flags & GCLK_STOP_FOR_UPDATE) {
            LOG_DEBUG("enabling %s for transition..\n", gclk_get_name(old_parent));
            gclk_enable(old_parent);
        };

        //_prepare_potentially_affected_peripherals();
        //printf("switching to %lu: %s\n", old_parent_idx - 1, gclk_get_name(gclk_get_parent(clk, old_parent_idx)));
        /* switch clk to old parent */
        gclk_set_parent(clk, old_parent_idx - 1);

        // The above was factored out to this function for now
        gclk_manager_setup_preferred_freqs();

        /* reinit "all children we disconnected" from clk with their targetted new freq */
        //_re_init_affected_peripherals(clk, reinit_confs, ARRAY_SIZE(reinit_confs), topo_bkup, old_topolen);

        /* reconnect "all children we disconnected" from clk */
        for (unsigned i = 0; i < gclk_get_cnt(); i++) {
            if (bf_isset(disabled_clks, i)) {
                gclk_enable(gclk_get(i));
            }
        }

        /* print after actually enabling them because UART may also be disabled ;) */
        LOG_DEBUG("children that were temporarily disconnected:\n");
        for (unsigned i = 0; i < gclk_get_cnt(); i++) {
            if (bf_isset(disabled_clks, i)) {
                LOG_DEBUG("[%s]\n", gclk_get_name(gclk_get(i)));
            }
        }

    } else {
        LOG_ERROR("can not transition using other parent because there is none!\n");
    }
    //irq_restore(state);
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

static bool _breaks_constraint(const gclk_freq_constraint_t *constraints, unsigned constr_cnt, clk_topology_entry_t *topo_conf, uint32_t topo_len) {
    for (unsigned c = 0; c < constr_cnt; c++) {
        for (unsigned t = 0; t < topo_len; t++) {
            if (constraints[c].clk == topo_conf[t].clk) {
                if ((constraints[c].type == GCLK_ENSURE_MIN_FREQ) &&
                    (topo_conf[t].clk_freq < constraints[c].freq)) {
                    return true;
                }
                if ((constraints[c].type == GCLK_ENSURE_MAX_FREQ) &&
                    (topo_conf[t].clk_freq > constraints[c].freq)) {
                    return true;
                }
            }
        }
    }
    return false;
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
                    gclk_print_topology_conf(ct, ct_len, true, false);
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

                    if (!_breaks_constraint(relevant_clock_constraints, rel_constr_cnt, ct, ct_len)) {
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
            gclk_print_topology_conf(topology, max_involved_clks, true, false);
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
            //current_core_topolen = max_involved_clks;
            //current_core_topo_id = target_topology;
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
    const gclk_scale_setting_t *s = active_core_scale_setting;
    if (!s) {
        printf("%s: no scale setting appliccable at the moment!\n", __FUNCTION__);
        return false;
    }

    LOG_DEBUG("%s: scale freq of [%s](topo %d) to %luHz\n", __FUNCTION__, gclk_get_name(gclock_core_clock_handle), current_core_topo_id, freq);

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
        for (unsigned i = 0; i < dfs_frequencies_cnt; i++) {
            if (topology_conf_cache[i][0].clk_freq == freq) {
                new_uptree_factor = topology_conf_cache[i][0].factor;
                adapted_clk_new_freq = topology_conf_cache[i][0].factor * gclk_get_input_freq(adapted_clk);
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
            new_freq = gclk_get_current_freq(gclock_core_clock_handle);
            break;
        case SCALE_INTERMEDIATE_TOPO_AUTO:
            {
            //TODO: unify with derive sequence function, update notification handling
            gclk_manager_sequence_step_t adhoc_seq[20];
            int seq_len = 0;
            gclk_manager_sequence_step_t *seq = NULL;
            for (unsigned i = 0; i < dfs_frequencies_cnt; i++) {
                if (topology_conf_cache[i][0].clk_freq == freq) {
                    seq = &prepared_rescale_sequences[i][0];
                    seq_len = prepared_rescale_sequence_lengths[i];
                    new_freq = freq;
                    break;
                }
            }

            /* if there is no applicable prepared sequence, derive it on the fly
             * (expensive, but okay for manual testing purposes) */
            if (!seq) {
                uint32_t max_topo_len = max_clocks_in_core_topology;
                clk_topology_entry_t target_topology[max_topo_len];
                memset(target_topology, 0, sizeof(clk_topology_entry_t) * max_topo_len);
                target_topology[0].clk = gclock_core_clock_handle;
                target_topology[0].clk_freq = GCLK_INVALID_FREQ;
                gclk_cmp_func_t cmp_func = gclk_cmp_topology_for_closest_leaf_freq;
                uint32_t target_freq = freq;
                size_t valid_cnt = 0;
                int force_nth = -1;
                /* brute force a configuration for the current topology */
                new_freq = gclk_manager_brute_force_freq_conf(gclock_core_clock_handle, target_topology, &max_topo_len, &current_core_topo_id,
                        cmp_func, (void*)&target_freq, &valid_cnt, force_nth, NULL);

                seq_len = gclk_manager_derive_sequence(current_core_topology, current_core_topolen,
                                                       target_topology, max_topo_len, adhoc_seq, ARRAY_SIZE(adhoc_seq));
                seq = adhoc_seq;
            }

            if (seq_len > 0) {
                //gclk_manager_print_step_sequence(seq, seq_len);
                _gclk_manager_run_sequence__dyn_freq(seq, seq_len, freq);
                new_freq = gclk_get_current_freq(gclock_core_clock_handle);
            } else {
                printf("transition from [%s] topology from %d to %d infeasible!\n", gclk_get_name(gclock_core_clock_handle), current_core_topo_id, current_core_topo_id);
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

static bool contains(unsigned *list, size_t len, unsigned elem) {
    for (unsigned i = 0; i < len; i++) {
        if (list[i] == elem) {
            return true;
        }
    }
    return false;
}

static size_t _append_if_not_contained(unsigned *list, size_t len, unsigned val) {
    if (!contains(list, len, val)) {
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
extern int _derive_sequence_chain(int stid, int ttid, unsigned *seq_chain, size_t topo_cnt) {
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

    //// print graph matrix
    //printf("   ");
    //for (unsigned i = 0; i < unique_tids; i++) {
    //    printf("%2d ", tidx_tid[i]);
    //}
    //printf("\n");
    //for (unsigned i = 0; i < unique_tids; i++) {
    //    printf("%d: ", tidx_tid[i]);
    //    for (unsigned j = 0; j < unique_tids; j++) {
    //        printf("%2d ", tid_graph[i][j]);
    //    }
    //    printf("\n");
    //}
    //printf("\n");



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
