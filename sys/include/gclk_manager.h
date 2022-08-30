/*
 * Copyright (C) 2021 HAW Hamburg <michel.rottleuthner@haw-hamburg.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */
/**
 * @defgroup    sys_gclk_manager Clock configuration manager
 * @ingroup     sys_gclk
 *
 * @{
 *
 * @file
 * @brief       Generic clock configuriton API
 *
 * Interface for high-level control of gclk and its interaction with related modules
 *
 *  @todo Some aspects that are still WIP/under consideration
 *  - It might be helpful to provide information on whether a clock is a *pure* itermediate
 *    or also a *consumer* itself. This would allow more advanced ressource allocation and management
 *    for automatic gating of unused clocks. A pure intermediate can be considered safe for disable if
 *    it has no active children, while a clock that is used for something even if it has no children
 *    shall not be disabled.
 *  - More advanced dynamic constraints would allow to (temporarily) limit adjustments as required:
 *    - Require a fixed frequency.
 *      - Require a clocks frequency to be within a specified frequency range.
 *    - Require a fixed topology.
 *    - Block gating.
 *    - There are nodes that are read-only regarding configuration, but are only usable under specific
 *      settings. E.g., the PLL on stm32l0x3 has a special 48 MHz output for USB, only valid when
 *      PLL_VCO is set to 96 MHz. Can be handled via dynamic constraint.
 *    - Inter-peripheral dependency management / coordination
 *      - E.g. peripheral 1 wants clock X at Y Hz, peripheral 2 wants clock X at Z Hz.
 *        - a device may lock a clock to a specific value to prohibit changes.
 *        - This info could be dynamically appended to a clock node, so other tree explorations and
 *          configurations treat incompatible settings as invalid or opt for alternatives.
 *        - For many cases those settings could also just be switched between modes on demand.
 *  - There are clocks that need other clocks to be enabled prior to configuration. How to handle this?
 *    - Explicit dependencies e.g., implemented via ressource allocation and runtime constraints.
 *    - dependency tree that holds *clock X requires clock Y* dependencies. Sould be optional as it is
 *      easy to workaround with platform init code and custom clock drivers that handle this manually.
 *  - Integrate more peripheral drivers that are affected by the clock config to the generic clock module
 *    to streamline init/reconfiguration steps and deduplicate code there.
 *
 * @author      Michel Rottleuthner <michel.rottleuthner@haw-hamburg.de>
 */
#ifndef GCLK_MANAGER_H
#define GCLK_MANAGER_H

#include "gclk.h"
#include "list.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Max number of tasks to allocate PU-stats memory for. */
#ifndef GCLK_MANAGER_PU_STATS_TASK_NUM
#define GCLK_MANAGER_PU_STATS_TASK_NUM 10
#endif

/** @brief Max number of prepared clock configs to allocate memory for.
 *
 * By default allocate enough space to store one prepared config per DFS frequency step.
 */
#ifndef GCLK_MANAGER_PREP_CONFS_MAX_NUMOF
#define GCLK_MANAGER_PREP_CONFS_MAX_NUMOF (MAX_DFS_FREQ_VALUES_NUM)
#endif

/**
 * @brief max number of clocks in a topology conf.
 *
 * There can never be more clocks in any topology than there are clocks.
 * @note This is a very pessimistic estimation, as in practice the number of clocks
 *       which are part of the core clock (sub-)topology is by far lower than this.
 * @todo Implement automatic determination of a more realistic platform-specific
 *       upper bound for this by exploring the maximum length of any possible
 *       core (sub-)topology.
 */
#ifndef GCLK_MANAGER_PREP_CONFS_TOPO_MAX_LEN
#define GCLK_MANAGER_PREP_CONFS_TOPO_MAX_LEN (GCLK_NUM_OF_CLOCKS)
#endif

/**
 * @brief max number of steps in a prepared reconfiguration sequence.
 *
 * By default we assume a reconfiguration sequence will touch each clock once at most.
 * @note This is a very pessimistic estimation, as in practice reconfiguration sequences
 *       involve far less operations.
 * @todo Implement automatic determination of a more realistic platform-specific
 *       upper bound for this by exploring a representative set of possible
 *       reconfiguration sequences.
 */
#ifndef GCLK_MANAGER_MAX_PREPARED_SEQUENCE_LEN
#define GCLK_MANAGER_MAX_PREPARED_SEQUENCE_LEN (GCLK_NUM_OF_CLOCKS)
#endif

/**
 * Default minimum CPU time per thread to accumulate before considering it enough
 * for valid PU metric calculation.
 */
#define GCLK_MANAGER_MIN_PU_STATS_CPU_TIME (32768)

/**
 * Default minimum number of thread schedules that must have occured per thread
 * before considering it enough for valid PU metric calculation.
 */
#define GCLK_MANAGER_MIN_PU_STATS_SCHEDULES (10)


/**
 * @brief Clock instance array that is provided by the platform implementation.
 *
 * This is defined in the low-level (hardware-specific) part of the implementation.
 */
extern const gclk_t *gclks[GCLK_NUM_OF_CLOCKS];

/**
 * @brief Clock change notififcation callback prototype.
 *
 * Defines the interface to be used for pre- and post- clock change callback functions.
 *
 * @param[in] altered_clk     The clock that was modified causing the callback to trigger.
 * @param[in] affected_clk    The affected clock this callback was registered for.
 * @param[in] f_old           The ololdrequency of the modified clock.
 * @param[in] f_new           The new frequency of the modified clock.
 * @param[in] post_change     True if the call indicates the change is about to happen.
 *                            False if the change was already executed.
 */
typedef void (*clock_change_cb_t)(const gclk_t* altered_clk, const gclk_t* affected_clk, uint32_t f_old, uint32_t f_new, bool post_change);

/**
 * @brief Core frequency reconfiguration callback type.
 *
 * This callback type is used to issue core clock changes. Depending on which implementation sits
 * behind it, that may be a very efficient operation (only changing a prescaler), a slightly more
 * expensive variant that also (pre- and post-) notifies registered clients that are affected by
 * this change, or even a very complex one that temporarily adapts the source topology of the clock
 * to be sourced by a different clock in order to actually be able to change its value.
 * The latter case is required for clocks that can not be directly scaled during operation but must
 * be switched off and on again when changing their config.
 * Things that use this interface are e.g. the frequency-cycler-thread that changes the core
 * frequency while collecting metadata to calculate the PU metric for the different running threads.
 * Another use for this is when actually applying DVFS to switch to the most appropriate frequency
 * of the thread being executed.
 */
typedef void (*gclk_manager_core_freq_reconf_cb_t)(uint32_t new_freq);

/**
 * @brief Clock change callback list type.
 *
 * A linkable type that stores one callback that is executed before and after
 * a clock frequency is changed.
 */
typedef struct {
    list_node_t node; /**< base type for constructing a linked list. */
    clock_change_cb_t change_cb; /**< callback that is executed as notification. */
} gclk_change_cb_list_t;

/**
 * @brief Clock change callback registration type.
 *
 * A linkable type that stores a clock and registered callbacks to be notified about
 * its frequency changes. For each clock that has registrations one such element constitutes
 * the entry point. Further registrations for the same clock are then linked via the
 * @ref change_cb_list member so that once a clock was identified to have pending
 * notifications all callcacks can iterated more quickly. */
typedef struct {
    list_node_t node; /**< base type for constructing a linked list. */
    const gclk_t *clk; /**< the clock this registration notifies changes for. */
    gclk_change_cb_list_t change_cb_list; /**< a list of all callbacks registered for
                                               this clock */
} gclk_clock_change_notify_list_t;

/**
 * @brief Desriptor for core voltage, frequency, and flash waitstate limits.
 *
 * Used to store platform-specific limits that apply to frequency, flash waitstates,
 * and core voltage. */
typedef struct {
    uint32_t freq_max; /**< maximum frequency up to which the below properties are applicable. */
    uint8_t vc_idx_min; /**< minimum required core voltage index. */
    uint8_t ws_min; /**< minimum required flash wait states. */
} freq_conf_limit_t;

/**
 * @brief Policies that affect which optimization goal is prioritized for DVS.
 *
 * There are cases where platform limits mutually exclude voltage or frequency optimizations.
 * I.e., further optimization of one parameter limits optimization of the other.
 * In those ranges, the (runtime configurable) policy decides what to prefer. */
typedef enum {
    DVS_PREFER_LOW_VOLTAGE, /**< prefer lower voltage operation over faster flash access. */
    DVS_PREFER_FAST_FLASH,  /**< prefer faster flash access over lower voltage operation. */
} gclk_manager_dvs_policy_t;

/**
 * @brief Basic operation types to be performed on clock instances.
 *
 * Those operations can be used to describe clock reconfiguration steps that operate
 * on the abstract clock configuration interface. This is useful to define complex reconfiguration
 * procedures as multistep sequences. Those may be prepared ahead of time ot even automatically
 * derived by dynamic exploration mechanisms.
 */
typedef enum {
    CLK_SET_FREQ,       /**< set a predefined frequency (fixed value given by sequence step) */
    CLK_SET_FACTOR,     /**< set a predefined factor (fixed value given by sequence step) */
    CLK_SET_PARENT,     /**< set a predefined parent (fixed value given by sequence step) */
    CLK_SET_PARENT_IDX, /**< set a predefined parent via index (fixed value given by sequence step) */
    CLK_ENABLE,         /**< guess what ;) */
    CLK_DISABLE,        /**< guess what ;) */
    CLK_CONFIG_TARGET,  /**< this is a placeholder operation to express the clock that this
                            sequence step is operating on, shall be set to the value it holds
                            in the target config.
                            Note: only sets parent/factor configs whereas enable/disable operations
                            must be encoded explicitly */
    /* Below types are only used for basic debugging/visualization purposes concerned with aspects
     * that appear during the execution of multi-step clock adjustments, where other debugging
     * are severely constrained or too invasive. */
    BUSY_SPIN,          /**< does some busy CPU spinning to delay further execution */
    SET_LED,            /**< enables /disables the debug led based on numval (1/0) */
} gclk_manager_op_id_t;

/**
 * @brief Clock reconfiguration step descriptor.
 *
 * This descriptor is meant to be used for building more complex high-level transition patterns
 * on top of the low-level API e.g. to express a specific configuration step which involves
 * multiple topology and frequency configuration changes.
 * Further it may be used to store (cache) automatically detected transition steps which are
 * expensive to discover but otherwise fast to execute. */
typedef struct {
    gclk_manager_op_id_t op;    /**< the operation to execute in this step */
    const gclk_t *clk;          /**< the clock to operate on or NULL if not needed for @ref op*/
    /** The parameter used by @ref op. Depending on the operation there is only one possible argument,
     *  hence the union. */
    union {
        gclk_t const *clk_arg;  /**< points to a clock if op takes a clock as parameter. */
        uint32_t num_arg; /**< holds a number if op needs a number as parameter
                               (e.g. a frequency/factor/idx). */
    };
} gclk_manager_sequence_step_t;

/**
 * @brief Topology switch descriptor.
 *
 * Points to a sequence that switches from one topology to another one and carries
 * related metadata. For now it is intended to be used only for the core clock so
 * it does not hold a clock reference. Topology ids always refer to the core-clock
 * specific topology ids.
 */
typedef struct {
    const gclk_manager_sequence_step_t *steps; /**< pointer to the first sequence step */
    uint8_t src_topo_id; /**< core-clock specific topology id that must be active before this
                              sequence can be applied. Refers to the n-th topology variant
                              able to drive the core clock, i.e., the topology with that index.*/
    uint8_t dst_topo_id; /**< topology id the transition will configure the clock to. */
    uint8_t step_cnt; /**< number of sequence steps in @ref steps */
} gclk_manager_topo_switch_desc_t;

/**
 * @brief Different scaling approaches.
 *
 * Different options of how clock scaling is performed.
 */
typedef enum {
    SCALE_DIRECT,            /**< clock can be scaled by changing a single scale factor of a clock instance directly. */
    SCALE_UPTREE_RELATIVE,   /**< clock can only be scaled indirectly by changing a single clock scaler up the tree,
                                  where the scaled clock frequency is not the same as the frequency at the output clock.
                                  I.e., the scaled clock is still subject to some sort of scaling before feeding the output clock. */
    SCALE_SEQUENCE,          /**< clock can be scaled by a predefined multi-step sequence */
    SCALE_INTERMEDIATE_TOPO_AUTO,  /**< clock can be scaled by temporarily changing to another topology before
                                        adapting the current one and switching back to it. AUTO refers to the
                                        fact that not a fixed intermediate topology must be used but instead the most
                                        viable option can be selected at runtime, depending on active constraints */
    /* TODO: evaluate simple mux-based scaling for bimodal switching between HF/LF clock(-topologies). */
} gclk_scale_approach_t;

/**
 * @brief Defines for one clock instance how it can be scaled.
 */
typedef struct {
    const gclk_t *output_clk; /**< The clock wich is updated to a new frequency with this scale setting (i.e. usualy the core clock) */
    union {
        const gclk_t *scale_clk; /**< If this setting uses the @ref SCALE_DIRECT or @ref SCALE_UPTREE_RELATIVE approach for scaling,
                                      the output_clock is scaled by only updating this clock.
                                      @note @ref SCALE_DIRECT does *not* necessarily imply the factor of output_clk handle itself is
                                      adapted. It only defines that there are no other scaling factors active between scale_clk and
                                      output_clk, (i.e., both run at the same frequency, but scaling is done via scale_clk). */
        gclk_manager_sequence_step_t *sequence; /**< A predefined sequence of steps to update the frequency. */
    };
    size_t sequence_len; /**< For approaches that use the sequence field, it defines the sequence length. */
    const uint32_t *default_freqs; /**< Default frequencies for this scale setting. Should be set to values preferred by the hardware.
                                        Setting this to NULL instructs the manager to determine frequencies automatically. */
    unsigned default_freqs_cnt: 8; /**< Number of default frequencies for this scale setting, if default_freqs != NULL. */
    unsigned topology_id: 8; /**< Topology (output_clk-specific ID) at which this scaling method is applicable. */
    gclk_scale_approach_t approach: 8; /**< The scaling approach used to adjust the frequency. */
} gclk_scale_setting_t;

/**
 * @brief A list of limits that apply to one specific clock handle.
 */
typedef struct {
    const freq_conf_limit_t *limits; /**< Core voltage and Wait state limits that apply to clk. */
    const size_t            len;     /**< The number of limits pointed to by limits. */
    const gclk_t            *clk;    /**< The clock instance these limits apply to. */
} clock_freq_conf_limits_t;

/**
 * @brief Physical clock properties of the clock power model.
 *
 * Respective values must be determined from a measurement study once per target platform.
 * With precise enough model parameters for all relevant clock nodes, the power consumption
 * of the clock-subsystem can be accurately pre-caluclated from the the clock configuration.
 * Alternative configuration variants can then be compared regarding their power consumption
 * in order to minimize it.
 */
typedef struct {
    const gclk_t *clk;   /**< The clock handle this data refers to */
    uint32_t P_en_nW;    /**< The static power this clock node draws when enabled (i.e. F(clk) > 0).
                              May be 0 for clocks that are not gateable. */
    uint32_t C_fF;       /**< The (equivalent) capacitance of the clock node in femto Farad.
                              This value affects the dynamic (frequency dependent) consumption.
                              May be 0 for clocks that only suport a fixed frequency. */
} gclk_manager_power_properties_t;

/**
 * @brief Callback type which hands over explored configurations.
 *
 * This callback is used to handover explored configuration settings from within the exploration context
 * to other entities which are interested in the explored results. This pattern is helpful to use the
 * potentially huge number of results on-the-fly as its size can usually neither be pre-determined nor
 * allocated completely. This allows to iteratively process each individual result of a full exploration
 * run (e.g. for immediate output), whithout storing all configs in memory.
 *
 * @param[in]  conf       The configuration found during exploration.
 * @param[in]  len        Number of config entries in @p conf.
 * @param[in]  res        The comparison result between @p conf and the best config explored before.
 * @param[in]  valid_idx  The zero based count of valid configurations that identifies the given config.
 *                        @note This value is clock handle, topology, and constraint-specific.
 * @param[in]  ctx        A callback-specific context that may be used for additional data and state
 *                        handed into and out of the callback.
 */
typedef void (*gclk_exploration_result_cb_t)(clk_topology_entry_t *conf, size_t len, gclk_cmp_result_t res, unsigned valid_idx, void *ctx);

/**
 * @brief Filters defining on which comparison results the exploration callback shall be called.
 */
typedef enum {
    CB_ON_VALID,     /**< cb is executed for every valid configuration found. */
    CB_ON_BEST_ONLY, /**< cb is executed only for the best configuration found. */
    CB_ON_BETTER,    /**< cb is executed everytime a better configuration is found.
                          @note This is mainly useful for debugging (observing the exploration). */
} gclk_exploration_result_cb_mode_t;

/**
 * @brief Configuration of the exploration result callback.
 */
typedef struct {
    gclk_exploration_result_cb_t valid_conf_found_cb; /**< The callback to execute per result. */
    void *ctx; /**< The context pointer handed to the callback. */
    gclk_exploration_result_cb_mode_t cb_mode; /**< Mode that defines which results the cb shall be called for. */
} gclk_exploration_result_cb_conf_t;

/**
 * @brief Tuple that maps a clock config compare function to a human readable name.
 */
typedef struct {
    gclk_cmp_func_t func; /**< The function that compares two clock configurations. */
    const char *name; /**< The human readable name of the compare function. */
} topology_cmp_func_names_t;

/**
 * @brief Context for a topo conf cmp function which ties to fit a list of frequencies.
 *
 * The context data is used by the @ref gclk_manager_cmp_lowest_freq_list_abs_err compare function
 * which tries to fit a list of frequencies as good as possible when only a single specific
 * scaler can be adjusted.
 **/
typedef struct {
    const uint32_t *freqs; /**< the target frequency values to aim for */
    size_t target_freqs_cnt; /**< the number of target frequencies in @ref freqs that are aimed for
                                  (which may not be possible or applicable). */
    size_t match_freqs_cnt; /**< number of frequencies that shall be matched, which may be lower than
                                 @ref target_freqs_cnt if the scaled clock has less options than the
                                 number of 'wished for' target frequencies. */
    uint32_t lowest_err; /**< lowest error of the combined absolute frequency found so far. */
    const gclk_t *scale_clk; /**< the single clock scaler that will be used for DFS adaptations. */
} lflae_cmp_fun_ctx_t;


/**
 * @brief Context used by the @ref gclk_manager_cmp_single_scaler_range_limited compare function.
 */
typedef struct {
    const gclk_t *scale_clk;      /**< Single clock scaler that will be used for DFS adaptations */
    unsigned scale_clk_topo_idx;  /**< Topology entry index (specific to scale_clock memeber).
                                       Note: this must be valid for all calls of the compare function.
                                             I.e. this compare function is not suitable for comparing different
                                             topologies (but different factor configs of the same topology). */
    uint32_t target_freq;               /**< The target frequency of the last clock in the topology. This usually
                                             refers to the core clock. */
    gclk_freq_limit_t scaler_fo_limits; /**< Absolute limits for the output freq. at the scaled clock. */
    uint32_t scaler_factor_target; /**< One specific factor of the scaled clock instance that defines the subset
                                        of configurations that will be evaluated in more detail. Used to skip the
                                        more complex comparison step for all other factors (as every comparison
                                        step always considers every possible factor anyway). */
    gclk_factor_limit_t scaler_factor_limits; /**< Factor limits for the scaled clock instance such that every
                                                   config that is out of these limits could already be ruled out
                                                   as being invalid. */
    uint32_t min_error; /**< Cached value of the minimum target frequency error found during previous comparisons.
                             Should be initialized to the highest value possible before starting the exploration. */
    uint32_t min_infeasible_cnt; /**< Cached count of infeasible scale factors of previous comparisons.
                                      A lower value means more valid frequency steps were found to be achievable
                                      at the respective configuration found. */
} range_limit_cmp_fun_ctx_t;


/**
 * @brief Topology configuration compare function for explicit frequency list fitting.
 *
 * A compare function that evaluates how well a configuration is suitable for DFS when using a single scaler
 * for the frequency adaptation. This function is applicable if an explicit list of given target frequencies
 * shall be fitted as good as possible. Simply put, this tries to minimize sum(abs(ft_i - f_i))), where
 * tf_i is the target frequency and f_i is the closest possible frequency, i being an element of
 * {0 ... *num of target freqs*}.
 *
 * @see @ref gclk_cmp_func_t for the general compare function interface.
 * @note @p ctx **MUST** point to a properly initialized @ref lflae_cmp_fun_ctx_t variable.
 */
gclk_cmp_result_t gclk_manager_cmp_lowest_freq_list_abs_err(clk_topology_entry_t *topo_best, size_t len1,
                                                            clk_topology_entry_t *topo_cmp, size_t len2,
                                                            void *ctx);

/**
 * @brief Topology configuration compare function maximizing applicable frequency steps.
 * Compares configurations on how well they are suitable for DFS when using a single scaler for the
 * frequency adaptation. Opposed to @ref gclk_manager_cmp_lowest_freq_list_abs_err this performs
 * a more complex check on the applicability of all resulting frequency steps to ensure configurations
 * are not only fitting the factors well but also result in a greater number of feasible settings.
 * A pointer to a properly initialized @ref range_limit_cmp_fun_ctx_t struct must be given as context.
 * This function is applicable if no particular list of target frequencies is given as target
 * but instead the goal is to exploit the available range of the single used scaler as good as possible.
 * Configurations are compared regarding the following aspects (in descending priority):
 * - Configs matching the single given target frequency closer are better (with the initial factor)
 * - Configs enabling a greater number of frequency steps are better
 * - Configs that result in lower power consumption are better.
 *
 * @see @ref gclk_cmp_func_t for the general compare function interface.
 * @note @p ctx **MUST** point to a properly initialized @ref range_limit_cmp_fun_ctx_t variable.
 */
gclk_cmp_result_t gclk_manager_cmp_single_scaler_range_limited(clk_topology_entry_t *topo_best, size_t len1,
                                                               clk_topology_entry_t *topo_cmp, size_t len2,
                                                               void *ctx);

/**
 * @brief Compare function only accepting exact frequency matches, preferring less power.
 *
 * All configuration variants which are not able to produce exactly the given target frequency at the
 * topology output (leaf node of the topology), are treated as invalid. Out of all options that result
 * in the exact target frequency the one with the lowest projected power consumption is best.
 * The consumption is calculated with the clock power model that is parameterized with
 * platform-specific properties for relevant clock nodes.
 *
 * @see @ref gclk_cmp_func_t for the general compare function interface.
 * @note @p ctx **MUST** hold the target frequency as value.
 */
gclk_cmp_result_t gclk_manager_cmp_topology_exact_leaf_freq_pmin(clk_topology_entry_t *topo_best, size_t len1,
                                                                clk_topology_entry_t *topo_cmp, size_t len2,
                                                                void *ctx);

/**
 * @brief Compare function preferring closer frequency matches and less power.
 *
 * Same as @ref gclk_manager_cmp_topology_exact_leaf_freq_pmin but does also accept inexact
 * frequency matches at the output clock. First priority of the comparison is to match the
 * frequency as close as possible, second priority is minimizing the power consumption
 * in case there are mutltiple configurations that are equally close to the frequency target.
 *
 * @see @ref gclk_cmp_func_t for the general compare function interface.
 * @note @p ctx **MUST** hold the target frequency as value.
 */
gclk_cmp_result_t gclk_manager_cmp_topology_closest_leaf_freq_pmin(clk_topology_entry_t *topo_best, size_t len1,
                                                                   clk_topology_entry_t *topo_cmp, size_t len2,
                                                                   void *ctx);

/**
 * @brief Data that maps topology configuration compare functions to unique strings.
 *
 * Usable by shell handlers or debug utils to refer to compare functions by
 * human-readable names.
 */
static const topology_cmp_func_names_t topology_cmp_funcs[] = {
    { .func = gclk_cmp_topology_for_closest_leaf_freq,         .name = "closest_leaf" },
    { .func = gclk_cmp_topology_for_closest_leaf_freq_min_sum, .name = "min_sum" },
    { .func = gclk_cmp_topology_for_closest_leaf_freq_max_sum, .name = "max_sum" },
    { .func = gclk_cmp_topology_for_closest_leaf_freq_min_max, .name = "min_max" },
    { .func = gclk_cmp_topology_for_closest_leaf_freq_max_max, .name = "max_max" },
    { .func = gclk_cmp_topology_for_exact_leaf_freq,           .name = "exact_leaf" },
    { .func = gclk_manager_cmp_topology_exact_leaf_freq_pmin,  .name = "fexact_pmin" },
};

/**
 * @brief Tuple that maps a factor match function to a human readable name.
 */
typedef struct {
    gclk_factor_match_func_t func; /**< the factor match function. */
    const char *name; /**< human readable name of the match function. */
} factor_match_func_names_t;

/**
 * @brief Data that maps factor matching functions to unique strings.
 *
 * Usable by shell handlers or debug utils to refer to match functions by
 * human-readable names.
 */
static const factor_match_func_names_t factor_match_funcs[] = {
    { .func = gclk_match_iter_mul_recurse_div,   .name = "iter_mul_recurse_div" },
    { .func = gclk_match_iter_mul_factorize_div, .name = "iter_mul_factorize_div" },
    { .func = gclk_match_exact_full_iter,        .name = "exact_full_iter" },
    { .func = gclk_match_closest_full_iter,      .name = "closest_full_iter" },
};

/**
 * @brief     Initializes the clock manager.
 *
 * @retval    0    on success.
 * @retval   <0    on error.
 */
int gclk_manager_init(void);

/**
 * @brief Executes a sequence of clock operations.
 *
 * @param[in]   steps      List of sequence steps to execute.
 * @param[in]   step_cnt   Number of elements @p steps points to.
 */
void gclk_manager_run_sequence(gclk_manager_sequence_step_t *steps, size_t step_cnt);

/**
 * @brief Checks if a topology config breaks given constraints.
 *
 * @param[in] constraints    Constraints to be checked.
 * @param[in] constr_cnt     number of elements @p constraints points to.
 * @param[in] topo           clock topology entries describing the checked topology.
 * @param[in] topo_len       number of clock instances in @p topo.
 *
 * @return    The first unfulfilled constraint of @p topo if not all are fulfilled.
 *            NULL if all constraints are fulfilled.
 */
const gclk_freq_constraint_t* gclk_manager_conf_breaks_constraint(const gclk_freq_constraint_t *constraints, unsigned constr_cnt, clk_topology_entry_t *topo, uint32_t topo_len);

/**
 * @brief Get an equivalent fraction that combines all involved scaling factors.
 *
 * @param[in]     topo       The topology all scaling factors will be combined of.
 * @param[in]     topo_len   Length of @p topo.
 * @param[in,out] f          Pointer to where the equivalent factor will be stored.
 */
void gclk_manager_get_combined_topology_fraction(clk_topology_entry_t *topo, size_t topo_len, gclk_fraction_t *f);

/**
 * @brief Notify multiple clocks of a topology about a configuration change.
 *
 * @param[in]     old_topo       The topology config before the change.
 * @param[in]     old_topo_len   Number of elements in @p old_topo.
 * @param[in]     new_topo       The topology config after the change.
 * @param[in]     new_topo_len   Number of elements in @p new_topo.
 * @param[in]     post_change    true if the change was already performed.
 *                               false if the change is about to happen.
 */
void gclk_manager_notify_multi_clk_change(clk_topology_entry_t *old_topo, size_t old_topo_len,
                                          clk_topology_entry_t *new_topo, size_t new_topo_len,
                                          bool post_change);

/**
 * @brief Runs a sequence of reconfiguration steps and notify about changes.
 *
 * @param[in]     seq         The sequence to execute.
 * @param[in]     seq_len     Number of elements in @p seq.
 * @param[in]     print_only  true if the effects of this call shall only be printed.
 *                            false if the sequence shall be executed.
 */
void gclk_manager_run_sequence_with_notify(gclk_manager_sequence_step_t *seq, size_t seq_len, bool print_only);

/**
 * @brief Register a callback for notification of a clock frequency change.
 *
 * @param[in]      clk  Clock to register a notification for.
 * @param[in,out]  nle  Storage that will hold the registration data.
 * @param[in]      cb   Callback that will be executed before and after the frequency of clk is changed
 */
void gclk_manager_register_clk_change_cb(const gclk_t *clk, gclk_clock_change_notify_list_t *nle,
                                         clock_change_cb_t cb);

/**
 * @brief Unregister a callback for notification of a clock frequency change.
 *
 * @param[in]  nle  The previously registered notification entry.
 */
void gclk_manager_unregister_clk_change_cb(gclk_clock_change_notify_list_t *nle);

/**
 * @brief Notify a changed configuration of a single clock.
 *
 * @param[in]  clk          The changed clock.
 * @param[in]  f_old        The old frequency of @p clk.
 * @param[in]  f_new        The new frequency of @p clk.
 * @param[in]  post_change  true if the change was already performed.
 *                          false if the change is about to happen.
 */
void gclk_manager_notify_clk_change(const gclk_t *clk, uint32_t f_old, uint32_t f_new, bool post_change);

/**
 * @brief Get the frequency configuration limit that applies to a clock at a given frequency.
 *
 * @param[in]  clk          The clock a limit is requested for.
 * @param[in]  freq         The frequency of @p clk to get the limit for.
 * @param[in]  optimize_ws  true if faster flash shall be preferred.
 *                          false if lower voltage shall be preferred.
 *
 * @return  The limit specifier marking the best applicable limit at @p freq.
 */
const freq_conf_limit_t *gclk_manager_get_freq_conf_limit(const gclk_t *clk, uint32_t freq, bool optimize_ws);

/**
 * @brief Enable/disable automatic switching of voltage range on clock changes
 *
 * @param[in]  on  true for enable.
 *                 false for disable.
 *
 * @note When **enabled**, the manager will automatically adjust the voltage according to hardware
 *       constraints whenever the frequency is changed via an automatic frequency adaptation
 *       function that issues notification calbacks. For lower-level reconfiguration functions
 *       (i.e., the ones that operate directly on individual clock instances), this is not
 *       guaranteed, requiring the caller to take care to ensure applicability of the setting.
 *       **Disabling** this will immediately set the voltage to the highest possible value in
 *       order to maintain stable operation for all possible frequency and flash settings.
 *
 * TODO: Even though the capabilities on DVS, DFS, WSA and the related policy (@ref can be controlled independently
 *       (because that is nice for detailed evaluation purposes), in reality, this is most likely not how these parameters
 *       are expected to work. The policy should actually only matter if both DVS *and* WSA are enabled because
 *       disabling one of them removes interdependencies to the other. I.e. there is no reason for the policy to favour
 *       low voltage if DVS is disabled anyway (always optimizing for flash access speed would be preferrable in that case).
 *       One exception to this might be scenarios where the decision whether to prefer DVS/WSA would change depending on other aspects.
 *       I.e. a fast rate of voltage changes may not be wanted due to its time overhead.
 */
void gclk_manager_enable_voltage_auto_scale(bool on);

/**
 * @brief Enable/disable automatic adaptation of flash waitstates on clock changes.
 *
 * Similar to @ref gclk_manager_enable_voltage_auto_scale() but for automatic flash
 * wait-state adaptation.
 *
 * @param[in]  on  true for enable.
 *                 false for disable.
 *
 * @note When **enabled**, the manager will automatically adjust the wait-states to hardware
 *       constraints whenever the frequency is changed via an automatic frequency adaptation
 *       function that issues notification calbacks. For lower-level reconfiguration functions
 *       (i.e., the ones that operate directly on individual clock instances), this is not
 *       guaranteed, requiring the caller to take care to ensure applicability of the setting.
 *       **Disabling** this will immediately set the highest possible value in order to
 *       maintain stable operation for all possible frequency and voltage settings.
 */
void gclk_manager_enable_flashws_auto_update(bool on);

/**
 * @brief Enable/disable automatic adaptation of the core frequency.
 *
 * @param[in]  enable   true for enable.
 *                      false for disable.
 *
 * @note When **enabled**, the manger will automatically adapt the core frequency to task demands.
 *       This is triggered via a scheduler hook (see @ref gclk_manager_pre_sched_hook()).
 *       On the enable call the manager will store the currently active frequency, which
 *       will be restored once it is disabled again. Other manager functions to adjust the
 *       frequency/topology shall *not* be used for manual reconfigurations while this feature is
 *       active.
 *       When **disabled** the manager will not perform any core clock adjustments on it own and
 *       manual adjustments via manager functions are allowed again.
 */
void gclk_manager_enable_dynamic_frequency_scaling(bool enable);

/**
 * @brief Enable/disable automatic assessment of thread specific PU-metric.
 *
 * Controls whether instrumentation and data collection for the performance utilization (PU)
 * assessment is active. If enabled, the respective scheduler hooks are used to collect
 * metadata on the threads scheduling behavior. See also @ref gclk_manager_pre_sched_hook()
 * and @ref gclk_manager_post_sched_hook().
 *
 * @param[in]  enable   true for enable.
 *                      false for disable.
 */
void gclk_manager_enable_pu_assessment(bool enable);

/**
 * @brief Get currently active DVS adaptation policy.
 *
 * @return  The DVS policy.
 */
gclk_manager_dvs_policy_t gclk_manager_get_dvs_policy(void);

/**
 * @brief Set DVS adaptation policy.
 *
 * @param policy  Policy that defines which optimization to prefer.
 */
void gclk_manager_set_dvs_policy(gclk_manager_dvs_policy_t policy);

/**
 * @brief Print thread/CPU utilization metrics.
 * @note For debug/testing purposes.
 */
void gclk_manager_print_util_metrics(void);

/**
 * @brief Print formatted topology configuration data.
 *
 * Outpus a human readable string that represents a topology config.
 *
 * @param[in] topology  The topology entries to print. Expected to represent a single topology chain
 *                      where topology[0] points to the leaf and topology[size-1] points to the source.
 * @param[in] size      Number of elements in @p topology.
 * @param[in] min_max   Flag on whether the min and max frequency should be output for each clock node.
 *                      The values will represent the min/max frequencey possible at each clock assuming
 *                      the configuration up the tree is fixed and the scaling factor of this one clock
 *                      would be set to its min/max scaling factors. Indicated values do not consider
 *                      active constraints and therfore do not guarantee the configuration is actually
 *                      applicable at the moment. I.e., treat those values as *theoretically possible
 *                      based on the scaling factor range*.
 * @param[in] factors   Similar to @p min_max, a flag on whether the possible factors of each clock
 *                      should be printed for each clock node. Same logic regarding applicability
 *                      guarantees applies.
 */
void gclk_manager_print_topology_conf(clk_topology_entry_t *topology, uint32_t size, bool min_max, bool factors);

/**
 * @brief Clear all performance util and scheduler stats data.
 *
 * This resets all threads performance metrics collected at scheduling events.
 * Useful for starting a clean performance assessment phase.
 */
void gclk_manager_clear_performance_util_data(void);

/**
 * @brief Calculate performance utilization factor for a thread.
 *
 * @param[in]  thread_id    The unique ID of the thread to calculate the PU for (PID).
 * @param[in]  debug_print  Flag enabling verbose statistics/metadata output for testing.
 *
 * @return The PU factor calculated for the given thread.
 */
int gclk_manager_calculate_pu_factor(uint32_t thread_id, bool debug_print);

/**
 * @brief set parameters that control PU-based DVFS
 *
 * @param[in] fboost           The core frequency to be used for a thread if its PU is at least @p fboost_pu_th.
 * @param[in] fthrottle        The core frequency to be used for a thread if its PU is at most @p fthrottle_pu_th.
 * @param[in] fboost_pu_th     The minimum PU value a thread must have to setup @p fboost before scheduling it.
 * @param[in] fthrottle_pu_th  The max PU value a thread must have to setup @p fthrottle before scheduling it.
 */
void gclk_manager_set_dvfs_pu_params(uint32_t fboost, uint32_t fthrottle, int fboost_pu_th, int fthrottle_pu_th);

/**
 * @brief Run thread to cycle through different core frequencies.
 *
 * The call will block till the cycling is complete. The frequency cyler will use the frequency values currently
 * set up for DFS. Those may be altered either dynamically via @ref gclk_mananger_set_dfs_frequencies() or
 * @ref gclk_mananger_set_default_dfs_frequencies() or statically via the scale_settings provided by hardware
 * configuration (see @ref gclk_manager_get_scale_settings()).
 *
 * @param[in]  cycle_us       The min cycle time each frequency should be used for in us.
 * @param[in]  min_schedules  The min number of schedules each thread should be observed for.
 */
void gclk_manager_start_freq_cycler(unsigned int cycle_us, uint32_t min_schedules);

/**
 * @brief Set (scalable) clock to the given frequency.
 *
 * This is a high level set frequency function that calls the lower layer @ref gclk_set_freq()
 * function but additionally issues clock change notifications before and after the change.
 *
 * @pre @p freq must be guaranteed to be valid and currently applicable for @p clk.
 *
 * @param[in]  clk   The (scalable) clock that shall be set to the given frequency.
 * @param[in]  freq  The new frequency in Hz.
 *
 * @return   true   If @p clk was successfully set to @p freq Hz.
 *           false  If the frequency was not set up properly.
 */
bool gclk_manager_set_freq(const gclk_t *clk, uint32_t freq);

/**
 * @brief Set (scalable) clock to the given scaling factor.
 *
 * This is a high level set factor function that calls the lower layer @ref gclk_set_factor()
 * function but additionally issues clock change notifications before and after the change.
 *
 * @pre @p factor must be guaranteed to be valid and currently applicable for @p clk.
 *
 * @param[in]  clk     The (scalable) clock that shall be set to the given scaling factor.
 * @param[in]  factor  The new scaling factor.
 *
 * @return   true   If @p clk was successfully set to a scaling factor of @p factor.
 *           false  If the factor was not set up properly.
 */
bool gclk_manager_set_factor(const gclk_t *clk, uint32_t factor);

/**
 * @brief Set the core clock to the given frequency using the currently active scale setting.
 *
 * This is a high level frequency/topology adaptation function that also issues clock change
 * notifications. Depending on the active scale setting the effects of this function will differ:
 * - It may update a single scaling factor of one clock instance.
 * - It may update multiple scaling factors within the core topology.
 * - It may intermediately or permanently alter the core topology.
 *
 * @pre @p freq must be guaranteed to be valid and currently applicable. Foremost, it must be
 *      a frequency that was preconfigured for DFS use. I.e., it must be a value obtained from
 *      @ref gclk_manager_get_dfs_freqs().
 *
 * @param[in]  freq  The new core frequency in Hz.
 *
 * @return   true   If the scaling operation was successful.
 *           false  If the scaling operation failed.
 */
bool gclk_manager_scale_core_freq(uint32_t freq);

/**
 * @brief Set the topology of the given clock from the currently active setting to a new topology.
 *
 * Changes the topology (a.k.a. clock path or clock routing) to a different topology.
 * This is a high level topology adaptation function that also issues clock change notifications.
 *
 * @param[in] clk              The clock instance that will be switched to another topology.
 * @param[in] target_topology  The zero based id of the topology that the clock is switched to.
 * @param[in] target_freq      The frequency that is aimed for with the new topology,
 *                             GCLK_INVALID_FREQ if current frequency should be used.
 * @param[in] cmp_func         The compare function that will be used to find the most suitable
 *                             topology config.
 *
 * @return   The new frequency of clk if the switch was successful.
 *           GCLK_INVALID_FREQ if the switch failed.
 */
uint32_t gclk_manager_switch_topology(const gclk_t *clk, int target_topology, uint32_t target_freq, gclk_cmp_func_t cmp_func);

/**
 * @brief Temporarily block clock adjustments.
 */
void gclk_manager_block(void);

/**
 * @brief Unblock clock adjustments again.
 */
void gclk_manager_unblock(void);

/**
 * @brief Prepare configs for D(V)FS operation with given frequencies (as close as possible).
 *
 * Sets up a list of frequencies applicable for D(V)FS operation.
 * A scale setting must be active for this to work (see @ref gclk_manager_get_scale_settings()
 * and @ref gclk_mananger_set_active_scale_setting()). Resulting frequencies depend on the
 * capabilities of the selected scale setting and the current topology/frequency configuration.
 * The frequency values handed to this function are subject to a matching procedure which can not
 * guarantee all frequencies can be obtained exactly as specified (due to hardware limits).
 * As applicability and effectivity of different frequency configurations for some scale settings
 * may strongly depend on the currently active topology/frequency configuration, it is recommended
 * to use this function after setting up a config that is well suited for D(V)FS. The
 * recommended way to do this is by calling @ref gclk_manager_setup_default_dfs_topo_conf() before.
 * Calling this function with other configs may result in fewer, less flexible, or no frequency options
 * being explored.
 *
 * @param[in]  freqs  Pointer to an array of frequencies to be set up for DFS operation.
 * @param[in]  cnt    Number of elements in @p freqs.
 *
 * @return  The number of distinct frequencies that were prepared on success.
 *          <0 on error.
 */
int gclk_mananger_set_dfs_frequencies(const uint32_t *freqs, size_t cnt);

/**
 * @brief Prepare configs for D(V)FS operation with default frequencies.
 *
 * Same as @ref gclk_mananger_set_dfs_frequencies() but using default frequencies.
 * If provided, this uses frequencies defined by platform-specific configuration. If the platform
 * defines no default values explicitly, it tries to automatically come up with reasonabe settings
 * based on available configuration options and the active scale setting.
 *
 * @return  The number of distinct frequencies that were prepared on success.
 *          <0 on error.
 */
int gclk_mananger_set_default_dfs_frequencies(void);

/**
 * @brief Return the clock handle that represents the CPU core clock.
 *
 * @return The core clock handle.
 */
const gclk_t* gclk_manager_get_core_clock_handle(void);

/**
 * @brief Disable clocks that are currently not being used by other active clocks.
 *
 * Currently this does not consider 'invisible' dependencies. I.e. if an intermediate clock
 * (one that is not a leaf) is indeeed needed for operations even if the clock is not used
 * by other clock instances. One way to handle this is to integrate simple resource allocation
 * to respective peripherl drivers or other code that e.g. needs clocks for specific bus access.
 */
void gclk_manager_disable_unused(void);

/**
 * @brief Get clock freqs used for DFS and PUA frequency cycle.
 *
 * @param[in,out]  freqs  Location where to store the pointer to the frequency values array.
 *
 * @return  Number of elements contained in the frequency array.
 */
unsigned int gclk_manager_get_dfs_freqs(uint32_t **freqs);

/**
 * @brief Get available scale settings that define how to perform clock scaling.
 *
 * @param[in,out]  s  Location where to store the pointer to the scale settings array.
 *
 * @return  Number of scale setting entries pointed to by @p s.
 */
int gclk_manager_get_scale_settings(const gclk_scale_setting_t **s);

/**
 * @brief Get the currently active scale setting.
 *
 * @return     A pointer to the currently active scale setting.
 *             NULL if no scale setting applies to the current clock configuration.
 */
const gclk_scale_setting_t* gclk_mananger_get_active_scale_setting(void);

/**
 * @brief Set the active scale setting to the given index if applicable.
 *
 * @see @ref gclk_manager_get_scale_settings() to get available scale settings.
 *
 * @param[in]  i  Index of the scale setting that will be set active.
 *
 * @return     true if applied.
 *             false if not appliccable.
 */
bool gclk_mananger_set_active_scale_setting(unsigned i);

/**
 * @brief Set request flag for PU stat collection for a given thread.
 *
 * Instructs the PUA instrumentation to collect the required data for calculating
 * the PU metric for the given thread.
 *
 * @see @ref gclk_manager_start_freq_cycler() for settings on metadata collection.
 *
 * @param tid  The PID of the thread PU data will be collected for.
 */
void gclk_manager_enable_pu_stat_request_for_thread(kernel_pid_t tid);

/**
 * @brief Get a list of clock sources which may be used to drive the core clock.
 *
 * @param[in,out]  clks  Location to store the clock pointer array at.
 *
 * @return Number of clocks in @p clks.
 */
int gclk_manager_get_allowed_core_clock_sources(const gclk_t ***clks);

/**
 * @brief Execute a single clock reconfiguration sequence step.
 *
 * This executes only the low level operation defined in the step without
 * issuing any clock change notifications.
 *
 * @param[in]  step  The step to execute.
 */
void gclk_manager_execute_sequence_step(gclk_manager_sequence_step_t *step);

/**
 * @brief Derive a reconfiguration sequence to switch from one topology config to another.
 *
 * Depending on the current configuration and hardware constraints the complexity of a
 * derived sequence may differ a lot.
 *
 * @see core_clk_topo_switch_descs defined in the manager configuration of your platform
 *      for details on predefined topology switch options.
 *
 * @param[in]      src_topo       The topology config to start from.
 * @param[in]      src_len        Number of clocks in @p src_topo.
 * @param[in]      target_topo    The topology config that shall be set up.
 * @param[in]      target_len     Number of clocks in @p target_topo.
 * @param[in,out]  out_seq        Location where to store the derived sequence steps.
 * @param[in]      max_seq_steps  Number of elements that can be stored in @p out_seq.
 *
 * @return  Length of derived sequence on success.
 *          <0 on error.
 */
int gclk_manager_derive_sequence(const clk_topology_entry_t *src_topo, uint32_t src_len,
                                 const clk_topology_entry_t *target_topo, uint32_t target_len,
                                 gclk_manager_sequence_step_t *out_seq, unsigned max_seq_steps);

/**
 * @brief Get min required flash waitstates and core voltage required for the given tree conf.
 *
 * Considers all gclk_freq_conf_limits as defined by the platform specific configuration data
 * via the manager config.
 *
 * @param[in]  tree_conf   Tree configuration as a list of arbitrarily sorted clock nodes.
 * @param[in]  tree_size   Number of clocks in the given tree i.e., the length.
 * @param[out] min_ws      Variable that will be set to the minimum required flash wait states.
 * @param[out] min_vc_idx  Variable that will be set to the minimum required core voltage index.
 * @param[in]  dvspolicy   The policy that determines how mutually exlusive optimizations on wait
 *                         states and core voltage are resolved. In cases where only either voltage
 *                         can be reduced or flash access can be sped up, this defines which of
 *                         both options will be prefered.
 */
void gclk_get_min_required_ws_vc_from_tree_config(clk_topology_entry_t *tree_conf, size_t tree_size, unsigned *min_ws, unsigned *min_vc_idx,
                                                  gclk_manager_dvs_policy_t dvspolicy);

/**
 * @brief Applies a reconfiguration step on a tree configuration model.
 *
 * Modifies a tree model by applying a reconfiguration step on it. The tree model will be modified to
 * reflect the new state as if @p step would have been applied to the hardware. This can be used
 * to check the impact of a configuration change (including the effects on other clocks) before
 * actually applying it to the hardware configuration.
 *
 * @param[in]  step        The reconfiguration step to apply.
 * @param[in]  tree_conf   Tree configuration as a list of arbitrarily sorted clock nodes.
 * @param[in]  tree_size   Number of clocks in @p tree_conf.
 */
void gclk_manager_simulate_seq_step_on_tree_conf(gclk_manager_sequence_step_t *step, clk_topology_entry_t *tree_conf, size_t tree_size);

/**
 * @brief Bruteforce the best frequency configuration based on a given compare function.
 *
 * This brite force algorithm explores every topology that is able to drive clk.
 * For each topology it bruteforce-tests all possible frequency-configurations of involved
 * intermediate clock nodes. All possible parents, frequencies and constraints are considered,
 * if not indicated otherwise via below parameters.
 *
 * @note Brute force is obviously not optimal and can take quite some time with more complex
 *       clock trees as configuration possibilities can quickly expand beyond the order of 10k
 *       or even 100k variants. However, the major benefit of brute forcing comes from the fact
 *       that no assumptions need to be made about internals of the clock tree and its nodes
 *       internals. Moreover, it is easy to guarantee that all possible configurations are covered,
 *       and resulsts of this rather complex operation may easily be cached for later use.
 *       Refer to factor matching approaches (i.e., the gclk_factor_match_func_t interface) for
 *       an alternative way to explore configurations (e.g., gclk_match_iter_mul_factorize_div()).
 *       For future improvements a more generic implementation for ruling out specific configs
 *       a priori together with a more sophisticated branch-and-bound-style iteration may
 *       further improve this.
 *
 *  @todo An optimization could first check if clk (and other intermediate clocks) have output
 *        constraints to limit possible configurations. A variant of that is already implemented
 *        in one of the more complex compare functions,
 *        see @ref gclk_manager_cmp_single_scaler_range_limited
 *
 *  @todo there is also potential for optimizing the resulting topology properties, e.g.:
 *        - reduce domain counts (only enable the least required domains)
 *        - reduce the frequency of higher order clocks
 *          (prefer nodes to use a lower frequency if the clock can be multiplied further down the tree)
 *  @todo refactor this to a more generic implementation that judges the derived topology configurations via a generic
 *        function pointer (i.e. a compare function that can be handed by the above layer)
 *
 * @param[in]  clk                 The clock instance a topology config is searched for.
 * @param[out] best_topology       Pointer to where the topology config result will be stored.
 * @param[in,out] topo_len         in: max length of @p best_topology.
 *                                 out: the actual length of the best config found.
 * @param[in,out] topo_idx         in: GCLK_UNDEFINED_TOPOLOGY if any topology is fine,
 *                                     A specific (0 based) topology index to be used for the target config,
 *                                 out: the topology idx that was chosen.
 * @param[in]  cmp_func            compare function that is used to diceide which topology config is best.
 * @param[in]  cmp_func_ctx        optional context variable handed to each call of cmp_func.
 * @param[out] ret_n_valid         If @p force_nth < 0 this will indicate the number of configurations that are
 *                                 considered valid by the given compare function. Set to NULL for don't care.
 * @param[in]  force_nth           For when multiple configurations are valid / applicable this forces the function
 *                                 to return the nth valid config (0 based).
 * @param[in]  conf_found_cb_cfg   Config struct for a callback that will be executed per found config if the
 *                                 specified callback condition is fulfilled. NULL if not needed.
 *
 * @return   The matched leaf frequency on success.
 *           GCLK_INVALID_FREQ on error.
 */
uint32_t gclk_manager_brute_force_freq_conf(const gclk_t *clk, clk_topology_entry_t *best_topology, uint32_t *topo_len,
                                            int *topo_idx, gclk_cmp_func_t cmp_func, void *cmp_func_ctx, size_t *ret_n_valid, int force_nth,
                                            gclk_exploration_result_cb_conf_t *conf_found_cb_cfg);

/**
 * @brief Sets up a topology configuration that 'works well' with the given scale setting.
 *
 * Shall be called with the currently active scale setting and will affect the core clock
 * topology configuration. The initial configuration that will be set up aims for the highest
 * allowed DFS frequency.
 *
 * NOTE: the used scaling approach directly affects if there are constraints on how the initial
 * topology and frequency config must be set up.
 * (1) If the scaling approach involves adjusting multiple clocks (as is the case for
 * the SCALE_INTERMEDIATE_TOPO_AUTO), the initial configuration is subject to less constraints
 * becasue each scaling step can set multiple involved clocks to another config. This variant
 * allows full reconfiguration of the topology config (i.e. multiple scaler instances). It
 * therfore must only ensure each individual config to be valid on its own.
 * (2) A scaling approach meant to change as few settings as possible (e.g., with
 * SCALE_DIRECT or SCALE_UPTREE approach) the initial configuration of the topology significantly
 * impacts properties and applicablility of different frequency steps. In this case, the
 * preliminary config setup must be evaluated in more detail because DFS adjustents will only
 * touch a single scaler instance. The fixed part of the config must therfore apply to *all* steps.
 * Fixing a part of the topology to a static config like that limits the applicability of frequency
 * steps more severely. It therefore prioritzes maximizing the number of frequency options to ensure
 * adjusting the single scaler still gives enough range for DFS adjustment. Lower power configuration
 * variants are still preferred but this is given less priority than more DFS frequency options.
 *
 * @param[in] scs    scale setting structure describing how DFS shall be performed
 *
 * @return   true on success.
 *           false on error.
 */
bool gclk_manager_setup_default_dfs_topo_conf(const gclk_scale_setting_t *scs);

/**
 * @brief Print reconfiguration sequence.
 *
 * Outputs a given reconfiguration sequence with parameters as human readable string.
 *
 * @param[in]  seq    The sequence to print.
 * @param[in]  len    Number of steps in @p seq.
 */
void gclk_manager_print_step_sequence(gclk_manager_sequence_step_t *seq, size_t len);


/**
 * @brief A default clock change notification callback for stdio/UART.
 *
 * Used to prepare/restore the RIOT stdio/UART subsystem on relevant clock changes.
 * For now this function only uses abstract API calls of RIOT which is expected to
 * apply to many target platforms as is. Future ports may need more sophisticated
 * handling, therfore the actual registration of those callbacks is controlled by
 * the platform- specific init hook of the manager @see gclk_manager_platform_init().
 *
 * @param[in] altered_clk     The clock that was modified causing the callback to trigger.
 * @param[in] affected_clk    The affected clock this callback was registered for.
 * @param[in] f_old           The ololdrequency of the modified clock.
 * @param[in] f_new           The new frequency of the modified clock.
 * @param[in] post_change     True if the call indicates the change is about to happen.
 *                            False if the change was already executed.
 */
void gclk_manager_default_stdio_reinit_cb(const gclk_t* altered_clk, const gclk_t* affected_clk, uint32_t f_old, uint32_t f_new, bool post_change);

/**
 * @brief A default clock change notification callback for the timer subsystem.
 *
 * Used to prepare/restore the RIOT timer subsystem on relevant clock changes.
 * For now this function uses abstract API calls of RIOT together with a custom
 * extension for writing the timer. This should apply to many target platforms,
 * as long as the timer extension is provided. Future ports may need more
 * sophisticated handling, therfore the actual registration of those callbacks
 * is controlled by the platform-specific init hook of the manager
 * @see gclk_manager_platform_init().
 *
 * @param[in] altered_clk     The clock that was modified causing the callback to trigger.
 * @param[in] affected_clk    The affected clock this callback was registered for.
 * @param[in] f_old           The ololdrequency of the modified clock.
 * @param[in] f_new           The new frequency of the modified clock.
 * @param[in] post_change     True if the call indicates the change is about to happen.
 *                            False if the change was already executed.
 */
void gclk_manager_default_timer_reinit_cb(const gclk_t* altered_clk, const gclk_t* affected_clk, uint32_t f_old, uint32_t f_new, bool post_change);

#ifdef __cplusplus
}
#endif

#endif /* GCLK_MANAGER_H */
/**
 * @}
 */
