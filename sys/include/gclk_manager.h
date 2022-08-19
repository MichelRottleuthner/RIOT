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
 * @author      Michel Rottleuthner <michel.rottleuthner@haw-hamburg.de>
 */
#ifndef GCLK_MANAGER_H
#define GCLK_MANAGER_H

#include "gclk.h"
#include "list.h"

#ifdef __cplusplus
extern "C" {
#endif

/* @brief Max number of tasks to allocate PU-stats memory for. */
#ifndef GCLK_MANAGER_PU_STATS_TASK_NUM
#define GCLK_MANAGER_PU_STATS_TASK_NUM 10
#endif

/* @brief max number of prepared clock configs to allocate memory for.
 * By default allocate enough space to store one prepared config per DFS frequency step. */
#ifndef GCLK_MANAGER_PREP_CONFS_MAX_NUMOF
#define GCLK_MANAGER_PREP_CONFS_MAX_NUMOF (MAX_DFS_FREQ_VALUES_NUM)
#endif

/* @brief max number of clocks in a topology conf.
 * There can never be more clocks in any topology than there are clocks.
 * NOTE: this is a very pessimistic estimation, as in practice the number of clocks
 *       which are part of the core clock (sub-)topology is by far lower than this.
 * TODO: add testing code to automatically determine a more realistic platform-specific
 *       upper bound for this by exploring the maximum length of any possible
 *       core (sub-)topology. */
#ifndef GCLK_MANAGER_PREP_CONFS_TOPO_MAX_LEN
#define GCLK_MANAGER_PREP_CONFS_TOPO_MAX_LEN (GCLK_NUM_OF_CLOCKS)
#endif

/* @brief max number of steps in a prepared reconfiguration sequence.
 * By default we assume a reconfiguration sequence will touch each clock once at most.
 * NOTE: this is a very pessimistic estimation, as in practice reconfiguration sequences
 *       involve far less operations.
 * TODO: add testing code to automatically determine a more realistic platform-specific
 *       upper bound for this by exploring a representative set of possible
 *       reconfiguration sequences. */
#ifndef GCLK_MANAGER_MAX_PREPARED_SEQUENCE_LEN
#define GCLK_MANAGER_MAX_PREPARED_SEQUENCE_LEN (GCLK_NUM_OF_CLOCKS)
#endif

/* below values define thresholds for when to consider collected thread stats data to be enough
 * to calculate a valid PU metric properly */
/* default minimum CPU time per thread to accumulate before PU calculation */
#define GCLK_MANAGER_MIN_PU_STATS_CPU_TIME (32768)
/* defualt minimum number of thread schedules that must have occured before PU calculation */
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
 * @breif Different scaling approaches.
 *
 * Different options of how clock scaling is performed.
 */
typedef enum {
    SCALE_DIRECT,            /**< clock can be scaled by changing a single scale factor of a clock instance directly. */
    SCALE_UPTREE_RELATIVE,   /**< clock can only be scaled indirectly by changing a single clock scaler up the tree,
                                  where the scaled clock frequency is not the same as the frequency at the output clock.
                                  I.e., the scaled clock is still subject to some sort of scaling before feeding the output clock. */
    //TODO: for predefined sequences it could make sense to provide one for up and down, or even a list/LUT for a set of target frequencies
    SCALE_SEQUENCE,          /**< clock can be scaled by a predefined multi-step sequence */
    SCALE_INTERMEDIATE_TOPO_AUTO,  /**< clock can be scaled by temporarily changing to another topology before
                                        adapting the current one and switching back to it. AUTO refers to the
                                        fact that not a fixed intermediate topology must be used but instead the most
                                        viable option can be selected at runtime, depending on active constraints */
    //TODO: another feasible approach would be to do a plain mux between preconfigured HF and LF clocks
} gclk_scale_approach_t;

/* Defines for one clock instance how it can be scaled
 * TODO: the scaling methods should define for which topology this applies.
 *       E.g., in case of nucleo-l476rg SYSCLK may be scaled by a direct change of MSIRANGE in case of topology 1 ([SYSCLK]-->[MSI]-->[MSIMUX]-->[MSIRANGE]-->[MSI_BASE]),
 *       but in case of topology 6 ([SYSCLK]-->[PLL_R]-->[PLL_VCO]-->[PLL_M]-->[PLL_PREDIV_MUX]-->[MSI]-->[MSIMUX]-->[MSIRANGE]-->[MSI_BASE]) it is only possible to update it indirectly.
 *       Overall it seems more applicable to define scale settings per topology of the clock handle.
 *       */
typedef struct {
    const gclk_t *output_clk;   /**< the clock that we want to update to a new frequency (i.e. the core clock in most cases) */
    union {
        const gclk_t *scale_clk; /**< in case it is a direct or uptree scale operation: the output_clock is scaled by only changing this node */
        gclk_manager_sequence_step_t *sequence;
    };
    size_t sequence_len;
    const uint32_t *default_freqs;       /**< preferable default frequencies for this scale setting (NULL tells the manager to derive frequencies dynamically) */
    unsigned topology_id: 8;            /**< the topology (of the core clock handle) at which this scaling method is applicable */
    unsigned default_freqs_cnt: 8;      /**< number of default frequencies for this scale setting */
    gclk_scale_approach_t approach: 8;
} gclk_scale_setting_t;

/* Wait state value to indicate a don't care condition for a frequency conf limit */
#define GCLK_WS_NOSPEC (255)

typedef struct {
    const freq_conf_limit_t *limits; /* Core voltage and Wait state limits that apply to a clock */
    const size_t            len;     /* The number of limits pointed to by limits */
    const gclk_t            *clk;    /* The clock instance these limits apply to */
} clock_freq_conf_limits_t;

/* stores data of the physical clock property model which is used to calculate the
 * power consumption for different frequency configurations. */
typedef struct {
    const gclk_t *clk;   /* The clock handle this data refers to */
    uint32_t P_en_nW;    /* The static power this clock node draws when enabled (i.e. F(clk) > 0).
                            May be 0 for clocks that are not gateable. */
    uint32_t C_fF;       /* The (equivalent) capacitance of the clock node in femto Farad.
                            This value affects the dynamic (frequency dependent) consumption.
                            May be 0 for clocks that only suport a fixed frequency. */
} gclk_manager_power_properties_t;

/* Callback type used to return configurations found during exploration.
 * This is helpful to directly use each individual result of a full exploration run 
 * (e.g. for immediate output) instead of repeating the same exploration multiple times
 * to eventually query all configs via separate query-response calls */
typedef void (*gclk_exploration_result_cb_t)(clk_topology_entry_t *conf, size_t len, gclk_cmp_result_t res, unsigned valid_idx, void *ctx);

typedef enum {
    CB_ON_VALID,     /*< cb is executed for every valid configuration found */
    CB_ON_BEST_ONLY, /*< cb is executed only for the best configuration found */
    CB_ON_BETTER,    /*< cb is executed everytime a better configuration is found 
                         Note: this is mainly useful for debugging (observing the exploration) */
} gclk_exploration_result_cb_mode_t;

typedef struct gclk_exploration_result_cb_conf {
    gclk_exploration_result_cb_t valid_conf_found_cb;
    void *ctx;
    gclk_exploration_result_cb_mode_t cb_mode;
} gclk_exploration_result_cb_conf_t;

typedef struct {
    gclk_cmp_func_t func;
    const char *name;
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

/* Context data used by the @ref gclk_manager_cmp_single_scaler_range_limited compare function */
typedef struct {
    const gclk_t *scale_clk;      /*< the single clock scaler that will be used for DFS adaptations */
    unsigned scale_clk_topo_idx;  /*< the topology entry index that refers to the above clock.
                                      Note: this must be valid for all calls of the compare function.
                                            I.e. the compare function is not suitable for comparing different
                                            topologies (but different factor configs of the same topology). */
    uint32_t target_freq;               /*< The target frequency of the last clock in the topology. This usually
                                            refers to the core clock */
    gclk_freq_limit_t scaler_fo_limits; /*< absolute limits for the output freq. at the scaled clock */
    uint32_t scaler_factor_target; /*< one specific factor of the scaled clock instance that defines the subset
                                       of configurations that will be evaluated in more detail.
                                       Used to skip the more complex comparison step for all other factors 
                                       (because every comparison step always considers every possible factor anyway) */ 
    gclk_factor_limit_t scaler_factor_limits; /*< factor limits for the scaled clock instance. Every config that is out
                                                  of this limits could already be predetermined to be invalid. */ 
    uint32_t min_error; /*< cached value of the minimum target frequency error found during previous comparisons.
                            Should be initialized to the highest value possible before starting comparison. */
    uint32_t min_infeasible_cnt; /*< cached count of infeasible scale factors of previous comparisons.
                                     The lower this value, the more frequency steps were found  */
} range_limit_cmp_fun_ctx_t;


/**
 * @brief Topology configuration compare function for explicit frequency list fitting.
 *
 * A compare function that evaluates how well a configuration is suitable for DFS when using a single scaler
 * for the frequency adaptation. This function is applicable if an explicit list of given target frequencies
 * shall be fitted as good as possible. Simply put, this tries to minimize sum(abs(ft_i - f_i))), where
 * tf_i is the target frequency and f_i is the closest possible frequency, i being an element of
 * (0 ... *num of target freqs*).
 * A pointer to a properly initialized @lflae_cmp_fun_ctx_t struct must be given as context.
 */
gclk_cmp_result_t gclk_manager_cmp_lowest_freq_list_abs_err(clk_topology_entry_t *topo_best, size_t len1,
                                                            clk_topology_entry_t *topo_cmp, size_t len2,
                                                            void *arg);

/**
 * @brief Topology configuration compare function maximizing applicable frequency steps.
 * Compares configurations on how well they are suitable for DFS when using a single scaler for the
 * frequency adaptation. Opposed to @ref gclk_manager_cmp_lowest_freq_list_abs_err this performs
 * a more complex check on the applicability of all resulting frequency steps to ensure configurations
 * are not only fitting the factors well but also result in a greater number of feasible settings.
 * A pointer to a properly initialized @range_limit_cmp_fun_ctx_t struct must be given as context.
 * This function is applicable if no particular list of target frequencies is given as target
 * but instead the goal is to exploit the available range of the single used scaler as good as possible.
 * Configurations are compared regarding the following aspects (in descending priority):
 * - Configs matching the single given target frequency closer are better (with the initial factor)
 * - Configs enabling a greater number of frequency steps are better
 * - Configs that result in lower power consumption are better */ 
gclk_cmp_result_t gclk_manager_cmp_single_scaler_range_limited(clk_topology_entry_t *topo_best, size_t len1,
                                                               clk_topology_entry_t *topo_cmp, size_t len2,
                                                               void *arg);

/* a compare function that only considers exact frequency matches valid and prefers configurations
 * with a lower power consumption. The consumption is calculated with the clock power model that is
 * parameterized with platform-specific power properties for relevant clock nodes. */
gclk_cmp_result_t gclk_manager_cmp_topology_exact_leaf_freq_pmin(clk_topology_entry_t *topo_best, size_t len1,
                                                         clk_topology_entry_t *topo_cmp, size_t len2,
                                                         void *arg);

/* a compare function that prioritizes to match the frequency as close as possible but prefers
 * configurations with a lower power consumption if possible. (calculated via clock tree power model) */
gclk_cmp_result_t gclk_manager_cmp_topology_closest_leaf_freq_pmin(clk_topology_entry_t *topo_best, size_t len1,
                                                                   clk_topology_entry_t *topo_cmp, size_t len2,
                                                                   void *arg);

static const topology_cmp_func_names_t topology_cmp_funcs[] = {
    { .func = gclk_cmp_topology_for_closest_leaf_freq,         .name = "closest_leaf" },
    { .func = gclk_cmp_topology_for_closest_leaf_freq_min_sum, .name = "min_sum" },
    { .func = gclk_cmp_topology_for_closest_leaf_freq_max_sum, .name = "max_sum" },
    { .func = gclk_cmp_topology_for_closest_leaf_freq_min_max, .name = "min_max" },
    { .func = gclk_cmp_topology_for_closest_leaf_freq_max_max, .name = "max_max" },
    { .func = gclk_cmp_topology_for_exact_leaf_freq,           .name = "exact_leaf" },
    { .func = gclk_manager_cmp_topology_exact_leaf_freq_pmin,  .name = "fexact_pmin" },
};

typedef struct {
    gclk_factor_match_func_t func;
    const char *name;
} factor_match_func_names_t;

static const factor_match_func_names_t factor_match_funcs[] = {
    { .func = gclk_match_iter_mul_recurse_div,   .name = "iter_mul_recurse_div" },
    { .func = gclk_match_iter_mul_factorize_div, .name = "iter_mul_factorize_div" },
    { .func = gclk_match_exact_full_iter,        .name = "exact_full_iter" },
    { .func = gclk_match_closest_full_iter,      .name = "closest_full_iter" },
};

/* @brief perform initialization operations of the clock manager
 * @return 0 on success */
int gclk_manager_init(void);

/*
 * @brief execute a sequence of clock operations
 *
 * @param steps    list of sequence steps to execute
 * @param step_cnt number of steps @steps contains
 */
void gclk_manager_run_sequence(gclk_manager_sequence_step_t *steps, size_t step_cnt);

/*
 * @brief check if the given topology config breaks constraints.
 *
 * @param[in] constraints    constraints to be checked.
 * @param[in] constr_cnt     number of elements @constraints points to.
 * @param[in] topo           clock topology entries describing the checked topology.
 * @param[in] topo_len       number of clock instances in @topo.
 *
 * @return    The first unfulfilled constraint of @topo or NULL if all constraints are fulfilled.
 */
const gclk_freq_constraint_t* gclk_manager_conf_breaks_constraint(const gclk_freq_constraint_t *constraints, unsigned constr_cnt, clk_topology_entry_t *topo, uint32_t topo_len);

/*
 * @brief Get an equivalent fraction that combines all involved scaling factors.
 *
 * @param[in]     topo       The topology all scaling factors will be combined of.
 * @param[in]     topo_len   Length of @p topo.
 * @param[in,out] f          Pointer to where equivalent factor will be stored.
 *
 */
void gclk_manager_get_combined_topology_fraction(clk_topology_entry_t *topo, size_t topo_len, gclk_fraction_t *f);

void gclk_manager_notify_multi_clk_change(gclk_manager_sequence_step_t *seq, size_t seq_len,
                                          clk_topology_entry_t *old_topo, size_t old_topo_len,
                                          clk_topology_entry_t *new_topo, size_t new_topo_len,
                                          bool post_change);

void gclk_manager_run_sequence_with_notify(gclk_manager_sequence_step_t *seq, size_t seq_len, bool print_only);

/* @brief Register a callback for notification of a clock frequency change.
 *
 * @param clk  Reference to the clock to register a notification for.
 * @param nle  Storage that will hold the registration data.
 * @param cb   Callback that will be executed before and after the frequency of clk is changed */
void gclk_manager_register_clk_change_cb(const gclk_t *clk, gclk_clock_change_notify_list_t *nle,
                                   clock_change_cb_t cb);

/* @brief Unregister a callback for notification of a clock frequency change.
 *
 * @param nle  The previously registered notification entry.
 */
void gclk_manager_unregister_clk_change_cb(gclk_clock_change_notify_list_t *nle);

void gclk_manager_notify_clk_change(const gclk_t *clk, uint32_t f_old, uint32_t f_new, bool post_change);

/**
 * @TODO: this should also be removed. As a replacement the transition manager should
 *        use information provided by this files interface to register hooks that automatically
 *        call functions to setup the required voltage(s?) by also considering other
 *        depenencies (such as flash waitstates, peripheral use etc..).
 * @brief get the available options for parents that can be configured
 *
 * @note  There are two cases to consider:
 *        (A): virtual/logical parent association (some node is the source, but it can not be changed, nor read from HW)
 *        (B): runtime-dynamic config (selecting one of multiple parents), can (and must be) read/written from/to HW
 *
 * @param[in] f_core_old_hz   the current core frequency
 * @param[in] f_core_new_hz   the new wanted core frequency
 *
 * @return    f_core_new_hz    If the new frequency can now be set up
 *            < f_core_new_hz  If the transition is not possible in one step
 *                              That means at least one intermediate frequency step is required at maximum the returned
 *                              frequency. After that was performed, this function can be called again with the new
 *                              value for f_core_old_hz. Repeat this till the transition is completed.
 *            -1               If updating the core voltage was not possible.
 */
uint32_t core_voltage_pre_change_hook(uint32_t f_core_old_hz, uint32_t f_core_new_hz, void *ctx);

const freq_conf_limit_t *gclk_manager_get_freq_conf_limit(const gclk_t *clk, uint32_t freq, bool optimize_ws);

/**
 * @brief Enable/disable automatic switching of voltage range on clock changes
 *
 * TODO: Even though the capabilities DVS, DFS, WSA and the LowVoltage/FastFlash policy can be controlled independently
 *       (because that is nice for detailed evaluation purposes), in reality, this is most likely not how these parameters
 *       are expected to work. The policy should actually only matter if both DVS *and* WSA are enabled because
 *       disabling one of them removes interdependencies to the other. I.e. there is no reason for the policy to favor
 *       LowVoltage if DVS is disabled (as we can then always further optimize the flash access speed instead).
 *       One exception to this might be scenarios where the decision whether to prefer DVS/WSA would change depending on other aspects.
 *       I.e. a fast rate of voltage changes may not be wanted due to its time overhead.
 */
void gclk_manager_enable_voltage_auto_scale(bool on);

/**
 * @brief Enable/disable automatic adaptation of flash waitstates on clock changes
 */
void gclk_manager_enable_flashws_auto_update(bool on);

/**
 * @brief Enable/disable automatic adaptation of core frequency
 */
void gclk_manager_enable_dynamic_frequency_scaling(bool enable);

/**
 * @brief Enable/disable automatic assessment of performance utilizaiton of threads
 */
void gclk_manager_enable_pu_assessment(bool enable);

/**
 * @brief Returns the current DVS adaptation policy.
 *
 */
gclk_manager_dvs_policy_t gclk_manager_get_dvs_policy(void);

/**
 * @brief Set policy on how to adapt DVS settings.
 *
 * @param policy  Policy that defines which optimization to prefer
 */
void gclk_manager_set_dvs_policy(gclk_manager_dvs_policy_t policy);

/* @brief print thread/CPU utilization metrics
 * @note For debug/testing purposes
 */
void gclk_manager_print_util_metrics(void);

/* @brief print topology configuration with some matadata
 */
void gclk_manager_print_topology_conf(clk_topology_entry_t *topology, uint32_t size, bool min_max, bool factors);

/* @brief clear performance util data
 *
 * This resets all threads performance metrics collected at scheduling events
 */
void gclk_manager_clear_performance_util_data(void);

/* @brief calculate performance utilization factor for the given task id */
int gclk_manager_calculate_pu_factor(uint32_t task_id, bool debug_print);

/* @brief set parameters that control Performance Utilization-based DVFS
 *
 * @param fboost           The frequency that is set up if the PU of a thread that is about to be scheduled is above fboost_pu_th
 * @param fthrottle        The frequency that is set up if the PU of a thread that is about to be scheduled is below fthrottle_pu_thesh
 * @param fboost_pu_th     The minimum PU threshold a thread must have to setup fboost before scheduling
 * @param fthrottle_pu_th  The max PU threshold a thread must have to setup fthrottle before scheduling
 */
void gclk_manager_set_dvfs_pu_params(uint32_t fboost, uint32_t fthrottle, int fboost_pu_th, int fthrottle_pu_thesh);

/* @brief Start thread to cycle through different core frequencies
 *
 * The call will block till the cycling is complete. Currently there is no way to change the
 * frequencies used for the cycle and static configuration is used for that instead.
 *
 * @param cycle_us the cycle time each frequency stays active in us.
 * @param min_schedules the min number of schedules to accumulate per thread.
 */
void gclk_manager_start_freq_cycler(unsigned int cycle_us, uint32_t min_schedules);

/* @brief Do a complex transition a clock to a new frequency
 *
 * @note This may temporariy switch the clock to another topology before setting up the final configuration in cases
 *       that prohibit changing the clock while being used
 *
 * @param clk  The clock tansitioned to a new frequency
 * @param freq The new target frequency
 */
void gclk_manager_transition(const gclk_t *clk, uint32_t freq);

/* @brief Set clock to the given frequency
 * @note this is a high level set frequency function that also handles pre- / and post-processing
 *       to notify clocks about the change */
bool gclk_manager_set_freq(const gclk_t *clk, uint32_t freq);

/* @brief Set clock to the given factor
 * @note this is a high level set frequency function that also handles pre- / and post-processing
 *       to notify clocks about the change */
bool gclk_manager_set_factor(const gclk_t *clk, uint32_t factor);

/* @brief Set the core clock to the given frequency using the currently active scale setting.
 * @note this is a high level set frequency function that also handles pre- / and post-processing
 *       to notify clocks about the change. It might intermediately alter topology settings */
bool gclk_manager_scale_core_freq(uint32_t freq);

/* @brief Set the topology of the given clock from the currently active setting to a new topology.
 * @note this is a high level set frequency function that also handles pre- / and post-processing
 *       to notify clocks about the change.
 *
 * @param[in] clk              The clock instance that will be switched to another topology.
 * @param[in] target_topology  The zero based id of the topology that the clock is switched to.
 * @param[in] target_freq      The frequency that is aimed for with the new topology,
 *                             GCLK_INVALID_FREQ if current frequency should be used.
 * @param[in] cmp_func         The compare function that will be used to find the most suitable topology config.
 *
 * @return   The new frequency of clk.
 **/
uint32_t gclk_manager_switch_topology(const gclk_t *clk, int target_topology, uint32_t target_freq, gclk_cmp_func_t cmp_func);

/* @brief Set clock instance that is used for dynamic frequency scaling
 */
void gclk_manager_set_dfs_clock_handle(const gclk_t *clk);

/**
 * @brief Prepare configs for D(V)FS operation with given frequencies (as close as possible).
 *
 * @return  The number of distinct frequencies that were prepared on success.
 *          <0 on error.
 *
 * @note This function sets up a list of frequencies applicable for D(V)FS operation.
 * A scale setting must be active for this to work (@see \ref gclk_mananger_set_active_scale_setting()
 * and \ref gclk_manager_ctx_t.active_core_scale_setting). Resulting frequencies depend on the
 * capabilities of the selected scale setting and the current topology/frequency configuration.
 * The frequency values handed to this function are subject to a matching procedure which can not
 * guarantee all frequencies can be obtained exactly as specified (due to hardware limits).
 *  As applicability and effectivity of different frequency configurations for
 * some scale settings may strongly depend on the currently active topology/frequency configuration, it
 * is recommended to use this function after setting up a config that is well suited for D(V)FS. The
 * recommended way to do this is by calling @ref gclk_manager_setup_default_dfs_topo_conf() before.
 * Calling this function with other configs may result in fewer, less flexible, or no frequency options
 * being explored.
 */
int gclk_mananger_set_dfs_frequencies(const uint32_t *freqs, size_t cnt);

/**
 * @brief Prepare configs for D(V)FS operation with default frequencies.
 *
 * @return  The number of distinct frequencies that were prepared on success.
 *          <0 on error.
 *
 * Same as @ref gclk_mananger_set_dfs_frequencies() but using default frequencies.
 * If provided, this uses frequencies defined by platform-specific configuration. If the platform
 * defines no default values explicitly, it tries to automatically come up with reasonabe settings
 * based on available configuration options and the active scale setting.
 */
int gclk_mananger_set_default_dfs_frequencies(void);

const gclk_scale_setting_t* gclk_mananger_get_active_scale_setting(void);

/** @brief Return the clock handle that directly drives the CPU */
const gclk_t* gclk_manager_get_core_clock_handle(void);

/* @brief Disable clocks that are currently not being used by other active clocks
 * @note Currently this does not consider 'invisible' dependencies. I.e. if an intermediate clock
 *       (one that is not a leaf) is indeeed needed for operations even if the clock is not used
 *       by other clock instances. One way to handle this is to integrate simple resource allocation
 *       to respective peripherl drivers or other code that e.g. needs clocks tof specific bus access.
 */
void gclk_manager_disable_unused(void);

/* @brief Get clock freqs used for DFS and freq cycle thread
 *
 * @param[out] pointer that will point to an array of frequency values
 * @return number of elements contained in the frequency array
 */
unsigned int gclk_manager_get_dfs_freqs(uint32_t **freqs);

/* @brief get available options for automatic scaling
 *
 * @param s pointer that will point to the array of scale settings after return
 * @return number of scale settings entries
 * */
int gclk_manager_get_scale_settings(const gclk_scale_setting_t **s);

/* @brief set request flag for PU stat collection for a given thread
 *
 * @param tid  the thread PU data will be collected for
 * */
void gclk_manager_enable_pu_stat_request_for_thread(kernel_pid_t tid);

/* @brief set the active scale setting to the given index if applicable
 *
 * @param i   index of the scale setting that will be set active
 * @return    true if applied, fasle if not appliccable
 * */
bool gclk_mananger_set_active_scale_setting(unsigned i);

/* @brief get sources that are allowed to drive the core clock
 *
 * @param clks pointer that will point to the array of clocks after return
 * @return number of clocks
 * */
int gclk_manager_get_allowed_core_clock_sources(const gclk_t ***clks);

void gclk_manager_execute_sequence_step(gclk_manager_sequence_step_t *step);

int gclk_manager_derive_sequence(const clk_topology_entry_t *src_topo, uint32_t src_len,
                                 const clk_topology_entry_t *target_topo, uint32_t target_len, gclk_manager_sequence_step_t *out_seq, unsigned max_seq_steps);

/* NOTE: This is only meant to be used to establish baselines for fine grained evaluation of respective impact of waitstates and voltage settings.
 * ONLY use this function directly if you know EXACTLY what you are doing!
 * i.e. it is not safe to call this for a new smaller frequency when this frequency is not yet set up.
 * similarly performing manual freq changes after this has been called manually can lead to prohibited state
 * as there is no hook in place that takes care of raising voltages/waitstates again.*/
void gclk_manger_update_vcore_and_ws_config(bool optimize_flash, bool enable_vscale, bool fup);

/* @brief returns the min required flash waitstates and core voltage required for the given tree conf
 *
 * @param[in] tree_conf              tree configuration as a list of arbitrarily sorted clock nodes.
 * @param[in] tree_size              number of clocks in the given tree i.e., the length.
 * @param[out] min_ws                variable that will be set to the minimum required flash wait states.
 * @param[out] min_vc_idx            variable that will be set to the minimum required core voltage index.
 * @param[in]  dvspolicy             the policy that determines how mutually exlusive optimizations to wait states or core voltage are resolved.
 *                                   In case either voltage can be reduced or flash access can be sped up, this defines which of both options
 *                                   will be prefered.
 */
void gclk_get_min_required_ws_vc_from_tree_config(clk_topology_entry_t *tree_conf, size_t tree_size, unsigned *min_ws, unsigned *min_vc_idx,
                                                  gclk_manager_dvs_policy_t dvspolicy);

/* @brief simulates a reconfiguration step on a tree configuration model
 *
 * @param[in] step                   the reconfiguration step to apply.
 * @param[in] tree_conf              tree configuration as a list of arbitrarily sorted clock nodes.
 * @param[in] tree_size              number of clocks in the given tree i.e., the length.
 *
 */
void gclk_manager_simulate_seq_step_on_tree_conf(gclk_manager_sequence_step_t *step, clk_topology_entry_t *tree_conf, size_t tree_size);

/* @brief bruteforces the best frequency configuration based on a given compare function
 *
 * @param[in]  clk                        the clock instance a topology config is searched for.
 * @param[out] best_topology              pointer to the topology config where the result will be stored.
 * @param[in,out] topo_len                in: max length of best_topology, out: the actual length of the topology config found.
 * @param[in,out] topo_idx                in: GCLK_UNDEFINED_TOPOLOGY if any topology is fine,
 *                                            the 0 based topology index of the given clk used to find a topology config,
 *                                        out: the topology idx that was chosen.
 * @param[out] ret_n_valid                out: if force_nth < 0 this will indicate the number of configurations that are
 *                                             considered valid by the given compare function. Set to NULL for don't care.
 * @param[in]  force_nth                  for when multiple configurations are valid / applicable this forces the function
 *                                        to return the nth valid config (zero based).
 * @param[in]  valid_conf_found_cb_conf   Config struct for a callback that will be executed once for each valid config found,
 *                                        or NULL if not needed. If force_nth is < 0 it is only called for the specific index given.
 *
 * @param[in] cmp_func         compare function that is used to diceide which topology config is best.
 * @param[in] cmp_func_ctx     optional context variable handed to each call of cmp_func. */
uint32_t gclk_manager_brute_force_freq_conf(const gclk_t *clk, clk_topology_entry_t *best_topology, uint32_t *topo_len,
                                            int *topo_idx, gclk_cmp_func_t cmp_func, void *cmp_func_ctx, size_t *ret_n_valid, int force_nth,
                                            gclk_exploration_result_cb_conf_t *valid_conf_found_cb_conf);

/* @brief Sets up a topology configuration that 'works well' with the given scale setting.
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
 * @param scs    scale setting structure describing how DFS shall be performed
 *
 */
bool gclk_manager_setup_default_dfs_topo_conf(const gclk_scale_setting_t *scs);

void gclk_manager_print_step_sequence(gclk_manager_sequence_step_t *seq, size_t len);

void gclk_manager_default_stdio_reinit_cb(const gclk_t* altered_clk, const gclk_t* affected_clk, uint32_t f_old, uint32_t f_new, bool post_change);
void gclk_manager_default_timer_reinit_cb(const gclk_t* altered_clk, const gclk_t* affected_clk, uint32_t f_old, uint32_t f_new, bool post_change);

#ifdef __cplusplus
}
#endif

#endif /* GCLK_MANAGER_H */
/**
 * @}
 */
