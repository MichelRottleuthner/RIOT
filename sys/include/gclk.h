/*
 * Copyright (C) 2020 HAW Hamburg <michel.rottleuthner@haw-hamburg.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */
/**
 * @defgroup    sys_gclk Generic clock configuration
 * @ingroup     sys
 * @brief       Provides a generic clock configuration module
 *
 * This module is intended to configure and control platform specific clocks via a generic API.
 * The purpose of this API is to unifiy the hardware access for different clock instances.
 *
 * The API generalizes access to various clock configuration capabilities such as:
 * - Clock gating: basic enable/disable control.
 * - Clock scaling: adjusting prescaler/multiplier factors.
 * - Clock muxing: adjusting clock routing, i.e., switching between different sources.
 *
 * By itself, the API does **not** guarantee a specific setting to be possible or allowed under
 * all cicumstances and it does **not** prevent the user from setting up invalid configurations!
 * It provides only hardware presentation abstraction. The API therefore simply does what it is
 * instructed to do without any semantic checks on whether that change is reasonable or
 * even applicable. E.g., this API will allow you to things like:
 * -Disabling the oscillator currently feeding the core clock (effectively freezing the system).
 * -Overclocking the CPU far beyond the manufacturer specification.
 * -Configuring combinations of scaling factors that effectively violate hardware constraints.
 *
 * Therefore this API should only be used manually if you know **exactly** what you are doing.
 * For higher level control and safe operation we refer to the @ref gclk_manager instead, which
 * uses this API to to automatically set up specific valid configurations and performs dynamic
 * adaptations.
 *
 * All clocks have runtime information and metadata (possible configuration options) which can
 * be accessed. In many cases respective configurations can also be changed dynamically:
 * - The current parent clock that serves as input to a clock.
 * - If applicable, a list of parent clocks which can be used as alternative input to a clock
 *   (possible parents).
 * - The enabled / disabled state of the clock (depends on the parent if not gateable itself).
 * - The current frequency (depends on the parent configuration and the own scaling factor).
 * - If applicable, a list of possible scaling factors a clock can be set to.
 *
 * @todo Some aspects that are still WIP/under consideration
 *  - Additional properties (not implemented yet)
 *    - low power capabilities (e.g., pm mode availability)
 *    - type of clock (internal/external, RC/Crystal)
 *    - accuracy (PPM)
 *  - power consumption metrics are currently under development in form of a clock-tree power model.
 *  - There are MCUs where the clock configuration uses a relatively big number of functionally identical instances.
 *    e.g., on the SAMd21 there are ~8 of the same clock generators, and ~the same order of magnitude gateable muxes
 *    after that. Configuring the instances happens by writing the instance ID to a register, and then reading/writing
 *    a config register (always the same across all instances). Since the memory map pattern is different those can not
 *    (fully) reuse the generic implementations for gates/scalers/muxes.
 *    - An aproach to deal with this would be e.g., decorating the generic implementation with code that writes the
 *      respective idx before using the (always same) config regs. So all the instances share the same config, the same
 *      registers, but differ only by index. The current approach of storing the data would create unneccessary
 *      overhead on that platform. It would be better to reuse the same function calls for all classes.
 *      And reuse the same register definitions across (almost) all instances.
 *  - Some platforms may not have same-sized register access for all registers (example SAMd21, require 32 bit, some 16)
 *  - A generic way to handle additional custom properties (get/set) could be useful for very specific settings that are
 *    generally useful but not very common/generic (like phase locking control between different clock domains).
 *
 * @{
 *
 * @file
 * @brief       Generic clock configuriton API
 *
 * @author      Michel Rottleuthner <michel.rottleuthner@haw-hamburg.de>
 */
#ifndef GCLK_H
#define GCLK_H

#include <stdbool.h>
#include <stdint.h>

#include "kernel_defines.h"
#include "bitarithm.h"
#include "gclk_conf.h"
#include "clist.h"
#include "periph/gpio.h"
#include "ppbitmask.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Value that indicates an invalid frequency.
 */
#define GCLK_INVALID_FREQ       (0xFFFFFFFFU)

/**
 * @brief The highest frequency value that may be encoded.
 */
#define GCLK_HIGHEST_VALID_FREQ (GCLK_INVALID_FREQ - 1U)

/**
 * @brief Value that indicates an undefined topology index.
 *
 * As input this may be used to define the specific topology index as *dont care* / wildcard.
 * Given as a result from operations that should return a specific topology index this indicates
 * an empty / invalid result.
 */
#define GCLK_UNDEFINED_TOPOLOGY (-1)

/**
 * @brief Flags encoding topology related reconfiguration constraints of a clock instance.
 *
 */
enum gclk_clk_topology_flags {
    GCLK_STOP_FOR_UPDATE          = 0x01, /*< The clock must be stopped before it can be updated to another frequency. */
    GCLK_STOP_PARENT_FOR_UPDATE   = 0x02, /*< The clock may not be changed while the parent is active.
                                              (e.g. on STM32 the PLL VCO scaler must be stopped before any of its
                                              children (P,Q,R) can be updated) */
    GCLK_STOP_CHILDREN_FOR_UPDATE = 0x04, /*< The clock may not be used during an update.
                                              (e.g. on STM32 all children of PLLM (PLL prescaler) must be stopped before
                                              it can be updated.) */
    GCLK_STRICT_UPTREE_DEPENDENT  = 0x08, /*< Indicates that a clock can not be reconfigured arbitrarily, but instead
                                              its state strictly depends on another clock up in the tree. This is usually
                                              the case if there is some hardwired dependency e.g., if a prescaler is shared
                                              across two outputs or if there is a clock that limits its output automatically
                                              if its input is above some value. */
};

/**
 * @brief values to encode a specific set operation for enable, or a pure read operation to get the current state
 */
typedef enum gclk_enable_option {
    GCLK_ENABLE,  /*< enables the clock */
    GCLK_DISABLE, /*< disables the clock */
    GCLK_READ,    /*< request the current state only without modification */
} gclk_enable_option_t;

#ifndef GCLK_CONF_REG_IDX_BITWIDTH
/**
 * @brief Number of bits used to store a configuration register index.
 *
 * This can be overwritten by the platform config in case more bits are needed.
 * @note if more than 4 bits are used the regref definition will not fit into
 *       a single 32 bit integer anymore.
 */
#define GCLK_CONF_REG_IDX_BITWIDTH  (4)
#endif

/**
 * @brief Config register descriptor.
 *
 * Combines commonly needed data to describe which registers and bits are used for clock control.
 * @note members of this struct shall never be accessed directly. Instead, use utility wrappers
 *       defined below in order to still allow seamless changes of the encoding in the future.
 */
typedef struct __attribute__((packed)) {
    unsigned int         en: GCLK_CONF_REG_IDX_BITWIDTH; /**< The register(-ID) that contains the enable bit. */
    unsigned int        rdy: GCLK_CONF_REG_IDX_BITWIDTH; /**< The register(-ID) that contains the ready bit. */
    unsigned int       conf: GCLK_CONF_REG_IDX_BITWIDTH; /**< The register(-ID) that contains configuration bits. */
    unsigned int     en_bit: 5; /**< Bit index of the enable bit (0-31) */
    unsigned int    rdy_bit: 5; /**< Bit index of the ready bit (0-31) */
    unsigned int   conf_lsb: 5; /**< Bit index of the lowest configuration bit (0-31) */
    unsigned int   conf_msb: 5; /**< Bit index of the highest configuration bit (0-31) */
} gclk_reg_ref_t;

/**
 * @brief Configuration registers defined externally by platform code.
 */
extern uint32_t volatile * const conf_regs[];

/**
 * @brief Get the register for enable control from a regref value.
 */
static inline volatile uint32_t * gclk_regref2enable_reg(gclk_reg_ref_t regref) {
    return conf_regs[regref.en];
}

/**
 * @brief Get the enable mask from a regref value.
 */
static inline uint32_t gclk_regref2enable_mask(gclk_reg_ref_t regref) {
    return 1 << regref.en_bit;
}

/**
 * @brief Get the register for the ready state from a regref value.
 */
static inline volatile uint32_t * gclk_regref2ready_reg(gclk_reg_ref_t regref) {
    return conf_regs[regref.rdy];
}

/**
 * @brief Get the ready mask from a regref value.
 */
static inline uint32_t gclk_regref2ready_mask(gclk_reg_ref_t regref) {
    return 1 << regref.rdy_bit;
}

/**
 * @brief Get the register for value configuration from a regref value.
 */
static inline volatile uint32_t * gclk_regref2conf_reg(gclk_reg_ref_t regref) {
    return conf_regs[regref.conf];
}

/**
 * @brief Get the configuration value mask from a regref value.
 */
static inline uint32_t gclk_regref2conf_mask(gclk_reg_ref_t regref) {
    uint32_t mask = 0xFFFFFFFF;
    mask = mask >> regref.conf_lsb;
    mask = mask << regref.conf_lsb;
    mask = mask << (31 - regref.conf_msb);
    mask = mask >> (31 - regref.conf_msb);
    return mask;
}

/**
 * @brief Forward-declaration of the generic clock handle type.
 */
typedef struct gclk_base gclk_t;


/**
 * @brief Entry describing the config of one clock node of a specific topology configuration.
 */
typedef struct clk_topology_entry {
    const gclk_t  *clk;          /**< The clock this data refers to. */
    uint32_t       clk_freq;     /**< The frequency clk is set to (which depends on the config below and uptree settings). */
    uint32_t       factor: 24;   /**< The scaling factor of clk. */
    uint32_t       par_idx: 6;   /**< The idx of the parent clock of clk. */
    uint32_t       enabled : 1;  /**< The enable (gating) state of the clock. */
    uint32_t       propagation_pending : 1; /**< Marker indicating this config is *dirty*, meaning the change must still
                                                 be propagated downtree (only used for operations on the tree model,
                                                 does not refer to state of an active topology). */
} clk_topology_entry_t;

/**
 * @brief Different types of scalers supported.
 */
enum gclk_scaler_type {
    GCLK_NOSCALE = 0, /**< The clock is **not** scalable. */
    GCLK_MUL     = 1, /**< The clock multiplies the input frequency (scalable). */
    GCLK_DIV     = 2, /**< The clock divides the input frequency (scalable). */
};

/* As an alternative to the above, the individual operations of a clock could be split.
 * Following that approach there are several ways to achive this.
 * A potential problem with this is that we do not want to save a huge list of function pointers.
 * A possible solution to this could be to save the functions as fixed size array that only contains supported functions
 * which are than mapped via a bitfield helper and a guaranteed order of functions (probably shitty performance).
 * To decide what kind of splitting makes sense, we collect all possible functionalities and check how they are mandatory/optional in which case:
 *
 * **Mandatory operations**
 * -get_parent: pretty much mandatory. Albeit arguable for source clocks, the handling should actually be unified to always
 *              go through this call.
 *              - information on possible parents always needs to be stored (either single fixed or some acceptable list)
 *                So the same memory and pattern used for storing a list of acceptable values can and should be reused for static
 *                parents
 *
 * **Routing operations**
 * -enable: not mandatory -> if not present, the HW-agnostic part should forward it uptree (if possible)
 *          -a plain mux may just be switched between various sources
 *          -a fixed scaler may not be able to be en/disabled
 *          -should be integrated to get/set (NULL)
 *           ATTENTION: when disabling a mux this operation can not be transparently reverted (save previous state in that case?)
 * -set_parent: not mandatory -> only if parent is flexible (mux)
 *
 * **Frequency operations**
 * -set_freq: not mandatory (only applies to scalable clocks)
 * -get_freq: not really mandatory
 *            -only applies to clocks that are either fixed freq, (user-)scalable or alter its input towards their output)
 *            -if not present the parent freq is assumed
 * -check_freq: should be dropped (replaced) with functionality to access numerical data and mapping separately.
 *
 * **Trim Operations**
 * -trim: not implemented yet. Trims the clock by a given fraction e.g. +- N PPB
 * -get_accuracy: not implemented yet. returns accuracy limits as derived from oscillator spec (modified by topology)
 * */

/**
 * @brief  Low-level interface to configure clock routing.
 *
 * @detail This interface is meant to wrap the interaction with the hardware.
 *         This is done completely without knowledge on the configuration options.
 *         The only assumption is that all possible options can be described or
 *         referenced by unabiguous indexes.
 *
 */
typedef struct gclk_scale_ops {
    /* @brief Get currently configured factor from the hardware
     * @param clk  The clock to get the factor of.
     * @return     The currently configured factor as numerical value. */
    unsigned int (*get_factor)(const gclk_t *clk);

    /* @brief Configure new factor by writing to hardware
     * @param clk     The clock that the factor will be changed of.
     * @param factor  The new factor as numerical value.
     * @pre   @factor must be a valid value for clk.
     * */
    void (*set_factor)(const gclk_t *clk, unsigned int factor);
} gclk_scale_ops_t;

typedef struct gclk_mux_ops {
    /* @brief Get currently configured parent from the hardware
     * @param clk  The clock to get the parent of.
     * @return     The currently configured parent as reference. */
    const gclk_t* (*get_parent)(const gclk_t *clk);

    /* @brief Configure new parent by writing to hardware
     * @param clk  The clock that the parent will be changed of.
     * @param idx  The new parent option as index of possible options.
     * @pre   @idx must be a valid value for clk.
     * */
    void (*set_parent)(const gclk_t *clk, unsigned int idx);
} gclk_mux_ops_t;

typedef struct gclk_gate_ops {
    /* @brief Get enabled state from hardware
     * @param clk  The clock to get the ienabled state of.
     * @return     true if enabled, flase if of (gated). */
    bool (*is_enabled)(const gclk_t *clk);

    /* @brief Enable disable clock
     * @param clk  The clock that will be enabled/disabled.
     * @param on   The new enabled state of @clk.
     * */
    void (*enable)(const gclk_t *clk, bool on);
} gclk_gate_ops_t;

/* TODO to be designed and documented properly */
typedef struct gclk_trim_ops {
    unsigned int (*get_accuracy)(const gclk_t *clk);
    unsigned int (*trim)(const gclk_t *clk, unsigned int ppb);
} gclk_trim_ops_t;

typedef union {
    const gclk_scale_ops_t scale_ops;
    const gclk_mux_ops_t   mux_ops;
    const gclk_gate_ops_t  gate_ops;
    const gclk_trim_ops_t  trim_ops;
} gclk_op_t;

typedef struct gclk_range8 {
  uint8_t min;
  uint8_t max;
} gclk_range8_t;

typedef struct gclk_range16 {
  uint16_t min;
  uint16_t max;
} gclk_range16_t;

typedef struct gclk_range32 {
  uint32_t min;
  uint32_t max;
} gclk_range32_t;

/* @todo: provide alternatives with different widths (?) */
typedef uint8_t  gclk_regval_t;
typedef uint16_t gclk_numval_t;

/* @todo provide alternative width (single compiletime-fixed option only?) */
typedef struct gclk_reg_val_factor_lut {
    uint16_t factor;  /* integer factor the clock ins divided or multiplied (i.e. scaled) by */
    uint8_t reg_val;  /* value that must be written to its configuration register to set the above factor */
} gclk_reg_val_factor_lut_t;

/* This mapping makes the configuration of a clock just strictly depend on another clocks configuration.
 * I.e. one clock (A) statically multiplies by either X or Y depending on the setting of another clock (B) */
typedef struct gclk_reg_val_cross_ref_luf {
    const gclk_t *ref_clk;
    /* conf is either set to a specific configuration of the other clock (B) (which is the clock *this* clock (A)
     * depends on) or it is set to NULL if the current state of clock (A) shall be determined (based on the
     * current state of clock (B)) */
    uint32_t (* const luf)(const gclk_t *clk, const clk_topology_entry_t *conf);
} gclk_reg_val_cross_ref_luf_t;

typedef struct gclk_reg_val_clk_lut {
    const gclk_t *clk;  /* clock that is set up for the below register value */
    uint8_t reg_val;     /* value that must be written to its configuration register to set the above clock */
} gclk_reg_val_clk_lut_t;

/* @todo provide alternative width (single compiletime-fixed option only?) */
typedef struct gclk_reg_val_ptr_lut {
    uint16_t                 factor;       /* integer factor the clock ins divided or multiplied (i.e. scaled) by */
    const volatile uint32_t* reg_val_ptr;  /* pointer to the memory that holds the value that must be written to
                                              its configuration register to set the above factor */
} gclk_reg_val_ptr_lut_t;

typedef union factor_mapping {
    const gclk_reg_val_factor_lut_t    *lut;
    const gclk_reg_val_ptr_lut_t       *ptr_lut;
    const gclk_reg_val_cross_ref_luf_t *cross_ref;
    const gclk_range8_t                *range8;
    const gclk_range16_t               *range16;
    const uint8_t                      *list8;
    const uint16_t                     *list16;
    const uint32_t                     fixed_factor;
} gclk_factor_mapping_t;

/* Type that maps a specific parent selection to a config register value */
typedef struct {
    const gclk_t * const parent;     /* parent that is selected in this config */
    const uint32_t  config_reg_val;  /* value that needs to be written to respective config register to select this parent */
} gclk_parent_config_lut_t;

typedef union parent_mapping {
    const gclk_t                   * const * plist;
    const gclk_parent_config_lut_t * const lut;
} gclk_parent_config_mapping_t;

/**
 * @brief clk low level handle
 *
 * This is the common generic low-level handle.
 */
typedef struct gclk_base {
    const gclk_op_t *separated_ops;
    /* TODO: -combine factor_map_op and mapping type into separate type that we just hold a reference to?
     *       -they are tied together anyway so combining them saves memory if multiple clocks
     *        use the same get_factor_op + factor mapping type
     *       -hardware access could benefit from a similar abstraction by providing separate functions
     *        that read/write register values
     *        - for that we would need a somewhat flexible description of involved registers
     *          (e.g. tinyregref and privateptr) and (a maybe separate) description for procedures
     *          how to interface those registers.
     *       -clearly separate access to numerical values from hw specific configuration mapping */
    /* This is not part of the base type ops as it is expected that this often varies even for
     * clocks that (apart from the value mapping) share the same fuctionalities (ops).
     * Having it here makes it possible to overwrite it on a per-instace basis.
     */
    union {
        /*
         * @param idx     the index of the wanted factor (must be guaranteed to be valid!)
         * @param regval  true when mapping idx to register value
         *                false when mapping idx to numerical value
         */
        uint32_t (* const factor_map_op)(const gclk_t *clk, unsigned int idx, bool to_regval);
        /*
         * @param idx            the key that will be mapped to a parent (must be guaranteed to be valid!)
         * @return               register value that corresponds to parent and index
         */
        uint32_t (* const parent_map_op)(const gclk_t *clk, const gclk_t **parent, unsigned int idx);
        /*
         * @Note          This only applies to read-only clocks that are strictily dependent on another clock.
         *                Therefore the topology_flags.GCLK_STRICT_UPTREE_DEPENDENT must be set.
         *
         * @param conf    the configuration of the cross-referenced clock this clock depends on
         *                may be NULL if current value shall be read from the current config of the cross-referenced clock
         */
        uint32_t (* const cross_ref_factor_map_op)(const gclk_t *clk, const clk_topology_entry_t *conf);
    };

    /* this field is used to store mapping information in one of the following ways:
     * NOTE: a single instance may only use ONE of these options (they are mutually exclusive)
     * - a private (HW driver implementation specific) void pointer
     * - a mapping type that provides information on possible scaling factors
     * - a mapping type that provides information on possible parent section options
     **/
    union {
        const void                         *private_data;
        const gclk_factor_mapping_t        factor_mapping;
        const gclk_parent_config_mapping_t parent_mapping;
    };

    union {
        /* this must be set if this clock is ia source, see flags below */
        const uint32_t fixed_input_freq;
        /* this must be set for a non-muxable clock, see flags below */
        const gclk_t   *fixed_parent;
    };

    /* @todo: add an application specific generic context pointer? */
    const char          *name;      /**< TODO: can be made compile-time optional as it is not
                                         functionally required internally. only provides better
                                         usability for human interaction */

    /* @todo: partition generic and user flags */
    struct __attribute__((packed)) {
        enum gclk_clk_topology_flags   topology_flags  : 4;
        enum gclk_scaler_type          scaler_type     : 2;  /*@ todo: could/should? be moved to user flags */
        unsigned int                   conf_cnt        : 16; /* number of configuration options available */
        unsigned int                   scalable        : 1;  /* 1 if the clock can scale its input frequency.
                                                                If set, @separated_ops *must* contain a
                                                                gclk_scale_ops_t compatible reference.*/
        unsigned int                   muxable         : 1;  /* 1 if the clock be switched to different clock inputs.
                                                                If set, @separated_ops *must* contain a
                                                                gclk_mux_ops_t compatible reference.
                                                                If 0, @fixed_parent must hold a reference
                                                                to the parent clock that provides input to
                                                                this clock */
        unsigned int                   gateable        : 1;  /* 1 if the clock be enabled/disabled.
                                                                If set, @separated_ops *must* contain a
                                                                gclk_gate_ops_t compatible reference.*/
        unsigned int                   trimmable       : 1;  /* 1 if the clock be trimmed.
                                                                If set, @separated_ops *must* contain a
                                                                gclk_trim_ops_t compatible reference.*/
        unsigned int                   is_source       : 1;  /* 1 if the clock is a source.
                                                                If set, @fixed_freq must be provided.
                                                                NOTE: not compatible with @muxable!. */
        unsigned int                   user_flags      : 32 - (1 + 1 + 1 + 1 + 1 + 16 + 2 + 4); /* reserved for platform use */
    } flags;
} gclk_t;

/**
 * @brief Extended clock type for primitive clock instances
 *
 * This type extends the clock base type with fields that can be used to express
 * the low level register interface for many common primitive clock instances.
 * The gclk_reg_ref_t type is used to encode register access information in a compact
 * format. This clock type is meant to be used as the datatype for basic gates, muxes
 * and scalers. It is also possible to combine either a mux or a scaler together with
 * a gate in the same instance.
 * A muxable scaler on the other hand must be split into two separate instances that
 * allow either scaling or muxing.
 * In any case, it is always possible to define a fully custom clock type for clocks
 * that are more complex to interface.
 **/
typedef struct {
  gclk_t base;
  const gclk_reg_ref_t regref;
} gclk_basic_clock_t;

/**
 * @brief get more specific basic clock from generic clock type
 */
static inline gclk_basic_clock_t *gclk_to_basic_clock_t(const gclk_t *clk)
{
    return container_of(clk, gclk_basic_clock_t, base);
}

/* TODO: check if static inline is preferable here */
const gclk_scale_ops_t *gclk_get_scale_ops(const gclk_t *clk);
const gclk_mux_ops_t *gclk_get_mux_ops(const gclk_t *clk);
const gclk_gate_ops_t *gclk_get_gate_ops(const gclk_t *clk);
const gclk_trim_ops_t *gclk_get_trim_ops(const gclk_t *clk);

/* combines a constraint that limits input- or output frequency to or from a clock instance */
typedef struct gclk_in_out_clock_constraint {
    gclk_range32_t in_freq;
    gclk_range32_t out_freq;
} gclk_in_out_clock_constraint_t;

typedef enum {
    GCLK_ENSURE_MIN_FREQ,
    GCLK_ENSURE_MAX_FREQ,
    GCLK_ENSURE_EXACT_FREQ,
    GCLK_ENSURE_MIN_FACT,
    GCLK_ENSURE_MAX_FACT,
    GCLK_ENSURE_EXACT_FACT,
    GCLK_ENSURE_EXACT_PARENT,
    GCLK_ENSURE_FIXED_CONF,
} gclk_constraint_type_t;

typedef struct {
    gclk_constraint_type_t type;
    const gclk_t *clk;
    union {
        uint32_t freq;
        uint32_t fact;
        const gclk_t *parent_clk;
        const clk_topology_entry_t *confs;
    };
} gclk_freq_constraint_t;

/* the below probably has to go to a separate DVFS implementation file */
typedef struct {
    const gclk_t   *clk;
    const uint32_t max_freq;
    const uint32_t vcore_mv; /* @todo: maybe just link to the rangedef? */
} dvfs_conf_t;

typedef struct reinit_trigger_conf {
    const gclk_t *affected_clock;
    void (*pre_change_hook_fptr)(void *ctx);
    void (*post_change_hook_fptr)(void *ctx);
    const char *name;
    uint32_t   pre_change_freq;
} reinit_trigger_conf_t;

typedef struct preferred_freq_conf {
    const gclk_t *clk;
    uint32_t     preferred_freq;
} preferred_freq_conf_t;

/* return values for gclk_enable_pin_output function */
enum {
    ENABLE_PIN_OUTPUT_OK = 0,
    ENABLE_PIN_OUTPUT_INVALID_CLOCK,
    ENABLE_PIN_OUTPUT_INVALID_PIN,
};

/* a type to represent a factional number in form of n/d */
typedef struct {
    uint32_t n;
    uint32_t d;
} gclk_fraction_t;

/* a type to represent a frequency limit via a min/max value */
typedef struct {
    uint32_t min;
    uint32_t max;
} gclk_freq_limit_t;

/* a type to represent a factor limit via a min/max value */
typedef struct {
    uint32_t min;
    uint32_t max;
} gclk_factor_limit_t;


//TODO: do we need a global state object for the gclk module itself?

int gclk_module_init(void);
const char *gclk_get_name(const gclk_t *clk);
const gclk_t* gclk_get_clk_by_name(const char *name);

/* @brief compares two fractions
 *
 * returns <0 if mul1/div1 is smaller than mul2/div2
 * returns 0  if mul1/div1 is equal to mul2/div2
 * returns >0 if mul1/div1 is greater to mul2/div2
 */
int gclk_compare_fraction(gclk_fraction_t *a, gclk_fraction_t *b);

/* @brief checks if a frequency is within a given limit.
 *
 * @retval   true   if @p freq is within @p limit.
 * @retval   false  if @p freq is lower or higher than @p limit.
 */
static inline bool gclk_freq_within_limit(uint32_t freq, const gclk_freq_limit_t *limit) {
    return (freq < limit->max) && (freq > limit->min);
}

/**
 * @brief get the current frequency of this clock
 */
uint32_t gclk_get_current_freq(const gclk_t *gclk);

/**
 * @brief get an equivalent representation of uptree clocks
 *
 * @param[in] clk   clock instance for which the uptree config is read
 * @param[in,out] m    equivalent multiplier combining all uptree clocks (must be set to 1 before calling)
 * @param[in,out] d    equivalent divisor combining all uptree clocks (must be set to 1 before calling)
 *
 * @return the frequency of the root source clock
 */
uint32_t gclk_get_current_equivalent_uptree_factors(const gclk_t *clk, uint32_t *m, uint32_t *d);

/**
 * @brief returs the current scaling factor for scalable clocks
 */
unsigned int gclk_get_current_factor(const gclk_t *gclk);


/**
 * @Note this only applies to clocks with flags.topology_flags.GCLK_STRICT_UPTREE_DEPENDENT flag set.
 * @return the factor of the clock which depends on the given state if another uptree clock
 */
unsigned int gclk_get_uptree_dependent_factor(const gclk_t *gclk, const clk_topology_entry_t *tree_confs, unsigned int conf_cnt);

/**
 * @brief  Get the input frequency of a clock.
 * @return The internal base frequency before any scaling is applied.
 * @note   For clocks that act as a source this returns their internal
 *         frequency property. As any non-source clock depends on its
 *         parent, this function returns the frequency of its current
 *         parent. */
uint32_t gclk_get_input_freq(const gclk_t *clk);

/* @todo below utility functionality will probably have to go to a separate file at some point */
unsigned int gclk_get_clk_subtree_max_depth(const gclk_t *clk, unsigned depth);
uint32_t gclk_get_max_topology_depth(void);
void gclk_print_topology(clk_topology_entry_t *topology_list, uint32_t depth);
/* @brief get the number of clock entries of the currently active topoogy of clk
 *
 * @return topology len including the given clock and the source
 * */
unsigned int gclk_get_current_topology_len(const gclk_t *clk);
uint32_t gclk_get_min_freq_using_topology(const gclk_t *gclk, clk_topology_entry_t *input_topology, uint32_t topology_len);
uint32_t gclk_get_next_higher_freq_using_topology(const gclk_t *gclk, clk_topology_entry_t *input_topology, uint32_t topology_len, uint32_t hz);
uint32_t gclk_get_max_freq_using_topology(const gclk_t *gclk, clk_topology_entry_t *input_topology, uint32_t topology_len);

uint32_t gclk_get_min_freq_of_current_topology(const gclk_t *gclk);
uint32_t gclk_get_max_freq_of_current_topology(const gclk_t *gclk);

/* topology[size - 1] must point to the source */
bool gclk_init_topology_freqs(clk_topology_entry_t *topology, int size);

void gclk_enable(const gclk_t *gclk);

void gclk_disable(const gclk_t *gclk);

bool gclk_is_enabled(const gclk_t *gclk);

/* @return the number of available distinct configurations
 * As of now this either reflects the number of available scaling factors or parents because
 * for now the base type gclk_t doesn't support composite clocks that support scaling and routing in the same instance.
 * (still possible via custom implementation by using driver specific mapping)*/
unsigned int gclk_config_cnt(const gclk_t *clk);

/* @return the number of available factor configs of this clock
 * */
unsigned int gclk_factor_cnt(const gclk_t *clk);

/* @return the number of available parent options
 * */
unsigned int gclk_parent_cnt(const gclk_t *clk);

unsigned int gclk_idx2factor(const gclk_t *gclk, unsigned int idx);

unsigned int gclk_factor2idx(const gclk_t *gclk, unsigned int factor);

const gclk_t* gclk_idx2parent(const gclk_t *gclk, unsigned int idx);

int gclk_parent2idx(const gclk_t *gclk, const gclk_t *parent);

/*
 * @param gclk    the clock to get the register value for
 * @param factor  the factor to get the register value for
 * @return        the normalized register value
 *
 * 'normalized' as in right aligned, i.e. not shifted to the actual position it
 * must be written to the register. */
uint32_t gclk_factor2regval(const gclk_t *gclk, uint32_t factor);

/*
 * @param gclk    the clock to get the factor for
 * @param regval  the inormalized register value to get the numerical value for
 * @return        the numerical factor
 *
 * 'normalized' as in right aligned, i.e. not shifted to the actual position it
 * must be written to the register. */
uint32_t gclk_regval2factor(const gclk_t *gclk, uint32_t regval);

/*
 * @param gclk    the clock to get the register value for
 * @param parent  the parent to get the register value for
 * @return        the normalized register value
 *
 * 'normalized' as in right aligned, i.e. not shifted to the actual position it
 * must be written to the register. */
uint32_t gclk_parent2regval(const gclk_t *gclk, const gclk_t *parent);

/*
 * @param gclk    the clock to get the parent for
 * @param regval  the inormalized register value to get the parent for
 * @return        the numerical factor
 *
 * 'normalized' as in right aligned, i.e. not shifted to the actual position it
 * must be written to the register. */
const gclk_t* gclk_regval2parent(const gclk_t *gclk, uint32_t regval);

unsigned int gclk_idx2parentregval(const gclk_t *gclk, unsigned int idx);

/* @param  clk  The clock to get the scaling factor for.*
 * @return the smallest number clk can be scaled by */
static inline unsigned int gclk_factor_min(const gclk_t *gclk) {
    return gclk_idx2factor(gclk, 0);
}

/* @param  clk  The clock to get the scaling factor for.*
 * @return the largest number clk can be scaled by */
static inline unsigned int gclk_factor_max(const gclk_t *clk) {
    /* TODO: add a flag for inverted order? */
    return gclk_idx2factor(clk, clk->flags.conf_cnt - 1);
}

/* @param[in]      clk     The clock to get the scaling factor for.
 * @param[in,out]  limits  Combined factor limit holding both, the min and max factor.
 */
static inline void gclk_get_factor_minmax(const gclk_t *clk, gclk_factor_limit_t *limits) {
  limits->min = gclk_factor_min(clk);
  limits->max = gclk_factor_max(clk);
}

/**
 * @brief get the available options for parents that can be configured
 *
 * @note  There are two cases to consider:
 *        (A): virtual/logical parent association (some node is the source, but it can not be changed, nor read from HW)
 *        (B): runtime-dynamic config (selecting one of multiple parents), can (and must be) read/written from/to HW
 *
 * @param[in] gclk   the clock you want to have the parent options for
 * @param[in] idx    number of the parent
 *
 * @return    the parent clock option at idx position (may be NULL if the respective config disconnects any parent)
 *            the given gclk if no more parents are available at idx
 */
const gclk_t *gclk_get_parent(const gclk_t *gclk, unsigned int idx);

const gclk_t *gclk_get_current_parent(const gclk_t *gclk);

int gclk_set_parent(const gclk_t *gclk, unsigned int idx);

/** @brief Set a target frequency on clocks that support that. */
uint32_t gclk_set_freq(const gclk_t *gclk, uint32_t freq);

/** @brief Set a scaling factor on clocks that support that. */
int gclk_set_factor(const gclk_t *gclk, uint32_t factor);

/**
 * @brief   get the accuracy of the clock
 * @note    this value may change when this clock is switched to another sources
 */
unsigned long gclk_get_accuracy(const gclk_t *gclk);

bool gclk_get_next_freq_conf_of_topology(clk_topology_entry_t *topology, int size);

uint32_t gclk_get_current_topology_config(clk_topology_entry_t *topology, uint32_t size);

typedef enum gclk_cmp_result {
    GCLK_CONF_INVALID, /*< the compared conf is not valid at all (in terms of the compare function) */
    GCLK_CONF_WORSE,   /*< the compared conf is valid but worse than the reference conf */
    GCLK_CONF_EQUAL,   /*< the compared conf is valid and as good as the reference conf */
    GCLK_CONF_BETTER,  /*< the compared conf is valid and better than the refernce conf */
    GCLK_CONF_BEST,    /*< the compared conf is known to be the best option */
} gclk_cmp_result_t;

/** @brief   A function prototype to compare two different topology configs. */
typedef gclk_cmp_result_t (*gclk_cmp_func_t)(clk_topology_entry_t *topo_best, size_t len1,
                                             clk_topology_entry_t *topo_cmp, size_t len2, void *arg);

typedef struct constrained_cmp_ctx {
    const gclk_t *constraint_clk;
    uint32_t     constraint_clk_freq;
    uint32_t     target_freq;
} gclk_constrained_cmp_ctx_t;

void gclk_print_topology_metadata(clk_topology_entry_t *t, int len);
uint32_t gclk_print_scale_freq(uint32_t val);
char *gclk_freq_scale_unit(uint32_t val);

/** Searches the maximum output frequency for a given clk and stores the first found topology in best_topology.
 *  The algorithm explores every topology that is able to drive clk.
 *  For each topology it bruteforce-tests all possible frequency-configurations of involved intermediate clock nodes.
 *  All possible parents, frequencies and constraints are considered.
 *  Exploration and testing adaptations always starts at the supplied leaf node (clk). Starting from the source is not
 *  feasible for various reasons.
 *  I.e. trying to find the max possible frequency may not resolve properly when starting from the source:
 *
 *  @note This is far from an optimal implementation becasue brute forcing can take quite some time with more complex
 *        clock trees as configuration possibilities can quickly expand beyond the order of 10k or even 100k.
 *        Solutions besides brute force could calculate viable solutions much more efficently, but then need at least
 *        access to internal state of clock nodes, and a way to express all constraints in a generic way.
 *        Though, this information is in many cases very implementation-, platform- or configuration-specific.
 *        And complex implementation specific details like configuration data, internal dependencies and constraints
 *        are often hard to model and encode in a completely implementation-agnostic way.
 *        The major benefit of brute forcing comes from the fact that no assumptions need to be made about internals
 *        of the clock tree and it's nodes internals.
 *        Brute forcing specific configurations allows to generically consider all constraints and configuration data
 *        by creating a significant runtime overhead.
 *        Another point why this is prefered: the results of this operation can easily be cached for later use.
 *
 *  @todo an optimization could first check if clk (and other intermediate clocks) have output constraints to limit
 *        possible configurations
 *
 *  @todo there is also potential for optimizing the resulting topology properties, e.g.:
 *        - reduce domain counts (only enable the least required domains)
 *        - reduce the frequency of higher order clocks
 *          (prefer nodes to use a lower frequency if the clock can be multiplied further down the tree)
 *  @todo refactor this to a more generic implementation that judges the derived topology configurations via a generic
 *        function pointer (i.e. a compare function that can be handed by the above layer)
 *
 *  @return
 *   GCLK_CONF_INVALID  if the compared conf is not valid at all (in terms of the compare function)
 *   GCLK_CONF_WORSE    if the compared conf is valid but worse than the reference conf
 *   GCLK_CONF_EQUAL    if the compared conf is valid and as good as the reference conf
 *   GCLK_CONF_BETTER   if the compared conf is valid and better than the refernce conf
 *
 */
gclk_cmp_result_t gclk_cmp_topology_for_max_leaf_freq(clk_topology_entry_t *topo_best, size_t len1, clk_topology_entry_t *topo_cmp, size_t len2, void *arg);
gclk_cmp_result_t gclk_cmp_topology_for_min_nz_leaf_freq(clk_topology_entry_t *topo_best, size_t len1, clk_topology_entry_t *topo_cmp, size_t len2, void *arg);
gclk_cmp_result_t gclk_cmp_topology_for_closest_leaf_freq(clk_topology_entry_t *topo_best, size_t len1, clk_topology_entry_t *topo_cmp, size_t len2, void *arg);
gclk_cmp_result_t gclk_cmp_topology_for_closest_leaf_freq_min_sum(clk_topology_entry_t *topo_best, size_t len1, clk_topology_entry_t *topo_cmp, size_t len2, void *arg);
gclk_cmp_result_t gclk_cmp_topology_for_closest_leaf_freq_max_sum(clk_topology_entry_t *topo_best, size_t len1, clk_topology_entry_t *topo_cmp, size_t len2, void *arg);
gclk_cmp_result_t gclk_cmp_topology_for_closest_leaf_freq_min_max(clk_topology_entry_t *topo_best, size_t len1, clk_topology_entry_t *topo_cmp, size_t len2, void *arg);
gclk_cmp_result_t gclk_cmp_topology_for_closest_leaf_freq_max_max(clk_topology_entry_t *topo_best, size_t len1, clk_topology_entry_t *topo_cmp, size_t len2, void *arg);
gclk_cmp_result_t gclk_cmp_topology_for_closest_constrained_leaf_freq(clk_topology_entry_t *topo_best, size_t len1, clk_topology_entry_t *topo_cmp, size_t len2, void *arg);
gclk_cmp_result_t gclk_cmp_topology_for_exact_leaf_freq(clk_topology_entry_t *topo_best, size_t len1, clk_topology_entry_t *topo_cmp, size_t len2, void *arg);

const gclk_t *gclk_get_child(const gclk_t *gclk, uint32_t child_idx);

uint32_t gclk_get_cnt(void);

const gclk_t *gclk_get(uint32_t idx);

unsigned int gclk_get_index(const gclk_t *gclk);

/* Helper function to update a specific part of a 32 bit register.
   The mask defines which bits will be updated. The value will be shifted up to the mask before writing */
static inline void gclk_reg_util_write_masked(volatile uint32_t *reg, uint32_t mask, uint32_t val)
{
    *reg = (*reg & ~mask) | (val << bitarithm_lsb(mask));
}

static inline uint32_t gclk_reg_util_read_masked(volatile uint32_t *reg, uint32_t mask)
{
    uint32_t cur_config_reg_val = (*reg & mask);
    /* normalize masked value according to mask */
    return cur_config_reg_val >> bitarithm_lsb(mask);
}

static inline void gclk_reg_util_set_mask(uint32_t volatile * const reg, uint32_t mask)
{
    *reg |= mask;
}

static inline void gclk_reg_util_clear_mask(volatile uint32_t * const reg, uint32_t mask)
{
    *reg &= (~mask);
}

static inline bool gclk_is_muxable(const gclk_t *clk) {
    return clk ? clk->flags.muxable : false;
}

static inline bool gclk_is_gateable(const gclk_t *clk) {
    return clk ? clk->flags.gateable : false;
}

static inline bool gclk_is_scalable(const gclk_t *clk) {
    return clk ? clk->flags.scaler_type != GCLK_NOSCALE : false;
}

static inline bool gclk_is_divider(const gclk_t *clk) {
    return clk ? clk->flags.scaler_type == GCLK_DIV : false;
}

static inline bool gclk_is_multiplier(const gclk_t *clk) {
    return clk ? clk->flags.scaler_type == GCLK_MUL : false;
}

static inline bool gclk_is_source(const gclk_t *clk) {
    /* NULL never has a parent so must be a source */
    return clk ? clk->flags.is_source : true;
}

/* TODO: below functions could go to a separate file for mapping helpers */
/* applicable for encoding via gclk_factor_mapping_t.gclk_reg_val_factor_lut_t (lut) */
uint32_t gclk_map_func_lut(const gclk_t *clk, unsigned int idx, bool to_regval);

/* applicable for encoding via gclk_factor_mapping_t.gclk_reg_val_ptr_lut_t (ptr_lut) */
uint32_t gclk_map_func_ptr_lut(const gclk_t *clk, unsigned int idx, bool to_regval);

/* applicable for encoding via gclk_factor_mapping_t.gclk_reg_val_cross_ref_luf_t (cross_ref)
 * conf may be NULL to lookup the current conf */
uint32_t gclk_map_func_uptree_cross_ref_luf(const gclk_t *clk, const clk_topology_entry_t *conf);

/* applicable for encoding via gclk_factor_mapping_t.gclk_range8_t
 * for ranges where the register value equals to the numerical value */
uint32_t gclk_map_func_regval_as_numval_range8(const gclk_t *clk, unsigned int idx, bool to_regval);

/* applicable for encoding via gclk_factor_mapping_t.gclk_range8_t
 * for ranges where the register value equals the index of the mapping entry */
uint32_t gclk_map_func_idx_as_regval_range8(const gclk_t *clk, unsigned int idx, bool to_regval);

/* applicable for encoding via gclk_factor_mapping_t.gclk_range16_t
 * If the register value equals to the numerical value */
uint32_t gclk_map_func_regval_as_numval_range16(const gclk_t *clk, unsigned int idx, bool to_regval);

/* applicable for encoding via gclk_factor_mapping_t.gclk_range16_t
 * for ranges where the register value equals the index of the mapping entry */
uint32_t gclk_map_func_idx_as_regval_range16(const gclk_t *clk, unsigned int idx, bool to_regval);

/* applicable for encoding via gclk_factor_mapping_t.uint8_t (list8) */
uint32_t gclk_map_func_list8(const gclk_t *clk, unsigned int idx, bool to_regval);

/* applicable for encoding via gclk_factor_mapping_t.uint16_t (list16) */
uint32_t gclk_map_func_list16(const gclk_t *clk, unsigned int idx, bool to_regval);

/* applicable for encoding via gclk_factor_mapping_t.uint32_t (fixed_factor) */
uint32_t gclk_map_func_fixed_factor(const gclk_t *clk, unsigned int idx, bool to_regval);

uint32_t gclk_map_parent_lut(const gclk_t *clk, const gclk_t **parent, unsigned int idx);

uint32_t gclk_map_parent_list(const gclk_t *clk, const gclk_t **parent, unsigned int idx);

/*
 * @brief sets up a gpio pin to output the clock signal
 *
 * @param clk The clock that will be routed to the pin
 * @param pin The pin to output the signal on
 *
 * @return ENABLE_PIN_OUTPUT_OK             on success
 * @return ENABLE_PIN_OUTPUT_INVALID_CLOCK  if clock can not be output
 * @return ENABLE_PIN_OUTPUT_INVALID_PIN    if clock can not be output on this pin
 * * */
int gclk_enable_pin_output(const gclk_t *clk, const gpio_t pin);

/* @return the overall count of possible configuration states the list of given clocks can be put into.
 *
 * The returned number considers all possible states that can be configured in theory. I.e., no physical or
 * logical constraints or other limits are considered that may apply due to inter-dependencies or other
 * restirction that occur at runtime. In some scenarios the number of actually feasible configurations may
 * be a lot lower - but never higher.
 * @param clks  pointer to an array of clock pointers
 * @param cnt   number of clock instances held in clks */
size_t gclk_get_factors_config_cnt(const gclk_t **clks, size_t cnt);

/* same as gclk_get_factors_config_cnt but with a topology as input */
size_t gclk_get_factors_config_cnt_from_topology(clk_topology_entry_t *topology, size_t len);

/* @return the overall count of possible topologies this clock can be driven by.
 *
 * The returned number considers all possible states that can be configured in theory. I.e., no physical or
 * logical constraints or other limits are considered that may apply due to inter-dependencies or other
 * restirction that occur at runtime. In some scenarios the number of actually feasible configurations may
 * be a lot lower - but never higher.
 * @param clk  pointer to the clock to get number of topologies for */
size_t gclk_get_topology_config_cnt(const gclk_t *clk);

/* Calculate for each clock in the topology the frequency given the configuration
 * parameters (factors).
 * @pre topology[0] holds the output clk and topology[len -1] holds the source */
void gclk_calculate_topology_config_freqs(clk_topology_entry_t *topology, size_t len);

/* @pre topology must conatin a valid configuration
 * @pre must refer to a valid conf_id (i.e. any value of 0 - gclk_get_factors_config_cnt)
 * @note the order how this iteration happens is defined by the topology. I.e., there is
 *       no guarantee that frequencies will change only in one direction for each step.
 *       Calling this for all possible configuration IDs will return all possible frequency
 *       configurations. Albeit no configuration is returned twice, the resulting frequency
 *       for multiple different configurations may still be equal.
 * return true if a new setting was possible
 *        false if the last configuration was already reached
 */
void gclk_advance_topology_to_next_frequency_setting(clk_topology_entry_t *topology, size_t len, size_t conf_id);

/* a basic helper to iterate all possible factor configurations of multiple sets */
void gclk_get_nth_factors_config(const gclk_t **clks, uint32_t *factors, size_t set_cnt, size_t n);

/* For a given topology config it returns the (leaf-unique) topology id for the leaf clock where (topology_conf[0] it the leaf) */
unsigned int gclk_topology2id(const clk_topology_entry_t *topology_conf, uint32_t topo_len);

uint32_t gclk_get_nth_config_equivalent_factor(const gclk_t **clks, size_t clks_cnt, size_t n);

/* @pre topology[0] must conatin the output clock
 * @param topology  The destination where the requested topology config will be written
 * @param max_len   Tat valid maximum number of entries topologie can hold
 * @param tid       Unique ID of the topology that is requested
 *
 * return the length of the new topology.
 */
unsigned int gclk_get_nth_topology(clk_topology_entry_t *topology, size_t max_len, size_t tid);

uint32_t gclk_get_factor_config_freq(uint32_t fi, uint32_t *mfacts, size_t mfact_cnt, uint32_t *dfacts, size_t dfact_cnt);

bool gclk_match_closest_full_iter(uint32_t fi, uint32_t fo,
                                 const gclk_t **mul_clks, size_t mul_clks_cnt, uint32_t *mfacts,
                                 const gclk_t **div_clks, size_t div_clks_cnt, uint32_t *dfacts);

static inline uint32_t gclk_abs_freq_diff(uint32_t a, uint32_t b) {
    if (a > b) {
        return a - b;
    }
    return b - a;
}

/* @param  topology  List of clocks that form the topology to adjust factors for.
 *                   topology[0] contains the output clock node that shall take Fo as frequency
 *                   topology[1] contains the parent of topology[0] and so forth.
 * @param  topo_len  the toplology length i.e., number of clocks in the given topology
 * @param  clks      pointer to where the references to all dividers will be stored.
 *                   must be big enough to store all divider clock references.
 *                   may be NULL to query the size before actually getting the references.
 *
 * @return the number of dividers within the given topology */
uint32_t gclk_get_dividers_from_topology(const gclk_t **topology, uint32_t topo_len, const gclk_t **div_clks);

/* @param  topology  List of clocks that form the topology to adjust factors for.
 *                   topology[0] contains the output clock node that shall take Fo as frequency
 *                   topology[1] contains the parent of topology[0] and so forth.
 * @param  topo_len  the toplology length i.e., number of clocks in the given topology
 * @param  clks      pointer to where the references to all multipliers will be stored.
 *                   must be big enough to store all multiplier clock references.
 *                   may be NULL to query the size before actually getting the references.
 *
 * @return the number of multipliers within the given topology */
uint32_t gclk_get_multipliers_from_topology(const gclk_t **topology, uint32_t topo_len, const gclk_t **mul_clks);

typedef bool (*gclk_factor_match_func_t)(uint32_t fi, uint32_t fo,
                                 const gclk_t **mul_clks, size_t mul_clks_cnt, uint32_t *mfacts,
                                 const gclk_t **div_clks, size_t div_clks_cnt, uint32_t *dfacts);

uint32_t gclk_match_freq_conf(clk_topology_entry_t *topology, uint32_t topo_len,
                              uint32_t f_in, uint32_t f_out_target,
                              gclk_factor_match_func_t match_op);

bool gclk_match_iter_mul_recurse_div(uint32_t fi, uint32_t fo,
                                 const gclk_t **mul_clks, size_t mul_clks_cnt, uint32_t *mfacts,
                                 const gclk_t **div_clks, size_t div_clks_cnt, uint32_t *dfacts);

bool gclk_match_iter_mul_factorize_div(uint32_t fi, uint32_t fo,
                                 const gclk_t **mul_clks, size_t mul_clks_cnt, uint32_t *mfacts,
                                 const gclk_t **div_clks, size_t div_clks_cnt, uint32_t *dfacts);

bool gclk_match_exact_full_iter(uint32_t fi, uint32_t fo,
                                 const gclk_t **mul_clks, size_t mul_clks_cnt, uint32_t *mfacts,
                                 const gclk_t **div_clks, size_t div_clks_cnt, uint32_t *dfacts);

/* @brief check if a change to a specific clock affects another clock
 **/
bool gclk_affected_by_change(const gclk_t *altered_clock, const gclk_t *affected_clock);

/* @brief check if a clock must be stopped before changing its configuration */
bool gclk_must_be_stopped_for_change(const gclk_t *clk);

/* @brief check if the parent of a clock must be stopped before changing its configuration */
bool gclk_parent_must_be_stopped_for_change(const gclk_t *clk);

/* @brief returns true if clk is directly or indirectly driven by src
 * @note does not check if the clock is actually gate by an intermediate clock.
 *       I.e. a clock is considered to be sources by another clock even if it currently is switched off */
bool gclk_is_sourced_by(const gclk_t *clk, const gclk_t *src);

/* @brief check if a clock is not useable as parent by other clocks
 *
 * @return true if no other clock can use this clock as parent
 *         false otherwise */
bool gclk_is_leaf(const gclk_t *clk);

/* @brief check if a clock is currently used as parent by other active clocks
 *
 * @return true if another active clock is currently sourced by this clock
 *         false otherwise */
bool gclk_is_used(const gclk_t *clk);

#define GCLK_REGVAL_AS_NUMVAL_RANGE8_STATIC_INIT(X) .base.factor_mapping.range8 = &(X), \
                                                    .base.flags.conf_cnt = (X).max - (X).min + 1U, \
                                                    .base.factor_map_op = gclk_map_func_regval_as_numval_range8

#define GCLK_IDX_AS_REGVAL_RANGE8_STATIC_INIT(X) .base.factor_mapping.range8 = &(X), \
                                                 .base.flags.conf_cnt = (X).max - (X).min + 1U, \
                                                 .base.factor_map_op = gclk_map_func_idx_as_regval_range8

#define GCLK_LIST8_STATIC_INIT(X) .base.factor_mapping.list8 = &(X[0]), \
                                  .base.flags.conf_cnt = ARRAY_SIZE(X), \
                                  .base.factor_map_op = gclk_map_func_list8

#define GCLK_LIST16_STATIC_INIT(X) .base.factor_mapping.list16 = &(X[0]), \
                                   .base.flags.conf_cnt = ARRAY_SIZE(X), \
                                   .base.factor_map_op = gclk_map_func_list16

#define GCLK_REGVAL_AS_NUMVAL_RANGE16_STATIC_INIT(X) .base.factor_mapping.range16 = &(X), \
                                                     .base.flags.conf_cnt = (X).max - (X).min + 1U, \
                                                     .base.factor_map_op = gclk_map_func_regval_as_numval_range16

#define GCLK_IDX_AS_REGVAL_RANGE16_STATIC_INIT(X) .base.factor_mapping.range16 = &(X), \
                                                  .base.flags.conf_cnt = (X).max - (X).min + 1U, \
                                                  .base.factor_map_op = gclk_map_func_idx_as_regval_range16

#define GCLK_PTR_LUT_STATIC_INIT(X) .base.factor_mapping.ptr_lut = &(X)[0], \
                                    .base.flags.conf_cnt = ARRAY_SIZE((X)), \
                                    .base.factor_map_op = gclk_map_func_ptr_lut

#define GCLK_FACTOR_LUT_STATIC_INIT(X) .base.factor_map_op       = gclk_map_func_lut,\
                                       .base.flags.conf_cnt      = ARRAY_SIZE(X),\
                                       .base.factor_mapping.lut  = &X[0]

#define GCLK_PARENT_LUT_STATIC_INIT(X) .base.parent_map_op       = gclk_map_parent_lut,\
                                       .base.flags.conf_cnt      = ARRAY_SIZE(X),\
                                       .base.parent_mapping.lut  = &X[0]

#define GCLK_FACTOR_CROSSREF_UPTREE_LUF_STATIC_INIT(X) .base.factor_mapping.cross_ref = &X,\
                                                       .base.flags.conf_cnt           = 1,\
                                                       .base.cross_ref_factor_map_op  = gclk_map_func_uptree_cross_ref_luf,\
                                                       .base.flags.topology_flags     = GCLK_STRICT_UPTREE_DEPENDENT


#ifdef __cplusplus
}
#endif


#endif /* GCLK_H */

/**
 * @}
 */
