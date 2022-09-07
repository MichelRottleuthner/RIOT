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
 * For higher level control and safe operation we refer to the @ref sys_gclk_manager instead,
 * which uses this API to to automatically set up specific valid configurations and performs
 * dynamic adaptations.
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
 *  - Additional properties / features to be implemented
 *    - low power capabilities (e.g., pm mode availability)
 *    - type of clock (internal/external, RC/Crystal)
 *    - get_accuracy: to return accuracy limits as derived from oscillator spec. (PPM)
 *      - methods to evaluate related effect of the topology.
 *    - trim operations to trim the clock by a given fraction e.g. +- N PPB
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

/**
 * @brief  Low-level interface to configure clock scale factors.
 *
 * To check whether this optional interface capability is supported by a specific
 * clock instance use @ref gclk_is_scalable().
 *
 * @detail This interface is meant to wrap the interaction with the hardware.
 *         No hardware specific knowledge on how to set up respective hardware
 *         state is required as a user of the API. However, it must be ensured
 *         by the caller that values handed to this interface are valid and
 *         applicable to the clock. There are several helper functions to query
 *         possible factor options, see @ref gclk_factor_cnt(),
 *         @ref gclk_idx2factor(), and @ref gclk_factor2idx().
 *
 * @note The interface can be used for both multipliers and dividers. The effect
 *       of the scaling operation then depends on the type of scaler, which
 *       can be checked by @ref gclk_is_divider() and @ref gclk_is_multiplier().
 *
 */
typedef struct gclk_scale_ops {
    /**
     * @brief Get currently configured scaling factor from the hardware.
     * @param clk  The clock whose scaling factor to get.
     * @return     The currently configured scaling factor as numerical value.
     */
    unsigned int (*get_factor)(const gclk_t *clk);

    /**
     * @brief Configure new scaling factor by writing to hardware
     * @param clk     The clock whose scaling factor to set.
     * @param factor  The new scaling factor as numerical value.
     * @pre   @p factor must be a valid value for clk.
     */
    void (*set_factor)(const gclk_t *clk, unsigned int factor);
} gclk_scale_ops_t;

/**
 * @brief  Low-level interface to configure clock muxing (i.e., routing).
 *
 * To check whether this optional interface capability is supported by a specific
 * clock instance use @ref gclk_is_muxable().
 *
 * @detail This interface is meant to wrap the interaction with the hardware.
 *         No hardware specific knowledge on how to set up respective hardware
 *         state is required as a user of the API. However, it must be ensured
 *         by the caller that values handed to this interface are valid and
 *         applicable to the clock. There are several helper functions to query
 *         possible parent options, see @ref gclk_parent_cnt(),
 *         @ref gclk_idx2parent(), and @ref gclk_parent2idx().
 */
typedef struct gclk_mux_ops {
    /**
     * @brief Get currently configured parent from the hardware.
     * @param clk  The clock whose parent to get.
     * @return     The currently configured parent as reference.
     */
    const gclk_t* (*get_parent)(const gclk_t *clk);

    /**
     * @brief Configure new parent by writing to hardware.
     * @param clk  The clock that the parent will be changed of.
     * @param idx  The new parent option as index of possible options.
     * @pre   @idx must be a valid value for clk.
     */
    void (*set_parent)(const gclk_t *clk, unsigned int idx);
} gclk_mux_ops_t;

/**
 * @brief  Low-level interface to configure clock gating (enable/disable control).
 *
 * To check whether this optional interface capability is supported by a specific
 * clock instance use @ref gclk_is_gateable().
 */
typedef struct gclk_gate_ops {
    /**
     * @brief Get enabled state from hardware.
     * @param clk  The clock whose enabled state to get.
     * @return     true if enabled.
     *             false if off (gated).
     */
    bool (*is_enabled)(const gclk_t *clk);

    /**
     * @brief Enable/disable a clock.
     * @param clk  The clock to enable/disable.
     * @param on   The new enabled state of @clk.
     */
    void (*enable)(const gclk_t *clk, bool on);
} gclk_gate_ops_t;

/**
 * @brief  Low-level interface to configure clock trimming.
 * @todo Not yet implemented, subject to change.
 */
typedef struct gclk_trim_ops {
    /**
     * @brief Get clock accuracy.
     * @param clk  The clock whose accuracy to get.
     * @return     Minimal guaranteed accuracy in PPM.
     */
    unsigned int (*get_accuracy)(const gclk_t *clk);

    /**
     * @brief Trim the clock speed.
     * @param clk  The clock to trim.
     * @param ppb  The trimming value in PPB (parts per billion).
     * @return     0 on success.
     */
    unsigned int (*trim)(const gclk_t *clk, unsigned int ppb);
} gclk_trim_ops_t;

/**
 * @brief  Common wrapper type for optional clock cpabilities.
 *
 * All capability interfaces are optional, and only those supported will
 * be referenced by a particular clock type driver. As they all have the
 * same size, the wrapping type can be used to store them in a flexible
 * length union array. Which op refers to which interface type is then
 * defined by combining a fixed order and dynamic feature availability.
 */
typedef union {
    const gclk_scale_ops_t scale_ops; /**< Clock scaler interface */
    const gclk_mux_ops_t   mux_ops;   /**< Clock mux interface */
    const gclk_gate_ops_t  gate_ops;  /**< Clock gate interface */
    const gclk_trim_ops_t  trim_ops;  /**< Clock trim interface */
} gclk_op_t;

/**
 * @brief  Number range with uint8_t values.
 */
typedef struct gclk_range8 {
  uint8_t min; /**< min. value. */
  uint8_t max; /**< max. value. */
} gclk_range8_t;

/**
 * @brief  Number range with uint16_t values.
 */
typedef struct gclk_range16 {
  uint16_t min; /**< min. value. */
  uint16_t max; /**< max. value. */
} gclk_range16_t;

/**
 * @brief  Number range with uint32_t values.
 */
typedef struct gclk_range32 {
  uint32_t min; /**< min. value. */
  uint32_t max; /**< max. value. */
} gclk_range32_t;

/**
 * @brief Numerical factor / register value LUT type.
 *
 * Used to store lookup tables which map between 16 bit numerical factors and
 * 8 bit register values.
 */
typedef struct gclk_reg_val_factor_lut {
    uint16_t factor;  /**< Integer factor the clock is scaled by. */
    uint8_t reg_val;  /**< Value that must be written to the configuration register
                           to set the above factor */
} gclk_reg_val_factor_lut_t;

/**
 * @brief Cross referenced value lookup function descriptor.
 *
 * This mapping can be used for clocks whose cnfiguration strictly depend
 * on another clocks configuration. I.e. one clock (A) statically multiplies/divides
 * by either X or Y, depending on the setting of another clock (B). */
typedef struct gclk_reg_val_cross_ref_luf {
    const gclk_t *ref_clk; /**< The reference clock (B), which another clock (A) strictly depends on. */
    /**
     * @brief The lookup function translating a specific config state to a numeric value.
     *
     * @param[in] clk   The (dependent) clock (A) whose value to get.
     * @param[in] conf  Either a specific configuration (of the other clock (B)), or
     *                  NULL to get the current state of clock (A), based on the *current* state (B).
     * @return The numerical scaling factor of @p clk.
     */
    uint32_t (* const luf)(const gclk_t *clk, const clk_topology_entry_t *conf);
} gclk_reg_val_cross_ref_luf_t;

/**
 * @brief Maps numerical values and (OTP) factory calibrated register contents.
 *
 * Some devices have special memory locations that carry one time programmable / factory-
 * calibrated configuration parameters. This mapping type can be used to map effective
 * numerical scaling factors to those respective memory locations.
 */
typedef struct gclk_reg_val_ptr_lut {
    uint16_t                 factor;       /**< Integer factor the clock is scaled by. */
    const volatile uint32_t* reg_val_ptr;  /**< Pointer to the memory that holds the respective
                                                value that must be written to the configuration
                                                register to setup the above factor. */
} gclk_reg_val_ptr_lut_t;

/**
 * @brief Container union to store either a reference to a mapping type or a fixed factor.
 *
 * Most of these types are used to encode mappings between numerical values and values
 * that must be written to a (specific location of) a configuration register.
 * These types encode (sets of) values, but the complete semantic of the mapping procedure
 * only forms when combined with a specific mapping function that slightly alter how the
 * value encodings are translated to register values. See for example
 * @ref gclk_map_func_regval_as_numval_range8() and
 * @ref gclk_map_func_idx_as_regval_range8().
 */
typedef union factor_mapping {
    const gclk_reg_val_factor_lut_t    *lut;       /**< 16 bit num. val. to 8 bit register content LUT. */
    const gclk_reg_val_ptr_lut_t       *ptr_lut;   /**< 16 bit num. val. to 32 bit memory location LUT. */
    const gclk_reg_val_cross_ref_luf_t *cross_ref; /**< Strictly dependent (RO) 32 bit num. val. cross-ref LUF. */
    const gclk_range8_t                *range8;    /**< 8 bit num. val. range. */
    const gclk_range16_t               *range16;   /**< 16 bit num. val. range. */
    const uint8_t                      *list8;     /**< 8 bit value list. */
    const uint16_t                     *list16;    /**< 16 bit value list. */
    const uint32_t                     fixed_factor; /**< (up to) 32 bit single fixed value. */
} gclk_factor_mapping_t;

/**
 * @brief Encodes the config register value for a specific parent selection.
 */
typedef struct {
    const gclk_t * const parent;    /**< Parent that is selected with this config value. */
    const uint32_t  config_reg_val; /**< Value read/written to respective config register
                                         to select this parent. */
} gclk_parent_config_lut_t;

/**
 * @brief Container union to store different parent option encodings.
 */
typedef union parent_mapping {
    /**
     * @brief A list of possible parents as clock instance pointers.
     */
    const gclk_t                   * const * plist;
    /**
     * @brief A reference to a LUT which maps parent options to regiter values.
     */
    const gclk_parent_config_lut_t * const lut;
} gclk_parent_config_mapping_t;

/**
 * @brief Generic low-level clock handle.
 *
 * This is the generic low-level handle which describes a clock with its properties and its capbilities.
 * Clock instances are expected to be provided as const instances via platform code. Clock references
 * are used to identify a clock, therefore it is **not allowed** to hand copies to the API.
 *
 * @todo
 *  - Factor_map_op and factor_mapping could be merged into one (flexibly)combined reference?
 *    - For a specific instance they are tied together anyway, so combining them could saves
 *      some memory if multiple clocks use the same get_factor_op + factor_mapping type.
 *    - The related performance impact of pointer indirection would need investigation.
 */
typedef struct gclk_base {
    /**
     * @brief separate low-level capability interface ops.
     *
     * Points to a list of all supported low-level interface capability functions.
     * The location of each particular function depends on the instance-specific
     * feature availability. The order is fixed as follows:
     * - 0: gclk_scale_ops_t
     * - 1: gclk_mux_ops_t
     * - 2: gclk_gate_ops_t
     * - 3: gclk_trim_ops_t
     *
     * If a capability is not supported by a clock, the respective op pointer is not present
     * and all greater indexes are effectively decremented. For example, a clock that is
     * only scalable ad gateable would populate the ops like that:
     * - 0: gclk_scale_ops_t
     * - 1: gclk_gate_ops_t
     *
     * This logic is hidden from the user for simplicity and to allow future changes
     * to this pattern or platform-specific optimizations for performance reasons.
     * Refer to corresponding utility functions (@ref gclk_get_scale_ops(),
     * @ref gclk_get_mux_ops(), @ref gclk_get_gate_ops(), and @ref gclk_get_trim_ops())
     * to access those function pointers in an implementation agnostic way.
     */
    const gclk_op_t *separated_ops;
    /**
     * @brief Configuration map ops.
     *
     * This is not part of the base type separated_ops to allow overwriting
     * it on a per-instace basis, as it is expected to vary a lot - even
     * between clocks that share the same separated_ops code.
     */
    union {
        /**
         * @brief Translate between config index and factor/register value.
         *
         * @param[in] clk     The clock instance to get the data from.
         * @param[in] idx     The (0-based) index of the wanted value (must be guaranteed to be valid!).
         * @param[in] regval  true when mapping idx to register value.
         *                    false when mapping idx to numerical value.
         */
        uint32_t (* const factor_map_op)(const gclk_t *clk, unsigned int idx, bool to_regval);

        /**
         * @brief Translate between config index and factor/register value.
         *
         * @param[in]     clk     The clock instance to get the data from.
         * @param[in,out] parent  Location where to store the parent reference.
         * @param[in]     idx     The (0-based) index that will be mapped to a parent and
         *                        register value (must be guaranteed to be valid!).
         *
         * @return        The register value corresponding to parent and index.
         */
        uint32_t (* const parent_map_op)(const gclk_t *clk, const gclk_t **parent, unsigned int idx);

        /**
         * @brief Cross-referenced strictly dependent (read-only) clock config.
         *
         * @note This only applies to read-only clocks that are strictily dependent on another
         *       clock further up in the tree.  Therefore the
         *       topology_flags.GCLK_STRICT_UPTREE_DEPENDENT must be set.
         *
         * @param[in]  clk     The clock instance to get the data from.
         * @param[in]  conf    The configuration of the cross-referenced clock this clock depends on
         *                     NULL to get the current config via the current config of the
         *                     cross-referenced clock.
         *
         * @return     The numeric factor.
         */
        uint32_t (* const cross_ref_factor_map_op)(const gclk_t *clk, const clk_topology_entry_t *conf);
    };

    /**
     * @breif Factor/parent mapping data reference.
     *
     * This field is used to store mapping information in one of the following ways:
     * - a private (HW driver implementation-specific) void pointer.
     * - a mapping type that provides information on possible scaling factors.
     * - a mapping type that provides information on possible parent selection options.
     *
     * @note A single instance may only use **ONE** of these options at a time,
     *       they are mutually exclusive! To avoid misconfigurations also refer
     *       to the static initialization macro helpers provided in this header.
     *       See e.g., @ref GCLK_LIST8_STATIC_INIT.
     */
    union {
        const void                         *private_data;
        const gclk_factor_mapping_t        factor_mapping;
        const gclk_parent_config_mapping_t parent_mapping;
    };

    /**
     * @brief Fixed input properties.
     *
     * There are two variants of fixed input for a clock, which mutually exlude
     * each other, hence sharing a union for both.
     * - Either a clock has a fixed input freq (only possible for a source).
     *   -> this implies it is not muxable (as in that case the frequency would
     *      dynamically depend on the selected parent.
     * - Or the clock has a fixed parent which makes the input frequency strictly depend
     *   on the parent output.
     *   - Technically, the frequency may still be fixed in that case,
     *     (if the parent frequency is fixed) but it keeps the explicit topological
     *     dependency information.
     */
     union {
        /**
         * @brief Fixed input frequency in case this is a root source.
         *
         * @note This **MUST** be set if this clock is a source, also see flags below.
         */
        const uint32_t fixed_input_freq;

        /**
         * @brief Fixed parent clock reference.
         *
         * @note This **MUST** be set for phone mascota non-muxable clock, also see flags below.
         */
        const gclk_t   *fixed_parent;
    };

    /**
     * @brief Unique human readable name that unabiguously identifies a clock instance.
     *
     * The name should be short and chosen as close as possible to names used in the
     * manufacturer specification, to make it easy to look up furhter HW-specific
     * information if needed.
     * This name schould only be used for user-oriented utilities and features used for
     * debugging/logging. Specifically, the name shall not be used by low-level code to
     * reference a clock, search a clock, or compare clock instances to each other.
     *
     * @todo Add compile-time configuration setting and preprocessor helpers to
     *       optionaly enable disable presence of this property (and referenced memory).
     */
    const char          *name;

    /**
     * @brief Clock properties and flags describing features and capabilities.
     */
    struct __attribute__((packed)) {
        enum gclk_clk_topology_flags topology_flags: 4;
        enum gclk_scaler_type        scaler_type   : 2;  /**< Type of scaler (mul/div/none). */
        unsigned int                 conf_cnt      : 16; /**< Number of configuration options available. */
        unsigned int                 scalable      : 1;  /**< 1 if the clock can scale its input frequency.
                                                              If set, #separated_ops **MUST** contain a
                                                              @ref gclk_scale_ops_t compatible reference.*/
        unsigned int                 muxable       : 1;  /**< 1 if the clock can be switched to different
                                                              clock inputs. If set, separated_ops **MUST**
                                                              contain a @ref gclk_mux_ops_t compatible reference.
                                                              If 0, @ref fixed_parent **MUST** hold a reference
                                                              to the parent feeding this this clock. */
        unsigned int                 gateable      : 1;  /**< 1 if the clock can be enabled/disabled.
                                                              If set, separated_ops **MUST** contain a
                                                              @ref gclk_gate_ops_t compatible reference.*/
        unsigned int                 trimmable     : 1;  /**< 1 if the clock can be trimmed. If set,
                                                              separated_ops **MUST** contain a
                                                              @ref gclk_trim_ops_t compatible reference.*/
        unsigned int                 is_source     : 1;  /**< 1 if the clock is a source.
                                                              If set, @ref fixed_input_freq must be provided.
                                                              Never used together with @ref muxable!. */
        /** @brief Space reserved for platform-specific flags. */
        unsigned int                 user_flags    : 32 - (1 + 1 + 1 + 1 + 1 + 16 + 2 + 4);
    } flags;
} gclk_t;

/**
 * @brief Generic clock type that includes data for primitive clock instances.
 *
 * This type extends the clock base type with fields that can be used to express
 * the low level register interface for many common primitive clock instances.
 * The @ref gclk_reg_ref_t type is used to encode common register access information
 * in a compact format. This clock type is meant to be used as the datatype for basic
 * gates, muxes and scalers. It is also possible to combine either a mux or a scaler
 * together with a gate in the same instance.
 * A muxable scaler on the other hand must be split into two separate instances that
 * allow either scaling or muxing.
 * In any case, custom clock types can be defined for clocks that are more complex
 * to interface.
 **/
typedef struct {
  gclk_t base; /**< Clock base type. */
  const gclk_reg_ref_t regref; /**< Compact register access descriptor. */
} gclk_basic_clock_t;

/**
 * @brief Get more specific basic clock from generic clock type.
 *
 * @pre @p clk must refer to a @ref gclk_basic_clock_t instance.
 *
 * @param[in] clk  The generic clock reference.
 * @return The reference to the basic clock type.
 */
static inline gclk_basic_clock_t *gclk_to_basic_clock_t(const gclk_t *clk)
{
    return container_of(clk, gclk_basic_clock_t, base);
}

/**
 * @brief Get the scaling interface pointer of @p clk
 *
 * @return  A reference to the instance-specific scaler interface if @p clk is scalable.
 *          NULL if @p clk is not scalable.
 */
const gclk_scale_ops_t *gclk_get_scale_ops(const gclk_t *clk);

/**
 * @brief Get the muxing interface pointer of @p clk
 *
 * @return  A reference to the instance-specific muxing interface if @p clk is muxable.
 *          NULL if @p clk is not muxable.
 */
const gclk_mux_ops_t *gclk_get_mux_ops(const gclk_t *clk);

/**
 * @brief Get the gating interface pointer of @p clk
 *
 * @return  A reference to the instance-specific gating interface if @p clk is gateable.
 *          NULL if @p clk is not gateable.
 */
const gclk_gate_ops_t *gclk_get_gate_ops(const gclk_t *clk);

/**
 * @brief Get the trim interface pointer of @p clk
 *
 * @return  A reference to the instance-specific trim interface if @p clk is trimmable.
 *          NULL if @p clk is not trimmable.
 */
const gclk_trim_ops_t *gclk_get_trim_ops(const gclk_t *clk);

/**
 * @brief Types of constraints a clock may be subject to.
 */
typedef enum {
    GCLK_ENSURE_MIN_FREQ,  /**< A lower bound for the frequency (incl. value). */
    GCLK_ENSURE_MAX_FREQ,  /**< An upper bound for the frequency (incl. value). */
    /* below constraints are not needed/implemented as of now. */
    /* ! @cond Doxygen_Suppress */
    GCLK_ENSURE_EXACT_FREQ,
    GCLK_ENSURE_MIN_FACT,
    GCLK_ENSURE_MAX_FACT,
    GCLK_ENSURE_EXACT_FACT,
    GCLK_ENSURE_EXACT_PARENT,
    GCLK_ENSURE_FIXED_CONF,
    /* ! @endcond */
} gclk_constraint_type_t;

/**
 * @brief Constraint descriptor that encodes limits for a specific clock instance.
 */
typedef struct {
    gclk_constraint_type_t type; /**< The type of constraint, defining what kind of limit
                                      the clock is subject to and what kind of data this
                                      descriptor refers to additionally. */
    const gclk_t *clk; /**< The clock that is subject to the constraint. */
    /**
     * @brief Additional data defining the value of the constraint. */
    union {
        uint32_t freq; /**< Frequency value. */
        uint32_t fact; /**< Scaling factor value. */
        const gclk_t *parent_clk; /**< Parent reference value. */
        const clk_topology_entry_t *confs; /**< Fully sepecified clock configuration state. */
    };
} gclk_freq_constraint_t;

/**
 * @brief Return values for @ref gclk_enable_pin_output function.
 */
enum {
    ENABLE_PIN_OUTPUT_OK = 0, /**< All good, pin now outputs the clock signal. */
    ENABLE_PIN_OUTPUT_INVALID_CLOCK, /**< The given clock can not be output on a pin. */
    ENABLE_PIN_OUTPUT_INVALID_PIN, /**< The clock can not be output on the given pin. */
};

/**
 * @brief A factional number in form of n/d.
 */
typedef struct {
    uint32_t n; /**< Numerator */
    uint32_t d; /**< Denominator */
} gclk_fraction_t;

/**
 * @brief A frequency limit with min/max value.
 */
typedef struct {
    uint32_t min; /**< Minimum value. */
    uint32_t max; /**< Maximum value. */
} gclk_freq_limit_t;

/**
 * @brief Factor limit with min/max value.
 */
typedef struct {
    uint32_t min; /**< Minimum value. */
    uint32_t max; /**< Maximum value. */
} gclk_factor_limit_t;

/**
 * @brief Initialize the generic clock configuration module.
 *
 * @return   0   On success.
 *           <0  On Error.
 */
int gclk_module_init(void);

/**
 * @brief Get the human readable name of a clock instance.
 *
 * @param[in]  clk  The clock instance.
 * @return  Pointer to constant zero-terminated string holidng the name.
 */
const char *gclk_get_name(const gclk_t *clk);

/**
 * @brief Get a clock via its human readable name.
 *
 * Shall only be used for testing and command line features, not for
 * finding specific instances in dynamic reconfiguration code. Name properties
 * are likely to be disabled/made optional in production code.
 *
 * @param[in]  name  The human readable name of the wanted clock instance.
 * @return     Pointer to the clock instance if found.
 *             Null if no clock with that name exists.
 */
const gclk_t* gclk_get_clk_by_name(const char *name);

/**
 * @brief Compare two fractions.
 *
 * @param[in] a  Reference to a properly initialized fraction a.
 * @param[in] a  Reference to a properly initialized fraction b.
 *
 * return <0 if @p a is smaller than @p b.
 * return 0  if @p a is equal to @p b.
 * return >0 if @p a is greater to @p b.
 */
int gclk_compare_fraction(gclk_fraction_t *a, gclk_fraction_t *b);

/**
 * @brief Check if a frequency is within a given limit.
 *
 * @param[in] freq   The frequency that shall be >= min and <= max of the given limit.
 * @param[in] limit  The limit.
 *
 * @retval   true   if @p freq is within @p limit.
 * @retval   false  if @p freq is lower or higher than @p limit.
 */
static inline bool gclk_freq_within_limit(uint32_t freq, const gclk_freq_limit_t *limit) {
    return (freq <= limit->max) && (freq >= limit->min);
}

/**
 * @brief Get the current frequency of this clock.
 *
 * This function accesses the hardware configuration of the given clock and all clocks
 * it depends on to get the currently configuraed frequency.
 *
 * @param[in] clk  The clock instance to get the frequency of.
 *
 * @return The current frequency in Hz.
 */
uint32_t gclk_get_current_freq(const gclk_t *clk);

/**
 * @brief Get an equivalent factor representation of uptree clock scalers.
 *
 * @param[in] clk      Clock instance for which the uptree config is read.
 * @param[in,out] m    Equivalent multiplier combining all uptree clocks (must be set to 1 before calling).
 * @param[in,out] d    Equivalent divisor combining all uptree clocks (must be set to 1 before calling).
 *
 * @return The frequency of the root source clock.
 */
uint32_t gclk_get_current_equivalent_uptree_factors(const gclk_t *clk, uint32_t *m, uint32_t *d);

/**
 * @brief Get the current scaling factor for a scalable clock.
 *
 * Accesses the current hardware configuration of the given clock instance to determine the
 * active frequency scaling factor.
 *
 * @param[in] clk  The clock to geth the scaling factor of.
 *
 * @return The current scaling factor for a scalable clock.
 *         1 for a clock that is not scalable.
 */
unsigned int gclk_get_current_factor(const gclk_t *clk);

/**
 * @brief Get an uptree-dependent scaling factor of a clock instance.
 *
 * Get the scaling factor of a clock that strictly depends on another clock up in the tree.
 * This function is for an a priori query of the state from a virtual representation of the tree.
 * I.e. this function answers the question of *how will the state of @p clk look like in case
 * the other clocks are configured like @p tree_confs*. To get the current factor, just use
 * @ref gclk_get_current_factor() as usual.
 *
 * @note this only applies to clocks with flags.topology_flags.GCLK_STRICT_UPTREE_DEPENDENT flag set.
 *
 * @param[in] clk          The clock to get the scaling factor of.
 * @param[in] tree_confs   A list of clock configuration states which describe (the relevant subset of)
 *                         the tree configuration at which the state of @p clk is requested.
 * @param[in] conf_cnt     Number of elements in @p tree_confs.
 *
 * @return The factor of the clock which depends on the given state of another uptree clock.
 */
unsigned int gclk_get_uptree_dependent_factor(const gclk_t *clk, const clk_topology_entry_t *tree_confs, unsigned int conf_cnt);

/**
 * @brief  Get the input frequency of a clock.
 *
 * @note   For clocks that act as a source this returns their internal source
 *         frequency property. For non-source clocks (where the input depends
 *         on the parent clock). This function returns the current frequency
 *         of its current parent.
 *
 * @param[in] clk   The clock to get the input frequency of.
 *
 * @return The internal base frequency of @p clk before any scaling is applied.
 */
uint32_t gclk_get_input_freq(const gclk_t *clk);

/**
 * @brief Get the maximum number of clocks in any source topology of the given clock.
 *
 * For all topology variants that are able to drive @p clk, this function determines
 * the longest one and returns its length (not including @p clk itself).
 *
 * @param[in] clk     The clock to get the max input topology length of.
 * @param[in] depth   The depth to add (for recursive calls).
 *
 * @return  The maximum length of any topology which may supply @p clk (excl. @p clk).
 */
unsigned int gclk_get_clk_subtree_max_depth(const gclk_t *clk, unsigned depth);

/**
 * @brief Get the maximum length of any topology chain in the clock tree.
 *
 * Determines the global maximum number of elements in any possible clock path from
 * leaf to source. This defines an upper limit of clock instance any topology chain
 * may include.
 *
 * @return  The maximum length of any possible topology chain (incl. root and leaf).
 */
uint32_t gclk_get_max_topology_depth(void);

/**
 * @brief Utility function to print a topology chain config.
 *
 * Prints the given topology chain. Elements must be in correct topological order
 * from leaf (topology_list[0]) to root source (topology_list[len-1]).
 *
 * @param[in] topology_list  Pointer to an array of clock topology config entries.
 * @param[in] len            Number of elements in @p topology_list.
 */
void gclk_print_topology(clk_topology_entry_t *topology_list, uint32_t len);

/**
 * @brief Get the number of clocks in the currently active topology chain of clk.
 *
 * @param[in]  clk   The clock to get the current topology chain length of.
 *
 * @return     Length of the topology chain that currently drives @p clk
 *             (incl. @p clk and the root source).
 */
unsigned int gclk_get_current_topology_len(const gclk_t *clk);

/**
 * @brief Get lowest possible frequency of a clock for its current topology.
 *
 * This is a utility function to roughly evaluate the lower limit of the frequency range
 * that @p clk may be configured to when using the current input topology. The function
 * determines this purely based on the available scaling factor ranges in the current
 * topology clock path. It explicitly does **NOT** evaluate whether that configuration
 * is valid or applicable at the moment.
 * The only information used of the current topology state is the clock routing/muxing
 * settings, whereas scaling factors are determined from all possible values of each
 * clock. I.e. the active scaling factors of the current topology config are effectively
 * ignored.
 *
 * @param[in]  clk             The clock to get the min frequency of.
 */
uint32_t gclk_get_min_freq_of_current_topology(const gclk_t *clk);

/**
 * @brief Get highest possible frequency of a clock for its current topology.
 *
 * This is a utility function to roughly evaluate the uper limit of the frequency range
 * that @p clk may be configured to when using the current input topology. The function
 * determines this purely based on the available scaling factor ranges in the current
 * topology clock path. It explicitly does **NOT** evaluate whether that configuration
 * is valid or applicable at the moment.
 * The only information used of the current topology state is the clock routing/muxing
 * settings, whereas scaling factors are determined from all possible values of each
 * clock. I.e. the active scaling factors of the current topology config are effectively
 * ignored.
 *
 * @param[in]  clk             The clock to get the max frequency of.
 */
uint32_t gclk_get_max_freq_of_current_topology(const gclk_t *gclk);

/**
 * @brief Enable a clock.
 *
 * This function enables a clock. After calling this, the clock will provide a clock singal
 * at its output.
 * If @p clk is not gateable (i.e., it can not be enabled/disabled itself), the request is
 * delegated up the tree to the first gateable clock, if possible.
 *
 * @param[in]  clk   The clock to enable.
 */
void gclk_enable(const gclk_t *clk);

/**
 * @brief Disable a clock.
 *
 * This function disables a clock. After calling this, the clock will not provide a clock
 * singal at its output.
 * If @p clk is not gateable (i.e., it can not be enabled/disabled itself), the request is
 * delegated up the tree to the first gateable clock, if possible.
 *
 * @param[in]  clk   The clock to disable.
 */
void gclk_disable(const gclk_t *clk);

/**
 * @brief Get the enabled state of a clock.
 *
 * If @p clk can be gated, it reports whether a clock is enabled or not.
 * If @p clk can **not** be gated, it reports the effective enabled state:
 * A non-gateable clock is always considered to be enabled, in that case
 * the request is delegated up the tree to the first gateable clock, if possible.
 *
 * @note The enabled state of one clock alone does not necesarily imply it is
 *       outputting a clock signal. The effective output signal also depends on
 *       the configuration of uptree clocks that @p clk is driven by.
 *
 * @param[in]  clk   The clock to disable.
 *
 * @return  true   If the clock is enabled.
 *          false  If the clock is disabled.
 */
bool gclk_is_enabled(const gclk_t *clk);

/**
 * @brief Get the number of possible configuration values of a clock.
 *
 * This either reflects the number of available scaling factors or parents because
 * for now the base type gclk_t doesn't support composite clocks that support scaling
 * and routing in the same instance.
 *
 * @note It is recommended to use the explicit functions @ref gclk_factor_cnt() and
 *       @ref gclk_parent_cnt() to ease future separation between both values.
 *
 * @return  The number of configurations @p clk can be set to.
 */
unsigned int gclk_config_cnt(const gclk_t *clk);

/**
 * @brief Get number of possible scaling factors the clock can be set to.
 *
 * @param[in] clk  The clock to get the factor count of.
 *
 * @return   The number of available factor configs of this clock.
 */
unsigned int gclk_factor_cnt(const gclk_t *clk);

/**
 * @brief Get number of possible parent selections the clock can be set to.
 *
 * @param[in] clk  The clock to get the parent count of.
 *
 * @return   The number of available parent configs of this clock.
 */
unsigned int gclk_parent_cnt(const gclk_t *clk);

/**
 * @brief Convert a factor index to the the respective numeric scaling factor value.
 *
 * Gets the nth possible scaling factor of @p clk. Use @ref gclk_factor_cnt() to
 * determine the number of possible values for a specific clock instance.
 * Valid indexes are from 0 to @ref gclk_factor_cnt() - 1.
 * This is the inverse operation of @ref gclk_factor2idx().
 *
 * @pre @p idx must be a valid factor index for @p clk.
 *
 * @param[in] clk  The clock to get the factor of.
 * @param[in] idx  The index of the factor (zero based).
 *
 * @return   The requested scaling factor of @p clk.
 *           1 for clocks that have no factor mapping.
 */
unsigned int gclk_idx2factor(const gclk_t *clk, unsigned int idx);

/**
 * @brief Convert a numeric scaling factor value to its respective index.
 *
 * This is the inverse operation of @ref gclk_idx2factor().
 *
 * @pre @p factor must be a valid factor for @p clk.
 *
 * @param[in] clk     The clock to get the scaling factor index of.
 * @param[in] factor  The factor of which to get the index of.
 *
 * @return   The index of the given factor for @p clk.
 *           An invalid index (>= value reported by @ref gclk_factor_cnt())
 *           if factor is not found.
 */
unsigned int gclk_factor2idx(const gclk_t *clk, unsigned int factor);

/**
 * @brief Convert a parent index to the the respective parent clock reference.
 *
 * Gets the nth possible parent selection of @p clk. Use @ref gclk_parent_cnt() to
 * determine the number of possible parents for a specific clock instance.
 * Valid indexes are from 0 to @ref gclk_parent_cnt() - 1.
 * This is the inverse operation of @ref gclk_parent2idx().
 *
 * @pre @p idx must be a valid parent index for @p clk.
 *
 * @param[in] clk  The clock to get the parent of.
 * @param[in] idx  The index of the parent (zero based).
 *
 * @return   A reference to the requested parent of @p clk.
 *           NULL for clocks that have no factor mapping.
 *           The fixed parent in case @p clk is not muxable.
 */
const gclk_t* gclk_idx2parent(const gclk_t *clk, unsigned int idx);

/**
 * @brief Convert a parent reference to its respective index.
 *
 * This is the inverse operation of @ref gclk_idx2parent().
 *
 * @pre @p parent must be a valid parent for @p clk.
 *
 * @param[in] clk     The clock to get the parent index of.
 * @param[in] parent  The parent of which to get the index of.
 *
 * @return   The index of the given parent for @p clk.
 *           An invalid index (-1)
 *           if parent is not found.
 *
 * @todo Unify how an invalid index is indicated between this
 *       function and @ref gclk_factor2idx().
 */
 int gclk_parent2idx(const gclk_t *clk, const gclk_t *parent);

/**
 * @brief Convert a scaling factor to its respective register value.
 *
 * Meant to be used for clock instances using the generic scaler
 * driver functions.
 *
 * @pre @p factor must be a valid factor for @p clk.
 *
 * @param clk     The clock to get the register value for.
 * @param factor  The factor to get the register value for.
 *
 * @return        The normalized register value. *Normalized* as in
 *                right aligned, i.e., not shifted to the actual position
 *                where it must be written to the register.
 */
uint32_t gclk_factor2regval(const gclk_t *clk, uint32_t factor);

/**
 * @brief Convert a configuration register value to the respective scaling factor.
 *
 * @pre @p regval must be a valid register value for @p clk.
 *
 * @param clk     The clock to get the factor for.
 * @param regval  The normalized register value to get the numerical value for.
 *                *Normalized* as in right aligned, i.e., not shifted to the
 *                actual position where it is located in the register.
 *
 * @return        The numerical scaling factor
 */
uint32_t gclk_regval2factor(const gclk_t *clk, uint32_t regval);

/**
 * @brief Convert a parent selection to the respective configuration register value.
 *
 * @pre @p parent must be a valid parent for @p clk.
 *
 * @param clk     The clock to get the register value for.
 * @param parent  The parent to get the register value for.
 *
 * @return        The normalized register value. *Normalized* as in right aligned,
 *                i.e., not shifted to the actual position where it is located in
 *                the register.
 *                0 if the parent option is not found.
 */
uint32_t gclk_parent2regval(const gclk_t *clk, const gclk_t *parent);

/**
 * @brief Convert a configuration register value to the respective parent reference.
 *
 * @pre @p regval must be a valid register value for @p clk.
 *
 * @param clk     The clock to get the parent register value for.
 * @param regval  The normalized register value to get the parent for
 *                *Normalized* as in right aligned, i.e. not shifted to the actual
 *                position it is located in the register.
 *
 * @return        The parent reference.
 *                NULL if the register value is not found.
 */
const gclk_t* gclk_regval2parent(const gclk_t *clk, uint32_t regval);

/**
 * @brief Convert a parent index to its respective configuration register value.
 *
 * @pre @p idx must be a valid parent idx for @p clk.
 *
 * @param clk     The clock to get the parent register value for.
 * @param idx     The index of the parent option (zero-based).
 *
 * @return        The parent reference.
 *                0 if the register value is not found or the clock has no
 *                factor mapping.
 */
unsigned int gclk_idx2parentregval(const gclk_t *clk, unsigned int idx);

/**
 * @brief Get the smallest possible scaling factor of a clock.
 *
 * @pre @p clk must be scalable.
 *
 * @param  clk  The clock to get the scaling factor for.
 * @return      The smallest factor @p clk can be scaled by.
 */
static inline unsigned int gclk_factor_min(const gclk_t *clk) {
    /* Static factor options must be defined lowest value first, therefore it is
     * enough to just return the first possible factor. Defining this function
     * separately, however, still allows to change this convention transparent
     * to the application, if needed. */
    return gclk_idx2factor(clk, 0);
}

/**
 * @brief Get the greatest possible scaling factor of a clock.
 *
 * @pre @p clk must be scalable.
 *
 * @param[in]  clk  The clock to get the scaling factor for.
 *
 * @return      The greatest factor @p clk can be scaled by.
 */
static inline unsigned int gclk_factor_max(const gclk_t *clk) {
    /* same as with gclk_factor_min(), this works because of the
     * enforced convention on how configuration options must be defined. */
    return gclk_idx2factor(clk, clk->flags.conf_cnt - 1);
}

/**
 * @brief Utility funciton to read min/max factor opption into a limit struct.
 *
 * @param[in]      clk     The clock to get the scaling factor for.
 * @param[in,out]  limits  Combined factor limit holding both, the min and max factor.
 */
static inline void gclk_get_factor_minmax(const gclk_t *clk, gclk_factor_limit_t *limits) {
  limits->min = gclk_factor_min(clk);
  limits->max = gclk_factor_max(clk);
}

/**
 * @brief Get the available options for parents that can be configured.
 *
 * @note  There are two cases to consider:
 *        (A): virtual/logical parent association. In this case, the parent can not
 *             be changed, this info is read from static clock data instead fo HW.
 *        (B): runtime-dynamic config. In this case one of multiple parents
 *             can be selected. This must be read from HW.
 *
 * @param[in] clk    The clock to get the possible parent selection for.
 * @param[in] idx    Index of the parent.
 *
 * @return    The parent clock option with the given @p idx.
 *            May be NULL if @p clk is NULL, @p clk is a source instance,
 *            or the given index explicitly connects the clock to no parent.
 *            For non-muxable clocks this always returns the fixed parent
 *            regardless of the @p idx value given.
 */
const gclk_t *gclk_get_parent(const gclk_t *clk, unsigned int idx);

/**
 * @brief Get the parent currently selected by a clock.
 *
 * @param[in] clk  The clock to get the current parent for.
 *
 * @return  The currently selected parent of @p clk if it is muxable.
 *          The fixed parent @p clk is always connected to if @p is not muxable.
 *          NULL if @p clk is a root source clock.
 */
const gclk_t *gclk_get_current_parent(const gclk_t *clk);

/**
 * @brief Set the active parent to the nth possible value (the given idx).
 *
 * @pre @p clk must be muxable.
 * @pre @p idx must be a valid index value for @p clk.
 *
 * @param[in] clk  The clock of which the parent shall be set.
 * @param[in] idx  The index of the parent to set.
 *
 * @return  0  on success..
 *          <0 on error.
 */
int gclk_set_parent(const gclk_t *clk, unsigned int idx);

/**
 * @brief Set a clock to the given frequency.
 *
 * This is the most basic function to set a frequency value of a clock.
 * It doesn't take into account any dependencies and constraints that might apply.
 * For more user-friendly / automatic functions for updating the frequency,
 * refer to respective functions of the @ref sys_gclk_manager instead.
 *
 * @pre @p clk must be scalable.
 * @pre @p freq must be a valid value that can be obtained with the current
 * configuration of @p clk by setting it to one of its possible sclaing factors.
 *
 * @param[in] clk   The clock to set the frequency for.
 * @param[in] freq  The wanted (valid!) frequency in Hz.
 *
 * @return The given frequency on success.
 *         0 on error.
 */
uint32_t gclk_set_freq(const gclk_t *clk, uint32_t freq);

/**
 * @brief Set a scaling factor of the given clock.
 *
 * This is the most basic function to set a scaling factor of a clock.
 * It doesn't take into account any dependencies and constraints that might apply.
 * For more user-friendly / automatic functions for updating the frequency,
 * refer to respective functions of the @ref sys_gclk_manager instead.
 *
 * @pre @p clk must be scalable.
 * @pre @p factor must be a valid factor for @p clk.
 *
 * @param[in] clk     The clock to set the factor for.
 * @param[in] factor  The wanted (valid!) scaling factor.
 *
 * @return 0 on success.
 *         <0 on error.
 */
int gclk_set_factor(const gclk_t *clk, uint32_t factor);

/**
 * @brief Get the current configuration state of the source topology of a clock.
 *
 * Reads the current hardware state of each clock up the tree from the given leaf.
 * The first element of @p topology will contain the state of the given @p leaf clock.
 * topology[1] will contain its parent configuration, and so forth.
 * At max @p size elements are read or up to reaching the clock source, whichever comes first.
 *
 * @param[in]  leaf      The clock to get the source topology data for.
 * @param[in]  topology  Location where to store the topology data.
 * @param[in]  size      Number of elements that can be stored in @p topology.
 *
 * @return Number of elements written to @p topology if there was enough storage to reach the source.
 *         0 if the source was not reached (@p topology now contains an incomplete topology).
 */
uint32_t gclk_get_current_topology_config(const gclk_t *leaf, clk_topology_entry_t *topology, uint32_t size);

/**
 * @brief Possible results of a topology configuration comparison.
 *
 * Used as return type for @ref gclk_cmp_func_t compare functions.
 */
typedef enum gclk_cmp_result {
    GCLK_CONF_INVALID, /*< the compared conf is not valid at all (in terms of the compare function) */
    GCLK_CONF_WORSE,   /*< the compared conf is valid but worse than the reference conf */
    GCLK_CONF_EQUAL,   /*< the compared conf is valid and as good as the reference conf */
    GCLK_CONF_BETTER,  /*< the compared conf is valid and better than the refernce conf */
    GCLK_CONF_BEST,    /*< the compared conf is known to be the best option */
} gclk_cmp_result_t;

/**
 * @brief   Compare function prototype for comparing two different topology configs.
 *
 * This kind of function can be used in an exploration phase to evaluate how well different
 * potential topology configurations to drive a specific clock fulfill a given optimization goal.
 * There is a selection of different predefined compare functions that aim e.g. for more accurate
 * frequency matching, reducing the maximum frequency in the tree, or reducing the overall power
 * consumption. It is also possible to define custom compare functions to favour special user
 * defined metrics.
 *
 * @note It is compare function specific (implementation defined) whether a compare function
 *       is applicable to different topology paths or only different topology factor configurations
 *       of the same topology path. Depending on the compare function it may also be needed to
 *       initialize the context @pr arg in a certain way. Therfore, follow the specific compare
 *       function documentation carefully.
 *
 * @param[in]      topo_best  The best topology found so far. topo_best[0].clk_freq must be initialized
 *                            to GCLK_INVALID_FREQ if no valid topology is known yet.
 * @param[in]      len1       Number of clock instances in @p topo_best.
 * @param[in]      topo_cmp   The topology to compare against @p topo_best.
 * @param[in]      len2       Number of clock instances in @p topo_cmp.
 * @param[in,out]  arg        Opaque (compare function specific) context pointer. May also be used
 *                            to hand out information from the compare function.
 *
 * @return  A comparison result indicating how both topology configs compare.
 */
typedef gclk_cmp_result_t (*gclk_cmp_func_t)(clk_topology_entry_t *topo_best, size_t len1,
                                             clk_topology_entry_t *topo_cmp, size_t len2, void *arg);

/**
 * @brief  Context structure for a constrained comparison.
 *
 * Meant to be used with @ref gclk_cmp_topology_for_closest_constrained_leaf_freq().
 */
typedef struct constrained_cmp_ctx {
    const gclk_t *constraint_clk; /**< Clock that shall be fixed to a given frequency. */
    uint32_t     constraint_clk_freq; /**< Fixed frequency of the constrained_clk. */
    uint32_t     target_freq; /**< Target frequency of the toppology leaf clock. */
} gclk_constrained_cmp_ctx_t;

/**
 * @brief  Scale a frequency value for better human readability.
 *
 * Can be combined with @ref gclk_freq_scale_unit() to get a human readable frequency string.
 * Only scales down exact multiples of MHz and KHz values. I.e.,
 * return 10 for 10000 Hz and 10 for 10000000 (@ref gclk_freq_scale_unit() takes care of
 * returning an appropriate Hz-suffix (kHz MHz). Values that are not exact MHz or kHz values
 * (e.g., 12500) are left unmodified to not loose information.
 *
 * @todo replace this with proper decimal formatting.
 *
 * @param[in] val  The frequency value to (potentially) scale down.
 *
 * @return The scaled down frequency for exact multiples of kHz and Mhz,
 *         The unmodivied value if @p val can not be scaled without information loss.
 */
uint32_t gclk_print_scale_freq(uint32_t val);

/**
 * @brief  Get the highest frequency unit suffix (MHz, kHz) that fits @p val exactly.
 *
 * @param[in] val  The frequency value to (potentially) scale down.
 *
 * @return "Mhz" or "kHz" depending to @p val.
 */
char *gclk_freq_scale_unit(uint32_t val);

/**
 * @brief Prefers higher leaf frequencies.
 *
 * A very basic compare function used for searching the maximum output frequency for a given clock.
 *
 * @note Also works with different topology paths (i.e., different source topology and
 *       different topology lengths).
 *
 * @param[in] topo_best  See interface definition for @ref gclk_cmp_func_t.
 * @param[in] len1       See interface definition for @ref gclk_cmp_func_t.
 * @param[in] topo_cmp   See interface definition for @ref gclk_cmp_func_t.
 * @param[in] arg        Not used for this compare function.
 *
 * @return  The comparison result, see @ref gclk_cmp_result_t.
 */
gclk_cmp_result_t gclk_cmp_topology_for_max_leaf_freq(clk_topology_entry_t *topo_best, size_t len1, clk_topology_entry_t *topo_cmp, size_t len2, void *arg);

/**
 * @brief Prefers lower leaf frequencies (while still being non-zero).
 *
 * A very basic compare function used for searching the smallest possible frequency that is
 * still above zero.
 *
 * @note Also works with different topology paths (i.e., different source topology and
 *       different topology lengths).
 *
 * @param[in] topo_best  See interface definition for @ref gclk_cmp_func_t.
 * @param[in] len1       See interface definition for @ref gclk_cmp_func_t.
 * @param[in] topo_cmp   See interface definition for @ref gclk_cmp_func_t.
 * @param[in] arg        Not used for this compare function.
 *
 * @return  The comparison result, see @ref gclk_cmp_result_t.
 */
gclk_cmp_result_t gclk_cmp_topology_for_min_nz_leaf_freq(clk_topology_entry_t *topo_best, size_t len1, clk_topology_entry_t *topo_cmp, size_t len2, void *arg);

/**
 * @brief Prefers closer matches of the given target frequency.
 *
 * A very basic compare function used for searching the closest possible match to a given target frequency.
 * The frequency difference is compared in absolute terms.
 *
 * @note Also works with different topology paths (i.e., different source topology and
 *       different topology lengths).
 *
 * @param[in] topo_best  See interface definition for @ref gclk_cmp_func_t.
 * @param[in] len1       See interface definition for @ref gclk_cmp_func_t.
 * @param[in] topo_cmp   See interface definition for @ref gclk_cmp_func_t.
 * @param[in] arg        Must contain the target frequency as value.
 *
 * @return  The comparison result, see @ref gclk_cmp_result_t.
 */
gclk_cmp_result_t gclk_cmp_topology_for_closest_leaf_freq(clk_topology_entry_t *topo_best, size_t len1, clk_topology_entry_t *topo_cmp, size_t len2, void *arg);

/**
 * @brief Prefers closer matches of the given target frequency, opting for a smaller frequency sum if possible.
 *
 * This compare function is a bit more complex as it involves two stages. Its first priority is matching the target
 * frequency as close as possible. The frequency difference is compared in absolute terms.
 * For two configurations that are equally close, the second priority is minimizing
 * the sum of all involved frequencies. This may be used as a heuristic approach to find configurations with lower
 * power consumption (at the same frequency). This effectively avoids selecting scaling factors that scale
 * up the frequency unnecessarily high at intermediate clock instances. This, however, can not guarantee to
 * yield the lowest possible power configuration in all cases, because the equivalent capacitance is typically not
 * uniform across all clock instances. For more accurate results a properly established power model of the platform
 * must be provided (see e.g., @ref gclk_manager_cmp_topology_closest_leaf_freq_pmin(), and respective model data
 * in the platform-specific manager config files).
 *
 * @note Also works with different topology paths (i.e., different source topology and
 *       different topology lengths).
 *
 * @param[in] topo_best  See interface definition for @ref gclk_cmp_func_t.
 * @param[in] len1       See interface definition for @ref gclk_cmp_func_t.
 * @param[in] topo_cmp   See interface definition for @ref gclk_cmp_func_t.
 * @param[in] arg        Must contain the target frequency as value.
 *
 * @return  The comparison result, see @ref gclk_cmp_result_t.
 */
gclk_cmp_result_t gclk_cmp_topology_for_closest_leaf_freq_min_sum(clk_topology_entry_t *topo_best, size_t len1, clk_topology_entry_t *topo_cmp, size_t len2, void *arg);

/**
 * @brief Prefers closer matches of the given target frequency, opting for a higher frequency sum if possible.
 *
 * Same as @ref gclk_cmp_topology_for_closest_leaf_freq_min_sum(), but favours a higher frequency sum instead.
 * This is intended to be used for evaluation purposes and practically showing the power consumption impact of
 * specific scaling factor selection methods. The assumption is that this function will most of the time yield
 * configurations with higher power consumption even if the exact same frequency is matched.
 *
 * @note Also works with different topology paths (i.e., different source topology and
 *       different topology lengths).
 *
 * @param[in] topo_best  See interface definition for @ref gclk_cmp_func_t.
 * @param[in] len1       See interface definition for @ref gclk_cmp_func_t.
 * @param[in] topo_cmp   See interface definition for @ref gclk_cmp_func_t.
 * @param[in] arg        Must contain the target frequency as value.
 *
 * @return  The comparison result, see @ref gclk_cmp_result_t.
 */
gclk_cmp_result_t gclk_cmp_topology_for_closest_leaf_freq_max_sum(clk_topology_entry_t *topo_best, size_t len1, clk_topology_entry_t *topo_cmp, size_t len2, void *arg);

/**
 * @brief Prefers closer matches of the given target frequency, opting for minimizing the max occurring frequency.
 *
 * This compare function is another alternative of a heuristic approach for searching a preferrably lower power
 * configuration when matching for the closest frequency. It does so by preferrably keeping the maximum
 * accurring frequency across all clocks in the topology as low as possible. Used for evaluation purposes,
 * to determine how well such simplified approaches compete to more sophisticated power models and against the
 * ground truth.
 *
 * @note Also works with different topology paths (i.e., different source topology and
 *       different topology lengths).
 *
 * @param[in] topo_best  See interface definition for @ref gclk_cmp_func_t.
 * @param[in] len1       See interface definition for @ref gclk_cmp_func_t.
 * @param[in] topo_cmp   See interface definition for @ref gclk_cmp_func_t.
 * @param[in] arg        Must contain the target frequency as value.
 *
 * @return  The comparison result, see @ref gclk_cmp_result_t.
 */
gclk_cmp_result_t gclk_cmp_topology_for_closest_leaf_freq_min_max(clk_topology_entry_t *topo_best, size_t len1, clk_topology_entry_t *topo_cmp, size_t len2, void *arg);

/**
 * @brief Prefers closer matches of the given target frequency, opting for maximizing the max occurring frequency.
 *
 * Same as @ref gclk_cmp_topology_for_closest_leaf_freq_min_max(), but favours a max frequency sum instead
 * to compare againt the opposite extreme.
 *
 * @note Also works with different topology paths (i.e., different source topology and
 *       different topology lengths).
 *
 * @param[in] topo_best  See interface definition for @ref gclk_cmp_func_t.
 * @param[in] len1       See interface definition for @ref gclk_cmp_func_t.
 * @param[in] topo_cmp   See interface definition for @ref gclk_cmp_func_t.
 * @param[in] arg        Must contain the target frequency as value.
 *
 * @return  The comparison result, see @ref gclk_cmp_result_t.
 */
gclk_cmp_result_t gclk_cmp_topology_for_closest_leaf_freq_max_max(clk_topology_entry_t *topo_best, size_t len1, clk_topology_entry_t *topo_cmp, size_t len2, void *arg);

/**
 * @brief Prefers closer matches of the given target frequency, but locks a specific clock to a given frequency.
 *
 * This compare function flags all configurations as invalid that are not able to drive the constrained clock
 * exactly at its specified frequency. Out of the configurations that fulfill this constraint, the one with
 * the closest leaf frequency match is preferred. The constrained clock must be part of the topology,
 * otherwise it also considered invalid.
 *
 * @todo The constraint handling can be extended and generalized:
 *       - Attatching a flexible list of more than one constaint.
 *       - Adding more expressive constraint types (like X <= f < Y)
 *       - Allowing to specify constraints on clocks that are not (directly) included
 *         in the topology (but still depend on it).
 *
 * @note Also works with different topology paths (i.e., different source topology and
 *       different topology lengths).
 *
 * @param[in] topo_best  See interface definition for @ref gclk_cmp_func_t.
 * @param[in] len1       See interface definition for @ref gclk_cmp_func_t.
 * @param[in] topo_cmp   See interface definition for @ref gclk_cmp_func_t.
 * @param[in] arg        Must contain a pointer to a properly initialized @ref gclk_constrained_cmp_ctx_t.
 *
 * @return  The comparison result, see @ref gclk_cmp_result_t.
 */
gclk_cmp_result_t gclk_cmp_topology_for_closest_constrained_leaf_freq(clk_topology_entry_t *topo_best, size_t len1, clk_topology_entry_t *topo_cmp, size_t len2, void *arg);

/**
 * @brief Accepts only exact leaf frequency matches.
 *
 * A very basic compare function (similar to @ref gclk_cmp_topology_for_closest_leaf_freq())
 * but flags all configs as invalid which are not able to match the target frequency exactly.
 *
 * @note Also works with different topology paths (i.e., different source topology and
 *       different topology lengths).
 *
 * @param[in] topo_best  See interface definition for @ref gclk_cmp_func_t.
 * @param[in] len1       See interface definition for @ref gclk_cmp_func_t.
 * @param[in] topo_cmp   See interface definition for @ref gclk_cmp_func_t.
 * @param[in] arg        Must contain the target frequency as value.
 *
 * @return  The comparison result, see @ref gclk_cmp_result_t.
 */
gclk_cmp_result_t gclk_cmp_topology_for_exact_leaf_freq(clk_topology_entry_t *topo_best, size_t len1, clk_topology_entry_t *topo_cmp, size_t len2, void *arg);

/**
 * @brief Get nth child of a clock.
 *
 * Utility function to iterate through all children of a clock.
 * The API does not guarantee a particular order, however, the result is deterministic
 * for a particular configuration of the clock tree being active. I.e., with the same
 * clock tree setup a given index will deterministically pint to the same clock.
 * If n clocks use @p clk as parent, this function will return a child clock for all index
 * values between 0 and n-1.
 *
 * @param[in]  clk        Reference to the clock instance to get a child of.
 * @param[in]  child_idx  Index identifying a child of @p clk.
 *
 * @return Reference to the nth child clock of @p clk.
 *         Null if no child with that index exists.
 */
const gclk_t *gclk_get_child(const gclk_t *clk, uint32_t child_idx);

/**
 * @brief Get number of clock instances available on this platform.
 *
 * The returned number refers to the total number of all available clock instances.
 * This number includes all sources, intermediate scalers, muxes and gates. Specifically,
 * this number does not refer to the number of distinct clock *sources*! Use
 * @ref gclk_is_source() and the like to get such meta data.
 *
 * @return Number of distinct clock instances available.
 */
uint32_t gclk_get_cnt(void);

/**
 * @brief Get a clock reference by its unique index.
 *
 * The index value for a clock is fixed and will never change during runtime.
 * Yet, using the index to access a particular clock instance must be done with caution as
 * future changes to the platforms clock model may change this association.
 *
 * @param[in] idx    The index of the clock. Valid values are
 *                   between 0 and @ref gclk_get_cnt()-1.
 *
 * @return A reference to the clock instance.
 */
const gclk_t *gclk_get(uint32_t idx);

/**
 * @brief Get the unique index for a given clock reference.
 *
 * Inverse function of @ref gclk_get().
 *
 * @param[in] clk  The clock reference.
 *
 * @return  The unique index of the given clock.
 */
unsigned int gclk_get_index(const gclk_t *gclk);

/**
 * @brief Write a value to a specific section of a register.
 *
 * Helper function to update a specific part of a 32 bit register.
 * The value is treated right aligned (normalized to the value it would have if
 * the register was only the size of the mask) and will also be shifted!
 *
 * @param[in] reg   Register address to write to.
 * @param[in] mask  Mask that indicates which bits to update.
 * @param[in] val   Value to write (LSB will be shifted to lowest bit set in @p mask).
 */
static inline void gclk_reg_util_write_masked(volatile uint32_t *reg, uint32_t mask, uint32_t val)
{
    *reg = (*reg & ~mask) | (val << bitarithm_lsb(mask));
}

/**
 * @brief Read a specific section of a register.
 *
 * Helper function to read a specific part of a 32 bit register.
 * The mask indicates which section to return. The value will be returned as
 * normalized value, i.e., the value will be shifted down so its LSB sits at bit 0.
 *
 * @param[in] reg   Register address to read from.
 * @param[in] mask  Mask that indicates which bits to read.
 *
 * @return The normalized value of the register section.
 */
static inline uint32_t gclk_reg_util_read_masked(volatile uint32_t *reg, uint32_t mask)
{
    uint32_t cur_config_reg_val = (*reg & mask);
    /* normalize masked value according to mask */
    return cur_config_reg_val >> bitarithm_lsb(mask);
}

/**
 * @brief Set masked bits of a register to 1.
 *
 * @param[in] reg   Register address to write to.
 * @param[in] mask  Mask indicating the bits to be set to 1.
 */
static inline void gclk_reg_util_set_mask(uint32_t volatile * const reg, uint32_t mask)
{
    *reg |= mask;
}

/**
 * @brief Clear masked bits of a register.
 *
 * @param[in] reg   Register address to write to.
 * @param[in] mask  Mask indicating the bits to be set to 0.
 */
static inline void gclk_reg_util_clear_mask(volatile uint32_t * const reg, uint32_t mask)
{
    *reg &= (~mask);
}

/**
 * @brief Check if a clock instance is muxable.
 *
 * A muxable clock can set its input to different parent clocks. Therefore,
 * it must support the optional @ref gclk_mux_ops_t interface capability.
 * The number of possible parents and the actual parent instances can be read
 * via @ref gclk_parent_cnt() and @ref gclk_get_parent() respectively.
 *
 * @param[in] clk  The clock instance.
 *
 * @return  true if the clock is muxable.
 *          false if the clock is not muxable.
 */
static inline bool gclk_is_muxable(const gclk_t *clk) {
    return clk ? clk->flags.muxable : false;
}

/**
 * @brief Check if a clock instance is gateable.
 *
 * A gateable clock can enable/disable forwarding of its input clock signal, or,
 * in case of a clock source is may enable/disable generation of a clock signal.
 * Therefore, it must support the optional @ref gclk_gate_ops_t interface capability.
 *
 * @param[in] clk  The clock instance.
 *
 * @return  true if the clock is gateable.
 *          false if the clock is not gateable.
 */
static inline bool gclk_is_gateable(const gclk_t *clk) {
    return clk ? clk->flags.gateable : false;
}

/**
 * @brief Check if a clock instance is scalable.
 *
 * A scalable clock can apply a scaling factor to a clock signal to modify
 * its input clock before forwarding it. Therefore, it must support the
 * optional @ref gclk_scale_ops_t interface capability.
 *
 * @param[in] clk  The clock instance.
 *
 * @return  true if the clock is scalable.
 *          false if the clock is not scalable.
 */
static inline bool gclk_is_scalable(const gclk_t *clk) {
    return clk ? clk->flags.scaler_type != GCLK_NOSCALE : false;
}

/**
 * @brief Check if a clock instance is a divider.
 *
 * A divider clock may scale the frequency fed to it by a given set of
 * divider scaling values. This implies @ref gclk_is_scalable() will be
 * true for the same instance.
 *
 * @param[in] clk  The clock instance.
 *
 * @return  true if the clock is a divider.
 *          false if the clock is not divider.
 */
static inline bool gclk_is_divider(const gclk_t *clk) {
    return clk ? clk->flags.scaler_type == GCLK_DIV : false;
}

/**
 * @brief Check if a clock instance is a multiplier.
 *
 * A multiplier clock may scale the frequency fed to it by a given set of
 * multiplier values. This implies @ref gclk_is_scalable() will be
 * true for the same instance.
 *
 * @param[in] clk  The clock instance.
 *
 * @return  true if the clock is a multiplier.
 *          false if the clock is not multiplier.
 */
static inline bool gclk_is_multiplier(const gclk_t *clk) {
    return clk ? clk->flags.scaler_type == GCLK_MUL : false;
}

/**
 * @brief Check if a clock instance is a clock soure.
 *
 * A clock source has no further parents and forms the root of one subtree of
 * the whole clock network.
 *
 * @param[in] clk  The clock instance.
 *
 * @return  true if the clock is a source.
 *          false if the clock is not source.
 */
static inline bool gclk_is_source(const gclk_t *clk) {
    /* NULL never has a parent so must be a source */
    return clk ? clk->flags.is_source : true;
}

/**
 * @brief Mapping function for values encoded as @ref gclk_reg_val_factor_lut_t.
 *
 * Maps the index of a configuration option either to its numeric or register value.
 * See also @ref gclk_factor_mapping_t.
 *
 * @param[in] clk        Clock instance to map the value for.
 * @param[in] idx        Zero based index (number of value pair) to map the value for.
 * @param[in] to_regval  Mapping direction (true: idx -> reg.value, false: idx -> numeric value).
 *
 * return The register value if @p to_regval is true.
 *        The numerical value if @p to_regval is false.
 */
uint32_t gclk_map_func_lut(const gclk_t *clk, unsigned int idx, bool to_regval);

/**
 * @brief Mapping function for values encoded as @ref gclk_reg_val_ptr_lut_t.
 *
 * Maps the index of a configuration option either to its numeric or register value.
 * See also @ref gclk_factor_mapping_t.
 *
 * @param[in] clk        Clock instance to map the value for.
 * @param[in] idx        Zero based index (number of value pair) to map the value for.
 * @param[in] to_regval  Mapping direction (true: idx -> reg.value, false: idx -> numeric value).
 *
 * return The register value if @p to_regval is true.
 *        The numerical value if @p to_regval is false.
 */
uint32_t gclk_map_func_ptr_lut(const gclk_t *clk, unsigned int idx, bool to_regval);

/**
 * @brief Mapping function for values encoded as @ref gclk_reg_val_cross_ref_luf_t.
 *
 * Special mapping function that does not map operate on specific register values but the
 * logical configuration state of another clock instance. The function translates the
 * (dependent) state by deriving it form the virtual representation of the sate it
 * depends on.
 * Alternatively, if no state is provided, it returns the currently active state
 * (which is not possible with other mapping functions).
 * See also @ref gclk_factor_mapping_t.
 *
 * @param[in] clk     Clock instance to map the value for.
 * @param[in] conf    State of the clock that the state of @p clk depends on.
 *
 * return The numerical value.
 */
uint32_t gclk_map_func_uptree_cross_ref_luf(const gclk_t *clk, const clk_topology_entry_t *conf);

/**
 * @brief Mapping function for values encoded as @ref gclk_range8_t.
 *
 * Can be applied for value ranges where the register value also implicitly encodes
 * the numerical value.
 * See also @ref gclk_factor_mapping_t.
 *
 * @param[in] clk        Clock instance to map the value for.
 * @param[in] idx        Zero based index (number of value pair) to map the value for.
 * @param[in] to_regval  Mapping direction (true: idx -> reg.value, false: idx -> numeric value).
 *
 * return The register value if @p to_regval is true.
 *        The numerical value if @p to_regval is false.
 */
uint32_t gclk_map_func_regval_as_numval_range8(const gclk_t *clk, unsigned int idx, bool to_regval);

/**
 * @brief Mapping function for values encoded as @ref gclk_range8_t.
 *
 * Can be applied for value ranges where the index also implicitly encodes
 * the register value.
 * See also @ref gclk_factor_mapping_t.
 *
 * @param[in] clk        Clock instance to map the value for.
 * @param[in] idx        Zero based index (number of value pair) to map the value for.
 * @param[in] to_regval  Mapping direction (true: idx -> reg.value, false: idx -> numeric value).
 *
 * return The register value if @p to_regval is true.
 *        The numerical value if @p to_regval is false.
 */
uint32_t gclk_map_func_idx_as_regval_range8(const gclk_t *clk, unsigned int idx, bool to_regval);

/**
 * @brief Mapping function for values encoded as @ref gclk_range16_t.
 *
 * Can be applied for value ranges where the register value also implicitly encodes
 * the numerical value.
 * See also @ref gclk_factor_mapping_t.
 *
 * @param[in] clk        Clock instance to map the value for.
 * @param[in] idx        Zero based index (number of value pair) to map the value for.
 * @param[in] to_regval  Mapping direction (true: idx -> reg.value, false: idx -> numeric value).
 *
 * return The register value if @p to_regval is true.
 *        The numerical value if @p to_regval is false.
 */
uint32_t gclk_map_func_regval_as_numval_range16(const gclk_t *clk, unsigned int idx, bool to_regval);

/**
 * @brief Mapping function for values encoded as @ref gclk_range16_t.
 *
 * Can be applied for value ranges where the index also implicitly encodes
 * the register value.
 * This is a 16 bit variant of @ref gclk_map_func_idx_as_regval_range8().
 * See also @ref gclk_factor_mapping_t.
 *
 * @param[in] clk        Clock instance to map the value for.
 * @param[in] idx        Zero based index (number of value pair) to map the value for.
 * @param[in] to_regval  Mapping direction (true: idx -> reg.value, false: idx -> numeric value).
 *
 * return The register value if @p to_regval is true.
 *        The numerical value if @p to_regval is false.
 */
uint32_t gclk_map_func_idx_as_regval_range16(const gclk_t *clk, unsigned int idx, bool to_regval);

/**
 * @brief Mapping function for values encoded as @c uint8_t array.
 *
 * Can be applied for value lists where the index also implicitly encodes
 * the register value, whereas the list itself contains the numerical value.
 *
 * @param[in] clk        Clock instance to map the value for.
 * @param[in] idx        Zero based index (number of value pair) to map the value for.
 * @param[in] to_regval  Mapping direction (true: idx -> reg.value, false: idx -> numeric value).
 *
 * return The register value if @p to_regval is true.
 *        The numerical value if @p to_regval is false.
 */
uint32_t gclk_map_func_idx_as_regval_list8(const gclk_t *clk, unsigned int idx, bool to_regval);

/**
 * @brief Mapping function for values encoded as @c uint16_t array.
 *
 * Can be applied for value lists where the index also implicitly encodes
 * the register value, whereas the list itself contains the numerical value.
 * This is a 16 bit variant of @ref gclk_map_func_idx_as_regval_list8().
 *
 * @param[in] clk        Clock instance to map the value for.
 * @param[in] idx        Zero based index (number of value pair) to map the value for.
 * @param[in] to_regval  Mapping direction (true: idx -> reg.value, false: idx -> numeric value).
 *
 * return The register value if @p to_regval is true.
 *        The numerical value if @p to_regval is false.
 */
uint32_t gclk_map_func_idx_as_regval_list16(const gclk_t *clk, unsigned int idx, bool to_regval);

/**
 * @brief Mapping function wrapper for a fixed 32 bit value.
 *
 * This is a dummy mapping function that simply returns the fixed numerical value.
 * Strictly sticking to the pattern of a mapping function allows keeping the calling
 * code free of conditional code. A fixed factor does not require setting a register
 * value and has only one possible value, both, @p idx and @p to_regval are ignored
 * and the function always returns the numerical value.
 *
 * @param[in] clk        Clock instance to map the value for.
 * @param[in] idx        Ignored.
 * @param[in] to_regval  Ignored.
 *
 * return The numerical value.
 */
uint32_t gclk_map_func_fixed_factor(const gclk_t *clk, unsigned int idx, bool to_regval);

/**
 * @brief Mapping function for parent and register value encoded as lookup table (@ref gclk_parent_config_lut_t).
 *
 * Can be applied for a lookup table that contains an arbitrary register value for each parent.
 *
 * @param[in] clk      Clock instance to map the value for.
 * @param[in] parent   Location where to store the parent reference.
 * @param[in] idx      Parent index.
 *
 * return The register value to select the parent with the given index.
 */
uint32_t gclk_map_parent_lut(const gclk_t *clk, const gclk_t **parent, unsigned int idx);

/**
 * @brief Mapping function for parent and register value encoded as simple list of @ref gclk_t references.
 *
 * Can be applied for an array that contains parent references where the position (index) in the array
 * also implicitly encodes the respective register value.
 *
 * @param[in] clk      Clock instance to map the value for.
 * @param[in] parent   Location where to store the parent reference.
 * @param[in] idx      Parent index.
 *
 * return The register value to select the parent with the given index.
 */
uint32_t gclk_map_parent_list(const gclk_t *clk, const gclk_t **parent, unsigned int idx);

/**
 * @brief Set up a GPIO pin to output the clock signal.
 *
 * This function does **not** automatically modify the topology configuration to try
 * routing the signal of arbitrary clocks to the pin output. Therfore, only clocks
 * that directly support pin output shall be handed to this function. Yet, the
 * standard configuration functions can of course still be used to explicitly set
 * up aspecific topology to feed the clock instance that supports the pin output.
 *
 * @param  clk   The clock that will be routed to the pin.
 * @param  pin   The pin to output the signal on.
 *
 * @return ENABLE_PIN_OUTPUT_OK             On success.
 * @return ENABLE_PIN_OUTPUT_INVALID_CLOCK  If clock can not be output.
 * @return ENABLE_PIN_OUTPUT_INVALID_PIN    If clock can not be output on this pin.
 */
int gclk_enable_pin_output(const gclk_t *clk, const gpio_t pin);

/**
 * @brief Get combined count of possible factor configs for a list of clocks.
 *
 * The returned number considers all possible states that can be configured in theory. I.e., no physical or
 * logical constraints or other limits are considered that may apply due to inter-dependencies or other
 * restirction that occur at runtime. In some scenarios the number of actually feasible configurations may
 * be a lot lower - but never higher.
 * Same as @ref gclk_get_factors_config_cnt_from_topology() but takes a list of clock references
 * instead of a topology config.
 *
 * @param[in]  clks   Pointer to an array of clock pointers.
 * @param[in]  cnt    Number of clock instances held in clks.
 *
 * @return Number of factor combinations possible with all factors of all given clocks.
 */
size_t gclk_get_factors_config_cnt(const gclk_t **clks, size_t cnt);

/**
 * @brief Get combined count of possible factor configs for all clock in a topology.
 *
 * Same as @ref gclk_get_factors_config_cnt() but takes the clock references from a
 * topology instead of a reference list.
 *
 * @param[in]  topology  Topology to get the count of possible factor combinations for.
 * @param[in]  len       Number of elements in @p topology.
 *
 * @return Number of factor combinations possible with the given topology.
 */
size_t gclk_get_factors_config_cnt_from_topology(clk_topology_entry_t *topology, size_t len);

/**
 * @brief Get overall count of topologies this clock can be driven by.
 *
 * The returned number considers all possible states that can be configured in
 * theory. I.e., no physical or logical constraints or other limits are
 * considered that may apply due to inter-dependencies or other restirction
 * that occur at runtime. In some scenarios the number of actually feasible configurations may
 * be a lot lower - but never higher.
 *
 * @param[in] clk  Reference tp the clock to get number of topologies for.
 *
 * @return Number of possible input topologies.
 */
size_t gclk_get_topology_config_cnt(const gclk_t *clk);

/**
 * @brief Calculate for each clock in the topology the frequency based the set factors.
 *
 * Updates the frequency values in the configuration based of factors that are set in
 * the same config. Frequency values are calculated based on the root frequency and
 * combined uptree factors at each level to avoid adding up rounding errors at each
 * level.
 *
 * @todo For very big equivalent uptree factors 64 bit arithmetic or a more
 *       sophisticated calculation (using lossless scaling) may be needed.
 *
 * @pre topology[0] holds the output clock and topology[len -1] holds the source.
 *
 * @param[in,out] topology  The topology config to update the frequencies for.
 * @param[in]     len       Number of elements in @p topology.
 */
void gclk_calculate_topology_config_freqs(clk_topology_entry_t *topology, size_t len);

/**
 * @brief Iteration helper to advance a topology to its next possible factor configuration.
 *
 * @pre @p topology must contain a valid configuration.
 * @pre @p conf_id must be a valid configuration index for this topology
 *      (i.e. any value of 0 - @ref gclk_get_factors_config_cnt() a.k.a.
 *      0 - @ref gclk_get_factors_config_cnt_from_topology()).
 *
 * @note the order how this iteration happens is defined by the topology. I.e., there is
 *       no guarantee that frequencies will change only in one direction for each step.
 *       But calling this for all possible configuration IDs will cover all possible
 *       frequency configurations. Albeit no configuration is returned twice, the
 *       resulting frequency for multiple different configurations may still be equal.
 *       Even configurations with the exact same topology path and same output frequency
 *       may differ in their power consumption. See functions related to the power model
 *       (e.g., @ref gclk_manager_cmp_topology_closest_leaf_freq_pmin()) on how this can
 *       be precalculated to find the lowest power option.
 *
 * @param[in] topology  The topology to advance to the next factor config.
 * @param[in] len       Number of elements in @p topology.
 * @param[in] conf_id   Unique index of the config to store in @p topology (unique per topology).
 */
void gclk_advance_topology_to_next_frequency_setting(clk_topology_entry_t *topology, size_t len, size_t conf_id);

/**
 * @brief Iteration helper to advance a list of factors to its next possible combination.
 *
 * Alternative to @ref gclk_advance_topology_to_next_frequency_setting() but operates on
 * a list of clock references and a corresponding list of factors instead of a topology.
 * The clock list is still expected to form atopology.
 * The factor corresponding to clks[n] will be stored at factors[n].
 *
 * @pre @p clks must point to a valid list of clock references.
 * @pre @p factors must point to a factor list with the same length as @p clks.
 *
 * @param[in]      clks     Pointer to an array of clock references (to be used for factor range metadata).
 * @param[in,out]  factors  Pointer to an array of factors where the respective scaling factors will be stored.
 * @param[in]      set_cnt  Number of elements in @p clks as well as @p factors (must be equal!).
 * @param[in]      conf_id  Unique index of the config to store in @p factors (unique per clock set).
 */
void gclk_get_nth_factors_config(const gclk_t **clks, uint32_t *factors, size_t set_cnt, size_t conf_id);

/**
 * @brief Get the (clock specific) unique topology id from a clock topology config.
 *
 * For a given topology config it returns the (clock-specific) unique topology id.
 * The id relates to the leaf clock of the given topology which is given in
 * topology_conf[0].
 *
 * @param[in]  topology_conf  Topology configuration to get the topology id for.
 * @param[in]  topo_len       Number of elements in @p topology_conf.
 *
 * @return The topology id that identifies the given config.
 */
unsigned int gclk_topology2id(const clk_topology_entry_t *topology_conf, uint32_t topo_len);

/**
 * @brief Get the product of a specific factor combination of the given clock list.
 *
 * Useful for factor match functions that do not operate on the whole topology and
 * instead represent a set of factors in a topology chain by an quivalent factor.
 * This function shall only be used on either a set of only divider or only
 * multiplier scalers, as mixing them will not yield a scalar value.
 *
 * @todo Check whether it is preferrable to replace this by
 *       @ref gclk_get_current_equivalent_uptree_factors() or the like.
 *
 * @param[in]      clks      Pointer to an array of clock references (to be used for factor range metadata).
 * @param[in]      clks_cnt  Number of elements in @p clks as well as @p factors (must be equal!).
 * @param[in]      n         Unique index of the config to store in @p factors (unique per clock set).
 *
 * return Product of all scale factor values.
 */
uint32_t gclk_get_nth_config_equivalent_factor(const gclk_t **clks, size_t clks_cnt, size_t n);

/**
 * @brief Get a topology from its unique index.
 *
 * @todo make the leaf clock a separate parameter to avoid duplicated code setting the leaf
 *       clock and unify style with e.g. @ref gclk_get_current_topology_config().
 *
 * @pre topology[0] must conatin the output clock
 * @param[in,out] topology  The location where the requested topology config will be written to.
 * @param[in]     max_len   Maximum number of entries topology can hold.
 * @param[in]     tid       Unique ID of the topology that is requested (specific to the leaf clock).
 *
 * @return The length of the new topology.
 */
unsigned int gclk_get_nth_topology(clk_topology_entry_t *topology, size_t max_len, size_t tid);

/**
 * @brief Get the resulting output frequency from an input frequency and a set of mul/div factors.
 *
 * @param[in]     fi         Input frequency.
 * @param[in]     mfacts     Array of multiplier factor values.
 * @param[in]     mfact_cnt  Number of multipliers in *p mfacts.
 * @param[in]     dfacts     Array of divider factor values.
 * @param[in]     dfact_cnt  Number of dividers in *p dfacts.
 *
 * @return The resulting output frequency.
 */
uint32_t gclk_get_factor_config_freq(uint32_t fi, uint32_t *mfacts, size_t mfact_cnt, uint32_t *dfacts, size_t dfact_cnt);

/**
 * @brief Get all divider clocks from a list of clocks.
 *
 * @param  clks      Array of arbitrary clock references to filter for dividers.
 * @param  len       Number of elements in @p clks.
 * @param  div_clks  Pointer to where the references of all dividers will be stored.
 *                   Must be big enough to store all divider clock references.
 *                   Can be set to NULL to query the size before actually getting the references.
 *
 * @return The number of dividers within the given list of clocks.
 */
uint32_t gclk_get_dividers_from_topology(const gclk_t **clks, uint32_t len, const gclk_t **div_clks);

/**
 * @brief Get all multiplier clocks from a list of clocks.
 *
 * @param  clks      Array of arbitrary clock references to filter for multipliers.
 * @param  topo_len  the toplology length i.e., number of clocks in the given topology
 * @param  mul_clks  Pointer to where the references of all multipliers will be stored.
 *                   Must be big enough to store all multiplier clock references.
 *                   Can be set to NULL to query the size before actually getting the references.
 *
 * @return The number of multipliers within the given list of clocks.
 */
uint32_t gclk_get_multipliers_from_topology(const gclk_t **clks, uint32_t topo_len, const gclk_t **mul_clks);

/**
 * @brief Prototype for factor matching functions.
 *
 * @pre @p mfacts must be big enough to hold @p mul_clks_cnt factors.
 * @pre @p dfacts must be big enough to hold @p div_clks_cnt factors.
 *
 * @param[in]     fi            Input frequency.
 * @param[in]     fo            Output frequency.
 * @param[in]     mul_clks      Pointer to clock reference array of multiplier clocks.
 * @param[in]     mul_clks_cnt  Number of elements in @p mul_clks.
 * @param[in,out] mfacts        Location where to store the best selected multiplier factors.
 * @param[in]     div_clks      Pointer to clock reference array of multiplier clocks.
 * @param[in]     div_clks_cnt  Number of elements in @p div_clks.
 * @param[in,out] dfacts        Location where to store the best selected divider factors.
 *
 * @return true    If a match was found.
 * @return false   If the frequency could not be found.
 */
typedef bool (*gclk_factor_match_func_t)(uint32_t fi, uint32_t fo,
                                 const gclk_t **mul_clks, size_t mul_clks_cnt, uint32_t *mfacts,
                                 const gclk_t **div_clks, size_t div_clks_cnt, uint32_t *dfacts);

/**
 * @brief Find a matching factor configuration to achieve a target frequency for the given topology.
 *
 * @param topology      The topology chain to find a scaling factor combination for.
 * @param topo_len      Number of elements in @p topology.
 * @param f_in          Frequency at the input of the topology chain.
 * @param f_out_target  The wanted target frequency at the topology output.
 * @param match_op      A factor matching function to be used for the matching process.
 *
 * @return  The matched target frequency if a matching config was found.
 * @return  @p GCLK_INVALID_FREQ if the factors could not be matched.
 */
uint32_t gclk_match_freq_conf(clk_topology_entry_t *topology, uint32_t topo_len,
                              uint32_t f_in, uint32_t f_out_target,
                              gclk_factor_match_func_t match_op);

/**
 * @brief Find the best matching factors for a given in/out frequency via full iteration.
 *
 * An implementation of the @ref gclk_factor_match_func_t factor matching interface that
 * just iterates through all possible combinations.
 *
 * @copydetails gclk_factor_match_func_t
 *
 * @return true    Always, because this function is not only searching for exact matches.
 */
bool gclk_match_closest_full_iter(uint32_t fi, uint32_t fo,
                                 const gclk_t **mul_clks, size_t mul_clks_cnt, uint32_t *mfacts,
                                 const gclk_t **div_clks, size_t div_clks_cnt, uint32_t *dfacts);
/**
 * @brief Find exactly matching factors for a given in/out frequency via full iteration.
 *
 * Same as @ref gclk_match_closest_full_iter() but only accepts exact matches.
 *
 * @copydetails gclk_factor_match_func_t
 */
bool gclk_match_exact_full_iter(uint32_t fi, uint32_t fo,
                                 const gclk_t **mul_clks, size_t mul_clks_cnt, uint32_t *mfacts,
                                 const gclk_t **div_clks, size_t div_clks_cnt, uint32_t *dfacts);

/**
 * @brief Find exactly matching factors for a given in/out frequency via iteration and recursion.
 *
 * An implementation of the @ref gclk_factor_match_func_t factor matching interface that
 * combines multiple scaler values before searching for fitting combinations as it
 * potentially allows to rule out bigger parts of the possible configuration space in each
 * step. Secifically, the multiplier values are iterated by first combining all multipliers
 * into a single value per possible combination. For each of those values the required optimal
 * (combined) divider value for the remaining clocks is then claculated, which is tried to
 * be recursively distributed to the remaining scalers.
 *
 * @copydetails gclk_factor_match_func_t
 */
bool gclk_match_iter_mul_recurse_div(uint32_t fi, uint32_t fo,
                                 const gclk_t **mul_clks, size_t mul_clks_cnt, uint32_t *mfacts,
                                 const gclk_t **div_clks, size_t div_clks_cnt, uint32_t *dfacts);

/**
 * @brief Find exactly matching factors for a given in/out frequency via iteration and factorization.
 *
 * This matching function iterates through possible (combined) multipliers same as
 * @ref gclk_match_iter_mul_recurse_div but instead of recursively trying to match the resulting
 * combined div factor, it factorizes this combined div value and tries to allocate the factors
 * across available dividers and returns early in case the min/max possible frequency abtainable
 * by the dividers rules out a feasibility.
 *
 * @note The current implementation of this function does **not** guarantee to cover the whole
 * configuration space because distributing the prime factors is only tried in one particular
 * order with a simplified greedy allocation. I.e., each prime factor is allocated to the first
 * clock that is able to accomodate it.
 *
 * @todo As of now this is only safely usable for scalers that cover a full contiguous range
 *       because feasability of a particular value is only checked against the max value
 *       of the divider.
 *
 * @copydetails gclk_factor_match_func_t
 */
bool gclk_match_iter_mul_factorize_div(uint32_t fi, uint32_t fo,
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

/**
 * @brief Check if a clock is a leaf clock (no other clock may use the clock as parent).
 *
 * @param[in] clk  The clock instance to check.
 *
 * @return  true   If no other clock can use this clock as parent.
 * @return  false  If there is at least one clock that can use the given clock as parent.
 */
bool gclk_is_leaf(const gclk_t *clk);

/**
 * @brief Check if a clock is currently used as parent by other active clocks.
 *
 * This function is useful to determine clocks or sub-trees that can actually
 * be switched off.
 *
 * @param[in] clk  The clock instance to check.
 *
 * @return   true   If another active clock is currently sourced by this clock.
 * @return   false  If no other clock is using the given clock as parent.
 */
bool gclk_is_used(const gclk_t *clk);

/**
 * @brief Static initialization helper for a range-based scaler (8 bit, regval equal to numval mapping).
 *
 * Initializes all required metadata to handle data encoded via @ref gclk_range8_t.
 * @param X Should be a statically defined @ref gclk_range8_t.
 */
#define GCLK_REGVAL_AS_NUMVAL_RANGE8_STATIC_INIT(X) .base.factor_mapping.range8 = &(X), \
                                                    .base.flags.conf_cnt = (X).max - (X).min + 1U, \
                                                    .base.factor_map_op = gclk_map_func_regval_as_numval_range8

/**
 * @brief Static initialization helper for a range-based scaler (8 bit, regval equal to idx mapping).
 *
 * Initializes all required metadata to handle data encoded via @ref gclk_range8_t.
 * @param X Should be a statically defined @ref gclk_range8_t.
 */
#define GCLK_IDX_AS_REGVAL_RANGE8_STATIC_INIT(X) .base.factor_mapping.range8 = &(X), \
                                                 .base.flags.conf_cnt = (X).max - (X).min + 1U, \
                                                 .base.factor_map_op = gclk_map_func_idx_as_regval_range8

/**
 * @brief Static initialization helper for a list-based scaler (8 bit, regval equal to idx mapping).
 *
 * Initializes all required metadata to handle data encoded via a uint8_t array.
 * @param X Should be a statically defined uint8_t array.
 */
#define GCLK_LIST8_STATIC_INIT(X) .base.factor_mapping.list8 = &(X[0]), \
                                  .base.flags.conf_cnt = ARRAY_SIZE(X), \
                                  .base.factor_map_op = gclk_map_func_idx_as_regval_list8

/**
 * @brief Static initialization helper for a list-based scaler (16 bit, regval equal to idx mapping).
 *
 * Initializes all required metadata to handle data encoded via a uint16_t array.
 * @param X Should be a statically defined uint16_t array.
 */
#define GCLK_LIST16_STATIC_INIT(X) .base.factor_mapping.list16 = &(X[0]), \
                                   .base.flags.conf_cnt = ARRAY_SIZE(X), \
                                   .base.factor_map_op = gclk_map_func_idx_as_regval_list16

/**
 * @brief Static initialization helper for a range-based scaler (16 bit, regval equal to numval mapping).
 *
 * Initializes all required metadata to handle data encoded via @ref gclk_range16_t.
 * @param X Should be a statically defined @ref gclk_range16_t.
 */
#define GCLK_REGVAL_AS_NUMVAL_RANGE16_STATIC_INIT(X) .base.factor_mapping.range16 = &(X), \
                                                     .base.flags.conf_cnt = (X).max - (X).min + 1U, \
                                                     .base.factor_map_op = gclk_map_func_regval_as_numval_range16

/**
 * @brief Static initialization helper for a range-based scaler (16 bit, regval equal to idx mapping).
 *
 * Initializes all required metadata to handle data encoded via @ref gclk_range16_t.
 * @param X Should be a statically defined @ref gclk_range16_t.
 */
#define GCLK_IDX_AS_REGVAL_RANGE16_STATIC_INIT(X) .base.factor_mapping.range16 = &(X), \
                                                  .base.flags.conf_cnt = (X).max - (X).min + 1U, \
                                                  .base.factor_map_op = gclk_map_func_idx_as_regval_range16

/**
 * @brief Static initialization helper for a pointer-LUT-based scaler.
 *
 * The LUT holds explicit pairs of a factor and a pointer. The memory referred to by
 * the pointer holds the register value corresponding to the factor.  Useful for OTP/flash factory
 * calibration values.
 *
 * Initializes all required metadata to handle data encoded via @ref gclk_reg_val_ptr_lut_t.
 * @param X Should be a statically defined array of @ref gclk_reg_val_ptr_lut_t.
 */
#define GCLK_PTR_LUT_STATIC_INIT(X) .base.factor_mapping.ptr_lut = &(X)[0], \
                                    .base.flags.conf_cnt = ARRAY_SIZE((X)), \
                                    .base.factor_map_op = gclk_map_func_ptr_lut

/**
 * @brief Static initialization helper for a factor-LUT-based scaler.
 *
 * The LUT holds explicit pairs of a factor and a register value.
 *
 * Initializes all required metadata to handle data encoded via @ref gclk_reg_val_factor_lut_t.
 * @param X Should be a statically defined array of @ref gclk_reg_val_factor_lut_t.
 */
#define GCLK_FACTOR_LUT_STATIC_INIT(X) .base.factor_map_op       = gclk_map_func_lut,\
                                       .base.flags.conf_cnt      = ARRAY_SIZE(X),\
                                       .base.factor_mapping.lut  = &X[0]

/**
 * @brief Static initialization helper for a parent-LUT-based mux.
 *
 * The LUT holds explicit pairs of a parent reference and a register value.
 *
 * Initializes all required metadata to handle data encoded via @ref gclk_parent_config_lut_t.
 * @param X Should be a statically defined array of @ref gclk_parent_config_lut_t.
 */
#define GCLK_PARENT_LUT_STATIC_INIT(X) .base.parent_map_op       = gclk_map_parent_lut,\
                                       .base.flags.conf_cnt      = ARRAY_SIZE(X),\
                                       .base.parent_mapping.lut  = &X[0]

/**
 * @brief Static initialization helper for a clock statically depending on another clock.
 *
 * Initializes all required metadata to handle data encoded via @ref gclk_reg_val_cross_ref_luf_t.
 * @param X Should be a statically defined array of @ref gclk_reg_val_cross_ref_luf_t.
 */
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
