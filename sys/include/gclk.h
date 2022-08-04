/*
 * Copyright (C) 2020 HAW Hamburg <michel.rottleuthner@haw-hamburg.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */
/**
 * @defgroup    sys_gclk generic clock config
 * @ingroup     sys
 * @brief       Provides a generic clock configuration
 *
 * This module is intended to configure and control platform specific clocks via a generic API.
 * Instances that might be controlled by this API could be one of the following (non exhaustive list):
 *  - static clock source (e.g. a fixed 32k crystal)
 *  - configurable clock provider (e.g. a PLL, multipliers, dividers)
 *  - muxes (e.g. selecting a specific clock from a ist of options)
 * For all these types it may be applicable to read information about the clocks, some neccessary, others optional
 *  - Neccessary:
 *               - is the clock is enabled / disabled
 *               - the current frequency
 *               - the frequency when connected to parent X at frequency Y
 *               - if applicable, which other clock currently serves as input to this clock (parent)
 *               - if applicable, which clocks *can* be used as input to this clock (available parents)
 *
 *  - Optional properties (considered but not implemented yet and open for discussion)
 *    -low power capabilities (pm mode availability, consumption)
 *    -type of clock (internal/external, RC/Crystal)
 *    -accuracy (PPM)
 * For some clocks there are also configurable parameters available e.g.:
 *  - enabling/disabling a clock (i.e. gating the clock)
 *  - changing the clocks frequency
 *      - by changing internal properties like multipliers, dividers etc.
 *      - by switching to another clock source
 *
 * Some of the desing-goals:
 *  - should be more much more lightweight (especially in size) than Linux CCF
 *    - no need for dynamic loading of clock nodes (we know the target hardware at compile time)
 *  - no dynamic allocation
 *  - make it possible to povide (but not neccessarily compile in) all available clocks
 *  - pull in clock information preferably only if used by somebody
 *  - possibly reuse the configuration data for offline (pre compile time) config tools
 *
 * Things still under consideration:
 *  - It might be helpful to provide information on whether a clock is just an itermediate or also a "consumer".
 *    A pure intermediate can be considered safe for disable, if it has no active children. While a clock that is used
 *    for something even if it has no children can never be just disabled without thoughts.
 *    Maybe a blocking mechanism would solve that.
 *  - A blocking mechanism could have multiple operation modes like:
 *    - do not change frequency!
 *    - do not change topology!
 *    - do not disable!
 *    - do whatever you want, but notify me (before/after) e.g., for re-initialization!
 *  - There are several "update freq/topology" strategies that can be implemented
 *    (A) before updating a clock, all effects of that transition could be eliminated (by cutting children conections)
 *        to not require complex tree traversals. Then an update only needs to take constraints of the changed clock
 *        itself into account.
 *        Possible problems: what if a conflic is detected after several incremental changes? Rollback possible at all?
 *                           on-the-fly adjustments may be less intrusive than incremental disable + reconfigure.
 *    (B) All side effects need to be extracted before an update.
 *
 *  - There are clocks that need other clocks to be enabled prior to configuration - how can we handle this?
 *    (A) Model them as gates -> only feasible when there is some form of base-sharing/compositing so that only the bare
 *    minimum of a gate needs to be stored.
 *    (B) Model a separate dependency tree that holds clock X requires clock Y dependencies and only provide it as
 *    optional feature.
 *  - There are nodes that are read-only regarding conigurton, but only allow specific settings to be used
 *    (e.g., the PLL on stm32l0x3 has a special 48 MHz output for USB, only valid when PLL_VSO is set to 96)
 *    Should this be considered via special "block"-enable constraints or do we simple leave this up to the user?
 *  - There are MCUs where the clock configuration uses a relatively big number of functionally identical instances.
 *    e.g., on the SAMd21 there are ~8 of the same clock generators, and ~the same order of magnitude gateable muxes
 *    after that. Configuring the instances happens by wrinting the instance ID to a register, and then reading/writing
 *    a config register (always the same across all instances). Since the config pattern is different we cannot (fully)
 *    reuse the generic implementations for gates/scalers/muxes. (wrapping in code that writes the idx before?)
 *    So all the instances share the same config, the same
 *    registers, but differ "only" by index. The current approach of storing the data would create a big unneccessary
 *    overhead on that platform. Better would be to reuse the same function calls for all classes (easy).
 *    And reuse the same register definitions across (almost) all instances (harder).
 *  - Some platforms may not have same-sized register access for all registers (example SAMd21, require 32 bit, some 16)
 *  - utility function to get first parent option with zero based idx?
 *  - how to indentify/link specific clock instances in an efficient and easy way
 *  - by ID vs. per reference
 *  - single array of clk structs where index is used as ID
 *  - how to identify the clock that is driving the CPU
 *  - how to provide (which) additional properties (get/set)
 *  - how to model differrent configurations of similar boards (i.e. which data structures to share, where to put files
 *    for specific vendors/MCU-series/boards)
 *  - do we want compile-time dynamic sizes for the generic clock types?
 *    e.g. a GCLK_MAX_SCALE_FACTOR_BITS that maps the type of gclk_reg_val_lut_t.factor to the smallest applicable size.
 *  - the "query possibility" part of the get_freq API could be made optional in a way that only allows getting the
 *    current config.
 *    - this would then allow to set up semi-static configurations (e.g. a fixed set of different options) while
 *      get_freq would still allow to get the current frequency.
 *  - would it make sense to separate topology and frequency ops?
 *    -this would allow saving unused function pointer space (e.g. set_parent for gates)
 *    -for nodes that have no topology-functionality the topology part could be completely handled by the high-layer API
 *  - enable / disable could be implemented using set_freq(0) / set_freq(hz)
 *  - setting up a frequency can happen in different ways:
 *    - possible values could be asked from from a gclk instance
 *      - does it make sense to allow asking for the next higher/next lower frequency?
 *      - parameters could constrain the result to a specifc subset of solutions:
 *        - (A) specific to a given parent
 *        - (B) specific to a given parent at the given frequency
 *        - (C) under the assumption that the parent can be switched -> this would need to return the required parent
 *        - (D) under the assumption the frequency of the parent can be changed -> this would need to return the required parent rate
 *        - (E) both C and D -> would need to return the required parent at the respective frequency
 *        - (F) assuming any clock up the tree can be changed at will -> would need to return all configs for the affected subtree
 *          - this has the additional problem of recursion that might cause trouble for more complex clock trees
 *    - Instead of asking for possible configs all the changes could just be executed directly (if applicable)
 *  - how can we manage configurations of different entities?
 *    - e.g. peripheral 1 wants clock X at Y hz, peripheral 2 wants clock X at Z hz (how to get, compare and decide)
 *    - placing dynamic constraints that contain possible configurations couls help
 *      e.g. if peripheral A needs freq X at clock 1, it could dynamically append this info to the node, so other tree
 *      explorations and configurations treat other configs as invalid and search for other options
 *  - do we need (want) some notofication mechanism to trigger HW/driver reconfigurations/re-init?
 *    - possible approaches for that:
 *       - (A) manage only the clock tree itself -> a device may lock a clock to a specific value to prohibit changes
 *       - (B) move also peripheral clock configs to the gclk module so that a clock update can reconfigure it
 *             seamlessly
 *       - (C) add reconfigure callbacks that get executed when a clock changes (could include lock/release of the
 *             peripheral) and additionally constraints that limit the configuration space.
 *
 * Discovered Problems that should be evaluated at some point
 * -on nucleo-l476rg USART2 can not be configured to run from LSE
 *  (communication breaks after switching the parent, others work fine)
 *  - also confirmed to not work with 9600 baud.. 2000 baud actually works with LSE AN4635 says the limit for LSE is
 *    2000 baud (with oversampling of 16) and 4000 baud with oversampling of 8.
 *
 * @todo -Add hooks general interface for freq change hooks
 *          - usable for voltage scaling pre/post and updating flash waitstates
 *       - improve verbose describe cmd to start min freq at >= 0 (maybe show 0 separately if actually possible)
 *       - improve transition cmd to forward change requests in cases where it is possible
 *
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

#ifdef __cplusplus
extern "C" {
#endif

/* TODO: this should move to some static bitarithm utility header
 *       and there is probably a nicer way to implement this functionality,
 *       or it can be avoided all together. For now we leave it to have an easy way
 *       to map vendor bitmask to MSB/LSB index declarations without manually
 *       looking up all the values */
#define BITMASK_BIT_0_SET(VAL)  ((((VAL & (1 <<  0))) != 0) ? 1 : 0)
#define BITMASK_BIT_1_SET(VAL)  ((((VAL & (1 <<  1))) != 0) ? 1 : 0)
#define BITMASK_BIT_2_SET(VAL)  ((((VAL & (1 <<  2))) != 0) ? 1 : 0)
#define BITMASK_BIT_3_SET(VAL)  ((((VAL & (1 <<  3))) != 0) ? 1 : 0)
#define BITMASK_BIT_4_SET(VAL)  ((((VAL & (1 <<  4))) != 0) ? 1 : 0)
#define BITMASK_BIT_5_SET(VAL)  ((((VAL & (1 <<  5))) != 0) ? 1 : 0)
#define BITMASK_BIT_6_SET(VAL)  ((((VAL & (1 <<  6))) != 0) ? 1 : 0)
#define BITMASK_BIT_7_SET(VAL)  ((((VAL & (1 <<  7))) != 0) ? 1 : 0)
#define BITMASK_BIT_8_SET(VAL)  ((((VAL & (1 <<  8))) != 0) ? 1 : 0)
#define BITMASK_BIT_9_SET(VAL)  ((((VAL & (1 <<  9))) != 0) ? 1 : 0)
#define BITMASK_BIT_10_SET(VAL) ((((VAL & (1 << 10))) != 0) ? 1 : 0)
#define BITMASK_BIT_11_SET(VAL) ((((VAL & (1 << 11))) != 0) ? 1 : 0)
#define BITMASK_BIT_12_SET(VAL) ((((VAL & (1 << 12))) != 0) ? 1 : 0)
#define BITMASK_BIT_13_SET(VAL) ((((VAL & (1 << 13))) != 0) ? 1 : 0)
#define BITMASK_BIT_14_SET(VAL) ((((VAL & (1 << 14))) != 0) ? 1 : 0)
#define BITMASK_BIT_15_SET(VAL) ((((VAL & (1 << 15))) != 0) ? 1 : 0)
#define BITMASK_BIT_16_SET(VAL) ((((VAL & (1 << 16))) != 0) ? 1 : 0)
#define BITMASK_BIT_17_SET(VAL) ((((VAL & (1 << 17))) != 0) ? 1 : 0)
#define BITMASK_BIT_18_SET(VAL) ((((VAL & (1 << 18))) != 0) ? 1 : 0)
#define BITMASK_BIT_19_SET(VAL) ((((VAL & (1 << 19))) != 0) ? 1 : 0)
#define BITMASK_BIT_20_SET(VAL) ((((VAL & (1 << 20))) != 0) ? 1 : 0)
#define BITMASK_BIT_21_SET(VAL) ((((VAL & (1 << 21))) != 0) ? 1 : 0)
#define BITMASK_BIT_22_SET(VAL) ((((VAL & (1 << 22))) != 0) ? 1 : 0)
#define BITMASK_BIT_23_SET(VAL) ((((VAL & (1 << 23))) != 0) ? 1 : 0)
#define BITMASK_BIT_24_SET(VAL) ((((VAL & (1 << 24))) != 0) ? 1 : 0)
#define BITMASK_BIT_25_SET(VAL) ((((VAL & (1 << 25))) != 0) ? 1 : 0)
#define BITMASK_BIT_26_SET(VAL) ((((VAL & (1 << 26))) != 0) ? 1 : 0)
#define BITMASK_BIT_27_SET(VAL) ((((VAL & (1 << 27))) != 0) ? 1 : 0)
#define BITMASK_BIT_28_SET(VAL) ((((VAL & (1 << 28))) != 0) ? 1 : 0)
#define BITMASK_BIT_29_SET(VAL) ((((VAL & (1 << 29))) != 0) ? 1 : 0)
#define BITMASK_BIT_30_SET(VAL) ((((VAL & (1 << 30))) != 0) ? 1 : 0)
#define BITMASK_BIT_31_SET(VAL) ((((VAL & (1 << 31))) != 0) ? 1 : 0)

#define STATIC_BITMASK_MSB(X) (BITMASK_BIT_31_SET(X) ? 31 : (\
                               BITMASK_BIT_30_SET(X) ? 30 : (\
                               BITMASK_BIT_29_SET(X) ? 29 : (\
                               BITMASK_BIT_28_SET(X) ? 28 : (\
                               BITMASK_BIT_27_SET(X) ? 27 : (\
                               BITMASK_BIT_26_SET(X) ? 26 : (\
                               BITMASK_BIT_25_SET(X) ? 25 : (\
                               BITMASK_BIT_24_SET(X) ? 24 : (\
                               BITMASK_BIT_23_SET(X) ? 23 : (\
                               BITMASK_BIT_22_SET(X) ? 22 : (\
                               BITMASK_BIT_21_SET(X) ? 21 : (\
                               BITMASK_BIT_20_SET(X) ? 20 : (\
                               BITMASK_BIT_19_SET(X) ? 19 : (\
                               BITMASK_BIT_18_SET(X) ? 18 : (\
                               BITMASK_BIT_17_SET(X) ? 17 : (\
                               BITMASK_BIT_16_SET(X) ? 16 : (\
                               BITMASK_BIT_15_SET(X) ? 15 : (\
                               BITMASK_BIT_14_SET(X) ? 14 : (\
                               BITMASK_BIT_13_SET(X) ? 13 : (\
                               BITMASK_BIT_12_SET(X) ? 12 : (\
                               BITMASK_BIT_11_SET(X) ? 11 : (\
                               BITMASK_BIT_10_SET(X) ? 10 : (\
                               BITMASK_BIT_9_SET(X) ?   9 : (\
                               BITMASK_BIT_8_SET(X) ?   8 : (\
                               BITMASK_BIT_7_SET(X) ?   7 : (\
                               BITMASK_BIT_6_SET(X) ?   6 : (\
                               BITMASK_BIT_5_SET(X) ?   5 : (\
                               BITMASK_BIT_4_SET(X) ?   4 : (\
                               BITMASK_BIT_3_SET(X) ?   3 : (\
                               BITMASK_BIT_2_SET(X) ?   2 : (\
                               BITMASK_BIT_1_SET(X) ?   1 : (\
                               BITMASK_BIT_0_SET(X) ?   0 : 0\
                               ))))))))))))))))))))))))))))))))

#define STATIC_BITMASK_LSB(X) (BITMASK_BIT_0_SET(X) ?   0 : (\
                               BITMASK_BIT_1_SET(X) ?   1 : (\
                               BITMASK_BIT_2_SET(X) ?   2 : (\
                               BITMASK_BIT_3_SET(X) ?   3 : (\
                               BITMASK_BIT_4_SET(X) ?   4 : (\
                               BITMASK_BIT_5_SET(X) ?   5 : (\
                               BITMASK_BIT_6_SET(X) ?   6 : (\
                               BITMASK_BIT_7_SET(X) ?   7 : (\
                               BITMASK_BIT_8_SET(X) ?   8 : (\
                               BITMASK_BIT_9_SET(X) ?   9 : (\
                               BITMASK_BIT_10_SET(X) ? 10 : (\
                               BITMASK_BIT_11_SET(X) ? 11 : (\
                               BITMASK_BIT_12_SET(X) ? 12 : (\
                               BITMASK_BIT_13_SET(X) ? 13 : (\
                               BITMASK_BIT_14_SET(X) ? 14 : (\
                               BITMASK_BIT_15_SET(X) ? 15 : (\
                               BITMASK_BIT_16_SET(X) ? 16 : (\
                               BITMASK_BIT_17_SET(X) ? 17 : (\
                               BITMASK_BIT_18_SET(X) ? 18 : (\
                               BITMASK_BIT_19_SET(X) ? 19 : (\
                               BITMASK_BIT_20_SET(X) ? 20 : (\
                               BITMASK_BIT_21_SET(X) ? 21 : (\
                               BITMASK_BIT_22_SET(X) ? 22 : (\
                               BITMASK_BIT_23_SET(X) ? 23 : (\
                               BITMASK_BIT_24_SET(X) ? 24 : (\
                               BITMASK_BIT_25_SET(X) ? 25 : (\
                               BITMASK_BIT_26_SET(X) ? 26 : (\
                               BITMASK_BIT_27_SET(X) ? 27 : (\
                               BITMASK_BIT_28_SET(X) ? 28 : (\
                               BITMASK_BIT_29_SET(X) ? 29 : (\
                               BITMASK_BIT_30_SET(X) ? 30 : (\
                               BITMASK_BIT_31_SET(X) ? 31 : 0\
                               ))))))))))))))))))))))))))))))))

/**
 * @brief flags for encoding topology related details of specific clock node instances
 *
 */
enum gclk_clk_topology_flags {
    GCLK_STOP_FOR_UPDATE          = 0x01, /*< The clock must be stopped before it can be updated to another freq config */
    GCLK_STOP_PARENT_FOR_UPDATE   = 0x02, /*< e.g. on STM32 the PLL VCO scaler must be stopped before any of its
                                            children (P,Q,R) can be updated */
    GCLK_STOP_CHILDREN_FOR_UPDATE = 0x04, /*< e.g. on STM32 all children of PLLM (PLL prescaler) must be stopped before
                                              it can be updated.
                                              This flag will be interpreted with a *forward semantic* i.e. if the next
                                              children can not be disabled the children of that will be disabled */
    GCLK_STRICT_UPTREE_DEPENDENT  = 0x08, /*< indicates that a clock can not be reconfigured arbitrarily, but instead
                                              its state strictly depends on another clock up in the tree. This is usually
                                              the case if there is some hardwired dependency e.g., if a prescaler is shared
                                              across two outputs of if there is a clock that limits its output automatically
                                              if its input is above some value. */
    /* GCLK_INPUT_VALUE_LOCKED? (for clocks that prohibit on the fly changes during use but dont need to be stopped
     *                           i.e. when neither of the clocks must be stopped for reconfiguration but it is forbidden
     *                           to change the input signal..) */
};

/**
 * @brief flags for encoding detail deifferneces of specific clock node instances
 * @TODO  it might be overall more efficient to just provide a mapping function with the instance
 *        The mapping function can then be moved to a common file for reuse.
 *
 */
enum gclk_clk_conf_flags {
    NOT_USED_ANYMORE = 0x01,
//    GCLK_CONF_MAP_OFFS_ONE        = 0x01, /*< Indicates for implicitly encoded values (i.e. in form of the index of
//                                              GCLK_CONF_LISTX) that an offset of 1 must be added to the implicit value */
//    GCLK_CONF_IDX_AS_NUM_VAL      = 0x02, /*< For implicitly (index) encoded values this indicates the implicit value
//                                              will be interpreted as the numerical "calculation" value. When this flag
//                                              is not set, the defaut is to use the implicit value as the configuration
//                                              register value.
//                                              Note: this can also be used together with ranges (i.e. GCLK_CONF_RANGEX)
//                                              to interpret the index as the numeric value instead of the value at that
//                                              position */
//    GCLK_REG_VAL_INC_PRE_MAP      = 0x04, /*< Indicates that values need to be incremented before they are used to map
//                                              to the numerical value. This is useful for example if a register value of
//                                              0 is equivalent to a multiplier or divisor of 1. This always works in both
//                                              directions: value determined by the mapping function will be decremented
//                                              before it is written to the config register */
//    GCLK_REG_VAL_DEC_PRE_MAP      = 0x08, /*< Same as above but decrementing before mapping. */
// @todo under consideration, could potentially be used to speed up queries:
//    GCLK_PARENT_FREQ_PASSED     = 0x04, /*< indicates if the parent frequency is just passed thru this node, nodes that
//                                            can have passed, as well as modified frequencies are not supported and
//                                            must be modeled manually */
//    GCLK_FREQ_VARIABLE          = 0x08, /*< indicates that this node can alter its output frequency internally.
//                                            Does not apply to nodes just forwarding a (variable) parent frequency */
//    GCLK_FREQ_DEPENDS_ON_PARENT = 0x10, /*< indicates get_freqthat the frequency of this clock depends on it's parent clock */
//    GCLK_EXCLUSIVE_PARENT       = 0x20, /*< indicates that the parent is only (exclusively) serving this particular
//                                            clock without affecting other clocks. This can be helpful when modelling
//                                            complex composite clock nodes as multiple base-type instances (e.g.) a
//                                            combination of a mux -> scaler -> gate.
//                                            In such a case the clock framework can safely delegate requests like
//                                            set_freq from the gate up until the mux. With that a user of the leaf-clock
//                                            (i.e. the gate in this case) can call into the api as if the gate suports
//                                            all operations directly (set_freq / set_parent etc.)
//                                            @todo: maybe it would be good to explicitly control if this is allowed
//                                                   for a specific call as it changes the semantics (e.g. set_parent)*/
};

/**
 * @brief   Describes the mapping between register values and calculation paramters
 *
 * @detail  There are many different ways on how to convert register contents to values that can be used to calculate
 *          frequencies etc. In many cases the same information can be encoded with more than one mapping type.
 *          E.g. a 4 bit register value that directly represents all values from 0 to 15 could be encoded as a lookup
 *          table, mappong 0 to 0 , 1 to 1 an so on - but encoding it as a value range by only providing a min and max
 *          value is obviously much more efficient. See the below descriptions for details on the mappping types.
 *
 */
enum gclk_reg_val_mapping_type {
    /* This kind of mapping type is meant for cases where the register content directly maps to its
       numerical value. It uses two 8 bit integer values (min, max) to describe the allowed range
       (including bounds). The respective datatype is gclk_range8_t. */
    GCLK_CONF_RANGE8  = 0x01,
    /* Same as GCLK_VALUE_RANGE8 but for 16 bit values. The respective datatype is gclk_range16_t */
    GCLK_CONF_RANGE16 = 0x02,
    /* With this type, valid configurations are described as a lookup table consisting of pairs.
       Each register value corresponds to a numeric value. It is allowed to specify multiple pairs
       that map multiple register values to different numeric values oand vice versa.
       The order MUST be from lowest numerical value to highest numerical value */
    GCLK_CONF_LUT     = 0x03,
    /* A 1-dimensional uint8_t array of which each entry implicitly encodes pairs (list[idx] , idx).
       Can be used together with flag GCLK_CONF_IDX_AS_NUM_VAL to specify the index encodes the
       numerical value (otherwise the index encodes the configuration register value).
       The flag GCLK_CONF_MAP_OFFS_ONE may be used to specify the encoded value is index + 1. */
    GCLK_CONF_LIST8   = 0x04,
    /* Same as GCLK_CONF_LIST8 but for 16 bit values */
    GCLK_CONF_LIST16  = 0x05,
    /* Same as GCLK_CONF_LIST8 but for 32 bit values */
    GCLK_CONF_LIST32  = 0x06,
    /* Stores a single, non-configurable integer value. @todo use pointer directly for storage?
       Note: This is not really a mapping and a simple fixed value can be modelled more efficiently.
             It's purpose is to allow reusing some of the existing functionality of the generic
             implementations instead of providing a specialized one. So using this only makes sense
             for a low count of GCLK_FIXED_INT instances and when other instances already pull in
             the relevant generic functions anyway */
    GCLK_FIXED_INT    = 0x07,
    /* Same as GCLK_CONF_LUT this holds pairs of numeric values and configuration register values.
     * But instead of holding the config value directly, it contains pointers that in turn point
     * to the actual config value at a fixed memory location.
     * This is useful if the device holds special factory-calibrated configuration register values
     * used to setup frequency configurations. */
    GCLK_CONF_PTR_LUT = 0x08,
    /* @todo: possibly needed/ beneficial at some point:
         -value encoded as 2 ^ x  would safe space e.g. for _stm32_ahb_div_confs, _stm32_apb_div_confs
         -value encoded as x * <fixedint>
         -LUF: a lookup function*/
};

/**
 * @brief values to encode what kind of frequency is queried
 */
enum gclk_api_flags {
    GCLK_FREQ_NEXT_HIGHER = 0x01, /*< requests the next higher possible frequency that is not the same */
    GCLK_FREQ_NEXT_LOWER  = 0x02, /*< requests the next lower possible frequency that is not the same */
    GCLK_FREQ_CLOSEST     = 0x04, /*< requests the closest possible frequency (may be equal, lower or higher)
                                      NOTE: a frequency of 0 Hz has a special meaning in this case.
                                      When requesting a frequency of 1 Hz implementations must always give the closest
                                      frequency that actually produces a clock signal. Only when explicitly requesting
                                      zero Hz, 0 should be returned */
};

#define GCLK_INVALID_FREQ       (0xFFFFFFFFU)
#define GCLK_HIGHEST_VALID_FREQ (GCLK_INVALID_FREQ - 1U)
#define GCLK_UNDEFINED_TOPOLOGY (-1)

/**
 * @brief values to encode a specific set operation for enable, or a pure read operation to get the current state
 */
typedef enum gclk_enable_option {
    GCLK_ENABLE,  /*< enables the clock */
    GCLK_DISABLE, /*< disables the clock */
    GCLK_READ,    /*< request the current state only without modification */
} gclk_enable_option_t;


/* the below section needs to be moved out of here once the best method is selected (or it is found that tiny regref
   is not enough for some platforms )*/
#ifdef GCLK_USE_TINY_REG_REF

#ifndef GCLK_CONF_REG_IDX_BITWIDTH
#define GCLK_CONF_REG_IDX_BITWIDTH  (4)
#endif

typedef struct __attribute__((packed)) {
    unsigned int         en: GCLK_CONF_REG_IDX_BITWIDTH;
    unsigned int        rdy: GCLK_CONF_REG_IDX_BITWIDTH;
    unsigned int       conf: GCLK_CONF_REG_IDX_BITWIDTH;
    unsigned int     en_bit: 5; /* 0-31 */
    unsigned int    rdy_bit: 5; /* 0-31 */
    unsigned int   conf_lsb: 5; /* 0-31 */
    unsigned int   conf_msb: 5; /* 0-31 */
} gclk_reg_ref_t;

extern uint32_t volatile * const conf_regs[];

static inline volatile uint32_t * gclk_regref2enable_reg(gclk_reg_ref_t regref) {
    return conf_regs[regref.en];
}

static inline uint32_t gclk_regref2enable_mask(gclk_reg_ref_t regref) {
    return 1 << regref.en_bit;
}

static inline volatile uint32_t * gclk_regref2ready_reg(gclk_reg_ref_t regref) {
    return conf_regs[regref.rdy];
}

static inline uint32_t gclk_regref2ready_mask(gclk_reg_ref_t regref) {
    return 1 << regref.rdy_bit;
}

static inline volatile uint32_t * gclk_regref2conf_reg(gclk_reg_ref_t regref) {
    return conf_regs[regref.conf];
}

static inline uint32_t gclk_regref2conf_mask(gclk_reg_ref_t regref) {
    uint32_t mask = 0xFFFFFFFF;
    mask = mask >> regref.conf_lsb;
    mask = mask << regref.conf_lsb;
    mask = mask << (31 - regref.conf_msb);
    mask = mask >> (31 - regref.conf_msb);
    return mask;
}

bool gclk_regref_enable_op(gclk_reg_ref_t regref, gclk_enable_option_t opt);
#endif

#define GCLK_CURRENT_PARENT_IDX     (0) /* index used to get the currently active parent with get_parent */

/* Notes on mapping between user facing and hardware facing API:
   - a the application/the developer is expected to always calls into the gclock API via the user facing API by using
     gclk_t handles.
   - a HW-facing function may need access to the HL handle
*/
struct gclk_base;
typedef struct gclk_base gclk_t; /* Handle for the generic part of the API */


/**
 * @brief Entry describing one node of a specific topology configuration
 *
 */
typedef struct clk_topology_entry {
    const gclk_t  *clk;          /* the clock for wich the following information is */
    uint32_t       clk_freq;     /* a frequency that clk could be set to under the parent identified by cur_par_idx */
    //uint32_t       clk_freq_max; /* the max frequency that clk can be set to (only valid for a particular parent config below) */
    //uint32_t       clk_freq_min; /* the min frequency that clk can be set to (only valid for a particular parent config below) */
    uint32_t       factor: 24;         /* the current factor of the clock */
    //uint32_t       par_freq;     /* the frequency parent par_idx must be set to, to allow clk to be set to clk_freq */
    uint32_t       par_idx: 6;      /* the parent option idx (one based) that can be used to set up clk to clk_freq */
    uint32_t       enabled : 1;      /* the enable state of the clock (only used for operations on the tree model, not for an active topology) */
    uint32_t       propagation_pending : 1; /* marker to indicate that this config is new/dirty and must be propagated downtree
                                           (only used for operations on the tree model, not for an active topology) */
} clk_topology_entry_t;

/* The types of scalers currently supported.
 * @todo split this out to generic_scaler and userflags */
enum gclk_scaler_type {
    GCLK_NOSCALE = 0, /**< This clock is not scalable */
    GCLK_MUL     = 1, /**< The scaler multiplies the input frequency */
    GCLK_DIV     = 2, /**< The scaler divides the input frequency */
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
        enum gclk_clk_conf_flags       conf_flags      : 4;  /*@ todo: could/should? be moved to user flags */
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
        unsigned int                   user_flags      : 32 - (1 + 1 + 1 + 1 + 1 + 10 + 4 + 4 + 6 + 2); /* reserved for platform use */
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

//TODO: do we need a global state object for the gclk module itself?

int gclk_module_init(void);
const char *gclk_get_name(const gclk_t *clk);
const gclk_t* gclk_get_clk_by_name(const char *name);

/**
 * @brief get the current frequency of this clock
 */
uint32_t gclk_get_current_freq(const gclk_t *gclk);

/**
 * @breif get an equivalent representation of uptree clocks
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

void gclk_print_topology_conf(clk_topology_entry_t *topology, uint32_t size, bool min_max, bool factors);

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

/* @todo: add a paremeter to allow/disallow automatic topology changes? */
uint32_t gclk_set_freq(const gclk_t *gclk, uint32_t freq);

/* set a scaling factor on clocks that support that */
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

/* @brief   a compare function that returns true if topo_cmp is a better topology than topo_best */
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
