/*
 * Copyright (C) 2023 HAW Hamburg
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    drivers_flashpatch flashpatch
 * @ingroup     drivers_misc
 * @brief       Utility module for the FPB hardware on Cortex-M MCUs
 *
 * @{
 *
 * @file
 *
 * @author      Michel Rottleuthner <michel.rottleuthner@haw-hamburg.de>
 */

#ifndef FLASHPATCH_H
#define FLASHPATCH_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name    Number of instruction comparators to allocate memory for.
 * @{
 */
#ifndef FLASHPATCH_INSTRUCTION_COMPARATORS_MAX_NUMOF
#define FLASHPATCH_INSTRUCTION_COMPARATORS_MAX_NUMOF (6)
#endif

/**
 * @name    Number of literal comparators to allocate memory for.
 * @{
 */
#ifndef FLASHPATCH_LITERAL_COMPARATORS_MAX_NUMOF
#define FLASHPATCH_LITERAL_COMPARATORS_MAX_NUMOF (2)
#endif

#define FLASHPATCH_ALL_COMPARATORS_MAX_NUMOF ((FLASHPATCH_INSTRUCTION_COMPARATORS_MAX_NUMOF) \
                                            + (FLASHPATCH_LITERAL_COMPARATORS_MAX_NUMOF))

/**
 * @name    Max number of bytes used for stack allocated arguments.
 *
 * WARNING: This number is critical for the construction of function calls
 * which is done for the function wrapper feature. It must be big enough to
 * fit all possible sizes of stack allocated parameters of any function intended to
 * be wrapped.
 *
 * The value will be used for the 'size' parameter for all calls of
 * __builtin_apply.
 * @{
 */
#ifndef FLASHPATCH_STACK_ALLOCATED_AGR_BYTES
#define FLASHPATCH_STACK_ALLOCATED_AGR_BYTES (16)
#endif

/**
 * @brief   Device descriptor for the driver
 */
typedef struct __attribute__ ((aligned(32))) {
    uint32_t remap_table[FLASHPATCH_ALL_COMPARATORS_MAX_NUMOF];
    /* serves as marker which comparators are used
     * for a remapped address. For each comparator used to
     * remap acertain address, the respective index in this array
     * holds the address. For each address, at least one and max
     * two comparators are used. */
    uint32_t remapped_addrs[FLASHPATCH_ALL_COMPARATORS_MAX_NUMOF];
    unsigned int instruction_cmp_cnt;
    unsigned int literal_cmp_cnt;
} flashpatch_t;

typedef struct {
    void *wrapper_function;
    void *wrapped_function;
    void *ctx;
} flashpatch_function_wrapper_t;

/**
 * @brief   Context struct for the generic pre-post-cb wrapper.
 * @see flashpatch_pre_post_cb_wrapper()
 */
typedef struct {
    void (*pre)(void *ctx);
    void (*post)(void *ctx);
    void *cb_ctx;
} flashpatch_ppcb_wrapper_ctx_t;

typedef void(*function_ptr_t)(void);

/**
 * @brief   Initialize the flash patch peripheral.
 *
 * Initializes all required data structures for later use of the FPB
 * peripheral.
 * Ensures the peripheral version is compatible with this module and
 * stores metadata about its functionality.
 *
 * @return                  0 on success
 * @return                 <0 on error
 */
int flashpatch_init(void);

/**
 * @brief Remap a function with another function.
 *
 * @param[in] old_func  Function pointer refering to the instruction to be remapped.
 * @param[in] new_func  Function pointer that shall be executed instead.
 * @param[in] cmp_idx   Index of an instruction comparator to be used.
 *                      Must be a valid index of an instruction comparator between
 *                      0 and 'num of available instruction comparators' - 1
 *
 * @return    num of comparators used for the remap on success (1 or 2)
 *            0  if the function could not be wrapped
 */
int flashpatch_remap_function(function_ptr_t old_func, function_ptr_t new_func, unsigned cmp_idx);

/**
 * @brief Remove the remapping of a function.
 *
 * @param[in] remapped_function  Pointer to the remapped function.
 *
 * @return    num of comparators freed on success (1 or 2)
 *            0  if the function was not remapped
 */
int flashpatch_delete_remap(function_ptr_t remapped_function);

/**
 * @brief Wrap a function with a generic function wrapper.
 *
 * @param[in] wrapped_function  Function that shall be called from the wrapper.
 * @param[in] wrapper_function  Function that will from now on be be executed instead of
                                @p wrapped_function.
 * @param[in] ctx               Context to be used by the wrapper.
 *
 * @return    num of comparators used for the wrap on success (1 or 2)
 *            <0  if the function could not be wrapped
 */
int flashpatch_wrap_function(function_ptr_t wrapped_function, function_ptr_t wrapper_function, void *ctx);

/**
 * @brief Remove a generic function wrapper.
 *
 * This removes a wrapper set up via @ref flashpatch_wrap_function().
 * After calling this function the wrapped function will again be
 * executed normally, i.e., without calling the wrapper first.
 *
 * @param[in] wrapped_function  Function to remove the wrapper from.
 *
 * @return   num of comparators freed
 *           0 if the function was not wrapped before
 */
int flashpatch_remove_function_wrapper(void *wrapped_function);

/**
 * @brief Generic fuction wrapper with pre- and post-callbacks
 *
 * This wrapper calls user-defined pre- and post-functions
 * before calling the wrapped function.
 * Intended to be set up via @ref flashpatch_wrap_function() by handing a pointer to
 * this function as wrapper_function parameter.
 *
 * @WARNING When using this wrapper, the ctx parameter handed
 *          to @ref flashpatch_wrap_function() *MUST* be a pointer
 *          to a @ref flashpatch_ppcb_wrapper_ctx_t instance.
 */
void flashpatch_pre_post_cb_wrapper(void);

/**
 * @brief Enable the flash patch peripheral block.
 *
 * Must be called in order to be able to use any FPB feature.
 */
void flashpatch_enable(void);

/**
 * @brief Disable the flash patch peripheral block.
 *
 * Disables all FPB features (no enabled comparator will fire).
 */
void flashpatch_disable(void);

/**
 * @brief Enable specific comparator.
 *
 * @note The global enable (via @ref flashpatch_enable()) also affects
 *       all comparators.
 */
void flashpatch_comparator_enable(unsigned idx);

/**
 * @brief Disable specific comparator.
 *
 * @note The global enable (via @ref flashpatch_enable()) also affects
 *       all comparators.
 */
void flashpatch_comparator_disable(unsigned idx);

#ifdef __cplusplus
}
#endif

#endif /* FLASHPATCH_H */
/** @} */
