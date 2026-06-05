/*
 * Copyright (C) 2023 HAW Hamburg
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    sys_nope NOP-execution util
 * @ingroup     sys
 * @brief       Provides helpers for NOP based delays
 *
 * @{
 *
 * @file
 * @brief       NOP execution helper
 *
 * @author      Michel Rottleuthner <michel.rottleuthner@haw-hamburg.de>
 */

#ifndef NOPE_H
#define NOPE_H

#ifdef __cplusplus
extern "C" {
#endif

#define _NOP_ASM_STR "mov r8, r8;"
#define DO_NOP __asm__(_NOP_ASM_STR);
#define DO_10_NOPS DO_NOP DO_NOP DO_NOP DO_NOP DO_NOP DO_NOP DO_NOP DO_NOP DO_NOP DO_NOP
#define DO_100_NOPS DO_10_NOPS DO_10_NOPS DO_10_NOPS DO_10_NOPS DO_10_NOPS DO_10_NOPS DO_10_NOPS DO_10_NOPS DO_10_NOPS DO_10_NOPS
#define DO_1000_NOPS DO_100_NOPS DO_100_NOPS DO_100_NOPS DO_100_NOPS DO_100_NOPS DO_100_NOPS DO_100_NOPS DO_100_NOPS DO_100_NOPS DO_100_NOPS

typedef struct {
  uint32_t cnt:5;
  uint32_t unused:27;
} lt32_cnt_t;

#define NOPE_MAX_UNROLL_NOPS (31)
#define NOPE_NOP_CNT2DELAY_MARKER(X) (((X) > NOPE_MAX_UNROLL_NOPS) ? ~(X) : (X))

/* NOTE: this *MUST* be synced to nope_rom_nops definition */
#define NOPE_ROM_NOPS_MAX (200)
#define NOPE_RAM_NOPS_MAX (200)

#define DO_LT32UR_NOPS(N) \
    switch (N) {\
        _NOPE_NOP_CASE(31) \
        _NOPE_NOP_CASE(30) \
        _NOPE_NOP_10CASES_DESC(2) \
        _NOPE_NOP_10CASES_DESC(1) \
        _NOPE_NOP_10CASES_DESC(0x) \
        break; \
        default: \
           __builtin_unreachable(); \
    }\

/* build a no argument function pointer that points to an address in ROM that holds N
 * NOPs + a branch to the link register to jump back to where the fptr was called.
 * N must be <= NOPE_ROM_NOPS_MAX */
#define NOPE_GET_ROM_NOP_FPTR(N) ((&nope_rom_nops + 2 * NOPE_ROM_NOPS_MAX) - ((N) * 2))

/* build a no argument function pointer that points to an address in RAM that holds N
 * NOPs + a branch to the link register to jump back to where the fptr was called.
 * + 1 is needed for thumb flag.
 * N must be <= NOPE_RAM_NOPS_MAX  */
#define NOPE_GET_RAM_NOP_FPTR(N) (nope_fixed_nop_cb_t)((((uint32_t)&ram_nops[0]) + 1 + 2 * NOPE_RAM_NOPS_MAX) - ((N) * 2))

extern uint16_t ram_nops[NOPE_RAM_NOPS_MAX + 1];

/**
 * @brief   Do lower than 32 unrolled NOPs or arbitrary NOP iterations.
 * 
 * Both, unrolled and iterated nop count will have a static (but different) overhead
 * related to the switch keyword and the corresponding jumb table.
 * The overhead for unrolled NOPS is only the switch, whereas 
 * Unrolled NOPs will give a better resolution of the resulting delay, whereas 
 * iterative NOPs allow for longer timespans with more coarse resolution.
 *
 * @param[in]   N  number of NOPs (must be < 32) or ~arbitrary_number_of_nops
 */
#define DO_LT32UR_GE32_ITER_NOPS(N) \
    switch (N) {\
        _NOPE_NOP_CASE(31) \
        _NOPE_NOP_CASE(30) \
        _NOPE_NOP_10CASES_DESC(2) \
        _NOPE_NOP_10CASES_DESC(1) \
        _NOPE_NOP_10CASES_DESC(0x) \
            break;\
        default: \
            NOPE_DO_ITER_NOPS(~N);\
    }\


#define _NOPE_NOP_CASE(__CASE_NUM) \
    /* fall through */ \
    case __CASE_NUM: \
        DO_NOP \

#define _NOPE_NOP_10CASES_DESC(__LEADING_DIGITS) \
_NOPE_NOP_CASE(__LEADING_DIGITS ## 9) \
_NOPE_NOP_CASE(__LEADING_DIGITS ## 8) \
_NOPE_NOP_CASE(__LEADING_DIGITS ## 7) \
_NOPE_NOP_CASE(__LEADING_DIGITS ## 6) \
_NOPE_NOP_CASE(__LEADING_DIGITS ## 5) \
_NOPE_NOP_CASE(__LEADING_DIGITS ## 4) \
_NOPE_NOP_CASE(__LEADING_DIGITS ## 3) \
_NOPE_NOP_CASE(__LEADING_DIGITS ## 2) \
_NOPE_NOP_CASE(__LEADING_DIGITS ## 1) \
_NOPE_NOP_CASE(__LEADING_DIGITS ## 0)


#define NOPE_DO_ITER_NOPS(NOPS) \
    uint32_t __nop_cnt = NOPS; \
    do {\
        __asm__ volatile("" : "+g" (__nop_cnt) : :); \
    } while (--__nop_cnt); \


/**
 * @brief   Do an iteration of n NOPs (or similarly short) instructions.
 *
 * @param[in]   n  number of NOPs (must be <= 150)
 */
__attribute__((noinline)) void nope_iter_nops(uint32_t n);

/**
 * @brief   Do up to 32 NOPs .
 *
 * @param[in]   n  number of NOPs (must be <= 150)
 */
__attribute__((noinline)) void nope_ut32_switched_nops(uint32_t n);

void __attribute__((aligned(4),naked,noinline)) nope_rom_nops(void);

typedef void (*nope_var_nop_cb_t)(uint32_t);
typedef void (*nope_fixed_nop_cb_t)(void);

typedef struct {
    uint32_t nop_ps;       /* duration of a NOP */
    uint32_t call_ovhd_ps; /* call overhead for doing multiple NOPS */
} nope_delay_conf_t;

/* This contains calibration values for the different kinds of NOP delay implementations. */
typedef struct {
    nope_delay_conf_t raw;      /* inlined raw NOPs whithout any call overhead (only limited by code size) */
    nope_delay_conf_t switched; /* variable (within a compile time configured max) number of NOPs where the
                                   number of executed NOPs is decided dynamically via a big switch case. */
    nope_delay_conf_t iter;     /* unlimited variable number of NOPs where the number of NOPs
                                   defines the number of NOP iterations. */
    nope_delay_conf_t ram;      /* compile time and optionally runtime configured maximum number of NOPs
                                   where a predetermined number of op codes reside in RAM. */
    nope_delay_conf_t rom;      /* compile time limited maximum number of NOPs where a target branch to a
                                   given number of NOPs can be precomputed as long as it is within
                                   the maximum number of NOPs of the delay function. */
    nope_delay_conf_t lt32aur;  /* a mix of 'switched' and 'iter' implementations above. A compile time cofigurable
                                   max number of NOPs will be executed by jumping into a switch. Longer delays will
                                   use iteration. This defines the timing for a 'small' number of NOPs without iteration.*/
    nope_delay_conf_t ge32aur;  /* Same as above, lt32aur but for the case where the number of NOPS will be reflected
                                   by iterations. */
} nope_calibration_t;
    

void nope_calibrate(nope_calibration_t *cal);

void nope_print_calibration(nope_calibration_t *cal);

/**
 * @brief   Do up to 200 NOPs
 *
 * @param[in]   N  number of NOPs (must be <= 150)
 */
#define DO_UT200_NOPS(NOPS) \
switch (NOPS) {\
   _NOPE_NOP_CASE(200) \
   _NOPE_NOP_10CASES_DESC(19) \
   _NOPE_NOP_10CASES_DESC(18) \
   _NOPE_NOP_10CASES_DESC(17) \
   _NOPE_NOP_10CASES_DESC(16) \
   _NOPE_NOP_10CASES_DESC(15) \
   _NOPE_NOP_10CASES_DESC(14) \
   _NOPE_NOP_10CASES_DESC(13) \
   _NOPE_NOP_10CASES_DESC(12) \
   _NOPE_NOP_10CASES_DESC(11) \
   _NOPE_NOP_10CASES_DESC(10) \
   _NOPE_NOP_10CASES_DESC(9) \
   _NOPE_NOP_10CASES_DESC(8) \
   _NOPE_NOP_10CASES_DESC(7) \
   _NOPE_NOP_10CASES_DESC(6) \
   _NOPE_NOP_10CASES_DESC(5) \
   _NOPE_NOP_10CASES_DESC(4) \
   _NOPE_NOP_10CASES_DESC(3) \
   _NOPE_NOP_10CASES_DESC(2) \
   _NOPE_NOP_10CASES_DESC(1) \
   _NOPE_NOP_10CASES_DESC(0x) \
}

#define DO_UT150_NOPS(N) \
switch (N) {\
   _NOPE_NOP_CASE(150) \
   _NOPE_NOP_10CASES_DESC(14) \
   _NOPE_NOP_10CASES_DESC(13) \
   _NOPE_NOP_10CASES_DESC(12) \
   _NOPE_NOP_10CASES_DESC(11) \
   _NOPE_NOP_10CASES_DESC(10) \
   _NOPE_NOP_10CASES_DESC(9) \
   _NOPE_NOP_10CASES_DESC(8) \
   _NOPE_NOP_10CASES_DESC(7) \
   _NOPE_NOP_10CASES_DESC(6) \
   _NOPE_NOP_10CASES_DESC(5) \
   _NOPE_NOP_10CASES_DESC(4) \
   _NOPE_NOP_10CASES_DESC(3) \
   _NOPE_NOP_10CASES_DESC(2) \
   _NOPE_NOP_10CASES_DESC(1) \
   _NOPE_NOP_10CASES_DESC(0x) \
}

#ifdef __cplusplus
}
#endif
#endif /* NOPE_H */
/** @} */



