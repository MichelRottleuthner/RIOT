/**
 * Copyright (C) 2023 HAW Hamburg
 * 
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 *
 * @ingroup     sys_nope
 * @{
 * @file
 * @brief    NOP execution helper
 *
 * @author   Michel Rottleuthner <michel.rottleuthner@haw-hamburg.de>
 *
 * @}
 */

#include <stdint.h>
#include <stdio.h>
#include "nope.h"
#include "ztimer.h"

#define _REPEAT_10_TIMES(REP) REP REP REP REP REP REP REP REP REP REP
#define _REPEAT_100_TIMES(REP) _REPEAT_10_TIMES(_REPEAT_10_TIMES(REP))
#define _REPEAT_1000_TIMES(REP) _REPEAT_10_TIMES(_REPEAT_100_TIMES(REP))

#define REPEAT0(X)
#define REPEAT1(X)  X
#define REPEAT2(X)  REPEAT1(X) X
#define REPEAT3(X)  REPEAT2(X) X
#define REPEAT4(X)  REPEAT3(X) X
#define REPEAT5(X)  REPEAT4(X) X
#define REPEAT6(X)  REPEAT5(X) X
#define REPEAT7(X)  REPEAT6(X) X
#define REPEAT8(X)  REPEAT7(X) X
#define REPEAT9(X)  REPEAT8(X) X
#define REPEAT10(X) REPEAT9(X) X

#define REPEAT_HTO(HUNDREDS,TENS,ONES,X) \
  REPEAT##HUNDREDS(REPEAT10(REPEAT10(X))) \
  REPEAT##TENS(REPEAT10(X)) \
  REPEAT##ONES(X)

//#define _OP_CODE_NOP      (0xbf00) /* NOP */
#define _OP_CODE_NOP      (0x46c0) /* mov r8, r8 */
#define _OP_CODE_BX_LR    (0x4770) /* jump back to LR */

/* bx lr  (jump back to where the nop delay was called */ 
uint16_t ram_nops[NOPE_RAM_NOPS_MAX + 1]; 

#pragma GCC push_options
#pragma GCC optimize ("Ofast")
__attribute__((noinline)) void nope_iter_nops(uint32_t n)
{
    NOPE_DO_ITER_NOPS(n)
}

__attribute__((noinline)) void nope_lt32_switched_nops(uint32_t n) 
{
    DO_LT32UR_NOPS(n)
}

__attribute__((noinline)) void nope_ut32_switched_nops(uint32_t n) 
{
    DO_LT32UR_GE32_ITER_NOPS(n)
}
#pragma GCC pop_options

/* must be called once before using ram nop delay.
 * ram_nopy must be of size nops + 1 elements. */
void _init_ram_nops(uint16_t *ramnops, size_t nops)
{
    for (unsigned i = 0; i < nops; i++) {
        ramnops[i] = _OP_CODE_NOP;
    }
    ramnops[nops] = _OP_CODE_BX_LR;
}

const void * const ram_nops_addr = (void*)ram_nops;

void __attribute__((aligned(4),naked,noinline)) nope_rom_nops(void)
{
     __asm__(
             REPEAT_HTO(2,0,0, _NOP_ASM_STR)
             "bx lr\n"
    : /* no outputs */ 
    : /* no inputs */
    : /* no clobber */ 
    );
}    

#define DO_RAM_NOPS \
     __asm__( \
    "blx %0\n" \
    : /* no outputs */ \
    : [abc] "r" (ram_nops_addr + 1) /* inputs: address of ram_nops with thumb flag set (shall be placed in literal pool) */ \
    : "%0" \
    );

void __attribute__((aligned(4),naked,noinline)) nope_prepared_ram_nops(void)
{
     __asm__(
    "ldr pc, %[abc]\n"
    : /* no outputs */ 
    : [abc] "irm" (((void*)&ram_nops) + 1) /* inputs: address of ram_nops with thumb flag set (shall be placed in literal pool) */
    : /* no clobber */ 
    );
}
#if 0
/* Note: this could serve as an alterntive to the above direct PC loading on other platforms.
 * When tested, it showed the same speed as the above variant, but additionally clobbers one
 * register */ 
void __attribute__((aligned(4),naked,noinline)) nope_prepared_ram_nops(void)
{
     __asm__(
    "bx %0\n"
    : /* no outputs */ 
    : [abc] "r" (ram_nops_addr + 1) /* inputs: address of ram_nops with thumb flag set (shall be placed in literal pool) */
    : "%0" 
    );
}
#endif

void _nope_get_raw_nop_timing(uint32_t *ps_per_nop, uint32_t *ps_per_loop_iter)
{
    /* force reading value at runtime to avoid optimizations that 
     * isn't possible when used with variable value */
    volatile uint32_t nc = 1000000;
    uint32_t yiter = nc / 200;
    uint32_t cnt = yiter;

    int irqstate = irq_disable();
    uint32_t ts = ztimer_now(ZTIMER_USEC);
    do {
        DO_100_NOPS
        DO_100_NOPS
    } while (--cnt);
    uint32_t te = ztimer_now(ZTIMER_USEC);
    uint32_t y = (te - ts);
    
    /* volatile needed to ensure it is actually loaded from meory and doesn't assume
     * fixed value for further opimization */
    volatile uint32_t loops = 10000;
    uint32_t xiter = loops;
    cnt = loops;

    ts = ztimer_now(ZTIMER_USEC);
    do {
        DO_100_NOPS
    } while (--cnt);
    te = ztimer_now(ZTIMER_USEC);
    uint32_t x = (te - ts);
    irq_restore(irqstate);
    
    // x = xiter * loop_overhead + xiter * 100 NOP
    // y = yiter * loop_overhead + yiter * 200 NOP
    //
    // with 
    // loops = 10000
    // iter = 5000
    //
    // loop_overhead = (x - y)/5000
    // NOP = (2 y - x)/1000000
    // NOP = ((xiter/yiter)*y - x)/1000000
    // NOP = ((xiter/yiter)*y - x)/ yiter * x_nops
    uint32_t loop_overhead_ps = ((x - y) * 1000) / (yiter / 1000);
    uint32_t nop_dur_ps = ((xiter/yiter) * y - x);
    
    *ps_per_nop = nop_dur_ps;
    *ps_per_loop_iter = loop_overhead_ps;
}

uint32_t _nope_get_switched_nop_overhead(uint32_t ps_per_nop, uint32_t ps_per_loop_iter) {
    volatile uint32_t loops = 10000;
    
    volatile uint32_t bench_nops = 120;
    uint32_t bn = bench_nops;
    int cnt = loops;
    int irqstate = irq_disable();
    uint32_t ts = ztimer_now(ZTIMER_USEC);
    do {
        DO_UT200_NOPS(bn)
    } while(--cnt);
    uint32_t te = ztimer_now(ZTIMER_USEC);
    irq_restore(irqstate);
        
    uint32_t theoretical_time_us = ((loops * bench_nops) / 1000) * ps_per_nop / 1000;
    uint32_t call_overhead_sum_us = te - ts -  theoretical_time_us;
    uint32_t nop_call_overhead_ns = (call_overhead_sum_us * 1000 - (loops * ps_per_loop_iter / 1000)) / (loops / 1000);
    return nop_call_overhead_ns;
}
nope_var_nop_cb_t nop_cb = nope_iter_nops;

uint32_t _get_iter_nop_timing(uint32_t iterations, uint32_t nops) {
    int irqstate = irq_disable();
    uint32_t _nops = nops;  
    uint32_t _iters = iterations;  
    nope_var_nop_cb_t _nop_cb = nop_cb;
    uint32_t ts = ztimer_now(ZTIMER_USEC);
    while(_iters--) {
        _nop_cb(_nops);
    }
    uint32_t te = ztimer_now(ZTIMER_USEC);
    irq_restore(irqstate);
    uint32_t duration_us = (te - ts);
    return duration_us;
}

uint32_t _get_10iter_nop_timing(uint32_t iterations, uint32_t nops) {
    int irqstate = irq_disable();
    uint32_t _nops = nops;  
    uint32_t _iters = iterations;  
    nope_var_nop_cb_t _nop_cb = nop_cb;
    uint32_t ts = ztimer_now(ZTIMER_USEC);
    while(_iters--) {
        _nop_cb(_nops);
        _nop_cb(_nops);
        _nop_cb(_nops);
        _nop_cb(_nops);
        _nop_cb(_nops);
        _nop_cb(_nops);
        _nop_cb(_nops);
        _nop_cb(_nops);
        _nop_cb(_nops);
        _nop_cb(_nops);
    }
    uint32_t te = ztimer_now(ZTIMER_USEC);
    irq_restore(irqstate);
    uint32_t duration_us = (te - ts);
    return duration_us;
}

/* nops must be big enough to eliminiate call overhead of nope_iter_nops() */
uint32_t _get_duration_us(uint32_t nops) {
    int irqstate = irq_disable();
    uint32_t _nops = nops;  
    uint32_t ts = ztimer_now(ZTIMER_USEC);
    nope_iter_nops(_nops);
    uint32_t te = ztimer_now(ZTIMER_USEC);
    irq_restore(irqstate);
    uint32_t duration_us = (te - ts);
    return duration_us;
}

void _nope_get_iter_nop_timing(uint32_t *ps_per_nop, uint32_t *ps_per_nop_call, uint32_t repetitions) {
    // below calculations only work if smaller than 1000000 and bigger values will result in very long execution time
    assert(repetitions <= 1000000); 
    uint32_t rescale = 1000000 / repetitions;
    /* this is heavily dominated by the number of NOPs.
     * Therefore, it can be used to determine the time one NOP takes with reasonable accuracy. */
    uint32_t iterz = 1;
    volatile uint32_t znops = repetitions;
    uint32_t a = _get_iter_nop_timing(iterz, znops) * rescale;
    
    /* determine time if dominated by looping the nop call */
    iterz = repetitions;
    znops = 1;
    uint32_t b = _get_iter_nop_timing(iterz, znops) * rescale;
    
    /* determine time when dominated by looping NOP call but with multiple unrolled calls */
    uint32_t times10 = _get_10iter_nop_timing(iterz, znops) * rescale;
    
    *ps_per_nop = a; // znops is 1M -> therefore us to ps conversion can be dropped
    *ps_per_nop_call = (times10 - b) / 9; // - b removes the loop overhead and leaves only the nine calls
}

#define _MEASURE_LOOPED_OPERATION(RESULTVAR, LOOPS, _OP_TO_BE_MEASURED) \
    { \
    volatile uint32_t l = LOOPS; \
    uint32_t cnt = l; \
    int irqstate = irq_disable(); \
    uint32_t ts = ztimer_now(ZTIMER_USEC); \
    do { \
        _OP_TO_BE_MEASURED \
    } while(--cnt); \
    uint32_t te = ztimer_now(ZTIMER_USEC); \
    irq_restore(irqstate); \
    RESULTVAR = (te - ts); \
    }


uint32_t _nope_measure_looped_ram_nops(uint32_t loops, uint32_t nops)
{
    //_set_ram_nop_cnt(nops); NOT needed any more as fptr is determined dynamically
    nope_fixed_nop_cb_t nop_cb = NOPE_GET_ROM_NOP_FPTR(nops);
    uint32_t res;
    _MEASURE_LOOPED_OPERATION(res, loops, nop_cb(););
    return res;
}

void _nope_get_naked_rom_nop_timing(uint32_t *ps_per_nop, uint32_t *ps_per_nop_call)
{
    volatile uint32_t anops = 200;
    volatile uint32_t loops = 10000;
    nope_fixed_nop_cb_t nop_cb = NOPE_GET_ROM_NOP_FPTR(anops);

    uint32_t a;
    _MEASURE_LOOPED_OPERATION(a, loops, nop_cb(););
    
    uint32_t a10;
    _MEASURE_LOOPED_OPERATION(a10, loops,  
            nop_cb();
            nop_cb();
            nop_cb();
            nop_cb();
            nop_cb();
            nop_cb();
            nop_cb();
            nop_cb();
            nop_cb();
            nop_cb();
             );
    
    
    uint32_t bnops = anops / 2;
    nop_cb = NOPE_GET_ROM_NOP_FPTR(bnops);
    uint32_t b; 
    _MEASURE_LOOPED_OPERATION(b, loops, nop_cb(););

    /* b - a leaves only the time difference reltd to the NOPS (without any call/loop overhead, as that is the same for both runs).
     * As time is measured in us, 1M NOPS give the time in ps without furhter coversion. */
    *ps_per_nop = (a - b) / ((anops - bnops) * loops / 1000000);
    
    uint32_t u_us = ((a10 - a) / 9) ;
    uint32_t r_ns = u_us / (loops / 1000);
    uint32_t x = r_ns * 1000 - (anops * *ps_per_nop);
    *ps_per_nop_call = x; 
}

void _nope_get_prepared_ram_nop_timing(uint32_t *ps_per_nop, uint32_t *ps_per_nop_call) {

    uint32_t anops = 200;
    uint32_t loops = 10000;
    printf("RAM NOPS go from 0x%08lx to 0x%08lx\n", (uint32_t)&ram_nops[0], (uint32_t)&ram_nops[NOPE_RAM_NOPS_MAX]);
    _init_ram_nops(ram_nops, NOPE_RAM_NOPS_MAX);
   
    uint32_t a = _nope_measure_looped_ram_nops(loops, anops);
    
    nope_fixed_nop_cb_t nop_cb = NOPE_GET_ROM_NOP_FPTR(anops);
    
    uint32_t a10;
    _MEASURE_LOOPED_OPERATION(a10, loops,  
            nop_cb();
            nop_cb();
            nop_cb();
            nop_cb();
            nop_cb();
            nop_cb();
            nop_cb();
            nop_cb();
            nop_cb();
            nop_cb();
             );
    
    uint32_t bnops = anops / 2;
    uint32_t b = _nope_measure_looped_ram_nops(loops, bnops);

    /* b - a leaves only the time difference reltd to the NOPS (without any call/loop overhead, as that is the same for both runs).
     * As time is measured in us, 1M NOPS give the time in ps without furhter coversion. */
    *ps_per_nop = (a - b) / ((anops - bnops) * loops / 1000000);

    /* a   = loops * loopoverhead + loops * (calloverhead + anops * t_NOP) 
     * a10 = loops * loopoverhead + loops * (calloverhead + anops * t_NOP) * 10
     * p = a10 - a 
     * p = loops * (calloverhead + anops * t_NOP) * 9
     * u = p / 9
     *   = loops * (calloverhead + anops * t_NOP)
     *   = loops * calloverhead + loops * anops * t_NOP
     * r = u / loops
     *   = calloverhead + anops * t_NOP
     * x = r - anops * t_NOP
     *   = calloverhead
     * */
    uint32_t u_us = ((a10 - a) / 9) ;
    uint32_t r_ns = u_us / (loops / 1000);
    uint32_t x = r_ns * 1000 - (anops * *ps_per_nop);
    *ps_per_nop_call = x; 
}

void _nope_get_lt32ur_nop_timing(uint32_t *ps_per_iter_nop, uint32_t *ps_per_iter_call,
                                 uint32_t *ps_per_ur_nop, uint32_t *ps_per_ur_call) {
    /* this is heavily dominated by the number of NOPs.
     * Therefore, it can be used to determine the time one NOP takes with reasonable accuracy. */
    uint32_t iterz = 1;
    volatile uint32_t znops = ~1000000;
    uint32_t a = _get_iter_nop_timing(iterz, znops);
    *ps_per_iter_nop = a; // znops is 1M -> therefore us to ps conversion can be dropped
    
    /* speed up the measurement but keep scaling for ps resolution of the timing result. */
    uint32_t rescale = 100;

    iterz = 1000000 / rescale;
    znops = ~(NOPE_MAX_UNROLL_NOPS + 1); // must be bigger than max unroll
    uint32_t c = _get_iter_nop_timing(iterz, znops) * rescale;

    /* determine time when dominated by looping NOP call but with multiple unrolled calls */
    uint32_t times10 = _get_10iter_nop_timing(iterz, znops) * rescale;
    *ps_per_iter_call = (times10 - c) / 9 - ((NOPE_MAX_UNROLL_NOPS + 1) * a); // - c removes the loop overhead and leaves only the nine calls
    
    rescale = 10;
    iterz = 1000000 / rescale;
    volatile uint32_t bnops = 1;
    uint32_t b = _get_iter_nop_timing(iterz, bnops) * rescale;
    
    volatile uint32_t dnops = 20;
    uint32_t d = _get_iter_nop_timing(iterz, dnops) * rescale;
    *ps_per_ur_nop = (d - b) / (dnops - bnops); 
    
    /* determine time when dominated by looping NOP call but with multiple unrolled calls */
    times10 = _get_10iter_nop_timing(iterz, dnops) * rescale;
    *ps_per_ur_call = ((times10 - d) / (9)) - (dnops * *ps_per_ur_nop); // - b removes the loop overhead and leaves only the nine calls   
}

void nope_calibrate(nope_calibration_t *cal) {
    /* over head per iteration of  while(--) {} */
    uint32_t loop_overhead_ps = 0;
    cal->raw.call_ovhd_ps = 0;
    _nope_get_raw_nop_timing(&cal->raw.nop_ps, &loop_overhead_ps);

    _nope_get_prepared_ram_nop_timing(&cal->ram.nop_ps, &cal->ram.call_ovhd_ps);

    _nope_get_naked_rom_nop_timing(&cal->rom.nop_ps, &cal->rom.call_ovhd_ps);
     
    cal->switched.nop_ps = cal->raw.nop_ps;
    cal->switched.call_ovhd_ps = _nope_get_switched_nop_overhead(cal->raw.nop_ps, loop_overhead_ps);
    
    _nope_get_iter_nop_timing(&cal->iter.nop_ps, &cal->iter.call_ovhd_ps, 100000);

    nop_cb = nope_ut32_switched_nops;

    _nope_get_lt32ur_nop_timing(&cal->ge32aur.nop_ps, &cal->ge32aur.call_ovhd_ps,
                                &cal->lt32aur.nop_ps, &cal->lt32aur.call_ovhd_ps);
    nop_cb = nope_iter_nops;
}

void nope_print_calibration(nope_calibration_t *cal)
{
    printf("RAW        t_NOP: %6lu ps\n", cal->raw.nop_ps);
    printf("RAW       t_call: %6lu ps\n", cal->raw.call_ovhd_ps);
    printf("SWITCHED   t_NOP: %6lu ps\n", cal->switched.nop_ps);
    printf("SWITCHED  t_call: %6lu ps\n", cal->switched.call_ovhd_ps);
    printf("RAM        t_NOP: %6lu ps\n", cal->ram.nop_ps);
    printf("RAM       t_call: %6lu ps\n", cal->ram.call_ovhd_ps);
    printf("ROM        t_NOP: %6lu ps\n", cal->rom.nop_ps);
    printf("ROM       t_call: %6lu ps\n", cal->rom.call_ovhd_ps);
    printf("ITER       t_NOP: %6lu ps\n", cal->iter.nop_ps);
    printf("ITER      t_call: %6lu ps\n", cal->iter.call_ovhd_ps);
    printf("LT32AUR    t_NOP: %6lu ps\n", cal->lt32aur.nop_ps);
    printf("LT32AUR   t_call: %6lu ps\n", cal->lt32aur.call_ovhd_ps);
    printf("GE32AUR    t_NOP: %6lu ps\n", cal->ge32aur.nop_ps);
    printf("GE32AUR   t_call: %6lu ps\n", cal->ge32aur.call_ovhd_ps);

}
