/*
 * Copyright (C) 2023 HAW Hamburg
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_flashpatch
 * @{
 *
 * @file
 * @brief       Device driver implementation for the flashpatch
 *
 * @author      Michel Rottleuthner <michel.rottleuthner@haw-hamburg.de>
 *
 * @}
 */

#include <string.h>
#include <stdio.h>

#include "kernel_defines.h"
#include "flashpatch.h"
#include "flashpatch_constants.h"

#define FP_COMP_ENABLE                     (0x1)
#define FP_COMP_RESERVED_MASK              ((1 << 1) | (1 << 29))
#define FP_COMP_REMAP_REPLACE_REMAP_ADDR   (0x00 << 30)
#define FP_COMP_REMAP_REPLACE_BRKPT_LSB_00 (0x01 << 30)
#define FP_COMP_REMAP_REPLACE_BRKPT_LSB_10 (0x02 << 30)
#define FP_COMP_REMAP_REPLACE_BRKPT_LSB_X0 (0x03 << 30)

#define INSTRUCTION_PTR_THUMB_MARKER_BIT   (0x1)

#define WRAPPER_FUNCTION_PUSHED_REGS_NUMOF_PRE_POST_WRAPPER (6)
#define WRAPPER_FUNCTION_PUSHED_ADDITIONAL_STACK_PLUS (6)
#define CMP_IDX_HINT_FRAME_LOC (124)
#define WRAPPER_FUNCTION_PUSHED_REGS_NUMOF (1)

/* FPB peripheral memory layout. */
typedef struct {
    uint32_t FP_CTRL;
    uint32_t *FP_REMAP;
    uint32_t FP_COMPn[FLASHPATCH_ALL_COMPARATORS_MAX_NUMOF];
} fpb_peripheral_t;

#define FPB_DEV  ((fpb_peripheral_t*) FLASHPATCH_FPB_PERIPHERAL_BASE_ADDR)

typedef struct {
    const function_ptr_t wrapper;
    int * const cmp_idx_offset_ptr;
} function_wrapper_meta_t;

/* This is an ARMV7-M instruction that jumps to the instruction at
 * the address stored in the next word, by writing the next
 * address into the PC register. I.e.
 * asm("ldr.w	pc, [pc]").
 * It is used to build a flexible jump table with arbitrary
 * (and dynamically changeable) target addresses in to RAM. */
#define JUMP_TO_BELOW_ADDR_LDR_INSTRUCTION (0xF000F8DF)

/* This table will hold executable instructions, where each
 * even line contains machine code to jump to the address
 * that is stored at the next (odd) index after it.
 * This is done to perform a an arbitrary branch to a
 * dynamically changeable address *whithout* modifying
 * any register or the stack (to preserve all this
 * state for later forwarding to any kind of function).
 * The content will be initiaqlized to
 *  JUMP_TO_BELOW_ADDR_LDR_INSTRUCTION,
 *  NULL,
 *  JUMP_TO_BELOW_ADDR_LDR_INSTRUCTION,
 *  NULL,
 *  ... and so on.
 */
uint32_t _indirection_ptr_table[FLASHPATCH_INSTRUCTION_COMPARATORS_MAX_NUMOF*2];

static flashpatch_t _fpdata;
static flashpatch_function_wrapper_t function_wrappers[FLASHPATCH_ALL_COMPARATORS_MAX_NUMOF];

static volatile int _test_wrapper_called = 0;
void test_wrapper(void);

/* cmp_idx hint location offset for flashpatch_pre_post_cb_wrapper function */
static int _pre_post_cb_wrapper_cmp_hint_frame_offset = 0xFFFFFFFF;
static int _test_wrapper_cmp_hint_frame_offset = 0xFFFFFFFF;

static const function_wrapper_meta_t _function_wrapper_meta[] = {
    { .wrapper = flashpatch_pre_post_cb_wrapper, .cmp_idx_offset_ptr = &_pre_post_cb_wrapper_cmp_hint_frame_offset },
    { .wrapper = test_wrapper, .cmp_idx_offset_ptr = &_test_wrapper_cmp_hint_frame_offset },
};

static bool _fits_in_b_t2(uint32_t base_addr, uint32_t target_addr)
{
    int32_t offset = target_addr - base_addr;
    return (offset >= -2048) && (offset <= 2046);
}

static bool _is_32_bit_instruction(uint16_t instr_hw1)
{
    /* If instr_hw1[15:11] takes any of the following values, it is the first halfword of
     * a 32-bit instruction: 0b11101 0b11110 0b11111.
     * see ARMv7-M reference manual "A5.1 Thumb instruction set encoding" for details.*/
    uint16_t hw_15to11 = ((instr_hw1 >> 11));

    return (hw_15to11 == 0x1D) || (hw_15to11 == 0x1E) || (hw_15to11 == 0x1F);
}

bool _is_sub_sp_t1(uint32_t instr)
{
    return (instr >> 7) == 0x161;
}

uint8_t _sub_sp_t1__imm(uint16_t instr)
{
    assert(_is_sub_sp_t1(instr));
    uint32_t imm7 = instr & 0x7F;
    return imm7 << 2;
}

uint32_t _swap_hws(uint32_t instr)
{
    return ((instr & 0xFFFF) << 16) | (instr >> 16);
}

bool _is_push_t2(uint32_t instr)
{
    instr = _swap_hws(instr);
    return ((instr >> 15) == 0x1D25A) && (((instr >> 13) & 0x1) == 0);
}

uint8_t _push_t2__get_reg_cnt(uint32_t instr)
{
    assert(_is_push_t2(instr));
    instr = _swap_hws(instr);
    uint16_t zmzregister_list = instr & 0xFFFF;
    uint8_t reg_cnt = 0;
    for (unsigned i = 0; i < 15; i++) {
        zmzregister_list = zmzregister_list >> 1;
        reg_cnt += (zmzregister_list & 0x1);
    }
    return reg_cnt;
}

/* Calculate a branch instruction in T4 encoding (ARMv7-M) from
 * an instruction address and a new target address.
 * Max possible offsets are -16777216 to 16777214. */
static uint32_t _branch_instruction_t4(uint32_t instr_addr, uint32_t target_addr)
{
    int32_t offset = (target_addr - instr_addr) - 4;

    uint16_t imm10 = (offset >> 12) & 0x3FF;
    uint16_t imm11 = (offset >> 1)  & 0x7FF;

    uint8_t s = (target_addr < instr_addr);
    uint8_t i1 = (offset >> 23) & 0x1;
    uint8_t i2 = (offset >> 22) & 0x1;
    uint8_t j1 = i1 == s;
    uint8_t j2 = i2 == s;

    uint16_t thumb_hw1 = (0x1E << 11) | (s << 10) | imm10;
    uint16_t thumb_hw2 = (0x2 << 14) | (j1 << 13) | (0x1 << 12) | (j2 << 11) | imm11;

    /* instructions are stored as two halfwords (16 bit) with little endian.
       See Armv7-M Architecture Reference Manual (ID021621) page 86
       (A3.3.1 Control of endianness in Armv7-M)). */
    return thumb_hw1 | (thumb_hw2 << 16);
}

/* Calculate a branch instruction in T2 encoding (ARMv7-M) from
 * an instruction address and a new target address.
 * Max possible offsets are -2048 to 2046. */
static uint16_t _branch_instruction_t2(uint32_t instr_addr, uint32_t target_addr)
{
    int32_t offset = (target_addr - instr_addr) - 4;

    uint16_t imm11 = (offset >> 1)  & 0x7FF;

    uint16_t thumb_hw1 = (0x7 << 13) | imm11;

    return thumb_hw1;
}

uint32_t _target_addr_of_b_t2_instr_exploc(uint16_t *instruction, uint32_t *location_ptr)
{
    uint32_t instr_addr = (uint32_t)location_ptr;

    uint16_t imm11 = *instruction & 0x7FF;
    uint32_t imm32 = imm11 << 1;

    if (imm32 & (1 << 11)) {
        /* extend sign */
        imm32 |= ~((1 << 12) - 1);
    }

    uint32_t target_addr = instr_addr + imm32 + 4;
    return target_addr;
}

/* Convert a function pointer to a an address without
 * a thumb indication bit. */
static uint32_t _thumb_fptr2addr(function_ptr_t f)
{
    return (uint32_t)f & ~INSTRUCTION_PTR_THUMB_MARKER_BIT;
}

/* Get (both) comparator index(es) of a wrapped function.
 * Returns a bitwise combination of the comparator index(es) if remapping
 * was set up for the given address via @ref flashpatch_remap_function()
 *
 * @param[in]      remapped_addr  remapped address to get the cmp indexes
 * @param[in,out]  idx            memory to store the cmp indexes at
 *
 * @return  number of cmp indexes written to @p idx.
 * @return  <0 if no wrapper was set up for that function.
 */
static int _get_remaped_addr_comp_idxs(uint32_t remapped_addr, uint8_t *idx)
{
    int cnt = 0;
    for (unsigned i = 0; i < (_fpdata.instruction_cmp_cnt + _fpdata.literal_cmp_cnt); i++) {
        //TODO: for literal remap the LSB shall not be ignored!
        if ((_fpdata.remapped_addrs[i] & ~INSTRUCTION_PTR_THUMB_MARKER_BIT) ==
            (remapped_addr & ~INSTRUCTION_PTR_THUMB_MARKER_BIT)){
            idx[cnt++] = i;
            /* return the count on the first non-matching comparator */
        } else if (cnt) {
            return cnt;
        }
    }
    return 0;
}

/* WARNING: THIS IS HIGHLY EXPERIMENTAL!
 * This funciton Inspects the first instructions of the given wrapper function
 * to figure out how many bytes it pushes to the stack.
 * This is done to later locate a hint that is placed on the not (yet) allocated
 * part of the function stack (by the intermediate wrapper function).
 * This function assumes that the first thing the wrapper function does,
 * is performing the instructions generated by the gcc builtin
 * "__builtin_apply_args()" to capture all arguments that will later handed
 * to the wrapped function in its original form.
 * Changing your compiler, compiling options, doing something unexpected
 * in the wrapper function, may cause this function to report nonsense.
 * TODO: Implementing the whole wrapper function in pure assembly
 *       would make this much cleaner, as the intermediate shim
 *       function that calles the actual wrapper could then just push
 *       the information on the stack, which can then just be popped
 *       by the wrapper before doing its work. unfortunately, placing
 *       inline-assembly at the top of the wrapper function does not
 *       seem to have the desired effect when also using the compiler
 *       builtins. Those are placed at the top of the function, taking
 *       priority over any other inline assembly.
 */
static int _get_cmpidx_wrapper_frame_offset(function_ptr_t wrapper)
{
    uint32_t *first_instr = (uint32_t*)(((uint32_t)wrapper) & ~0x1);

    uint8_t pushed_reg_cnt = 0;
    if (_is_32_bit_instruction(first_instr[0]) && _is_push_t2(first_instr[0])) {
        pushed_reg_cnt = _push_t2__get_reg_cnt(first_instr[0]);
    } else {
        printf("ERROR: unexpected instruction 0x%08lx @ %p %s\n", first_instr[0], &first_instr[0],
               "expected a 32-bit (PUSH/STMDB T2) instruction\n");
    }

    uint8_t additional_stack_byte_cnt = 0;
    if (!_is_32_bit_instruction(first_instr[1]) && _is_sub_sp_t1(first_instr[1] & 0xFFFF)) {
        additional_stack_byte_cnt = _sub_sp_t1__imm(first_instr[1] & 0xFFFF);
    } else {
        printf("ERROR: unexpected instruction 0x%08lx @ %p %s\n", first_instr[1], &first_instr[1],
               "expected a 16-bit (SUB SP T1) instruction\n");
    }

    int word_offset = -((CMP_IDX_HINT_FRAME_LOC/4 - (pushed_reg_cnt + (additional_stack_byte_cnt/4)) + 1));
    return word_offset;
}


int flashpatch_init(void)
{
    uint32_t ctrl = FPB_DEV->FP_CTRL;
    int rev = (ctrl >> 28) & 0xF;
    if (rev == 1) {
        printf("Flash patch not implemented on FPB ver. 2!\n");
        return -1;
    }

    _fpdata.instruction_cmp_cnt = ((ctrl >> 4) & 0xF) | (((ctrl >> 12) & 0x7) << 4);
    _fpdata.literal_cmp_cnt = ((ctrl >> 8) & 0xF);

    unsigned all_cmp_cnt = _fpdata.instruction_cmp_cnt + _fpdata.literal_cmp_cnt;

    assert((uint32_t)_fpdata.remap_table % 32 == 0);
    FPB_DEV->FP_REMAP = _fpdata.remap_table;

    /* On init, disable the FPB block and all its comparators.
     * The FPB peripheral can keep it's state across (soft) resets, therefore,
     * all comparators must be disabled exlicitly to avoid the remapping of
     * functions to take effect before set up explicitly again */
    flashpatch_disable();

    for (unsigned i = 0; i < all_cmp_cnt; i++) {
        flashpatch_comparator_disable(i);

        /* initialize data structures for generic function wrappers
         * which are only usable with instruction comparators */
        if (i < _fpdata.instruction_cmp_cnt) {
            function_wrappers[i].wrapped_function = NULL;
            function_wrappers[i].wrapper_function = NULL;
            /* initialize the dynamic jump table */
            _indirection_ptr_table[i*2] = JUMP_TO_BELOW_ADDR_LDR_INSTRUCTION;
            _indirection_ptr_table[i*2+1] = 0;
        }
    }

    return 0;
}

/* calculates the number of comparators needed for
 * a particular instruction remapping case. */
static unsigned _cmpcnt_needed_for_instr_remap(function_ptr_t old, function_ptr_t new)
{
    uint32_t old_addr = _thumb_fptr2addr(old);
    uint32_t new_addr = _thumb_fptr2addr(new);
    /* any word aligned address can be remapped with a single comparator.
     * For other addresses (halfword-aligned but *not* word aligned)
     * the only way to fit the remapping into one comparator is by
     * fitting it into a single halfword instruction. */
    if ((old_addr % 4 == 0) || _fits_in_b_t2(old_addr, new_addr)) {
        return 1;
    }

    return 2;
}

/* Below code is to generate a bunch of rather nasty shim functions intended
 * to inject information about the specific comparator used for a particular
 * instruction remapping.
 * WARNING: DO NOT EVEN TOUCH THIS WITH A STICK.
 *
 * To enable generic wrapping/decoration of any function during runtime
 * some things must be achieved.
 * (1) The wrapped function shall be called with its original arguments.
 * (2) The wrapper must know the wrapped function to be able to call it.
 *
 * In case a specific wrapper is only ever used for a particular function,
 * property (2) is easy to achive due to the trivial 1-to-1 mapping of
 * wrapper and wrapped funciton. E.g., when setting up the wrapper, its
 * specific function pointer is set to the wrapped function, which is then
 * just called by the wrapper.
 *
 * However, if the same wraper function is used for multiple functions,
 * ensuring (2) is not that easy.
 * One way would be to use the return address to inspect the function
 * that was executed before. I.e., the instruction that lead to the wrapper
 * being called in the forst place. This works fine for basic instructions
 * like a plain branch to the original function. However, there are
 * virtually countless ways on how the programm counter ended up at the
 * instruction to be replaced, rendering decoding of those instructions
 * way to complex, and in many cases impossible.
 *
 * An alternative way is to use comparator-specific shim funcitons that
 * inject the needed information about the comparator into the stack
 * or a register that can then be inspected by the wrapper to determine
 * the wrapped function.
 *
 * As any modification to the stack could interfere with stack-allocated,
 * arguments the information is to an unallocated location on the stack
 * that is easily determined by the wrapper.
 */
#define INTERMEDIATE_REDIRECT_FUNCTION_NAME(X) _intermediate_redirect_func##X

void __attribute__((aligned (4))) _intermediate_redirect_func(void)
{
    __asm__(
    "nop\n"
    "push {lr}\n"          // save real return address on stack
    "add.w lr, pc, #5\n"   // (1) update the LR to return to (3)
    "ldr.w pc, [pc, #8]\n" // (2) jump to the wrapper function stored at (5)
    "pop {lr}\n"           // (3) restore real return address
    "nop\n"
    "bx lr\n"              // (4) return
    ".word 0x08001efd\n"   // (5) function ptr used by (2)
    );
}

#define DEFINE_INTERMEDIATE_REDIRECT_FUNCTION(IDX) \
void __attribute__((aligned (4), used)) INTERMEDIATE_REDIRECT_FUNCTION_NAME(IDX)(void)\
{\
     __asm( \
    "push {r0}\n" /* save r0 content */\
    "mov r0, %[cmp_idx_num]\n" /* load the comparator index specific to this shim */\
    "str r0, [sp, %[cmp_hint_offset]]\n" /* store the cmp idx at not yet used stack location */\
    "pop {r0}\n" /* restore r0 */\
    "ldr pc, [pc, #0]\n" /* jump to the specific redirect instruction of this function*/\
    ".word %[redirectTarget] + 0x1\n" /* jump to the specific redirect instruction of this function*/\
    : /* No outputs */\
    : [redirectTarget] "X" (_indirection_ptr_table[IDX*2]),\
      [cmp_hint_offset] "X" (-CMP_IDX_HINT_FRAME_LOC),\
      [cmp_idx_num] "X" (IDX)\
    : /* No clobber */\
    );\
}

DEFINE_INTERMEDIATE_REDIRECT_FUNCTION(0);
DEFINE_INTERMEDIATE_REDIRECT_FUNCTION(1);
DEFINE_INTERMEDIATE_REDIRECT_FUNCTION(2);
DEFINE_INTERMEDIATE_REDIRECT_FUNCTION(3);
DEFINE_INTERMEDIATE_REDIRECT_FUNCTION(4);
DEFINE_INTERMEDIATE_REDIRECT_FUNCTION(5);

static function_ptr_t _intermediate_redirect_funcs[FLASHPATCH_INSTRUCTION_COMPARATORS_MAX_NUMOF] = {
   INTERMEDIATE_REDIRECT_FUNCTION_NAME(0),
   INTERMEDIATE_REDIRECT_FUNCTION_NAME(1),
   INTERMEDIATE_REDIRECT_FUNCTION_NAME(2),
   INTERMEDIATE_REDIRECT_FUNCTION_NAME(3),
   INTERMEDIATE_REDIRECT_FUNCTION_NAME(4),
   INTERMEDIATE_REDIRECT_FUNCTION_NAME(5),
};

int flashpatch_remap_function_intermediate(function_ptr_t old, function_ptr_t new, unsigned cmp_idx)
{
    uint32_t old_addr = _thumb_fptr2addr(old);

    /* mark at least this one comparator for this address */
    _fpdata.remapped_addrs[cmp_idx] = old_addr;

    /* setup an intermediate indirection in order to inject comparator specific metadata
     * to let the wrapper identify the function that was originally called */
    _indirection_ptr_table[cmp_idx*2+1] = (uint32_t)new;

    assert(cmp_idx < _fpdata.instruction_cmp_cnt);
    uint32_t intermediate_redirect_fun_addr = (uint32_t)_intermediate_redirect_funcs[cmp_idx] & ~0x1;

    /* instruction to be remapped is word-aligned */
    if (old_addr % 4 == 0) {
        _fpdata.remap_table[cmp_idx] = _branch_instruction_t4(old_addr, intermediate_redirect_fun_addr);

        FPB_DEV->FP_COMPn[cmp_idx] = (FP_COMP_REMAP_REPLACE_REMAP_ADDR |
                old_addr | FP_COMP_ENABLE) &
            ~FP_COMP_RESERVED_MASK;
    } else {
        /* in case the instruction to be replaced is 2-aligned, there are several things to
         * consider for the remapping:
         * (A) The instruction to be replaced may be either 16 Bit or 32 bit.
         *
         * (B) The new target address is close enough to the replaced instruction to use
         *     a branch instruction with T2 encoding. The T2 encoded branch only uses a single
         *     halfword, thus, only this one halfword will be remapped (and the other halfword
         *     *must* keep its original value).
         * (C) The new target address requires a bigger offset, therefore a T4 instruction is
         *     needed. With the instruction to be replaced being 2-aligned (and not 4-aligned),
         *     One comparator would only cover one halfword of the instruction.
         *     Therefore, to "inject" a full 32-bit Instruction, two comparators are needed.
         */

        /* halfword-wise access to the word-aligned instruction that contains the instruction
         * to be replaced. */
        uint16_t *old_hw = (uint16_t*)(old_addr & ~0x3);

        bool is32bit = _is_32_bit_instruction(old_hw[1]);
        bool fitsbt2 = _fits_in_b_t2(old_addr, intermediate_redirect_fun_addr);

        if (is32bit) {
            printf("WARNING! instruction @0x%08lx not word-aligned but 32 bit!\n", old_addr);
        }

        if (!fitsbt2) {
            printf("WARNING! instruction @0x%08lx not word-aligned and won't fit 16 bit!\n", old_addr);
        }

        /* If a 32 bit instruction is needed */
        if (is32bit || !fitsbt2) {
            /* if two cmps are needed but there is no cmp after the given one or
             * the one after is already taken for another remap */
            if ((cmp_idx >= (_fpdata.instruction_cmp_cnt - 1)) || (_fpdata.remapped_addrs[cmp_idx+1])) {
                return 0;
            }
            uint32_t new_instruction = _branch_instruction_t4(old_addr, intermediate_redirect_fun_addr);

            _fpdata.remap_table[cmp_idx]   = (new_instruction & 0xFFFF) << 16 | old_hw[0];
            _fpdata.remap_table[cmp_idx+1] = (old_hw[3] << 16) | (new_instruction >> 16);

            FPB_DEV->FP_COMPn[cmp_idx] = (FP_COMP_REMAP_REPLACE_REMAP_ADDR |
                    (old_addr - 2) | FP_COMP_ENABLE) &
                ~FP_COMP_RESERVED_MASK;

            /* also mark the second comparator used */
            _fpdata.remapped_addrs[cmp_idx+1] = old_addr;
            FPB_DEV->FP_COMPn[cmp_idx+1] = (FP_COMP_REMAP_REPLACE_REMAP_ADDR |
                    (old_addr + 2) | FP_COMP_ENABLE) &
                ~FP_COMP_RESERVED_MASK;
            return 2;
        } else {
            /* A 16 bit B T2 instruction is sufficient */
            uint16_t new_instruction = _branch_instruction_t2(old_addr, intermediate_redirect_fun_addr);

            _fpdata.remap_table[cmp_idx]   = new_instruction << 16 | old_hw[0];

            FPB_DEV->FP_COMPn[cmp_idx] = (FP_COMP_REMAP_REPLACE_REMAP_ADDR |
                    (old_addr - 2) | FP_COMP_ENABLE) &
                ~FP_COMP_RESERVED_MASK;
        }
    }

    return 1;
}

int flashpatch_remap_function(function_ptr_t old, function_ptr_t new, unsigned cmp_idx)
{
    uint32_t old_addr = _thumb_fptr2addr(old);
    uint32_t new_target_addr = _thumb_fptr2addr(new);

    /* mark at least this one comparator for this address */
    _fpdata.remapped_addrs[cmp_idx] = old_addr;

    /* instruction to be remapped is word-aligned */
    if (old_addr % 4 == 0) {
        _fpdata.remap_table[cmp_idx] = _branch_instruction_t4(old_addr, new_target_addr);
        FPB_DEV->FP_COMPn[cmp_idx] = (FP_COMP_REMAP_REPLACE_REMAP_ADDR |
                old_addr | FP_COMP_ENABLE) &
            ~FP_COMP_RESERVED_MASK;
    } else {
        /* in case the instruction to be replaced is 2-aligned, there are several things to
         * consider for the remapping:
         * (A) The instruction to be replaced may be either 16 Bit or 32 bit.
         *
         * (B) The new target address is close enough to the replaced instruction to use
         *     a branch instruction with T2 encoding. The T2 encoded branch only uses a single
         *     halfword, thus, only this one halfword will be remapped (and the other halfword
         *     *must* keep its original value).
         * (C) The new target address requires a bigger offset, therefore a T4 instruction is
         *     needed. With the instruction to be replaced being 2-aligned (and not 4-aligned),
         *     One comparator would only cover one halfword of the instruction.
         *     Therefore, to "inject" a full 32-bit Instruction, two comparators are needed.
         */

        /* halfword-wise access to the word-aligned instruction that contains the instruction
         * to be replaced. */
        uint16_t *old_hw = (uint16_t*)(old_addr & ~0x3);

        bool is32bit = _is_32_bit_instruction(old_hw[1]);
        bool fitsbt2 = _fits_in_b_t2(old_addr, new_target_addr);

        if (is32bit) {
            printf("WARNING! instruction @0x%08lx not word-aligned but 32 bit!\n", old_addr);
        }

        if (!fitsbt2) {
            printf("WARNING! instruction @0x%08lx not word-aligned and won't fit 16 bit!\n", old_addr);
        }

        // TODO: there is another case where the old instruction is 32 bit
        //       but it can be replaced with a 16 bit instruction (saving a comparator)
        /* If a 32 bit instruction is needed */
        if (is32bit || !fitsbt2) {
            /* if two cmps are needed but there is no cmp after the given one or
             * the one after is already taken for another remap */
            if ((cmp_idx >= (_fpdata.instruction_cmp_cnt - 1)) || (_fpdata.remapped_addrs[cmp_idx+1])) {
                return 0;
            }
            uint32_t new_instruction = _branch_instruction_t4(old_addr, new_target_addr);

            _fpdata.remap_table[cmp_idx]   = (new_instruction & 0xFFFF) << 16 | old_hw[0];
            _fpdata.remap_table[cmp_idx+1] = (old_hw[3] << 16) | (new_instruction >> 16);

            FPB_DEV->FP_COMPn[cmp_idx] = (FP_COMP_REMAP_REPLACE_REMAP_ADDR |
                    (old_addr - 2) | FP_COMP_ENABLE) &
                ~FP_COMP_RESERVED_MASK;

            /* also mark the second comparator used */
            _fpdata.remapped_addrs[cmp_idx+1] = old_addr;
            FPB_DEV->FP_COMPn[cmp_idx+1] = (FP_COMP_REMAP_REPLACE_REMAP_ADDR |
                    (old_addr + 2) | FP_COMP_ENABLE) &
                ~FP_COMP_RESERVED_MASK;
            return 2;
        } else {
            /* A 16 bit B T2 instruction is sufficient */
            uint16_t new_instruction = _branch_instruction_t2(old_addr, _thumb_fptr2addr(new));

            _fpdata.remap_table[cmp_idx]   = new_instruction << 16 | old_hw[0];

            FPB_DEV->FP_COMPn[cmp_idx] = (FP_COMP_REMAP_REPLACE_REMAP_ADDR |
                    (old_addr - 2) | FP_COMP_ENABLE) &
                ~FP_COMP_RESERVED_MASK;
        }
    }

    return 1;
}

int flashpatch_delete_remap(function_ptr_t remapped_function)
{
    uint8_t cmp_idxs[2];
    int cnt = _get_remaped_addr_comp_idxs((uint32_t)remapped_function, &cmp_idxs[0]);

    for (int i = 0; i < cnt; i++) {
        flashpatch_comparator_disable(cmp_idxs[i]);
        _fpdata.remapped_addrs[cmp_idxs[i]] = 0;
    }

    return cnt;
}

int flashpatch_wrap_function(function_ptr_t wrapped_function, function_ptr_t wrapper_function, void *ctx)
{
    unsigned cmpcnt = _cmpcnt_needed_for_instr_remap(wrapped_function, wrapper_function);
    for (unsigned i = 0; i < FLASHPATCH_INSTRUCTION_COMPARATORS_MAX_NUMOF; i++) {
        if ((_fpdata.remapped_addrs[i] == 0) &&
                ((cmpcnt == 1) ||
                 ((_fpdata.instruction_cmp_cnt > i) && (_fpdata.remapped_addrs[i+1] == 0)))) {

            function_wrappers[i].wrapped_function = wrapped_function;
            function_wrappers[i].wrapper_function = wrapper_function;
            function_wrappers[i].ctx = ctx;

            bool found = false;
            for (unsigned x = 0; x < ARRAY_SIZE(_function_wrapper_meta); x++) {
                if (_function_wrapper_meta[x].wrapper == wrapper_function) {
                    int cmpidx_hint_word_offset = _get_cmpidx_wrapper_frame_offset(wrapper_function);
                    *(_function_wrapper_meta[x].cmp_idx_offset_ptr) = cmpidx_hint_word_offset;
                    found = true;
                    break;
                }
            }

            if (!found) {
                printf("ERROR: can not initialize wrapper metadata!\n");
            }

            flashpatch_remap_function_intermediate(wrapped_function, wrapper_function, i);
            return cmpcnt;
        }
    }
    return 0;
}

int flashpatch_remove_function_wrapper(void *wrapped_function)
{
    uint8_t cmp_idxs[2];
    int cnt = _get_remaped_addr_comp_idxs((uint32_t)wrapped_function, &cmp_idxs[0]);

    for (int i = 0; i < cnt; i++) {
        flashpatch_comparator_disable(cmp_idxs[i]);
        _fpdata.remapped_addrs[cmp_idxs[i]] = 0;
        function_wrappers[cmp_idxs[i]].wrapped_function = NULL;
        function_wrappers[cmp_idxs[i]].wrapper_function = NULL;
        function_wrappers[cmp_idxs[i]].ctx = NULL;
    }

    return cnt;
}

void flashpatch_enable(void)
{
    FPB_DEV->FP_CTRL = 0x03UL;
}

void flashpatch_disable(void) {
    FPB_DEV->FP_CTRL = 0x02UL;
}

void flashpatch_comparator_enable(unsigned idx)
{
    FPB_DEV->FP_COMPn[idx] |= 0x1;
}

void flashpatch_comparator_disable(unsigned idx)
{
    FPB_DEV->FP_COMPn[idx] &= ~0x1;
}

/* Disable the the comparator at the given index and the next one
 * if it is also used for the same address (as is the case for unaligned
 * instructions that occupy two comparators. */
static void _flashpatch_comparators_disable_chained(uint8_t cmp_idx)
{
    flashpatch_comparator_disable(cmp_idx);

    if ((cmp_idx < (_fpdata.instruction_cmp_cnt + _fpdata.literal_cmp_cnt)) &&
        (_fpdata.remapped_addrs[cmp_idx] == _fpdata.remapped_addrs[cmp_idx+1])) {
        flashpatch_comparator_disable(cmp_idx + 1);
    }
}

/* same as above but with enable */
static void _flashpatch_comparators_enable_chained(uint8_t cmp_idx)
{
    flashpatch_comparator_enable(cmp_idx);

    if ((cmp_idx < (_fpdata.instruction_cmp_cnt + _fpdata.literal_cmp_cnt)) &&
        (_fpdata.remapped_addrs[cmp_idx] == _fpdata.remapped_addrs[cmp_idx+1])) {
        flashpatch_comparator_enable(cmp_idx + 1);
    }
}

void test_wrapper(void)
{
    void *args = __builtin_apply_args();

    /* get the cmp idx hint from the precalculated offset location on the stack */
    int cmp_idx = (((uint32_t*)__builtin_frame_address(0)) + _test_wrapper_cmp_hint_frame_offset)[0];

    assert((cmp_idx >= 0) && ((unsigned)cmp_idx < _fpdata.instruction_cmp_cnt));

    function_ptr_t func = function_wrappers[cmp_idx].wrapped_function;

    _flashpatch_comparators_disable_chained(cmp_idx);
    void *result = __builtin_apply(func, args, FLASHPATCH_STACK_ALLOCATED_AGR_BYTES);
    _flashpatch_comparators_enable_chained(cmp_idx);
    _test_wrapper_called++;
    __builtin_return(result);
}

void pre_function(void *ctx) {
    int *ppc = (int*)ctx;
    ppc[0]++;
}

void post_function(void *ctx) {
    int *ppc = (int*)ctx;
    ppc[1]++;
}

void flashpatch_pre_post_cb_wrapper(void)
{
    void *args = __builtin_apply_args();

    /* get the cmp idx hint from the precalculated offset location on the stack */
    int cmp_idx = (((uint32_t*)__builtin_frame_address(0)) + _pre_post_cb_wrapper_cmp_hint_frame_offset)[0];

    assert((cmp_idx >= 0) && ((unsigned)cmp_idx < _fpdata.instruction_cmp_cnt));

    function_ptr_t func = function_wrappers[cmp_idx].wrapped_function;

    flashpatch_ppcb_wrapper_ctx_t *ppctx =
        (flashpatch_ppcb_wrapper_ctx_t*)function_wrappers[cmp_idx].ctx;

    /* temporarily disable the responsible comparator(s) to avoid recursive remapping.
     * The FPB feature is normally intended to be used for replacing buggy functions.
     * However, we want to wrap the function (i.e., at some point also calling the
     * original function) - that implies that the remapping must be
     * temporarily disabled. Otherwise, this call would also be remapped to this
     * wrapper, causing an infinite reucursion. */
    _flashpatch_comparators_disable_chained(cmp_idx);
    ppctx->pre(ppctx->cb_ctx);
    void *result = __builtin_apply(func, args, FLASHPATCH_STACK_ALLOCATED_AGR_BYTES);
    ppctx->post(ppctx->cb_ctx);
    _flashpatch_comparators_enable_chained(cmp_idx);
    __builtin_return(result);
}

volatile int _some_vola = 0;
void __attribute__((aligned(2), noinline)) test_void__void(void)
{
    _some_vola = _some_vola * 2;
}

int __attribute__((aligned(32), noinline)) test_int__int(int a) {
    return a + a;
}

int __attribute__((aligned(16), noinline)) test_int__int_int_int(int a, int b, int c) {
    return a + b + c;
}

int __attribute__((aligned(8), noinline)) test_int__4int(int a, int b, int c, int d) {
    return a + b + c + d;
}

int __attribute__((aligned(4), noinline)) test_int__5int(int a, int b, int c, int d, int e) {
    return a + b + c + d + e;
}

int __attribute__((aligned(4), noinline)) test_int__5int_3_str(int a, int b,
          int c, int d, int e, const char *x, const char *y, const char *z) {
    return a + b + c + d + e + strlen(x) + strlen(y) + strlen(z);
}

void __attribute__((aligned(1), noinline)) test_void__int_aligned1(int arg)
{
    (void)arg;
    __asm__(
            "nop\n"
           );
}

void __attribute__((aligned(1), noinline)) test_void__int_aligned2(int arg)
{
    (void)arg;
    __asm__(
            "nop\n"
            "nop\n"
           );
}

void __attribute__((aligned(1), noinline)) test_void__int_aligned3(int arg)
{
    (void)arg;
    __asm__(
            "nop\n"
            "nop\n"
            "nop\n"
           );
}

static int _replacemnt_for_aligned_call_cnt = 0;

void __attribute__((aligned(4), noinline)) _replacemnt_for_aligned(int arg)
{
    _some_vola = arg;
    _replacemnt_for_aligned_call_cnt++;
}

typedef void(*void__int_ptr_t)(int);

void__int_ptr_t pot2alignedfuncs[] = {
    test_void__int_aligned1,
    test_void__int_aligned2,
    test_void__int_aligned3,
};

static int _pp_call_cnt[2] = {0};

flashpatch_ppcb_wrapper_ctx_t pre_post_cb_ctx = {
    .pre = pre_function,
    .post = post_function,
    .cb_ctx = _pp_call_cnt,
};

static bool _test_remove_wrapper(function_ptr_t fun, int expected_cnt) {
    int removed = flashpatch_remove_function_wrapper(fun);
    int removed2 = flashpatch_remove_function_wrapper(fun);
    bool success = (removed == expected_cnt) && (removed2 == 0);
    printf("remove wrapper of %p: [%s]\n", fun, success ? "OK" : "ERROR");
    return success;
}

int flashpatch_test(void) {
    if (flashpatch_init() != 0) {
        printf("flashpatch_init failed!\n");
    }

    flashpatch_enable();

    void__int_ptr_t two_aligned_func = NULL;

    bool tested_function_aligned2 = false;
    for (unsigned i = 0; i < ARRAY_SIZE(pot2alignedfuncs); i++) {
        if (((uint32_t)pot2alignedfuncs[i] & ~INSTRUCTION_PTR_THUMB_MARKER_BIT) % 4 == 2) {
            two_aligned_func = pot2alignedfuncs[i];
            tested_function_aligned2 = true;
            _replacemnt_for_aligned_call_cnt = 0;
            int cmps_used = flashpatch_remap_function((function_ptr_t)two_aligned_func, (function_ptr_t)_replacemnt_for_aligned, 0);
            two_aligned_func(42);
            printf("remapping %p: [%s]\n", test_void__void,
                    _replacemnt_for_aligned_call_cnt == 1 ? "OK" : "ERROR");
            int cmps_freed = flashpatch_delete_remap((function_ptr_t)two_aligned_func);
            if ((cmps_used >0) && (cmps_used != cmps_freed)) {
                printf("ERROR: used %d comparators but only freed %d\n", cmps_used, cmps_freed);
            }
        }
    }

    if (!tested_function_aligned2) {
        printf("WARNING: can not perform test for 2-aligned-functions\n");
    }

    _test_wrapper_called = 0;

    flashpatch_wrap_function((function_ptr_t)test_void__void, test_wrapper, 0);
    int cmp_cnt_w1 = 1;

    test_void__void();

    printf("wrapping %p: [%s]\n", test_void__void, _test_wrapper_called == 1 ? "OK" : "ERROR");

    int cmp_cnt_w2 = flashpatch_wrap_function((function_ptr_t)test_int__int, test_wrapper, NULL);

    printf("calling wrpd(%p): [%s]\n", test_int__int, test_int__int(521) == (521*2) ? "OK" : "ERROR");
    printf("wrapping %p: [%s]\n", test_int__int, _test_wrapper_called == 2 ? "OK" : "ERROR");

    printf("calling wrpd(%p): [%s]\n", test_int__int, test_int__int(2500) == (2500*2) ? "OK" : "ERROR");
    printf("wrapping %p: [%s]\n", test_int__int, _test_wrapper_called == 3 ? "OK" : "ERROR");

    volatile int a = 1;
    volatile int b = 2;
    volatile int c = 3;
    volatile int d = 4;
    volatile int e = 5;
    volatile int f = 6;
    volatile int g = 7;
    volatile int h = 8;

    int res1 = test_int__int_int_int(a, b, c);

    int cmp_cnt_w3 = flashpatch_wrap_function((function_ptr_t)test_int__int_int_int, test_wrapper, NULL);

    printf("calling wrpd(%p): [%s]\n", test_int__int_int_int, test_int__int_int_int(a, b, c) == res1 ? "OK" : "ERROR");
    printf("wrapping %p: [%s]\n", test_int__int_int_int, _test_wrapper_called == 4 ? "OK" : "ERROR");

    int res2 = test_int__4int(b, c, d, e);
    int res3 = test_int__5int(c, d, e, f, g);

    int cmp_cnt_w4 = flashpatch_wrap_function((function_ptr_t)test_int__4int, test_wrapper, NULL);
    int cmp_cnt_w5 = flashpatch_wrap_function((function_ptr_t)test_int__5int, test_wrapper, NULL);

    printf("calling wrpd(%p): [%s]\n", test_int__4int, test_int__4int(b, c, d, e) == res2 ? "OK" : "ERROR");
    printf("wrapping %p: [%s]\n", test_int__4int, _test_wrapper_called == 5 ? "OK" : "ERROR");

    printf("calling wrpd(%p): [%s]\n", test_int__5int, test_int__5int(c, d, e, f, g) == res3 ? "OK" : "ERROR");
    printf("wrapping %p: [%s]\n", test_int__5int, _test_wrapper_called == 6 ? "OK" : "ERROR");

    int res4 = test_int__5int_3_str(d, e, f, g, h, "abc", "def", "ghi");

    int cmp_cnt_w6 = flashpatch_wrap_function((function_ptr_t)test_int__5int_3_str,
            flashpatch_pre_post_cb_wrapper, &pre_post_cb_ctx);

    int res5 = test_int__5int_3_str(d, e, f, g, h, "abc", "def", "ghi");
    printf("calling wrpd(%p): [%s]\n", test_int__5int_3_str, res5 == res4 ? "OK" : "ERROR");
    printf("wrapping %p: [%s]\n", test_int__5int,
            (_pp_call_cnt[0] == 1) && (_pp_call_cnt[1] == 1) ?
            "OK" : "ERROR");

    int before = _test_wrapper_called;

    _test_remove_wrapper((function_ptr_t)test_void__void, cmp_cnt_w1);
    _test_remove_wrapper((function_ptr_t)test_int__int, cmp_cnt_w2);
    _test_remove_wrapper((function_ptr_t)test_int__int_int_int, cmp_cnt_w3);
    _test_remove_wrapper((function_ptr_t)test_int__4int, cmp_cnt_w4);
    _test_remove_wrapper((function_ptr_t)test_int__5int, cmp_cnt_w5);
    _test_remove_wrapper((function_ptr_t)test_int__5int_3_str, cmp_cnt_w6);

    test_void__void();
    test_int__int_int_int(1, 2, 3);
    test_int__4int(2, 3, 4, 5);
    test_int__5int(3, 4, 5, 6, 7);
    test_int__5int_3_str(4, 5, 6, 7, 8, "abc", "def", "ghi");

    printf("all wrappers inactive: [%s]\n", before == _test_wrapper_called ? "OK" : "ERROR");

    flashpatch_disable();

    return 0;
}
