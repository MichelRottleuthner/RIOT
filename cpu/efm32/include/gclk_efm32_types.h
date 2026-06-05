/*
 * Copyright (C) 2021 HAW Hamburg <michel.rottleuthner@haw-hamburg.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */
/**
 * @ingroup     cpu_efm32
 *
 * @{
 *
 * @file
 * @brief       EFM32 specific clock types for the gclock interface
 *
 * @author      Michel Rottleuthner <michel.rottleuthner@haw-hamburg.de>
 */
#ifndef EFM32_GCLK_TYPES_H
#define EFM32_GCLK_TYPES_H

#include "gclk.h"
#include "list.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Custom descriptor for separate select and status config registers.
 *
 * Used for muxes that have one register to read the current config from (status) and one to write
 * the new config (status)
 */
typedef struct {
  uint32_t select_reg_idx:         GCLK_CONF_REG_IDX_BITWIDTH;
  uint32_t status_reg_idx:         GCLK_CONF_REG_IDX_BITWIDTH;
  uint32_t select_msb:             5;
  uint32_t select_lsb:             5;
  uint32_t status_msb:             5;
  uint32_t status_lsb:             5;
} gclk_efm32_select_status_regs_t;

/**
 * @brief This models a pure gate (for switching a clock on and off) that is put after another clock-providing node.
 *        Different to the generic gate type we add this as the EFM32 needs separate bits for enable/disable.
 * @todo: Consider merging/adaptation with the generic base type.
 * For now a separate implementation is used - slimming it down should follow later when all the
 * details are worked out on how generic datatypes can handle this too.
 * An idea to achive this is by organizing the register description in a more dynamic way.
 * E.g., one bitfiled could specify which registers/bits are available, where each bit stands for a
 * specific predetermined bit type like enable-disable-bit, enable-bit, disable-bit, rdy-bit, busy-bit,
 * scale-msb-bit, scale-lsb-bit, enablestatus-bit.
 **/
typedef struct {
  gclk_t base;
  uint32_t en_dis_reg_idx: GCLK_CONF_REG_IDX_BITWIDTH; /**< register that holds control bits to trigger
                                                            enable and disable commands */
  uint32_t ens_reg_idx:    GCLK_CONF_REG_IDX_BITWIDTH; /**< register that holds the status bit that reports
                                                            the clock's enable state. The enable state is
                                                            asserted *before* the clock is ready to be used. */
  uint32_t rdy_reg_idx:    GCLK_CONF_REG_IDX_BITWIDTH; /**< register that holds a status bit that reports
                                                            if the clock is ready to be used */
  /* TODO: On this specific platform one of the enable/disable bits could be encoded implicitly
   *       because the disable bit seems to be always enablebit+1 (not checked everywhere) */
  uint32_t enable_bit:   5; /**< set to enable the clock via @ref en_dis_reg_idx */
  uint32_t disable_bit:  5; /**< set to disable the clock via @ref en_dis_reg_idx */
  uint32_t en_state_bit: 5; /**< set if the clock is enabled (does not guarantee ready) */
  uint32_t ready_bit:    5; /**< set after the startup time is over */
  uint32_t freq_hz;
} gclk_efm32_gate_t;

/**
 * @brief A Platform specific mux type that extends the base type with select/state register access.
 */
typedef struct {
  gclk_t base;
  gclk_efm32_select_status_regs_t regs;
} gclk_efm32_mux_t;

#ifdef __cplusplus
}
#endif

#endif /* GCLK_EFM32_TYPES_H */
/**
 * @}
 */
