/*
 * Copyright (C) 2020 HAW Hamburg
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */
/**
 * @ingroup     cpu_stm32_common
 * @ingroup     drivers_periph_core_voltage
 * @{
 *
 * @file
 * @brief       Core Voltage Scaling Implementation
 *
 * @author      Michel Rottleuthner <michel.rottleuthner@haw-hamburg.de>
 *
 * @}
 */
#include <stdint.h>
#include "cpu.h"
#include "kernel_defines.h"
#include "periph/core_voltage.h"
#include "gclk.h"

#define LOG_LEVEL LOG_NONE
#include "log.h"

typedef struct {
    uint32_t vdd_mv;
    uint32_t vcore_mv;
    uint32_t fcore_max_hz;
} voltage_range_t;

#if defined(CPU_FAM_STM32L0)
//#define CORE_VOLTAGE_FIXED_VDD_MV (3300)

/* only tested on nucleo-l073rz */

/*The respective configuration value for the register is implicitly encoded as 'array index + 1' */
static voltage_range_t vranges[] = {
    { .vdd_mv = 1710, .vcore_mv = 1800 , .fcore_max_hz = 32000000},
    { .vdd_mv = 1650, .vcore_mv = 1500 , .fcore_max_hz = 16000000},
    { .vdd_mv = 1650, .vcore_mv = 1200 , .fcore_max_hz =  4200000},
};

/* voltage detector ranges according to PLS[2:0] (PWR_CR[7:5]) */
uint16_t vdd_voltages[] = {
    1900,
    2100,
    2300,
    2500,
    2700,
    2900,
    3100,
};
#define PVD_REG    (PWR->CR)
#define PVD_ENABLE (PWR_CR_PVDE)
#define PLS_POS    (PWR_CR_PLS_Pos)
#define PLS_MSK    (PWR_CR_PLS_Msk)
#define PVDO_REG   (PWR->CSR)
#define PVDO_FLAG  (PWR_CSR_PVDO)

#define VOS_REG    (PWR->CR)
#define VOS_POS    (PWR_CR_VOS_Pos)
#define VOS_MASK   (PWR_CR_VOS)

#define VOSF_REG   (PWR->CSR)
#define VOSF_FLAG  (PWR_CSR_VOSF)


#elif defined(CPU_FAM_STM32L4)
/* only tested on nucleo-l476rg */
/*
  The below values are only applicable if *no* external SMPS is connected to VDD12.
  Following are the possible voltage range configuration. Applicability depend on specific HW configurations of your
  board e.g., presece of an external SMPS and its voltage config.
  The internal voltage regulator is always available and can be configured to the following three ranges. *NO* external
  SMPS is allowed to be connected in that case!:
    range 1:            internal regulator @1.2 V
    range 2:            internal regulator @1.0 V
  If there is an external SMPS 3 additional ranges are possible:
    SMPS range 1:       internal regulator in range 1 (1.2 V); Vcore supplied by external SMPS with VDD12 > 1.25 V
    SMPS range 2 High:  internal regulator in range 2 (1.0 V); Vcore supplied by external SMPS with VDD12 > 1.08 V
    SMPS range 2 Low:   internal regulator in range 2 (1.0 V); Vcore supplied by external SMPS with VDD12 > 1.05 V

  No specific requirements / frequency dependencies were found regarding the input voltage (the voltage going into the
  internal regulator) vdd_mv is not set for this platform (yet).
 * The only semantics enforced by the generic interface is that lower voltage range indexes actually refer to lower voltages
*/
static voltage_range_t vranges[] = {
    { .vcore_mv = 1000 , .fcore_max_hz = 26000000}, /* named "range 2": Low-power        (VOS[0:1] = 0b10) */
    { .vcore_mv = 1200 , .fcore_max_hz = 80000000}, /* named "range 1": High performance (VOS[0:1] = 0b01) */
};

/* voltage detector ranges according to PLS[2:0] (PWR_CR2[3:1]) */
uint16_t vdd_voltages[] = {
    2000,
    2200,
    2400,
    2500,
    2600,
    2800,
    2900,
};
#define PVD_REG    (PWR->CR2)
#define PVD_ENABLE (PWR_CR2_PVDE)
#define PLS_POS    (PWR_CR2_PLS_Pos)
#define PLS_MSK    (PWR_CR2_PLS_Msk)
#define PVDO_REG   (PWR->SR2)
#define PVDO_FLAG  (PWR_SR2_PVDO)

#define VOS_REG    (PWR->CR1)
#define VOS_POS    (PWR_CR1_VOS_Pos)
#define VOS_MASK   (PWR_CR1_VOS)

#define VOSF_REG   (PWR->SR2)
#define VOSF_FLAG  (PWR_SR2_VOSF)
#endif


unsigned int core_voltage_idx2mv(unsigned voltage_idx) {
    return vranges[voltage_idx].vcore_mv;
}
unsigned int core_voltage_cnt(void) {
    return ARRAY_SIZE(vranges);
}

int core_voltage_init(void) {
    return 0;
}

int core_voltage_get_current_vdd(void) {
#ifdef CORE_VOLTAGE_FIXED_VDD_MV
    return CORE_VOLTAGE_FIXED_VDD_MV;
#else
    int voltage = 0;

    /* enable access to the SYSCFG controller */
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN_Msk;

#if defined(SYSCFG_CFGR3_VREFINT_RDYF)
    /* @todo does this flag really not exist on L4 !? */
    /* internal reference is needed for the voltage detector so we need to wait till its ready */
    while (!(SYSCFG->CFGR3 & SYSCFG_CFGR3_VREFINT_RDYF)) { }
#endif

    /* enable voltage detector */
    PVD_REG |= PVD_ENABLE;

    for (unsigned i = 0; i < ARRAY_SIZE(vdd_voltages); i++) {

        /* set up the compare threshold */
        PVD_REG = (PVD_REG & ~PLS_MSK) | (i << PLS_POS);

        /* check if the voltage is above the threshold */
        if (!(PVDO_REG & PVDO_FLAG)) {
            /* save the threshold as voltage if it is the highest one */
            if (vdd_voltages[i] > voltage) {
                voltage = vdd_voltages[i];
            }
        }
    }

    /* disable voltage detector */
    PVD_REG &= ~PVD_ENABLE;

    return voltage;
#endif
}

static inline int _idx2vosval(unsigned idx) {
    return idx == 0 ? 2 : 1;
}

static inline int _vos_val2idx(unsigned vosval) {
    return vosval == 2 ? 0 : 1;
}

void core_voltage_set(unsigned int voltage_idx) {
//     1.
//     Check VDD to identify which ranges are allowed (see Figure 11: Performance versus
//     VDD and VCORE range).
//     2.
//     Poll VOSF bit of in PWR_CSR. Wait until it is reset to 0.
//     3.
//     Configure the voltage scaling range by setting the VOS[1:0] bits in the PWR_CR
//     register.
//     4.
//     Poll VOSF bit of in PWR_CSR register. Wait until it is reset to 0.

    if (voltage_idx >= ARRAY_SIZE(vranges)) {
        LOG_ERROR("invalid voltage range %u!\n", voltage_idx);
        return;
    }

    while (VOSF_REG & VOSF_FLAG) {};

    gclk_reg_util_write_masked(&VOS_REG, VOS_MASK, _idx2vosval(voltage_idx));

    while (VOSF_REG & VOSF_FLAG) {};
}

/**
 * @brief get the current core voltage
 *
 * @return         The current core voltage in mV
 */
int core_voltage_get(void) {
    uint32_t vos_val = (VOS_REG & VOS_MASK) >> VOS_POS;

    return _vos_val2idx(vos_val);
}
