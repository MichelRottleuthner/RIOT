#ifndef DEBUGHELPERS_H
#define DEBUGHELPERS_H

#ifdef __cplusplus
extern "C" {
#endif
// TODO: something in this file causes issues on raspi-pico-2, needs further investigation
#ifndef BOARD_RPI_PICO_2_RISCV
#include "periph/gpio.h"
#include "mutex.h"
extern bool debughelpers_is_coordinator;
extern bool _cfp_start_lock_instr_enabled;
extern mutex_t dsme_cfp_start_lock;

#ifdef BOARD_NATIVE
#define DMM_TRIGGER_PIN_START   GPIO_UNDEF
#define DMM_TRIGGER_PIN_STOP    GPIO_UNDEF
#endif

#ifdef BOARD_NUCLEO_L476RG

/* was used to sync DUT and DMM - not used anymore */
#define _UNUSED_DMM_SYNC_PIN          GPIO_PIN(1,8)
/* not used anymore */
#define _UNUSED_TRANSITION_DEBUG_PIN  GPIO_PIN(2,0)
#define DMM_EVENT_PIN                 GPIO_PIN(2,0)

/* pins used for some specific measurements */
// PC1 (A4)
#define LOGIC_ANALYZER_PIN GPIO_PIN(2,1)

/* Fixed function pins of the Coordinatior device (e.g. radio control pis or SPI).
 * These should not b used for custom instrumentation.
 * */
#define LA_PIN1 GPIO_PIN(1,10)
// use this config to measure cached clock init timings
//#define LA_PIN1 GPIO_PIN(0,5)

/* Freely usable debug pins connected to the Coordinator device */
#define LA_PIN2 GPIO_PIN(2,9)
#define LA_PIN3 GPIO_PIN(2,8)
#define LA_PIN12 GPIO_PIN(0,12)
#define LA_PIN13 GPIO_PIN(0,11)
#define LA_PIN14 GPIO_PIN(1,12)
#define LA_PIN15 GPIO_PIN(1,11)

/* Freely usable debug pins connected to the RFD */
#define LA_PIN4 GPIO_PIN(2,8)
#define LA_PIN7 GPIO_PIN(1,7)
#define LA_PIN6 GPIO_PIN(2,10)
#define LA_PIN8 GPIO_PIN(2,9)

#define LA_DEBUG_PINS_ENABLED (0)

#if LA_DEBUG_PINS_ENABLED == 1
/* Mapping of above logic analyzer pins to specific operations to be traced.
 * To dosable tracing of an operation, set the pin to GPIO_UNDEF. */
#define LA_PIN_COORD_CCA                   LA_PIN2
#define LA_PIN_RFD_CCA                     LA_PIN8

//SAME AS CCA PIN! keep in sync!
//#define LA_PIN_COORD_ON_IDLE               LA_PIN2
//#define LA_PIN_RFD_ON_IDLE                 LA_PIN8
#define LA_PIN_COORD_ON_IDLE               GPIO_UNDEF
#define LA_PIN_RFD_ON_IDLE                 GPIO_UNDEF

#define LA_PIN_COORD_SET_RX_RXD            LA_PIN12
#define LA_PIN_RFD_SET_RX_RXD              LA_PIN4

#define LA_PIN_COORD_TXNOW_TXD             LA_PIN13
#define LA_PIN_RFD_TXNOW_TXD               LA_PIN7

#define LA_PIN_COORD_RADIO_IRQ             LA_PIN15
#define LA_PIN_COORD_SCHED                 LA_PIN3

/* This marks the moment where the delegate receive functions are
 * being updated. I.e. the Beacon/CAP/CFP timing can be observed
 * with the following pattern:
 *
 *                +----------Beacon--------+
 *                |                        |
 *                |  +----CAP---+--CFP----+|
 *                v  v          v         vv
 *                    __________               _____
 * pin level:  ___|__|          |__________|__|
 */
#define LA_PIN_COORD_UPDT_DLGT_BCN_CAP_CFP LA_PIN14
#define LA_PIN_RFD_UPDT_DLGT_BCN_CAP_CFP   LA_PIN6
#else
#define LA_PIN_COORD_CCA                   GPIO_UNDEF
#define LA_PIN_RFD_CCA                     GPIO_UNDEF

#define LA_PIN_COORD_SET_RX_RXD            GPIO_UNDEF
#define LA_PIN_RFD_SET_RX_RXD              GPIO_UNDEF

#define LA_PIN_COORD_TXNOW_TXD             GPIO_UNDEF
#define LA_PIN_RFD_TXNOW_TXD               GPIO_UNDEF

#define LA_PIN_COORD_ON_IDLE               GPIO_UNDEF
#define LA_PIN_RFD_ON_IDLE                 GPIO_UNDEF

#define LA_PIN_COORD_RADIO_IRQ             GPIO_UNDEF
#define LA_PIN_COORD_SCHED                 GPIO_UNDEF

#define LA_PIN_COORD_UPDT_DLGT_BCN_CAP_CFP GPIO_UNDEF
#define LA_PIN_RFD_UPDT_DLGT_BCN_CAP_CFP   GPIO_UNDEF
#endif /* LA_DEBUG_PINS_ENABLED == 1 */

#endif /* BOARD_NUCLEO_L476RG */
#ifdef BOARD_SLSTK3402A

#define LOGIC_ANALYZER_PIN GPIO_PIN(1,11)

/* Fixed function pins of the Coordinatior device (e.g. radio control pis or SPI).
 * These should not b used for custom instrumentation.
 * */
#define LA_PIN1 GPIO_PIN(1,10)

/* Freely usable debug pins connected to the Coordinator device */
#define LA_PIN2 GPIO_PIN(2,9)
#define LA_PIN3 GPIO_PIN(2,8)
#define LA_PIN12 GPIO_PIN(0,12)
#define LA_PIN13 GPIO_PIN(0,11)
#define LA_PIN14 GPIO_PIN(1,12)
#define LA_PIN15 GPIO_PIN(1,11)

/* Freely usable debug pins connected to the RFD */
#define LA_PIN4 GPIO_PIN(2,8)
#define LA_PIN7 GPIO_PIN(1,7)
#define LA_PIN6 GPIO_PIN(2,10)
#define LA_PIN8 GPIO_PIN(2,9)

#define LA_DEBUG_PINS_ENABLED (0)

#if LA_DEBUG_PINS_ENABLED == 1
/* Mapping of above logic analyzer pins to specific operations to be traced.
 * To dosable tracing of an operation, set the pin to GPIO_UNDEF. */
#define LA_PIN_COORD_CCA                   LA_PIN2
#define LA_PIN_RFD_CCA                     LA_PIN8

//SAME AS CCA PIN! keep in sync!
//#define LA_PIN_COORD_ON_IDLE               LA_PIN2
//#define LA_PIN_RFD_ON_IDLE                 LA_PIN8
#define LA_PIN_COORD_ON_IDLE               GPIO_UNDEF
#define LA_PIN_RFD_ON_IDLE                 GPIO_UNDEF

#define LA_PIN_COORD_SET_RX_RXD            LA_PIN12
#define LA_PIN_RFD_SET_RX_RXD              LA_PIN4

#define LA_PIN_COORD_TXNOW_TXD             LA_PIN13
#define LA_PIN_RFD_TXNOW_TXD               LA_PIN7

#define LA_PIN_COORD_RADIO_IRQ             LA_PIN15
#define LA_PIN_COORD_SCHED                 LA_PIN3

/* This marks the moment where the delegate receive functions are
 * being updated. I.e. the Beacon/CAP/CFP timing can be observed
 * with the following pattern:
 *
 *                +----------Beacon--------+
 *                |                        |
 *                |  +----CAP---+--CFP----+|
 *                v  v          v         vv
 *                    __________               _____
 * pin level:  ___|__|          |__________|__|
 */
#define LA_PIN_COORD_UPDT_DLGT_BCN_CAP_CFP LA_PIN14
#define LA_PIN_RFD_UPDT_DLGT_BCN_CAP_CFP   LA_PIN6
#else
#define LA_PIN_COORD_CCA                   GPIO_UNDEF
#define LA_PIN_RFD_CCA                     GPIO_UNDEF

#define LA_PIN_COORD_SET_RX_RXD            GPIO_UNDEF
#define LA_PIN_RFD_SET_RX_RXD              GPIO_UNDEF

#define LA_PIN_COORD_TXNOW_TXD             GPIO_UNDEF
#define LA_PIN_RFD_TXNOW_TXD               GPIO_UNDEF

#define LA_PIN_COORD_ON_IDLE               GPIO_UNDEF
#define LA_PIN_RFD_ON_IDLE                 GPIO_UNDEF

#define LA_PIN_COORD_RADIO_IRQ             GPIO_UNDEF
#define LA_PIN_COORD_SCHED                 GPIO_UNDEF

#define LA_PIN_COORD_UPDT_DLGT_BCN_CAP_CFP GPIO_UNDEF
#define LA_PIN_RFD_UPDT_DLGT_BCN_CAP_CFP   GPIO_UNDEF
#endif /* LA_DEBUG_PINS_ENABLED == 1 */

#endif



#define DBG_PIN_SET(X)   if (X != GPIO_UNDEF) {gpio_set(X);}
#define DBG_PIN_CLEAR(X) if (X != GPIO_UNDEF) {gpio_clear(X);}

typedef struct {
    const char *name;
    uint8_t lsb;
    uint8_t msb;
} named_reg_field_t;

typedef struct {
    const char *name;
    const uint32_t volatile * const reg;
    named_reg_field_t *reg_fileds;
    size_t reg_fileds_len;
} named_reg_map_t;

extern named_reg_map_t hfrcoctrl_regmap;
extern void _bin_dump(uint32_t val);
extern void _reg_map_bin_val_dump(const named_reg_map_t *r, uint32_t val);
extern uint32_t _reg_map_bin_dump(const named_reg_map_t *r);
#else /* #ifndef BOARD_RPI_PICO_2_RISCV */
#define LA_PIN6 GPIO_PIN(0,15)
#define LA_PIN7 GPIO_PIN(0,15)
#endif
#ifdef __cplusplus
}
#endif

#endif /* DEBUGHELPERS_H */
/** @} */
