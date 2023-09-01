#ifndef DEBUGHELPERS_H
#define DEBUGHELPERS_H

#ifdef __cplusplus
extern "C" {
#endif

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

/* Mapping of above logic analyzer pins to specific operations to be traced.
 * To dosable tracing of an operation, set the pin to GPIO_UNDEF. */
#define LA_PIN_COORD_CCA                   LA_PIN2
#define LA_PIN_RFD_CCA                     LA_PIN8

#define LA_PIN_COORD_SET_RX_RXD            LA_PIN12
#define LA_PIN_RFD_SET_RX_RXD              LA_PIN4

#define LA_PIN_COORD_TXNOW_TXD             LA_PIN13
#define LA_PIN_RFD_TXNOW_TXD               LA_PIN7

#define LA_PIN_COORD_ON_IDLE               GPIO_UNDEF
#define LA_PIN_RFD_ON_IDLE                 GPIO_UNDEF

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

#define DBG_PIN_SET(X)   if (X != GPIO_UNDEF) {gpio_set(X);}
#define DBG_PIN_CLEAR(X) if (X != GPIO_UNDEF) {gpio_clear(X);}

#ifdef __cplusplus
}
#endif

#endif /* DEBUGHELPERS_H */
/** @} */
