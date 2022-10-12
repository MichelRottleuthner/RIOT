#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "periph_cpu.h"
#include "periph/gpio.h"

typedef struct dwt_ctrl_option {
    char *name;
    uint32_t mask;
} dwt_ctrl_option_t;

#if defined(CPU_FAM_STM32L4)
dwt_ctrl_option_t dwtopts[] = {
    { .name = "CYCEVT",   .mask = DWT_CTRL_CYCEVTENA_Msk },
    { .name = "FOLDEVT",  .mask = DWT_CTRL_FOLDEVTENA_Msk },
    { .name = "LSUEVT",   .mask = DWT_CTRL_LSUEVTENA_Msk },
    { .name = "SLEEPEVT", .mask = DWT_CTRL_SLEEPEVTENA_Msk },
    { .name = "EXCEVT",   .mask = DWT_CTRL_EXCEVTENA_Msk },
    { .name = "CPIEVT",   .mask = DWT_CTRL_CPIEVTENA_Msk },
    { .name = "EXCTRC",   .mask = DWT_CTRL_EXCTRCENA_Msk },
    { .name = "CYCCNT",   .mask = DWT_CTRL_CYCCNTENA_Msk },
};
#endif

static void _print_perf_cntrs(void) {
#if defined(CPU_FAM_STM32L4)
    uint32_t CTRL_first = DWT->CTRL;
    uint32_t CYCCNT_first = DWT->CYCCNT;
    uint32_t CPICNT_first = DWT->CPICNT;
    uint32_t EXCCNT_first = DWT->EXCCNT;
    uint32_t SLEEPCNT_first = DWT->SLEEPCNT;
    uint32_t LSUCNT_first = DWT->LSUCNT;
    uint32_t FOLDCNT_first = DWT->FOLDCNT;
    uint32_t PCSR_first = DWT->PCSR;

    printf("DWT->CTRL:     0x%08lx\n",     CTRL_first);
    printf("DWT->CYCCNT:   0x%08lx\n",   CYCCNT_first);
    printf("DWT->CPICNT:   0x%08lx\n",   CPICNT_first);
    printf("DWT->EXCCNT:   0x%08lx\n",   EXCCNT_first);
    printf("DWT->SLEEPCNT: 0x%08lx\n", SLEEPCNT_first);
    printf("DWT->LSUCNT:   0x%08lx\n",   LSUCNT_first);
    printf("DWT->FOLDCNT:  0x%08lx\n",  FOLDCNT_first);
    printf("DWT->PCSR:     0x%08lx\n",     PCSR_first);

    uint32_t DEMCR = CoreDebug->DEMCR;
    uint32_t DBGMCUCR = DBGMCU->CR;
    uint32_t TPI_SPPR = TPI->SPPR;
    uint32_t TPI_ACPR = TPI->ACPR;
    uint32_t TPI_FFCR = TPI->FFCR;
    uint32_t ITM_LAR = ITM->LAR;
    uint32_t ITM_TCR = ITM->TCR;
    uint32_t ITM_TPR = ITM->TPR;
    uint32_t ITM_TER = ITM->TER;

    printf("DEMCR:      0x%08lx\n", DEMCR);
    printf("DBGMCUCR:   0x%08lx\n", DBGMCUCR);
    printf("TPI_SPPR:   0x%08lx\n", TPI_SPPR);
    printf("TPI_ACPR:   0x%08lx\n", TPI_ACPR);
    printf("TPI_FFCR:   0x%08lx\n", TPI_FFCR);
    printf("ITM_LAR:    0x%08lx\n", ITM_LAR);
    printf("ITM_TCR:    0x%08lx\n", ITM_TCR);
    printf("ITM_TPR:    0x%08lx\n", ITM_TPR);
    printf("ITM_TER:    0x%08lx\n", ITM_TER);
#else
#pragma message "WARNING: DEBUG ACCESS IMPLEMENTATION WAS NOT YET ADAPTED TO THIS PLATFORM"
#endif
}

static void dbg_control_swo_init(uint32_t stim_port_mask, uint32_t cpu_freq, uint32_t baudrate)
{
#if defined(CPU_FAM_STM32L4)
    /*
     This bit must be set to 1 to enable use of the trace and debug blocks:
         Data Watchpoint and Trace (DWT)
         Instrumentation Trace Macrocell (ITM)
         Embedded Trace Macrocell (ETM)
         Trace Port Interface Unit (TPIU).
     This enables control of power usage unless tracing is required.
     The application can enable this, for ITM use, or use by a debugger.
     See Cortex-M3 Technical Reference Manual r1p1 for more info:
     https://developer.arm.com/documentation/ddi0337/e/CEGHJDCF
     */
    CoreDebug->DEMCR = CoreDebug_DEMCR_TRCENA_Msk;

    /* TRACE_IOEN=1:
       + TRACE_MODE=00: TRACE pin assignment for Asynchronous Mode (default)
       + TRACE_MODE=01: TRACE pin assignment for Synchronous Mode with a TRACEDATA size of 1
       DBG_STANDBY: Debug Standby mode
        0: (FCLK=Off, HCLK=Off) The whole digital part is unpowered.
        From software point of view, exiting from Standby is identical than fetching reset vector
        (except a few status bit indicated that the MCU is resuming from Standby)
        1: (FCLK=On, HCLK=On) In this case, the digital part is not unpowered and FCLK and
        HCLK are provided by the internal RC oscillator which remains active. In addition, the MCU
        generate a system reset during Standby mode so that exiting from Standby is identical than
        fetching from reset.
       DBG_STOP: Debug Stop mode
        0: (FCLK=Off, HCLK=Off) In STOP mode, the clock controller disables all clocks (including
        HCLK and FCLK). When exiting from STOP mode, the clock configuration is identical to the
        one after RESET (CPU clocked by the 8 MHz internal RC oscillator (HSI16)). Consequently,
        the software must reprogram the clock controller to enable the PLL, the Xtal, etc.
        1: (FCLK=On, HCLK=On) In this case, when entering STOP mode, FCLK and HCLK are
        provided by the internal RC oscillator which remains active in STOP mode. When exiting
        STOP mode, the software must reprogram the clock controller to enable the PLL, the Xtal,
        etc. (in the same way it would do in case of DBG_STOP=0)
       DBG_SLEEP: Debug Sleep mode
        0: (FCLK=On, HCLK=Off) In Sleep mode, FCLK is clocked by the system clock as
        previously configured by the software while HCLK is disabled.
        In Sleep mode, the clock controller configuration is not reset and remains in the previously
        programmed state. Consequently, when exiting from Sleep mode, the software does not
        need to reconfigure the clock controller.
        1: (FCLK=On, HCLK=On) In this case, when entering Sleep mode, HCLK is fed by the same
        clock that is provided to FCLK (system clock as previously configured by the software).

        See e.g. RM0351 p. 1854 for more info:
        https://www.st.com/resource/en/reference_manual/rm0351-stm32l47xxx-stm32l48xxx-stm32l49xxx-and-stm32l4axxx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf#page=1854 */
    DBGMCU->CR = (DBGMCU_CR_DBG_SLEEP | DBGMCU_CR_DBG_STOP | DBGMCU_CR_DBG_STANDBY | DBGMCU_CR_TRACE_IOEN);

    /* Selected PIN Protocol Register: Select which protocol to use for trace output
     *
     *  00: Synchronous Trace Port Mode
     *  01: Serial Wire Output - manchester (default value)
     *  10: Serial Wire Output - NRZ
     *  11: reserved
     *
     *   See e.g. RM0351 p. 1863 for more info:
     * https://www.st.com/resource/en/reference_manual/rm0351-stm32l47xxx-stm32l48xxx-stm32l49xxx-and-stm32l4axxx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf#page=1863
     */
    TPI->SPPR    = 0x00000002 ;

    uint32_t prescaler = (cpu_freq / baudrate) - 1 ;
    /* Async Clock Prescaler Register: scales the baud rate of the asynchronous output */
    TPI->ACPR    = prescaler ;

    /* For further ITM Regisgter documentation also see the ARMv7-M Architecture Reference Manual:
     * https://developer.arm.com/documentation/ddi0403/ee */

    /* ITM Lock Access Register
     * writing 0xC5ACCE55 enables more write access to Control Register 0xE00 :: 0xFFC
     * any other invalid value removes write access again
     * See ARM System Debug documentation for more info, e.g.:
     * https://developer.arm.com/documentation/ddi0337/e/System-Debug/ITM/Summary-and-description-of-the-ITM-registers?lang=en#BABHEEJB */
    ITM->LAR = 0xC5ACCE55 ;

    /* ITM Trace Control Register
     * ITMENA enables the control for the ITM unit. A debugger must set this bit to 1 to permit writes
     * to all Stimulus Port registers. A power-on reset clears this bit to 0
     * SYNCENA enables Synchronization packet transmission for a synchronous TPIU. A power-on reset clears this bit to 0.
     *  Note: If a debugger sets this bit to 1 it must also configure DWT_CTRL.SYNCTAP for the correct synchronization speed, see Control register, DWT_CTRL on page C1-737.
     * DWTENA (TXENA) enables forwarding of hardware event packet from the DWT unit to the ITM for output to the TPIU.
     *  It is IMPLEMENTATION DEFINED whether the DWT discards packets that it cannot forward to the ITM.
     *  Note: If a debugger changes this bit from 0 to 1, the DWT mightforward a hardware event packet that it has previously generated.
     *  A power-on reset clears this bit to 0.
     */
    //ITM->TCR = (ITM_TCR_TraceBusID_Msk | ITM_TCR_DWTENA_Msk | ITM_TCR_SYNCENA_Msk | ITM_TCR_ITMENA_Msk);
    ITM->TCR = (ITM_TCR_TraceBusID_Msk | ITM_TCR_DWTENA_Msk | ITM_TCR_ITMENA_Msk);

    /* ITM Trace Privilege Register: All stimulus ports
     * Controls which stimulus ports can be accessed by unprivileged code.
     * Each register bit controls access to eight stimulus ports.
     * The number of implemented stimulus ports is a multiple of eight. Implemented stimulus ports number consecutively from 0.
     * Bits corresponding to unimplemented stimulus ports are RAZ/WI.
     * PRIVMASK, bits[31:0] Bit[n] of PRIVMASK controls stimulus ports 8n to 8n+7:
     * 0 Unprivileged access permitted.
     * 1 Privileged access only */
    ITM->TPR = ITM_TPR_PRIVMASK_Msk;

    /* ITM Trace Enable Register provides an individual enable bit for each ITM_STIM register.
     * Bits corresponding to unimplemented ITM_STIM registers are RAZ/WI. See Trace
     * Privilege Register, ITM_TPR for information about the number of implemented ITM_STIM registers. */
    ITM->TER = stim_port_mask;

    /* Data Watchpoint and Trace Register
     * CYCTAP Selects the position of the POSTCNT tap on the CYCCNT counter:
     *  0 POSTCNT tap at CYCCNT[6].
     *  1 POSTCNT tap at CYCCNT[10].
     *  For more information see The POSTCNT timer on page C1-732.
     *  This bit is UNK/SBZP if the NOCYCCNT bit is RAO. The reset value is UNKNOWN.
     * POSTINIT Initial value for the POSTCNT counter. For more information seeEnabling POSTCNT, and
     *  behavior of accesses to the DWT_CTRL.POSTINIT field on page C1-740 and The POSTCNT timer on page C1-732.
     *  This field is UNK/SBZP if the NOCYCCNT bit is RAO. The reset value is UNKNOWN.
     * POSTPRESET Reload value for the POSTCNT counter. For more information seeThe POSTCNT timer on page C1-732.
     * This field is UNK/SBZP if the NOCYCCNT bit is RAO. The reset value is UNKNOWN. */
    DWT->CTRL = DWT_CTRL_CYCTAP_Msk | DWT_CTRL_POSTINIT_Msk | DWT_CTRL_POSTPRESET_Msk;
#else
    (void)stim_port_mask;
    (void)cpu_freq;
    (void)baudrate;
#pragma message "WARNING: DEBUG ACCESS IMPLEMENTATION WAS NOT YET ADAPTED TO THIS PLATFORM"
#endif
}

static void _bitbang_serial(gpio_t clkpin, gpio_t datapin, uint8_t *serial_data, size_t datalen) {
    unsigned spinval = 8;
    volatile unsigned spincnt;

    for (unsigned i = 0; i < datalen; i++) {
        uint8_t byte = serial_data[i];
        for (unsigned b = 0; b < 8; b++) {
            gpio_write(datapin, byte & 1);
            byte = byte >> 1;
            spincnt = spinval / 2;
            while (spincnt--) {}
            gpio_set(clkpin);
            spincnt = spinval / 2;
            while (spincnt--) {}
            gpio_clear(clkpin);
            spincnt = spinval;
            while (spincnt--) {}
        }
    }
}

/* NOTE: preliminary evaluation shows an increase of ~250µA with debugger enabled and SWO output of LSU overflow events */
int dbg_control_sc(int argc, char **argv)
{
    if ((argc >= 2) && (strcmp(argv[1], "swoinit") == 0)) {
        int speed = atoi(argv[2]);
        if (argc != 3) {
            printf("usage: %s %s <swofreq>\n", argv[0], argv[1]);
            return 1;
        }
        printf("SWO init: %d Hz\n", speed);
        dbg_control_swo_init(1, 80000000, speed);
        return 0;
#if defined(CPU_FAM_STM32L4)
    } else if ((argc >= 2) && (strcmp(argv[1], "swosend") == 0)) {
        if (argc != 4) {
            printf("usage: %s %s <portnum> <string>\n", argv[0], argv[1]);
            return 1;
        }
        int portnum = atoi(argv[2]);
        printf("SWO send to port %d: %s\n", portnum, argv[3]);
        for (unsigned i = 0; i < strlen(argv[3]); i++) {
            ITM_SendChar(argv[3][i]);
        }
        return 0;
    } else if ((argc >= 2) && (strcmp(argv[1], "dwtset") == 0)) {
        if (argc == 3) {
            if (strcmp(argv[2], "NONE") == 0) {
                DWT->CTRL = 0;
                return 0;
            }
            for (unsigned i = 0; i < ARRAY_SIZE(dwtopts); i++) {
                if (strcmp(argv[2], dwtopts[i].name) == 0) {
                    DWT->CTRL |= dwtopts[i].mask;
                    return 0;
                }
            }
        }

        printf("usage: %s %s { NONE",argv[0], argv[1]);
        for (unsigned i = 0; i < ARRAY_SIZE(dwtopts); i++) {
            printf(" | %s", dwtopts[i].name);
        }
        printf(" }\n");

        return 1;
#endif
    } else if ((argc == 2) && (strcmp(argv[1], "regdump") == 0)) {
        _print_perf_cntrs();
    } else if ((argc == 2) && (strcmp(argv[1], "switchswd") == 0)) {
        gpio_init(DEBUG_INTERFACE_SWDCLK_STIM_PIN, GPIO_OUT);
        gpio_init(DEBUG_INTERFACE_SWDIO_STIM_PIN, GPIO_OUT);

        /* Generate a signal that switches the debug interface of the MCU from JTAG to SWD (to enable using SWO as traceoutput).
         * The data correspons to the follwoing steps:
         * (1) send 56 clks with SWDIO = 1
         * (2) send the JTAG to SWD sequence (0b1110011110011110 starting from LSB)
         * (3) send 56 clks with SWDIO = 1 */
        uint8_t switch_jtag_to_swd[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x9E, 0xE7, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
        _bitbang_serial(DEBUG_INTERFACE_SWDCLK_STIM_PIN,
                        DEBUG_INTERFACE_SWDIO_STIM_PIN, &switch_jtag_to_swd[0], sizeof(switch_jtag_to_swd));

        /* switch back pins to input mode to not interfere with normal debugger output */
        gpio_init(DEBUG_INTERFACE_SWDCLK_STIM_PIN, GPIO_IN); /* pulled low by default */
        gpio_init(DEBUG_INTERFACE_SWDIO_STIM_PIN, GPIO_IN); /* pulled high by default */
    } else {
        printf("Usage: %s {swoinit | swosend | dwtset | regdump | switchswd}\n", argv[0]);
        return 1;
    }

    return 0;
}

