/*
 * Copyright (C) 2016-2018 Bas Stottelaar <basstottelaar@gmail.com>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     tests
 * @{
 *
 * @file
 * @brief       Power management peripheral test.
 *
 * @author      Bas Stottelaar <basstottelaar@gmail.com>
 * @author      Vincent Dupont <vincent@otakeys.com>
 *
 * @}
 */

#include <stdio.h>
#include <stdlib.h>

#include "periph/pm.h"
#ifdef MODULE_PM_LAYERED
#ifdef MODULE_PERIPH_RTC
#include "periph/rtc.h"
#endif
#include "pm_layered.h"
#endif
#include "shell.h"

/* this variable is allocated at the start of SRAM2 address space */
extern uint32_t _sram2;
#define RET_RAM_TEST_LEN (64)
__attribute__ ((__section__(".retram")))
uint8_t retention_test_buf[RET_RAM_TEST_LEN];

#ifdef MODULE_PM_LAYERED
static int check_mode(int argc, char **argv)
{
    if (argc < 2) {
        printf("Usage: %s <power mode>\n", argv[0]);
        return -1;
    }

    return 0;
}

static int parse_mode(char *argv)
{
    uint8_t mode = atoi(argv);

    if (mode >= PM_NUM_MODES) {
        printf("Error: power mode not in range 0 - %d.\n", PM_NUM_MODES - 1);
        return -1;
    }

    return mode;
}

#ifdef MODULE_PERIPH_RTC
static int check_mode_duration(int argc, char **argv)
{
    if (argc != 3) {
        printf("Usage: %s <power mode> <duration (s)>\n", argv[0]);
        return -1;
    }

    return 0;
}

static int parse_duration(char *argv)
{
    int duration = atoi(argv);

    if (duration < 0) {
        puts("Error: duration must be a positive number.");
        return -1;
    }

    return duration;
}

static void cb_rtc(void *arg)
{
    int level = (int)arg;

    pm_block(level);
}
#endif /* MODULE_PERIPH_RTC */
#endif /* MODULE_PM_LAYERED */

static int cmd_off(int argc, char **argv)
{
    (void) argc;
    (void) argv;

    puts("CPU will turn off.");
    fflush(stdout);

    pm_off();

    return 0;
}

static int cmd_reboot(int argc, char **argv)
{
    (void) argc;
    (void) argv;

    puts("CPU will reboot.");
    fflush(stdout);

    pm_reboot();

    return 0;
}

#ifdef MODULE_PM_LAYERED
static int cmd_block(int argc, char **argv)
{
    if (check_mode(argc, argv) != 0) {
        return 1;
    }

    int mode = parse_mode(argv[1]);

    if (mode < 0) {
        return 1;
    }

    printf("Blocking power mode %d.\n", mode);
    fflush(stdout);

    pm_block(mode);

    return 0;
}

static int cmd_set(int argc, char **argv)
{
    if (check_mode(argc, argv) != 0) {
        return 1;
    }

    int mode = parse_mode(argv[1]);

    if (mode < 0) {
        return 1;
    }

    printf("CPU will enter power mode %d.\n", mode);
    fflush(stdout);

    pm_set(mode);

    return 0;
}

static int cmd_unblock(int argc, char **argv)
{
    if (check_mode(argc, argv) != 0) {
        return 1;
    }

    int mode = parse_mode(argv[1]);

    if (mode < 0) {
        return 1;
    }

    printf("Unblocking power mode %d.\n", mode);
    fflush(stdout);

    pm_unblock(mode);

    return 0;
}

#ifdef MODULE_PERIPH_RTC
static int cmd_unblock_rtc(int argc, char **argv)
{
    if (check_mode_duration(argc, argv) != 0) {
        return 1;
    }

    int mode = parse_mode(argv[1]);
    int duration = parse_duration(argv[2]);

    if (mode < 0 || duration < 0) {
        return 1;
    }

    printf("Unblocking power mode %d for %d seconds.\n", mode, duration);
    fflush(stdout);

    struct tm time;

    rtc_get_time(&time);
    time.tm_sec += duration;
    mktime(&time);
    rtc_set_alarm(&time, cb_rtc, (void *)mode);

    pm_unblock(mode);

    return 0;
}
#endif /* MODULE_PERIPH_RTC */
#endif /* MODULE_PM_LAYERED */

/**
 * @brief   List of shell commands for this example.
 */
static const shell_command_t shell_commands[] = {
    { "off", "turn off", cmd_off },
    { "reboot", "reboot", cmd_reboot },
#ifdef MODULE_PM_LAYERED
    { "block", "block power mode", cmd_block },
    { "set", "set power mode", cmd_set },
    { "unblock", "unblock power mode", cmd_unblock },
#ifdef MODULE_PERIPH_RTC
    { "unblock_rtc", "temporary unblock power mode", cmd_unblock_rtc },
#endif
#endif
    { NULL, NULL, NULL }
};

/**
 * @brief   Application entry point.
 */
int main(void)
{
    char line_buf[SHELL_DEFAULT_BUFSIZE];

    printf("\n&_sram2:   %p\n", &_sram2);
    printf("should be: 0x10000000\n\n");
    printf("&retention_test_buf:     %p\n", &retention_test_buf);
    printf("should be:               0x10000000\n\n");

    bool mismatch = false;
    /* print some data from SRAM2. If the system woke up from a low power state
       with ram retention this should print an upcounting value.
       Otherwise the RAM will be uninitialized*/
    for(unsigned i = 0; i < RET_RAM_TEST_LEN; i++) {
        uint8_t expected_value = i % 256;
        if (retention_test_buf[i] != expected_value ){
            mismatch = true;
        }
        /* write the expected pattern so we can compare on next boot */
        retention_test_buf[i] = expected_value;
    }

    printf("** RAM was%spreserved from previous boot **\n\n", mismatch ? " NOT " : " ");

    /* print test application information */
#ifdef MODULE_PM_LAYERED
    printf("This application allows you to test the CPU power management.\n"
           "The available power modes are 0 - %d. Lower-numbered power modes\n"
           "save more power, but may require an event/interrupt to wake up\n"
           "the CPU. Reset the CPU if needed.\n",
           PM_NUM_MODES - 1);
#else
    puts("This application allows you to test the CPU power management.\n"
         "Layered support is not unavailable for this CPU. Reset the CPU if\n"
         "needed.");
#endif

    /* run the shell and wait for the user to enter a mode */
    shell_run(shell_commands, line_buf, SHELL_DEFAULT_BUFSIZE);

    return 0;
}
