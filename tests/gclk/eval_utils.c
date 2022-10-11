#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "periph/gpio.h"
#include "periph/pm.h"
#include "xtimer.h"
#include "eval_utils.h"
#include "gclk_manager.h"
#include "gclk_idle_timer.h"
#include "shell.h"
#include "xfa.h"

XFA_USE_CONST(shell_command_t*, shell_commands_xfa);

#define CMDLIST_MAX_CMD_CNT     (32)
#define CMDLIST_MAX_ARG_CNT     (32)
#define CMDLIST_FULL_ARG_STRLEN (1024)

enum CALC_OP {
    OP_ADD,
    OP_MUL,
    OP_DIV,
};

enum MEM_ACCESS_TYPE {
    MA_REG,
    MA_RAM,
    MA_FLASH,
};

extern const shell_command_t shell_commands[];
char cmdlist_argvs[CMDLIST_FULL_ARG_STRLEN];
char cmdlist_argcs[CMDLIST_MAX_CMD_CNT * CMDLIST_MAX_ARG_CNT];
unsigned cmdlist_cmd_cnt = 0;
unsigned cmdlist_argvpos = 0;

static uint32_t _flash_data[] = { 5661177, 1087733112, 89276885, 1485921008, 2924143, 1898452982, 506231721, 3445732,
    133699536, 1432395912, 46058663, 90581009, 80613705, 64283940, 33739179, 1927871390, 49476937, 209534284, 1269532278, 946806246, 97744510, 1401137674,
    1883038, 62362979, 525214363, 79667859, 460729923, 134958513, 134450536, 12626199, 1736381142, 91791054, 2619640,
    1726742218, 477476879, 160291205, 688230878, 99835550, 1575314112, 760497830, 23456676, 485608668, 550981142, 2060668, 9134462,
    1926958515, 85684726, 87106343, 192689934, 1722066104, 4881254, 204288843, 311742, 2039198364
};

#define PRE_ITER NOTIFY_START_TO_DMM
#define POST_ITER NOTIFY_STOP_TO_DMM

/* this variable is not volatile so it can stay in the CPU registers while calculating */
#define PRE_ITER_INT_CALC_VAR_REG     uint32_t var = iterations

/* volatile requires this to be reloaded so we need ram access (make sure the address is in RAM) */
#define PRE_ITER_INT_CALC_VAR_RAM     volatile uint32_t var = iterations

#define OP_LOOP_INSTRS_ADD            var += iterations
#define OP_LOOP_INSTRS_MUL            var *= iterations; var = iterations * var;
#define OP_LOOP_INSTRS_DIV            var /= iterations; var = iterations / var;

/* the static variable is stored in ROM section so it must be read from FLASH. (ensure it is way bigger than cache?)
   var in this case can be cached in register (could maybe be moved to volatile ram area?) */
#define OP_LOOP_INSTRS_ADD_FLASH      var += _flash_data[i]; var = _flash_data[i+1] + var
#define OP_LOOP_INSTRS_MUL_FLASH      var *= _flash_data[i]; var = _flash_data[i+1] * var
#define OP_LOOP_INSTRS_DIV_FLASH      var /= _flash_data[i]; var = _flash_data[i+1] / var

/* this volatile variable is misused as target to store the calculation results
   to not optimize away the caluculation */
volatile uint32_t result = 0;

//printf("%s %s\n", #OP_ID, #MA_ID);
#define OP_BENCH(OP_ID, MA_ID, PRE_ITER_LOOP_DEFS, OPERATIONS) \
if (calc_op == OP_ID && mem_access_type == MA_ID) {\
\
    PRE_ITER;\
    PRE_ITER_LOOP_DEFS;\
    while (iterations--){\
        for (uint32_t i = 0; i < ARRAY_SIZE(_flash_data) - 1; i++) {\
            OPERATIONS;\
        }\
    }\
    POST_ITER;\
    result = var;\
}


int _sc_TCI(int argc, char **argv) {

    int calc_op = 0;
    int mem_access_type = 0;

    if (argc != 4) {
        printf("Usage: %s <op> <mem_access_type> <iterations>\n", argv[0]);
    } else {
        if (strcmp(argv[1], "add") == 0) {
            calc_op = OP_ADD;
        } else if (strcmp(argv[1], "mul") == 0) {
            calc_op = OP_MUL;
        } else if (strcmp(argv[1], "div") == 0) {
            calc_op = OP_DIV;
        }

        if (strcmp(argv[2], "reg") == 0) {
            mem_access_type = MA_REG;
        } else if (strcmp(argv[2], "ram") == 0) {
            mem_access_type = MA_RAM;
        } else if (strcmp(argv[2], "flash") == 0) {
            mem_access_type = MA_FLASH;
        }
    }

    uint32_t iterations = atoi(argv[3]);

    OP_BENCH(OP_ADD, MA_REG,   PRE_ITER_INT_CALC_VAR_REG, OP_LOOP_INSTRS_ADD)
    OP_BENCH(OP_ADD, MA_RAM,   PRE_ITER_INT_CALC_VAR_RAM, OP_LOOP_INSTRS_ADD)
    /* TODO: check if var allocation has significant impact */
    OP_BENCH(OP_ADD, MA_FLASH, PRE_ITER_INT_CALC_VAR_REG, OP_LOOP_INSTRS_ADD_FLASH)

    OP_BENCH(OP_MUL, MA_REG,   PRE_ITER_INT_CALC_VAR_REG, OP_LOOP_INSTRS_MUL)
    OP_BENCH(OP_MUL, MA_RAM,   PRE_ITER_INT_CALC_VAR_RAM, OP_LOOP_INSTRS_MUL)
    OP_BENCH(OP_MUL, MA_FLASH, PRE_ITER_INT_CALC_VAR_REG, OP_LOOP_INSTRS_MUL_FLASH)

    OP_BENCH(OP_DIV, MA_REG,   PRE_ITER_INT_CALC_VAR_REG, OP_LOOP_INSTRS_DIV)
    OP_BENCH(OP_DIV, MA_RAM,   PRE_ITER_INT_CALC_VAR_RAM, OP_LOOP_INSTRS_DIV)
    OP_BENCH(OP_DIV, MA_FLASH, PRE_ITER_INT_CALC_VAR_REG, OP_LOOP_INSTRS_DIV_FLASH)

    return 0;
}

/* measure the automatic clock gating impact (ACGI) */
int _sc_ACGI(int argc, char **argv)
{
    if (argc != 1) {
        printf("Usage: %s\n", argv[0]);
    }

    gclk_manager_disable_unused();

    return 0;
}

int _sc_sleep(int argc, char **argv) {

    if (!(argc == 4 || argc == 5)) {
        printf("Usage: %s {xtimer|idletimer} <sleep_ms> <runcnt> [dmmsync]\n", argv[0]);
        return 1;
    }

    bool xtimer = strcmp(argv[1], "xtimer") == 0;
    bool idletimer = strcmp(argv[1], "idletimer") == 0;
    uint32_t ms  = atoi(argv[2]);
    uint32_t run_cnt = atoi(argv[3]);
    bool dmmsync = false;
    
    if (argc == 5 && strcmp(argv[4], "dmmsync") == 0) {
        dmmsync = true;
    }

    for (unsigned i = 0; i < run_cnt; i++) {
        if (dmmsync) { NOTIFY_START_TO_DMM; }

        if (xtimer) {
            xtimer_usleep(ms * 1000);
        } else if(idletimer) {
            idle_timer_wait(ms);
        } else {
            printf("no valid timer type given!\n");
        }

        if (dmmsync) { NOTIFY_STOP_TO_DMM; }

        if (dmmsync) { WAIT_FOR_DMM_READY; }
    }

    return 0;
}

int _sc_test_time_trace(int argc, char **argv) {

    if (argc != 4) {
        printf("Usage: %s <low_us> <high_us> <iterations>\n", argv[0]);
        return 1;
    }

    uint32_t low_us  = atoi(argv[1]);
    uint32_t high_us = atoi(argv[2]);
    uint32_t iterations = atoi(argv[3]);

    for (unsigned i = 0; i < iterations; i++) {
        gpio_set(LOGIC_ANALYZER_PIN);
        xtimer_usleep(high_us);
        gpio_clear(LOGIC_ANALYZER_PIN);
        xtimer_usleep(low_us);
    }

    return 0;
}

int _sc_gpio_test(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    gpio_toggle(DBG_GPIO_WFI);
    printf("_sc_gpio_test\n");
    return 0;
}

int _sc_blink(int argc, char **argv)
{
    if (argc != 3) {
        printf("Usage: %s <interval> <cnt>\n", argv[0]);
        return 1;
    }

    gpio_init(LED0_PIN, GPIO_OUT);
    volatile uint32_t interval = atoi(argv[1]);
    volatile uint32_t cnt = atoi(argv[2]);
    for (unsigned i = 0; i < cnt; i++) {
        gpio_set(LED0_PIN);
        volatile uint32_t work = interval;
        while (work--) {}
        gpio_clear(LED0_PIN);
        work = interval;
        while (work--) {}
    }
    return 0;
}

/* TODO merge with _sc_timer_test via additional parameterization */
int _sc_ctl_idle_timer(int argc, char **argv){

    if (!(argc == 2 || argc == 3)) {
        printf("Usage: %s {on|off|read|set|wait <ms>}\n", argv[0]);
        return -1;
    }

    if (strcmp(argv[1], "on") == 0) {
        idle_timer_enable();
    } else if (strcmp(argv[1], "off") == 0) {
        idle_timer_disable();
    } else if(strcmp(argv[1], "read") == 0) {
        uint64_t t = idle_timer_read();
        printf("current time: %08lx%08lx\n", (uint32_t)(t >> 32), (uint32_t)(t & 0xFFFFFFFF));
        uint64_t a = idle_timer_read_alarm();
        printf("alarm time:   %08lx%08lx\n", (uint32_t)(a >> 32), (uint32_t)(a & 0xFFFFFFFF));
    } else if (strcmp(argv[1], "set") == 0) {
        /* as basic test this just sets the timer to one second */
        idle_timer_set_alarm(idle_timer_read() + IDLE_TMR_LL_TIMER_FREQ);
    } else if (strcmp(argv[1], "wait") == 0 && argc == 3) {
        uint32_t ms = atoi(argv[2]);
        idle_timer_wait(ms);
        printf("wait done!\n");
    } else {
        printf("Usage: %s <on/off>\n", argv[0]);
    }

    return 0;
}

int _sc_timer_test(int argc, char **argv)
{
    uint32_t seconds = atoi(argv[2]);

    if (argc == 3 && (strcmp(argv[1], "xtimer") == 0)) {
        uint64_t t_start = xtimer_now_usec64();

        for (unsigned i = 1; i <= seconds; i++) {
            t_start += 1000000;
            uint32_t spin_cnt = 0;
            while (xtimer_now_usec64() < t_start) {spin_cnt++;}
            printf("%u (CPU spin: %lu)\n", i, spin_cnt);
        }
    } else if (argc == 3 && (strcmp(argv[1], "idle_timer") == 0)) {
        uint64_t t_start = idle_timer_read();

        for (unsigned i = 1; i <= seconds; i++) {
            /* add one second */
            t_start += IDLE_TMR_LL_TIMER_FREQ;
            uint32_t spin_cnt = 0;
            while (idle_timer_read() < t_start) {spin_cnt++;}
            printf("%u (CPU spin: %lu)\n", i, spin_cnt);
        }
    } else {
        printf("Usage: %s {xtimer|idle_timer} <seconds>\n", argv[0]);
        return 1;
    }

    return 0;
}

int _sc_clk_pin(int argc, char **argv)
{
    if (argc != 4) {
        printf("Usage: %s <clk_name> <gpio_portnum> <gpio_pin_num>\n", argv[0]);
        return 1;
    }

    const gclk_t *clk = gclk_get_clk_by_name(argv[1]);

    uint32_t port_num = atoi(argv[2]);
    uint32_t pin_num = atoi(argv[3]);

    printf("enabling clock output for %s on P%c%lu\n", gclk_get_name(clk), (char)('A' + port_num), pin_num);
    int res = gclk_enable_pin_output(clk, GPIO_PIN(port_num, pin_num));

    if (res == ENABLE_PIN_OUTPUT_OK) {
        printf("OK\n");
    } else if (res == ENABLE_PIN_OUTPUT_INVALID_CLOCK) {
        printf("Given clock not appliccable!\n");
    } else if (res == ENABLE_PIN_OUTPUT_INVALID_PIN) {
        printf("Given pin not appliccable!\n");
    } else {
        printf("Unknown ERROR!\n");
    }
    return 0;
}

int _sc_cmdlist_add(int argc, char **argv) {
    cmdlist_argcs[cmdlist_cmd_cnt] = argc - 1;
    for (int i = 1; i < argc; i++) {
        unsigned arg_len = strlen(argv[i]);
        if ((cmdlist_argvpos + arg_len + 1) < CMDLIST_FULL_ARG_STRLEN) {
            strcpy(&cmdlist_argvs[cmdlist_argvpos], argv[i]);
            cmdlist_argvpos += arg_len;
            cmdlist_argvs[cmdlist_argvpos] = '\0';
            cmdlist_argvpos++;
        } else {
            printf("no space left to stack commands\n");
            return -1;
        }
    }
    cmdlist_cmd_cnt++;

    return 0;
}

int _sc_cmdlist_exe(int argc, char **argv) {
    (void)argv;
    (void)argc;
    int read_pos = 0;
    for (unsigned ci = 0; ci < cmdlist_cmd_cnt; ci++) {

        unsigned argcnt = cmdlist_argcs[ci];
        char* argvars[argcnt];
        argvars[0] = &cmdlist_argvs[0];
        for (unsigned ai = 0; ai < argcnt; ai++) {
            argvars[ai] = &cmdlist_argvs[read_pos];
            read_pos += strlen(argvars[ai]) + 1;
        }

        bool found_local = false;

        for (unsigned sci = 0; shell_commands[sci].name != NULL; sci++) {
            if (strcmp(shell_commands[sci].name, argvars[0]) == 0) {
                shell_commands[sci].handler(argcnt, argvars);
                found_local = true;
            }
        }

        if (!found_local) {
            unsigned n = XFA_LEN(shell_command_t*, shell_commands_xfa);
            for (unsigned i = 0; i < n; i++) {
                const volatile shell_command_t *entry = shell_commands_xfa[i];
                if (strcmp(entry->name, argvars[0]) == 0) {
                    entry->handler(argcnt, argvars);
                }
            }
        }
    }

    cmdlist_cmd_cnt = 0;
    cmdlist_argvpos = 0;
    return 0;
}
