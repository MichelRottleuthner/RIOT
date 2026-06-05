/*
 * Copyright (C) 2022  HAW Hamburg.
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     sys_shell_commands
 * @{
 *
 * @file
 * @brief       Shell commands for the gclk module
 *
 * @author      Michel Rottleuthner <michel.rottleuthner@haw-hamburg.de>
 *
 * @}
 */
#include <string.h>
#include <stdlib.h>

#include "gclk.h"
#include "gclk_manager.h"
#include "shell.h"
#include "periph/core_voltage.h"
#include "periph/flash_opt.h"
#include "ztimer.h"
#include "frac.h"

#define LOG_LEVEL LOG_NONE
#include "log.h"

#define DESCRIPTION_LINE_FRAME_WIDTH (100)
#define PRINT_USAGE (-1)
/* custom instrumented version of the function, not part of public interface */
extern bool _gclk_manager_set_freq_instrumented(const gclk_t *clk, uint32_t freq);

static char* _dvfs_pol2_str(gclk_manager_dvs_policy_t pol) {
    switch (pol) {
        case DVS_PREFER_FAST_FLASH:  return "DVS_PREFER_FAST_FLASH";
        case DVS_PREFER_LOW_VOLTAGE: return "DVS_PREFER_LOW_VOLTAGE";
    }
    return "INVALID";
}

static const char* _scale_type_to_str(gclk_scale_approach_t st) {
    switch (st) {
        case SCALE_DIRECT:
            return "SCALE_DIRECT";
        case SCALE_UPTREE_RELATIVE:
            return "SCALE_UPTREE_RELATIVE";
        case SCALE_SEQUENCE:
            return "SCALE_SEQUENCE";
        case SCALE_INTERMEDIATE_TOPO_AUTO:
            return "SCALE_INTERMEDIATE_TOPO_AUTO";
        default:
            return "UNDEFINED";
    }
}

static void _print_scale_setting_description_line(const gclk_scale_setting_t *s) {
    printf("[%s]@topology[%d] %s ", gclk_get_name(s->output_clk),
            s->topology_id,
            _scale_type_to_str(s->approach));

    if (s->approach == SCALE_DIRECT ||
            s->approach == SCALE_UPTREE_RELATIVE) {
        printf("via %s ", gclk_get_name(s->scale_clk));
    }


    if (s->approach == SCALE_DIRECT ||
            s->approach == SCALE_UPTREE_RELATIVE) {
        if (gclk_is_divider(s->scale_clk)) {
            printf("/");
        } else if (gclk_is_multiplier(s->scale_clk)) {
            printf("*");
        } else {
            printf("NOSCALER");
        }

        uint32_t min_fact = gclk_idx2factor(s->scale_clk, 0);
        bool noncontiguous = false;
        for (unsigned fi = 0; fi < gclk_factor_cnt(s->scale_clk); fi++) {
            if (gclk_idx2factor(s->scale_clk, fi) != (min_fact + fi)) {
                noncontiguous = true;
            }
        }

        if (noncontiguous) {
            for (unsigned fi = 0; fi < gclk_factor_cnt(s->scale_clk); fi++) {
                printf(" %d", gclk_idx2factor(s->scale_clk, fi));
            }
        } else {
            printf(" FROM %u TO %u", gclk_factor_min(s->scale_clk), gclk_factor_max(s->scale_clk));
        }
    }

    printf("\n");
}

/* TODO: remove after testing or move to gclk private state if deemed useful */
extern bool __use_default_clk_init;

int _sc_dvfs(int argc, char **argv)
{
    bool default_clk_init, dvs, dfs, wsa, on, off, ff_pol, lv_pol, info, set, get, freqs, topo;
    default_clk_init = dvs = dfs = wsa = on = off = ff_pol = lv_pol = info = set = get = freqs = topo = false;

    if (argc >= 3) {
        if (strcmp(argv[1], "dvs") == 0) {
            dvs = true;
        } else if(strcmp(argv[1], "dfs") == 0) {
            dfs = true;
        } else if(strcmp(argv[1], "wsa") == 0) {
            wsa = true;
        } else if(strcmp(argv[1], "topo") == 0) {
            topo = true;
        } else if(strcmp(argv[1], "default_clk_init") == 0) {
            default_clk_init = true;
        }

        if (strcmp(argv[2], "on") == 0) {
            on = true;
        } else if (strcmp(argv[2], "off") == 0) {
            off = true;
        } else if (strcmp(argv[2], "ffpol") == 0) {
            ff_pol = true;
        } else if (strcmp(argv[2], "lvpol") == 0) {
            lv_pol = true;
        } else if (strcmp(argv[2], "info") == 0) {
            info = true;
        } else if (strcmp(argv[2], "set") == 0) {
            set = true;
        } else if (strcmp(argv[2], "get") == 0) {
            get = true;
        } else if (strcmp(argv[2], "freqs") == 0) {
            freqs = true;
        }

        if (default_clk_init) {
            if (on || off) {
                __use_default_clk_init = on;
                return 0;
            }
        }
        if ((dvs || dfs) && (on || off)) {
            if (dfs) {
                gclk_manager_enable_dynamic_frequency_scaling(on);
            } else if (dvs) {
                gclk_manager_enable_voltage_auto_scale(on);
            }
            return 0;
        } else if (wsa && (on || off || get || set)) {
            if (get) {
                printf("current flash waitstates: %u\n", flash_opt_get_wait_states());
            } else if (set) {
                unsigned int ws = atoi(argv[3]);
                printf("setting flash wait states to %u\n", ws);
                flash_opt_set_wait_states(ws);
            } else if (on || off) {
                gclk_manager_enable_flashws_auto_update(on);
            }
            return 0;
        } else if (dvs && (ff_pol || lv_pol || info || set)) {
            gclk_manager_dvs_policy_t pol;
            if (info) {
                pol = gclk_manager_get_dvs_policy();
                printf("current DVS policy: %s\n", _dvfs_pol2_str(pol));

                int cur_v_idx = core_voltage_get();

                if (cur_v_idx >= 0) {
                    printf("current core voltage: %u mV\n", core_voltage_idx2mv(cur_v_idx));
                } else {
                    printf("couldn't read active core voltage setting!\n");
                }

                for (unsigned i = 0; i < core_voltage_cnt(); i++) {
                    printf("core voltage option %u: %u mV\n", i, core_voltage_idx2mv(i));
                }
                return 0;
            } else if (set) {
                unsigned int vcidx = atoi(argv[3]);
                if (argc == 4) {
                    if (vcidx < core_voltage_cnt()) {
                        printf("setting core voltage to range %u (%u mV)\n", vcidx, core_voltage_idx2mv(vcidx));
                        core_voltage_set(vcidx);
                        return 0;
                    } else {
                        printf("invalid voltage range given! Acceptable valueas are 0 to %u\n", core_voltage_cnt() - 1);
                        return -1;
                    }
                }
            } else if (ff_pol || lv_pol) {
                pol = ff_pol ? DVS_PREFER_FAST_FLASH : DVS_PREFER_LOW_VOLTAGE;
                printf("setting DVS poliy to %s\n", _dvfs_pol2_str(pol));
                gclk_manager_set_dvs_policy(pol);
                return 0;
            }
        } else if (dfs && info) {
            const gclk_scale_setting_t *scale_settings;
            int scale_settings_cnt = gclk_manager_get_scale_settings(&scale_settings);
            printf("core clock handle: %s\n", gclk_get_name(gclk_manager_get_core_clock_handle()));
            const gclk_scale_setting_t* cass = gclk_mananger_get_active_scale_setting();

            printf("Currently active scale setting: ");
            if (!cass) {
                printf("(NULL)\n");
            }
            for (int i = 0; i < scale_settings_cnt; i++) {
                if(&scale_settings[i] == cass) {
                    printf("(%d)\n", i);
                    break;
                }
            }

            printf("All available scale settings:\n");
            printf("------------------------------------------\n");
            for (int i = 0; i < scale_settings_cnt; i++) {
                printf("(%d) ", i);
                _print_scale_setting_description_line(&scale_settings[i]);
            }
            printf("------------------------------------------\n");

            const gclk_t **src_clks;
            unsigned cnt = gclk_manager_get_allowed_core_clock_sources(&src_clks);
            for (unsigned i = 0; i < cnt; i++) {
                printf("allowed core source: %s\n", gclk_get_name(src_clks[i]));
            }

            return 0;
        } else if (dfs && freqs) {
            if (argc >= 4) {
                if (strcmp(argv[3], "set") == 0) {
                    if ((argc == 5) && strcmp(argv[4], "default") == 0) {
                        printf("setting default DFS frequencies...\n");
                        gclk_mananger_set_default_dfs_frequencies();
                        printf("[DONE!]\n");
                        return 0;
                    } else if ((argc == 5) && strcmp(argv[4], "default_topo") == 0) {
                        printf("setting up default config for DFS with current topology...\n");
                        const gclk_scale_setting_t* cass = gclk_mananger_get_active_scale_setting();
                        if (cass) {
                            gclk_manager_setup_default_dfs_topo_conf(cass);
                            printf("[DONE!]\n");
                        } else {
                            printf("No active scale setting!\n");
                        }
                        return 0;
                    } else if (argc >= 5) {
                        printf("Adding the following DFS frequencies:");
                        uint32_t fvals[argc - 4];
                        for (int i = 0; i < (argc - 4); i++) {
                            fvals[i] = atoi(argv[i + 4]);
                            printf(" %lu", fvals[i]);
                        }
                        printf("\n");
                        gclk_mananger_set_dfs_frequencies(fvals, argc - 4);
                        return 0;
                    }
                } else if (strcmp(argv[3], "get") == 0) {
                    uint32_t *freqs;
                    unsigned int cnt = gclk_manager_get_dfs_freqs(&freqs);
                    printf("allowed DFS frequencies:");
                    for (unsigned i = 0; i < cnt; i++) {
                        printf(" %lu", freqs[i]);
                    }
                    printf("\n");
                    return 0;
                }
            }
        } else if (dfs && set) {
            if (argc == 4) {
                unsigned scale_setting_id = atoi(argv[3]);
                bool success = gclk_mananger_set_active_scale_setting(scale_setting_id);
                if (success) {
                    printf("changed active scale setting\n");
                    return 0;
                } else {
                    printf("scale setting %d not appliccable for the currently active topology!\n", scale_setting_id);
                }
            }
        } else if (topo && set) {
            //TODO: integrate the whle switch command functionality
            if (argc == 5) {
                bool current = strcmp(argv[3], "current") == 0;
                bool dflt = strcmp(argv[4], "default") == 0;

                const gclk_t *coreclk = gclk_manager_get_core_clock_handle();

                uint32_t target_freq;
                if (dflt) {
                    /* just use the current frequency as target for the topology switch */
                    target_freq = gclk_get_current_freq(coreclk);
                } else {
                    /* otherwise specifiy the target explicilty */
                    target_freq = atoi(argv[4]);
                }

                /* if another topology was requested, switch first */
                if (!current) {
                    unsigned tidx = atoi(argv[3]);

                    const gclk_t *coreclk = gclk_manager_get_core_clock_handle();
                    gclk_manager_switch_topology(coreclk,
                                                 tidx, target_freq,
                                                 gclk_cmp_topology_for_closest_leaf_freq);
                }

                if (dflt) {
                    const gclk_scale_setting_t *cass = gclk_mananger_get_active_scale_setting();
                    if (cass) {
                        gclk_manager_setup_default_dfs_topo_conf(cass);
                        gclk_mananger_set_default_dfs_frequencies();
                    } else {
                       printf("There is no applicable scale setting for this topology!\n");
                    }
                }
                return 0;
            }
        }
    }
    printf("Usage: %s {wsa {on|off|get|set <ws>}} |\n"
           "          {dvs {on|off|info|set <voltage_idx>|ffpol|lvpol}} |\n"
           "          {dfs {on|off|info|set <scalesetting_id>|freqs {get|set {default| <freq>+}}}}\n"
           "          {topo set {current|<topo_idx>} {<target_freq>|default}\n"
           , argv[0]);
    return 0;
}

static gclk_cmp_func_t _get_topology_cmp_fptr(char *fname) {
    for (unsigned i = 0; i < ARRAY_SIZE(topology_cmp_funcs); i++) {
        if (strcmp(topology_cmp_funcs[i].name, fname) == 0) {
            return topology_cmp_funcs[i].func;
        }
    }
    return NULL;
}

static gclk_factor_match_func_t _get_factor_match_fptr(char *fname) {
    for (unsigned i = 0; i < ARRAY_SIZE(factor_match_funcs); i++) {
        if (strcmp(factor_match_funcs[i].name, fname) == 0) {
            return factor_match_funcs[i].func;
        }
    }
    return NULL;
}

static int _get_topo_match_params(int argc, char **argv, gclk_t const **clk, uint32_t *target_freq,
                                  int *tidx, gclk_cmp_func_t *cmp_func, int *forced_conf_idx) {
    if (!((argc >= 3) && (argc <= 7))) {
        printf("Usage: %s <clk_name> <target_freq_hz> [<topology_id> [<cmp_fun> [<forced_conf_idx>]]]", argv[0]);
        return -1;
    }

    *clk = gclk_get_clk_by_name(argv[1]);
    *target_freq = atoi(argv[2]);
    *tidx = -1;
    *forced_conf_idx = -1;

    if (argc >= 4) {
        *tidx = atoi(argv[3]);
    } else {
        *tidx = GCLK_UNDEFINED_TOPOLOGY;
    }

    *cmp_func = gclk_cmp_topology_for_closest_leaf_freq;
    if (argc >= 5) {
        *cmp_func = _get_topology_cmp_fptr(argv[4]);
        if (!(*cmp_func)) {
            printf("cmp_fun must be one of {");
            for (unsigned i = 0; i < ARRAY_SIZE(topology_cmp_funcs); i++) {
                printf("%c%s", i > 0 ? '|' : ' ', topology_cmp_funcs[i].name);
            }
            printf(" }\n");
            return -2;
        }
        if (argc >= 6) {
            *forced_conf_idx = atoi(argv[5]);
        }
    }
    return 0;
}

static void _print_topology_flag_err_msgs(const gclk_t *clk) {
    if (clk->flags.topology_flags & (GCLK_STOP_FOR_UPDATE | GCLK_STOP_PARENT_FOR_UPDATE |
                                                GCLK_STOP_CHILDREN_FOR_UPDATE)) {
        printf("This problem could be related to the following:\n");
         if (clk->flags.topology_flags & GCLK_STOP_FOR_UPDATE) {
             printf(" - The clock must be stopped before updating it\n");
         }

         if (clk->flags.topology_flags & GCLK_STOP_PARENT_FOR_UPDATE) {
             printf(" - The clock's parent must be stopped before updating it\n");
         }

         if (clk->flags.topology_flags & GCLK_STOP_CHILDREN_FOR_UPDATE) {
             printf(" - The clock's children must be stopped before updating it\n");
         }
    }
}

static int _set_parent(const gclk_t *child_clk, const gclk_t *parent_clk) {
    if (!gclk_is_muxable(child_clk)) {
        LOG_ERROR("[%s] can not be switched to a different parent!\n", gclk_get_name(child_clk));
    }
    int idx = gclk_parent2idx(child_clk, parent_clk);

    if (idx < 0) {
        printf("[%s] does not have a parent named [%s]\n", gclk_get_name(child_clk), gclk_get_name(parent_clk));
        return -1;
    }

    uint32_t f_old = gclk_get_current_freq(child_clk);
    uint32_t f_new = gclk_get_current_freq(parent_clk);
    gclk_manager_notify_clk_change(child_clk, f_old, f_new, false);

    gclk_set_parent(child_clk, idx);

    gclk_manager_notify_clk_change(child_clk, f_old, f_new, true);

    const gclk_t *chk_parent = gclk_get_current_parent(child_clk);
    if (parent_clk != chk_parent) {
        LOG_ERROR("WARNING: [%s].set_parent(%s) update didn't work correctly! (new parent reported: [%s])\n",
               gclk_get_name(child_clk), gclk_get_name(parent_clk), gclk_get_name(chk_parent));
        if (LOG_LEVEL >= LOG_ERROR) {
            _print_topology_flag_err_msgs(child_clk);
        }
    }

    return 0;
}

int _sc_match(int argc, char **argv)
{
    if (! ((argc == 3) || (argc == 4) || (argc == 5))) {
        printf("Usage: %s <clk_name> <target_freq_hz> [topology_id] [match_fun]\n", argv[0]);
        return 1;
    }

    const gclk_t *clk = gclk_get_clk_by_name(argv[1]);
    uint32_t target_freq = atoi(argv[2]);
    uint32_t tidx = 0;
    uint32_t *tidx_ptr = NULL;

    if (argc == 4 || argc == 5) {
        tidx = atoi(argv[3]);
        tidx_ptr = &tidx;
        printf("tidx: %lu \n", tidx);
    }

    gclk_factor_match_func_t match_fun = gclk_match_exact_full_iter;
    if (argc == 5) {
        match_fun = _get_factor_match_fptr(argv[4]);
        if (!match_fun) {
            printf("match_fun must be one of {");
            for (unsigned i = 0; i < ARRAY_SIZE(factor_match_funcs); i++) {
                printf("%c%s", i > 0 ? '|' : ' ', factor_match_funcs[i].name);
            }
            printf(" }\n");
            return -1;
        }
    }

    if (clk) {
        uint32_t max_involved_clks = gclk_get_clk_subtree_max_depth(clk, 0);
        printf("topology for driving %s employs max %lu clock nodes\n", gclk_get_name(clk), max_involved_clks);

        clk_topology_entry_t topology[max_involved_clks];
        memset(topology, 0, sizeof(clk_topology_entry_t) * max_involved_clks);

        //gpio_clear(LOGIC_ANALYZER_PIN);
        /* this should be the only field set to describe where we want to start,
           everything else must be zero at the beginning */
        topology[0].clk = clk;
        printf("%s can be driven by %u topologies\n", gclk_get_name(clk),
               gclk_get_topology_config_cnt(clk));
        uint32_t leaf_freq = 0;

        for (unsigned ti = 0; ti < gclk_get_topology_config_cnt(clk); ti++) {
            if (!tidx_ptr || (ti == tidx)) {
                size_t size = gclk_get_nth_topology(topology, max_involved_clks, ti);
                const gclk_t *src_clk = topology[size - 1].clk;
                uint32_t src_freq = gclk_get_input_freq(src_clk);

                leaf_freq = gclk_match_freq_conf(topology, size, src_freq, target_freq, match_fun);

                if (leaf_freq != GCLK_INVALID_FREQ) {
                    printf("Matched frequency of %s from %u possible configs: %lu Hz\n",
                           gclk_get_name(clk), gclk_get_factors_config_cnt_from_topology(topology, size) , leaf_freq);
                    printf("Obtainable using the following configuration:\n");
                    gclk_manager_print_topology_conf(topology, size, true, false);
                } else {
                    printf("Couldn't match\n");
                }
            }
        }
        //gpio_set(LOGIC_ANALYZER_PIN);

    } else {
        printf("could not find clock named %s\n", argv[1]);
    }

    ///* reset logic analyzer pin to idle state */
    //gpio_clear(LOGIC_ANALYZER_PIN);
    return 0;
}

int _sc_topo_switch_to_closest_freq(int argc, char **argv) {
    const gclk_t *clk;
    uint32_t target_freq;
    int tidx;
    gclk_cmp_func_t cmp_func;
    int forced_conf_idx;

    int res = _get_topo_match_params(argc, argv, &clk, &target_freq, &tidx, &cmp_func, &forced_conf_idx);

    if (res < 0) {
        printf("\n");
        return res;
    }

    if (clk) {
        uint32_t new_freq = gclk_manager_switch_topology(clk, tidx, target_freq, cmp_func);
        if (new_freq != GCLK_INVALID_FREQ) {
            printf("switched [%s] to topology [%d] @%lu Hz\n", gclk_get_name(clk), tidx, new_freq);
        } else {
            printf("could not switch [%s] to topology [%d]\n", gclk_get_name(clk), tidx);
        }
        return 0;

    }

    printf("could not find clock named %s\n", argv[1]);
    return -1;
}

int _sc_derive_sequence(int argc, char **argv) {
    const gclk_t *clk;
    uint32_t target_freq;
    int tidx;
    gclk_cmp_func_t cmp_func;
    int forced_nth_conf_idx;

    int res = _get_topo_match_params(argc, argv, &clk, &target_freq, &tidx, &cmp_func, &forced_nth_conf_idx);

    if (res < 0) {
        /* append rest of usage string */
        printf(" [run]\n");
        return res;
    } else {
        printf("derive for forced idx: %d\n", forced_nth_conf_idx);
    }

    bool run = false;
    if (argc == 7 && strcmp(argv[6], "run") == 0) {
        run = true;
    }

    if (clk) {
        uint32_t max_target_topo_len = gclk_get_clk_subtree_max_depth(clk, 0);
        clk_topology_entry_t target_topology[max_target_topo_len];
        memset(target_topology, 0, sizeof(clk_topology_entry_t) * max_target_topo_len);
        target_topology[0].clk = clk;
        target_topology[0].clk_freq = GCLK_INVALID_FREQ;
        size_t valid_cnt = 0;
        /* brute force a configuration for the target topology */
        uint32_t new_freq = gclk_manager_brute_force_freq_conf(clk, target_topology, &max_target_topo_len,
                                                               &tidx, cmp_func, (void*)&target_freq, &valid_cnt, forced_nth_conf_idx, NULL);
        unsigned int cur_topolen = gclk_get_current_topology_len(clk);
        clk_topology_entry_t cur_topology[cur_topolen];
        if (new_freq != target_freq) {
            printf("best frequency match: %lu Hz\n", new_freq);
        }
        gclk_get_current_topology_config(clk, cur_topology, cur_topolen);
        int cur_topo_idx = gclk_topology2id(cur_topology, cur_topolen);

        if (!run) {
            gclk_freq_str_t cf;
            char *cfstr = gclk_freq2str(cur_topology[0].clk_freq, &cf);
            gclk_freq_str_t tf;
            char *tfstr = gclk_freq2str(target_topology[0].clk_freq, &tf);
            printf("Deriving topo switch sequence from %d@%s Hz to %d@%s Hz ...\n", cur_topo_idx, cfstr,
                                                                                      tidx, tfstr);
        }

        gclk_manager_sequence_step_t out_seq[20];
        int seq_size = gclk_manager_derive_sequence(cur_topology, cur_topolen,
                                                    target_topology, max_target_topo_len, out_seq, ARRAY_SIZE(out_seq));

        if (seq_size > 0) {
            if (!run) {
                gclk_manager_print_step_sequence(out_seq, seq_size);
            }
            gclk_manager_run_sequence_with_notify(out_seq, seq_size, !run);
        } else {
            printf("transition from [%s] topology from %d to %d infeasible!\n", gclk_get_name(clk), cur_topo_idx, tidx);
        }
        return 0;
    }

    printf("could not find clock named %s\n", argv[1]);
    return -1;
}

/* static allocation of worst case-sized buffers to be used in _print_valid_conf_cb to
 * determine a sequence for switching from the current tree config to thhe given target
 * tree conf, to then evaluate constraints on a global tree wide view */
clk_topology_entry_t _print_valid_confs_tree_conf_buf[GCLK_NUM_OF_CLOCKS];
clk_topology_entry_t _pvc_cur_topology[GCLK_NUM_OF_CLOCKS];
gclk_manager_sequence_step_t _pvc_seq[20];

/* TODO: optimize this function/the function that calls this for lower stack usage */
void _print_valid_conf_cb(clk_topology_entry_t *conf, size_t conf_len, gclk_cmp_result_t res, unsigned valid_idx, void *ctx) {
    (void)res; /* don't care about the comparison, just print all valid confs */
    (void)ctx; /* don't need context */
    bool print_min_max = false;
    bool print_factors = true;

    printf("--------------------\n");
    printf("Valid config IDX %u\n", valid_idx);
    gclk_manager_print_topology_conf(conf, conf_len, print_min_max, print_factors);

    unsigned int cur_topolen = gclk_get_current_topology_len(conf[0].clk);
    gclk_get_current_topology_config(conf[0].clk, _pvc_cur_topology, cur_topolen);

    int seq_len = gclk_manager_derive_sequence(_pvc_cur_topology, cur_topolen,
                                                conf, conf_len,
                                                _pvc_seq, ARRAY_SIZE(_pvc_seq));

    if (seq_len > 0) {
        /* copy the current tree conf into a buffer to simulate changes on it */
        for (unsigned i = 0; i < GCLK_NUM_OF_CLOCKS; i++) {
            gclk_get_current_topology_config(gclk_get(i), &_print_valid_confs_tree_conf_buf[i], 1);
        }

        for (size_t si = 0; si < (unsigned)seq_len; si++) {
            gclk_manager_simulate_seq_step_on_tree_conf(&_pvc_seq[si], _print_valid_confs_tree_conf_buf, GCLK_NUM_OF_CLOCKS);
        }

        unsigned ws_lv = 0;
        unsigned vc_lv = 0;
        unsigned ws_ff = 0;
        unsigned vc_ff = 0;
        gclk_get_min_required_ws_vc_from_tree_config(_print_valid_confs_tree_conf_buf, GCLK_NUM_OF_CLOCKS, &ws_lv, &vc_lv, DVS_PREFER_LOW_VOLTAGE);
        gclk_get_min_required_ws_vc_from_tree_config(_print_valid_confs_tree_conf_buf, GCLK_NUM_OF_CLOCKS, &ws_ff, &vc_ff, DVS_PREFER_FAST_FLASH);
        printf("LVPOL| WS: %u VCIDX: %u (%u mV)\n", ws_lv, vc_lv, core_voltage_idx2mv(vc_lv));
        printf("FFPOL| WS: %u VCIDX: %u (%u mV)\n", ws_ff, vc_ff, core_voltage_idx2mv(vc_ff));
    } else {
        printf("ERROR no applicable sequence to switch to the given config!\n");
    }
    printf("--------------------\n");
}

void _print_better_valid_conf_cb(clk_topology_entry_t *conf, size_t conf_len, gclk_cmp_result_t res, unsigned valid_idx, void *ctx) {
    if (res == GCLK_CONF_BETTER) {
        _print_valid_conf_cb(conf, conf_len, res, valid_idx, ctx);
    }
}

#define GCLK_MANAGER_INVALID_CONF_IDX (0xFFFFFFFF)

struct conf_idx_cb_ctx {
    uint32_t current_valid_idx;
    uint32_t best_valid_idx;
} gclk_manager_best_conf_idx_cb_t;

void _store_best_conf_idx_cb(clk_topology_entry_t *conf, size_t conf_len, gclk_cmp_result_t res, unsigned valid_idx, void *ctx) {
    (void)conf; /* don't care about the actual config, as we only want to find the best configs index */
    (void)conf_len;

    uint32_t *best_valid_idx = ctx;
    if (res == GCLK_CONF_BETTER) {
        *best_valid_idx = valid_idx;
    }
}

/* arg must be terminated with '\0' */
static bool _arg_is_valid_number(char *arg, bool must_be_positive) {
    unsigned i = 0;
    bool negative = false;
    while (arg[i] != '\0') {
        if (i == 0 && (arg[i] == '-')) {
            negative = true;
            if (must_be_positive) {
                return false;
            }
        } else if (arg[i] < '0' || arg[i] > '9') {
            return false;
        }
        i++;
    }
    return (negative && (i > 2)) || (i > 0);
}

/* trial division factorization */
static int get_factors(uint32_t n, uint32_t *facts, size_t fact_len){
    unsigned idx = 0;

    while (n % 2 == 0) {
        if (fact_len < idx) {
            return -1;
        }
        facts[idx++] = 2;
        n /= 2;
    }

    uint32_t f = 3;

    while (f * f <= n) {
        if (n % f == 0) {
            if (fact_len < idx) {
                return -1;
            }
            facts[idx++] = f;
            n /= f;
        } else {
            f += 2;
        }
    }
    if (n != 1) {
        if (fact_len < idx) {
            return -1;
        }
        facts[idx++] = n;
    }
    return idx;
}

void get_all_possible_partitions(size_t n_scalers, uint32_t factor2distribute) {
    (void)n_scalers;
    (void)factor2distribute;
    // ### prime factors of the factor we want to distribute across different scalers
    // prime_factors[] = factorize(factor2distribute);
    //
    // ### build all possible combinations of prime factors
    // ### this will hold all theoretically possibile combinations on how the prime numbers may be distributed across scalers
    // prime_powerset = powerset(factors)
    //
    //### the powerset is too big because it contains also all sorts of combinations that would not allocate all factors
    //### an important property/constraint is that *all* factors must be used
    //### partitions seem to be more useful (also with the constraint that we do not need to consider the partitions with arbitrary length)
    //
    // ### this will return all distinct factors that can be built from the prime combinations
    // ### i.e., this will hold all numbers that could potentially allocated to one scaler
    // possible_factors = set([product(x) for x in prime_powerset])
}

typedef struct {
    uint32_t *vals;
    size_t *val_len;
} mul_partition_elem_t;

typedef struct {
    uint32_t value;
    uint32_t exp;
} prime_exponential_t;

/* @pre vals must be sorted */
static int get_distinct_count(uint32_t *vals, size_t len)
{
    uint32_t last_val = 0;
    int cnt = 0;
    for (unsigned i = 0; i < len; i++) {
        if (i && (last_val == vals[i])) {
            continue;
        }
        last_val = vals[i];
        cnt++;
    }
    return cnt;
}

/* duplicate value counts, 8 bit are miore than enough within our context
 * because the number of values is strictly limited on the number of clock nodes.
 * { 1, 1, 1, 2, 2, 3} translates into {3, 2, 1} */
static size_t sorted_list_val_cnt(uint32_t *vals, uint8_t *cnts, size_t len)
{
    uint32_t last_val = 0;
    size_t cntidx = 0;
    for (unsigned i = 0; i < len; i++) {
        if (i) {
            if (last_val == vals[i]) {
                cnts[cntidx]++;
                continue;
            }
            cntidx++;
        }
        if (cntidx >= len) {
            return 0;
        }
        last_val = vals[i];
        cnts[cntidx] = 1;
    }
    return cntidx +1;
}

static size_t primes_rawlist2explist(uint32_t *primes, size_t prlen, prime_exponential_t *pes, size_t pelen)
{
    uint32_t last_val = 0;
    size_t pidx = 0;
    for (unsigned i = 0; i < prlen; i++) {
        if (i) {
            if (last_val == primes[i]) {
                pes[pidx].exp++;
                continue;
            }
            pidx++;
        }
        if (pidx >= pelen) {
            return 0;
        }
        pes[pidx].value = primes[i];
        pes[pidx].exp = 1;
        last_val = primes[i];
    }
    return pidx + 1;
}

/* For the first call use last = 1 (neutral element of multiplication) */
static uint32_t next_divisor(uint32_t n, uint32_t last)
{
    for (uint32_t i = last + 1; i <= n; ++i){
        if (n % i == 0) {
            return i;
        }
    }
    return 0;
}

# define PRIME_FACTORS_MAX_NUMOF (20)

typedef struct {
    /* pointer to the values in this partition */
     uint32_t *elements;
    size_t len;
} mul_partition_meta_t;

# define MUL_PART_MAX_VALUES_NUMOF (2048)

typedef struct {
    uint32_t *all_vals; /* pointer to raw buffer holding the values of all partitions */
    mul_partition_meta_t *partitions; /* pointer to metadata elements */
    size_t val_idx;   /* index pointing at the first empty raw value */
    size_t val_len;   /* length of raw value array */
    size_t last_closed_val_idx; /* index pointing at the end of the last closed partition */
    size_t part_idx; /* index pointing at the first empty meta data  */
    size_t part_len; /* length of partition meta data elements */
} mul_partition_set_t;

static void mp_init_set(mul_partition_set_t *mps,
                        uint32_t *val_buf, size_t vlen,
                        mul_partition_meta_t *parts_buf, size_t plen)
{
    mps->all_vals = val_buf;
    mps->val_len = vlen;
    mps->val_idx = 0;
    mps->partitions = parts_buf;
    mps->part_len = plen;
    mps->part_idx = 0;
    mps->last_closed_val_idx = 0;
}

static void mp_print_partition(mul_partition_meta_t *p, bool newline)
{
    printf("{ ");
    for (unsigned i = 0; i < p->len; i++) {
        printf("%lu ", p->elements[i]);

    }
    printf("}%s", newline ? "\n" : "");
}

static mul_partition_meta_t* mp_get_partition(mul_partition_set_t *parts, size_t n)
{
    return &parts->partitions[n];
}

static size_t mp_get_partition_cnt(mul_partition_set_t *mps)
{
    return mps->part_idx;
}

//static void mp_print_all_partitions(mul_partition_set_t *mps)
//{
//    size_t mp_cnt = mp_get_partition_cnt(mps);
//
//    printf("%d partitions with %u values:\n", mp_cnt, mps->val_idx);
//    for (unsigned i = 0; i < mp_cnt; i++) {
//        mul_partition_meta_t *part = mp_get_partition(mps, i);
//        mp_print_partition(part, true);
//    }
//}

/* start a partition by adding values */
static bool mp_append_value(mul_partition_set_t *mps, uint32_t val)
{
    if (mps->val_idx >= mps->val_len) {
        /* continue counting so a missing value can be detected on close */
        mps->val_idx++;
        return false;
    }
    //printf("append value @%u\n", mps->val_idx);
    mps->all_vals[mps->val_idx++] = val;
    return true;
}

/* to be called after some values were added */
static mul_partition_meta_t* mp_close_partition(mul_partition_set_t *mps)
{
    if ((mps->part_idx >= mps->part_len) ||
        (mps->val_idx >= (mps->val_len + 1) )) {
        printf("closing partition failed! slots used: %u/%u part | %u/%u value\n",
                mps->part_idx, mps->part_len, mps->val_idx, mps->val_len);
        return NULL;
    }
    mul_partition_meta_t *part = mp_get_partition(mps, mps->part_idx++);
    part->len = mps->val_idx - mps->last_closed_val_idx;
    part->elements = &mps->all_vals[mps->val_idx - part->len];
    mps->last_closed_val_idx = mps->val_idx;
    //printf("new partition with len %u:\n", part->len);
    //mp_print_partition(part, true);
    return part;
}

static size_t get_mul_partitions(uint32_t num2split, mul_partition_set_t *mps, size_t max_partsize)
{
    uint32_t cur_part_len = 1;
    uint32_t new_part_cnt = 1;

    /* there always at least the trivial partition so create it
     * before searching for more */
    if (!(mp_append_value(mps, num2split) && mp_close_partition(mps))) {
        printf("error creating trival partition %lu\n", num2split);
        return 0;
    }

    /* return if there is only the trivial single value partition */
    if (max_partsize == 1) {
        return 1;
    }

    while (new_part_cnt) {
        size_t current_part_cnt = mp_get_partition_cnt(mps);
        /* current partition index, i.e. the index to the partition which will
         * be split into multiple partitions next. */
        size_t splitidx = current_part_cnt - new_part_cnt;
        /* highest index of partitions already generated */
        size_t max_part_idx = current_part_cnt - 1;
        new_part_cnt = 0;

        /* stop if max partition size is fully explored */
        if ((max_partsize > 0) && (cur_part_len >= max_partsize)) {
            break;
        }

        /* partitions generated in this iteration will hold one more element than
         * the partitions that will be split to construct them */
        cur_part_len++;

        while (splitidx <= max_part_idx) {
            mul_partition_meta_t *split_part = mp_get_partition(mps, splitidx);
            //printf("splitpart[%d]@%p len=%u:\n", splitidx, split_part, split_part->len);
            //mp_print_partition(split_part, true);

            /* highest value of partition to be split */
            num2split = (splitidx == 0) ? split_part->elements[0] : split_part->elements[split_part->len-1];
            /* first partition has only one element, therfore no divisors are skipped */
            uint32_t mindiv = (splitidx == 0) ? 2 : split_part->elements[split_part->len-2];
            uint32_t div = mindiv;
            if (div != 1) {
                /* reduce div by one so that below divisor iteration
                 * will also go over values equal to the min divisor */
                div--;
            }
            /* for each divisor */
            while ((div = next_divisor(num2split, div)) != 0) {
                uint32_t sf = num2split / div;
                /* if the divisor is larger than the newly split factor
                 * the procedure must stop here to not add duplicates.
                 * as we iterate over divisors starting from smaller to larger,
                 * if sf is smaller than the divisor it means the case is already
                 * part of another partition. */
                if (div > sf) { //TODO: document this properly
                    break;
                }

                /* from the base partition add all but the last factor
                 * to a new partition. This wont add any factor if the base
                 * partition has only one element.*/
                for (unsigned i = 0; i < cur_part_len-2; i++) {
                    uint32_t cpval = split_part->elements[i];
                    if (!mp_append_value(mps, cpval)) {
                        printf("error appending value %lu\n", cpval);
                        return 0;
                    }
                }

                mp_append_value(mps, div);
                mp_append_value(mps, sf);
                if (mp_close_partition(mps) == NULL) {
                    printf("error creating new partition!\n");
                    return 0;
                }
                new_part_cnt++;
            }
            splitidx++;
        }

    }
    return mp_get_partition_cnt(mps);
}

//TODO: make this its own type that can easily overloaded for each platform */
//verwritten
uint32_t mul_part_val_buf[MUL_PART_MAX_VALUES_NUMOF];
mul_partition_meta_t mul_part_meta_buf[MUL_PART_MAX_VALUES_NUMOF/2];

void test_facotrization(uint32_t up_to_n)
{
    int maxlen = 0;
    uint32_t maxnum = 0;
    uint32_t factors[32];
    for (uint32_t i = 0; i < up_to_n; i++) {
       int len = get_factors(i, factors, ARRAY_SIZE(factors));
       if(len > maxlen) {
           maxlen = len;
           maxnum = i;
       }
       uint32_t num = 1;
       for (int x = 0; x < len; x++) {
           num = num * factors[x];
       }
       if (num != i) {
           printf("ERROR: did not factorize %lu correctly!!\n", i);
       }
    }
    printf("max count of factors was %u factors for %lu\n", maxlen, maxnum);
}

void test_mul_partition(uint32_t num2partition, size_t max_part_size)
{
    uint32_t factors[32];
    int len = get_factors(num2partition, factors, ARRAY_SIZE(factors));
    printf("prime factors of %lu: ", num2partition);
    for (int x = 0; x < len; x++) {
       printf("%s%lu", x ? ", " : "", factors[x]);
    }
    printf("\n");


    if (len > 0) {
        int distinct_primes = get_distinct_count(factors, len);
        prime_exponential_t pes[distinct_primes];
        size_t pelen = primes_rawlist2explist(factors, len, pes, distinct_primes);
        printf("%d distinct primes: ", pelen);
        for (unsigned i = 0; i < pelen; i++) {
            printf("%s%lu", i > 0 ? " * ": "", pes[i].value);
            if (pes[i].exp > 1) {
                printf("^%lu", pes[i].exp);
            }
        }
        printf("\n");
    }

    printf("Divisors of %lu:   ", num2partition);
    uint32_t div = 1;
    while (true) {
        div = next_divisor(num2partition, div);
        if (div == 0) {
            break;
        }
        printf("%lu ", div);
    }
    printf("\n");

    mul_partition_set_t partitions;
    mp_init_set(&partitions,
                mul_part_val_buf, ARRAY_SIZE(mul_part_val_buf),
                mul_part_meta_buf, ARRAY_SIZE(mul_part_meta_buf));

    printf("derive multiplicative partitions:\n");
    size_t mp_cnt = get_mul_partitions(num2partition, &partitions, max_part_size);
    printf("got %d partitions with %u values:\n", mp_cnt, partitions.val_idx);
    for (unsigned i = 0; i < mp_cnt; i++) {
        mul_partition_meta_t *part = mp_get_partition(&partitions, i);
        mp_print_partition(part, true);
    }
}

static void mp_partition_padded_copy(mul_partition_meta_t *src, uint32_t *dst_vals, size_t dst_len)
{
   size_t padcnt = (dst_len - src->len);
   for (unsigned i = 0; i < padcnt; i++) {
       dst_vals[i] = 1;
   }
   for (unsigned i = 0; i < src->len; i++) {
       dst_vals[padcnt + i] = src->elements[i];
   }
}

static void _perm_swap(uint32_t *x, uint32_t *y) {
    int temp = *x;
    *x = *y;
    *y = temp;
}

bool _already_contained(uint32_t *list, size_t list_len, uint32_t *tuple, size_t tuple_len) {
    for (unsigned i = 0; i < list_len; i++) {
        bool all_equal = true;
        uint32_t *perm = &list[i*tuple_len];
        for (unsigned x = 0; x < tuple_len; x++) {
            if (perm[x] != tuple[x]) {
                all_equal = false;
                break;
            }
        }
        if (all_equal) {
            return true;
        }
    }
    return false;
}

//TODO: combine target permutation stuff into combined structure */
void get_permutations(size_t depth, uint32_t* values, size_t n, uint32_t *perm, size_t *permidx, size_t permcnt) {
   if (depth == n-1) {
       if (!_already_contained(perm, permcnt, values, n)) {
            for (unsigned k = 0; k < n; ++k) {
                perm[*permidx * n + k] = values[k];
            }
            (*permidx)++;
       }
       return;
   }
   for (unsigned i = depth; i < n; i++) {
       if (i != depth) {
           _perm_swap(values + i, values + depth);
           get_permutations(depth+1, values, n, perm, permidx, permcnt);
           _perm_swap(values + i, values + depth);
       } else {
           get_permutations(depth+1, values, n, perm, permidx, permcnt);
       }
   }
   return;
}

/* @pre n < 13
 * calculate the factorial of the given number.
 * Shall only be called for small values. Values required for actual
 * MCU clock trees should never even get close to this size. */
static uint32_t get_factorial(uint32_t n)
{
    /* 32 bits can only hold up to 12! */
    assert(n <13);

    uint32_t factorial = 1;
    for (uint32_t i = 1; i <= n; i++) {
        factorial *= i;
    }
    return factorial;
}

size_t get_permutation_cnt(uint32_t *sorted_vals, size_t len)
{
    /* max possible number of permutations if there are no duplicate values */
    uint32_t max_perms = get_factorial(len);

    /* count how often each value is repeated */
    uint8_t val_cnts[len];
    uint8_t uniq_val_cnt = sorted_list_val_cnt(sorted_vals, val_cnts, len);

    /* duplicate perumtations because of duplicate numbers */
    uint32_t dups = 1;
    /* if there are duplicates, factor them out of the number of permutations */
    if (uniq_val_cnt != len) {
        for (unsigned c = 0; c < uniq_val_cnt; c++) {
            uint32_t d = get_factorial(val_cnts[c]);
            dups *= d;
        }
    }
    return max_perms / dups;
}

/* metadata descriptor for a permutation (list of values and length) */
typedef struct {
    uint32_t *vals;
    size_t cnt;
} perm_meta_t;

/* forward declaration to nest fact_perm_result_ctx_t */
typedef struct fact_perm_result_ctx fact_perm_result_ctx_t;

typedef struct fact_perm_result_ctx {
    uint32_t *vals; /* points to values[size * cnt] */
    gclk_factor_limit_t *limits; /* points to array with limits for each scaler */
    size_t size; /* number of elements per permutation */
    size_t cnt;  /* number of permutations */
    fact_perm_result_ctx_t *parent_perms; /* parent level permutations, null for root */
} fact_perm_result_ctx_t;

#define MAX_MUL_CNT (5) /* ensure to be larger than max number of multipliers in core topology */
#define MAX_DIV_CNT (5) /* ensure to be larger than max number of divisors in core topology */

typedef struct {
    uint32_t specific_config_index; /* filter for only a specific given config index */
    bool best_found; /* true once best config was found. In case of a specific id this can be
                        used to early-out. */
} topo_perm_filter_ctx_t;

typedef struct {
    size_t *mul_idxs;
    size_t *div_idxs;
    bool *contiguous_factor_range; /* cached marker for each scaler:
                                      true if factor range is contiguous,
                                      false if the range is fragmented.
                                      Used to speed up factor applicability checks. */
    size_t mul_len;
    size_t div_len;
} topo_scaler_index_meta_t;

typedef struct {
    void (*topo_conf_found)(clk_topology_entry_t *topology, size_t len, void *ctx);
    void *cbctx;
} gclk_topo_conf_iter_ctx_t;

typedef struct {
    clk_topology_entry_t *topo;
    size_t topo_len;
    topo_scaler_index_meta_t *scaler_idx_meta;
    mul_partition_set_t *mpartitions;
    mul_partition_set_t *dpartitions;
    uint32_t *mpv_buf; /* mul part value buffer free to be used for permutation combos */
    size_t mpv_len; /* num of elements in mpv_buf */
    mul_partition_meta_t *mpm_buf; /* mul part meta data */
    uint32_t mpm_len; /* num of elements in mpm_buf */
    uint32_t eff_mul; /* effective mul factor */
    uint32_t eff_div; /* effective div factor */
    uint32_t factor_part_cnt; /* number of multiplicative partitions found */
    uint32_t factor_part_len; /* number of values in multiplicative partitions found */
    uint32_t mappable_perm_cnt; /* number of factor combinations that could be represented by
                                   scaler values */
    uint32_t valid_perm_cnt; /* number of factor combinations that result in a valid configuration
                                which does not break any frequency constraint */
    gclk_freq_limit_t *flims; /* limits for each constrained clock in topo */
    clk_topology_entry_t **flim_tes; /* references to entries in topo corresponding to flim entries */

    size_t flim_cnt;        /* overall number of factor permutations found */
    gclk_factor_limit_t *dlims; /* limits for each divisor in topo */
    gclk_factor_limit_t *d_topo_lims; /* limits across all divisors in topo */
    gclk_factor_limit_t *mlims; /* limits for each multipliers in topo */
    gclk_factor_limit_t *m_topo_lims; /* limits across all multipliers in topo */
    topo_perm_filter_ctx_t *result_filter; /* a filter descriptor usable by the result cb.
                                              NULL if not filtering */
    uint32_t *result_buf; /* where to store the resulting factor combination */
    gclk_topo_conf_iter_ctx_t *topo_conf_iter;
} topo_perm_glob_ctx_t;

/**
 * @brief Contextual callback metadata.
 */
typedef struct gclk_factor_perm_cb_ctx {
    void (*cb)(topo_perm_glob_ctx_t *gctx,
               fact_perm_result_ctx_t *res_ctx); /**< The callback to execute. */
    topo_perm_glob_ctx_t *gctx;  /**< The lobal context pointer handed to each call of cb. */
    fact_perm_result_ctx_t *parent_perms; /* parent level permutations, null for root */
} gclk_factor_perm_cb_ctx_t;

/* generate all possible permutations for each partition in the given
 * set of multiplicative partitions. */
static size_t mp_set_permutations(mul_partition_set_t *mps, size_t target_size,
                                  gclk_factor_perm_cb_ctx_t *fpctx,
                                  gclk_factor_limit_t *limits_per_scaler,
                                  gclk_factor_limit_t *limit_across_scalers)
{
    size_t permcntsum = 0;
    for (unsigned i = 0; i < mp_get_partition_cnt(mps); i++) {
        uint32_t permutation[target_size];
        mul_partition_meta_t *p = mp_get_partition(mps, i);

        /* To generate a factor configuration for the target topology,
         * each scaler clock node must be assigned one value. The partition size
         * must therefore be equal to the number of scalers. This step ensures
         * that partitions which are smaller than the number of involved clock
         * scaler nodes are padded to the required size whithout changing the
         * effective scaling factor. This effectively just inserts "1" values
         * at the left of the partition till the target size is reached, which
         * keeps the order in tact and the resulting partition is equivalent
         * to the too-small partition. */
        mp_partition_padded_copy(p, permutation, target_size);

        /* number of actually possible permutations considering duplicate values */
        size_t perm_cnt = get_permutation_cnt(permutation, target_size);
        bool perm_skip = false;
        /* early out if largest factor cannot be mapped (no permutations must be generated at all)
         * Only last element in permutations array must be checked because it is sorted. */
        if (permutation[target_size - 1] > limit_across_scalers->max) {
            //printf("skip permutations of: ");
            //for (unsigned j = 0 ; j < target_size; j++) {
            //    printf("%lu ", permutation[j]);
            //}
            //printf("\n");
            perm_skip = true;
        }
        if (perm_skip) {
            continue;
        }
        /* TODO: investigate alternative ways for detecting duplicates
         *       or permutation generators that do not generate duplicates.
         *       For the expected small permutation sizes this shoud suffice. */
        uint32_t all_perms[target_size * perm_cnt];

        size_t pidx = 0;
        get_permutations(0, permutation, target_size, all_perms, &pidx, perm_cnt);

        fact_perm_result_ctx_t res_ctx = {
            .vals = all_perms,
            .limits = limits_per_scaler,
            .size = target_size,
            .cnt = perm_cnt,
            //TODO: instead iteration could just happen here and pass combined result
            .parent_perms = fpctx->parent_perms,
        };

        /* call the given callback with the given context and result */
        fpctx->cb(fpctx->gctx, &res_ctx);

        permcntsum += pidx;
    }
    return permcntsum;
}

static bool _map_factors_to_topo(clk_topology_entry_t *topo,
                                 size_t *cidxs, size_t cidx_len, uint32_t *factors,
                                 bool *contiguous_range)
{
    size_t ci;
    for (unsigned i = 0; i < cidx_len; i++) {
        ci = cidxs[i];
        const gclk_t *clk = topo[ci].clk;

        /* if factor range is contiguous, the min/max limit checks
         * which happened before already confirm that this factor is mappable.
         * If the factor range is non-contiguous, check if the factor is actually one of the
         * appliccable values. */
        if (!contiguous_range[i]) {
            /* check if the factor can be matched */
            if (gclk_factor2idx(clk, factors[i]) >= gclk_factor_cnt(clk)) {
                return false;
            }
        }

        topo[ci].factor = factors[i];
    }
    return true;
}

static bool _map_mdfactors_to_topo(clk_topology_entry_t *topo,
                                 topo_scaler_index_meta_t *idx_meta,
                                 uint32_t *mf, uint32_t *df)
{
    if (!_map_factors_to_topo(topo, idx_meta->mul_idxs, idx_meta->mul_len,
                              mf, idx_meta->contiguous_factor_range)) {
        return false;
    }

    return _map_factors_to_topo(topo, idx_meta->div_idxs, idx_meta->div_len,
                                df, &idx_meta->contiguous_factor_range[idx_meta->mul_len]);

    //size_t ci;
    //for (unsigned i = 0; i < idx_meta->mul_len; i++) {
    //    ci = idx_meta->mul_idxs[i];
    //    const gclk_t *clk = topo[ci].clk;

    //    /* if factor range is contiguous, the min/max limit checks
    //     * which happened before already confirm that this factor is mappable.
    //     * If the factor range is non-contiguous, check if the factor is actually one of the
    //     * appliccable values. */
    //    if (!idx_meta->contiguous_factor_range[i]) {
    //        /* check if the factor can be matched */
    //        if (gclk_factor2idx(clk, mf[i]) >= gclk_factor_cnt(clk)) {
    //            return false;
    //        }
    //    }

    //    topo[ci].factor = mf[i];
    //    //topology[ci].propagation_pending = true;
    //}

    //for (unsigned i = 0; i < idx_meta->div_len; i++) {
    //    ci = idx_meta->div_idxs[i];
    //    const gclk_t *clk = topo[ci].clk;

    //    /* if factor range is contiguous, the min/max limit checks
    //     * which happened before already confirm that this factor is mappable.
    //     * If the factor range is non-contiguous, check if the factor is actually one of the
    //     * appliccable values. */
    //    if (!idx_meta->contiguous_factor_range[idx_meta->mul_len + i]) {
    //        /* check if the factor can be matched */
    //        if (gclk_factor2idx(clk, mf[i]) >= gclk_factor_cnt(clk)) {
    //            return false;
    //        }
    //    }

    //    topo[ci].factor = df[i];
    //}

    //return true;
}

/* check if permutation is allowed (len of limits must match len of pemutation) */
static bool _perm_within_limits(perm_meta_t *p, gclk_factor_limit_t *limits)
{
    for (unsigned j = 0 ; j < p->cnt; j++) {
        uint32_t factor = p->vals[j];
        if ((factor < limits[j].min) || (factor > limits[j].max)) {
            return false;
        }
    }
    return true;
}

static void _print_md_vals(perm_meta_t *m, perm_meta_t *d)
{
    printf(" x { ");
    for (unsigned i = 0; i < m->cnt; i++) {
        printf("%lu ", m->vals[i]);
    }
    printf("} / { ");
    for (unsigned i = 0; i < d->cnt; i++) {
        printf("%lu ", d->vals[i]);
    }
    printf("}\n");
}

static void _cpy_md_vals_to_res_buf(perm_meta_t *m, perm_meta_t *d, uint32_t *res)
{
    size_t fidx = 0;
    for (unsigned i = 0; i < m->cnt; i++) {
        res[fidx++] = m->vals[i];
    }
    for (unsigned i = 0; i < d->cnt; i++) {
        res[fidx++] = d->vals[i];
    }
}

void factor_permutation_found_print_cb(topo_perm_glob_ctx_t *globctx, fact_perm_result_ctx_t *resctx)
{
    /* for each permutation of div values ... */
    for (unsigned dpi = 0 ; dpi < resctx->cnt; dpi++) {
        /* current div permutation */
        perm_meta_t dperm = { .vals = &resctx->vals[dpi * resctx->size],
                              .cnt = resctx->size };

        /* if there are divisors in the topology, check their limits */
        if ((globctx->scaler_idx_meta->div_len > 0) &&
            !_perm_within_limits(&dperm, resctx->limits)) {
            /* try with next div permutation, mul permutations don't need to be checked */
            continue;
        }

        fact_perm_result_ctx_t *mul_permutations = resctx->parent_perms;
        assert(mul_permutations);

        /* ... combine with each permutation of mul values */
        for (unsigned mpi = 0; mpi < mul_permutations->cnt; mpi++) {
            /* current mul permutation */
            //uint32_t *mperm = &mul_permutations->vals[mpi * mul_permutations->size];
            perm_meta_t mperm = { .vals = &mul_permutations->vals[mpi * mul_permutations->size],
                                  .cnt = mul_permutations->size };

            if ((globctx->scaler_idx_meta->mul_len > 0) &&
                !_perm_within_limits(&mperm, globctx->mlims)) {
                /* try with next div permutation, mul permutations don't need to be checked */
                continue;
            }

            /* now that we know the factor combination can be matched, check if the resulting
             * frequency adheres to all limits */
            bool flim_conflict = false;
            bool mappable = _map_mdfactors_to_topo(globctx->topo, globctx->scaler_idx_meta, mperm.vals, dperm.vals);
            if (!mappable) {
                continue;
            }

            globctx->mappable_perm_cnt++;

            // TODO: there may be room for speedups if there is only a small number of
            //       limits and we can check them directly instead of populating the whole
            //       topology */
            bool all_freqvals_valid = gclk_calculate_topology_config_freqs(globctx->topo, globctx->topo_len);
            if (!all_freqvals_valid) {
                continue;
            }
            for (unsigned fli = 0; fli < globctx->flim_cnt; fli++) {
                gclk_freq_limit_t *fl = &globctx->flims[fli];
                clk_topology_entry_t *te = globctx->flim_tes[fli];
                if (te->clk_freq < fl->min || te->clk_freq > fl->max) {
                    flim_conflict = true;
                    break;
                }
            }

            if(flim_conflict) {
                continue;
            }

            if (globctx->result_filter &&
               (globctx->result_filter->specific_config_index == globctx->valid_perm_cnt)){
                /* indicate to caller that the searched config was found */
                globctx->result_filter->best_found = true;
                _print_md_vals(&mperm, &dperm);
                _cpy_md_vals_to_res_buf(&mperm, &dperm, globctx->result_buf);
            } else {
                if (globctx->topo_conf_iter) {
                    void * cbctx = globctx->topo_conf_iter->cbctx;
                    globctx->topo_conf_iter->topo_conf_found(globctx->topo, globctx->topo_len, cbctx);
                }
            }
            /* incremant counter of valid configs */
            globctx->valid_perm_cnt++;
        }
    }
}

/* for each mul permutation this calback gets called for, derive all applicable divisor permutations
 * and combine each mul permutation with each div permutations*/
void on_mp_found_combine_with_dp(topo_perm_glob_ctx_t *gctx, fact_perm_result_ctx_t *resctx)
{
    (void)resctx;

    mp_init_set(gctx->dpartitions,
            mul_part_val_buf, ARRAY_SIZE(mul_part_val_buf),
            mul_part_meta_buf, ARRAY_SIZE(mul_part_meta_buf));
    size_t dclen = gctx->scaler_idx_meta->div_len;
    if (!dclen) { /* provide enough space for the trivial div of /1 */
        dclen = 1;
    }

    gclk_factor_perm_cb_ctx_t fpctx = {
        .cb = factor_permutation_found_print_cb,
        .gctx = gctx,
        .parent_perms = resctx,
    };

    uint32_t dpcnt = get_mul_partitions(gctx->eff_div, gctx->dpartitions, dclen);
    gctx->factor_part_cnt += dpcnt;
    gctx->factor_part_len += gctx->dpartitions->val_idx;
    //printf("got %lu partitions with %u values:\n", dpcnt, gctx->dpartitions->val_idx);
    //for (unsigned i = 0; i < dpcnt; i++) {
    //    mul_partition_meta_t *part = mp_get_partition(gctx->dpartitions, i);
    //    mp_print_partition(part, true);
    //}

    if (dpcnt > 0) {
        mp_set_permutations(gctx->dpartitions, dclen, &fpctx,
                            &gctx->dlims[0], gctx->d_topo_lims);
    }

}

static size_t get_scalable_idxs(clk_topology_entry_t *topo, size_t len, int types_msk, size_t *idxs)
{
    size_t cnt = 0;
    for (unsigned t = 0; t < len; t++) {
        if ( ((types_msk & GCLK_MUL) && gclk_is_multiplier(topo[t].clk)) ||
             ((types_msk & GCLK_DIV) && gclk_is_divider(topo[t].clk))) {
            if (idxs) {
                idxs[cnt] = t;
            }
            cnt++;
        }
    }
    return cnt;
}
extern uint64_t gclk_get_factors_config_cnt64_from_topology(clk_topology_entry_t *topology, size_t len);
#include "fmt.h"

static void _update_limits(gclk_factor_limit_t *bound, gclk_factor_limit_t *val)
{
    if (val->max > bound->max) {
        bound->max = val->max;
    }
    if (val->min < bound->min) {
        bound->min = val->min;
    }
}

static void _get_scaler_limits(size_t *clk_idxs, size_t clk_idx_cnt, const clk_topology_entry_t *topo,
                               gclk_factor_limit_t *scaler_limits,   /* limits of each scaler instance */
                               gclk_factor_limit_t *list_limits,     /* limits over all given scalers */
                               gclk_factor_limit_t *chain_limits)    /* limits when chaining all scalers */
{
    /* init to 1 as neutral element for mul / div */
    chain_limits->min = 1;
    chain_limits->max = 1;
    /* statically limit to 1 if there is no scaler at all */
    list_limits->min = clk_idx_cnt > 0 ? UINT32_MAX : 1;
    list_limits->max = 1;

    for (size_t i = 0; i < clk_idx_cnt; i++) {
        int ci = clk_idxs[i];
        const gclk_t *c = topo[ci].clk;
        gclk_factor_limit_t *slim = &scaler_limits[i];
        gclk_get_factor_minmax(c, slim);

        /* update list-wide limits */
        _update_limits(list_limits, slim);

        /* update limits for chained scalers */
        chain_limits->min *= slim->min;
        chain_limits->max *= slim->max;
    }
}

static void _populate_contiguous_markers(const clk_topology_entry_t *topo,
                                         size_t *cidxs, size_t cidx_len,
                                         bool *contiguous_markers)
{
    for (size_t i = 0; i < cidx_len; i++) {
        size_t ci = cidxs[i];
        const gclk_t *clk = topo[ci].clk;

        unsigned cnt = gclk_factor_cnt(clk);
        //unsigned min = gclk_factor_min(clk);
        //unsigned max = gclk_factor_min(clk);
        //if ((max - min) > cnt) {
        //    idx_meta->contiguous_factor_range[i] = false;
        //    continue;
        //}

        bool contiguous = true;
        unsigned last = 0;
        /* check if the factor range is contiguous */
        for (unsigned fi = 0; fi < cnt; fi++) {
            unsigned x = gclk_idx2factor(clk, fi);
            if (fi > 0) {
                if (last + 1 != x) {
                    contiguous = false;
                    break;
                }
            }
            last = x;
        }

        contiguous_markers[i] = contiguous;
    }
}

static void _populate_factor_range_contiguous_markers(const clk_topology_entry_t *topo,
                                                      topo_scaler_index_meta_t *idx_meta)
{
    _populate_contiguous_markers(topo, idx_meta->mul_idxs, idx_meta->mul_len,
                                 idx_meta->contiguous_factor_range);

    _populate_contiguous_markers(topo, idx_meta->div_idxs, idx_meta->div_len,
                                 &idx_meta->contiguous_factor_range[idx_meta->mul_len]);
}

void _print_topo_conf_header(clk_topology_entry_t *topology, size_t len)
{
    printf("gcid, mcid, vcid, ");
    for (uint32_t i = 0; i < len; i++) {
        const gclk_t *c = topology[i].clk;
        if (i > 0) {
            printf(", ");
        }
        printf("%s", gclk_get_name(c));

        if (gclk_is_scalable(c)) {
            printf(", __%s_%s", gclk_is_multiplier(c) ? "mul" : "div", gclk_get_name(c));
        }
    }
    printf(", ws, vcidx, vcmv, dvspol");
    printf("\n");
}

clk_topology_entry_t _vs_alt_topo[GCLK_NUM_OF_CLOCKS];
gclk_manager_sequence_step_t _vs_alt_seq[20];

int _get_vsvc_alternatives(clk_topology_entry_t *target_topo, size_t len,
                            unsigned *ws_lv, unsigned *vc_lv,
                            unsigned *ws_ff, unsigned *vc_ff)
{
    unsigned int cur_topolen = gclk_get_current_topology_len(target_topo[0].clk);
    gclk_get_current_topology_config(target_topo[0].clk, _vs_alt_topo, cur_topolen);

    int seq_len = gclk_manager_derive_sequence(_vs_alt_topo, cur_topolen,
                                                target_topo, len,
                                                _vs_alt_seq, ARRAY_SIZE(_vs_alt_seq));

    if (seq_len > 0) {
        /* copy the current tree conf into a buffer to simulate changes on it */
        for (unsigned i = 0; i < GCLK_NUM_OF_CLOCKS; i++) {
            gclk_get_current_topology_config(gclk_get(i), &_print_valid_confs_tree_conf_buf[i], 1);
        }

        for (size_t si = 0; si < (unsigned)seq_len; si++) {
            gclk_manager_simulate_seq_step_on_tree_conf(&_vs_alt_seq[si], _print_valid_confs_tree_conf_buf, GCLK_NUM_OF_CLOCKS);
        }

        gclk_get_min_required_ws_vc_from_tree_config(_print_valid_confs_tree_conf_buf,
                                                     GCLK_NUM_OF_CLOCKS, ws_lv, vc_lv,
                                                     DVS_PREFER_LOW_VOLTAGE);

        gclk_get_min_required_ws_vc_from_tree_config(_print_valid_confs_tree_conf_buf,
                                                     GCLK_NUM_OF_CLOCKS, ws_ff, vc_ff,
                                                     DVS_PREFER_FAST_FLASH);
        return 1;
    }

    return -1;
}

void _print_topo_conf_row(clk_topology_entry_t *topology, size_t len, uint32_t permid, uint32_t vcid)
{
    unsigned ws_lv = 0;
    unsigned vc_lv = 0;
    unsigned ws_ff = 0;
    unsigned vc_ff = 0;

    _get_vsvc_alternatives(topology, len, &ws_lv, &vc_lv, &ws_ff, &vc_ff);

    for (unsigned i = 0; i < 2; i++) {
        unsigned ws = i == 0 ? ws_lv : ws_ff;
        unsigned vc = i == 0 ? vc_lv : vc_ff;

        uint64_t gcid = gclk_topology2factor_conf_id(topology, len);
        print_u64_dec(gcid);
        printf(", ");
        printf("%lu, ", permid);
        printf("%lu, ", vcid);

        for (uint32_t i = 0; i < len; i++) {
            if (i > 0) {
                printf(", ");
            }

            gclk_freq_str_t fs;
            char *fstr = gclk_freq2str(topology[i].clk_freq, &fs);
            printf("%s", fstr);

            if (gclk_is_scalable(topology[i].clk)) {
                printf(", %u", topology[i].factor);
            }
        }
        printf(", %u", ws);
        printf(", %u", vc);
        printf(", %u", core_voltage_idx2mv(vc));
        printf(", %s", i == 0 ? "LV" : "FF");
        printf("\n");
        if (ws_lv == ws_ff && vc_lv == vc_ff) {
            break;
        }
    }
}

void _topo_conf_found_print_cb(clk_topology_entry_t *topology, size_t len, void *ctx)
{
    topo_perm_glob_ctx_t *gctx = (topo_perm_glob_ctx_t*)ctx;
    uint32_t mappable_id = gctx->mappable_perm_cnt;
    uint32_t valid_id = gctx->valid_perm_cnt;
    _print_topo_conf_row(topology, len, mappable_id, valid_id);
}

clk_topology_entry_t _cgcmd_current_topology[20];
gclk_manager_sequence_step_t _cgcmd_out_seq[20];

static const char* _USAGE_CLOSEST = "<clk_name> <target_freq_hz> [<topology_id> [<cmp_fun> {cntall | listall | <conf_idx>]]]]";

static int _get_closest_freq(int argc, char **argv)
{
    if (! ((argc >= 3) && (argc <= 6))) {
        goto usage;
    }

    const gclk_t *clk = gclk_get_clk_by_name(argv[1]);
    uint32_t target_freq = atoi(argv[2]);
    int tidx = GCLK_UNDEFINED_TOPOLOGY;
    bool listall = false;
    bool cntall = false;
    bool best_only = false;

    if (argc >= 4 && argc <= 6) {
        if (_arg_is_valid_number(argv[3], true)) {
            tidx = atoi(argv[3]);
        } else {
            goto usage;
        }
    }

    gclk_cmp_func_t cmp_func = gclk_cmp_topology_for_closest_leaf_freq;
    int specific_conf_idx = -1;

    if (argc >= 5) {
        cmp_func = _get_topology_cmp_fptr(argv[4]);
        if (!cmp_func) {
            printf("cmp_fun must be one of {");
            for (unsigned i = 0; i < ARRAY_SIZE(topology_cmp_funcs); i++) {
                printf("%c%s", i > 0 ? '|' : ' ', topology_cmp_funcs[i].name);
            }
            printf(" }\n");
            goto usage;
        }

        if (argc == 6) {
            if (strcmp("listall", argv[5]) == 0) {
                listall = true;
            } else if (strcmp("cntall", argv[5]) == 0) {
                cntall = true;
            } else if (_arg_is_valid_number(argv[5], true)) {
                specific_conf_idx = atoi(argv[5]);
            } else {
                goto usage;
            }
         } else {
            best_only = true;
         }
    }

    if (clk) {
        uint32_t max_involved_clks = gclk_get_clk_subtree_max_depth(clk, 0);
        printf("max topology for driving %s employs %lu clock nodes\n", gclk_get_name(clk), max_involved_clks);
        clk_topology_entry_t topology[max_involved_clks];
        memset(topology, 0, sizeof(clk_topology_entry_t) * max_involved_clks);

        gpio_clear(LOGIC_ANALYZER_PIN);
        /* this should be the only field set to describe where we want to start,
           everything else must be zero at the beginning */
        topology[0].clk = clk;
        topology[0].clk_freq = GCLK_INVALID_FREQ;

        size_t valid_cnt = 0;

        /* first, always check how many valid confs are there */
        int force_nth = -1;
        uint32_t target_freq_copy = target_freq;
        uint32_t best_valid_idx = GCLK_MANAGER_INVALID_CONF_IDX;

        gclk_exploration_result_cb_conf_t valid_cb_save_best_idx_conf = {
            .valid_conf_found_cb = _store_best_conf_idx_cb,
            .ctx = &best_valid_idx,
        };

        if (tidx != GCLK_UNDEFINED_TOPOLOGY) {
            uint32_t _tmp_max_len =  max_involved_clks;
            clk_topology_entry_t _tmp_topo[_tmp_max_len];
            _tmp_topo[0].clk = clk;
            int _tmp_len = gclk_get_nth_topology(_tmp_topo, _tmp_max_len, tidx);
            uint32_t src_freq = gclk_get_input_freq(_tmp_topo[_tmp_len-1].clk);

            topo_scaler_index_meta_t scaler_idx_meta;

            scaler_idx_meta.mul_len = get_scalable_idxs(_tmp_topo, _tmp_len, GCLK_MUL, NULL);
            scaler_idx_meta.div_len = get_scalable_idxs(_tmp_topo, _tmp_len, GCLK_DIV, NULL);

            size_t mclks[scaler_idx_meta.mul_len];
            size_t dclks[scaler_idx_meta.div_len];
            get_scalable_idxs(_tmp_topo, _tmp_len, GCLK_MUL, mclks);
            get_scalable_idxs(_tmp_topo, _tmp_len, GCLK_DIV, dclks);

            scaler_idx_meta.mul_idxs = mclks;
            scaler_idx_meta.div_idxs = dclks;

            bool contiguous_factor_range[scaler_idx_meta.mul_len + scaler_idx_meta.div_len];
            scaler_idx_meta.contiguous_factor_range = contiguous_factor_range;
            _populate_factor_range_contiguous_markers(_tmp_topo, &scaler_idx_meta);

            gclk_factor_limit_t mlims[scaler_idx_meta.mul_len];
            gclk_factor_limit_t m_list_lims;
            gclk_factor_limit_t m_chain_lims;
            _get_scaler_limits(mclks, scaler_idx_meta.mul_len, _tmp_topo, mlims, &m_list_lims, &m_chain_lims);

            gclk_factor_limit_t dlims[scaler_idx_meta.div_len];
            gclk_factor_limit_t d_list_lims;
            gclk_factor_limit_t d_chain_lims;
            _get_scaler_limits(dclks, scaler_idx_meta.div_len, _tmp_topo, dlims, &d_list_lims, &d_chain_lims);

            uint32_t hf = target_freq;
            uint32_t lf = src_freq;

            uint32_t gcd = gcd32(hf, lf);
            uint32_t base_frac_vals[] = {hf / gcd, lf / gcd};

            printf("equivalent factor: * (%lu/%lu)\n", base_frac_vals[0], base_frac_vals[1]);
            size_t mfcnt = scaler_idx_meta.mul_len;
            /* if no real scalers are there to carry an actual value,
             * at least provide enough space for the trivial (logical) partition *1 or /1 */
            if (!mfcnt) {
                mfcnt = 1;
            }
            size_t dfcnt = scaler_idx_meta.div_len;
            if (!dfcnt) {
                dfcnt = 1;
            }

            uint32_t result_factors[mfcnt + dfcnt];
            uint32_t mult = 1;

            size_t constr_clk_cnt = gclk_manager_get_constrained_clk_cnt(_tmp_topo, _tmp_len);
            gclk_freq_limit_t freq_lims[constr_clk_cnt];
            clk_topology_entry_t *flim_tes[constr_clk_cnt];

            unsigned flidx = 0;
            for (int c = 0; c < _tmp_len; c++) {
                const gclk_t *clk = _tmp_topo[c].clk;
                if (gclk_manager_get_freq_constrained_freq_limit(clk, &freq_lims[flidx])) {
                    /* store a reference to the topo entry for fast lookup later */
                    assert(flidx < constr_clk_cnt);
                    flim_tes[flidx] = &_tmp_topo[c];
                    flidx++;
                    if (flidx == constr_clk_cnt) {
                        break;
                    }
                }
            }

            topo_perm_filter_ctx_t result_filter = {
                .specific_config_index = specific_conf_idx,
                .best_found = false,
            };

            gclk_topo_conf_iter_ctx_t topo_conf_iter = {
                .topo_conf_found = cntall ? NULL : _topo_conf_found_print_cb,
                .cbctx = NULL,
            };

            topo_perm_glob_ctx_t gctx = {
                .topo = &_tmp_topo[0],
                .topo_len = _tmp_len,
                .scaler_idx_meta = &scaler_idx_meta,
                .factor_part_cnt = 0,
                .factor_part_len = 0,
                .valid_perm_cnt = 0,
                .mappable_perm_cnt = 0,
                .flims = freq_lims,
                .flim_tes = flim_tes,
                .flim_cnt = constr_clk_cnt,
                .dlims = dlims,
                .d_topo_lims = &d_list_lims,
                .mlims = mlims,
                .m_topo_lims = &m_list_lims,
                .result_filter = (specific_conf_idx == -1) ? NULL : &result_filter,
                .result_buf = result_factors,
                .topo_conf_iter = listall ? &topo_conf_iter : NULL,
            };

            _print_topo_conf_header(_tmp_topo, _tmp_len);

            while (true) {
                uint32_t frac_vals[] = {mult * base_frac_vals[0], mult * base_frac_vals[1]};

                if (frac_vals[0] > m_chain_lims.max) {
                    break;
                } else if (frac_vals[1] > d_chain_lims.max) {
                    break;
                }

                mult++;
                /* if the whole chain minimum is not reached the iteration may immediately
                 * continue at the next higher multiple */
                if (frac_vals[0] < m_chain_lims.min) {
                    continue;
                } else if (frac_vals[1] < d_chain_lims.min) {
                    continue;
                }

                mul_partition_set_t mpartitions;
                mp_init_set(&mpartitions,
                            mul_part_val_buf, ARRAY_SIZE(mul_part_val_buf),
                            mul_part_meta_buf, ARRAY_SIZE(mul_part_meta_buf));

                size_t mparts = get_mul_partitions(frac_vals[0], &mpartitions, mfcnt);

                mul_partition_set_t dpartitions;

                /* verify if values and partition metadata could be stored in the buffer */
                assert(mpartitions.val_idx < mpartitions.val_len);
                assert(mpartitions.part_idx < mpartitions.part_len);

                gctx.mpartitions = &mpartitions;
                gctx.dpartitions = &dpartitions;
                gctx.mpv_buf = &mul_part_val_buf[mpartitions.val_idx],
                gctx.mpv_len = ARRAY_SIZE(mul_part_val_buf) - mpartitions.val_idx,
                gctx.mpm_buf = &mul_part_meta_buf[mpartitions.part_idx],
                gctx.mpm_len = ARRAY_SIZE(mul_part_meta_buf) - mpartitions.part_len,
                gctx.eff_mul = frac_vals[0],
                gctx.eff_div = frac_vals[1],

                topo_conf_iter.cbctx = &gctx;

                gclk_factor_perm_cb_ctx_t fpctx = {
                    .cb = on_mp_found_combine_with_dp,
                    .gctx = &gctx,
                    .parent_perms = NULL,
                };

                if (mparts > 0) {
                    //mp_print_all_partitions(&partitions);
                    mp_set_permutations(&mpartitions, mfcnt, &fpctx,
                                        gctx.mlims, gctx.m_topo_lims);
                }

                if (gctx.result_filter && gctx.result_filter->best_found) {
                    break;
                }
            }

            if (specific_conf_idx != -1) {
                for (unsigned i = 0; i < (mfcnt + dfcnt); i++) {
                    printf("%lu ", result_factors[i]);
                }
                printf("\n");
                size_t tlen = gclk_get_nth_topology(topology, max_involved_clks, tidx);

                /* offset div factors for trivial *1 if there are no real mul factors */
                size_t dfoffset = scaler_idx_meta.mul_len ? scaler_idx_meta.mul_len : 1;
                _map_mdfactors_to_topo(topology, &scaler_idx_meta, &result_factors[0], &result_factors[dfoffset]);
                gclk_calculate_topology_config_freqs(topology, tlen);
                gclk_print_topology(topology, tlen);
                gclk_manager_print_topology_conf(topology, tlen, false, true);

                memset(_cgcmd_current_topology, 0, sizeof(clk_topology_entry_t) * max_involved_clks);
                unsigned int cur_topolen = gclk_get_current_topology_len(clk);
                gclk_get_current_topology_config(clk, _cgcmd_current_topology, cur_topolen);
                int cur_topo_idx = gclk_topology2id(_cgcmd_current_topology, cur_topolen);

                int seq_size = gclk_manager_derive_sequence(_cgcmd_current_topology, cur_topolen,
                                                            topology, tlen, _cgcmd_out_seq, ARRAY_SIZE(_cgcmd_out_seq));

                if (seq_size > 0) {
                    gclk_manager_print_step_sequence(_cgcmd_out_seq, seq_size);
                    //gclk_manager_run_sequence_with_notify(_cgcmd_out_seq, seq_size, true);
                    gclk_manager_run_sequence_with_notify(_cgcmd_out_seq, seq_size, false);
                } else {
                    printf("transition from [%s] topology from %d to %d infeasible!\n", gclk_get_name(clk), cur_topo_idx, tidx);
                }
            } else {
                uint64_t configs = gclk_get_factors_config_cnt64_from_topology(_tmp_topo, _tmp_len);
                printf("Size of configuration space: ");
                print_u64_dec(configs);
                printf("\nOverall partitions: %lu\n", gctx.factor_part_cnt);
                printf("Overall mappable permutations: %lu\n", gctx.mappable_perm_cnt);
                printf("Overall valid permutations: %lu\n", gctx.valid_perm_cnt);
            }

            return 0;

        }


        printf("start BF\n");
        /* get count of possible configs first */
        gclk_manager_brute_force_freq_conf(clk, topology, &max_involved_clks, &tidx,
                cmp_func, (void*)&target_freq_copy, &valid_cnt, force_nth, &valid_cb_save_best_idx_conf);
        printf("BF done\n");

        size_t all_fact_cnt = gclk_get_factors_config_cnt_from_topology(topology, max_involved_clks);
        printf("There are %u valid configs (out of %u possible configs) for [%s]@%lu Hz via topology [", valid_cnt, all_fact_cnt, gclk_get_name(clk), target_freq_copy);

        if (tidx == GCLK_UNDEFINED_TOPOLOGY) {
            printf("*]:\n");
        } else {
            printf("%d]:\n", tidx);
        }

        if (valid_cnt > 0) {
            /* only show one specific conf (which must be in range of valid confs) */
            if (best_only) {
                if (valid_cnt > best_valid_idx) {
                    specific_conf_idx = best_valid_idx;
                    force_nth = specific_conf_idx;
                    printf("config with index %d:\n", specific_conf_idx);
                } else {
                    printf("there is no valid configuration with idx %d (try 0 - %u)\n", specific_conf_idx, valid_cnt - 1);
                    goto usage;
                }
            } else if (listall) {
                printf("all valid configs:\n");
            }

            memset(topology, 0, sizeof(clk_topology_entry_t) * max_involved_clks);
            topology[0].clk = clk;
            topology[0].clk_freq = GCLK_INVALID_FREQ;

            gclk_exploration_result_cb_conf_t valid_cb_print_conf = {
                .valid_conf_found_cb = _print_valid_conf_cb,
                .ctx = NULL,
                .cb_mode =  listall ? CB_ON_VALID : CB_ON_BEST_ONLY,
            };

            gclk_manager_brute_force_freq_conf(clk, topology, &max_involved_clks, &tidx,
                    cmp_func, (void*)&target_freq_copy, &valid_cnt, force_nth, &valid_cb_print_conf);

        } else {
            printf("No valid frequency configuration found!\n");
        }
    } else {
        printf("could not find clock named %s\n", argv[1]);
    }

    ///* reset logic analyzer pin to idle state */
    //gpio_clear(LOGIC_ANALYZER_PIN);
    return 0;

usage:
    return PRINT_USAGE;
}

static const char* _USAGE_CONFIG = "<clk_name> <topology_id> <topo_conf_id>";
static int _clockman_config(int argc, char **argv)
{
    if (!(argc == 4)) {
        return PRINT_USAGE;
    }

    const gclk_t *clk = gclk_get_clk_by_name(argv[1]);
    size_t tidx = atoi(argv[2]);
    //uint64_t topo_conf_id = atoi(argv[3]);
    char *str_end;
    uint64_t topo_conf_id = strtoull(argv[3], &str_end, 10);
    printf("topo id: %d\n", tidx);
    printf("topo conf id:");
    print_u64_dec(topo_conf_id);
    printf("\n");

    uint32_t max_involved_clks = gclk_get_clk_subtree_max_depth(clk, 0);
    printf("max topology for driving %s employs %lu clock nodes\n", gclk_get_name(clk), max_involved_clks);
    clk_topology_entry_t tgt_topo[max_involved_clks];
    memset(tgt_topo, 0, sizeof(clk_topology_entry_t) * max_involved_clks);
    tgt_topo[0].clk = clk;
    size_t ttlen = gclk_get_nth_topology(tgt_topo, max_involved_clks, tidx);
    gclk_confid2_topology_conf(topo_conf_id, tgt_topo, ttlen);
    gclk_calculate_topology_config_freqs(tgt_topo, ttlen);
    gclk_manager_print_topology_conf(tgt_topo, ttlen, false, true);

    unsigned int cur_topolen = gclk_get_current_topology_len(clk);
    clk_topology_entry_t cur_topo[cur_topolen];
    memset(cur_topo, 0, sizeof(clk_topology_entry_t) * cur_topolen);
    gclk_get_current_topology_config(clk, cur_topo, cur_topolen);
    int cur_topo_idx = gclk_topology2id(cur_topo, cur_topolen);

    gclk_manager_sequence_step_t out_seq[20];

    int seq_size = gclk_manager_derive_sequence(cur_topo, cur_topolen,
                                                tgt_topo, ttlen, out_seq,
                                                ARRAY_SIZE(out_seq));

    if (seq_size > 0) {
        gclk_manager_print_step_sequence(out_seq, seq_size);
        gclk_manager_run_sequence_with_notify(out_seq, seq_size, false);
    } else {
        printf("transition from [%s] topology from %d to %d infeasible!\n", gclk_get_name(clk), cur_topo_idx, tidx);
    }

    return 0;
}

//TODO: rework the use of the intermediate clock to just use the topology switch definitiions provided by gclk_manager_conf.h file
int _sc_setup_closest_freq(int argc, char **argv)
{
    if (!((argc == 5) || (argc == 6) || (argc == 7))) {
        printf("Usage: %s <clk_name> <intermediate_clk_name> <target_freq_hz> <vscaleid> [topology_id] [cmp_fun]\n", argv[0]);
        return 1;
    }

    const gclk_t *clk = gclk_get_clk_by_name(argv[1]);
    const gclk_t *intermediate_clk = gclk_get_clk_by_name(argv[2]);

    uint32_t target_freq = atoi(argv[3]);
    //uint32_t vscale_id = atoi(argv[4]);

    int tidx = GCLK_UNDEFINED_TOPOLOGY;

    if ((argc == 6) || (argc == 7)) {
        tidx = atoi(argv[5]);
    }

    gclk_cmp_func_t cmp_func = gclk_cmp_topology_for_closest_leaf_freq;
    if (argc == 7) {
        cmp_func = _get_topology_cmp_fptr(argv[6]);
    }

    if (clk) {
        uint32_t max_involved_clks = gclk_get_clk_subtree_max_depth(clk, 0);

        clk_topology_entry_t best_topology[max_involved_clks];
        memset(best_topology, 0, sizeof(clk_topology_entry_t) * max_involved_clks);
        best_topology[0].clk = clk;
        best_topology[0].clk_freq = GCLK_INVALID_FREQ;

        gpio_clear(LOGIC_ANALYZER_PIN);
        size_t valid_cnt = 0;
        int force_nth = -1;

        long leaf_freq = gclk_manager_brute_force_freq_conf(clk, best_topology, &max_involved_clks, &tidx,
                                                            cmp_func, (void*)&target_freq, &valid_cnt, force_nth, NULL);

        gpio_set(LOGIC_ANALYZER_PIN);
        gclk_enable(intermediate_clk);
        _set_parent(clk, intermediate_clk);
        gpio_clear(LOGIC_ANALYZER_PIN);
        (void)leaf_freq;

        for (unsigned i = (max_involved_clks - 1); i >= 1; i--) {
            if (best_topology[i].clk->flags.topology_flags & GCLK_STOP_FOR_UPDATE) {
                LOG_DEBUG("disable: [%s]\n", gclk_get_name(best_topology[i].clk));
                gclk_disable(best_topology[i].clk);
            }

            if (best_topology[i].clk->flags.topology_flags & GCLK_STOP_PARENT_FOR_UPDATE) {
                LOG_DEBUG("disable: [%s]\n", gclk_get_name(best_topology[i+1].clk));
                gclk_disable(best_topology[i+1].clk);
            }

            if (best_topology[i].clk->flags.topology_flags & GCLK_STOP_CHILDREN_FOR_UPDATE) {
                LOG_DEBUG("disable: [%s]\n", gclk_get_name(best_topology[i-1].clk));
                gclk_disable(best_topology[i-1].clk);
            }

            gclk_set_freq(best_topology[i].clk, best_topology[i].clk_freq);
            gclk_freq_str_t fs;
            char *fstr = gclk_freq2str(best_topology[i].clk_freq, &fs);
            LOG_DEBUG("set_freq of [%s] to %s\n", gclk_get_name(best_topology[i].clk), fstr);
        }

        gpio_set(LOGIC_ANALYZER_PIN);

        for (unsigned i = (max_involved_clks - 1); i >= 1; i--) {
            LOG_DEBUG("enable: [%s]\n", gclk_get_name(best_topology[i].clk));
            gclk_enable(best_topology[i].clk);
        }

        gpio_clear(LOGIC_ANALYZER_PIN);

        LOG_DEBUG("set parent of [%s] to [%s]\n",  gclk_get_name(clk), gclk_get_name(best_topology[1].clk));
        _set_parent(clk, best_topology[1].clk);

        gpio_set(LOGIC_ANALYZER_PIN);
        gpio_clear(LOGIC_ANALYZER_PIN);

    } else {
        printf("could not find clock named %s\n", argv[1]);
    }

    /* output something to ensure flushing the UART after the clock change */
    printf("sync\n");
    printf("pulse_description: bruteforce-cfg set-temp-parent set-freqs enable-all-clks set-final-parent adapt-flash-ws\n");

    /* switch instrumentation pin back to idle state when done */
    gpio_set(LOGIC_ANALYZER_PIN);

    return 0;
}

static const char* _CLOCKMAN_SUBCMD_CLOSEST = "closest";
static const char* _CLOCKMAN_SUBCMD_WAKEUP_SEQ = "wakeupseq";
static const char* _CLOCKMAN_SUBCMD_CONFIG = "config";

int _sc_clockman(int argc, char **argv)
{
    int res = PRINT_USAGE;
    if (strcmp(argv[1], _CLOCKMAN_SUBCMD_CLOSEST) == 0) {
        res = _get_closest_freq(argc - 1, &argv[1]);
        if (res == PRINT_USAGE) {
            printf("usage: %s %s %s\n", argv[0], _CLOCKMAN_SUBCMD_CLOSEST, _USAGE_CLOSEST);
        }
    } else if (strcmp(argv[1], _CLOCKMAN_SUBCMD_WAKEUP_SEQ) == 0) {
        gclk_manager_sequence_step_t *seq;
        size_t len = gclk_manager_get_wakeup_seq(&seq);
        gclk_manager_print_step_sequence(seq, len);
        res = 0;
    } else if (strcmp(argv[1], _CLOCKMAN_SUBCMD_CONFIG) == 0) {
        res = _clockman_config(argc - 1, &argv[1]);
        if (res == PRINT_USAGE) {
            printf("usage: %s %s %s\n", argv[0], _CLOCKMAN_SUBCMD_CONFIG, _USAGE_CONFIG);
        }
    } else {
        printf("usage: %s { %s | %s | %s } [...]\n", argv[0],
                                                     _CLOCKMAN_SUBCMD_CLOSEST,
                                                     _CLOCKMAN_SUBCMD_WAKEUP_SEQ,
                                                     _CLOCKMAN_SUBCMD_CONFIG);
        return -1;
    }

    return 0;
}

int _sc_get_closest_freq_cunder_constraint(int argc, char **argv)
{
    if (argc != 5) {
        printf("Usage: %s <clk_name> <target_freq_hz> <constraint_clk_name> <constraint_clk_hz>\n", argv[0]);
        return 1;
    }

    const gclk_t *clk = gclk_get_clk_by_name(argv[1]);

    uint32_t target_freq = atoi(argv[2]);

    const gclk_t *constraint_clk = gclk_get_clk_by_name(argv[3]);

    uint32_t constraint_hz = atoi(argv[4]);

    if (clk && constraint_clk) {

        uint32_t max_involved_clks = gclk_get_clk_subtree_max_depth(clk, 0);
        printf("max topology for driving %s employs %lu clock nodes\n", gclk_get_name(clk), max_involved_clks);

        clk_topology_entry_t best_topology[max_involved_clks];
        memset(best_topology, 0, sizeof(clk_topology_entry_t) * max_involved_clks);
        best_topology[0].clk = clk;
        best_topology[0].clk_freq = GCLK_INVALID_FREQ;

        gclk_constrained_cmp_ctx_t ctx = { .constraint_clk = constraint_clk, .constraint_clk_freq = constraint_hz,
                                            .target_freq = target_freq };
        int topo_idx = GCLK_UNDEFINED_TOPOLOGY;
        size_t valid_cnt = 0;
        int force_nth = -1;
        uint32_t match_freq = gclk_manager_brute_force_freq_conf(clk, best_topology, &max_involved_clks, &topo_idx,
                                                                 gclk_cmp_topology_for_closest_constrained_leaf_freq, (void*)&ctx,
                                                                 &valid_cnt, force_nth, NULL);

        printf("Matched frequency for %s: %lu Hz\n", gclk_get_name(clk), match_freq);
        printf("Obtainable by using the following topology:\n");
        gclk_manager_print_topology_conf(best_topology, max_involved_clks, false, true);
    } else {
        printf("could not find clock named %s\n", argv[1]);
    }

    return 0;
}

/* TODO: remove me after testing...*/
extern void _gclk_manager_print_conf_cache_state(void);

int _sc_actopo(int argc, char **argv)
{
    const gclk_t *clk;

    if (argc == 1) {
        clk = gclk_manager_get_core_clock_handle();
    } else if  (argc == 2) {
        clk = gclk_get_clk_by_name(argv[1]);
    } else {
        printf("Usage: %s [clk_name]\n", argv[0]);
        return 1;
    }

    if (clk) {
        unsigned int topolen = gclk_get_current_topology_len(clk);
        clk_topology_entry_t topo[topolen];
        gclk_get_current_topology_config(clk, topo, topolen);
        int tidx = gclk_topology2id(topo, topolen);
        printf("current topology ID of %s: %d\n", gclk_get_name(clk), tidx);

        for (unsigned i = 0; i < topolen; i++) {
            gclk_freq_str_t cfs;
            char *cfstr = gclk_freq2str(gclk_get_current_freq(topo[i].clk), &cfs);
            printf("[%s@%s]%s", gclk_get_name(topo[i].clk), cfstr, i < (topolen-1) ? "-->" : "\n");
        }
    } else {
        printf("could not find clock named %s\n", argv[1]);
    }

    _gclk_manager_print_conf_cache_state();

    return 0;
}

static bool _factors_are_continuous(const gclk_t *clk) {
    bool continuous = true;
    uint32_t prev = gclk_idx2factor(clk, 0);

    for (unsigned i = 1; i < gclk_factor_cnt(clk); i++) {
        uint32_t f = gclk_idx2factor(clk, i);
        continuous = continuous && ((f-1) == prev);
        prev = f;
        if (!continuous) {
            return false;
        }
    }
    return continuous;
}

int _sc_valueacc(int argc, char **argv) {
    (void)argc;
    (void)argv;
    for (unsigned i = 0; i < gclk_get_cnt(); i++) {
        const gclk_t *clk = gclk_get(i);
        printf("[%s]\n", gclk_get_name(clk));

        if (gclk_is_scalable(clk)) {
            bool continuous = _factors_are_continuous(clk);
            for (unsigned n = 0; n < gclk_factor_cnt(clk); n++) {
                uint32_t f = gclk_idx2factor(clk, n);
                printf("factor[%u]: %lu\n", n, f);
                if (continuous && (gclk_factor_cnt(clk) > 3)) {
                    /* skip ahead to the end of the factor list */
                    n = gclk_factor_cnt(clk) - 2;
                    /* falsify condition for after last print */
                    continuous = false;
                    printf("(...)\n");
                }
            }
        }
        for (unsigned i = 0; i < gclk_parent_cnt(clk); i++) {
            const gclk_t *c = gclk_idx2parent(clk, i);
            printf("parent[%u]: %s\n", i, gclk_get_name(c));
        }
        printf("#######################\n");
    }
    return 0;
}

int _sc_set_parent(int argc, char **argv)
{
    if (argc != 3) {
        printf("Usage: %s <child_clk_name> <parent_clk_name>\n", argv[0]);
        return -1;
    }

    const gclk_t *child_clk = gclk_get_clk_by_name(argv[1]);
    const gclk_t *parent_clk = gclk_get_clk_by_name(argv[2]);

    /* child clock must be a valid instance != NULL (the parent may indeed be NULL) */
    if (!child_clk) {
        if (strcmp(argv[1], "NULL") == 0) {
            printf("%s is not a valid child clock\n", argv[1]);
        } else {
            printf("could not find clock named %s\n", argv[1]);
        }
        return -1;
    }

    return _set_parent(child_clk, parent_clk);
}

static bool _simple_set_freq_instrumented(const gclk_t *clk, uint32_t freq, bool print_pulse_desc) {

    bool res = _gclk_manager_set_freq_instrumented(clk, freq);

    if (print_pulse_desc) {
        printf("pulse_description: bkup_curconf pre_hook set_freq re_init_affected_peripherals post_hook\n");
    }
    return res;
}

/* copy of normal set_freq but with instrumented gpio things */
int _sc_set_freq_eval(int argc, char **argv)
{
    if (argc != 3) {
        printf("Usage: %s <clk_name> <freq>\n", argv[0]);
        return 1;
    }

    const gclk_t *clk = gclk_get_clk_by_name(argv[1]);

    uint32_t freq = atoi(argv[2]);

    if (clk) {
        bool success = _simple_set_freq_instrumented(clk, freq, true);

        if (!success) {
            uint32_t chk_new = gclk_get_current_freq(clk);

            LOG_ERROR("WARNING: set_freq(%s,%lu) update didn't work correctly! (new freq reported: %lu %s)\n",
                   gclk_get_name(clk), freq, gclk_print_scale_freq(chk_new),
                   gclk_freq_scale_unit(chk_new));

            if (LOG_LEVEL >= LOG_ERROR) {
                _print_topology_flag_err_msgs(clk);
            }

            return false;
        }
    } else {
        printf("could not find clock named %s\n", argv[1]);
    }

    return 0;

}

int _sc_scale_freq(int argc, char **argv)
{
    if (argc != 2) {
        printf("Usage: %s <freq>\n", argv[0]);
        return 1;
    }

    const gclk_t *clk = gclk_manager_get_core_clock_handle();

    uint32_t freq = atoi(argv[1]);

    if (clk) {
        bool success = gclk_manager_scale_core_freq(freq);

        if (!success) {
            uint32_t chk_new = gclk_get_current_freq(clk);

            LOG_ERROR("WARNING: set_freq(%s,%lu) update didn't work correctly! (new freq reported: %lu %s)\n",
                   gclk_get_name(clk), freq, gclk_print_scale_freq(chk_new),
                   gclk_freq_scale_unit(chk_new));
            if (LOG_LEVEL >= LOG_ERROR) {
                _print_topology_flag_err_msgs(clk);
            }

            return false;
        }
    } else {
        printf("ERROR no core clock handle available!\n");
    }

    return 0;
}

static void _print_clock_description_line(const gclk_t *clk, bool verbose, int max_clk_name)
{
    uint32_t current_freq = gclk_get_current_freq(clk);
    if (verbose) {
        uint32_t min_freq = gclk_get_min_freq_of_current_topology(clk);
        uint32_t max_freq = gclk_get_max_freq_of_current_topology(clk);

        /* print name with enabled and possible enabled state */
        printf("[%s]%*s %-9s @", gclk_get_name(clk), max_clk_name - strlen(gclk_get_name(clk)),  "",
                                 gclk_is_gateable(clk) ? (gclk_is_enabled(clk) ? ">ON< OFF " : " ON >OFF<") : " ON ");

        /* print current and possible frequencies */
        printf("%lu %s {%lu %s .. %lu %s} ", gclk_print_scale_freq(current_freq), gclk_freq_scale_unit(current_freq),
                                                 gclk_print_scale_freq(min_freq), gclk_freq_scale_unit(min_freq),
                                                 gclk_print_scale_freq(max_freq), gclk_freq_scale_unit(max_freq));

        /* if scaleable, print current ad possible scaling factors */
        if (gclk_is_scalable(clk)) {
            printf(" %c%u {", gclk_is_multiplier(clk) ? 'x' : '/', gclk_get_current_factor(clk));
            bool continuous = _factors_are_continuous(clk);
            for (unsigned n = 0; n < gclk_factor_cnt(clk); n++) {
                uint32_t f = gclk_idx2factor(clk, n);
                printf("%s%lu", n == 0 ? "" : ", ", f);
                if (continuous && (gclk_factor_cnt(clk) > 3)) {
                    /* skip ahead to the end of the factor list */
                    n = gclk_factor_cnt(clk) - 2;
                    /* falsify condition for after last print */
                    continuous = false;
                    printf(", ...");
                }
            }
            printf("}");
        }

        printf("\n");
    } else {
        printf("[%s]%*s @%lu %s %s\n", gclk_get_name(clk),
               max_clk_name - strlen(gclk_get_name(clk)),  " ", /* space padding */
               gclk_print_scale_freq(current_freq), gclk_freq_scale_unit(current_freq),
               gclk_is_enabled(clk) ? "ON" : "OFF");
    }
}

static void _print_hline(void) {
    unsigned cnt = DESCRIPTION_LINE_FRAME_WIDTH;
    printf("+");
    while(cnt--) {
        printf("-");
    }
    printf("+\n");
}

static void _print_clock_description(const gclk_t *clk, bool verbose)
{
    const gclk_t *current_parent = gclk_get_current_parent(clk);

    _print_hline();

    /* get max clock name length per group for prettier print */
    int max_len = 0;
    for (unsigned i = 0; i < gclk_parent_cnt(clk); i++) {
        const gclk_t *parent = gclk_get_parent(clk, i);
        int len = strlen(gclk_get_name(parent));
        if (len > max_len) {
            max_len = len;
        }
    }

    for (unsigned i = 0; i < gclk_parent_cnt(clk); i++) {
        const gclk_t *parent = gclk_get_parent(clk, i);
        printf("  |----%c", parent == current_parent ? '>' : ' ');
        _print_clock_description_line(parent, verbose, max_len);
        parent = gclk_get_parent(clk, i);
    }
    printf("  |\n  |\n  ");
    _print_clock_description_line(clk, verbose, strlen(gclk_get_name(clk)));
    _print_hline();
}

static bool _append_if_new(const gclk_t **clks, const gclk_t *clk, uint32_t *fill_cnt) {
    for (uint32_t i = 0; i < *fill_cnt; i++) {
        if (clk == clks[i]) {
            return false;
        }
    }

    clks[(*fill_cnt)++] = clk;
    return true;
}

static void _print_roots(const gclk_t *clk)
{
    uint32_t max_involved_clks = gclk_get_clk_subtree_max_depth(clk, 0);
    printf("All possible roots to drive [%s]\n", gclk_get_name(clk));
    uint32_t root_cnt = gclk_get_topology_config_cnt(clk);
    const gclk_t * roots[root_cnt];
    memset(roots, 0, sizeof(gclk_t*) * root_cnt);
    root_cnt = 0;

    clk_topology_entry_t topology[max_involved_clks];
    memset(topology, 0, sizeof(clk_topology_entry_t) * max_involved_clks);
    topology[0].clk = clk;
    for (unsigned ti = 0; ti < gclk_get_topology_config_cnt(clk); ti++) {
        size_t size = gclk_get_nth_topology(topology, max_involved_clks, ti);
        const gclk_t *root = topology[size-1].clk;
        if (_append_if_new(roots, root, &root_cnt)) {
            printf("[%s]\n", gclk_get_name(root));
        }
    }
}

static void _print_children(const gclk_t *parent, bool recursive) {
    uint32_t child_idx = 0;
    const gclk_t *child = gclk_get_child(parent, child_idx);
    while (child != NULL) {
        printf("[%s] (%s)\n", gclk_get_name(child), gclk_is_enabled(child) ? "ON" : "OFF" );
        child_idx++;
        if (recursive) {
            _print_children(child, true);
        }
        child = gclk_get_child(parent, child_idx);
    }
}

void _no_such_clock_err(char *name)
{
    printf("No clock instance named %s\n", name);
}

int _sc_clock(int argc, char **argv)
{
    if (argc < 2) {
        goto clkusage;
    }

    const gclk_t *clk = NULL;
    const gclk_t *clk_param = NULL;

    bool list, describe, roots, children, affected, conf, get, set;
    list = describe = roots = children = affected = conf = get = set = false;

    if (strcmp(argv[1], "list") == 0 && (argc == 2 || argc == 3)) {
        list = true;
    } else if (strcmp(argv[1], "describe") == 0 && (argc == 2 || argc == 3)) {
        describe = true;
    } else if (strcmp(argv[1], "roots") == 0 && argc == 3) {
        roots = true;
    } else if (strcmp(argv[1], "children") == 0 && (argc == 3 || argc == 4)) {
        children = true;
    } else if (strcmp(argv[1], "affected") == 0 && argc == 4) {
        affected = true;
    } else if (strcmp(argv[1], "conf") == 0 && argc >= 4 && argc <= 7) {
        conf = true;
    } else {
        goto clkusage;
    }

    if ((describe && argc == 3) || roots || children || affected || conf) {
        clk = gclk_get_clk_by_name(argv[2]);
        if (!clk) {
            _no_such_clock_err(argv[2]);
            goto clkusage;
        }
        if (affected) {
            clk_param = gclk_get_clk_by_name(argv[3]);
            if (!clk_param) {
                _no_such_clock_err(argv[3]);
                goto clkusage;
            }
        }
    }

    if (list) {
        bool enabled = argc < 3 ? false : strcmp(argv[2], "enabled") == 0;
        bool disabled = argc < 3 ? false : strcmp(argv[2], "disabled") == 0;
        for (unsigned i = 0; i < gclk_get_cnt(); i++) {
            const gclk_t *clk = gclk_get(i);

            if (!(enabled || disabled) ||
                enabled == gclk_is_enabled(clk) ||
                disabled == !gclk_is_enabled(clk)) {
                printf("[%s]\n", gclk_get_name(clk));
            }
        }
    } else if (describe) {
        /* clock name was specified */
        if (argc == 3) {
            _print_clock_description(clk, true);
        } else {
            /* no clock name was specified -> describe all clocks */
            for (unsigned i = 0; i < gclk_get_cnt(); i++) {
                _print_clock_description(gclk_get(i), true);
            }
        }
    } else if (roots) {
        _print_roots(clk);
    } else if (children) {
        bool recursive = argc < 4 ? false : strcmp(argv[3], "r") == 0;
        _print_children(clk, recursive);
    } else if (affected) {
        printf("[%s] is %saffected by a change of [%s]!\n", gclk_get_name(clk),
               gclk_affected_by_change(clk, clk_param) ? "" : "Not ",
               gclk_get_name(clk_param));
    } else if (conf) {
        if (strcmp(argv[3], "enable") == 0) {
            gclk_enable(clk);
        } else if (strcmp(argv[3], "disable") == 0) {
            gclk_disable(clk);
        } else if (strcmp(argv[3], "get") == 0) {
            get = true;
        } else if (strcmp(argv[3], "set") == 0) {
            set = true;
        } else {
            goto clkusage;
        }

        if (get || set) {
            if ((get && argc < 5) || (set && argc < 6)) {
                goto clkusage;
            }
            bool auto_reconf = (set && argc == 7 && (strcmp(argv[6], "autoreconf") == 0)) ? true : false;

            if (strcmp(argv[4], "factor") == 0) {
                if (get) {
                    unsigned factor = gclk_get_current_factor(clk);
                    printf("current scale factor of [%s] is %u\n", gclk_get_name(clk), factor);
                }
                if (set) {
                    unsigned factor = atoi(argv[5]);
                    if (auto_reconf) {
                        gclk_manager_set_factor(clk, factor);
                    } else {
                        gclk_set_factor(clk, factor);
                    }
                }
            } else if (strcmp(argv[4], "freq") == 0) {
                if (get) {
                    unsigned freq = gclk_get_current_freq(clk);
                    printf("current scale frequency of [%s] is %u Hz\n", gclk_get_name(clk), freq);
                }
                if (set) {
                    unsigned freq = atoi(argv[5]);
                    if (auto_reconf) {
                        gclk_manager_set_freq(clk, freq);
                    } else {
                        gclk_set_freq(clk, freq);
                    }
                }
            } else if (strcmp(argv[4], "parent") == 0) {
                if (get) {
                    const gclk_t *parent = gclk_get_current_parent(clk);
                    printf("current parent of [%s] is [%s]\n", gclk_get_name(clk), gclk_get_name(parent));
                }
                if (set) {
                    const gclk_t *parent = gclk_get_clk_by_name(argv[5]);
                    if (!parent) {
                        printf("no clock instance named %s\n", argv[5]);
                        return -1;
                    }
                    unsigned pidx = gclk_parent2idx(clk, parent);
                    printf("set parent of [%s] to [%s] (idx %u)\n", gclk_get_name(clk), gclk_get_name(parent), pidx);
                    if (auto_reconf) {
                        /* move to manager as utility function */
                        uint32_t f_old = gclk_get_current_freq(clk);
                        uint32_t f_new = gclk_get_current_freq(parent);
                        gclk_manager_notify_clk_change(clk, f_old, f_new, false);
                        gclk_set_parent(clk, pidx);
                        gclk_manager_notify_clk_change(clk, f_old, f_new, true);
                    } else {
                        gclk_set_parent(clk, pidx);
                    }
                }
            }
        }
    }

    return 0;
clkusage:
    printf("Usage: %s list [enabled|disabled] |\n"
           "             describe [<clk_name>] |\n"
           "             roots <clk_name> |\n"
           "             children <clk_name> [r] |\n"
           "             affected <altered_clk_name> <affected_clk_name> |\n"
           "             conf <clk_name> { enable | disable | get {factor | freq | parent} | set {factor | freq | parent} <val_or_clk_name> [autoreconf] }\n", argv[0]);
    return -1;
}

int _sc_disable_unused(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    gclk_manager_disable_unused();

    return 0;
}

int _sc_list_topologies(int argc, char **argv){

    if (argc != 2) {
        printf("Usage: %s <clk_name>\n", argv[0]);
        return 1;
    }

    const gclk_t *clk = gclk_get_clk_by_name(argv[1]);

    if (clk) {
        uint32_t max_involved_clks = gclk_get_clk_subtree_max_depth(clk, 0);
        printf("All possible topologies to drive [%s] (max %lu clock nodes)\n", gclk_get_name(clk), max_involved_clks);

        clk_topology_entry_t topology[max_involved_clks];
        memset(topology, 0, sizeof(clk_topology_entry_t) * max_involved_clks);
        topology[0].clk = clk;
        printf("ID | Topology\n---|-------------------\n");
        for (unsigned ti = 0; ti < gclk_get_topology_config_cnt(clk); ti++) {
            size_t size = gclk_get_nth_topology(topology, max_involved_clks, ti);
            printf("%2u | ", ti);
            gclk_print_topology(topology, size);
        }

    } else {
        printf("could not find clock named %s\n", argv[1]);
        return -1;
    }

    return 0;
}

int _sc_get_max_freq(int argc, char **argv)
{
    if (!(argc == 2 || argc == 3)) {
        printf("Usage: %s <clk_name> [topology_id]\n", argv[0]);
        return 1;
    }

    const gclk_t *clk = gclk_get_clk_by_name(argv[1]);

    int tidx = GCLK_UNDEFINED_TOPOLOGY;

    if (argc == 3) {
        tidx = atoi(argv[2]);
    }

    if (clk) {
        uint64_t t_start = ztimer_now(ZTIMER_USEC);
        uint32_t max_involved_clks = gclk_get_clk_subtree_max_depth(clk, 0);

        clk_topology_entry_t best_topology[max_involved_clks];
        memset(best_topology, 0, sizeof(clk_topology_entry_t) * max_involved_clks);
        best_topology[0].clk = clk;
        best_topology[0].clk_freq = GCLK_INVALID_FREQ;
        size_t valid_cnt = 0;
        int force_nth = -1;

        unsigned long leaf_freq = gclk_manager_brute_force_freq_conf(clk, best_topology, &max_involved_clks, &tidx,
                                                                     gclk_cmp_topology_for_max_leaf_freq, NULL, &valid_cnt, force_nth, NULL);


        printf("Max frequency for %s: %lu Hz\n", gclk_get_name(clk), leaf_freq);
        printf("Obtainable by using the following topology:\n");
        gclk_manager_print_topology_conf(best_topology, max_involved_clks, true, false);
        printf("took %lu ms\n", (uint32_t)((ztimer_now(ZTIMER_USEC) - t_start)/1000));
    } else {
        printf("could not find clock named %s\n", argv[1]);
    }

    return 0;
}

int _sc_get_min_freq(int argc, char **argv)
{
    if (!(argc == 2 || argc == 3)) {
        printf("Usage: %s <clk_name> [topology_id]\n", argv[0]);
        return 1;
    }

    const gclk_t *clk = gclk_get_clk_by_name(argv[1]);

    int tidx = GCLK_UNDEFINED_TOPOLOGY;

    if (argc == 3) {
        tidx = atoi(argv[2]);
    }

    if (clk) {
        uint64_t t_start = ztimer_now(ZTIMER_USEC);
        uint32_t max_involved_clks = gclk_get_clk_subtree_max_depth(clk, 0);

        clk_topology_entry_t best_topology[max_involved_clks];
        memset(best_topology, 0, sizeof(clk_topology_entry_t) * max_involved_clks);
        best_topology[0].clk = clk;
        best_topology[0].clk_freq = GCLK_INVALID_FREQ; /* set this to max possible value before searching the minimum */
        size_t valid_cnt = 0;
        int force_nth = -1;

        unsigned long leaf_freq = gclk_manager_brute_force_freq_conf(clk, best_topology, &max_involved_clks, &tidx,
                                                                     gclk_cmp_topology_for_min_nz_leaf_freq, NULL, &valid_cnt, force_nth, NULL);

        printf("Min frequency for %s: %lu Hz\n", gclk_get_name(clk), leaf_freq);
        printf("Obtainable by using the following topology:\n");
        gclk_manager_print_topology_conf(best_topology, max_involved_clks, true, false);
        printf("took %lu ms\n", (uint32_t)((ztimer_now(ZTIMER_USEC) - t_start)/1000));
    } else {
        printf("could not find clock named %s\n", argv[1]);
    }

    return 0;
}

/* commands to query/set parameters of individual clock instances */
/* commands to query/explore topology-related information and possible configurations */
SHELL_COMMAND(topologies, "list all possible topologies to drive a clock", _sc_list_topologies);
SHELL_COMMAND(actopo, "list the currently active topology of a clock", _sc_actopo);
SHELL_COMMAND(min, "get min configurable frequency !=0 of a clock", _sc_get_min_freq);
SHELL_COMMAND(max, "get max configurable frequency of a clock", _sc_get_max_freq);
SHELL_COMMAND(closestc, "get closest configurable frequency of a clock under a constraint", _sc_get_closest_freq_cunder_constraint);
SHELL_COMMAND(derive_seq, "automagically derive a sequence that sets up a target topology configuration", _sc_derive_sequence);
SHELL_COMMAND(match, "match output freq by iterative calculation", _sc_match);
/* commands that automatically perform changes to the frequency/topology config of the clock tree */
SHELL_COMMAND(disable_unused, "disable all unused clocks that can safely be disabled", _sc_disable_unused);
//SHELL_COMMAND(transition, "transition a clock to a new frequency by switching to another clock temporarily", _transition);
SHELL_COMMAND(setup_closest, "setup closest possible freq (full topology)", _sc_setup_closest_freq);
SHELL_COMMAND(switch_topo, "switch to new topology config closest to given freq", _sc_topo_switch_to_closest_freq);
SHELL_COMMAND(scale_freq, "set the frequency using an explicit complex transition if needed", _sc_scale_freq);
/* commands to get/set (runtime) configuration parameters that affect e.g. online-self adaptation */
SHELL_COMMAND(dvfs, "ATTENTION!! this configures DVFS settings (including core voltage)", _sc_dvfs);
/* commands for evaluation measurements and microbenchmarks */
SHELL_COMMAND(set_freq_eval, "set the frequency of a clock (with GPIO intrumentation)", _sc_set_freq_eval);
SHELL_COMMAND(clock, "query and modify the configuration of individual clocks", _sc_clock);
SHELL_COMMAND(clockman, "interact with the clock manager", _sc_clockman);
