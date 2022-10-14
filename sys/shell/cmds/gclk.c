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
#include "xtimer.h"

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

int _sc_dvfs(int argc, char **argv)
{
    bool dvs, dfs, wsa, on, off, ff_pol, lv_pol, info, set, get, freqs;
    dvs = dfs = wsa = on = off = ff_pol = lv_pol = info = set = get = freqs = false;

    if (argc >= 3) {
        if (strcmp(argv[1], "dvs") == 0) {
            dvs = true;
        } else if(strcmp(argv[1], "dfs") == 0) {
            dfs = true;
        } else if(strcmp(argv[1], "wsa") == 0) {
            wsa = true;
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
        }
    }
    printf("Usage: %s {wsa {on|off|get|set <ws>}} | {dvs {on|off|info|set <voltage_idx>|ffpol|lvpol}} | {dfs {on|off|info|set <scalesetting_id>|freqs {get|set {default| <freq>+}}}}\n", argv[0]);
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
            printf("Deriving topo switch sequence from %d@%lu Hz to %d@%lu Hz ...\n", cur_topo_idx, cur_topology[0].clk_freq,
                                                                                      tidx, target_topology[0].clk_freq);
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

static const char* _USAGE_CLOSEST = "<clk_name> <target_freq_hz> [<topology_id> [<cmp_fun> {listall | <conf_idx>]]]]";
static int _get_closest_freq(int argc, char **argv)
{
    if (! ((argc >= 3) && (argc <= 6))) {
        goto usage;
    }

    const gclk_t *clk = gclk_get_clk_by_name(argv[1]);
    uint32_t target_freq = atoi(argv[2]);
    int tidx = GCLK_UNDEFINED_TOPOLOGY;
    bool listall = false;
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

        /* get count of possible configs first */
        gclk_manager_brute_force_freq_conf(clk, topology, &max_involved_clks, &tidx,
                cmp_func, (void*)&target_freq_copy, &valid_cnt, force_nth, &valid_cb_save_best_idx_conf);

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
            LOG_DEBUG("set_freq of [%s] to %lu\n", gclk_get_name(best_topology[i].clk), best_topology[i].clk_freq);
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

int _sc_clockman(int argc, char **argv)
{
    int res = PRINT_USAGE;
    if (strcmp(argv[1], _CLOCKMAN_SUBCMD_CLOSEST) == 0) {
        res = _get_closest_freq(argc - 1, &argv[1]);
    }

    if (res == PRINT_USAGE) {
        printf("usage: %s %s %s\n", argv[0], _CLOCKMAN_SUBCMD_CLOSEST, _USAGE_CLOSEST);
    }
    return -1;
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
            printf("[%s@%lu]%s", gclk_get_name(topo[i].clk), gclk_get_current_freq(topo[i].clk), i < (topolen-1) ? "-->" : "\n");
        }
    } else {
        printf("could not find clock named %s\n", argv[1]);
    }
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
        //unsigned long max_freq = _get_max_freq(clk, 0);
        uint64_t t_start = xtimer_now_usec64();
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
        printf("took %lu ms\n", (uint32_t)((xtimer_now_usec64() - t_start)/1000));
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
        //unsigned long max_freq = _get_max_freq(clk, 0);
        uint64_t t_start = xtimer_now_usec64();
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
        printf("took %lu ms\n", (uint32_t)((xtimer_now_usec64() - t_start)/1000));
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
