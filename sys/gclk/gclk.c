#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include "gclk.h"

#if defined (CPU_FAM_STM32L4)
extern const gclk_t *gclks[GCLK_NUM_OF_CLOCKS];
#elif defined (CPU_MODEL_EFM32PG12B500F1024GL125)
/* @todo: move to generic header once it is aligned with the above */
extern const gclk_t *gclks[GCLK_NUM_OF_CLOCKS];
#endif


#define LOG_LEVEL LOG_NONE
#include "log.h"

const gclk_scale_ops_t *gclk_get_scale_ops(const gclk_t *clk) {
    return clk->flags.scalable ? (gclk_scale_ops_t*)&clk->separated_ops[0] : NULL;
}

const gclk_mux_ops_t *gclk_get_mux_ops(const gclk_t *clk) {
    return clk->flags.muxable ? (gclk_mux_ops_t*)&clk->separated_ops[clk->flags.scalable] : NULL;
}

const gclk_gate_ops_t *gclk_get_gate_ops(const gclk_t *clk) {
    return clk->flags.gateable ?
           (gclk_gate_ops_t*)&clk->separated_ops[clk->flags.scalable +
                                                 clk->flags.muxable] :
           NULL;
}

const gclk_trim_ops_t *gclk_get_trim_ops(const gclk_t *clk) {
    return clk->flags.trimmable ?
           (gclk_trim_ops_t*)&clk->separated_ops[clk->flags.scalable +
                                                 clk->flags.muxable +
                                                 clk->flags.gateable] :
           NULL;
}

uint32_t gclk_print_scale_freq(uint32_t val) {
    if (val % 1000000 == 0) {
        return val / 1000000;
    } else if (val % 1000 == 0) {
        return val / 1000;
    }

    return val;
}

char *gclk_freq_scale_unit(uint32_t val) {

    if (val) {
        if (val % 1000000 == 0) {
            return "MHz";
        } else if (val % 1000 == 0) {
            return "kHz";
        }
    }

    return "Hz";
}

uint32_t gclk_get_cnt(void) {
    return GCLK_NUM_OF_CLOCKS;
}

const gclk_t *gclk_get(uint32_t idx) {
    return gclks[idx];
}

unsigned int gclk_get_index(const gclk_t *gclk) {

    for (unsigned i = 0; i < GCLK_NUM_OF_CLOCKS; i++) {
        if (gclk == gclks[i]) {
            return i;
        }
    }

    return 0xFFFFFFFF;
}

//int gclk_struct_init(gclk_t *gclk, gclk_t *ll_handle, const gclk_t *parent)
//{
//    ll_handle->hl_handle = gclk;
//    gclk = ll_handle;
//    gclk->parent = parent;
//    //gclk->parent /* do nothing for now -> should actually do something with the list of parents, right? *
//    /* TODO: call ll bootstrap / init code */
//    return 0;
//}
//int gclk_struct_init(gclk_t *gclk, const gclk_t *parent)
//{
//    gclk->parent = parent;
//    return 0;
//}

int gclk_module_init(void)
{
    /* any bootstrapping code for the generic clock module (high level managment part) should go here */
    return 0;
}

const char *gclk_get_name(const gclk_t *clk)
{
    if (clk != NULL) {
        return clk->name;
    }
    return "NULL";
}

const gclk_t* gclk_get_clk_by_name(const char *name) {
    for (unsigned i = 0; i < GCLK_NUM_OF_CLOCKS ; i++) {
        const char *othername = gclk_get_name(gclks[i]);
        if (strcmp(name, othername) == 0) {
            return gclks[i];
        }
    }

    return NULL;
}

unsigned int gclk_get_clk_subtree_max_depth(const gclk_t *clk, unsigned depth){
    if (!clk) {
        return depth;
    } else if (clk->flags.is_source) {
        return depth + 1;
    }

    if (clk->flags.muxable) {
        unsigned max_depth = 0;
        for (unsigned i = 0; i < gclk_parent_cnt(clk); i++) {
            const gclk_t *nth_par = gclk_idx2parent(clk, i);
            unsigned chk_depth = gclk_get_clk_subtree_max_depth(nth_par, depth + 1);
            if (chk_depth > max_depth) {
                max_depth = chk_depth;
            }
        }
        return max_depth;
    }

    /* if it is not muxable it means it must have a fixed parent */
    return gclk_get_clk_subtree_max_depth(clk->fixed_parent, depth + 1);
}

unsigned int gclk_get_current_topology_len(const gclk_t *clk) {
    unsigned int len = 1;
    while (!gclk_is_source(clk)) {
        len++;
        clk = gclk_get_current_parent(clk);
    }
    return len;
}

uint32_t gclk_get_max_topology_depth(void) {
    uint32_t max_involved_clks = 0;
    for (unsigned i = 0; i < GCLK_NUM_OF_CLOCKS ; i++) {
        uint32_t tmp = gclk_get_clk_subtree_max_depth(gclks[i], 0) + 1;
        if (tmp > max_involved_clks) {
            max_involved_clks = tmp;
        }
    }

    return max_involved_clks;
}

void gclk_print_topology(clk_topology_entry_t *topology_list, uint32_t depth) {
    for (unsigned i = 0; i < depth; i++) {
        printf("[%s]", gclk_get_name(topology_list[i].clk));
        if (i < (depth -1)) {
            printf("-->");
        }
    }

    printf("\n");
}

void gclk_enable(const gclk_t *gclk)
{
    if (!gclk) {
        return;
    }

    if (gclk_is_gateable(gclk)) {
        gclk_get_gate_ops(gclk)->enable(gclk, true);
    } else if (!gclk_is_source(gclk)) {
        /* for a non gateable clock that is not itself a source, forward the enable request to its parent */
        gclk_enable(gclk_get_current_parent(gclk));
    }
}

void gclk_disable(const gclk_t *gclk) {
    if (!gclk) {
        return;
    }

    if (gclk_is_gateable(gclk)) {
        gclk_get_gate_ops(gclk)->enable(gclk, false);
    } else {
        /* for a non gateable clock forward to its parent */
        gclk_disable(gclk_get_current_parent(gclk));
    }
}

bool gclk_is_enabled(const gclk_t *gclk)
{
    if (!gclk) {
        return false;
    }

    if (gclk_is_gateable(gclk)) {
        return gclk_get_gate_ops(gclk)->is_enabled(gclk);
    } else if (gclk_is_source(gclk)) {
        /* a non-gateable source is always enabled */
        return true;
    }

    /* for a non gateable clock forward to its parent */
    return gclk_is_enabled(gclk_get_current_parent(gclk));
}

unsigned int gclk_get_current_factor(const gclk_t *clk) {
   if (gclk_is_scalable(clk)) {
       return gclk_get_scale_ops(clk)->get_factor(clk);
   }
   return 1;
}

int gclk_compare_fraction(gclk_fraction_t *a, gclk_fraction_t *b) {
    uint32_t m1 = a->n * b->d;
    uint32_t m2 = b->n * a->d;

    return m1 - m2;
}

unsigned int gclk_get_uptree_dependent_factor(const gclk_t *gclk, const clk_topology_entry_t *tree_confs, unsigned int conf_cnt) {
    for (unsigned i = 0; i < conf_cnt; i++) {
        if (tree_confs[i].clk == gclk->factor_mapping.cross_ref->ref_clk) {
            return gclk->cross_ref_factor_map_op(gclk, &tree_confs[i]);
        }
    }
    printf("%s: determination of uptree dependent factor failed! -> relevant part of the tree not handed over?\n", __FUNCTION__);
    return 0;
}

uint32_t gclk_get_current_equivalent_uptree_factors(const gclk_t *clk, uint32_t *m, uint32_t *d)
{
    if (gclk_is_divider(clk)) {
        *d *= gclk_get_current_factor(clk);
    } else if (gclk_is_multiplier(clk)) {
        *m *= gclk_get_current_factor(clk);
    }

    if (gclk_is_source(clk)) {
        return gclk_get_input_freq(clk);
    } else {
        return gclk_get_current_equivalent_uptree_factors(gclk_get_current_parent(clk), m, d);
    }
}

unsigned long gclk_get_current_freq(const gclk_t *clk)
{
    if (clk == NULL) {
        return 0;
    }

    uint32_t mul = 1;
    uint32_t div = 1;

    uint32_t fi = gclk_get_current_equivalent_uptree_factors(clk, &mul, &div);

    return fi * mul / div;
}

const gclk_t *gclk_get_parent(const gclk_t *clk, unsigned int idx)
{
    if (clk == NULL || gclk_is_source(clk)) {
        return NULL;
    }

    if (clk->flags.muxable) {
        return gclk_idx2parent(clk, idx);
    } else {
        return clk->fixed_parent;
    }
}

const gclk_t *gclk_get_current_parent(const gclk_t *clk) {
    if (clk->flags.muxable) {
        return gclk_get_mux_ops(clk)->get_parent(clk);
    } else if (clk->flags.is_source) {
        return NULL;
    }
    return clk->fixed_parent;
}

int gclk_set_factor(const gclk_t *gclk, uint32_t factor) {
    if (gclk == NULL) {
        LOG_DEBUG("can not set factor of NULL\n");
        return -1;
    } else if (gclk->separated_ops) {
        if (gclk_is_scalable(gclk)) {
            gclk_get_scale_ops(gclk)->set_factor(gclk, factor);
            return 0;
        }
    }

    LOG_ERROR("called %s for a clock that is not scalable!\n", __FUNCTION__);
    return -1;
}

int gclk_set_parent(const gclk_t *clk, unsigned int idx) {
    if (clk == NULL) {
        LOG_ERROR("can not set parent of NULL\n");
        return -1;
    }

    /* if this clock implements the iseparated ops low-level interface pattern
     * use the provided functionality to access parent info */
    if (clk->separated_ops) {
        if (clk->flags.muxable) {
            gclk_get_mux_ops(clk)->set_parent(clk, idx);
            return 0;
        }
        /* this should never happen as calling this function for clocks that are not
         * muxable doesn't make sense */
        return -1;
    }

    printf("ERROR! The clock config implementation was not migrated to separated_ops pattern!\n");
    assert(false);
    return -1;
}

/* todo: could be replaced by calculating offset from gclks */
int _get_gclks_idx(const gclk_t *clk) {
    for (unsigned i = 0; i < GCLK_NUM_OF_CLOCKS; i++) {
        if (clk == gclks[i]) {
            return i;
        }
    }

    /* should never ever happen */
    printf("_get_gclks_idx ################### WARNING THIS should never happen!\n");
    return -1;
}

const gclk_t *gclk_get_child(const gclk_t *gclk, uint32_t child_idx) {
    uint32_t idx = 0;
    /* check all clocks if they are children of this clock */
    for (unsigned x = 0; x < GCLK_NUM_OF_CLOCKS; x++) {
        const gclk_t *chk_parent = gclk_get_current_parent(gclks[x]);
        if ((!(chk_parent == gclks[x])) &&
             (chk_parent == gclk)) {
            if (idx == child_idx) {
                return gclks[x];
            }
            idx++;
        }
    }

    /* NULL indocates that currently no clock uses gclk as parent */
    return NULL;
}

bool gclk_is_leaf(const gclk_t *clk) {
    for (unsigned i = 0; i < gclk_get_cnt(); i++) {

        const gclk_t *child = gclk_get(i);
        const gclk_t *parent;

        /* do no check for the clock itself */
        if (child != clk) {
            for (unsigned p = 0; p < gclk_parent_cnt(child); p++) {
                parent = gclk_get_parent(child, p);
                //printf("%s has parent %s\n", gclk_get_name(child), gclk_get_name(parent));
                /* if some other clock can configure this one as a parent, it can not be a leaf */
                if (parent == clk) {
                    return false;
                }
            }
        }
    }

    return true;
}

bool gclk_is_used(const gclk_t *clk)
{
    for (unsigned i = 0; i < gclk_get_cnt(); i++) {
        const gclk_t *child = gclk_get(i);
        if (child != clk && gclk_get_current_parent(child) == clk) {
            return true;
        }
    }
    return false;
}

uint32_t gclk_set_freq(const gclk_t *gclk, uint32_t freq) {
    if (gclk == NULL) {
        LOG_DEBUG("can not set frequency of NULL\n");
        return -1;
    } else if (gclk->separated_ops) {
        const gclk_t *parent = gclk_get_current_parent(gclk);
        uint32_t parent_freq = gclk_get_current_freq(parent);

        if (gclk_is_divider(gclk)) {
            gclk_get_scale_ops(gclk)->set_factor(gclk, parent_freq / freq);
        } else if (gclk_is_multiplier(gclk)) {
            gclk_get_scale_ops(gclk)->set_factor(gclk, freq / parent_freq);
        } else {
            LOG_ERROR("called %s for a clock that is not scalable!\n", __FUNCTION__);
            return 0;
        }
        return freq;
    }

    printf("ERROR! The clock config implementation was not migrated to separated_ops pattern!\n");
    assert(false);
    return 0;
}

/* the topology MUST contain all clocks from the source to the output clock that we are interesed in.
 * input_topology[0] conatins the ouput clock, input_topology[len - 1] contains the source */
uint32_t gclk_get_min_freq_using_topology_conf(clk_topology_entry_t *topology, uint32_t len) {

    if (!len || !topology[0].clk) {
        LOG_DEBUG("%s: min freq of an empty topology or NULL is always 0\n", __FUNCTION__);
        return 0;
    }

    uint32_t min_mul = 1;
    uint32_t max_div = 1;

    const gclk_t *clk = topology[len - 1].clk;
    uint32_t src_freq = gclk_get_input_freq(clk);

    if (!src_freq) {
        return 0;
    }

    for (int i = len - 1; i >= 0; i--) {
        clk = topology[i].clk;
        if (gclk_is_divider(clk)) {
            max_div *= gclk_factor_max(clk);
        } else if (gclk_is_multiplier(clk)) {
            min_mul *= gclk_factor_min(clk);
        }
    }

    return (src_freq * min_mul) / max_div;
}

void gclk_print_topology_metadata(clk_topology_entry_t *t, int len) {
    for (int i = 0; i < len; i++) {
        const gclk_t *clk = t[i].clk;
        printf("[%s]: scalable:", gclk_get_name(clk));

        if (gclk_is_scalable(clk)) {
            printf("1 (%c%u - %u)", gclk_is_divider(clk) ? '/' : 'x',
                                      gclk_factor_min(clk), gclk_factor_max(clk));;
        } else {
            printf("0");
        }

        printf("\n");
    }
}

uint32_t gclk_get_max_freq_using_topology_conf(clk_topology_entry_t *topology, uint32_t len) {

    if (!len || !topology[0].clk) {
        LOG_DEBUG("%s: max freq of an empty topology or NULL is always 0\n", __FUNCTION__);
        return 0;
    }

    uint32_t max_mul = 1;
    uint32_t min_div = 1;

    const gclk_t *clk = topology[len - 1].clk;
    uint32_t src_freq = gclk_get_input_freq(clk);

    if (!src_freq) {
        return 0;
    }

    for (int i = len - 1; i >= 0; i--) {
        clk = topology[i].clk;
        if (gclk_is_divider(clk)) {
            min_div *= gclk_factor_min(clk);
        } else if (gclk_is_multiplier(clk)) {
            max_mul *= gclk_factor_max(clk);
        }
    }

    return src_freq * max_mul / min_div;
}

void gclk_advance_topology_to_next_frequency_setting(clk_topology_entry_t *topology, size_t len, size_t conf_id) {
    const gclk_t *clks[len];
    uint32_t factors[len];

    for (unsigned i = 0; i < len; i++) {
        clks[i] = topology[i].clk;
    }

    gclk_get_nth_factors_config(clks, factors, len, conf_id);

    for (unsigned i = 0; i < len; i++) {
        if (gclk_is_scalable(clks[i])) {
            topology[i].factor = factors[i];
        }
    }
}

/* @todo: this needs to take "all possible frequency reconfigurations" of zhis particular topology into account */
uint32_t gclk_get_min_freq_of_current_topology(const gclk_t *gclk) {
    uint32_t topolen = gclk_get_clk_subtree_max_depth(gclk, 0) + 1;

    clk_topology_entry_t topology[topolen];

    /* must be set ast starting point for the current topology */
    topology[0].clk = gclk;
    topolen = gclk_get_current_topology_config(topology, topolen);
    return gclk_get_min_freq_using_topology_conf(topology, topolen);
}

/* @todo: this needs to take "all possible frequency reconfigurations" of zhis particular topology into account */
uint32_t gclk_get_max_freq_of_current_topology(const gclk_t *gclk) {
    uint32_t topolen = gclk_get_clk_subtree_max_depth(gclk, 0) + 1;

    clk_topology_entry_t topology[topolen];

    /* must be set ast starting point for the current topology */
    topology[0].clk = gclk;
    topolen = gclk_get_current_topology_config(topology, topolen);
    return gclk_get_max_freq_using_topology_conf(topology, topolen);
}

/**
 * @brief get the accuracy of the clock
 * @note  this value may change when this clock is siwtched to another sources
 */
unsigned long gclk_get_accuracy(const gclk_t *gclk);

uint32_t gclk_get_current_topology_config(clk_topology_entry_t *topology, uint32_t size) {
    if (topology) {
        /* ensure everything is zeroed, apart from the clock instance of the very first entry */
        const gclk_t *leaf = topology[0].clk;
        memset(topology, 0, sizeof(clk_topology_entry_t) * size);

        topology[0].clk = leaf;

        for (unsigned i = 0; i < size; i++) {
            topology[i].clk_freq = gclk_get_current_freq(topology[i].clk);

            if (gclk_is_gateable(topology[i].clk)) {
                topology[i].enabled = gclk_is_enabled(topology[i].clk);
            } else {
                /* a non-gatable clock is assumed to be always active */
                topology[i].enabled = true;
            }

            if (gclk_is_scalable(topology[i].clk)) {
                topology[i].factor = gclk_get_current_factor(topology[i].clk);
            }

            if (gclk_is_source(topology[i].clk)) {
                return i;
            }
            const gclk_t *parent = gclk_get_current_parent(topology[i].clk);
            topology[i].par_idx = gclk_parent2idx(topology[i].clk, parent);

            //if (i > 0) {
            //    topology[i-1].par_freq = topology[i].clk_freq;
            //}

            if (i < (size -1)) {
                topology[i+1].clk = parent;
            }
        }

    }
    return 0;
}

unsigned int gclk_config_cnt(const gclk_t *clk) {
    return clk->flags.conf_cnt;
}

unsigned int gclk_factor_cnt(const gclk_t *clk) {
    if (!clk) {
        return 0;
    }

    if (!gclk_is_scalable(clk)) {
        return 1;
    }

    return gclk_config_cnt(clk);
}

unsigned int gclk_parent_cnt(const gclk_t *clk) {
    if ((!clk) || clk->flags.is_source) {
        return 0;
    } else if (clk->flags.muxable) {
        return gclk_config_cnt(clk);
    }

    /* every clock that is not muxable and has no source
     * must have exactly on (static) parent */
    return 1;
}

unsigned int gclk_idx2factor(const gclk_t *gclk, unsigned int idx) {
    if (gclk->factor_map_op) {
        return gclk->factor_map_op(gclk, idx, false);
    }

    /* return 1 as neutral factor for div as well as mul */
    return 1;
}

unsigned int gclk_factor2idx(const gclk_t *gclk, unsigned int factor) {
    for (unsigned i = 0; i < gclk->flags.conf_cnt; i++) {
        if (factor == gclk_idx2factor(gclk, i)) {
            return i;
        }
    }
    return gclk->flags.conf_cnt + 1;
}

uint32_t gclk_factor2regval(const gclk_t *gclk, uint32_t factor) {
    if (gclk->factor_map_op) {
        uint32_t idx = gclk_factor2idx(gclk, factor);

        return gclk->factor_map_op(gclk, idx, true);
    }
    return 0;
}

uint32_t gclk_regval2factor(const gclk_t *gclk, uint32_t regval) {
    if (gclk->factor_map_op) {
        for (unsigned i = 0; i < gclk->flags.conf_cnt; i++) {
            if (regval == gclk->factor_map_op(gclk, i, true)) {
                return gclk->factor_map_op(gclk, i, false);
            }
        }
    }
    return 0;
}

const gclk_t* gclk_idx2parent(const gclk_t *gclk, unsigned int idx) {
    if (!gclk_is_muxable(gclk)) {
        return gclk->fixed_parent;
    }
    if (gclk->parent_map_op) {
        const gclk_t *parent;
        gclk->parent_map_op(gclk, &parent, idx);
        return parent;
    }

    return NULL;
}

unsigned int gclk_idx2parentregval(const gclk_t *gclk, unsigned int idx) {
    if (gclk->parent_map_op) {
        const gclk_t *parent;
        return gclk->parent_map_op(gclk, &parent, idx);
    }

    return 0;
}

int gclk_parent2idx(const gclk_t *gclk, const gclk_t *parent) {
    for (unsigned i = 0; i < gclk_config_cnt(gclk); i++) {
        if (gclk_idx2parent(gclk, i) == parent) {
            return i;
        }
    }
    return -1;
}

uint32_t gclk_parent2regval(const gclk_t *gclk, const gclk_t *parent) {
    if (gclk->parent_map_op) {
        for (unsigned i = 0; i < gclk_config_cnt(gclk); i++) {
            const gclk_t *tmp_parent;
            uint32_t regval = gclk->parent_map_op(gclk, &tmp_parent, i);
            if (parent == tmp_parent) {
                return regval;
            }
        }
    }
    return 0;
}

const gclk_t* gclk_regval2parent(const gclk_t *gclk, uint32_t regval) {
    if (gclk->parent_map_op) {
        for (unsigned i = 0; i < gclk_config_cnt(gclk); i++) {
            const gclk_t *parent;
            if (gclk->parent_map_op(gclk, &parent, i) == regval) {
                return parent;
            }
        }
    }
    return NULL;
}

uint32_t gclk_map_func_lut(const gclk_t *clk, unsigned int idx, bool to_regval) {
    if (to_regval) {
        return clk->factor_mapping.lut[idx].reg_val;
    }
    return clk->factor_mapping.lut[idx].factor;
}

uint32_t gclk_map_func_ptr_lut(const gclk_t *clk, unsigned int idx, bool to_regval) {
    if (to_regval) {
        return *(clk->factor_mapping.ptr_lut[idx].reg_val_ptr);
    }
    return clk->factor_mapping.ptr_lut[idx].factor;
}

uint32_t gclk_map_func_uptree_cross_ref_luf(const gclk_t *clk, const clk_topology_entry_t *conf) {
    // const gclk_t *ref_clk;
    return clk->factor_mapping.cross_ref->luf(clk, conf);
}

uint32_t gclk_map_func_regval_as_numval_range8(const gclk_t *clk, unsigned int idx, bool to_regval) {
    /* As the default mapping behavior for range8 is 1:1 between register value and numerical value,
     * there is no difference regardless on which direction the mapping happens in.
      */
    (void)to_regval;
    /* By default this mapping directly uses the numerical value as the register value.
     * Apart from the default case this may be manipulated by additional flags to eg. map the range
     * to 0 - N (ie. using the index as register value). */
    return idx + clk->factor_mapping.range8->min;
}

uint32_t gclk_map_func_idx_as_regval_range8(const gclk_t *clk, unsigned int idx, bool to_regval) {
    if (to_regval) {
        return idx;
    }
    return idx + clk->factor_mapping.range8->min;
}

uint32_t gclk_map_func_regval_as_numval_range16(const gclk_t *clk, unsigned int idx, bool to_regval) {
    (void)to_regval;
    return idx + clk->factor_mapping.range16->min;
}

uint32_t gclk_map_func_idx_as_regval_range16(const gclk_t *clk, unsigned int idx, bool to_regval) {
    if (to_regval) {
        return idx;
    }
    return idx + clk->factor_mapping.range16->min;
}

uint32_t gclk_map_func_list8(const gclk_t *clk, unsigned int idx, bool to_regval) {
    if (to_regval) {
        return idx;
    }
    return clk->factor_mapping.list8[idx];
}

uint32_t gclk_map_func_list16(const gclk_t *clk, unsigned int idx, bool to_regval) {
    if (to_regval) {
        return idx;
    }
    return clk->factor_mapping.list16[idx];
}

uint32_t gclk_map_func_fixed_factor(const gclk_t *clk, unsigned int idx, bool to_regval) {
    (void)idx;
    (void)to_regval; /* fixed factor has neither an index nor a configuration register */
    return clk->factor_mapping.fixed_factor;
}

uint32_t gclk_map_parent_lut(const gclk_t *clk, const gclk_t **parent, unsigned int idx) {
    *parent = clk->parent_mapping.lut[idx].parent;
    return clk->parent_mapping.lut[idx].config_reg_val;
}

uint32_t gclk_map_parent_list(const gclk_t *clk, const gclk_t **parent, unsigned int idx) {
    *parent = clk->parent_mapping.plist[idx];
    /* for a list mapping the register value is always implicitly encoded by the index
     * so there is no difference between index and register value */
    return idx;
}

size_t gclk_get_factors_config_cnt_from_topology(clk_topology_entry_t *topology, size_t len) {
    size_t combined_cnt = 1;
    for (unsigned i = 0; i < len; i++) {
        combined_cnt *= gclk_factor_cnt(topology[i].clk);
    }
    return combined_cnt;
}

size_t gclk_get_topology_config_cnt(const gclk_t *clk) {
    if (gclk_is_source(clk)) {
        return 1;
    }

    uint32_t src_topologies = 0;
    for (unsigned i = 0; i < gclk_parent_cnt(clk); i++) {
       src_topologies += gclk_get_topology_config_cnt(gclk_get_parent(clk, i));
    }

    return src_topologies;
}

size_t gclk_get_factors_config_cnt(const gclk_t **clks, size_t cnt) {
    size_t combined_cnt = 1;
    for (unsigned i = 0; i < cnt; i++) {
        combined_cnt *= gclk_factor_cnt(clks[i]);
    }
    return combined_cnt;
}

unsigned int gclk_get_nth_topology(clk_topology_entry_t *topology, size_t max_len, size_t tid) {
    uint32_t tmp_topo_idx = 0;

    for (unsigned i = 0; i < max_len; i++) {
        if (gclk_is_muxable(topology[i].clk)) {
            for (unsigned p = 0; p < gclk_parent_cnt(topology[i].clk); p++) {
                uint32_t par_topo_cnt = gclk_get_topology_config_cnt(gclk_get_parent(topology[i].clk, p));
                /* if the wanted topology is part of this parent continue there */
                if (tid < (tmp_topo_idx + par_topo_cnt)) {
                    topology[i].par_idx = p;
                    break;
                } else {
                    tmp_topo_idx += par_topo_cnt;
                }
            }
        } else {
            topology[i].par_idx = 0;
        }

        if (gclk_is_source(topology[i].clk)) {
            return i + 1;
        } else {
            topology[i + 1].clk = gclk_get_parent(topology[i].clk, topology[i].par_idx);
        }
    }

    LOG_ERROR("gclk_get_nth_topology: didn't find source!\n");
    return max_len;
}

unsigned int gclk_topology2id(const clk_topology_entry_t *topology_conf, uint32_t topo_len) {
    const gclk_t *leaf_clk = topology_conf[0].clk;
    uint32_t max_involved_clks = gclk_get_clk_subtree_max_depth(leaf_clk, 0) + 1;

    clk_topology_entry_t topology[max_involved_clks];
    memset(topology, 0, sizeof(clk_topology_entry_t) * max_involved_clks);

    unsigned int topocnt = gclk_get_topology_config_cnt(leaf_clk);
    for (unsigned ti = 0; ti < topocnt; ti++) {
        topology[0].clk = leaf_clk;
        size_t size = gclk_get_nth_topology(topology, max_involved_clks, ti);
        if (size == topo_len) {
            bool topo_match = true;
            for (unsigned n = 0; n < topo_len; n++) {
                if (topology_conf[n].clk != topology[n].clk) {
                    topo_match = false;
                    break;
                }
            }

            if (topo_match) {
                return ti;
            }
        }

    }

    return topocnt;
}

void gclk_get_nth_factors_config(const gclk_t **clks, uint32_t *factors, size_t set_cnt, size_t n) {
    uint32_t base = 1;
    for (unsigned i = 0; i < set_cnt; i++) {
        if (gclk_is_scalable(clks[i])) {
            uint32_t clk_conf_cnt = gclk_factor_cnt(clks[i]);
            factors[i] = gclk_idx2factor(clks[i], n / base % clk_conf_cnt);
            base *= clk_conf_cnt;
        }
    }
}

uint32_t gclk_get_nth_config_equivalent_factor(const gclk_t **clks, size_t clks_cnt, size_t n) {
    uint32_t combined_fact = 1;
    uint32_t base = 1;
    for (unsigned i = 0; i < clks_cnt; i++) {
        uint32_t clk_conf_cnt = gclk_config_cnt(clks[i]);
        combined_fact *= gclk_idx2factor(clks[i], n / base % clk_conf_cnt);
        base *= clk_conf_cnt;
    }
    return combined_fact;
}

uint32_t gclk_get_factor_config_freq(uint32_t fi, uint32_t *mfacts, size_t mfact_cnt, uint32_t *dfacts, size_t dfact_cnt) {
    uint32_t fo = fi;
    for (size_t m = 0; m < mfact_cnt; m++) {
        fo *= mfacts[m];
    }
    for (size_t d = 0; d < dfact_cnt; d++) {
        fo /= dfacts[d];
    }
    return fo;
}

void gclk_calculate_topology_config_freqs(clk_topology_entry_t *topology, size_t len) {
    uint32_t fs = gclk_get_input_freq(topology[len - 1].clk);
    uint32_t mul = 1;
    uint32_t div = 1;

    for (int i = len - 1; i >= 0; i--) {
        if (gclk_is_divider(topology[i].clk)) {
            div *= topology[i].factor;
        } else if (gclk_is_multiplier(topology[i].clk)) {
            mul *= topology[i].factor;
        }
        topology[i].clk_freq = fs * mul / div;
    }
}

bool gclk_match_closest_full_iter(uint32_t fi, uint32_t fo,
                                 const gclk_t **mul_clks, size_t mul_clks_cnt, uint32_t *mfacts,
                                 const gclk_t **div_clks, size_t div_clks_cnt, uint32_t *dfacts) {
    uint32_t best_diff = 0;
    uint32_t best_m = 0;
    uint32_t best_d = 0;
    uint32_t chk_diff;
    bool baseline_set = false;

    for (size_t m = 0; m < gclk_get_factors_config_cnt(mul_clks, mul_clks_cnt); m++) {
        gclk_get_nth_factors_config(mul_clks, mfacts, mul_clks_cnt, m);
        for (size_t d = 0; d < gclk_get_factors_config_cnt(div_clks, div_clks_cnt); d++) {
            gclk_get_nth_factors_config(div_clks, dfacts, div_clks_cnt, d);
            uint32_t fo_tmp = gclk_get_factor_config_freq(fi, mfacts, mul_clks_cnt, dfacts, div_clks_cnt);
            chk_diff = gclk_abs_freq_diff(fo_tmp, fo);
            if (chk_diff == 0) {
                best_m = m;
                best_d = d;
                best_diff = chk_diff;
                goto search_done;
            }
            if (baseline_set) {
                if (chk_diff < best_diff) {
                    best_m = m;
                    best_d = d;
                    best_diff = chk_diff;
                }
            } else {
                best_m = m;
                best_d = d;
                best_diff = chk_diff;
                baseline_set = true;
            }
        }
    }

    search_done:
    /* get the actual fitted params only at the end to avoid copying them every time
     * a better one is found */
    gclk_get_nth_factors_config(mul_clks, mfacts, mul_clks_cnt, best_m);
    gclk_get_nth_factors_config(div_clks, dfacts, div_clks_cnt, best_d);
    return true;
}

/* assuming we always have two divs after the mul here */
static uint32_t _get_combined_div(uint32_t fi, uint32_t fo, uint32_t mul) {

    /* as we can only scale down after the mul a higher freq is never possible */
    if (fo > (fi * mul)) {
       return 0;
    } else if (((fi * mul) % fo) != 0) {
        /* in this case we take advantage of the fact that any chain of integer valued dividers
         * can in any case only represent one combined integer value. So if modulo indicates
         * that value would need to be non-integer, we can rule out that option.
         * Of course we can only do that if we *know* that there indeed is a valid config for the given Fo. */
        //printf("fi * mul (%lu) can not be scaled down to %lu via integer dividers\n", fi*mul, fo);
       return 0;
    } else {
       return (fi * mul) / fo;
    }
}

static bool _fit_factors_recurse_trial(uint32_t product, const gclk_t **clks, uint32_t *factors, size_t clks_cnt) {
    const gclk_t *clk = clks[0];
    size_t set_size = gclk_config_cnt(clk);
    for (size_t i = 0; i < set_size; i++) {
        uint32_t chk_fact = gclk_idx2factor(clk, i);
        /* on the last set the remaining factor must match exactly */
        if (clks_cnt == 1) {
            if(chk_fact == product) {
                factors[0] = chk_fact;
                return true;
            } else if (chk_fact > product) {
                /* if chk_fact is already bigger than the remaining product
                 * there is no way to fit in even more factors.
                 * In that case we can early out */
                return false;
            }
        } else { /* we first need to propagate down to see if there are fitting factors */

            /* as we can assume that only integer scalers will are possible
             * only factors that leave proper integer factors behind are worth checking */
            if (product % chk_fact == 0) {
                /* if we can still fit this factor check if the remaining sets
                 * can represent the remaining factor */
                bool res = _fit_factors_recurse_trial(product / chk_fact, &clks[1], &factors[1], clks_cnt - 1);
                if (res) {
                    factors[0] = chk_fact;
                    return res;
                }
            }
            /* if remaining factors are non-integer or factors below could not be fitted,
             * try the next factor */
        }
    }
    /* factors were checked without success there is no valid option */
    return false;
}

bool gclk_match_iter_mul_recurse_div(uint32_t fi, uint32_t fo,
                                 const gclk_t **mul_clks, size_t mul_clks_cnt, uint32_t *mfacts,
                                 const gclk_t **div_clks, size_t div_clks_cnt, uint32_t *dfacts) {
    /* TODO: it could be preferable to first iterate mul than div or the other way around,
     * depending on the metadata (set lengths, sparsity etc.) */
    for (size_t i = 0; i < gclk_get_factors_config_cnt(mul_clks, mul_clks_cnt); i++) {
        uint32_t combined_mul = gclk_get_nth_config_equivalent_factor(mul_clks, mul_clks_cnt, i);
        uint32_t combined_div_fact = _get_combined_div(fi, fo, combined_mul);
        if (combined_div_fact != 0) {
            bool res = _fit_factors_recurse_trial(combined_div_fact, div_clks, dfacts, div_clks_cnt);
            if (res) {
                /* populate the factors we found matching */
                gclk_get_nth_factors_config(mul_clks, mfacts, mul_clks_cnt, i);
                return true;
            }
        }
    }
    return false;
}

/* trial division factorization */
static int get_factors(uint32_t n, uint32_t *facts, size_t fact_len){
    unsigned idx = 0;

    while (n % 2 == 0) {
        facts[idx < fact_len ? idx++ : idx] = 2;
        n /= 2;
    }

    uint32_t f = 3;

    while (f * f <= n) {
        if (n % f == 0) {
            facts[idx < fact_len ? idx++ : idx] = f;
            n /= f;
        } else {
            f += 2;
        }
    }
    if (n != 1) {
        facts[idx < fact_len ? idx++ : idx] = n;
    }
    return idx;
}

/* returns true if there is an applicable pair */
static bool _split_factor_across_clocks(uint32_t combined, const gclk_t **clks, uint32_t *fitted_factors, size_t numof_clks) {
    /* will hold all individual factors of the combined factor
     * TODO: use some reasonable upper bound on how many factors to expect
     * multiple *same* factors could also be stored more efficiently with counters */
    uint32_t factors[20];

    uint32_t max_possible_fact = 1;
    uint32_t min_possible_fact = 1;

    /* get min and max possible factors to early-out when violating these limits.
     * This assumes there is an order (or at least an ordered way of access) for available factors */
    for (unsigned i = 0; i < numof_clks; i++) {
        max_possible_fact *= gclk_factor_max(clks[i]);
        min_possible_fact *= gclk_factor_min(clks[i]);
        fitted_factors[i] = 1; /* start at one to add any additional factor by multiplication */
    }

    /* if we are out of absolute min/max limits we can already stop right here */
    if ((combined > max_possible_fact) || (combined < min_possible_fact)) {
        return false;
    }

    unsigned factcnt = get_factors(combined, factors, ARRAY_SIZE(factors));

    if (factcnt > ARRAY_SIZE(factors)) {
        //printf("couldn't factorize!\n");
        return false;
    } else {
        //printf("factors of %lu: ", combined);
        //for (unsigned i = 0; i < factcnt; i++) {
        //    printf("%lu ", factors[i]);
        //}
        //printf("\n");

        size_t set_idx = 0;
        for (unsigned i = 0; i < factcnt; i++) {
            //printf("%lu ", factors[i]);
            while (gclk_factor_max(clks[i]) <
                   (factors[i] * fitted_factors[set_idx]) ) {
                set_idx++;
                /* if there is no other set we can fit remaining factors to, this
                 * is infeasible */
                if (set_idx >= numof_clks) {
                   //printf("couldn't fit factor[%u]:%lu!\n", i, factors[i]);
                   return false;
                }
            }

            fitted_factors[set_idx] *= factors[i];
        }
    }
    return true;
}

bool gclk_match_iter_mul_factorize_div(uint32_t fi, uint32_t fo,
                                 const gclk_t **mul_clks, size_t mul_clks_cnt, uint32_t *mfacts,
                                 const gclk_t **div_clks, size_t div_clks_cnt, uint32_t *dfacts) {
    /* TODO: it could be preferrable to first iterate mul than div or the other way around,
     * depending on the metadata (set lengths, sparsity etc.) */
    for (size_t i = 0; i < gclk_get_factors_config_cnt(mul_clks, mul_clks_cnt); i++) {
        uint32_t combined_mul = gclk_get_nth_config_equivalent_factor(mul_clks, mul_clks_cnt, i);
        uint32_t combined_div_fact = _get_combined_div(fi, fo, combined_mul);
        if (combined_div_fact != 0) {
            bool res = _split_factor_across_clocks(combined_div_fact, div_clks, dfacts, div_clks_cnt);
            if (res) {
                /* populate the factors we found matching */
                gclk_get_nth_factors_config(mul_clks, mfacts, mul_clks_cnt, i);
                return true;
            }
        }
    }
    return false;
}

/* TODO: there is significant optimization potential here for fathoming ranges based on min/max
 *       ranges and also based on integer feasibility */
bool gclk_match_exact_full_iter(uint32_t fi, uint32_t fo,
                             const gclk_t **mul_clks, size_t mul_clks_cnt, uint32_t *mfacts,
                             const gclk_t **div_clks, size_t div_clks_cnt, uint32_t *dfacts) {
    for (size_t m = 0; m < gclk_get_factors_config_cnt(mul_clks, mul_clks_cnt); m++) {
        gclk_get_nth_factors_config(mul_clks, mfacts, mul_clks_cnt, m);
        for (size_t d = 0; d < gclk_get_factors_config_cnt(div_clks, div_clks_cnt); d++) {
            gclk_get_nth_factors_config(div_clks, dfacts, div_clks_cnt, d);
            uint32_t chk_fo = fi;
            for (unsigned midx = 0; midx < mul_clks_cnt; midx++) {
                chk_fo *= mfacts[midx];
            }
            for (unsigned didx = 0; didx < div_clks_cnt; didx++) {
                chk_fo /= dfacts[didx];
            }
            if (chk_fo == fo) {
               return true;
            }
        }
    }
    return false;
}

static uint32_t gclk_filter_for_specific_clks(bool (*clk_type_chk_fun)(const gclk_t *clk),
                                              const gclk_t **topology, uint32_t topo_len, const gclk_t **specific_clks) {
    uint32_t cnt = 0;

    for (unsigned i = 0; i < topo_len; i++) {
        const gclk_t *clk = topology[i];
        if (clk_type_chk_fun(clk)) {
            if (specific_clks) {
                specific_clks[cnt] = clk;
            }
            cnt++;
        }
    }

    return cnt;
}

uint32_t gclk_get_dividers_from_topology(const gclk_t **topology, uint32_t topo_len, const gclk_t **div_clks) {
    return gclk_filter_for_specific_clks(gclk_is_divider, topology, topo_len, div_clks);
}

uint32_t gclk_get_multipliers_from_topology(const gclk_t **topology, uint32_t topo_len, const gclk_t **mul_clks) {
    return gclk_filter_for_specific_clks(gclk_is_multiplier, topology, topo_len, mul_clks);
}

static uint32_t _get_clock_factor(const gclk_t **clks, const gclk_t *clk, uint32_t *factors, size_t len) {
    for (unsigned i = 0; i < len; i++) {
        if (clks[i] == clk) {
            return factors[i];
        }
    }
    return 1;
}

uint32_t gclk_match_freq_conf(clk_topology_entry_t *topology, uint32_t topo_len,
                              uint32_t f_in, uint32_t f_out_target,
                              gclk_factor_match_func_t match_op) {
    /* temporary workaround to convert the more verbose topology description into a simple
     * list of clocks to be used on gclk factor matching functions */
    const gclk_t *topo_clocks[topo_len];
    for (unsigned i = 0; i < topo_len; i++) {
        topo_clocks[i] = topology[i].clk;
    }

    /* get number of multipliers and scalers in this topology */
    uint32_t div_cnt = gclk_get_dividers_from_topology(topo_clocks, topo_len, NULL);
    uint32_t mul_cnt = gclk_get_multipliers_from_topology(topo_clocks, topo_len, NULL);

    /* allocate the needed space */
    const gclk_t* div_clks[div_cnt];
    const gclk_t* mul_clks[mul_cnt];

    gclk_get_dividers_from_topology(topo_clocks, topo_len, div_clks);
    gclk_get_multipliers_from_topology(topo_clocks, topo_len, mul_clks);

    uint32_t mfacts[mul_cnt];
    uint32_t dfacts[div_cnt];
    bool matched = match_op(f_in, f_out_target,
                            mul_clks, mul_cnt, mfacts,
                            div_clks, div_cnt, dfacts);

    if (!matched) {
        return GCLK_INVALID_FREQ;
    }

    topology[topo_len - 1].clk_freq = f_in;

    for (int i = topo_len - 2; i >= 0; i--) {
        const gclk_t *cur_clk = topology[i].clk;
        uint32_t parent_freq = topology[i + 1].clk_freq;
        if (gclk_is_divider(cur_clk)) {
             topology[i].clk_freq = parent_freq / _get_clock_factor(div_clks, cur_clk, dfacts, div_cnt);
        } else if (gclk_is_multiplier(cur_clk)) {
             topology[i].clk_freq = parent_freq * _get_clock_factor(mul_clks, cur_clk, mfacts, mul_cnt);
        } else {
             topology[i].clk_freq = parent_freq;
        }
    }
    return gclk_get_factor_config_freq(f_in, mfacts, mul_cnt, dfacts, div_cnt);
}

uint32_t gclk_get_input_freq(const gclk_t *clk) {
    if (clk) {
        if (gclk_is_source(clk)) {
            return clk->fixed_input_freq;
        }
        return gclk_get_current_freq(gclk_get_current_parent(clk));
    }

    return 0;
}

bool gclk_is_sourced_by(const gclk_t *clk, const gclk_t *src) {

    const gclk_t *current_parent = gclk_get_current_parent(clk);

    /* while we can walk up the tree */
    while (current_parent != clk) {
        if (clk == src) {
            return true;
        }
        clk = current_parent;
        current_parent = gclk_get_current_parent(clk);
    }

    return false;
}

/* @todo can also be implemented using "forward topology view" */
bool gclk_affected_by_change(const gclk_t *altered_clock, const gclk_t *affected_clock) {
    uint32_t child_idx = 0;
    if (altered_clock == affected_clock) {
        return true;
    }
    const gclk_t *child = gclk_get_child(altered_clock, child_idx);
    while (child != NULL) {
        child_idx++;
        if (child == affected_clock || gclk_affected_by_change(child, affected_clock)) {
            return true;
        }
        child = gclk_get_child(altered_clock, child_idx);
    }

    return false;
}

bool gclk_must_be_stopped_for_change(const gclk_t *clk) {
    return clk->flags.topology_flags & GCLK_STOP_FOR_UPDATE;
}

bool gclk_parent_must_be_stopped_for_change(const gclk_t *clk) {
    return clk->flags.topology_flags & GCLK_STOP_PARENT_FOR_UPDATE;
}

gclk_cmp_result_t gclk_cmp_topology_for_closest_leaf_freq(clk_topology_entry_t *topo_best, size_t len1,
                clk_topology_entry_t *topo_cmp, size_t len2, void *arg) {
    (void)len1;
    (void)len2;
    uint32_t target_freq = *(uint32_t*)arg;

    if (topo_best[0].clk_freq == GCLK_INVALID_FREQ) {
        return GCLK_CONF_BETTER;
    } else {
        uint32_t cmp_diff = gclk_abs_freq_diff(target_freq, topo_cmp[0].clk_freq);
        uint32_t best_diff = gclk_abs_freq_diff(target_freq, topo_best[0].clk_freq);
        if (cmp_diff < best_diff) {
            return GCLK_CONF_BETTER;
        } else if (cmp_diff == best_diff) {
            return GCLK_CONF_EQUAL;
        }
    }

    return GCLK_CONF_WORSE;
}

static inline uint32_t _topo_freq_sum(clk_topology_entry_t *topo, size_t len) {
    uint32_t sum = 0;
    for (uint32_t i = 0; i < len; i++) {
        sum += topo[i].clk_freq;
    }
    return sum;
}

gclk_cmp_result_t gclk_cmp_topology_for_closest_leaf_freq_min_sum(clk_topology_entry_t *topo_best, size_t len1,
                clk_topology_entry_t *topo_cmp, size_t len2, void *arg) {
    uint32_t target_freq = *(uint32_t*)arg;

    /* the frequency being closer is given priority over the max-sum condition */
    if ((topo_best[0].clk_freq == GCLK_INVALID_FREQ) ||
        (gclk_abs_freq_diff(target_freq, topo_cmp[0].clk_freq) <
         gclk_abs_freq_diff(target_freq, topo_best[0].clk_freq))) {
        return GCLK_CONF_BETTER;
    }else if (topo_cmp[0].clk_freq == topo_best[0].clk_freq) {
        /* if both frequencies are the same, the one with the lower frequency-sum is favoured */
        uint32_t best_mhz_sum = _topo_freq_sum(topo_best, len1);
        uint32_t cmp_mhz_sum = _topo_freq_sum(topo_cmp, len2);
        if (cmp_mhz_sum < best_mhz_sum) {
            return GCLK_CONF_BETTER;
        } else if (cmp_mhz_sum == best_mhz_sum) {
            return GCLK_CONF_EQUAL;
        }
    }

    return GCLK_CONF_WORSE;
}

gclk_cmp_result_t gclk_cmp_topology_for_closest_leaf_freq_max_sum(clk_topology_entry_t *topo_best, size_t len1,
                clk_topology_entry_t *topo_cmp, size_t len2, void *arg) {
    uint32_t target_freq = *(uint32_t*)arg;

    /* the frequency being closer is given priority over the max-sum condition */
    if ((topo_best[0].clk_freq == GCLK_INVALID_FREQ) ||
        (gclk_abs_freq_diff(target_freq, topo_cmp[0].clk_freq) <
         gclk_abs_freq_diff(target_freq, topo_best[0].clk_freq))) {
        /* the frequency being closer has priority */
        return GCLK_CONF_BETTER;
    }else if (topo_cmp[0].clk_freq == topo_best[0].clk_freq) {
        /* if both frequencies are the same, the one with the higher frequency sum is favoured */
        uint32_t best_mhz_sum = _topo_freq_sum(topo_best, len1);
        uint32_t cmp_mhz_sum = _topo_freq_sum(topo_cmp, len2);
        if (cmp_mhz_sum > best_mhz_sum) {
            return GCLK_CONF_BETTER;
        } else if (cmp_mhz_sum == best_mhz_sum) {
            return GCLK_CONF_EQUAL;
        }
    }

    return GCLK_CONF_WORSE;
}

static inline uint32_t _max_freq_in_topo(clk_topology_entry_t *topo, size_t len) {
    uint32_t max = 0;
    for (uint32_t i = 0; i < len; i++) {
        max = topo[i].clk_freq > max ? topo[i].clk_freq : max;
    }
    return max;
}

gclk_cmp_result_t gclk_cmp_topology_for_closest_leaf_freq_min_max(clk_topology_entry_t *topo_best, size_t len1,
                clk_topology_entry_t *topo_cmp, size_t len2, void *arg) {
    uint32_t target_freq = *(uint32_t*)arg;

    /* the frequency being closer is given priority over the min-max condition */
    if ((topo_best[0].clk_freq == GCLK_INVALID_FREQ) ||
        (gclk_abs_freq_diff(target_freq, topo_cmp[0].clk_freq) <
         gclk_abs_freq_diff(target_freq, topo_best[0].clk_freq))) {
        return GCLK_CONF_BETTER;
    } else if (topo_cmp[0].clk_freq == topo_best[0].clk_freq) {
        /* if both frequencies are the same, the one with the lower max frequency is favoured */
        uint32_t best_mhz_max = _max_freq_in_topo(topo_best, len1);
        uint32_t cmp_mhz_max = _max_freq_in_topo(topo_cmp, len2);
        if (cmp_mhz_max < best_mhz_max) {
            return GCLK_CONF_BETTER;
        } else if (cmp_mhz_max == best_mhz_max) {
            return GCLK_CONF_EQUAL;
        }
    }

    return GCLK_CONF_WORSE;
}

gclk_cmp_result_t gclk_cmp_topology_for_closest_leaf_freq_max_max(clk_topology_entry_t *topo_best, size_t len1,
                clk_topology_entry_t *topo_cmp, size_t len2, void *arg) {
    uint32_t target_freq = *(uint32_t*)arg;

    /* the frequency being closer is given priority over the max-max condition */
    if ((topo_best[0].clk_freq == GCLK_INVALID_FREQ) ||
        (gclk_abs_freq_diff(target_freq, topo_cmp[0].clk_freq) <
         gclk_abs_freq_diff(target_freq, topo_best[0].clk_freq))) {
        return GCLK_CONF_BETTER;
    }else if (topo_cmp[0].clk_freq == topo_best[0].clk_freq) {
        /* if both frequencies are the same, the one with the higher max frequency is favoured */
        uint32_t best_mhz_max = _max_freq_in_topo(topo_best, len1);
        uint32_t cmp_mhz_max = _max_freq_in_topo(topo_cmp, len2);
        return (cmp_mhz_max > best_mhz_max);
        if (cmp_mhz_max > best_mhz_max) {
            return GCLK_CONF_BETTER;
        } else if (cmp_mhz_max == best_mhz_max) {
            return GCLK_CONF_EQUAL;
        }
    }

    return GCLK_CONF_WORSE;
}

gclk_cmp_result_t gclk_cmp_topology_for_max_leaf_freq(clk_topology_entry_t *topo_best, size_t len1,
                clk_topology_entry_t *topo_cmp, size_t len2, void *arg) {
    (void)len1;
    (void)len2;
    (void)arg;
    if (topo_best[0].clk_freq == GCLK_INVALID_FREQ || topo_cmp[0].clk_freq > topo_best[0].clk_freq) {
        return GCLK_CONF_BETTER;
    } else if (topo_best[0].clk_freq == topo_cmp[0].clk_freq) {
        return GCLK_CONF_EQUAL;
    } else {
        return GCLK_CONF_WORSE;
    }
}

gclk_cmp_result_t gclk_cmp_topology_for_min_nz_leaf_freq(clk_topology_entry_t *topo_best, size_t len1,
        clk_topology_entry_t *topo_cmp, size_t len2, void *arg) {
    (void)len1;
    (void)len2;
    (void)arg;
    if (topo_cmp[0].clk_freq == 0) {
        return GCLK_CONF_INVALID;
    } else if (topo_best[0].clk_freq == GCLK_INVALID_FREQ || topo_best[0].clk_freq < topo_cmp[0].clk_freq) {
        return GCLK_CONF_BETTER;
    } else if (topo_best[0].clk_freq == topo_cmp[0].clk_freq) {
        return GCLK_CONF_EQUAL;
    } else {
        return GCLK_CONF_WORSE;
    }
}

static inline bool _clk_equals_if_present(const gclk_t *clk, uint32_t clk_freq, clk_topology_entry_t *topology, uint32_t topo_len)
{
    for (uint32_t i = 0; i < topo_len; i++) {
        if (topology[i].clk == clk) {
            if (topology[i].clk_freq == clk_freq) {
                return true;
            }
            return false;
        }
    }

    return false;
}

gclk_cmp_result_t gclk_cmp_topology_for_closest_constrained_leaf_freq(clk_topology_entry_t *topo_best, size_t len1,
                clk_topology_entry_t *topo_cmp, size_t len2, void *arg) {
    (void)len1;
    gclk_constrained_cmp_ctx_t *ctx = arg;

    return _clk_equals_if_present(ctx->constraint_clk, ctx->constraint_clk_freq, topo_cmp, len2) &&
           (gclk_abs_freq_diff(ctx->target_freq, topo_cmp[0].clk_freq) <
            gclk_abs_freq_diff(ctx->target_freq, topo_best[0].clk_freq));
}

gclk_cmp_result_t gclk_cmp_topology_for_exact_leaf_freq(clk_topology_entry_t *topo_best, size_t len1,
                clk_topology_entry_t *topo_cmp, size_t len2, void *arg) {
    (void)len1;
    (void)len2;
    uint32_t target_freq = *(uint32_t*)arg;
    if (topo_cmp[0].clk_freq == target_freq) {
        /* the only metric this compare function cares about is whether the target frequency is matched exactly.
         * Therfore the only way a configuration can be considered better than the reference is if the reference
         * has an invalid (or different to target) frequency */
        if (topo_best[0].clk_freq == GCLK_INVALID_FREQ || topo_best[0].clk_freq != target_freq) {
            return GCLK_CONF_BETTER;
        }

        /* in any other case an exactly matched frequency is considered equal */
        return GCLK_CONF_EQUAL;
    }

    /* any frequency that is no exact match is considered invalid */
    return GCLK_CONF_INVALID;
}
