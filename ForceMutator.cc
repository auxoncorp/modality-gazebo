#include <cstdlib>
#include <cstdint>
#include <cstddef>
#include <cstring>
#include <cassert>

#include "modality/error.h"
#include "modality/types.hpp"
#include "modality/mutator_interface.hpp"

#include "ForceMutator.hh"

#define FORCE_MAX_ABS (1000000.0)

using namespace modality;

static const char X_AXIS[] = "x";
static const char Y_AXIS[] = "y";
static const char Z_AXIS[] = "z";

static const modality_attr_val FORCE_MIN =
{
    .tag = modality_attr_val::Tag::MODALITY_ATTR_VAL_FLOAT,
    .FLOAT =
    {
        ._0 = -FORCE_MAX_ABS,
    }
};

static const modality_attr_val FORCE_MAX =
{
    .tag = modality_attr_val::Tag::MODALITY_ATTR_VAL_FLOAT,
    .FLOAT =
    {
        ._0 = FORCE_MAX_ABS,
    }
};

static const modality_attr_val FORCE_DEFAULT =
{
    .tag = modality_attr_val::Tag::MODALITY_ATTR_VAL_FLOAT,
    .FLOAT =
    {
        ._0 = 0.0,
    }
};

static const modality_mutator_param_descriptor PARAM_DESCS[] =
{
    {
        .value_type = modality_attr_type::MODALITY_ATTR_TYPE_FLOAT,
        .name = X_AXIS,
        .description = "x-axis component expressed in world coordinates",
        .value_min = &FORCE_MIN,
        .value_max = &FORCE_MAX,
        .default_value = &FORCE_DEFAULT,
        .least_effect_value = NULL,
        .value_distribution_kind = modality_value_distribution_kind::MODALITY_VALUE_DISTRIBUTION_KIND_CONTINUOUS,
        .value_distribution_scaling = modality_value_distribution_scaling::MODALITY_VALUE_DISTRIBUTION_SCALING_NONE,
        .value_distribution_option_set = NULL,
        .value_distribution_option_set_length = 0,
        .organization_custom_metadata = NULL,
    },
    {
        .value_type = modality_attr_type::MODALITY_ATTR_TYPE_FLOAT,
        .name = Y_AXIS,
        .description = "y-axis component expressed in world coordinates",
        .value_min = &FORCE_MIN,
        .value_max = &FORCE_MAX,
        .default_value = NULL,
        .least_effect_value = NULL,
        .value_distribution_kind = modality_value_distribution_kind::MODALITY_VALUE_DISTRIBUTION_KIND_CONTINUOUS,
        .value_distribution_scaling = modality_value_distribution_scaling::MODALITY_VALUE_DISTRIBUTION_SCALING_NONE,
        .value_distribution_option_set = NULL,
        .value_distribution_option_set_length = 0,
        .organization_custom_metadata = NULL,
    },
    {
        .value_type = modality_attr_type::MODALITY_ATTR_TYPE_FLOAT,
        .name = Z_AXIS,
        .description = "z-axis component expressed in world coordinates",
        .value_min = &FORCE_MIN,
        .value_max = &FORCE_MAX,
        .default_value = NULL,
        .least_effect_value = NULL,
        .value_distribution_kind = modality_value_distribution_kind::MODALITY_VALUE_DISTRIBUTION_KIND_CONTINUOUS,
        .value_distribution_scaling = modality_value_distribution_scaling::MODALITY_VALUE_DISTRIBUTION_SCALING_NONE,
        .value_distribution_option_set = NULL,
        .value_distribution_option_set_length = 0,
        .organization_custom_metadata = NULL,
    }
};

static const modality_mutator_descriptor MUT_DESC =
{
    .name = "world-force",
    .description = "Add a force expressed in world coordinates and applied at the center of mass of the link",
    .layer = modality_mutator_layer::MODALITY_MUTATOR_LAYER_IMPLEMENTATIONAL,
    .group = "gazebo",
    .operation = modality_mutator_operation::MODALITY_MUTATOR_OPERATION_SET_TO_VALUE,
    .statefulness = modality_mutator_statefulness::MODALITY_MUTATOR_STATEFULNESS_NONE,
    .organization_custom_metadata = NULL,
    .params = PARAM_DESCS,
    .params_length = 3,
};

static void get_description(void *state, const struct modality_mutator_descriptor **desc_ptr)
{
    struct force_mutator_state_s *s = (struct force_mutator_state_s*) state;
    assert(s != NULL);

    (*desc_ptr) = &MUT_DESC;
}

static int inject(void *state, const struct modality_mutation_id *mid, const struct modality_attr_kv *params, size_t params_len)
{
    size_t i;
    struct force_mutator_state_s *s = (struct force_mutator_state_s*) state;
    (void) mid;
    assert(s != NULL);
    for(i = 0; i < params_len; i += 1)
    {
        const struct modality_attr_kv *p = &params[i];
        assert(p->key != NULL);
        assert(p->val.tag == modality_attr_val::Tag::MODALITY_ATTR_VAL_FLOAT);
        if(strncmp(p->key, X_AXIS, 1) == 0)
        {
            s->x = p->val.FLOAT._0;
            s->x_pending = true;
        }
        else if(strncmp(p->key, Y_AXIS, 1) == 0)
        {
            s->y = p->val.FLOAT._0;
            s->y_pending = true;
        }
        else if(strncmp(p->key, Z_AXIS, 1) == 0)
        {
            s->z = p->val.FLOAT._0;
            s->z_pending = true;
        }
    }
    return MODALITY_ERROR_OK;
}

static int reset(void *state)
{
    struct force_mutator_state_s *s = (struct force_mutator_state_s*) state;
    assert(s != NULL);
    s->x_pending = false;
    s->y_pending = false;
    s->z_pending = false;
    return MODALITY_ERROR_OK;
}

void force_mutator_get_iface(struct modality_mutator *m)
{
    assert(m != NULL);
    m->get_description = &get_description;
    m->inject = &inject;
    m->reset = &reset;
}
