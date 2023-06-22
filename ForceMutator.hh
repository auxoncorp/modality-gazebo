#ifndef FORCE_MUTATOR_HH_
#define FORCE_MUTATOR_HH_

#include "modality/mutator_interface.hpp"

using namespace modality;

struct force_mutator_state_s
{
    bool x_pending;
    double x;
    bool y_pending;
    double y;
    bool z_pending;
    double z;
};

void force_mutator_get_iface(struct modality_mutator *m);

#endif /* FORCE_MUTATOR_HH_ */
