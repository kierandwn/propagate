#ifndef PROPAGATE_DYNAMICS_H_
#define PROPAGATE_DYNAMICS_H_

#include "C:/Users/kdwn/projects/propagate/propagate/lib/attitude/include/matrix.h"

namespace propagate{


static const size_t kStateDims = 6;
using states = attitude::vector<double, kStateDims>;
using inputs = attitude::vector<double, kStateDims>;
using rates = attitude::vector<double, kStateDims>;

rates plant_model(states x);
rates B(inputs u);

rates simulate(states, double, double=0.01);


}  // namespace propagate
#endif  // PROPAGATE_DYNAMICS_H_