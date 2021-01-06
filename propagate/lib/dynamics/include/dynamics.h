#ifndef PROPAGATE_DYNAMICS_H_
#define PROPAGATE_DYNAMICS_H_

#include "C:/Users/kdwn/projects/propagate/propagate/lib/attitude/include/matrix.h"

namespace propagate{


static const size_t kStateDims = 6;
using states = attitude::vector<double, kStateDims>;
using inputs = attitude::vector<double, kStateDims>;
using rates = attitude::vector<double, kStateDims>;

rates A(states x);
rates B(inputs u);

states u(states x);

rates simulate(double dt = 0.001);


}  // namespace propagate
#endif  // PROPAGATE_DYNAMICS_H_