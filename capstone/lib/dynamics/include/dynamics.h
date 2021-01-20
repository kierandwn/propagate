#ifndef PROPAGATE_DYNAMICS_H_
#define PROPAGATE_DYNAMICS_H_

#include <vector>
#include <string>

#include "../../attitude/include/matrix.h"
#include "../../config/include/config/solver.h"

namespace propagate{


using namespace std;

static const size_t kStateDims = 6;
using states = attitude::vector<double, kStateDims>;
using inputs = attitude::vector<double, kStateDims>;
using rates = attitude::vector<double, kStateDims>;

rates plant_model(states x);
rates B(inputs u);

rates simulate();

}  // namespace propagate
#endif  // PROPAGATE_DYNAMICS_H_