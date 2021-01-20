#ifndef PROPAGATE_CONTROL_H_
#define PROPAGATE_CONTROL_H_

#include "../../attitude/include/matrix.h"
#include "../../attitude/include/mrp.h"

#include "../../telemetry/include/log.h"


namespace propagate {
namespace control {

using namespace attitude;

using matrix3 = attitude::matrix<double, 3, 3>;
using vector3 = attitude::vector<double, 3>;

static const size_t kStateDims = 6;
using states = attitude::vector<double, kStateDims>;
using inputs = attitude::vector<double, kStateDims>;

inputs simple_pd(states x, double t, telemetry::log * tl);
inputs test_out_controller(states x, double t, telemetry::log * tl);
inputs closedloop_linear(states x, double t, telemetry::log * tl);

}  // namespace control
}  // namespace propagate
#endif  // PROPAGATE_CONTROL_H_