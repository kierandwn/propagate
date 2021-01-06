#ifndef PROPAGATE_CONTROL_H_
#define PROPAGATE_CONTROL_H_

#include "C:/Users/kdwn/projects/propagate/propagate/lib/attitude/include/matrix.h"
#include "C:/Users/kdwn/projects/propagate/propagate/lib/attitude/include/mrp.h"


namespace propagate {
namespace control {

using namespace attitude;

using matrix3 = attitude::matrix<double, 3, 3>;
using vector3 = attitude::vector<double, 3>;

static const size_t kStateDims = 6;
using states = attitude::vector<double, kStateDims>;
using inputs = attitude::vector<double, kStateDims>;

matrix3 B(vector3);

matrix3 Bdot(vector3 sigma, vector3 sigma_dot);

vector3 omega(vector3 sigma, vector3 sigma_dot);

vector3 omega_dot(vector3 sigma, vector3 sigma_dot, vector3 sigma_dot_dot,
                  vector3 omega);

matrix3 dcm_dot(vector3 omega, matrix3 dcm);

inputs control(states x, double t);

}  // namespace control
}  // namespace propagate
#endif  // PROPAGATE_CONTROL_H_