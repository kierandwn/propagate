#ifndef PROPAGATE_FRAME_H_
#define PROPAGATE_FRAME_H_

#include <cmath>

#include "C:/Users/kdwn/projects/propagate/propagate/lib/attitude/include/matrix.h"
#include "C:/Users/kdwn/projects/propagate/propagate/lib/attitude/include/euler.h"

namespace propagate {
namespace frame {

using vector3 = attitude::vector<double, 3>;
using matrix3 = attitude::matrix<double, 3, 3>;
using euler_set = attitude::euler<double>;

// make orbits/frames a class?

// get_orbit_position_and_velcoity (function)
// Computes the position and velocity defined by orbit_radius and 
// orbit_angles (euler angle set) for a circular orbit.
//
void get_orbit_position_and_velocity(euler_set, double, vector3 *, vector3 *,
                                     double = 42828.3  // km^3/s^2
);

const double lmo_orbit_rate = 0.000884797;

void compute_lmo_dcm(double t, matrix3 *);

}  // namesapce frame
}  // namespcae propagate
#endif  // PROPAGATE_FRAME_H_