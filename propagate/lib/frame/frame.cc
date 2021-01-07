#include "frame.h"

#include "C:/Users/kdwn/projects/propagate/propagate/lib/attitude/include/euler.h"
#include "C:/Users/kdwn/projects/propagate/propagate/lib/attitude/include/matrix.h"

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
void get_orbit_position_and_velocity(
    euler_set orbit_angles, 
    double orbit_radius, 
    vector3 * orbit_position,
    vector3 * orbit_velocity,
    double gravitational_constant
) {
  vector3 position_result =
      orbit_angles.matrix().transpose() * vector3{orbit_radius, 0., 0.};

  double orbital_rate = sqrt(gravitational_constant / pow(orbit_radius, 3));
  vector3 velocity_result = orbit_angles.matrix().transpose() *
      vector3{0., orbit_radius * orbital_rate, 0.};

  for (int i = 0; i < 3; ++i) { 
    orbit_position->operator[](i) = position_result[i];
    orbit_velocity->operator[](i) = velocity_result[i]; 
  }
}

void compute_lmo_dcm(double t, matrix3* dcm) {
  double dtheta = lmo_orbit_rate * t;

  euler_set angles = euler_set(20., 30., 60., 313);
  angles *= 3.14159265358979323846 / 180.;
  angles[2] += dtheta;

  matrix3 result = angles.matrix();

  for (int i = 0; i < 9; ++i) {
    dcm->operator()(i) = result(i);
  }
}

}  // namespace frame
}  // namespace propagate