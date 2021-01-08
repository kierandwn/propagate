#include "frame.h"

#include "C:/Users/kdwn/projects/propagate/propagate/lib/attitude/include/euler.h"
#include "C:/Users/kdwn/projects/propagate/propagate/lib/attitude/include/matrix.h"
#include "C:/Users/kdwn/projects/propagate/propagate/lib/attitude/include/dcm.h"

namespace propagate {
namespace frame {

using vector3 = attitude::vector<double, 3>;
using matrix3 = attitude::matrix<double, 3, 3>;
using euler_set = attitude::euler<double>;

double mu_mars = 42828.3;

// intial conditions
euler_set lmo_0 = euler_set(20., 30., 60., 313) * (3.14159265358979323846 / 180.);
euler_set gmo_0 = euler_set(0., 0., 250., 313) * (3.14159265358979323846 / 180.);

double lmo_radius = 3396.19 + 400;
double gmo_radius = 20424.2;

double lmo_rate = sqrt(mu_mars / pow(lmo_radius, 3));
double gmo_rate = sqrt(mu_mars / pow(gmo_radius, 3));

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
    double orbital_rate
) 
{
  vector3 position_result =
      orbit_angles.matrix().transpose() * vector3{orbit_radius, 0., 0.};

  vector3 velocity_result = orbit_angles.matrix().transpose() *
      vector3{0., orbit_radius * orbital_rate, 0.};

  for (int i = 0; i < 3; ++i) { 
    orbit_position->operator[](i) = position_result[i];
    orbit_velocity->operator[](i) = velocity_result[i]; 
  }
}

void compute_lmo_dcm(double t, matrix3 * dcm) {
  double dtheta = lmo_orbit_rate * t;

  euler_set angles = euler_set(20., 30., 60., 313);
  angles *= 3.14159265358979323846 / 180.;
  angles[2] += dtheta;

  matrix3 result = angles.matrix();

  for (int i = 0; i < 9; ++i) { dcm->operator()(i) = result(i); }
}

void compute_rnn_dcm(double t, matrix3 * dcm) {
  matrix3 RO = attitude::dcm::AXIS(2, 3.14159265358979323846);

  matrix3 ON;
  compute_lmo_dcm(t, &ON);

  matrix3 result = RO * ON;
  for (int i = 0; i < 9; ++i) { dcm->operator()(i) = result(i); }
}

void compute_rnn_omega(double t, vector3 * omega) {
  matrix3 RN;
  compute_rnn_dcm(t, &RN);

  vector3 result = RN.transpose() * vector3{0., 0., -1 * lmo_orbit_rate};
  for (int i = 0; i < 3; ++i) { omega->operator[](i) = result[i]; }
}


void compute_rsn_dcm(matrix3 * dcm) {
  matrix3 result = attitude::dcm::AXIS(1, 3.14159265358979323846 / 2.) *
                   attitude::dcm::AXIS(3, 3.14159265358979323846);

  for (int i = 0; i < 9; ++i) { dcm->operator()(i) = result(i); }
}



matrix3 compute_gmo_dcm(double t) {
  // compute delta_r
  vector3 lmo_position;
  vector3 lmo_velocity;

  euler_set lmo_angles = lmo_0;
  lmo_angles[2] += lmo_rate * t;

  get_orbit_position_and_velocity(lmo_angles, lmo_radius, &lmo_position,
                                  &lmo_velocity, lmo_rate);

  vector3 gmo_position;
  vector3 gmo_velocity;

  euler_set gmo_angles = gmo_0;
  gmo_angles[2] += gmo_rate * t;

  get_orbit_position_and_velocity(gmo_angles, gmo_radius, &gmo_position,
                                  &gmo_velocity, gmo_rate);

  vector3 delta_r = gmo_position - lmo_position;

  return matrix3{
    delta_r[0], delta_r[1], delta_r[2], 
    delta_r[1], -1 * delta_r[0], 0.,
    -1 * delta_r[0] * delta_r[2],
        delta_r[1] * delta_r[2],
        -1 * (pow(delta_r[0], 2) + pow(delta_r[1], 2))
  };
}

}  // namespace frame
}  // namespace propagate