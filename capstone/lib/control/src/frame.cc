#include "frame.h"

#include "C:/Users/kdwn/projects/propagate/propagate/lib/attitude/include/matrix.h"
#include "C:/Users/kdwn/projects/propagate/propagate/lib/attitude/include/dcm.h"
#include "C:/Users/kdwn/projects/propagate/propagate/lib/attitude/include/euler.h"
#include "C:/Users/kdwn/projects/propagate/propagate/lib/attitude/include/mrp.h"

namespace propagate {
namespace frame {

using vector3 = attitude::vector<double, 3>;
using matrix3 = attitude::matrix<double, 3, 3>;

using euler_set = attitude::euler<double>;
using mrp_set = attitude::mrp<double>;

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

matrix3 compute_lmo_dcm(double t) {
  double dtheta = lmo_orbit_rate * t;

  euler_set angles = lmo_0;
  angles[2] += dtheta;

  return angles.matrix();
}

matrix3 compute_rnn_dcm(double t) {
  matrix3 RO = attitude::dcm::AXIS(2, 3.14159265358979323846);
  matrix3 ON = compute_lmo_dcm(t);
  return RO * ON;
}

vector3 compute_rnn_omega(double t) {
  matrix3 RN = compute_rnn_dcm(t);
  return RN.transpose() * vector3{0., 0., -1 * lmo_orbit_rate};
}


matrix3 compute_rsn_dcm() {
  return attitude::dcm::AXIS(1, 3.14159265358979323846 / 2.) *
         attitude::dcm::AXIS(3, 3.14159265358979323846);
}


matrix3 compute_comm_dcm(double t) {
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

  vector3 r1 = delta_r * (-1. / delta_r.norm());
  
  vector3 r2 = cross(delta_r, vector3{0., 0., 1.});
  r2 /= r2.norm();

  vector3 r3 = cross(r1, r2);
  r3 /= r3.norm();

  return matrix3{
    r1[0], r1[1], r1[2],
    r2[0], r2[1], r2[2],
    r3[0], r3[1], r3[2] 
  };
}

vector3 compute_comm_omega(double t, double dt) {
  matrix3 gmo_dcm_forward = compute_comm_dcm(330. + dt);
  matrix3 gmo_dcm_center = compute_comm_dcm(330.);
  matrix3 gmo_dcm_backward = compute_comm_dcm(330. - dt);

  matrix3 dgmo_dcm = (gmo_dcm_forward - gmo_dcm_backward) / (2. * dt);
  //matrix3 minus_omega_tilde = dgmo_dcm * gmo_dcm_center.transpose();
  matrix3 minus_omega_tilde = gmo_dcm_center.transpose() * dgmo_dcm;

  return vector3{
    minus_omega_tilde[1][2], 
    minus_omega_tilde[2][0],
    minus_omega_tilde[0][1]
  };
}

mrp_set compute_attitude_error(double t, int phase, mrp_set sigma) {
  matrix3 dcm_ref;

  switch (phase) {
    case 0: {  // sun-pointing mode
      dcm_ref = compute_rsn_dcm();

    } break;
    case 1: {  // nadir-pointing mode
      dcm_ref = compute_rnn_dcm(t);

    } break;
    case 2: {  // GMO-pointing mode
      dcm_ref = compute_comm_dcm(t);

    } break;
  }
  mrp_set sigma_err = sigma - dcm_ref;
  if (sigma_err.norm() > 1.) { sigma_err = sigma_err.shadow(); }

  return sigma_err;
}

vector3 compute_omega_error(double t, int phase, mrp_set sigma, vector3 omega) {
  vector3 omega_ref;

  switch (phase) {
    case 0: {  // sun-pointing mode
      omega_ref = vector3{0.};

    } break;
    case 1: {  // nadir-pointing mode
      omega_ref = sigma.matrix() * compute_rnn_omega(t);

    } break;
    case 2: {  // GMO-pointing mode
      omega_ref = sigma.matrix() * compute_comm_omega(t);

    } break;
  }
  return omega - omega_ref;
}

}  // namespace frame
}  // namespace propagate