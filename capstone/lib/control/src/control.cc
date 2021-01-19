#include "control.h"

#include <cmath>

#include "C:/Users/kdwn/projects/propagate/propagate/lib/control/include/frame.h"
#include "C:/Users/kdwn/projects/propagate/propagate/lib/telemetry/include/log.h"

#include "C:/Users/kdwn/projects/propagate/propagate/lib/attitude/include/matrix.h"
#include "C:/Users/kdwn/projects/propagate/propagate/lib/attitude/include/mrp.h"
#include "C:/Users/kdwn/projects/propagate/propagate/lib/attitude/include/euler.h"


namespace propagate {
namespace control {

using namespace attitude;


using matrix3 = attitude::matrix<double, 3, 3>;
using vector3 = attitude::vector<double, 3>;

using states = attitude::vector<double, kStateDims>;
using inputs = attitude::vector<double, kStateDims>;

using mrp = attitude::mrp<double>;
using euler = attitude::euler<double>;

double deg2rad = 3.14159265358979323846 / 180.;
double mu_mars = 42828.3;


// intial conditions (orbit)
euler lmo_0 = euler(20., 30., 60., 313) * deg2rad;
euler gmo_0 = euler(0., 0., 250., 313) * deg2rad;

double lmo_radius = 3396.19 + 400.;
double gmo_radius = 20424.2;

double lmo_rate = sqrt(mu_mars / pow(lmo_radius, 3));
double gmo_rate = sqrt(mu_mars / pow(gmo_radius, 3));

vector3 lmo_position;
vector3 gmo_position;

vector3 lmo_velocity;
vector3 gmo_velocity;

double GAIN_P = 1. / 6.;
double GAIN_K = pow(GAIN_P, 2) / 5.;

const std::vector<std::string> ref_channel_names{"ref_sigma_0", "ref_sigma_1",
                                                 "ref_sigma_2", "ref_omega_0",
                                                 "ref_omega_1", "ref_omega_2"};

void update_logger_err(telemetry::log * tl, mrp sigma_ref, vector3 omega_ref) {
  for (int i = 0; i < 3; ++i) {
    tl->operator[](ref_channel_names[i]) = sigma_ref[i];
    tl->operator[](ref_channel_names[i + 3]) = omega_ref[i];
  }
}

inputs control(states x, double t, telemetry::log * tl) 
{ 
  mrp sigma( x[0], x[1], x[2] );
  vector3 omega{ x[3], x[4], x[5] };

  euler lmo_angles = lmo_0;
  euler gmo_angles = gmo_0;

  lmo_angles[2] += lmo_rate * t;
  gmo_angles[2] += gmo_rate * t;

  frame::get_orbit_position_and_velocity(
    lmo_angles, 
    lmo_radius,
    &lmo_position,
    &lmo_velocity, 
    lmo_rate
  );

  frame::get_orbit_position_and_velocity(
    gmo_angles, 
    gmo_radius,
    &gmo_position,
    &gmo_velocity, 
    gmo_rate
  );

  double angle = acos(lmo_position.inner(gmo_position) /
                     (lmo_position.norm() * gmo_position.norm()));
  int mission_phase;

  if (lmo_position[1] > 0.) {
    mission_phase = 0; // sun-pointing (recharge)
    //printf("sun->\n");
  } else if ((abs(angle) < (35. * deg2rad))) {
    mission_phase = 2; // GMO pointing (comm.)
    //printf("GMO->\n");
  } else {
    mission_phase = 1; // nadir-pointing (science)
    //printf("nadir->\n");
  }

  mrp sigma_err = frame::compute_attitude_error(t, 
    mission_phase,
    sigma
  );

  vector3 omega_err = frame::compute_omega_error(t, 
    mission_phase, 
    sigma,
    omega
  );

  return inputs{
    0.,
    0., 
    0., 
    -1. * (GAIN_K * sigma_err[0] + GAIN_P * omega_err[0]),
    -1. * (GAIN_K * sigma_err[1] + GAIN_P * omega_err[1]),
    -1. * (GAIN_K * sigma_err[2] + GAIN_P * omega_err[2])
  };
}



}  // namespace control
}  // namespace propagate
