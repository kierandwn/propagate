#include "dynamics.h"

#include <cmath>
#include <vector>
#include <functional>

#include "../attitude/include/matrix.h"
#include "../attitude/include/mrp.h"
#include "../control/include/control.h"

#include "../config/include/config.h"
#include "../telemetry/include/log.h"

namespace propagate {

config::solver CFG_;
telemetry::log TL_("C:/Users/kdwn/projects/propagate/log/");

using matrix3 = attitude::matrix<double, 3, 3>;
using vector3 = attitude::vector<double, 3>;

using mrp_set = attitude::mrp<double>;


double I[3]{10., 5., 7.5};
attitude::matrix<double, 3, 3> inertia_tensor{
  I[0], 0., 0., 
  0., I[1], 0.,
  0., 0., I[2]
};

attitude::matrix<double, 6, 6> torques_to_accels{
  1., 0., 0., 0., 0., 0.,
  0., 1., 0., 0., 0., 0.,
  0., 0., 1., 0., 0., 0.,
  0., 0., 0., 1. / I[0], 0., 0.,
  0., 0., 0., 0., 1. / I[1], 0.,
  0., 0., 0., 0., 0., 1. / I[2]
};

const std::vector<std::string> channel_names{
  "time",
  "sigma_0", "sigma_1", "sigma_2",
  "omega_0", "omega_1", "omega_2"};

const std::vector<std::string> err_channel_names{"ref_sigma_0", "ref_sigma_1",
                                                 "ref_sigma_2", "ref_omega_0",
                                                 "ref_omega_1", "ref_omega_2"};

states switch_mrp(states x) {
  double mrp_norm_sq = pow(x[0], 2) + pow(x[1], 2) + pow(x[2], 2);

  if (mrp_norm_sq > 1.) {
    x[0] = -1. * x[0] / mrp_norm_sq;
    x[1] = -1. * x[1] / mrp_norm_sq;
    x[2] = -1. * x[2] / mrp_norm_sq;
  }
  return x;
}

void update_logger(telemetry::log * TL_, double t, states x) {
  x = switch_mrp(x);

  TL_->operator[](channel_names[0]) = t;
  for (int i = 0; i < 6; ++i) {
    TL_->operator[](channel_names[i + 1]) = x[i];
  }
  TL_->write_row();
}

rates plant_model(states x) {
  double sigma_sq = pow(x[0], 2.) + pow(x[1], 2.) + pow(x[2], 2.);
  return rates{
    (0.25 * (1. - sigma_sq) + 0.5 * pow(x[0], 2.)) * x[3] + //
                    (0.5 * (x[0] * x[1] - x[2])) * x[4] +
                    (0.5 * (x[0] * x[2] + x[1])) * x[5],
                    (0.5 * (x[0] * x[1] + x[2])) * x[3] + //
    (0.25 * (1. - sigma_sq) + 0.5 * pow(x[1], 2.)) * x[4] +
                    (0.5 * (x[1] * x[2] - x[0])) * x[5],
                    (0.5 * (x[0] * x[2] - x[1])) * x[3] + //
                    (0.5 * (x[1] * x[2] + x[0])) * x[4] +
    (0.25 * (1. - sigma_sq) + 0.5 * pow(x[2], 2.)) * x[5],
                    (I[1] - I[2]) * x[4] * x[5],  //
                    (I[2] - I[0]) * x[3] * x[5],  //
                    (I[0] - I[1]) * x[3] * x[4]   //
  };
}

rates B(inputs u) { return attitude::eye<double, 6>() * u; }

states f(states x, inputs u) {
  return torques_to_accels * (plant_model(x) + B(u));
}

inputs u_1Hz;
int control_update_count = 10;

states rk4(states x0, double t, double dt) 
{
  if (control_update_count == 10) {
    u_1Hz = control::simple_pd(x0, t, &TL_);
    control_update_count = 0;
  } else {
    control_update_count += 1;
  }

  states k1 = f(x0, u_1Hz);
  states k2 = f(x0 + (k1 * (dt / 2.)), u_1Hz);
  states k3 = f(x0 + (k2 * (dt / 2.)), u_1Hz);
  states k4 = f(x0 + (k3 * dt), u_1Hz);

  return x0 + (k1 + k2 * 2. + k3 * 2. + k4) * (dt / 6.);
}

states euler(states x0, double t, double dt) 
{
  if (control_update_count == 10.) {
    u_1Hz = control::simple_pd(x0, t, &TL_);
    control_update_count = 0;
  } else {
    control_update_count += 1;
  }

  inputs u = control::simple_pd(x0, t, &TL_);
  return x0 + f(x0, u) * dt;
}

std::function<states(states, double, double)> determine_propagator() {
  
  if (CFG_.INTEGRATION_TYPE == "rk4") {
    return rk4;
  } else if (CFG_.INTEGRATION_TYPE == "euler") {
    return euler;

  } else {
    return euler;
  }
}

states simulate()
{
  CFG_.initialise();

  TL_.ready();
  TL_.init(channel_names);
  
  states x = CFG_.INITIAL_STATES;
  double t = 0.;

  update_logger(&TL_, t, x);

  u_1Hz = control::simple_pd(x, 0., &TL_);

  std::function<states(states, double, double)> propagate_fcn = determine_propagator();

  double normd;
  double next_timecheck = 3400.;

  while ((t += CFG_.TIMESTEP) < CFG_.END_TIME) {
    // x = switch_mrp(x);
    x = propagate_fcn(x, t, CFG_.TIMESTEP);

    update_logger(&TL_, t, x);

    if (abs(t - next_timecheck) < (CFG_.TIMESTEP / 2.0)) {
      normd = sqrt(pow(x[0], 2) + pow(x[1], 2) + pow(x[2], 2));
      printf("norm(%.1fs): %.6f\n", t, normd);
      display(mrp_set(x[0], x[1], x[2]));

      if (next_timecheck == 4400.) {
        next_timecheck = 5600.;
      } else {
        next_timecheck += 1000.;
      }
    }
  }

  /*mrp_set sigma{x[0], x[1], x[2]};
  vector3 omega{x[3], x[4], x[5]};

  vector3 angular_momentum = inertia_tensor * omega;
  printf("H(B) = \n");
  display(angular_momentum);

  printf("H(N) = \n");
  display(sigma.matrix().transpose() * angular_momentum);

  double kinetic_energy =
      vector3(omega.transpose() * inertia_tensor).inner(omega) * 0.5;
  printf("T = %.8f\n", kinetic_energy);

  printf("sigma: \n");
  display(sigma);*/

  return x;
}


}  // namespace propagate