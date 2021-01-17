#include "dynamics.h"

#include <cmath>
#include <vector>

#include "C:/Users/kdwn/projects/propagate/propagate/lib/control/include/control.h"

#include "C:/Users/kdwn/projects/propagate/propagate/lib/attitude/include/matrix.h"
#include "C:/Users/kdwn/projects/propagate/propagate/lib/attitude/include/mrp.h"
#include "C:/Users/kdwn/projects/propagate/propagate/lib/telemetry/include/log.h"

namespace propagate {

telemetry::log tl("C:/Users/kdwn/projects/propagate/log/");

using matrix3 = attitude::matrix<double, 3, 3>;
using vector3 = attitude::vector<double, 3>;

using mrp_set = attitude::mrp<double>;


double I[3]{100., 75., 80.};
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

void update_logger(telemetry::log * tl, double t, states x) {
  x = switch_mrp(x);

  tl->operator[](channel_names[0]) = t;
  for (int i = 0; i < 6; ++i) {
    tl->operator[](channel_names[i + 1]) = x[i];
  }
  tl->write_row();
}

//vector3 external_torques = {0.5, -0.3, 0.2};
vector3 external_torques = {0., 0., 0.};

rates plant_model(states x) {
  // x = switch_mrp(x);
  double sigma_sq = pow(x[0], 2.) + pow(x[1], 2.) + pow(x[2], 2.);

  rates xdot{
    (0.25 * (1. - sigma_sq) + 0.5 * pow(x[0], 2.)) * x[3] + //
                    (0.5 * (x[0] * x[1] - x[2])) * x[4] +
                    (0.5 * (x[0] * x[2] + x[1])) * x[5],
                    (0.5 * (x[0] * x[1] + x[2])) * x[3] + //
    (0.25 * (1. - sigma_sq) + 0.5 * pow(x[1], 2.)) * x[4] +
                    (0.5 * (x[1] * x[2] - x[0])) * x[5],
                    (0.5 * (x[0] * x[2] - x[1])) * x[3] + //
                    (0.5 * (x[1] * x[2] + x[0])) * x[4] +
    (0.25 * (1. - sigma_sq) + 0.5 * pow(x[2], 2.)) * x[5],
                    ((I[1] - I[2]) * x[4] * x[5]) + external_torques[0],  //
                    ((I[2] - I[0]) * x[3] * x[5]) + external_torques[1],  //
                    ((I[0] - I[1]) * x[3] * x[4]) + external_torques[2]   //
  };
  return xdot;
}

rates B(inputs u) { return attitude::eye<double, 6>() * u; }

states f(states x, inputs u) {
  return torques_to_accels * (plant_model(x) + B(u));
}

inputs u_1Hz;
int control_update_count = 0;

states rk4(states x0, double t, double dt) 
{
  if (control_update_count == 10) {
    u_1Hz = control::control(x0, t, dt, &tl);
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

states first_order_propagator(states x0, double t, double dt) 
{
  inputs u = control::control(x0, t, dt, &tl);
  return x0 + f(x0, u) * dt;
}

states simulate(states x0, double tf, double dt)
{
  tl.ready();
  tl.init(channel_names);
  
  states x = x0;
  double t = 0.;

  update_logger(&tl, t, x);

  u_1Hz = control::control(x0, 0., dt, &tl);

  double normd;
  double next_timecheck = 30.;

  while ((t += dt) < tf) {
    x = first_order_propagator(x, t, dt);
    //x = switch_mrp(x);

    update_logger(&tl, t, x);

    if (abs(t - next_timecheck) < (dt / 2.0)) {
      normd = sqrt(pow(x[0], 2) + pow(x[1], 2) + pow(x[2], 2));
      printf("norm(%.1fs): %.6f\n", t, normd);
      //display(mrp_set(x[0], x[1], x[2]));

      next_timecheck += 100.;
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
