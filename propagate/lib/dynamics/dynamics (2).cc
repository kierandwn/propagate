#include "dynamics.h"

#include <cmath>
#include <vector>

#include "C:/Users/kdwn/projects/propagate/propagate/lib/attitude/include/matrix.h"
#include "C:/Users/kdwn/projects/propagate/propagate/lib/telemetry/include/log.h"

namespace propagate {

const double kPi = 3.1415926536;

using matrix3 = attitude::matrix<double, 3, 3>;
using vector3 = attitude::vector<double, 3>;

states x0{0.1,
          0.2,
          -0.1,
          30.0 * (kPi / 180.),
          10.0 * (kPi / 180.),
          -20.0 * (kPi / 180.)};

vector3 I{100., 75., 80.};

states switch_mrp(states x) {
  double mrp_norm_sq = pow(x[0], 2) + pow(x[1], 2) + pow(x[2], 2);

  if (mrp_norm_sq > 1.) {
    x[0] = -1. * x[0] / mrp_norm_sq;
    x[1] = -1. * x[1] / mrp_norm_sq;
    x[2] = -1. * x[2] / mrp_norm_sq;
  }
  return x;
}

rates plant_model(states x) {
  double sigma_sq = pow(x[0], 2) + pow(x[1], 2) + pow(x[2], 2);

  return rates{
    (0.25 * (1. - sigma_sq + 2. * pow(x[0], 2))) * x[3] + //
                    (0.5 * (x[0] * x[1] - x[2])) * x[4] +
                    (0.5 * (x[0] * x[2] + x[1])) * x[5],
                    (0.5 * (x[0] * x[1] + x[2])) * x[3] + //
    (0.25 * (1. - sigma_sq + 2. * pow(x[1], 2))) * x[4] +
                    (0.5 * (x[1] * x[2] - x[0])) * x[5],
                    (0.5 * (x[0] * x[2] - x[1])) * x[3] + //
                    (0.5 * (x[1] * x[2] + x[0])) * x[4] +
    (0.25 * (1. - sigma_sq + 2. * pow(x[2], 2))) * x[5],
                   ((I[1] - I[2]) / I[0]) * x[4] * x[5],  //
                   ((I[2] - I[0]) / I[1]) * x[3] * x[5],  //
                   ((I[0] - I[1]) / I[2]) * x[3] * x[4]   //
  };
}

rates B(inputs u) { return attitude::eye<double, 6>() * u; }

states control(states x, double t) {
  matrix3 attitude_proportional_gain = attitude::eye<double, 3>() * 5.;
  matrix3 omega_proportional_gain = attitude::eye<double, 3>() * 10.;

  double f = 0.05;
  vector3 attitude_ref {
     0.2 * sin(f * t),
     0.3 * cos(f * t), 
    -0.3 * sin(f * t)
  }; // for regulation problem

  double mrp_norm_sq = pow(x[0], 2) + pow(x[1], 2) + pow(x[2], 2);

  vector3 omega_ref = (matrix3{
    1. - mrp_norm_sq + 2 * pow(x[0], 2), 
    2. * (x[0] * x[1] + x[2]),
    2. * (x[0] * x[2] - x[1]), 
    2. * (x[0] * x[1] - x[2]),
    1. - mrp_norm_sq + 2 * pow(x[1], 2),
    2. * (x[1] * x[2] + x[0]),
    2. * (x[0] * x[2] + x[1]),
    2. * (x[1] * x[2] - x[0]),
    1. - mrp_norm_sq + 2 * pow(x[2], 2)} * (4. / pow(1. + mrp_norm_sq, 2))) *
  vector3{
     0.2 * f * cos(f * t),
    -0.3 * f * sin(f * t),
    -0.3 * f * cos(f * t)
  };
  
  vector3 omega_dot_ref{0.};

  vector3 attitude_err{
    x[0] - attitude_ref[0], 
    x[1] - attitude_ref[1], 
    x[2] - attitude_ref[2]
  };
  vector3 omega_err{
    x[3] - omega_ref[0], 
    x[4] - omega_ref[1], 
    x[5] - omega_ref[2]
  };

  vector3 modelled_torque{0.};

  vector3 attitude_tracking_control = (attitude_proportional_gain * attitude_err) * -1.;
  vector3 rate_tracking_control = (omega_proportional_gain * omega_err) * -1.;

  vector3 feedforward_reference_tracking{
      I[0] * (omega_dot_ref[0] - x[4] * omega_ref[2] + x[5] * omega_ref[1]),
      I[1] * (omega_dot_ref[1] - x[5] * omega_ref[0] + x[3] * omega_ref[2]),
      I[2] * (omega_dot_ref[2] - x[3] * omega_ref[1] + x[4] * omega_ref[0])};

  vector3 gyroscopic_dynamics{
    (I[2] - I[1]) * x[4] * x[5],
    (I[0] - I[2]) * x[3] * x[5],
    (I[1] - I[0]) * x[3] * x[4]
  };

  vector3 control_torques = attitude_tracking_control + rate_tracking_control +
                            feedforward_reference_tracking +
                            gyroscopic_dynamics - modelled_torque;

  return states{
    0.,
    0.,
    0.,
    control_torques[0] / I[0],
    control_torques[1] / I[1],
    control_torques[2] / I[2]
  };
}

attitude::matrix<double, 6, 6> torques_to_accels{
    1., 0., 0., 0., 0., 0., 
    0., 1., 0., 0., 0., 0.,
    0., 0., 1., 0., 0., 0., 
    0., 0., 0., 1. / I[0], 0., 0.,
    0., 0., 0., 0., 1. / I[1], 0., 
    0., 0., 0., 0., 0., 1. / I[2]
};

states f(states x, inputs u) {
  return plant_model(x) + B(u);
}

states rk4(states x0, double t, double dt) {
  inputs u = control(x0, t);

  states k1 = f(x0, u);
  states k2 = f(x0 + k1 * (dt / 2.), u);
  states k3 = f(x0 + k2 * (dt / 2.), u);
  states k4 = f(x0 + k3 * dt, u);

  return x0 + (k1 + k2 * 2. + k3 * 2. + k4) * (dt / 6.);
}

const std::vector<std::string> channel_names{"sigma_0", "sigma_1", "sigma_2",
                                             "omega_0", "omega_1", "omega_2"};

void update_logger(telemetry::log * tl, states x) {
  for (int i = 0; i < 6; ++i) {
    tl->operator[](channel_names[i]) = x[i];
  }
  tl->write_row();
}

states simulate(double dt) {
  telemetry::log tl("C:/Users/kdwn/projects/propagate/log/");
  tl.ready();
  tl.init(channel_names);
  
  states x = x0;
  double t = 0.;

  double simulation_time = 120.;
  double normd;

  update_logger(&tl, x);

  while ((t += dt) < simulation_time) {
    x = rk4(x, t, dt);
    x = switch_mrp(x);

    update_logger(&tl, x);
    
    normd = sqrt(pow(x[3], 2) + pow(x[4], 2) + pow(x[5], 2));
    if (t == 30.) {
      normd = sqrt(pow(x[0], 2) + pow(x[1], 2) + pow(x[2], 2));
      printf("norm(%.1fs): %.6f", t, normd);
    }
    // printf("|omega| = %.10f\n", normd);
  }
  return x;
}


}  // namespace propagate