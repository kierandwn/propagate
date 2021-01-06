#include "dynamics.h"

#define _USE_MATH_DEFINES  
#include <cmath>
#include <vector>

// #include "C:/Users/kdwn/projects/propagate/propagate/lib/control/include/control.h"
#include "C:/Users/kdwn/projects/propagate/propagate/lib/attitude/include/matrix.h"
#include "C:/Users/kdwn/projects/propagate/propagate/lib/telemetry/include/log.h"

namespace propagate {

const double kPi = 3.14159265358979323846;

using matrix3 = attitude::matrix<double, 3, 3>;
using vector3 = attitude::vector<double, 3>;

//states x0{0.1,
//          0.2,
//          -0.1,
//          30.0 * (kPi / 180.),
//          10.0 * (kPi / 180.),
//          -20.0 * (kPi / 180.)};

states x0{
  0.0,
  0.0,
  0.0,
  1.,
  1.,
  0.
};

double I[3]{100., 75., 80.};
attitude::matrix<double, 3, 3> inertia_tensor{
  I[0], 0., 0., 
  0., I[1], 0.,
  0., 0., I[2]
};


matrix3 outer(vector3 lhs, vector3 rhs) { 
  return matrix3{
    lhs[0] * rhs[0], lhs[0] * rhs[1], lhs[0] * rhs[2],
    lhs[1] * rhs[0], lhs[1] * rhs[1], lhs[1] * rhs[2],
    lhs[2] * rhs[0], lhs[2] * rhs[1], lhs[2] * rhs[2],
  };
}

matrix3 tilde_here(vector3 v) { 
  return matrix3{
    0., -v[2], v[1],
    v[2], 0., -v[0], 
    -v[1], v[0], 0.
  }; 
}

double P = 3.;
double K = 0.11;

inputs control(states x, double t){
  vector3 omega{x[3], x[4], x[5]};
  vector3 sigma{x[0], x[1], x[2]};

  double sigma_norm_sq =
      pow(sigma[0], 2.) + pow(sigma[1], 2.) + pow(sigma[2], 2.);
  double omega_norm_sq =
      pow(omega[0], 2.) + pow(omega[1], 2.) + pow(omega[2], 2.);

  matrix3 cr = tilde_here(omega);
  vector3 H = inertia_tensor * omega;

  vector3 control_torques = cr * H + 
    inertia_tensor * 
    (
      omega * (-1 * P) -
      (
        outer(omega, omega) + 
        attitude::eye<double, 3>() * (((4. * K)/(1. + sigma_norm_sq)) - (omega_norm_sq/2.))
      ) * sigma
    );

  for (int i = 0; i < 3; ++i) {
    if (isnan(control_torques[i])) {
      printf("reached.\n");
    }
  }

  return inputs{
    0.,
    0.,
    0., 
    control_torques[0], 
    control_torques[1],
    control_torques[2]
  };
} 

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
  double sigma_sq = pow(x[0], 2.) + pow(x[1], 2.) + pow(x[2], 2.);

  rates xdot{
    (0.25 * (1. - sigma_sq) + 0.5 * pow(x[0], 2.)) * x[3] + //
                    (2. * (x[0] * x[1] - x[2])) * x[4] +
                    (2. * (x[0] * x[2] + x[1])) * x[5],
                    (2. * (x[0] * x[1] + x[2])) * x[3] + //
    (0.25 * (1. - sigma_sq) + 0.5 * pow(x[1], 2.)) * x[4] +
                    (2. * (x[1] * x[2] - x[0])) * x[5],
                    (2. * (x[0] * x[2] - x[1])) * x[3] + //
                    (2. * (x[1] * x[2] + x[0])) * x[4] +
    (0.25 * (1. - sigma_sq) + 0.5 * pow(x[2], 2.)) * x[5],
                   ((I[1] - I[2]) / I[0]) * x[4] * x[5],  //
                   ((I[2] - I[0]) / I[1]) * x[3] * x[5],  //
                   ((I[0] - I[1]) / I[2]) * x[3] * x[4]   //
  };

  for (int i = 0; i < 3; ++i) {
    if (isnan(xdot[i])) {
      printf("reached.\n");
    }
  }
  return xdot;
}

rates B(inputs u) { return attitude::eye<double, 6>() * u; }

states f(states x, inputs u) {
  return plant_model(x) + B(u);
}

states rk4(states x0, double t, double dt) {
  //inputs u = control::control(x0, t);
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

  double simulation_time = 30.;
  double normd;

  update_logger(&tl, x);

  while ((t += dt) < simulation_time) {
    x = rk4(x, t, dt);
    x = switch_mrp(x);

    update_logger(&tl, x);
    
    normd = sqrt(pow(x[3], 2) + pow(x[4], 2) + pow(x[5], 2));
    if (abs(t - 50.) < dt / 2.) {
      normd = sqrt(pow(x[0], 2) + pow(x[1], 2) + pow(x[2], 2));
      printf("norm(%.1fs): %.6f\n", t, normd);
    }
     // printf("|omega| = %.10f\n", normd);
  }
  return x;
}


}  // namespace propagate