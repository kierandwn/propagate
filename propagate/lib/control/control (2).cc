#include "control.h"

#include <cmath>

#include "C:/Users/kdwn/projects/propagate/propagate/lib/attitude/include/matrix.h"
#include "C:/Users/kdwn/projects/propagate/propagate/lib/attitude/include/mrp.h"


namespace propagate {
namespace control {

using namespace attitude;


using matrix3 = attitude::matrix<double, 3, 3>;
using vector3 = attitude::vector<double, 3>;

using states = attitude::vector<double, kStateDims>;
using inputs = attitude::vector<double, kStateDims>;


vector3 I{100., 75., 80.};

matrix3 B(vector3 sigma) {
  double mrp_norm_sq = pow(sigma[0], 2) + pow(sigma[1], 2) + pow(sigma[2], 2);
  return eye<double, 3>() * (1. - mrp_norm_sq) + tilde(sigma) * 2. + sigma.outer(sigma) * 2.;
}


matrix3 Bdot(vector3 sigma, vector3 sigma_dot) {
  double mrp_norm_sq = pow(sigma[0], 2) + pow(sigma[1], 2) + pow(sigma[2], 2);
  return attitude::eye<double, 3>() * -2. * sigma.inner(sigma_dot) + tilde(sigma_dot) * 2. +
      (sigma.outer(sigma_dot) + sigma_dot.outer(sigma)) * 2.;
}

vector3 omega(vector3 sigma, vector3 sigma_dot) {
  double sigma_norm_sq = pow(sigma[0], 2) + pow(sigma[1], 2) + pow(sigma[2], 2);
  return B(sigma).transpose() * sigma_dot * 4./ pow(1. + sigma_norm_sq, 2);
}

vector3 omega_dot(vector3 sigma, vector3 sigma_dot, vector3 sigma_dot_dot, vector3 omega) {
  return B(sigma).transpose() * (sigma_dot_dot * 4. - Bdot(sigma, sigma_dot) * omega);
}

matrix3 dcm_dot(vector3 omega, matrix3 dcm) { return tilde(omega) * dcm * -1; }


inputs control(states x, double t) {
  double attitude_proportional_gain = 5.;
  matrix3 omega_proportional_gain = attitude::eye<double, 3>() * 10.;

  double f = 0.05;
  vector3 sigma_ref {
    0.2 * sin(f * t), 
    0.3 * cos(f * t),
   -0.3 * sin(f * t)
  };  // for regulation problem

  vector3 sigma_dot_ref {
    0.2 * f * cos(f * t), 
   -0.3 * f * sin(f * t), 
   -0.3 * f * cos(f * t)
  };

  vector3 sigma_dot_dot_ref {
   -0.2 * f * f * sin(f * t), 
   -0.3 * f * f * cos(f * t), 
    0.3 * f * f * sin(f * t)
  };

  mrp<double> attitude_err = mrp<double>(x[0], x[1], x[2]) - 
    mrp<double>(sigma_ref[0], sigma_ref[1], sigma_ref[2]);
   
  matrix3 dcm = mrp<double>(x[0], x[1], x[2]).matrix(); // [BN]

  vector3 omega_ref = dcm * omega(sigma_ref, sigma_dot_ref); // body frame
  vector3 omega_dot_ref = dcm * omega_dot(sigma_ref, sigma_dot_ref, sigma_dot_dot_ref, omega_ref);

  vector3 omega_err{x[3] - omega_ref[0], x[4] - omega_ref[1],
                    x[5] - omega_ref[2]};

  vector3 modelled_torque{0.};

  vector3 attitude_tracking_control = vector3{
    attitude_err[0], attitude_err[1], attitude_err[2]
  } * (-1 * attitude_proportional_gain);
      //(attitude_proportional_gain * attitude_err) * -1.;

  vector3 rate_tracking_control = (omega_proportional_gain * omega_err) * -1.;

  vector3 feedforward_reference_tracking{
      I[0] * (omega_dot_ref[0] - x[4] * omega_ref[2] + x[5] * omega_ref[1]),
      I[1] * (omega_dot_ref[1] - x[5] * omega_ref[0] + x[3] * omega_ref[2]),
      I[2] * (omega_dot_ref[2] - x[3] * omega_ref[1] + x[4] * omega_ref[0])};

  vector3 gyroscopic_dynamics{(I[2] - I[1]) * x[4] * x[5],
                              (I[0] - I[2]) * x[3] * x[5],
                              (I[1] - I[0]) * x[3] * x[4]};

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



}  // namespace control
}  // namespace propagate