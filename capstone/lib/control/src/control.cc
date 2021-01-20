#include "control.h"
#include "frame.h"

#include <cmath>

#include "../../telemetry/include/log.h"
#include "../../attitude/include/matrix.h"
#include "../../attitude/include/mrp.h"
#include "../../attitude/include/euler.h"

// --- SPACECRAFT DYNAMICS AND CONTROL SPECIALISATION: CAPSTONE MISSION ---
//            .       .                   .       .      .     .      .
//           .    .         .    .            .     ______
//       .           .             .               ////////
//                 .    .   ________   .  .      /////////     .    .
//            .            |.____.  /\        ./////////    .
//     .                 .//      \/  |\     /////////
//        .       .    .//          \ |  \ /////////       .     .   .
//                     ||.    .    .| |  ///////// .     .
//      .    .         ||           | |//`,/////                .
//              .       \\        ./ //  /  \/   .
//   .                    \\.___./ //\` '   ,_\     .     .
//           .           .     \ //////\ , /   \                 .    .
//                        .    ///////// \|  '  |    .
//       .        .          ///////// .   \ _ /          .
//                         /////////                              .
//                  .   ./////////     .     .
//          .           --------   .                  ..             .
//   .               .        .         .                       .
//                         ________________________
// ____________------------                        -------------_________
// 

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
double GAIN_INTEGRAL = 0.;

matrix3 inertia_tensor{
  100.,  0.,  0.,
  0.,   75.,  0., 
  0.,    0., 80.
};

const std::vector<std::string> ref_channel_names{"ref_sigma_0", "ref_sigma_1",
                                                 "ref_sigma_2", "ref_omega_0",
                                                 "ref_omega_1", "ref_omega_2"};

void update_logger_err(telemetry::log * tl, mrp sigma_ref, vector3 omega_ref) {
  for (int i = 0; i < 3; ++i) {
    tl->operator[](ref_channel_names[i]) = sigma_ref[i];
    tl->operator[](ref_channel_names[i + 3]) = omega_ref[i];
  }
}

matrix3 B(vector3 sigma) {
  return attitude::eye<double, 3>() * (1. - pow(sigma.norm(), 2)) +
         tilde(sigma) * 2. + sigma.outer(sigma) * 2.;
}

matrix3 B_dot_transposed(vector3 sigma, vector3 sigma_dot) {
  return attitude::eye<double, 3>() * (sigma.inner(sigma_dot) * -2.) +
         (sigma_dot.outer(sigma) + sigma.outer(sigma_dot)) * 2.;
}

vector3 omega_from_mrp(vector3 sigma, vector3 sigma_dot) {
  double denom = pow(1 + pow(sigma.norm(), 2), 2);
  return B(sigma).transpose() * sigma_dot * (4. / denom);
}

double f = 0.05;
vector3 omega_dot_from_mrp(double t, double dt) {
  vector3 sigma_forward{0.2 * sin(f * (t + dt)), 0.3 * cos(f * (t + dt)),
                        -0.3 * sin(f * (t + dt))};

  vector3 sigma_dot_forward{0.2 * f * cos(f * (t + dt)),
                            -0.3 * f * sin(f * (t + dt)),
                            -0.3 * f * cos(f * (t + dt))};

  vector3 sigma_backward{0.2 * sin(f * (t + dt)), 0.3 * cos(f * (t + dt)),
                         -0.3 * sin(f * (t + dt))};

  vector3 sigma_dot_backward{0.2 * f * cos(f * (t - dt)),
                             -0.3 * f * sin(f * (t - dt)),
                             -0.3 * f * cos(f * (t - dt))};

  vector3 omega_forward = omega_from_mrp(sigma_forward, sigma_dot_forward);
  vector3 omega_backward = omega_from_mrp(sigma_backward, sigma_dot_backward);
  return (omega_forward - omega_backward) / (2. * dt);
}

vector3 omega_err0;
vector3 integral_sigma_err{0.};

void compute_ref_state_sinusoid(double t, 
  mrp sigma, 
  vector3 omega,
  mrp& sigma_ref, 
  vector3& omega_ref
) {
  vector3 sigma_vector{
     0.2 * sin(f * t), 
     0.3 * cos(f * t), 
    -0.3 * sin(f * t)
  };

  vector3 sigma_dot_ref{0.2 * f * cos(f * t), -0.3 * f * sin(f * t),
                        -0.3 * f * cos(f * t)};

  vector3 sigma_dot_dot_ref{-0.2 * f * f * sin(f * t),
                            -0.3 * f * f * cos(f * t),
                            0.3 * f * f * sin(f * t)};

  
  sigma_ref[0] = sigma_vector[0];
  sigma_ref[1] = sigma_vector[1];
  sigma_ref[2] = sigma_vector[2];

  omega_ref = omega_from_mrp(sigma_vector, sigma_dot_ref);  // body frame
}

void compute_ref_state_sinusoid(double t, 
  mrp sigma, 
  vector3 omega,
  mrp& sigma_ref, 
  vector3& omega_ref,
  vector3& omega_dot_ref
) {
  compute_ref_state_sinusoid(t, sigma, omega, sigma_ref, omega_ref);
  omega_dot_ref = omega_dot_from_mrp(t, 0.01);
}

void compute_ref_state_orbit(double t, 
  mrp sigma, 
  vector3 omega,
  mrp& sigma_err,
  vector3& omega_err
) {
  euler lmo_angles = lmo_0;
  euler gmo_angles = gmo_0;

  lmo_angles[2] += lmo_rate * t;
  gmo_angles[2] += gmo_rate * t;

  frame::get_orbit_position_and_velocity(lmo_angles, lmo_radius, &lmo_position,
                                         &lmo_velocity, lmo_rate);

  frame::get_orbit_position_and_velocity(gmo_angles, gmo_radius, &gmo_position,
                                         &gmo_velocity, gmo_rate);

  double angle = acos(lmo_position.inner(gmo_position) /
                      (lmo_position.norm() * gmo_position.norm()));
  int mission_phase;

  if (lmo_position[1] > 0.) {
    mission_phase = 0;  // sun-pointing (recharge)

  } else if ((abs(angle) < (35. * deg2rad))) {
    mission_phase = 2;  // GMO pointing (comm.)

  } else {
    mission_phase = 1;  // nadir-pointing (science)
  }

  sigma_err = frame::compute_attitude_error(t, mission_phase, sigma);
  omega_err = frame::compute_omega_error(t, mission_phase, sigma, omega);
}

inputs simple_pd(states x, double t, telemetry::log * tl) 
{ 
  mrp sigma( x[0], x[1], x[2] );
  vector3 omega{ x[3], x[4], x[5] };

  mrp sigma_err;
  vector3 omega_err;

  compute_ref_state_orbit(t, sigma, omega, sigma_err, omega_err);

  return inputs{
    0.,
    0., 
    0., 
    -1. * (GAIN_K * sigma_err[0] + GAIN_P * omega_err[0]),
    -1. * (GAIN_K * sigma_err[1] + GAIN_P * omega_err[1]),
    -1. * (GAIN_K * sigma_err[2] + GAIN_P * omega_err[2])
  };
}

inputs test_out_controller(states x, double t, double dt, telemetry::log* tl) 
{
  mrp sigma(x[0], x[1], x[2]);
  vector3 omega{x[3], x[4], x[5]};

  mrp sigma_ref;
  vector3 omega_ref;
  vector3 omega_dot_ref;

  compute_ref_state_sinusoid(t, sigma, omega, sigma_ref, omega_ref, omega_dot_ref);

  // Ensure sigma desribes short rotation
  mrp sigma_err = sigma - sigma_ref;
  if (sigma_err.norm() > 1.) { sigma_err = sigma_err.shadow(); }

  // Transform to body frame
  vector3 omega_ref_body_frame(sigma_err.matrix() * omega_ref);
  vector3 omega_dot_ref_body_frame(sigma_err.matrix() * omega_dot_ref);

  vector3 omega_err = omega - omega_ref_body_frame;
  if (t == 0.) { omega_err0 = omega_err; }

  vector3 attitude_control =
      vector3{sigma_err[0], sigma_err[1], sigma_err[2]} * GAIN_K;
  vector3 rate_control = omega_err * GAIN_P;

  vector3 integral_control = (integral_sigma_err * GAIN_K +
                              inertia_tensor * (omega_err - omega_err0)) *
                             GAIN_INTEGRAL * GAIN_P;

  vector3 gyroscopics = tilde(omega) * inertia_tensor * omega;

  vector3 feedforward_compensation =
      inertia_tensor * (omega_dot_ref_body_frame - cross(omega, omega_ref));

  vector3 external_torques{0., 0., 0.};

  vector3 control_torques = 
    attitude_control * -1. + 
    rate_control * -1. +
    gyroscopics +
    integral_control * -1. + 
    feedforward_compensation + 
    external_torques * -1;

  /*double u_max = 1.;
  for (int i = 0; i < 3; ++i) {
    if (abs(control_torques[i]) > u_max) {
      control_torques[i] =
          u_max * (control_torques[i] / abs(control_torques[i]));
    }
  }*/

  integral_sigma_err += vector3{sigma_err[0], sigma_err[1], sigma_err[2]} * dt;

  return inputs{
      0., 0., 0., control_torques[0], control_torques[1], control_torques[2]};
}

inputs closedloop_linear(states x, double t, double dt, telemetry::log* tl) 
{
  mrp sigma(x[0], x[1], x[2]);
  vector3 omega{x[3], x[4], x[5]};

  mrp sigma_ref;
  vector3 omega_ref;

  compute_ref_state_sinusoid(t, sigma, omega, sigma_ref, omega_ref);


  mrp sigma_err = sigma - sigma_ref;
  if (sigma_err.norm() > 1.) { sigma_err = sigma_err.shadow(); }

  vector3 omega_ref_body_frame(sigma_err.matrix() * omega_ref);
  vector3 omega_err = omega - omega_ref_body_frame;

  vector3 attitude_control =
      vector3{sigma_err[0], sigma_err[1], sigma_err[2]} * GAIN_K;
  vector3 rate_control = omega_err * GAIN_P;


  vector3 gyroscopics = tilde(omega) * inertia_tensor * omega;


  // vector3 external_torques = vector3{0.5, -0.3, 0.2};
  vector3 external_torques{0., 0., 0.};

  double norm_sigma = sigma_err.norm();
  double norm_omega = omega_err.norm();

  vector3 control_torques = gyroscopics -
    inertia_tensor * GAIN_P * omega_err -
    inertia_tensor * (
      omega_err.outer(omega_err) +
      attitude::eye<double, 3>() * ((4. * GAIN_K) / (1. + pow(norm_sigma, 2)) -
  pow(norm_omega, 2) / 2.) ) * vector3{sigma_err[0], sigma_err[1],
  sigma_err[2]};

  return inputs{
      0., 
      0., 
      0., 
      control_torques[0], 
      control_torques[1], 
      control_torques[2]
  };
}



}  // namespace control
}  // namespace propagate
