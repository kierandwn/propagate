#include "dynamics/model.h"

#include <cmath>

#include "attitude/matrix.h"

namespace capstone {
namespace system {

rates attitude_kinematics_mrp(states x, attitude::mn_matrix<double, 3, 3> inertia_tensor_) 
{
  double sigma_sq = pow(x[0], 2.) + pow(x[1], 2.) + pow(x[2], 2.);
  return rates{
            (0.25 * (1. - sigma_sq) + 0.5 * pow(x[0], 2.)) * x[3] +  //
                              (0.5 * (x[0] * x[1] - x[2])) * x[4] +
                              (0.5 * (x[0] * x[2] + x[1])) * x[5],
                              (0.5 * (x[0] * x[1] + x[2])) * x[3] +  //
            (0.25 * (1. - sigma_sq) + 0.5 * pow(x[1], 2.)) * x[4] +
                              (0.5 * (x[1] * x[2] - x[0])) * x[5],
                              (0.5 * (x[0] * x[2] - x[1])) * x[3] +  //
                              (0.5 * (x[1] * x[2] + x[0])) * x[4] +
            (0.25 * (1. - sigma_sq) + 0.5 * pow(x[2], 2.)) * x[5],
    (inertia_tensor_[1][1] - inertia_tensor_[2][2]) * x[4] * x[5],   //
    (inertia_tensor_[2][2] - inertia_tensor_[0][0]) * x[3] * x[5],   //
    (inertia_tensor_[0][0] - inertia_tensor_[1][1]) * x[3] * x[4]    //
  };
}

rates direct(inputs u) { return attitude::eye<double, 6>() * u; }

void system_model::initialise() { 
  cfg_.initialise(); 

  set_plant_model();
  set_actuator_model();

  torques_to_accels_ = attitude::mn_matrix<double, 6, 6>{
    1., 0., 0., 0., 0., 0.,
    0., 1., 0., 0., 0., 0., 
    0., 0., 1., 0., 0., 0.,
    0., 0., 0., 1. / inertia_tensor_[0][0], 0., 0.,
    0., 0., 0., 0., 1. / inertia_tensor_[0][0], 0.,
    0., 0., 0., 0., 0., 1. / inertia_tensor_[0][0]
  };
}

void system_model::set_plant_model() {
  if (cfg_.PLANT_MODEL == "attitude_kinematics_mrp") {
    plant_model_fcn_ = attitude_kinematics_mrp;
  } else {
    // default case
    plant_model_fcn_ = attitude_kinematics_mrp;
  }
}

void system_model::set_actuator_model() {
  if (cfg_.ACTUATOR_MODEL == "direct") {
    actuator_model_fcn_ = direct;
  } else {
    // default case
    actuator_model_fcn_ = direct;
  }
}


} // namespace system
} // namespace capstone

