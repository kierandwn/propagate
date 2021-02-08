#include "dynamics/dynamics.h"
#include "dynamics/model.h"

#include <cmath>
#include <vector>
#include <functional>

#include "attitude/matrix.h"
#include "attitude/mrp.h"

#include "control/control.h"

#include "config/config.h"
#include "telemetry/log.h"

namespace capstone {
namespace propagate {


using matrix3 = attitude::mn_matrix<double, 3, 3>;
using vector3 = attitude::vector<double, 3>;

using mrp_set = attitude::mrp<double>;


void propagator::attach_logger(telemetry::log * logger) { logger_ = logger; }
//void propagator::attach_system_model(system::system_model& system) {
//  f_ = system;
//}
//void propagator::attach_controller(control::controller& controller) {
//  control_ = controller;
//}

void propagator::initialise() { 
  cfg_.initialise(); 

  logger_->ready();
  logger_->init(cfg_.CHANNEL_NAMES);

  set_propagator();

  control_input_ = control_(cfg_.INITIAL_STATES, cfg_.START_TIME);

  logger_->update_buffer(
    cfg_.CHANNEL_NAMES, 
    cfg_.INITIAL_STATES.c_array(),
    cfg_.START_TIME
  );
  logger_->write_row();
}

void propagator::set_propagator() 
{
  if (cfg_.INTEGRATION_SCHEME == "rk4") {
    propagate_fcn_ = rk4;

  } else if (cfg_.INTEGRATION_SCHEME == "euler") {
    propagate_fcn_ = euler;

  } else {
    propagate_fcn_ = euler;
  }
}

states propagator::simulate() {
  states x = cfg_.INITIAL_STATES;
  double t = cfg_.START_TIME;

  while ((t += cfg_.TIMESTEP) < cfg_.END_TIME) {
    x = propagate_fcn_(x, t, cfg_.TIMESTEP, control_input_, f_);

    logger_->update_buffer(
      cfg_.CHANNEL_NAMES, 
      x.c_array(),
      t
    );
    logger_->write_row();
  }
  return x;
}

states rk4(states x0, double t, double dt, inputs u, 
  std::function<rates(states, inputs)> f) 
{
  states k1 = f(x0, u);
  states k2 = f(x0 + (k1 * (dt / 2.)), u);
  states k3 = f(x0 + (k2 * (dt / 2.)), u);
  states k4 = f(x0 + (k3 * dt), u);

  return x0 + (k1 + k2 * 2. + k3 * 2. + k4) * (dt / 6.);
}

states euler(states x0, double t, double dt, inputs u,
  std::function<rates(states, inputs)> f) 
{
  return x0 + f(x0, u) * dt;
}

}  // namespace propagate
}  // namespace capstone