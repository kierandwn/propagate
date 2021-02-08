#ifndef CAPSTONE_PROPAGATE_H_
#define CAPSTONE_PROPAGATE_H_

#include <vector>
#include <string>

#include "model.h"

#include "control/control.h"
#include "attitude/matrix.h"

#include "../../config/include/config/solver.h"
#include "telemetry/log.h"

namespace capstone {
namespace propagate {


static const size_t kStateDims = 6;
using states = attitude::vector<double, kStateDims>;
using inputs = attitude::vector<double, kStateDims>;
using rates = attitude::vector<double, kStateDims>;

states rk4(states x0, double t, double dt, inputs u,
           std::function<rates(states, inputs)> f);
states euler(states x0, double t, double dt, inputs u,
             std::function<rates(states, inputs)> f);

class propagator {
 private:
  config::solver cfg_;
  telemetry::log * logger_;

  std::function<states(states, double, double, inputs,
                       function<rates(states, inputs)>)> propagate_fcn_;

  control::controller& control_;
  inputs control_input_;

  system::system_model& f_;

  void set_propagator();

 public:
  propagator(system::system_model& sys, control::controller& ctrl) 
    : control_(ctrl),
      f_(sys)
  {}

  void initialise();

  void attach_logger(telemetry::log *);

  /*void attach_system_model(system::system_model&);
  void attach_controller(control::controller&);*/

  states simulate();
};

}  // namespace propagate
}  // namespace capstone
#endif  // CAPSTONE_PROPAGATE_H_