#ifndef CAPSTONE_MODEL_H_
#define CAPSTONE_MODEL_H_

#include <functional>
#include <string>

#include "attitude/matrix.h"
#include "../../config/include/config/system.h"

namespace capstone {
namespace system {


using namespace std;

static const size_t kStateDims = 6;
using states = attitude::vector<double, kStateDims>;
using inputs = attitude::vector<double, kStateDims>;
using rates = attitude::vector<double, kStateDims>;

rates attitude_kinematics_mrp(states, attitude::mn_matrix<double, 3, 3>);
rates direct(inputs);

function<rates(states)> determine_plant_model(string);
function<rates(inputs)> determine_actuator_model(string);


class system_model {
 private:
  config::system cfg_;

  std::function<rates(states, attitude::mn_matrix<double, 3, 3>)> plant_model_fcn_;
  std::function<rates(inputs)> actuator_model_fcn_;

  attitude::mn_matrix<double, 3, 3>& inertia_tensor_ = cfg_.INERTIA;
  attitude::mn_matrix<double, 6, 6> torques_to_accels_;

  void set_plant_model();
  void set_actuator_model();

 public:
  system_model() {}

  void initialise();

  rates operator()(states x, inputs u) {
    return torques_to_accels_ * (plant_model_fcn_(x, inertia_tensor_) + actuator_model_fcn_(u));
  }
};

}  // namespace system
}  // namespace capstone
#endif  // CAPSTONE_MODEL_H_
