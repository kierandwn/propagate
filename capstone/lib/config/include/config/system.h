#ifndef CAPSTONE_CONFIG_SYSTEM_H_
#define CAPSTONE_CONFIG_SYSTEM_H_

#include "config/config.h"

#include <string>
#include <vector>

#include "yaml-cpp/yaml.h"
#include "attitude/matrix.h"

namespace capstone {
namespace config {


using namespace std;

struct system {
  string PLANT_MODEL;
  string ACTUATOR_MODEL;

  attitude::mn_matrix<double, 3, 3> INERTIA;

  void initialise() {
    YAML::Node NODE = get_config_node("system");

    PLANT_MODEL     = NODE["plant_model"].as<string>();
    ACTUATOR_MODEL  = NODE["actuator_model"].as<string>();
  
    vector<double> temp_inertia = NODE["inertia"].as<vector<double>>();
    for (int i = 0; i < 9; ++i) {
      INERTIA(i) = temp_inertia[i];
    }
  }
};

}  // namespace config
}  // namespace capstone
#endif  // CAPSTONE_CONFIG_SYSTEM_H_