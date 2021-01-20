#ifndef CAPSTONE_CONFIG_SOLVER_H_
#define CAPSTONE_CONFIG_SOLVER_H_

#include "../config.h"

#include <string>
#include <vector>

#include "../../attitude/include/matrix.h"

namespace config {


using namespace std;

struct solver {
  string INTEGRATION_TYPE;

  double TIMESTEP;
  double START_TIME;
  double END_TIME;

  vector<bool> ANGLE_STATES;
  attitude::vector<double, 6> INITIAL_STATES;
  int N_DIM = 6;

  void initialise() {
    YAML::Node NODE = get_config_node("solver");

    INTEGRATION_TYPE = NODE["integration"].as<string>();

    TIMESTEP    = NODE["timestep"].as<double>();
    START_TIME  = NODE["start_time"].as<double>();
    END_TIME    = NODE["end_time"].as<double>();

    // config.N_DIM = yaml["state_ndim"].as<int>(); // support for 6 only
    // currently

    vector<bool> is_angle = NODE["states_are_angles"].as<vector<bool>>();
    bool angles_are_deg   = NODE["angles_are_deg"].as<bool>();

    vector<double> intial_states = NODE["initial_states"].as<vector<double>>();

    for (int i = 0; i < N_DIM; ++i) {
      if (angles_are_deg && is_angle[i]) {
        INITIAL_STATES[i] = intial_states[i] * (3.14159265358979323846 / 180.);
      } else {
        INITIAL_STATES[i] = intial_states[i];
      }
    }
  }
};

} // namespace config
#endif  // CAPSTONE_CONFIG_SOLVER_H_