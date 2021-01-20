#ifndef CAPSTONE_CONFIG_H_
#define CAPSTONE_CONFIG_H_

#include <string>
// #include "config/solver.h"

#include "yaml-cpp/yaml.h"

using namespace std;

namespace config {

void set_file(string);

void init_node(string);

YAML::Node get_base_node();
YAML::Node get_config_node(string);


} // namespace config
#endif // CAPSTONE_CONFIF_H_