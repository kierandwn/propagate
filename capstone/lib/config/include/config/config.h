#ifndef CAPSTONE_CONFIG_H_
#define CAPSTONE_CONFIG_H_

#include <string>

#include "yaml-cpp/yaml.h"

using namespace std;

namespace capstone {
namespace config {

void set_file(string);

void init_node(string);

YAML::Node get_base_node();
YAML::Node get_config_node(string);


}  // namespace config
}  // namespace capstone
#endif // CAPSTONE_CONFIF_H_