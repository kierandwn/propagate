#include "config.h"

#include <string>

using namespace std;

namespace config {

YAML::Node Y;

void init_node(string filename) {
  Y = YAML::LoadFile(filename);
}

YAML::Node get_base_node() { return Y; }
YAML::Node get_config_node(string id) { return Y[id]; }


} // namespace config

