#include "config/config.h"

#include <string>

namespace capstone {
namespace config {


using namespace std;

YAML::Node Y;

void init_node(string filename) {
  Y = YAML::LoadFile(filename);
}

YAML::Node get_base_node() { return Y; }
YAML::Node get_config_node(string id) { 
  YAML::Node component = Y[id];
  return Y[id]; 
}


} // namespace config
} // namespace capstone

