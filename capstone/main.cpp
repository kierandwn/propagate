// propagate.cpp : Defines the entry point for the application.
//
#include <iostream>

#include "dynamics/dynamics.h"
#include "dynamics/model.h"

#include "config/config.h"
#include "telemetry/log.h"


using namespace capstone;

int main()
{
  // PARSE COMMAND LINE INPUT FOR CONFIG FILEPATH
  string file_path = __FILE__;
  string src_directory = file_path.substr(0, file_path.rfind("/"));

  config::init_node(
    "/Users/kierandwn/projects/capstone/capstone/config/default.yaml"
  );

  telemetry::log state_log("capstone_states");
  state_log.set_dir(src_directory + "/../log/");

  system::system_model s;
  s.initialise();
  
  control::controller c;

  propagate::propagator p(s, c);
  p.attach_logger(&state_log);
  p.initialise();

  attitude::vector<double, 6> xf = p.simulate();
	return 0;
}
