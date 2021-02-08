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

  config::init_node(
    "/home/kierandwn/projects/capstone/capstone/config/default.yaml"
  );

  telemetry::log solver_log("capstone_states");

  system::system_model s;
  s.initialise();
  
  control::controller c;

  propagate::propagator p(s, c);
  p.attach_logger(&solver_log);
  p.initialise();

  attitude::vector<double, 6> xf = p.simulate();
	return 0;
}
