// propagate.cpp : Defines the entry point for the application.
//
#include <iostream>

#include "lib/dynamics/include/dynamics.h"
#include "lib/attitude/include/matrix.h"

#include "lib/config/include/config.h"

using namespace std;

int main()
{
  config::init_node(
      "C:\\Users\\kdwn\\projects\\capstone\\capstone\\config\\default.yaml");

  attitude::vector<double, 6> xf = propagate::simulate();
	return 0;
}
