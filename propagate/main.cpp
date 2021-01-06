// propagate.cpp : Defines the entry point for the application.
//
#include <iostream>

#include "lib/dynamics/include/dynamics.h"
#include "lib/attitude/include/matrix.h"

using namespace std;

int main()
{
  attitude::vector<double, 6> xf = propagate::simulate(0.01);
	return 0;
}
