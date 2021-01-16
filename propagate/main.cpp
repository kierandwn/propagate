// propagate.cpp : Defines the entry point for the application.
//
#include <iostream>

#include "lib/dynamics/include/dynamics.h"
#include "lib/attitude/include/matrix.h"

using namespace std;

const double kPi = 3.14159265358979323846;

attitude::vector<double, 6> x0{
    0.1, 0.2, -0.1, 30. * kPi / 180., 10. * kPi / 180., -20. * kPi / 180.
    //0.1, 0.2, -0.1, 3. * kPi / 180., 1. * kPi / 180., -2. * kPi / 180.
};

int main()
{
  attitude::vector<double, 6> xf = propagate::simulate(x0, 120., .1);
	return 0;
}
