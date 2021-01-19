// propagate.cpp : Defines the entry point for the application.
//
#include <iostream>

#include "lib/dynamics/include/dynamics.h"
#include "lib/attitude/include/matrix.h"

using namespace std;

const double kPi = 3.14159265358979323846;

attitude::vector<double, 6> x0{
    0.3, -0.4, 0.5, 1.00 * kPi / 180., 1.75 * kPi / 180., -2.20 * kPi / 180.
};

int main()
{
  attitude::vector<double, 6> xf = propagate::simulate(x0, 6500., .1);
	return 0;
}
