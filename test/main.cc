#include <cmath>

#include "C:/Users/kdwn/projects/propagate/propagate/lib/attitude/include/matrix.h"
#include "C:/Users/kdwn/projects/propagate/propagate/lib/frame/include/frame.h"

void main();

const double kPi = 3.14159265358979323846;
double deg2rad = kPi / 180.;

using vector3 = attitude::vector<double, 3>;
using matrix3 = attitude::matrix<double, 3, 3>;


void main() {
  // Task 1.1

  // LMO
  double orbit_radius = 3396.19 + 400.;

  double dtheta = 0.000884797 * 450.;
  attitude::euler<double> angles(20., 30., 60., 313);
  angles *= deg2rad;
  angles[2] += dtheta;

  vector3* orbit_position = new vector3;
  vector3* orbit_velocity = new vector3;

  propagate::frame::get_orbit_position_and_velocity(
      angles, orbit_radius, orbit_position, orbit_velocity);

  // Task 1.2
  // GMO
  orbit_radius = 20424.2;

  dtheta = 0.0000709003 * 1150.;
  angles[0] = 0.;
  angles[1] = 0.;
  angles[2] = 250.;
  angles *= deg2rad;

  angles[2] += dtheta;

  propagate::frame::get_orbit_position_and_velocity(
      angles, orbit_radius, orbit_position, orbit_velocity);

  delete orbit_position;
  delete orbit_velocity;

  // Task 2.1
  matrix3 dcm;

  propagate::frame::compute_lmo_dcm(300., &dcm);
  for (int i = 0; i < 9; ++i) {
    printf("dcm: %.6f\n", dcm(i));
  }
}
