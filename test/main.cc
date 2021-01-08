#include <cmath>

#include "C:/Users/kdwn/projects/propagate/propagate/lib/attitude/include/matrix.h"
#include "C:/Users/kdwn/projects/propagate/propagate/lib/frame/include/frame.h"

void main();

const double kPi = 3.14159265358979323846;
double deg2rad = kPi / 180.;

double gravitational_constant = 42828.3;

using vector3 = attitude::vector<double, 3>;
using matrix3 = attitude::matrix<double, 3, 3>;


void main() {
  // Task 1.1

  // LMO
  double orbit_radius = 3396.19 + 400.;
  double lmo_rate = sqrt(
    gravitational_constant / pow(orbit_radius, 3)
  );
  

  double dtheta = 0.000884797 * 450.;
  attitude::euler<double> angles(20., 30., 60., 313);
  angles *= deg2rad;
  angles[2] += dtheta;

  vector3* orbit_position = new vector3;
  vector3* orbit_velocity = new vector3;

  propagate::frame::get_orbit_position_and_velocity(
      angles, orbit_radius, orbit_position, orbit_velocity, lmo_rate);

  // Task 1.2
  // GMO
  orbit_radius = 20424.2;
  double gmo_rate = sqrt(
    gravitational_constant / pow(orbit_radius, 3)
  );

  dtheta = 0.0000709003 * 1150.;
  angles[0] = 0.;
  angles[1] = 0.;
  angles[2] = 250.;
  angles *= deg2rad;

  angles[2] += dtheta;

  propagate::frame::get_orbit_position_and_velocity(
      angles, orbit_radius, orbit_position, orbit_velocity, gmo_rate);

  delete orbit_position;
  delete orbit_velocity;

  // Task 2.1
  matrix3 dcm;
  vector3 omega;

  propagate::frame::compute_lmo_dcm(300., &dcm);
  /*for (int i = 0; i < 9; ++i) {
    printf("dcm(lmo): %.6f\n", dcm(i));
  }*/

  // Task 3.1
  propagate::frame::compute_rsn_dcm(&dcm);
  /*for (int i = 0; i < 9; ++i) {
    printf("dcm(rsn): %.6f\n", dcm(i));
  }*/

  // Task 3.2
  propagate::frame::compute_rnn_dcm(330., &dcm);
  /*for (int i = 0; i < 9; ++i) {
    printf("dcm(rnn): %.8f\n", dcm(i));
  }*/

  propagate::frame::compute_rnn_omega(330., &omega);
  /*for (int i = 0; i < 3; ++i) {
    printf("omega(rnn): %.8f\n", omega[i]);
  }*/
}
