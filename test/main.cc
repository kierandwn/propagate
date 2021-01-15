#include <cmath>

#include "C:/Users/kdwn/projects/propagate/propagate/lib/attitude/include/matrix.h"
#include "C:/Users/kdwn/projects/propagate/propagate/lib/frame/include/frame.h"

void main();

const double kPi = 3.14159265358979323846;
double deg2rad = kPi / 180.;

double gravitational_constant = 42828.3;

using vector3 = attitude::vector<double, 3>;
using matrix3 = attitude::matrix<double, 3, 3>;

using euler_set = attitude::euler<double>;
using mrp_set = attitude::mrp<double>;

void main() {
  // Task 1.1

  // LMO
  double orbit_radius = 3396.19 + 400.;
  double lmo_rate = sqrt(gravitational_constant / pow(orbit_radius, 3));

  double dtheta = 0.000884797 * 450.;
  attitude::euler<double> angles(20., 30., 60., 313);
  angles *= deg2rad;
  angles[2] += dtheta;

  vector3* orbit_position = new vector3;
  vector3* orbit_velocity = new vector3;

  propagate::frame::get_orbit_position_and_velocity(
      angles, orbit_radius, orbit_position, orbit_velocity, lmo_rate);

  /*for (int i = 0; i < 3; ++i) {
    printf("r(lmo): %.6f\n", orbit_position[i]);
  }*/

  // Task 1.2
  // GMO
  orbit_radius = 20424.2;
  double gmo_rate = sqrt(gravitational_constant / pow(orbit_radius, 3));

  dtheta = 0.0000709003 * 1150.;
  angles[0] = 0.;
  angles[1] = 0.;
  angles[2] = 250.;
  angles *= deg2rad;

  angles[2] += dtheta;

  propagate::frame::get_orbit_position_and_velocity(
      angles, orbit_radius, orbit_position, orbit_velocity, gmo_rate);

  /*for (int i = 0; i < 3; ++i) {
    printf("r(gmo): %.6f\n", orbit_position[i]);
  }*/

  delete orbit_position;
  delete orbit_velocity;

  // Task 2.1
  matrix3 lmo_dcm = propagate::frame::compute_lmo_dcm(300.);
  /*for (int i = 0; i < 9; ++i) {
    printf("dcm(lmo): %.6f\n", lmo_dcm(i));
  }*/

  // Task 3.1
  matrix3 rsn_dcm = propagate::frame::compute_rsn_dcm();
  /*for (int i = 0; i < 9; ++i) {
    printf("dcm(rsn): %.6f\n", rsn_dcm(i));
  }*/

  // Task 3.2
  matrix3 rnn_dcm = propagate::frame::compute_rnn_dcm(330.);
  for (int i = 0; i < 9; ++i) {
    printf("dcm(rnn): %.8f\n", rnn_dcm(i));
  }

  vector3 rnn_omega = propagate::frame::compute_rnn_omega(330.);
  for (int i = 0; i < 3; ++i) {
    printf("omega(rnn): %.8f\n", rnn_omega[i]);
  }

  // Task 3.3
  matrix3 gmo_dcm = propagate::frame::compute_comm_dcm(330.);
  /*printf("dcm(rcn)[%.6f]:\n", determinant(gmo_dcm));
  display(gmo_dcm);*/

  vector3 rcn_omega = propagate::frame::compute_comm_omega(330.);
  /*for (int i = 0; i < 3; ++i) {
    printf("omega(rcn): %.8f\n", rcn_omega[i]);
  }*/
  
  // Task 4.1
  mrp_set sigma_err;
  vector3 omega_err;

  mrp_set sigma_0 = mrp_set(0.3, -0.4, 0.5);
  vector3 omega_0 = vector3{1.00, 1.75, -2.20} * deg2rad;

  for (int i = 0; i < 3; ++i) {
    sigma_err = propagate::frame::compute_attitude_error(0., i, sigma_0);
    omega_err =
        propagate::frame::compute_omega_error(0., i, sigma_0, omega_0);

    printf("phase %d: \n", i);
    printf("sigma(%.4f): ", sigma_err.norm()); display(sigma_err);
    sigma_err = sigma_err.shadow();
    printf("sigma(%.4f): ", sigma_err.norm());
    display(sigma_err);
    printf("omega: "); display(omega_err);
  }
}

