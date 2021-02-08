#ifndef PROPAGATE_CONTROL_H_
#define PROPAGATE_CONTROL_H_

#include <functional>

#include "attitude/matrix.h"
#include "attitude/mrp.h"

#include "telemetry/log.h"

// --- SPACECRAFT DYNAMICS AND CONTROL SPECIALISATION: CAPSTONE MISSION ---
//            .       .                   .       .      .     .      .
//           .    .         .    .            .     ______
//       .           .             .               ////////
//                 .    .   ________   .  .      /////////     .    .
//            .            |.____.  /\        ./////////    .
//     .                 .//      \/  |\     /////////
//        .       .    .//          \ |  \ /////////       .     .   .
//                     ||.    .    .| |  ///////// .     .
//      .    .         ||           | |//`,/////                .
//              .       \\        ./ //  /  \/   .
//   .                    \\.___./ //\` '   ,_\     .     .
//           .           .     \ //////\ , /   \                 .    .
//                        .    ///////// \|  '  |    .
//       .        .          ///////// .   \ _ /          .
//                         /////////                              .
//                  .   ./////////     .     .
//          .           --------   .                  ..             .
//   .               .        .         .                       .
//                         ________________________
// ____________------------                        -------------_________
// 

namespace capstone {
namespace control {

using namespace attitude;

using matrix3 = attitude::mn_matrix<double, 3, 3>;
using vector3 = attitude::vector<double, 3>;

static const size_t kStateDims = 6;
using states = attitude::vector<double, kStateDims>;
using inputs = attitude::vector<double, kStateDims>;

inputs simple_pd(states x, double t);
inputs test_out_controller(states x, double t);
inputs closedloop_linear(states x, double t);

//std::function<inputs(states, double, telemetry::log *)> determine_control() {
//  if (CFG_.INTEGRATION_SCHEME == "simple_pd") {
//    return control::simple_pd;
//
//  } else if (CFG_.INTEGRATION_SCHEME == "tests") {
//    return control::test_out_controller;
//
//  } else {
//    return control::simple_pd;
//  }
//}

class controller
{
 private:
  std::function<inputs(states, double)> update_fcn_ = simple_pd;
  telemetry::log * logger_;

 public:
  inputs operator()(states x, double t) { return update_fcn_(x, t); }
};


}  // namespace control
}  // namespace propagate
#endif  // PROPAGATE_CONTROL_H_