#ifndef PROPAGATE_LOG_H_
#define PROPAGATE_LOG_H_

#include <fstream>
#include <ctime>

#include <string>
#include <vector>
#include <map>

namespace propagate {
namespace telemetry {

using namespace std;

class log {
 private:
  string dir_;
  // string logfile_fmt = "Ymd_H-M-S.csv";
  ofstream f_;

  char timestr_[32];

  map<string, double> buf_;

 public:
  log(string);
  ~log();

  bool ready();
  bool init(vector<string>);

  void write_headers();
  void write_row();

  double& operator[](string key) { return buf_[key]; }
};

}  // namespace telemetry
}  // namespace propagate
#endif  // PROPAGATE_LOG_H_