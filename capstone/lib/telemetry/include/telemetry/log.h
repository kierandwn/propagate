#ifndef PROPAGATE_LOG_H_
#define PROPAGATE_LOG_H_

#include <fstream>
#include <ctime>

#include <string>
#include <vector>
#include <map>

namespace capstone {
namespace telemetry {

using namespace std;

class log {
 private:
  string dir_;
  string id_;
  
  ofstream f_;

  map<string, double> buf_;

 public:
  log(string);
  ~log();

  void set_dir(string);
  bool ready();
  bool init(vector<string>);

  void update_buffer(vector<string>, double *, double);
  void write_headers();
  void write_row();

  double& operator[](string key) { return buf_[key]; }
};

}  // namespace telemetry
}  // namespace propagate
#endif  // PROPAGATE_LOG_H_