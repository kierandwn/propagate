#include "log.h"

#include <ctime>
#include <fstream>
#include <string>

namespace propagate {
namespace telemetry {


using namespace std;

log::log(string log_directory) : dir_(log_directory) {}
log::~log() { f_.close(); }

bool log::ready() {
  time_t curr_time;
  tm* curr_tm;

  time(&curr_time);
  curr_tm = localtime(&curr_time);

  strftime(timestr_, 32, "propagate_%Y%m%d_%H-%M-%S.csv", curr_tm);
  f_.open((dir_ + timestr_).c_str());
  return f_.is_open();
}

bool log::init (vector<string> channel_names) {
  vector<string>::iterator c;
  for (c = channel_names.begin(); c != channel_names.end(); ++c) {
    if (buf_.find(*(c)) == buf_.end()) {
      buf_[*(c)] = -1.;
    }
  }
  write_headers();
  return true;
}

void log::write_headers() {
  if (f_.is_open()) {
    map<string, double>::iterator h;
    string row = "";

    for (h = buf_.begin(); h != buf_.end(); ++h) {
      if (h != buf_.begin()) { row += ", "; }
      row += h->first;
    }
    f_ << row.c_str() << std::endl;
  }
}

void log::write_row() {
  if (f_.is_open()) {
    map<string, double>::iterator h;
    string row = "";

    for (h = buf_.begin(); h != buf_.end(); ++h) {
      if (h != buf_.begin()) { row += ", "; }
      row += to_string(h->second);
    }
    f_ << row.c_str() << std::endl;
  }
}


}  // namespace telemetry
}  // namespace propagate

