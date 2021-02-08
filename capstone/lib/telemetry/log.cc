#include "telemetry/log.h"

#include <ctime>
#include <fstream>
#include <string>

namespace capstone {
namespace telemetry {


using namespace std;

log::log(string string_id) : id_(string_id) {}
log::~log() { f_.close(); }

void log::set_dir(string log_directory) { dir_ = log_directory; }

bool log::ready() {
  time_t curr_time;
  tm* curr_tm;

  time(&curr_time);
  curr_tm = localtime(&curr_time);

  int len_timestr = 23 + id_.size();
  char * timestr = new char[len_timestr];

  strftime(timestr, len_timestr, id_.append("_%Y%m%d_%H-%M-%S.csv").c_str(), curr_tm);
  f_.open((dir_ + timestr).c_str());
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

void log::update_buffer(vector<string> channel_names, double * x, double t) {
  buf_[channel_names[0]] = t;

  for (int i = 0; i < 6; ++i) {
    buf_[channel_names[i + 1]] = x[i];
  }
}

}  // namespace telemetry
}  // namespace propagate

