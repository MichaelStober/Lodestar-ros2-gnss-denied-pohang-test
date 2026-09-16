#include "lodestar_odometry/statistics.h"

#include <cmath>
#include <iomanip>
#include <iostream>

namespace lodestar_odom {

statistics timing = statistics();

statistics::statistics() {}

void statistics::Document(const std::string& name, const double& value, bool report) {
  t[name].push_back(value);
  if (report)
    cout << "Statistics: " << std::quoted(name) << " = " << value << endl;
}

void statistics::ComputeStatistics(std::vector<reports>& rep) {
  for (executionTimes::iterator itr = t.begin(); itr != t.end(); itr++) {
    double sum_t = std::accumulate(std::begin(itr->second), std::end(itr->second), 0.0, std::plus<double>());
    double mean_t = sum_t / itr->second.size();
    double variance_t = 0;

    for (auto&& i : itr->second)
      variance_t += (i - mean_t) * (i - mean_t);
    variance_t = variance_t / itr->second.size();

    std::string name = itr->first;
    reports r = std::make_tuple(name, mean_t, variance_t, (int)itr->second.size());
    rep.push_back(r);
  }
}

void statistics::PresentStatistics() {
  std::vector<reports> rep;
  ComputeStatistics(rep);
  std::cout << "-------------- EXECUTION STATISTICS ---------------" << std::endl;
  for (auto&& r : rep)
    std::cout << std::quoted(std::get<0>(r)) << " - mean: " << std::get<1>(r)
              << ", sigma: " << std::get<2>(r) << ", N: " << std::get<3>(r) << endl;
  cout << "---------FINGERS CROSSED FOR GOOD RESUTLS --------" << endl;
}

std::string statistics::GetStatistics() {
  std::vector<reports> rep;
  ComputeStatistics(rep);
  std::string str;
  for (auto&& r : rep) {
    str += std::get<0>(r) + std::string(" avg, ") + std::to_string(std::get<1>(r)) + "\n";
    str += std::get<0>(r) + std::string(" dev [σ], ") + std::to_string(std::get<2>(r)) + "\n";
    str += std::get<0>(r) + std::string(" count, ") + std::to_string(std::get<3>(r)) + "\n";
  }
  return str;
}

double ToMs(const rclcpp::Duration& dur) {
  return static_cast<double>(dur.nanoseconds()) / 1000000.0;
}

double ToMs(const std::chrono::steady_clock::duration& dur) {
  return std::chrono::duration<double, std::milli>(dur).count();
}

double ToMsClock(const double& t) {
  return 1000 * ((double)t) / ((double)CLOCKS_PER_SEC);
}

double ToMsSince(const std::chrono::steady_clock::time_point& start) {
  return ToMs(std::chrono::steady_clock::now() - start);
}

}  // namespace lodestar_odom
