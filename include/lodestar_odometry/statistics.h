#pragma once

#include <time.h>

#include <chrono>
#include <iostream>
#include <map>
#include <numeric>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>

namespace lodestar_odom {

using std::cerr;
using std::cout;
using std::endl;

typedef std::unordered_map<std::string, std::vector<double>> executionTimes;
typedef std::tuple<std::string, double, double, int> reports;  // name, mean, variance, N samples

class statistics {
 public:
  statistics();

  void Document(const std::string& name, const double& value, bool report = false);

  void PresentStatistics();

  std::string GetStatistics();

 private:
  void ComputeStatistics(std::vector<reports>& rep);

  executionTimes t;
};

extern statistics timing;

/** Milliseconds from a ROS duration (message/bag timestamps). */
double ToMs(const rclcpp::Duration& dur);

/** Milliseconds from a steady-clock duration (wall-clock profiling).
 *  Preferred for timing code sections: it never mixes ROS time sources. */
double ToMs(const std::chrono::steady_clock::duration& dur);

double ToMsClock(const double& t);

/** Convenience: milliseconds elapsed since a steady-clock time point. */
double ToMsSince(const std::chrono::steady_clock::time_point& start);

}  // namespace lodestar_odom
