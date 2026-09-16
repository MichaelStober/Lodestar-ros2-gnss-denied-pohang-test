#pragma once

#include <time.h>

#include <cstdio>
#include <fstream>
#include <iterator>
#include <list>
#include <string>
#include <tuple>

#include <Eigen/Eigen>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include "ceres/autodiff_cost_function.h"
#include "ceres/ceres.h"
#include "ceres/manifold.h"

#include "angles/angles.h"

namespace lodestar_odom {

using std::cout;
using std::endl;

/** Convenience alias used throughout the package. */
using PointCloudXYZI = pcl::PointCloud<pcl::PointXYZI>;

/**
 * Reproduce the ROS 1 private-namespace behaviour of `ros::NodeHandle("~")`.
 *
 * Under ROS 1 every publisher in this package was created on a private node
 * handle, so a relative topic name such as "radar_odom" resolved to
 * "/lodestar_odom_node/radar_odom". Under ROS 2 a relative name resolves to
 * the node's namespace instead, so relative names get an explicit "~/" prefix
 * to keep the effective topic names (and therefore the RViz configuration)
 * identical. Absolute names ("/rot_lodestar") and names that are already
 * private ("~/x") are passed through untouched.
 */
inline std::string PrivateTopic(const std::string& name) {
  if (name.empty())
    return name;
  if (name.front() == '/' || name.front() == '~')
    return name;
  return "~/" + name;
}

inline double GetRelTimeStamp(const double x, const double y) {
  double a = atan2(y, x);
  double d = ((a > 0.00001 ? a : (2 * M_PI + a)) / (2 * M_PI));
  return (d - 0.5);
}

void Compensate(pcl::PointCloud<pcl::PointXYZI>& cloud, const std::vector<double>& mot);

void Compensate(pcl::PointCloud<pcl::PointXYZI>& cloud, const Eigen::Affine3d& Tmotion);

void Affine3dToVectorXYeZ(const Eigen::Affine3d& T, std::vector<double>& par);

void Affine3dToEigVectorXYeZ(const Eigen::Affine3d& T, Eigen::Vector3d& par);

Eigen::Vector3d getScaledTranslationVector(const std::vector<double>& parameters, double factor);

Eigen::Matrix3d getScaledRotationMatrix(const std::vector<double>& parameters, double factor);

template <typename T>
void printVector(const T& t) {
  std::copy(t.cbegin(), t.cend(), std::ostream_iterator<typename T::value_type>(std::cout, ", "));
}

}  // namespace lodestar_odom
