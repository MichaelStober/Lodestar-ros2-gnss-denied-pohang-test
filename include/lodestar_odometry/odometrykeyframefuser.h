#pragma once

#include <time.h>

#include <chrono>
#include <cstdio>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Eigen>

#include <boost/circular_buffer.hpp>

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "lodestar_odometry/lodestar.h"
#include "lodestar_odometry/n_scan_normal.h"
#include "lodestar_odometry/pointnormal.h"
#include "lodestar_odometry/statistics.h"
#include "lodestar_odometry/utils.h"

using std::cerr;
using std::cout;
using std::endl;
using std::string;

namespace lodestar_odom {

visualization_msgs::msg::Marker GetDefault();

typedef std::vector<std::pair<Eigen::Affine3d, MapNormalPtr> > PoseScanVector;

class OdometryKeyframeFuser {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

 public:
  class Parameters {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

   public:
    Parameters() {}
    std::string scan_registered_latest_topic = "radar_registered";
    std::string scan_registered_keyframe_topic = "radar_registered_keyframe";
    std::string odom_latest_topic = "radar_odom";
    std::string odom_keyframe_topic = "radar_odom_keyframe";
    std::string odometry_link_id = "world";
    std::string input_points_topic = "/marine/Filtered";
    std::string cost_type = "P2L";
    weightoption weight_opt = weightoption::Uniform;

    bool visualize = true;
    int submap_scan_size = 3;
    bool weight_intensity_ = false;

    bool use_guess = true, disable_registration = false, soft_constraint = false;
    bool compensate = true;
    bool use_keyframe = true, enable_filter = false, use_raw_pointcloud = false;
    double res = 3.5;
    double min_keyframe_dist_ = 1.5, min_keyframe_rot_deg_ = 5;
    std::string loss_type_ = "Huber";
    double loss_limit_ = 0.1;
    double covar_scale_ = 1.0;
    double regularization_ = 0.0;

    bool publish_tf_ = true;

    void GetParametersFromRos(rclcpp::Node& n) {
      input_points_topic = n.declare_parameter<std::string>("input_points_topic", "/marine/Filtered");
      scan_registered_latest_topic =
          n.declare_parameter<std::string>("scan_registered_latest_topic", "radar_registered");
      scan_registered_keyframe_topic = n.declare_parameter<std::string>(
          "scan_registered_keyframe_topic", "radar_registered_keyframe");
      odom_latest_topic = n.declare_parameter<std::string>("odom_latest_topic", "radar_odom");
      odom_keyframe_topic =
          n.declare_parameter<std::string>("odom_keyframe_topic", "radar_odom_keyframe");

      odometry_link_id = n.declare_parameter<std::string>("odometry_link_id", "world");
      visualize = n.declare_parameter<bool>("visualize", true);

      use_raw_pointcloud = n.declare_parameter<bool>("use_raw_pointcloud", false);
      submap_scan_size = n.declare_parameter<int>("submap_scan_size", 3);

      res = n.declare_parameter<double>("res", 3.0);
      MapPointNormal::downsample_factor = n.declare_parameter<double>("downsample_factor", 1.0);

      min_keyframe_dist_ = n.declare_parameter<double>("registered_min_keyframe_dist", 1.5);
      min_keyframe_rot_deg_ = n.declare_parameter<double>("min_keyframe_rot_deg_", 5.0);
      use_keyframe = n.declare_parameter<bool>("use_keyframe", true);
      use_guess = n.declare_parameter<bool>("use_guess", true);
      disable_registration = n.declare_parameter<bool>("disable_registration", false);
      soft_constraint = n.declare_parameter<bool>("soft_constraint", false);
      compensate = n.declare_parameter<bool>("compensate", true);
      cost_type = n.declare_parameter<std::string>("cost_type", "P2L");

      loss_type_ = n.declare_parameter<std::string>("loss_type", "Huber");
      loss_limit_ = n.declare_parameter<double>("loss_limit", 0.1);
      covar_scale_ = n.declare_parameter<double>("covar_scale", 1.0);
      regularization_ = n.declare_parameter<double>("regularization", 0.0);
      weight_intensity_ = n.declare_parameter<bool>("weight_intensity", false);
      publish_tf_ = n.declare_parameter<bool>("publish_tf", false);
    }

    std::string ToString() {
      std::ostringstream stringStream;
      stringStream << "input_points_topic, " << input_points_topic << endl;
      stringStream << "scan_registered_latest_topic, " << scan_registered_latest_topic << endl;
      stringStream << "scan_registered_keyframe_topic, " << scan_registered_keyframe_topic << endl;
      stringStream << "odom_latest_topic, " << odom_latest_topic << endl;
      stringStream << "odom_keyframe_topic, " << odom_keyframe_topic << endl;
      stringStream << "use raw pointcloud, " << std::boolalpha << use_raw_pointcloud << endl;
      stringStream << "submap keyframes, " << submap_scan_size << endl;
      stringStream << "resolution r," << res << endl;
      stringStream << "resample factor f, " << MapPointNormal::downsample_factor << endl;
      stringStream << "min. sensor distance [m], " << min_keyframe_dist_ << endl;
      stringStream << "min. sensor rot. [deg], " << min_keyframe_rot_deg_ << endl;
      stringStream << "use keyframe, " << std::boolalpha << use_keyframe << endl;
      stringStream << "use initial guess, " << std::boolalpha << use_guess << endl;
      stringStream << "disable registration, " << std::boolalpha << disable_registration << endl;
      stringStream << "soft velocity constraint, " << std::boolalpha << soft_constraint << endl;
      stringStream << "compensate, " << std::boolalpha << compensate << endl;
      stringStream << "cost type, " << cost_type << endl;
      stringStream << "loss type, " << loss_type_ << endl;
      stringStream << "loss limit, " << std::to_string(loss_limit_) << endl;
      stringStream << "covar scale, " << std::to_string(covar_scale_) << endl;
      stringStream << "regularization, " << std::to_string(regularization_) << endl;
      stringStream << "weight intensity, " << std::boolalpha << weight_intensity_ << endl;
      stringStream << "publish_tf, " << std::boolalpha << publish_tf_ << endl;
      stringStream << "Weight, " << weight_opt << endl;
      return stringStream.str();
    }
  };

 protected:
  Eigen::Affine3d Tcurrent, Tprev_fused, T_prev, Tmot;
  // Components for publishing
  std::shared_ptr<n_scan_normal_reg> radar_reg = nullptr;
  PoseScanVector keyframes_;

  unsigned int frame_nr_ = 0, nr_callbacks_ = 0;
  double distance_traveled = 0.0;
  const double Tsensor = 1.0 / 4.0;

  Parameters par;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_callback;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_rot_est;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pose_current_publisher;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pose_keyframe_publisher;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubsrc_cloud_latest;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_keyframe;
  std::unique_ptr<tf2_ros::TransformBroadcaster> Tbr;

 public:
  OdometryKeyframeFuser(const Parameters& pars, rclcpp::Node::SharedPtr node,
                        bool disable_callback = false);

  void pointcloudCallback(const pcl::PointCloud<pcl::PointXYZI>::Ptr& msg_in,
                          Eigen::Affine3d& Tcurr);
  void pointcloudCallback(const pcl::PointCloud<pcl::PointXYZI>::Ptr& msg_in, Eigen::Affine3d& Tcurr,
                          Eigen::Affine3d& Trot);
  void CallbackRot(const nav_msgs::msg::Odometry::ConstSharedPtr& msg);
  std::string GetStatus() {
    return "Distance traveled: " + std::to_string(distance_traveled) +
           ", nr sensor readings: " + std::to_string(frame_nr_);
  }

 private:
  bool AccelerationVelocitySanityCheck(const Eigen::Affine3d& Tmot_prev,
                                       const Eigen::Affine3d& Tmot_curr);

  bool KeyFrameBasedFuse(const Eigen::Affine3d& diff, bool use_keyframe, double min_keyframe_dist,
                         double min_keyframe_rot_deg);

  pcl::PointXYZI Transform(const Eigen::Affine3d& T, pcl::PointXYZI& p);

  nav_msgs::msg::Odometry FormatOdomMsg(const Eigen::Affine3d& T, const Eigen::Affine3d& Tmot,
                                        const rclcpp::Time& t, Matrix6d& Cov);

  pcl::PointCloud<pcl::PointXYZI> FormatScanMsg(pcl::PointCloud<pcl::PointXYZI>& cloud_in,
                                                Eigen::Affine3d& T);

  void processFrame(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, Eigen::Affine3d& Trot);

  void pointcloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg_in);
};

void AddToReference(PoseScanVector& reference, MapNormalPtr cloud, const Eigen::Affine3d& T,
                    size_t submap_scan_size);

void FormatScans(const PoseScanVector& reference, const MapNormalPtr& Pcurrent,
                 const Eigen::Affine3d& Tcurrent, std::vector<Matrix6d>& cov_vek,
                 std::vector<MapNormalPtr>& scans_vek, std::vector<Eigen::Affine3d>& T_vek);

}  // namespace lodestar_odom
