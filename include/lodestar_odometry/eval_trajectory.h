#pragma once

#include <filesystem>
#include <iostream>
#include <memory>
#include <numeric>
#include <string>
#include <vector>

#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/SVD>
#include <eigen3/Eigen/StdVector>

#include <pcl/common/transforms.h>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "lodestar_odometry/utils.h"

namespace lodestar_odom {

using std::cerr;
using std::cout;
using std::endl;
using std::string;

using poseStamped = std::pair<Eigen::Affine3d, rclcpp::Time>;
typedef std::vector<poseStamped, Eigen::aligned_allocator<poseStamped>> poseStampedVector;

class EvalTrajectory {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  class Parameters {
   public:
    Parameters() {}
    std::string est_output_dir = "";
    std::string sequence = "", method = "";
    std::string odom_est_topic = "";
    int job_nr = -1;
    bool save_pcd = false;
    bool synced_callback = true;

    void GetParametersFromRos(rclcpp::Node& n) {
      odom_est_topic = n.declare_parameter<std::string>("est_topic", "/lidar_odom");
      est_output_dir = n.declare_parameter<std::string>("est_output_dir", "");
      sequence = n.declare_parameter<std::string>("bag_name", "");
      method = n.declare_parameter<std::string>("method", "");
      save_pcd = n.declare_parameter<bool>("save_pcd", false);
      synced_callback = n.declare_parameter<bool>("synced_callback", true);
    }

    std::string ToString() {
      std::ostringstream stringStream;
      stringStream << "odom_est_topic, " << odom_est_topic << endl;
      stringStream << "est_output_dir, " << est_output_dir << endl;
      stringStream << "sequence, " << sequence << endl;
      stringStream << "job nr, " << job_nr << endl;
      stringStream << "save pcd, " << save_pcd << endl;
      stringStream << "method, " << method << endl;
      return stringStream.str();
    }
  };

  EvalTrajectory(const EvalTrajectory::Parameters& pars, rclcpp::Node::SharedPtr node,
                 bool disable_callback = false);

  std::string DatasetToSequence(const std::string& dataset);

  void CallbackEst(const nav_msgs::msg::Odometry::ConstSharedPtr& msg);

  void CallbackRot(const std_msgs::msg::Float32::ConstSharedPtr& msg);

  void Save();

  void CallbackESTEigen(const poseStamped& Test);

  void CallbackESTEigen(const poseStamped& Test, const pcl::PointCloud<pcl::PointXYZI>& cld);

  size_t GetSize() { return est_vek.size(); }

 private:
  void SavePCD(const std::string& folder);

  void PublishTrajectory(poseStampedVector& vek,
                         rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub);

  void Write(const std::string& path, const poseStampedVector& v, const poseStampedVector& r);

  poseStampedVector::iterator FindElement(const rclcpp::Time& t);

  Parameters par;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_est;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_rot_est;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_est;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud;
  std::unique_ptr<tf2_ros::TransformBroadcaster> br;

  poseStampedVector est_vek, rot_vek;
  Eigen::Affine3d rot_mat, est_mat;
  std::vector<pcl::PointCloud<pcl::PointXYZI>> clouds;
  pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled;
};

}  // namespace lodestar_odom
