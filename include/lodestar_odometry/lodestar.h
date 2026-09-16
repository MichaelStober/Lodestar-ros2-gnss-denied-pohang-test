#pragma once

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Eigen>

#include <opencv2/opencv.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include "lodestar_odometry/pointnormal.h"
#include "lodestar_odometry/statistics.h"

namespace lodestar_odom {

using std::cerr;
using std::cout;
using std::endl;

class lodestar {
 public:
  class Parameters {
   public:
    Parameters() {}
    float z_min = 60;  // min power
    float range_res = 0.05;
    int azimuths = 400;
    int nb_guard_cells = 20, window_size = 10;
    float false_alarm_rate = 0.01;
    float min_distance = 2.5, max_distance = 200;
    std::string radar_frameid = "sensor_est", topic_filtered = "/marine/Filtered";
    std::string dataset = "marine";
    std::string radar_topic = "/radar_data";
    int contour_threshold = 50;
    int k_nearest = 20;

    /**
     * Declare + read the parameters from a ROS 2 node.
     * Only used when the node is driven online; the offline bag reader in
     * lodestar_odom.cpp configures everything through boost::program_options.
     */
    void GetParametersFromRos(rclcpp::Node& n) {
      range_res = static_cast<float>(n.declare_parameter<double>("range_res", 0.0438));
      z_min = static_cast<float>(n.declare_parameter<double>("z_min", 60.0));
      min_distance = static_cast<float>(n.declare_parameter<double>("min_distance", 2.5));
      max_distance = static_cast<float>(n.declare_parameter<double>("max_distance", 130.0));
      topic_filtered = n.declare_parameter<std::string>("topic_filtered", "/marine/Filtered");
      radar_frameid = n.declare_parameter<std::string>("radar_frameid", "sensor_est");
      dataset = n.declare_parameter<std::string>("dataset", "marine");
      radar_topic = n.declare_parameter<std::string>("radar_topic_name", "/radar_data");
      contour_threshold = static_cast<int>(n.declare_parameter<int>("contour_threshold", 50));
      k_nearest = static_cast<int>(n.declare_parameter<int>("k_nearest", 20));
    }

    std::string ToString() {
      std::ostringstream stringStream;
      stringStream << "range res, " << range_res << endl;
      stringStream << "z min, " << z_min << endl;
      stringStream << "min distance, " << min_distance << endl;
      stringStream << "max distance, " << max_distance << endl;
      stringStream << "topic_filtered, " << topic_filtered << endl;
      stringStream << "radar_frameid, " << radar_frameid << endl;
      stringStream << "dataset, " << dataset << endl;
      stringStream << "nb guard cells, " << nb_guard_cells << endl;
      stringStream << "window size, " << window_size << endl;
      stringStream << "false alarm rate, " << false_alarm_rate << endl;

      return stringStream.str();
    }
  };

  lodestar(const Parameters& pars, rclcpp::Node::SharedPtr node, bool disable_callback = false);

  ~lodestar() {}

  void CallbackOffline(const sensor_msgs::msg::Image::ConstSharedPtr& marine_radar_img,
                       pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, Eigen::Affine3d& T,
                       Eigen::Vector3d& v);

 private:
  void InitAngles();

  void Callback(const sensor_msgs::msg::Image::ConstSharedPtr& marine_radar_img);

  void CallbackOxford(const sensor_msgs::msg::Image::ConstSharedPtr& marine_radar_img);

  pcl::PointCloud<pcl::PointXYZI>::Ptr toPointCloud(const cv::Mat& radar_img,
                                                    const double& range_resolution);

  cv::Mat polar_transform(const cv::Mat& cart_img);
  cv::Mat polarToNearPol(const cv::Mat& polar_img, const int& k);
  double rotationCorrection(const cv::Mat& img1, const cv::Mat& img2);
  double crossCorrelation(const std::vector<double>& a, const std::vector<double>& b);

  Parameters par;
  std::vector<float> sin_values;
  std::vector<float> cos_values;
  float max_distance_sqrd, min_distance_sqrd;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr FilteredPublisher;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr imgPublisher;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr rotPublisher;

  int initialize = 0;
  int count_d = 0;
  double acc_rots = 0.0;
  cv::Mat prev_img;
  cv::Mat curr_img;
  std::vector<cv::Mat> window_list, dense_list;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered;
  Eigen::Affine3d Trot;
  Eigen::Vector3d dense_trans = Eigen::Vector3d::Zero();
};

}  // namespace lodestar_odom
