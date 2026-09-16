#pragma once

#include <time.h>

#include <cstdio>
#include <fstream>
#include <list>
#include <map>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

#include <Eigen/Eigen>

#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl_conversions/pcl_conversions.h>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "tgmath.h"

#include "lodestar_odometry/statistics.h"
#include "lodestar_odometry/utils.h"

namespace lodestar_odom {

using std::cerr;
using std::cout;
using std::endl;

class cell;
using cellptr = std::shared_ptr<cell>;

class cell {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  cell(const pcl::PointCloud<pcl::PointXYZI>::Ptr input, const std::vector<int>& pointIdxNKNSearch,
       const bool weight_intensity, const Eigen::Vector2d& origin = Eigen::Vector2d(0, 0));

  cell TransformCopy(const Eigen::Affine2d& T);

  double GetAngle();

  double GetPlanarity() { return scale_; }

  static cell GetIdentityCell(const Eigen::Vector2d& u, const double intensity) {
    return cell(u, intensity);
  }  // Use only for raw data

  Eigen::Vector2d u_ = Eigen::Vector2d(0, 0);
  Eigen::Matrix2d cov_ = Eigen::Matrix2d::Identity() * 0.1;
  double scale_;
  Eigen::Vector2d snormal_, orth_normal;
  double lambda_min, lambda_max;
  double sum_intensity_, avg_intensity_;
  size_t Nsamples_;
  bool valid_;

 private:
  cell(const Eigen::Vector2d& u, const double intensity)
      :  // Only for raw data
        u_(u),
        scale_(1.0),
        snormal_(1, 0),
        orth_normal(0, 1),
        lambda_min(1),
        lambda_max(1),
        sum_intensity_(1.0),
        avg_intensity_(1.0),
        Nsamples_(1),
        valid_(true) {}

  bool ComputeNormal(const Eigen::Vector2d& origin);
};

class MapPointNormal;
using MapNormalPtr = std::shared_ptr<MapPointNormal>;

class MapPointNormal {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  MapPointNormal(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cld, float radius,
                 const Eigen::Vector2d& origin = Eigen::Vector2d(0, 0),
                 const bool weight_intensity = false, const bool raw = false);

  MapPointNormal(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cld, float radius,
                 std::vector<cell>& cell_orig, const Eigen::Affine3d& T);

  MapPointNormal() {
    input_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
    downsampled_ = pcl::PointCloud<pcl::PointXY>::Ptr(new pcl::PointCloud<pcl::PointXY>());
  }

  std::vector<cell> GetCells() { return cells; }

  cell& GetCell(const size_t i) { return cells[i]; }

  std::vector<cell*> GetClosest(Eigen::Vector2d& p, double d);

  std::vector<int> GetClosestIdx(const Eigen::Vector2d& p, double d);

  rclcpp::Time GetTime() {
    rclcpp::Time t;
    pcl_conversions::fromPCL(input_->header.stamp, t);
    return t;
  }

  double GetCellRelTimeStamp(const size_t index);

  void Compensate(const Eigen::Affine3d& Tmot);

  std::vector<cell> TransformCells(const Eigen::Affine3d& T);

  void Transform(const Eigen::Affine3d& T, Eigen::MatrixXd& means, Eigen::MatrixXd& normals);

  void Transform2d(const Eigen::Affine3d& T, Eigen::MatrixXd& means, Eigen::MatrixXd& normals);

  Eigen::MatrixXd GetNormals2d();

  std::vector<Eigen::Matrix2d> GetCovs();

  Eigen::Vector2d GetMean2d(const size_t i) { return cells[i].u_.block<2, 1>(0, 0); }

  Eigen::Matrix2d GetCov2d(const size_t i) { return cells[i].cov_.block<2, 2>(0, 0); }

  Eigen::Vector2d GetNormal2d(const size_t i) { return cells[i].snormal_.block<2, 1>(0, 0); }

  Eigen::MatrixXd GetMeans2d();

  Eigen::MatrixXd GetCloudTransformed(const Eigen::Affine3d& Toffset);

  Eigen::MatrixXd GetScales();

  MapNormalPtr TransformMap(const Eigen::Affine3d& T);

  pcl::PointCloud<pcl::PointXYZI>::Ptr GetScan() { return input_; }

  size_t GetSize() { return cells.size(); }

 private:
  MapPointNormal(pcl::PointCloud<pcl::PointXYZI>& cld);

  void ComputeSearchTreeFromCells();

  void ComputeNormals(const Eigen::Vector2d& origin);

  inline double Gausian(const double x, const double u, const double sigma);

  // MEMBER VARIABLES
  std::vector<cell> cells;
  pcl::PointCloud<pcl::PointXYZI>::Ptr input_ = nullptr;
  pcl::PointCloud<pcl::PointXY>::Ptr downsampled_ = nullptr;
  pcl::KdTreeFLANN<pcl::PointXY> kd_cells;
  float radius_;
  bool weight_intensity_ = false;

 public:
  static void PublishMap(const std::string& topic, MapNormalPtr map, Eigen::Affine3d& T,
                         const std::string& frame_id, const int value = 0, float alpha = 1.0);

  static void PublishDataAssociationsMap(
      const std::string& topic,
      const std::vector<std::tuple<Eigen::Vector2d, Eigen::Vector2d, double, int> >& vis_residuals);

  /**
   * ROS 2 has no global node handle, so the static visualization helpers above
   * need a node to create their publishers on. main() installs the one node of
   * this process here before any map is published; when it is left unset the
   * helpers simply do nothing instead of crashing.
   */
  static void SetNode(rclcpp::Node::SharedPtr node) { node_ = node; }

  static rclcpp::Node::SharedPtr GetNode() { return node_; }

  static std::map<std::string, rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr> pubs;

  static double downsample_factor;

 private:
  static rclcpp::Node::SharedPtr node_;
};

visualization_msgs::msg::Marker DefaultMarker(const rclcpp::Time& time, const std::string& frame);

visualization_msgs::msg::MarkerArray Cells2Markers(std::vector<cell>& cells, const rclcpp::Time& time,
                                                   const std::string& frame, const int val = 0,
                                                   float alpha = 1);

void intToRGB(int value, float& red, float& green, float& blue);

inline pcl::PointXYZI Pnt(Eigen::Vector2d& u);

inline pcl::PointXYZ PntXYZ(Eigen::Vector2d& u);

inline geometry_msgs::msg::Point Pntgeom(Eigen::Vector2d& u);

}  // namespace lodestar_odom
