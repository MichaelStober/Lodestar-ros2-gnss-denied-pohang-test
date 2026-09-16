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
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "ceres/autodiff_cost_function.h"
#include "ceres/ceres.h"
#include "ceres/manifold.h"

#include "angles/angles.h"

#include "lodestar_odometry/pointnormal.h"
#include "lodestar_odometry/utils.h"

namespace lodestar_odom {

class Registration;

typedef std::vector<std::pair<Eigen::Affine3d, pcl::PointCloud<pcl::PointXYZI> > > reference_scan;
typedef std::pair<int, int> int_pair;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
using regPtr = std::shared_ptr<lodestar_odom::Registration>;

typedef enum reg_mode { incremental_last_to_previous, many_to_many_refinement } regmode;

typedef enum weight_options {
  Uniform = 0,
  Sim_N = 1,
  Sim_direciton = 2,
  Sim_scale = 3,
  Combined_weights = 4
} weightoption;

const Matrix6d Identity66 = Matrix6d::Identity();

/* cost metric */
typedef enum costmetric { P2P, P2L, P2D } cost_metric;

cost_metric Str2Cost(const std::string& str);

/* robust loss function */
typedef enum losstype { None, Huber, Cauchy, SoftLOne, Combined, Tukey } loss_type;

std::string loss2str(const loss_type& loss);

loss_type Str2loss(const std::string& loss);

class Registration {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @param node  node used to advertise the debug/association markers.
   *              May be nullptr, in which case nothing is advertised.
   */
  explicit Registration(rclcpp::Node::SharedPtr node = nullptr);

  virtual ~Registration() = default;

  virtual bool Register(std::vector<MapNormalPtr>& scans, std::vector<Eigen::Affine3d>& Tsrc,
                        std::vector<Matrix6d>& reg_cov, bool soft_constraints = false) = 0;

  virtual double getScore();

  virtual std::string GetParameterString();

  void InitFixedBlocks(const size_t& nsize);

  void SetMode(const regmode mode) { mode_ = mode; }

  class Weights {
   public:
    Weights(double N1, double N2, double sim_dir, double plan1, double plan2)
        : N1_(N1), N2_(N2), sim_dir_(sim_dir), plan1_(plan1), plan2_(plan2) {}

    double GetWeight(const weightoption opt);

    double Similarity(const double x, const double y) { return 2 * std::min(x, y) / (x + y); }

    double N1_, N2_;
    double sim_dir_;
    double plan1_, plan2_;
  };

  weightoption weight_opt_ = weightoption::Uniform;
  std::map<int_pair, std::vector<Weights> > weight_associations_;

  std::map<int_pair, std::vector<int_pair> > scan_associations_;
  size_t itr_ = 0;

  ceres::Solver::Summary summary_;

 protected:
  ceres::LossFunction* GetLoss();

  cost_metric cost_ = P2L;
  loss_type loss_ = Huber;
  double loss_limit_ = 0.1;

  std::vector<MapNormalPtr> scans_;  // 0 = target, 1 = source
  double radius_ = 2.0;
  std::vector<std::vector<double> > parameters;
  std::vector<bool> fixedBlock_;
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<ceres::Problem> problem_;
  regmode mode_ = incremental_last_to_previous;

  ceres::Solver::Options options_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_association;
  double score_ = 0.0;
};

Eigen::Matrix3d getScaledRotationMatrix(const std::vector<double>& parameters, double factor);

Eigen::Vector3d getScaledTranslationVector(const std::vector<double>& parameters, double factor);

void normalizeEulerAngles(Eigen::Vector3d& euler);

Eigen::Affine3d vectorToAffine3d(double x, double y, double z, double ex, double ey, double ez);

Eigen::Affine3d vectorToAffine3d(const std::vector<double>& vek);

Eigen::Affine2d vectorToAffine2d(double x, double y, double ez);

void Affine3dToEigVectorXYeZ(const Eigen::Affine3d& T, Eigen::Vector3d& par);

void Affine3dToVectorXYeZ(const Eigen::Affine3d& T, std::vector<double>& par);

Eigen::Matrix3d Cov6to3(const Matrix6d& C);

}  // namespace lodestar_odom
