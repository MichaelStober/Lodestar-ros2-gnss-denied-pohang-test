#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <boost/program_options.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_filter.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "lodestar_odometry/eval_trajectory.h"
#include "lodestar_odometry/lodestar.h"
#include "lodestar_odometry/odometrykeyframefuser.h"

using namespace lodestar_odom;
namespace po = boost::program_options;

/** \brief Original code is based on CFEAR odometry by Daniel Adolfsson.
 * This is the revised version for estiating maritime radar odometry with lodestar descriptor.
 *
 * ROS 2 port notes:
 *  - rosbag::Bag/rosbag::View are replaced by rosbag2_cpp::Reader plus an
 *    explicit rclcpp::Serialization<sensor_msgs::msg::Image>, because rosbag2
 *    hands out serialized messages rather than typed ones.
 *  - All wall-clock profiling uses std::chrono::steady_clock. Mixing
 *    node->now() (ROS time) with durations is what breaks first when a bag is
 *    played with use_sim_time.
 *  - The process owns exactly one rclcpp::Node, created here and handed to
 *    every component; ROS 2 has no equivalent of the implicit global node
 *    handle that the ROS 1 version relied on.
 */

typedef struct eval_parameters_ {
  std::string bag_file_path = "";
  bool save_pcds = false;
} eval_parameters;

class radarReader {
 private:
  rclcpp::Node::SharedPtr node_;
  EvalTrajectory eval;
  lodestar driver;
  OdometryKeyframeFuser fuser;
  bool save = true;
  Eigen::Affine3d Toffset = Eigen::Affine3d::Identity();

 public:
  radarReader(rclcpp::Node::SharedPtr node,
              const OdometryKeyframeFuser::Parameters& odom_pars,
              const lodestar::Parameters& rad_pars,
              const EvalTrajectory::Parameters& eval_par,
              const eval_parameters& p)
      : node_(node),
        eval(eval_par, node, true),
        driver(rad_pars, node, true),
        fuser(odom_pars, node, true) {

    cout << "Loading bag from: " << p.bag_file_path << endl;

    rosbag2_cpp::Reader reader;
    reader.open(p.bag_file_path);  // directory containing metadata.yaml, or a .mcap/.db3 file

    rosbag2_storage::StorageFilter filter;
    filter.topics = {rad_pars.radar_topic};
    reader.set_filter(filter);

    rclcpp::Serialization<sensor_msgs::msg::Image> image_serialization;

    int frame = 0;
    std::chrono::steady_clock::duration tot{0};

    ///////////////////////////////   Bagfile Start ///////////////////////////////////////////
    while (reader.has_next()) {
      if (!rclcpp::ok())
        break;

      auto bag_message = reader.read_next();
      if (bag_message->topic_name != rad_pars.radar_topic)
        continue;

      auto image_msg = std::make_shared<sensor_msgs::msg::Image>();
      rclcpp::SerializedMessage serialized(*bag_message->serialized_data);
      image_serialization.deserialize_message(&serialized, image_msg.get());

      const auto tinit = std::chrono::steady_clock::now();
      pcl::PointCloud<pcl::PointXYZI>::Ptr lodestar_cloud(new pcl::PointCloud<pcl::PointXYZI>());
      Eigen::Affine3d Trot;
      Eigen::Vector3d dense_trans;
      driver.CallbackOffline(image_msg, lodestar_cloud, Trot,
                             dense_trans);  // Operating LodeStar based pointcloud generation
      Eigen::Matrix3d rotMat =
          Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()).toRotationMatrix();  // To initialize offset
      Trot.rotate(rotMat);                                                           // To initialize offset

      lodestar_odom::timing.Document("Filtered points", lodestar_cloud->size());
      Eigen::Affine3d Tcurrent;
      fuser.pointcloudCallback(lodestar_cloud, Tcurrent, Trot);

      Eigen::Affine3d T_final =
          Tcurrent * Trot;  // Tcurrent = Matrix from CFEAR, Trot = rotation from LodeStar

      const rclcpp::Time t(image_msg->header.stamp);
      if (eval_par.save_pcd)
        eval.CallbackESTEigen(std::make_pair(T_final, t), *lodestar_cloud);
      else
        eval.CallbackESTEigen(std::make_pair(T_final, t));

      const auto d = std::chrono::steady_clock::now() - tinit;
      tot += d;
      const double d_sec = std::chrono::duration<double>(d).count();
      const double tot_sec = std::chrono::duration<double>(tot).count();

      // NOTE: the ROS 1 version wrote `<< frame << ... << ++frame/tot.toSec()`
      // in a single stream expression. Under C++14 that was unsequenced and
      // therefore undefined behaviour; C++17 sequences `<<` operands
      // left-to-right, so the value printed can differ between the old and the
      // new build. Split out explicitly to keep it unambiguous.
      const int frame_index = frame;
      ++frame;
      cout << "Frame: " << frame_index << ", Odom duration: " << d_sec
           << "sec, avg: " << frame / tot_sec << " Hz " << endl;
      cout << "oooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo"
           << endl;
    }
    cout << fuser.GetStatus() << endl;
    reader.close();
    //lodestar_odom::timing.PresentStatistics();

    return;
  }

  void Save() {
    eval.Save();
    return;
  }

  ~radarReader() { return; }

  size_t GetSize() { return eval.GetSize(); }
};

void ReadOptions(const int argc, char** argv, OdometryKeyframeFuser::Parameters& par,
                 lodestar::Parameters& rad_par, lodestar_odom::EvalTrajectory::Parameters& eval_par,
                 eval_parameters& p) {

  po::options_description desc{"Options"};
  desc.add_options()
      ("help,h", "Help screen")
      ("res", po::value<double>()->default_value(3.5), "res")
      ("range-res", po::value<double>()->default_value(0.0438), "range resolution")
      ("min_distance", po::value<double>()->default_value(0.5), "min sensor distance")
      ("max_distance", po::value<double>()->default_value(2000), "mib sensor distance ")
      ("submap_scan_size", po::value<int>()->default_value(3), "submap_scan_size")
      ("weight_intensity", po::value<bool>()->default_value(true), "weight_intensity")
      ("job_nr", po::value<int>()->default_value(-1), "jobnr")
      ("registered_min_keyframe_dist", po::value<double>()->default_value(1.5), "registered_min_keyframe_dist")
      ("z-min", po::value<double>()->default_value(65), "zmin intensity, expected noise level")
      ("soft_constraint", po::value<bool>()->default_value(false), "soft_constraint")
      ("savepcd", "save_pcd_files")
      ("disable_compensate", po::value<bool>()->default_value(false), "disable_compensate")
      ("cost_type", po::value<std::string>()->default_value("P2L"), "P2L")
      ("loss_type", po::value<std::string>()->default_value("Huber"), "robust loss function eg. Huber Caunchy, None")
      ("loss_limit", po::value<double>()->default_value(0.1), "loss limit")
      ("covar_scale", po::value<double>()->default_value(1), "covar scale")  // Please fix combined parameter
      ("regularization", po::value<double>()->default_value(1), "regularization")
      ("est_directory", po::value<std::string>()->default_value(""), "output folder of estimated trajectory")
      ("sequence", po::value<std::string>()->default_value("2019-01-10-12-32-52-radar-oxford-10k"), "sequence contrained in \"bagfile\" to evaluate e.g. 2019-01-10-12-32-52-radar-oxford-10k")
      ("dataset", po::value<std::string>()->default_value("oxford"), "name of dataset, take special actions depending on radar file format etc")
      ("radar_topic", po::value<std::string>()->default_value("/radar_data"), "Radar topic name")
      ("k_nearest", po::value<int>()->default_value(20), "k_nearest points from the radar center")
      ("contour_threshold", po::value<int>()->default_value(50), "Threshold to extract the contour")
      ("method_name", po::value<std::string>()->default_value("method"), "method name")
      ("weight_option", po::value<int>()->default_value(0), "how to weight residuals")
      ("false-alarm-rate", po::value<float>()->default_value(0.01), "CA-CFAR false alarm rate")
      ("nb-guard-cells", po::value<int>()->default_value(10), "CA-CFAR nr guard cells")
      ("nb-window-cells", po::value<int>()->default_value(10), "CA-CFAR nr guard cells")
      ("bag_path", po::value<std::string>()->default_value(""), "ROS 2 bag to open: directory containing metadata.yaml, or a .mcap/.db3 file");

  po::variables_map vm;
  store(parse_command_line(argc, argv, desc), vm);
  notify(vm);

  if (vm.count("help"))
    std::cout << desc << '\n';
  if (vm.count("res"))
    par.res = vm["res"].as<double>();
  if (vm.count("min_distance"))
    rad_par.min_distance = vm["min_distance"].as<double>();
  if (vm.count("max_distance"))
    rad_par.max_distance = vm["max_distance"].as<double>();
  if (vm.count("job_nr"))
    eval_par.job_nr = vm["job_nr"].as<int>();
  if (vm.count("cost_type"))
    par.cost_type = vm["cost_type"].as<std::string>();
  if (vm.count("loss_type"))
    par.loss_type_ = vm["loss_type"].as<std::string>();
  if (vm.count("loss_limit"))
    par.loss_limit_ = vm["loss_limit"].as<double>();
  if (vm.count("covar_scale"))
    par.covar_scale_ = vm["covar_scale"].as<double>();
  if (vm.count("regularization"))
    par.regularization_ = vm["regularization"].as<double>();
  if (vm.count("submap_scan_size"))
    par.submap_scan_size = vm["submap_scan_size"].as<int>();
  if (vm.count("registered_min_keyframe_dist"))
    par.min_keyframe_dist_ = vm["registered_min_keyframe_dist"].as<double>();
  if (vm.count("est_directory"))
    eval_par.est_output_dir = vm["est_directory"].as<std::string>();
  if (vm.count("method_name"))
    eval_par.method = vm["method_name"].as<std::string>();
  if (vm.count("bag_path"))
    p.bag_file_path = vm["bag_path"].as<std::string>();
  if (vm.count("sequence"))
    eval_par.sequence = vm["sequence"].as<std::string>();
  if (vm.count("z-min"))
    rad_par.z_min = vm["z-min"].as<double>();
  if (vm.count("dataset"))
    rad_par.dataset = vm["dataset"].as<std::string>();
  if (vm.count("range-res"))
    rad_par.range_res = vm["range-res"].as<double>();
  if (vm.count("savepcd"))
    eval_par.save_pcd = true;
  if (vm.count("weight_option"))
    par.weight_opt = static_cast<weightoption>(vm["weight_option"].as<int>());
  if (vm.count("regularization"))
    rad_par.false_alarm_rate = vm["regularization"].as<double>();
  if (vm.count("covar_scale"))
    rad_par.window_size = vm["covar_scale"].as<double>();
  if (vm.count("radar_topic"))
    rad_par.radar_topic = vm["radar_topic"].as<std::string>();
  if (vm.count("k_nearest"))
    rad_par.k_nearest = vm["k_nearest"].as<int>();
  if (vm.count("contour_threshold"))
    rad_par.contour_threshold = vm["contour_threshold"].as<int>();

  par.weight_intensity_ = vm["weight_intensity"].as<bool>();
  par.compensate = !vm["disable_compensate"].as<bool>();
  par.use_guess = true;       //vm["soft_constraint"].as<bool>();
  par.soft_constraint = false; // soft constraint is rarely useful, this is changed for testing of initi // vm["soft_constraint"].as<bool>();
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("lodestar_odom_node");

  // The static visualization helpers in MapPointNormal need a node to publish on.
  MapPointNormal::SetNode(node);

  // Strip "--ros-args ..." before boost::program_options sees the command line,
  // otherwise `ros2 run lodestar_odometry lodestar_odom --ros-args ...` aborts
  // with "unrecognised option".
  std::vector<std::string> filtered_args = rclcpp::remove_ros_arguments(argc, argv);
  std::vector<char*> argv_filtered;
  argv_filtered.reserve(filtered_args.size());
  for (auto& a : filtered_args)
    argv_filtered.push_back(a.data());

  OdometryKeyframeFuser::Parameters odom_pars;
  lodestar::Parameters rad_pars;
  EvalTrajectory::Parameters eval_pars;
  eval_parameters eval_p;

  ReadOptions(static_cast<int>(argv_filtered.size()), argv_filtered.data(), odom_pars, rad_pars,
              eval_pars, eval_p);

  std::ofstream ofs_before(eval_pars.est_output_dir + std::string("../pars.txt"));  // Write
  std::string par_str_before = rad_pars.ToString() + odom_pars.ToString() + eval_pars.ToString() +
                               "nr_frames, " + std::to_string(0) + "\n" +
                               lodestar_odom::timing.GetStatistics();
  cout << "Odometry parameters:\n" << par_str_before << endl;
  ofs_before << par_str_before << endl;
  ofs_before.close();

  radarReader reader(node, odom_pars, rad_pars, eval_pars, eval_p);
  reader.Save();

  std::ofstream ofs(eval_pars.est_output_dir + std::string("../pars.txt"));  // Write
  std::string par_str = rad_pars.ToString() + odom_pars.ToString() + eval_pars.ToString() +
                        "\nnr_frames, " + std::to_string(reader.GetSize()) + "\n" +
                        lodestar_odom::timing.GetStatistics();
  ofs << par_str << endl;
  ofs.close();

  rclcpp::shutdown();
  return 0;
}
