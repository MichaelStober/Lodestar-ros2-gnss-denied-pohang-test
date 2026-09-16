#include "lodestar_odometry/eval_trajectory.h"

namespace lodestar_odom {


EvalTrajectory::EvalTrajectory(const EvalTrajectory::Parameters& pars, rclcpp::Node::SharedPtr node, bool disable_callback)
  : par(pars), node_(node), downsampled(new pcl::PointCloud<pcl::PointXYZI>()){

  if(!disable_callback){
    assert(!par.odom_est_topic.empty());
    if(par.synced_callback){
      // The ROS 1 version used message_filters::Synchronizer here, but the
      // registration was already commented out upstream, so there is nothing
      // to port.
    }
    else{
      sub_rot_est = node_->create_subscription<std_msgs::msg::Float32>(
          PrivateTopic("/rot_lodestar"), rclcpp::QoS(1000),
          std::bind(&EvalTrajectory::CallbackRot, this, std::placeholders::_1));
      sub_est = node_->create_subscription<nav_msgs::msg::Odometry>(
          PrivateTopic(par.odom_est_topic), rclcpp::QoS(1000),
          std::bind(&EvalTrajectory::CallbackEst, this, std::placeholders::_1));
    }
  }
  pub_est = node_->create_publisher<nav_msgs::msg::Path>(PrivateTopic("path_est"), rclcpp::QoS(10));
  pub_cloud = node_->create_publisher<sensor_msgs::msg::PointCloud2>(PrivateTopic("map_cloud"), rclcpp::QoS(10));
  br = std::make_unique<tf2_ros::TransformBroadcaster>(node_);

}


void EvalTrajectory::CallbackESTEigen(const poseStamped& Test){
  est_vek.push_back(Test);
}
void EvalTrajectory::CallbackESTEigen(const poseStamped& Test, const pcl::PointCloud<pcl::PointXYZI>& cld){
  clouds.push_back(cld);
  CallbackESTEigen(Test);
}

void EvalTrajectory::CallbackEst(const nav_msgs::msg::Odometry::ConstSharedPtr &msg){
  Eigen::Affine3d T;
  tf2::fromMsg(msg->pose.pose, T);
  //T = T*rot_vek;
  rclcpp::Time t(msg->header.stamp);
  est_vek.push_back(std::make_pair(T, t));
  //est_mat = T;
}

void EvalTrajectory::CallbackRot(const std_msgs::msg::Float32::ConstSharedPtr &msg){
  //rot_vek.push_back(msg_rot->data);
  cout << "callback" << endl;
  Eigen::Matrix3d rotationMatrix;
    rotationMatrix =
        Eigen::AngleAxisd(msg->data*M_PI/180, Eigen::Vector3d::UnitZ()).toRotationMatrix();

    // Now convert it into an Affine3d transformation.
    Eigen::Affine3d transform = Eigen::Affine3d::Identity();
    transform.rotate(rotationMatrix);
    rclcpp::Time t = node_->now();
  rot_vek.push_back(std::make_pair(transform, t));
  //rot_mat = transform;
}


void EvalTrajectory::Write(const std::string& path, const poseStampedVector& v,const poseStampedVector& r){
  std::ofstream evalfile;
  cout<<"Saving: "<<v.size()<<" poses to file: "<<path<<endl;
  //cout<<"With: "<<r.size()<<" rotations "<<endl;
  evalfile.open(path);
  for(size_t i=0;i<v.size();i++){
    Eigen::MatrixXd m(v[i].first.matrix());
    evalfile<< std::fixed << std::showpoint;
    assert(m.rows()== 4 && m.cols()==4);
//(2778m/2048px)/(0.05m/px)*0.75 = 20.346679687 // seadronix
//(1654.8/2048px)/0.05 = 16.16015625 // pohang
    evalfile<< m(0,0) <<" "<< m(0,1) <<" "<< m(0,2) <<" "<< m(0,3)*22.73 <<" "<<
               m(1,0) <<" "<< m(1,1) <<" "<< m(1,2) <<" "<< m(1,3)*22.73 <<" "<<
               m(2,0) <<" "<< m(2,1) <<" "<< m(2,2) <<" "<< m(2,3) <<std::endl;
  }
  evalfile.close();
  return;
}



std::string EvalTrajectory::DatasetToSequence(const std::string& dataset){
  return "01.txt";
}

void EvalTrajectory::PublishTrajectory(poseStampedVector& vek, rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub){
  nav_msgs::msg::Path path;
  path.header.frame_id="world";
  path.header.stamp = node_->now();

  for (size_t i=0;i<vek.size();i++) {
    Eigen::Affine3d T = vek[i].first;
    geometry_msgs::msg::PoseStamped Tstamped;
    Tstamped.pose = tf2::toMsg(T);
    path.poses.push_back(Tstamped);
  }
  pub->publish(path);
}

void EvalTrajectory::SavePCD(const std::string& folder){
  for (size_t i=0;i<clouds.size();i++) {
    pcl::PointCloud<pcl::PointXYZI> cld_transformed;
    pcl::transformPointCloud(clouds[i], cld_transformed, est_vek[i].first);
    pcl::io::savePCDFileBinary(folder+"cloud_"+std::to_string(i)+std::string(".pcd"), cld_transformed);
  }
}

void EvalTrajectory::Save(){
  cout << "Saving, outpout: " << par.est_output_dir << std::endl;
  if(est_vek.empty()){
    cout<<"Nothing estimated"<<endl;
    cerr<<"array size error. est_vek.size()="<<est_vek.size()<<endl;
    exit(0);
  }
  else{
    std::filesystem::create_directories(par.est_output_dir);
    std::string est_path = par.est_output_dir+DatasetToSequence(par.sequence);
    cout<<"Saving estimated "<<est_vek.size()<<" poses"<<endl;
    cout<<"To path: "<<est_path<<endl;
    if(par.save_pcd)
      SavePCD(par.est_output_dir);
    Write(est_path, est_vek, rot_vek);
    cout<<"Trajectoy saved"<<endl;
    return;
    
  }
  return;
}




}
