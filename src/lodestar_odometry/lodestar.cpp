#include "lodestar_odometry/lodestar.h"

namespace lodestar_odom {

lodestar::lodestar(const Parameters& pars, rclcpp::Node::SharedPtr node, bool disable_callback)
    : par(pars), node_(node) {

  min_distance_sqrd = par.min_distance*par.min_distance;
  max_distance_sqrd = par.max_distance*par.max_distance;
  Trot = Eigen::Affine3d::Identity();
  cloud_filtered = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());

  FilteredPublisher = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      PrivateTopic(par.topic_filtered), rclcpp::QoS(1000));
  imgPublisher = node_->create_publisher<sensor_msgs::msg::Image>(
      PrivateTopic("/radar_imported"), rclcpp::QoS(1000));
  rotPublisher = node_->create_publisher<nav_msgs::msg::Odometry>(
      PrivateTopic("/rot_lodestar"), rclcpp::QoS(1000));
  if(!disable_callback){
      sub = node_->create_subscription<sensor_msgs::msg::Image>(
          PrivateTopic(pars.radar_topic), rclcpp::QoS(1000),
          std::bind(&lodestar::Callback, this, std::placeholders::_1));
  }
}
/*
void lodestar::InitAngles(){
  sin_values.resize(par.azimuths);
  cos_values.resize(par.azimuths);
  for (int i=0;i<par.azimuths;i++){
    float theta=(float)(i+1)*2*M_PI/par.azimuths;
    sin_values[i]=std::sin(theta);
    cos_values[i]=std::cos(theta);
  }
}
*/
void lodestar::Callback(const sensor_msgs::msg::Image::ConstSharedPtr &marine_radar_img){
    if(!marine_radar_img){
        cerr<<"Radar image NULL"<<endl;
        exit(0);
    }
  const auto t0 = std::chrono::steady_clock::now();
  cv_bridge::CvImagePtr cv_marine_img;
  cv_marine_img = cv_bridge::toCvCopy(marine_radar_img, sensor_msgs::image_encodings::MONO8);
  cv_marine_img->header.stamp = marine_radar_img->header.stamp;

/////////////////////////////////////LodeSTAR///////////////////////////////////////////////////////////

const auto lodestar_start = std::chrono::steady_clock::now();
  //Generate polar image for cross correlation
  cv::Mat pol_img = polar_transform(cv_marine_img->image);
  window_list.push_back(pol_img);
  double angle = 0.0;
  if (initialize == 0){
    angle = rotationCorrection(pol_img,pol_img);//autocorrelation for the first image
    initialize += 1;
    //window_list.push_back(cv_marine_img->image);
  }
  else{
    angle = rotationCorrection(window_list.front(),window_list.back());
    window_list.erase(window_list.begin());
  }

  // Rotate the given image with our estimation
  cv::Point center_i = cv::Point(cv_marine_img->image.cols / 2, cv_marine_img->image.rows / 2);
  acc_rots += angle;
  cv::Mat rot_mat = cv::getRotationMatrix2D(center_i, acc_rots, 1);
  cv::Mat rotated_image;
  cv::warpAffine(cv_marine_img->image, cv_marine_img->image, rot_mat, cv_marine_img->image.size());
const double lodestar_time_ms = ToMsSince(lodestar_start);

cout << "Lodestar Computation time ================== " << lodestar_time_ms << " ms" << endl;

/////////////////////////////////////LodeSTAR///////////////////////////////////////////////////////////

 //Cart to Polar transformation
  cv::Point2f center(cv_marine_img->image.cols / 2.0F, cv_marine_img->image.rows / 2.0F);
  double maxRadius = cv::norm(cv::Point2f(cv_marine_img->image.cols - center.x, cv_marine_img->image.rows - center.y));
  cv::linearPolar(cv_marine_img->image, cv_marine_img->image, center, maxRadius, cv::WARP_FILL_OUTLIERS);
  cv_marine_img->image = cv_marine_img->image.t();
  cv::resize(cv_marine_img->image,cv_marine_img->image,cv::Size(), 1, 1, cv::INTER_NEAREST);
  uint16_t bins = cv_marine_img->image.rows;
  rotate(cv_marine_img->image, cv_marine_img->image, cv::ROTATE_90_COUNTERCLOCKWISE);



// Polar to nearest polar image 
  cloud_filtered = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
  cv::Mat img_th;

    const auto contour_start = std::chrono::steady_clock::now();
  cv::threshold(cv_marine_img->image,img_th, par.contour_threshold, 255, cv::THRESH_TOZERO); // Contour Extraction
    cout << "Contour Computation time =================== " << ToMsSince(contour_start) << " ms" << endl;

    const auto k_nearest_start = std::chrono::steady_clock::now();
  cv::Mat img = polarToNearPol(img_th,par.k_nearest); // K-Nearest feature
  //cv::Mat img = polarToNearPol(cv_marine_img->image, 10); // for K-Nearest feature only estimation
  //cv::Mat img = img_th; //for contour only estimation
    cout << "k_nearest Computation time ================= " << ToMsSince(k_nearest_start) << " ms" << endl;
    
    //imshow("lodestarred.",img);
    //imwrite("lck50.jpg", img);
    //cv::waitKey(0);

//Image to pointcloud conversion & publish for next step
  cloud_filtered = toPointCloud(img, par.range_res);
  cloud_filtered->header.frame_id = par.radar_frameid;
  const rclcpp::Time msg_stamp(marine_radar_img->header.stamp);
  const rclcpp::Time tstamp = msg_stamp.seconds() < 0.001 ? node_->now() : msg_stamp;
  pcl_conversions::toPCL(tstamp, cloud_filtered->header.stamp);

  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(*cloud_filtered, cloud_msg);
  cloud_msg.header.frame_id = par.radar_frameid;
  cloud_msg.header.stamp = tstamp;
  FilteredPublisher->publish(cloud_msg);

  // NOTE: the ROS 1 version passed "MONO8"; the canonical encoding string is
  // lower case, otherwise subscribers reject the image.
  sensor_msgs::msg::Image::SharedPtr img_msg =
      cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::MONO8, img).toImageMsg();
  img_msg->header.stamp = tstamp;
  img_msg->header.frame_id = par.radar_frameid;
  imgPublisher->publish(*img_msg);


//Publish rotation information from lodestar algorithm
  nav_msgs::msg::Odometry rot_msg;
  Eigen::Matrix3d rotMat;
  rotMat = Eigen::AngleAxisd(acc_rots*M_PI/180, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  Trot = Eigen::Affine3d::Identity();
  Trot.rotate(rotMat);
  rot_msg.header.stamp = node_->now();
  rot_msg.header.frame_id = "world";
  rot_msg.child_frame_id = "sensor";
  rot_msg.pose.pose = tf2::toMsg(Trot);

  rotPublisher->publish(rot_msg);
  lodestar_odom::timing.Document("Filtering", ToMsSince(t0));
}

double lodestar::rotationCorrection(const cv::Mat& img1, const cv::Mat& img2) {
    cv::Mat n_img1, n_img2;
    //cv::transpose(img1,n_img1);
    //cv::transpose(img2,n_img2);
    img1.copyTo(n_img1);
    img2.copyTo(n_img2);
    std::vector<double> img1_row(img1.cols, 0);
    std::vector<double> img2_row(img2.cols, 0);

    for (int i = 0; i < n_img1.rows; ++i) {
        for (int j = 0; j < n_img1.cols; ++j) {
            img1_row[j] += n_img1.at<uchar>(i, j);
            img2_row[j] += n_img2.at<uchar>(i, j);
        }
    }
    double shift = crossCorrelation(img1_row,img2_row);
    cout << "Rotation Shifts : " << shift << endl;
    return shift;
}

cv::Mat lodestar::polar_transform(const cv::Mat& cart_img) {
    if(cart_img.empty()) {
        std::cerr << "Could not read the radar image: " << std::endl;
        return cv::Mat();
    }

    cv::Mat gray;
    //cv::cvtColor(cart_img, gray, cv::COLOR_BGR2GRAY);
    cart_img.copyTo(gray);
    int range = gray.rows;
    int theta = 3600;
    cv::Mat pol_img = cv::Mat::zeros(range/2, theta, CV_8UC1);

    for(int r = 0; r < range/2; ++r) {
        for(int t = 0; t < theta; ++t) {
            int x = cvRound(range/2) - r * cos(t * 2 * CV_PI / theta);
            int y = cvRound(range/2) + r * sin(t * 2 * CV_PI / theta);
            x = cvRound(x);
            y = cvRound(y);

            if (x < range && y < range) {
                pol_img.at<uchar>(r,t) = gray.at<uchar>(x,y);
            }
        }
    }

    return pol_img;
}
double lodestar::crossCorrelation(const std::vector<double>& a, const std::vector<double>& b) {
    int n = a.size();
    std::vector<double> result(n, 0.0);

    for (int m = 0; m < n; ++m) {
        for (int i = 0; i < n; ++i) {
            int j = (i + m) % n; // for circular cross-correlation
            result[m] += a[i] * b[j];
        }
    }
    // Find the iterator of the maximum element
    auto max_it = std::max_element(result.begin(), result.end());

    // Subtract iterators to get the index
    int max_index = std::distance(result.begin(), max_it);
    double result_index = max_index*0.1;
    if(max_index>1800)
      result_index = (max_index-3600)*0.1;
    
    return result_index;
}


cv::Mat lodestar::polarToNearPol(const cv::Mat& polar_img, const int& k) {
    int range = polar_img.rows;
    int theta = polar_img.cols;

    cv::Mat polar_ref = cv::Mat::zeros(range, theta, polar_img.type());
//    polar_ref(cv::Range(120, 0), cv::Range::all()).copyTo(polar_img(cv::Range(120, 0), cv::Range::all()));

    for (int r = 0; r < range; ++r) {
        int non_zero_count = 0;
        for (int th = 0; th < theta; ++th) {
            if (polar_img.at<uchar>(r, th) != 0) {
                polar_ref.at<uchar>(r, th) = polar_img.at<uchar>(r, th);
                ++non_zero_count;
                if (non_zero_count == k) {
                    break;
                }
            }
        }
    }
    return polar_ref;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr lodestar::toPointCloud(const cv::Mat& radar_img, const double &range_resolution)
{
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  const int nb_ranges = radar_img.cols;
  const int nb_azimuths = radar_img.rows;
  //cout << "radar range pixel:  " << nb_ranges << endl;
  for(int azimuth_nb = 0; azimuth_nb < nb_azimuths; azimuth_nb++)
  {
    const double theta = ((double)(azimuth_nb + 1) / radar_img.rows) * 2. * M_PI;
    for(int range_bin = 20; range_bin < nb_ranges; range_bin++)
    {
      const double range = range_resolution * double(range_bin);
      const double intensity = (double)(radar_img.at<uchar>(azimuth_nb, range_bin));
      if(intensity > 0.)
      {
        pcl::PointXYZI p;
        p.x = range * std::cos(theta);
        p.y = range * std::sin(theta);
        p.intensity = intensity;
        p.z = 0;
        cloud->push_back(p);
      }
    }
  }
  return cloud;
}

void lodestar::CallbackOffline(const sensor_msgs::msg::Image::ConstSharedPtr& marine_radar_img,  pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, Eigen::Affine3d &T, Eigen::Vector3d &v){
  Callback(marine_radar_img);
  T = Trot;
  v = dense_trans;
  *cloud = *cloud_filtered;
  cloud->header = cloud_filtered->header;

}

}
