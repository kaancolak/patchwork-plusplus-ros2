#include "patchwork_plusplus/patchwork_plusplus_node.hpp"

namespace patchwork_plusplus {

PatchworkPlusPlus::PatchworkPlusPlus(const rclcpp::NodeOptions &node_options)
    : Node("patchwork_plusplus", node_options) {
  using std::placeholders::_1;
  using std::chrono_literals::operator""ms;

  pc_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "cloud", rclcpp::SensorDataQoS());
  pc_ground_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "ground", rclcpp::SensorDataQoS());
  pc_nonground_publisher_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>(
          "nonground", rclcpp::SensorDataQoS());

  pc_cloud_subscriber_ =
      this->create_subscription<sensor_msgs::msg::PointCloud2>(
          "/sensing/lidar/concatenated/pointcloud", rclcpp::SensorDataQoS(),
          std::bind(&PatchworkPlusPlus::point_cloud_callback, this, _1));

  // Patchwork parameters
  sensor_height_ = this->declare_parameter("sensor_height", 1.8);
  num_iter_ = this->declare_parameter("num_iter", 3);
  num_lpr_ = this->declare_parameter("num_lpr", 20);
  num_min_pts_ = this->declare_parameter("num_min_pts", 10);
  th_seeds_ = this->declare_parameter("th_seeds", 0.4);
  th_dist_ = this->declare_parameter("th_dist", 0.3);
  th_seeds_v_ = this->declare_parameter("th_seeds_v", 0.4);
  th_dist_v_ = this->declare_parameter("th_dist_v", 0.3);
  max_range_ = this->declare_parameter("max_range", 80.0);
  min_range_ = this->declare_parameter("min_range", 2.7);
  uprightness_thr_ = this->declare_parameter("uprightness_thr", 0.5);
  adaptive_seed_selection_margin_ =
      this->declare_parameter("adaptive_seed_selection_margin", -1.1);
  RNR_ver_angle_thr_ = this->declare_parameter("RNR_ver_angle_thr", -15.0);
  RNR_intensity_thr_ = this->declare_parameter("RNR_intensity_thr", 0.2);
  max_flatness_storage_ = this->declare_parameter("max_flatness_storage", 1000);
  max_elevation_storage_ =
      this->declare_parameter("max_elevation_storage", 1000);
  enable_RNR_ = this->declare_parameter("enable_RNR", true);
  enable_RVPF_ = this->declare_parameter("enable_RVPF", true);
  enable_TGR_ = this->declare_parameter("enable_TGR", true);

  // CZM denotes 'Concentric Zone Model'. Please refer to our paper
  num_zones_ = this->declare_parameter("num_zones", 3);

  this->declare_parameter("num_sectors_each_zone", std::vector<int>{1, 2, 4});

  std::vector<long> temp_vec;
  this->get_parameter("num_sectors_each_zone", temp_vec);
  num_sectors_each_zone_.resize(temp_vec.size());
  for (int i = 0; i < temp_vec.size(); i++) {
    num_sectors_each_zone_[i] = temp_vec[i];
  }

  const auto temp_vec2 =
      this->declare_parameter<std::vector<long>>("mum_rings_each_zone");
  num_rings_each_zone_.resize(temp_vec2.size());
  for (int i = 0; i < temp_vec2.size(); i++) {
    std::cout << temp_vec2[i] << std::endl;
    num_rings_each_zone_[i] = temp_vec2[i];
  }

  elevation_thr_ =
      this->declare_parameter<std::vector<double>>("elevation_thresholds");
  flatness_thr_ =
      this->declare_parameter<std::vector<double>>("flatness_thresholds");

  show_parameters();

  PatchworkppGroundSeg_ = std::make_shared<PatchWorkpp<PointType>>(
      sensor_height_, num_iter_, num_lpr_, num_min_pts_, th_seeds_, th_dist_,
      th_seeds_v_, th_dist_v_, max_range_, min_range_, uprightness_thr_,
      adaptive_seed_selection_margin_, RNR_ver_angle_thr_, RNR_intensity_thr_,
      max_flatness_storage_, max_elevation_storage_, enable_RNR_, enable_RVPF_,
      enable_TGR_, num_zones_, num_sectors_each_zone_, num_rings_each_zone_,
      elevation_thr_, flatness_thr_);
}

void PatchworkPlusPlus::point_cloud_callback(
    const sensor_msgs::msg::PointCloud2::SharedPtr point_cloud) {

  pcl::PointCloud<PointType>::Ptr input_cloud(new pcl::PointCloud<PointType>);

  pcl::fromROSMsg(*point_cloud, *input_cloud);

  pcl::PointCloud<PointType>::Ptr ground_cloud(new pcl::PointCloud<PointType>);
  pcl::PointCloud<PointType>::Ptr nonground_cloud(
      new pcl::PointCloud<PointType>);

  double aa;
  PatchworkppGroundSeg_->estimate_ground(*input_cloud, *ground_cloud,
                                         *nonground_cloud, aa);

  sensor_msgs::msg::PointCloud2 ground_cloud_msg;
  pcl::toROSMsg(*ground_cloud, ground_cloud_msg);
  ground_cloud_msg.header.frame_id = "base_link";
  ground_cloud_msg.header.stamp = point_cloud->header.stamp;

  pc_ground_publisher_->publish(ground_cloud_msg);

  sensor_msgs::msg::PointCloud2 nonground_cloud_msg;
  pcl::toROSMsg(*nonground_cloud, nonground_cloud_msg);
  nonground_cloud_msg.header.frame_id = "base_link";
  nonground_cloud_msg.header.stamp = point_cloud->header.stamp;
  pc_nonground_publisher_->publish(nonground_cloud_msg);
}

void PatchworkPlusPlus::show_parameters() {
  RCLCPP_INFO(this->get_logger(), "sensor_height: %f", sensor_height_);
  RCLCPP_INFO(this->get_logger(), "num_iter: %d", num_iter_);
  RCLCPP_INFO(this->get_logger(), "num_lpr: %d", num_lpr_);
  RCLCPP_INFO(this->get_logger(), "num_min_pts: %d", num_min_pts_);
  RCLCPP_INFO(this->get_logger(), "th_seeds: %f", th_seeds_);
  RCLCPP_INFO(this->get_logger(), "th_dist: %f", th_dist_);
  RCLCPP_INFO(this->get_logger(), "th_seeds_v: %f", th_seeds_v_);
  RCLCPP_INFO(this->get_logger(), "th_dist_v: %f", th_dist_v_);
  RCLCPP_INFO(this->get_logger(), "max_range: %f", max_range_);
  RCLCPP_INFO(this->get_logger(), "min_range: %f", min_range_);
  RCLCPP_INFO(this->get_logger(), "uprightness_thr: %f", uprightness_thr_);
  RCLCPP_INFO(this->get_logger(), "adaptive_seed_selection_margin: %f",
              adaptive_seed_selection_margin_);
  RCLCPP_INFO(this->get_logger(), "RNR_ver_angle_thr: %f", RNR_ver_angle_thr_);
  RCLCPP_INFO(this->get_logger(), "RNR_intensity_thr: %f", RNR_intensity_thr_);
  RCLCPP_INFO(this->get_logger(), "max_flatness_storage: %d",
              max_flatness_storage_);
  RCLCPP_INFO(this->get_logger(), "max_elevation_storage: %d",
              max_elevation_storage_);
  RCLCPP_INFO(this->get_logger(), "enable_RNR: %d", enable_RNR_);
  RCLCPP_INFO(this->get_logger(), "enable_RVPF: %d", enable_RVPF_);
  RCLCPP_INFO(this->get_logger(), "enable_TGR: %d", enable_TGR_);
  RCLCPP_INFO(this->get_logger(), "num_zones: %d", num_zones_);
  RCLCPP_INFO(this->get_logger(), "num_sectors_each_zone: ");
  for (int i = 0; i < num_sectors_each_zone_.size(); i++) {
    RCLCPP_INFO(this->get_logger(), "%d", num_sectors_each_zone_[i]);
  }
  RCLCPP_INFO(this->get_logger(), "num_rings_each_zone: ");
  for (int i = 0; i < num_rings_each_zone_.size(); i++) {
    RCLCPP_INFO(this->get_logger(), "%d", num_rings_each_zone_[i]);
  }
  RCLCPP_INFO(this->get_logger(), "elevation_thr: ");
  for (int i = 0; i < elevation_thr_.size(); i++) {
    RCLCPP_INFO(this->get_logger(), "%f", elevation_thr_[i]);
  }
  RCLCPP_INFO(this->get_logger(), "flatness_thr: ");
  for (int i = 0; i < flatness_thr_.size(); i++) {
    RCLCPP_INFO(this->get_logger(), "%f", flatness_thr_[i]);
  };
}

} // namespace patchwork_plusplus

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(patchwork_plusplus::PatchworkPlusPlus)