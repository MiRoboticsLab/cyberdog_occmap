// Copyright (c) 2023 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <yaml-cpp/yaml.h>

#include <chrono>
#include <string>
#include <vector>
#include <memory>

#include "occ_gridmap/create_gridmap.hpp"
#include "cyberdog_common/cyberdog_log.hpp"

using namespace std::chrono_literals;

CreateGridMap::CreateGridMap()
: Node("create_gridmap_node")
{
  this->declare_parameter<std::string>("config_file");
  this->get_parameter<std::string>("config_file", config_file_);
  ParseParameters(config_file_);

  create_srv_ = this->create_service<std_srvs::srv::SetBool>(
    topic_param_.create_service, std::bind(
      &CreateGridMap::CreateServiceCallback, this, std::placeholders::_1, std::placeholders::_2),
    rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile().get_rmw_qos_profile());

  finish_srv_ = this->create_service<cyberdog_visions_interfaces::srv::FinishMap>(
    topic_param_.finish_service, std::bind(
      &CreateGridMap::FinishServiceCallback, this, std::placeholders::_1, std::placeholders::_2),
    rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile().get_rmw_qos_profile());

  visualization_publisher_ = this->create_publisher<std_msgs::msg::Bool>(
    topic_param_.visualization_topic,
    rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile());
  pub_visualization_timer_ =
    this->create_wall_timer(1s, std::bind(&CreateGridMap::PublishVisualizationCallback, this));

  get_map_path_client_ = this->create_client<std_srvs::srv::Trigger>(topic_param_.get_map_path);
}

void CreateGridMap::ParseParameters(std::string & config_file)
{
  YAML::Node yaml_node = YAML::LoadFile(config_file);
  YAML::Node params = yaml_node["create_gridmap_node"];

  files_save_param_.create_map_path = params["create_map_path"].as<std::string>(
    "/SDCARD/miloc/maps/lidar_map_1/");
  files_save_param_.loop_pose_path = params["loop_pose_path"].as<std::string>(
    "/SDCARD/miloc/maps/default/viusal/");
  calibr_file_ = params["calibration_file"].as<std::string>("/SDCARD/calibrate.yaml");

  INFO("create map path %s", files_save_param_.create_map_path.c_str());
  INFO("loop pose path %s", files_save_param_.loop_pose_path.c_str());
  INFO("calibration_file %s", calibr_file_.c_str());

  // from yaml file
  visualization_flag_ = params["visualization_flag"].as<bool>(false);
  map_mode_ = params["map_mode"].as<std::string>("single");
  topic_param_.frame_id = params["frame_id"].as<std::string>("odom_slam");
  topic_param_.odom_topic = params["odom_topic"].as<std::string>("odom_slam");
  topic_param_.laser_topic = params["laser_topic"].as<std::string>("scan");
  topic_param_.range_data_topic = params["range_data_topic"].as<std::string>("range_data");
  topic_param_.visualization_topic = params["visualization_topic"].as<std::string>(
    "occmap_visualization");
  topic_param_.create_service = params["create_service"].as<std::string>("create_map_service");
  topic_param_.finish_service = params["finish_service"].as<std::string>("finish_map_service");
  topic_param_.start_nav_service = params["start_nav_service"].as<std::string>("start_nav_service");
  topic_param_.stop_nav_service = params["stop_nav_service"].as<std::string>("stop_nav_service");
  topic_param_.get_map_path = params["get_map_path"].as<std::string>("get_map_path");

  INFO("subscribe odom topic %s", topic_param_.odom_topic.c_str());
  INFO("subscribe laser topic %s", topic_param_.laser_topic.c_str());
  INFO("create map service %s", topic_param_.create_service.c_str());
  INFO("finish map service %s", topic_param_.finish_service.c_str());
  INFO("start_nav_service %s", topic_param_.start_nav_service.c_str());
  INFO("stop_nav_service %s", topic_param_.stop_nav_service.c_str());

  // Load filter params
  filter_param_.max_angle_radians = params["filter_param"]["max_angle_radians"].as<double>(0.0175);
  filter_param_.max_distance_meters = params["filter_param"]["max_distance_meters"].as<double>(0.2);
  filter_param_.max_length = params["filter_param"]["max_length"].as<double>(0.05);
  filter_param_.max_time_seconds = params["filter_param"]["max_time_seconds"].as<double>(160);
  filter_param_.min_num_points = params["filter_param"]["min_num_points"].as<double>(200);
  filter_param_.voxel_filter_size = params["filter_param"]["voxel_filter_size"].as<double>(0.025);

  // Load ceres params
  ceres_param_.use_nonmonotonic_steps = params["ceres_param"]["use_nonmonotonic_steps"].as<bool>(
    false);
  ceres_param_.max_num_iterations = params["ceres_param"]["max_num_iterations"].as<int>(20);
  ceres_param_.num_threads = params["ceres_param"]["num_threads"].as<int>(1);
  ceres_param_.occupied_space_weight = params["ceres_param"]["occupied_space_weight"].as<double>(
    10.0);
  ceres_param_.translation_weight = params["ceres_param"]["translation_weight"].as<double>(20.0);
  ceres_param_.rotation_weight = params["ceres_param"]["rotation_weight"].as<double>(40.0);

  // Load gridmap params
  probability_param_.p_occ = params["probability_param"]["p_occ"].as<double>(0.55);
  probability_param_.p_free = params["probability_param"]["p_free"].as<double>(0.45);
  probability_param_.p_prior = params["probability_param"]["p_prior"].as<double>(0.5);

  submap_param_.resolution = params["submap_param"]["resolution"].as<double>(0.05);
  submap_param_.sizex = params["submap_param"]["sizex"].as<int>(100);
  submap_param_.sizey = params["submap_param"]["sizey"].as<int>(100);
  submap_param_.initx = params["submap_param"]["initx"].as<int>(100);
  submap_param_.inity = params["submap_param"]["inity"].as<int>(100);
  submap_param_.num_accumulated = params["submap_param"]["num_accumulated"].as<int>(35);
  submap_param_.missing_data_ray_length =
    params["submap_param"]["missing_data_ray_length"].as<double>(6.0);
  submap_param_.max_range = params["submap_param"]["max_range"].as<double>(8.0);
  submap_param_.min_range = params["submap_param"]["min_range"].as<double>(0.2);

  // Load map beauti params
  mapbeauti_param_.sideFillThresh = params["mapbeauti"]["side_fill_thresh"].as<double>();
  mapbeauti_param_.approxPolythresh = params["mapbeauti"]["approx_poly_thresh"].as<double>();
  mapbeauti_param_.dilateKernelSize = params["mapbeauti"]["dilate_kernel_size"].as<int>();
  mapbeauti_param_.noiseRemovalThresh = params["mapbeauti"]["noise_removal_thresh"].as<int>();

  // Load tf
  std::vector<double> tf_laserq =
    params["laser_to_baselink"]["quaternion"].as<std::vector<double>>();
  std::vector<double> tf_laserp = params["laser_to_baselink"]["position"].as<std::vector<double>>();

  auto q_laser2baselink =
    Eigen::Quaterniond(tf_laserq[3], tf_laserq[0], tf_laserq[1], tf_laserq[2]);
  auto p_laser2baselink =
    Eigen::Vector3d(tf_laserp[0], tf_laserp[1], tf_laserp[2]);

  eigentf_.laser2baselink = transform::Rigid3d(p_laser2baselink, q_laser2baselink);
}


CreateGridMap::~CreateGridMap() {}

void CreateGridMap::CreateServiceCallback(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  if ("multiple" == map_mode_) {
    auto get_path_request = std::make_shared<std_srvs::srv::Trigger::Request>();
    while (!get_map_path_client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        WARN("Interrupted while waiting for the service. Exiting...");
      }
      INFO("Service not available, waiting again...");
    }
    auto future_result = get_map_path_client_->async_send_request(
      get_path_request, std::bind(&CreateGridMap::GetMapPathCallback, this, std::placeholders::_1));
  } else if ("single" == map_mode_) {
    INFO("accept create gridmap, current in CREATE_MAP_MODEL!");
    RemoveMap();
    files_save_param_.sub_range_data_path = files_save_param_.create_map_path + "/sub_range_data/";
    if (boost::filesystem::exists(files_save_param_.sub_range_data_path)) {
      boost::filesystem::remove_all(files_save_param_.sub_range_data_path);
    }
    if (common::createFolder(files_save_param_.sub_range_data_path)) {
      INFO("create path: %s", files_save_param_.sub_range_data_path.c_str());
    }
    g_mapper_ = std::make_shared<mapping::GridMapper>(
      eigentf_, probability_param_,
      submap_param_, files_save_param_, filter_param_, ceres_param_, mapbeauti_param_);
    odom_subscription_.subscribe(
      this, topic_param_.odom_topic,
      rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile().get_rmw_qos_profile());
    laser_subscription_.subscribe(
      this, topic_param_.laser_topic,
      rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile().get_rmw_qos_profile());

    approximate_sync_ = std::make_shared<message_filters::Synchronizer<mySyncPolicy>>(
      mySyncPolicy(10), odom_subscription_, laser_subscription_
    );
    approximate_sync_->registerCallback(
      std::bind(
        &CreateGridMap::SubCallback, this,
        std::placeholders::_1, std::placeholders::_2));
    grid_map_status_ = GridMapStatus::CREATE_MAP_MODEL;
    range_data_publisher_ = this->create_publisher<cyberdog_visions_interfaces::msg::RangeData>(
      topic_param_.range_data_topic,
      rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile());

    create_map_flag_ = true;
    range_data_publisher_ = this->create_publisher<cyberdog_visions_interfaces::msg::RangeData>(
      topic_param_.range_data_topic,
      rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile());
  }
}

void CreateGridMap::GetMapPathCallback(rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future)
{
  INFO("Got result: [%s]", future.get()->message.c_str());
  if (future.get()->success) {
    files_save_param_.create_map_path = future.get()->message;
  }

  if (grid_map_status_ != GridMapStatus::HANG_MODEL) {
    WARN("cannot run create_map, current not in HANG_MODEL!");
    return;
  }

  INFO("accept create gridmap, current in CREATE_MAP_MODEL!");
  files_save_param_.sub_range_data_path = files_save_param_.create_map_path + "/sub_range_data/";
  if (boost::filesystem::exists(files_save_param_.sub_range_data_path)) {
    boost::filesystem::remove_all(files_save_param_.sub_range_data_path);
  }
  if (common::createFolder(files_save_param_.sub_range_data_path)) {
    INFO("create path: %s", files_save_param_.sub_range_data_path.c_str());
  }
  g_mapper_ = std::make_shared<mapping::GridMapper>(
    eigentf_, probability_param_,
    submap_param_, files_save_param_, filter_param_, ceres_param_, mapbeauti_param_);
  odom_subscription_.subscribe(
    this, topic_param_.odom_topic,
    rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile().get_rmw_qos_profile());
  laser_subscription_.subscribe(
    this, topic_param_.laser_topic,
    rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile().get_rmw_qos_profile());

  approximate_sync_ = std::make_shared<message_filters::Synchronizer<mySyncPolicy>>(
    mySyncPolicy(10), odom_subscription_, laser_subscription_
  );
  approximate_sync_->registerCallback(
    std::bind(
      &CreateGridMap::SubCallback, this,
      std::placeholders::_1, std::placeholders::_2));
  grid_map_status_ = GridMapStatus::CREATE_MAP_MODEL;
  range_data_publisher_ = this->create_publisher<cyberdog_visions_interfaces::msg::RangeData>(
    topic_param_.range_data_topic,
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile());

  create_map_flag_ = true;
  range_data_publisher_ = this->create_publisher<cyberdog_visions_interfaces::msg::RangeData>(
    topic_param_.range_data_topic,
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile());
}

void CreateGridMap::FinishServiceCallback(
  const std::shared_ptr<cyberdog_visions_interfaces::srv::FinishMap::Request> request,
  std::shared_ptr<cyberdog_visions_interfaces::srv::FinishMap::Response> response)
{
  range_data_publisher_.reset();
  create_map_flag_ = false;

  if (grid_map_status_ != GridMapStatus::CREATE_MAP_MODEL) {
    WARN(
      "cannot use finish_map_service, please use create_map_service first!");
    return;
  }
  if (!request->finish || request->map_name.empty()) {
    INFO("Won't save map.");
    grid_map_status_ = GridMapStatus::HANG_MODEL;
    g_mapper_.reset();
    files_save_param_.sub_range_data_path = files_save_param_.create_map_path + "/sub_range_data/";
    if (boost::filesystem::exists(files_save_param_.sub_range_data_path)) {
      boost::filesystem::remove_all(files_save_param_.sub_range_data_path);
    }
    return;
  }
  INFO(
    "accept request, gridmap finish, save to %s",
    files_save_param_.create_map_path.c_str());
  g_mapper_->GenerateGrid(request->map_name);
  INFO("save map done!");
  grid_map_status_ = GridMapStatus::HANG_MODEL;
  g_mapper_.reset();
}


void CreateGridMap::SubCallback(
  const nav_msgs::msg::Odometry::ConstSharedPtr odom,
  const sensor_msgs::msg::LaserScan::ConstSharedPtr scan)
{
  if (grid_map_status_ == GridMapStatus::HANG_MODEL) {return;}
  Eigen::Quaterniond rotation(Eigen::Quaterniond::Identity());
  rotation.w() = odom->pose.pose.orientation.w;
  rotation.x() = odom->pose.pose.orientation.x;
  rotation.y() = odom->pose.pose.orientation.y;
  rotation.z() = odom->pose.pose.orientation.z;
  Eigen::Vector3d translation(0, 0, 0);
  translation.x() = odom->pose.pose.position.x;
  translation.y() = odom->pose.pose.position.y;
  translation.z() = odom->pose.pose.position.z;

  transform::Rigid3d baselink2world_pose(translation, rotation);
  transform::Rigid3d laser2baselink = eigentf_.laser2baselink;

  common::Time time = common::FromUniversal(
    (scan->header.stamp.sec +
    common::kUtsEpochOffsetFromUnixEpochInSeconds) *
    10000000ll +
    (scan->header.stamp.nanosec + 50) / 100);
  sensor::PointCloudCarto point_cloud = ScanToPointCloud(scan);
  std::unique_ptr<sensor::RangeDataCarto> range_data = g_mapper_->AddRangeData(
    time, point_cloud,
    baselink2world_pose,
    laser2baselink);

  if (range_data != nullptr) {
    // Publish range data
    cyberdog_visions_interfaces::msg::RangeData range_data_msg;
    range_data_msg.header = scan->header;
    range_data_msg.origin.x = range_data->origin.x();
    range_data_msg.origin.y = range_data->origin.y();
    range_data_msg.returns.resize(range_data->returns.points.size());
    for (size_t i = 0; i < range_data->returns.points.size(); ++i) {
      range_data_msg.returns[i].x = range_data->returns.points[i].x();
      range_data_msg.returns[i].y = range_data->returns.points[i].y();
    }
    range_data_msg.misses.resize(range_data->misses.points.size());
    for (size_t i = 0; i < range_data->misses.points.size(); ++i) {
      range_data_msg.misses[i].x = range_data->misses.points[i].x();
      range_data_msg.misses[i].y = range_data->misses.points[i].y();
    }
    range_data_publisher_->publish(range_data_msg);
  }
}

sensor::PointCloudCarto CreateGridMap::ScanToPointCloud(
  const sensor_msgs::msg::LaserScan::ConstSharedPtr scan)
{
  sensor::PointCloudCarto point_cloud;
  float angle = scan->angle_min;
  for (size_t laser_beam = 0; laser_beam < scan->ranges.size(); ++laser_beam) {
    double beam_dis = scan->ranges.at(laser_beam);
    if (scan->range_min <= beam_dis && beam_dis <= scan->range_max) {
      Eigen::AngleAxisf rotation(angle, Eigen::Vector3f::UnitZ());
      Eigen::Vector3f local_point(rotation * (beam_dis * Eigen::Vector3f::UnitX()));
      point_cloud.points.push_back(local_point.head<2>());
    }
    angle += scan->angle_increment;
  }

  return point_cloud;
}

void CreateGridMap::PublishVisualizationCallback()
{
  std_msgs::msg::Bool visualization_flag_msg;
  visualization_flag_msg.data = visualization_flag_ && create_map_flag_;
  visualization_publisher_->publish(visualization_flag_msg);
}

void CreateGridMap::LoadExtrinsicsFromYaml(const std::string & calibr_file)
{
  YAML::Node root;
  root = YAML::LoadFile(calibr_file);

  std::vector<std::vector<double>> T_cam0_imu =
    root["cam0"]["T_cam_imu"].as<std::vector<std::vector<double>>>();
  std::vector<std::vector<double>> T_cam0_odom =
    root["cam0"]["T_cam_odom"].as<std::vector<std::vector<double>>>();
  std::vector<std::vector<double>> T_c_lidar =
    root["cam0"]["T_c_lidar"].as<std::vector<std::vector<double>>>();

  Eigen::MatrixXd T_c0_imu = ConvertToEigenMatrix(T_cam0_imu);
  Eigen::MatrixXd T_c0_odom = ConvertToEigenMatrix(T_cam0_odom);
  Eigen::MatrixXd T_c0_lidar = ConvertToEigenMatrix(T_c_lidar);

  Eigen::MatrixXd T_imu2odom = T_c0_odom.inverse() * T_c0_imu;
  Eigen::MatrixXd T_lidar2odom = T_c0_odom.inverse() * T_c0_lidar;
  Eigen::MatrixXd T_lidar2imu = T_c0_imu.inverse() * T_c0_lidar;
  Eigen::MatrixXd T_odom2imu = T_imu2odom.inverse();

  Eigen::Quaterniond q_imu2odom(Eigen::Matrix3d(T_imu2odom.topLeftCorner(3, 3)));
  Eigen::Vector3d p_imu2odom(T_imu2odom.topRightCorner(3, 1));
  Eigen::Quaterniond q_lidar2odom(Eigen::Matrix3d(T_lidar2odom.topLeftCorner(3, 3)));
  Eigen::Vector3d p_lidar2odom(T_lidar2odom.topRightCorner(3, 1));
  Eigen::Quaterniond q_lidar2imu(Eigen::Matrix3d(T_lidar2imu.topLeftCorner(3, 3)));
  Eigen::Vector3d p_lidar2imu(T_lidar2imu.topRightCorner(3, 1));
  Eigen::Quaterniond q_odom2imu(Eigen::Matrix3d(T_odom2imu.topLeftCorner(3, 3)));
  Eigen::Vector3d p_odom2imu(T_odom2imu.topRightCorner(3, 1));

  eigentf_.imu2baselink = transform::Rigid3d(p_imu2odom, q_imu2odom);
  eigentf_.laser2baselink = transform::Rigid3d(p_lidar2odom, q_lidar2odom);
  transform::Rigid3d odom2imu = transform::Rigid3d(p_odom2imu, q_odom2imu);
  eigentf_.laser2imu =
    transform::ToRigid2(eigentf_.imu2baselink.inverse() * eigentf_.laser2baselink);
  eigentf_.odom2imu = transform::ToRigid2(odom2imu);
}

Eigen::MatrixXd CreateGridMap::ConvertToEigenMatrix(std::vector<std::vector<double>> & data)
{
  CHECK_NE(data.size(), 0);
  CHECK_NE(data[0].size(), 0);
  Eigen::MatrixXd matrix(data.size(), data[0].size());
  for (size_t i = 0; i < data.size(); ++i) {
    matrix.row(i) = Eigen::VectorXd::Map(&data[i][0], data[0].size());
  }
  return matrix;
}

void CreateGridMap::RemoveMap()
{
  INFO("Remove map.");
  if (boost::filesystem::exists(files_save_param_.create_map_path)) {
    boost::filesystem::remove_all(files_save_param_.create_map_path);
  }
  boost::filesystem::create_directory(files_save_param_.create_map_path);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;

  auto node = std::make_shared<CreateGridMap>();
  exec.add_node(node);

  exec.spin();
  rclcpp::shutdown();

  return 0;
}
