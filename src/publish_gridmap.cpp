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

#include <string>
#include <memory>
#include <vector>

#include "occ_gridmap/sensor.hpp"
#include "occ_gridmap/tic_toc.hpp"

#include "occ_gridmap/publish_gridmap.hpp"
#include "cyberdog_common/cyberdog_log.hpp"

using namespace std::chrono_literals;

constexpr float freeProbability = 0.196f;
constexpr float occProbability = 0.65f;

constexpr uint8_t maxThresh = static_cast<uint8_t>((1 - freeProbability) * 255.f);
constexpr uint8_t minThresh =
  static_cast<uint8_t>((1 - occProbability) * 255.f);
constexpr uint8_t freeThresh = 128;

PublishGridMap::PublishGridMap(const rclcpp::NodeOptions & options)
: Node("PublishGridMap", options)
{
  this->declare_parameter<std::string>("config_file");
  this->get_parameter<std::string>("config_file", config_file_);

  ParseParameters(config_file_);
  map_beauti_ = std::make_shared<MapBeauti>(mapbeauti_param_);
  PrepareCommunication();
}

void PublishGridMap::ParseParameters(std::string & config_file)
{
  YAML::Node yaml_node = YAML::LoadFile(config_file);
  YAML::Node params = yaml_node["publish_gridmap_node"];

  range_data_topic_ = params["range_data_topic"].as<std::string>("range_data");
  gridmap_topic_ = params["gridmap_topic"].as<std::string>("map");
  visualization_topic_ = params["visualization_topic"].as<std::string>("occmap_visualization");
  submap_resolution_ = params["submap"]["resolution"].as<double>(0.05);
  submap_initsize_ = params["submap"]["initsize"].as<int>(100);

  mapbeauti_param_.useMapBeauti = params["mapbeauti"]["use_map_beauti"].as<bool>();
  mapbeauti_param_.sideFillThresh = params["mapbeauti"]["side_fill_thresh"].as<double>();
  mapbeauti_param_.approxPolythresh = params["mapbeauti"]["approx_poly_thresh"].as<double>();
  mapbeauti_param_.dilateKernelSize = params["mapbeauti"]["dilate_kernel_size"].as<int>();
  mapbeauti_param_.noiseRemovalThresh = params["mapbeauti"]["noise_removal_thresh"].as<int>();
}

void PublishGridMap::RangeDataCallback(
  const cyberdog_visions_interfaces::msg::RangeData::ConstSharedPtr range_data_msg)
{
  TicToc t_r;
  if (first_pose_flag_) {
    Eigen::Vector2d first_pose(range_data_msg->origin.x, range_data_msg->origin.y);
    grid_2d_ = CreateGrid(first_pose);
    grid_inserter_2d_ = std::make_unique<mapping::GridInserter2D>();
    first_pose_flag_ = false;
  }

  sensor::RangeDataCarto range_data;
  range_data.origin = Eigen::Vector2f(range_data_msg->origin.x, range_data_msg->origin.y);
  for (size_t i = 0; i < range_data_msg->returns.size(); ++i) {
    range_data.returns.points.emplace_back(
      Eigen::Vector2f(
        range_data_msg->returns[i].x,
        range_data_msg->returns[i].y));
  }
  for (size_t i = 0; i < range_data_msg->misses.size(); ++i) {
    range_data.misses.points.emplace_back(
      Eigen::Vector2f(
        range_data_msg->misses[i].x,
        range_data_msg->misses[i].y));
  }

  grid_inserter_2d_->Insert(range_data, grid_2d_.get());
  header_ = range_data_msg->header;
}

void PublishGridMap::VisualizationCallback(
  const std_msgs::msg::Bool::ConstSharedPtr visualization_flag_msg)
{
  if (visualization_flag_msg->data ^ visualization_flag_) {
    if (visualization_flag_msg->data) {
      INFO("Start publish occupancy map.");
      PrepareCommunication();
    } else {
      INFO("Stop publish occupancy map.");
      ResetCommunication();
    }
  }
  visualization_flag_ = visualization_flag_msg->data;
}

void PublishGridMap::PublishOccupancyGridCallback()
{
  INFO("Publish occupancy map.");
  if (nullptr == grid_2d_ || nullptr == grid_inserter_2d_) {
    return;
  }
  nav_msgs::msg::OccupancyGrid occupancy_grid;
  Eigen::Vector2d origin;
  std::unique_ptr<io::Image> image = grid_2d_->ToOccupancyImage(origin);
  if (image != nullptr) {
    std::vector<uint8_t> pixel_data;
    if (mapbeauti_param_.useMapBeauti) {
      pixel_data = map_beauti_->FinalProcess(image->pixel_data(), image->width(), image->height());
    } else {
      pixel_data = map_beauti_->process(image->pixel_data(), image->width(), image->height());
    }

    occupancy_grid.info.resolution = submap_resolution_;
    occupancy_grid.info.width = image->width();
    occupancy_grid.info.height = image->height();
    occupancy_grid.info.origin.position.x = origin.x();
    occupancy_grid.info.origin.position.y = origin.y();
    occupancy_grid.info.origin.position.z = 0.;
    occupancy_grid.info.origin.orientation.w = 1.;
    occupancy_grid.info.origin.orientation.x = 0.;
    occupancy_grid.info.origin.orientation.y = 0.;
    occupancy_grid.info.origin.orientation.z = 0.;
    occupancy_grid.data.reserve(image->width() * image->height());
    constexpr uint8_t kUnkonwvalue = 128;
    for (int y = image->height() - 1; y >= 0; --y) {
      for (int x = 0; x < image->width(); ++x) {
        const uint8_t packed = pixel_data[y * image->width() + x];
        int value = -1;
        if (packed <= minThresh) {value = 100;} else if (packed == freeThresh) {value = -1;} else {
          value = 0;
        }
        CHECK_LE(-1, value);
        CHECK_GE(100, value);
        occupancy_grid.data.emplace_back(value);
      }
    }
    image.reset();
    occupancy_grid.header = header_;
    occupancy_grid.header.frame_id = "map";
    occupancy_grid.info.map_load_time = header_.stamp;
    occgrid_publisher_->publish(occupancy_grid);
  }
}

std::unique_ptr<mapping::Grid2D> PublishGridMap::CreateGrid(
  const Eigen::Vector2d & origin)
{
  const int kInitialSubmapSize = submap_initsize_;
  float resolution = submap_resolution_;

  return std::make_unique<mapping::Grid2D>(
    mapping::MapLimits(
      resolution,
      origin + 0.5 * kInitialSubmapSize * resolution * Eigen::Vector2d::Ones(),
      mapping::CellLimits(kInitialSubmapSize, kInitialSubmapSize)),
    mapping::kMinCorrespondenceCost,
    mapping::kMaxCorrespondenceCost,
    &value_conversion_tables_);
}

void PublishGridMap::PrepareCommunication()
{
  range_data_subscriber_ = this->create_subscription<cyberdog_visions_interfaces::msg::RangeData>(
    range_data_topic_,
    10,
    std::bind(&PublishGridMap::RangeDataCallback, this, std::placeholders::_1));
  visualization_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
    visualization_topic_,
    1,
    std::bind(&PublishGridMap::VisualizationCallback, this, std::placeholders::_1));
  occgrid_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    gridmap_topic_,
    rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
  pub_map_timer_ =
    this->create_wall_timer(1s, std::bind(&PublishGridMap::PublishOccupancyGridCallback, this));
}

void PublishGridMap::ResetCommunication()
{
  range_data_subscriber_.reset();
  pub_map_timer_.reset();
  occgrid_publisher_.reset();

  first_pose_flag_ = true;
  grid_2d_.reset();
  grid_inserter_2d_.reset();
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;

  auto node = std::make_shared<PublishGridMap>();
  exec.add_node(node);

  exec.spin();
  rclcpp::shutdown();

  return 0;
}
