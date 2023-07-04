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

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <stdlib.h>
#include <yaml-cpp/yaml.h>

#include <vector>
#include <chrono>
#include <fstream>
#include <string>
#include <memory>

#include "occ_gridmap/grid_mapper.hpp"
#include "occ_gridmap/sensor.hpp"

struct PointCloudPose
{
  common::Time time;
  transform::Rigid3d pose;
  sensor::PointCloudCarto pointcloud;
};

sensor::PointCloudCarto readCloud(const std::string & cloud_path)
{
  // read pointcloud
  sensor::PointCloudCarto pointcloud;
  std::fstream cloudIn(cloud_path);
  std::string line;
  if (cloudIn.is_open()) {
    while (std::getline(cloudIn, line)) {
      std::stringstream input(line);
      std::vector<std::string> res;
      res.reserve(2);
      std::string result;
      while (input >> result) {
        res.emplace_back(result);
      }
      if (res.size() != 2) {
        std::cout << "cloud file read failed." << std::endl;
        break;
      }
      Eigen::Vector2f point(std::stod(res[0].c_str()), std::stod(res[1].c_str()));
      pointcloud.points.emplace_back(point);
    }
  }
  return pointcloud;
}

std::vector<PointCloudPose> readDataset(const std::string & root_path)
{
  std::vector<PointCloudPose> vpointcloud_pose;
  std::string info_path = root_path + "/info.txt";
  std::string line;
  std::fstream fs;
  fs.open(info_path);
  if (fs.is_open()) {
    while (std::getline(fs, line)) {
      PointCloudPose pointcloud_pose;
      std::stringstream input(line);
      std::vector<std::string> res;
      res.reserve(8);
      std::string result;
      while (input >> result) {
        res.emplace_back(result);
      }
      if (res.size() != 8) {
        std::cout << "info file read failed." << std::endl;
        break;
      }
      common::Time time = common::FromUniversal(atoll(res[0].c_str()));
      pointcloud_pose.time = time;
      Eigen::Quaterniond quaternion(std::stod(res[7].c_str()), std::stod(res[4].c_str()),
        std::stod(res[5].c_str()), std::stod(res[6].c_str()));
      Eigen::Vector3d translation(std::stod(res[1].c_str()), std::stod(res[2].c_str()),
        std::stod(res[3].c_str()));
      quaternion.normalize();
      transform::Rigid3d pose(translation, quaternion);
      pointcloud_pose.pose = pose;
      std::string cloud_path = root_path + "/pointcloud/" + res[0] + ".txt";
      pointcloud_pose.pointcloud = readCloud(cloud_path);
      vpointcloud_pose.emplace_back(pointcloud_pose);
    }
  }
  return vpointcloud_pose;
}

void ParseParameters(
  const std::string & config_file,
  transform::Eigentf & eigentf,
  ProbabilityParam & probability_param,
  SubMapParam & submap_param,
  FilesSaveParam & files_save_param,
  FilterParam & filter_param,
  CeresScanMatchingParam & ceres_param,
  MapBeautiParam & mapbeauti_param
)
{
  YAML::Node yaml_node = YAML::LoadFile(config_file);
  YAML::Node params = yaml_node["create_gridmap_node"];

  // Load tf
  std::vector<double> tf_laserq =
    params["laser_to_baselink"]["quaternion"].as<std::vector<double>>();
  std::vector<double> tf_laserp = params["laser_to_baselink"]["position"].as<std::vector<double>>();

  auto q_laser2baselink =
    Eigen::Quaterniond(tf_laserq[3], tf_laserq[0], tf_laserq[1], tf_laserq[2]);
  auto p_laser2baselink =
    Eigen::Vector3d(tf_laserp[0], tf_laserp[1], tf_laserp[2]);

  eigentf.laser2baselink = transform::Rigid3d(p_laser2baselink, q_laser2baselink);

  files_save_param.create_map_path = params["create_map_path"].as<std::string>("/home/hpf/");
  files_save_param.sub_range_data_path = files_save_param.create_map_path + "/sub_range_data/";

  // Load filter params
  filter_param.max_angle_radians = params["filter_param"]["max_angle_radians"].as<double>(0.0175);
  filter_param.max_distance_meters = params["filter_param"]["max_distance_meters"].as<double>(0.2);
  filter_param.max_length = params["filter_param"]["max_length"].as<double>(0.05);
  filter_param.max_time_seconds = params["filter_param"]["max_time_seconds"].as<double>(160);
  filter_param.min_num_points = params["filter_param"]["min_num_points"].as<double>(200);
  filter_param.voxel_filter_size = params["filter_param"]["voxel_filter_size"].as<double>(0.025);

  // Load ceres params
  ceres_param.use_nonmonotonic_steps = params["ceres_param"]["use_nonmonotonic_steps"].as<bool>(
    false);
  ceres_param.max_num_iterations = params["ceres_param"]["max_num_iterations"].as<int>(20);
  ceres_param.num_threads = params["ceres_param"]["num_threads"].as<int>(1);
  ceres_param.occupied_space_weight =
    params["ceres_param"]["occupied_space_weight"].as<double>(10.0);
  ceres_param.translation_weight = params["ceres_param"]["translation_weight"].as<double>(20.0);
  ceres_param.rotation_weight = params["ceres_param"]["rotation_weight"].as<double>(40.0);

  // Load gridmap params
  probability_param.p_occ = params["probability_param"]["p_occ"].as<double>(0.55);
  probability_param.p_free = params["probability_param"]["p_free"].as<double>(0.45);
  probability_param.p_prior = params["probability_param"]["p_prior"].as<double>(0.5);

  submap_param.resolution = params["submap_param"]["resolution"].as<double>(0.05);
  submap_param.sizex = params["submap_param"]["sizex"].as<int>(100);
  submap_param.sizey = params["submap_param"]["sizey"].as<int>(100);
  submap_param.initx = params["submap_param"]["initx"].as<int>(100);
  submap_param.inity = params["submap_param"]["inity"].as<int>(100);
  submap_param.num_accumulated = params["submap_param"]["num_accumulated"].as<int>(35);
  submap_param.missing_data_ray_length =
    params["submap_param"]["missing_data_ray_length"].as<double>(6.0);
  submap_param.max_range = params["submap_param"]["max_range"].as<double>(8.0);
  submap_param.min_range = params["submap_param"]["min_range"].as<double>(0.2);

  // Load map beauti params
  mapbeauti_param.sideFillThresh = params["mapbeauti"]["side_fill_thresh"].as<double>();
  mapbeauti_param.approxPolythresh = params["mapbeauti"]["approx_poly_thresh"].as<double>();
  mapbeauti_param.dilateKernelSize = params["mapbeauti"]["dilate_kernel_size"].as<int>();
  mapbeauti_param.noiseRemovalThresh = params["mapbeauti"]["noise_removal_thresh"].as<int>();
}

std::unique_ptr<mapping::Grid2D> CreateGrid(
  const Eigen::Vector2d & origin,
  mapping::ValueConversionTables & value_conversion_tables)
{
  const int kInitialSubmapSize = 100;
  float resolution = 0.05;

  return std::make_unique<mapping::Grid2D>(
    mapping::MapLimits(
      resolution,
      origin + 0.5 * kInitialSubmapSize * resolution * Eigen::Vector2d::Ones(),
      mapping::CellLimits(kInitialSubmapSize, kInitialSubmapSize)),
    mapping::kMinCorrespondenceCost,
    mapping::kMaxCorrespondenceCost,
    &value_conversion_tables);
}

int main(int argc, char ** argv)
{
  const std::string root_path = argv[1];
  const std::string config_file = root_path + "/gridmap_node.yaml";
  std::vector<PointCloudPose> vpointcloud_pose = readDataset(root_path);

  // 初始化参数
  transform::Eigentf eigentf;
  ProbabilityParam probability_param;
  SubMapParam submap_param;
  FilesSaveParam files_save_param;
  FilterParam filter_param;
  CeresScanMatchingParam ceres_param;
  MapBeautiParam mapbeauti_param;
  ParseParameters(
    config_file, eigentf, probability_param, submap_param, files_save_param,
    filter_param, ceres_param, mapbeauti_param);
  if (boost::filesystem::exists(files_save_param.sub_range_data_path)) {
    std::cout << "remove path: " << files_save_param.sub_range_data_path << std::endl;
    boost::filesystem::remove_all(files_save_param.sub_range_data_path);
  }

  if (common::createFolder(files_save_param.sub_range_data_path)) {
    std::cout << "create path: " << files_save_param.sub_range_data_path << std::endl;
  }

  // 实例化对象
  auto g_mapper_ = std::make_shared<mapping::GridMapper>(
    eigentf, probability_param, submap_param,
    files_save_param, filter_param,
    ceres_param, mapbeauti_param);
  // 插入点云
  for (auto & pointcloud_pose : vpointcloud_pose) {
    std::unique_ptr<sensor::RangeDataCarto> range_data_ptr =
      g_mapper_->AddRangeData(
      pointcloud_pose.time, pointcloud_pose.pointcloud,
      pointcloud_pose.pose, eigentf.laser2baselink);
  }
  // 生成地图
  g_mapper_->GenerateGrid("map");
}
