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

#include <memory>
#include <vector>
#include <string>

#include "occ_gridmap/grid_mapper.hpp"
#include "occ_gridmap/tic_toc.hpp"

namespace mapping
{

GridMapper::GridMapper(
  const transform::Eigentf & eigentf,
  const ProbabilityParam & probability_param,
  const SubMapParam & submap_param,
  const FilesSaveParam & files_save_param,
  const FilterParam & filter_param,
  const CeresScanMatchingParam & ceres_param,
  const MapBeautiParam & mapbeauti_param)
: eigentf_(eigentf), probability_param_(probability_param), submap_param_(submap_param),
  files_save_param_(files_save_param),
  accumulated_data_(sensor::RangeData()),
  filter_param_(filter_param),
  motion_filter_(std::make_unique<sensor::MotionFilter>(filter_param)),
  ceres_param_(ceres_param),
  ceres_scan_matcher_(std::make_unique<mapping::CeresScanMatcher2D>(ceres_param)),
  map_beauti_(std::make_shared<MapBeauti>(mapbeauti_param)),
  active_submaps_(nullptr),
  first_pose(Eigen::Vector2d::Zero()),
  current_submap_pose(transform::Rigid3d::Identity())
{
  active_submaps_.reset(new mapping::ActiveSubmaps2D(submap_param));
}

GridMapper::~GridMapper()
{
  grid_2d_.reset();
  grid_inserter_2d_.reset();
  motion_filter_.reset();
}

std::unique_ptr<sensor::RangeDataCarto> GridMapper::AddRangeData(
  const common::Time time,
  const sensor::PointCloudCarto & points,
  const transform::Rigid3d & global_pose,
  const transform::Rigid3d & laser2baselink)
{
  if (first_pose_flag) {
    first_pose = global_pose.translation().head<2>();
    grid_2d_ = CreateGrid(first_pose);
    grid_inserter_2d_ = CreateGridInserter2D();
    first_pose_flag = false;
  }
  if (motion_filter_->IsSimilar(time, global_pose)) {
    return nullptr;
  }
  if (num_accumulated_ == 0) {
    accumulated_data_.time_stamp = common::ToUniversal(time) / 100ll;
    accumulated_data_.submap_pose = transform::ToRigid2(global_pose);
    current_submap_pose = global_pose;
  }
  sensor::PointCloud point_cloud;
  sensor::PointCloudCarto point_cloud_carto;
  for (size_t i = 0; i < points.points.size(); ++i) {
    point_cloud_carto.points.push_back(
      transform::ToRigid2(global_pose.cast<float>() * laser2baselink.cast<float>()) *
      points.points.at(i));
  }
  // Insert point cloud into submap
  point_cloud_carto = sensor::AdaptivelyVoxelFiltered(point_cloud_carto, filter_param_);
  sensor::RangeDataCarto range_data_carto;
  range_data_carto.origin = transform::ToRigid2(global_pose).translation().cast<float>();
  for (auto point : point_cloud_carto.points) {
    range_data_carto.returns.points.push_back(point);
  }
  range_data_carto =
    sensor::TransformRangeData(
    range_data_carto,
    transform::ToRigid2(global_pose.inverse()).cast<float>());
  point_cloud_carto =
    sensor::TransformPointCloud(
    point_cloud_carto, transform::ToRigid2(
      global_pose.inverse()).cast<float>());
  transform::Rigid2d pose_predicted = transform::ToRigid2(global_pose);

  std::unique_ptr<transform::Rigid2d> pose_estimated_2d =
    ScanMatch(pose_predicted, point_cloud_carto);
  if (pose_estimated_2d == nullptr) {
    LOG(WARNING) << "scan matching failed";
    *pose_estimated_2d = pose_predicted;
  }

  sensor::RangeDataCarto range_data_in_global = sensor::TransformRangeData(
    range_data_carto, pose_estimated_2d->cast<float>());;
  const transform::Rigid3d pose_estimated =
    transform::ToRigid3(*pose_estimated_2d);
  bool insertion_result = InsertIntoSubmap(
    range_data_in_global);
  if (!insertion_result) {
    LOG(INFO) << "Insert into submap failed";
  }
  grid_inserter_2d_->Insert(range_data_in_global, grid_2d_.get());

  // Add range data into accumulated_data_
  if (num_accumulated_ == 0) {
    accumulated_data_.submap_pose = transform::ToRigid2(pose_estimated);
    current_submap_pose = pose_estimated;
  }
  auto point_local_pose_estimated = current_submap_pose.inverse() * pose_estimated;
  point_cloud.local_pose = transform::ToRigid2(point_local_pose_estimated);

  point_cloud.returns_points.clear();
  point_cloud.returns_points.reserve(range_data_in_global.returns.points.size());
  for (auto point : range_data_in_global.returns.points) {
    Eigen::Vector3d local_point(static_cast<double>(point[0]), static_cast<double>(point[1]), 0);
    local_point = current_submap_pose.inverse() * local_point;
    point_cloud.returns_points.emplace_back(local_point.head<2>());
  }
  point_cloud.time_stamp = common::ToUniversal(time) / 100ll;

  accumulated_data_.point_clouds.emplace_back(point_cloud);

  ++num_accumulated_;
  if (num_accumulated_ >= submap_param_.num_accumulated) {
    num_accumulated_ = 0;
    RangeDataToProto(accumulated_data_);
    accumulated_data_.clear();
  }
  return std::make_unique<sensor::RangeDataCarto>(range_data_in_global);
}

std::unique_ptr<transform::Rigid2d> GridMapper::ScanMatch(
  const transform::Rigid2d & pose_predicted,
  const sensor::PointCloudCarto & pcc)
{
  if (active_submaps_->submaps().empty()) {
    LOG(INFO) << "submaps empty";
    return std::make_unique<transform::Rigid2d>(pose_predicted);
  }
  std::shared_ptr<const mapping::Submap2D> matching_submap =
    active_submaps_->submaps().front();
  transform::Rigid2d initial_ceres_pose = pose_predicted;

  auto pose_observation = std::make_unique<transform::Rigid2d>();
  ceres::Solver::Summary summary;
  ceres_scan_matcher_->Match(
    pose_predicted.translation(), initial_ceres_pose,
    pcc, *matching_submap->grid(),
    pose_observation.get(), &summary);
  return pose_observation;
}

bool GridMapper::InsertIntoSubmap(
  const sensor::RangeDataCarto & range_data_in_global)
{
  std::vector<std::shared_ptr<const mapping::Submap2D>> insertion_submaps =
    active_submaps_->InsertRangeData(range_data_in_global);

  // return the insertion result
  return true;
}

std::unique_ptr<Grid2D> GridMapper::CreateGrid(
  const Eigen::Vector2d & origin)
{
  const int kInitialSubmapSize = submap_param_.sizex;
  float resolution = submap_param_.resolution;

  return std::make_unique<Grid2D>(
    MapLimits(
      resolution,
      origin + 0.5 * kInitialSubmapSize * resolution * Eigen::Vector2d::Ones(),  // 位于位姿world系
      CellLimits(
        kInitialSubmapSize,
        kInitialSubmapSize)), kMinCorrespondenceCost, kMaxCorrespondenceCost,
    &value_conversion_tables_);
}

void GridMapper::RangeDataToProto(
  const sensor::RangeData & range_local_data)
{
  LOG(INFO) << "Start writing sub range data";
  const std::string proto_file = files_save_param_.sub_range_data_path + "/sub_range_data_" +
    std::to_string(sub_range_data_++) + ".pbstream";
  auto proto_range_data = sensor::ToProto(range_local_data);
  auto proto_writer = std::make_shared<io::ProtoStreamWriter>(proto_file);
  proto_writer->WriteProto(proto_range_data);
  proto_writer->Close();
  LOG(INFO) << "Stop writing sub range data to: " << proto_file.c_str();
}

void GridMapper::GenerateGrid(const std::string & name)
{
  auto grid_2d = CreateGrid(first_pose);
  auto grid_2d_insert = this->CreateGridInserter2D();
  const int sub_range_data = sub_range_data_;
  const std::string range_data_path = files_save_param_.sub_range_data_path;
  common::createFolder(range_data_path);
  for (int ii = 0; ii < sub_range_data; ii++) {
    const std::string proto_file = range_data_path + "/sub_range_data_" +
      std::to_string(ii) + ".pbstream";
    proto::RangeData proto_range_data;
    auto proto_reader = std::make_shared<io::ProtoStreamReader>(proto_file);
    proto_reader->ReadProto(&proto_range_data);
    proto_reader->eof();

    sensor::RangeData local_range_data = sensor::RangeData(proto_range_data);
    transform::Rigid2d submap_pose = local_range_data.submap_pose;
    for (auto point_cloud : local_range_data.point_clouds) {
      sensor::RangeDataCarto global_range_data;
      {
        auto local_pose = transform::ToRigid3(submap_pose) * transform::ToRigid3(
          point_cloud.local_pose);
        global_range_data.origin = local_pose.translation().head<2>().cast<float>();
      }
      global_range_data.returns.points.reserve(point_cloud.returns_points.size());
      for (auto point : point_cloud.returns_points) {
        global_range_data.returns.points.emplace_back(
          (submap_pose * point).cast<float>());
      }
      grid_2d_insert->Insert(global_range_data, grid_2d.get());
    }
  }
  grid_2d->SaveMap(files_save_param_.create_map_path, name, map_beauti_.get());
  grid_2d.reset();
  grid_2d_insert.reset();
}
}  // namespace mapping
