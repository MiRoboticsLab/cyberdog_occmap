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

#include <vector>
#include <map>
#include <utility>

#include "occ_gridmap/sensor.hpp"

namespace sensor
{

RangeData::RangeData()
: time_stamp(0),
  submap_pose(transform::Rigid2d::Identity()) {}

RangeData::RangeData(
  const int64_t & time_stamp_,
  const transform::Rigid2d & submap_pose_)
: time_stamp(time_stamp_),
  submap_pose(submap_pose_) {}

RangeData::RangeData(const proto::RangeData & range_data)
: time_stamp(range_data.time_stamp())
{
  submap_pose = transform::ToRigid2(range_data.submap_pose());
  for (const auto & proto_point_clouds : range_data.point_clouds()) {
    PointCloud point_cloud;
    for (const auto & proto_point : proto_point_clouds.returns_points()) {
      Eigen::Vector2d point(proto_point.x(), proto_point.y());
      point_cloud.returns_points.emplace_back(point);
    }
    point_cloud.local_pose = transform::ToRigid2(proto_point_clouds.local_pose());
    point_cloud.time_stamp = proto_point_clouds.time_stamp();
    this->point_clouds.emplace_back(point_cloud);
  }
}

void RangeData::clear()
{
  time_stamp = 0;
  std::vector<PointCloud>().swap(point_clouds);
  submap_pose = transform::Rigid2d::Identity();
}

PointCloudCarto TransformPointCloud(
  const PointCloudCarto & point_cloud_carto,
  const transform::Rigid2f & transform)
{
  std::vector<Eigen::Vector2f> points;
  points.reserve(point_cloud_carto.points.size());
  for (const Eigen::Vector2f & point : point_cloud_carto.points) {
    points.push_back(transform * point);
  }
  PointCloudCarto pcc;
  pcc.points = points;
  return pcc;
}

RangeDataCarto TransformRangeData(
  const RangeDataCarto & range_data_carto,
  const transform::Rigid2f & transform)
{
  return RangeDataCarto{
    transform * range_data_carto.origin,
    TransformPointCloud(range_data_carto.returns, transform),
    TransformPointCloud(range_data_carto.misses, transform)
  };
}

proto::RangeData ToProto(const RangeData & range_data)
{
  proto::RangeData proto;
  proto.set_time_stamp(range_data.time_stamp);
  for (auto & point_clouds : range_data.point_clouds) {
    proto::PointCloud * point_clouds_proto = proto.add_point_clouds();
    for (auto & point : point_clouds.returns_points) {
      proto::Vector2d * points_proto = point_clouds_proto->add_returns_points();
      points_proto->set_x(point.x());
      points_proto->set_y(point.y());
    }
    *point_clouds_proto->mutable_local_pose() = transform::ToProto(point_clouds.local_pose);
    point_clouds_proto->set_time_stamp(point_clouds.time_stamp);
  }
  *proto.mutable_submap_pose() = transform::ToProto(range_data.submap_pose);
  return proto;
}

int64_t GetVoxelCellIndex(
  const Eigen::Vector2d & point,
  const double resolution)
{
  const Eigen::Array2d index = point.array() / resolution;
  const int64_t x = common::RoundToInt(index.x());
  const int64_t y = common::RoundToInt(index.y());
  return (x << 42) + (y << 21);
}

int64_t GetVoxelCellIndex(
  const Eigen::Vector2f & point,
  const double resolution)
{
  const Eigen::Array2f index = point.array() / resolution;
  const int64_t x = common::RoundToInt(index.x());
  const int64_t y = common::RoundToInt(index.y());
  return (x << 42) + (y << 21);
}

template<class T, class PointFunction>
std::vector<bool> RandomizedVoxelFilterIndices(
  const std::vector<T> & point_cloud, const float resolution,
  PointFunction && point_function)
{
  // According to https://en.wikipedia.org/wiki/Reservoir_sampling
  std::minstd_rand0 generator;
  std::map<int64_t, std::pair<int, int>>
  voxel_count_and_point_index;
  for (size_t i = 0; i < point_cloud.size(); i++) {
    auto & voxel = voxel_count_and_point_index[GetVoxelCellIndex(
          point_function(point_cloud[i]), resolution)];
    voxel.first++;
    if (voxel.first == 1) {
      voxel.second = i;
    } else {
      std::uniform_int_distribution<> distribution(1, voxel.first);
      if (distribution(generator) == voxel.first) {
        voxel.second = i;
      }
    }
  }
  std::vector<bool> points_used(point_cloud.size(), false);
  for (const auto & voxel_and_index : voxel_count_and_point_index) {
    points_used[voxel_and_index.second.second] = true;
  }
  return points_used;
}

PointCloud VoxelFilter(const PointCloud & point_cloud, const double resolution)
{
  const std::vector<bool> points_used_return = RandomizedVoxelFilterIndices(
    point_cloud.returns_points, resolution,
    [](const Eigen::Vector2d & point) {return point;});

  std::vector<Eigen::Vector2d> returns_points;
  for (size_t ii = 0; ii < point_cloud.returns_points.size(); ii++) {
    if (points_used_return[ii]) {
      returns_points.emplace_back(point_cloud.returns_points[ii]);
    }
  }

  PointCloud temp_point_cloud;
  temp_point_cloud.returns_points.swap(returns_points);
  temp_point_cloud.local_pose = point_cloud.local_pose;
  temp_point_cloud.time_stamp = point_cloud.time_stamp;
  return temp_point_cloud;
}

PointCloudCarto VoxelFilter(
  const PointCloudCarto & point_cloud, const double resolution)
{
  const std::vector<bool> points_used = RandomizedVoxelFilterIndices(
    point_cloud.points, resolution,
    [](const Eigen::Vector2f & point) {return point;});

  std::vector<Eigen::Vector2f> filtered_points;
  for (size_t i = 0; i < point_cloud.points.size(); i++) {
    if (points_used[i]) {
      filtered_points.push_back(point_cloud.points[i]);
    }
  }

  return PointCloudCarto{std::move(filtered_points)};
}

PointCloud AdaptivelyVoxelFiltered(
  const PointCloud & point_cloud,
  const FilterParam & filter_param)
{
  if (point_cloud.returns_points.size() <= static_cast<size_t>(filter_param.min_num_points)) {
    // 'point_cloud' is already sparse enough.
    return point_cloud;
  }
  PointCloud result = VoxelFilter(point_cloud, filter_param.max_length);
  if (result.returns_points.size() >= static_cast<size_t>(filter_param.min_num_points)) {
    // Filtering with 'max_length' resulted in a sufficiently dense point cloud.
    return result;
  }
  // Search for a 'low_length' that is known to result in a sufficiently
  // dense point cloud. We give up and use the full 'point_cloud' if reducing
  // the edge length by a factor of 1e-2 is not enough.
  for (float high_length = filter_param.max_length;
    high_length > 1e-2f * filter_param.max_length; high_length /= 2.f)
  {
    float low_length = high_length / 2.f;
    result = VoxelFilter(point_cloud, low_length);
    if (result.returns_points.size() >= static_cast<size_t>(filter_param.min_num_points)) {
      // Binary search to find the right amount of filtering. 'low_length' gave
      // a sufficiently dense 'result', 'high_length' did not. We stop when the
      // edge length is at most 10% off.
      while ((high_length - low_length) / low_length > 1e-1f) {
        const float mid_length = (low_length + high_length) / 2.f;
        const PointCloud candidate = VoxelFilter(point_cloud, mid_length);
        if (candidate.returns_points.size() >= static_cast<size_t>(filter_param.min_num_points)) {
          low_length = mid_length;
          result = candidate;
        } else {
          high_length = mid_length;
        }
      }
      return result;
    }
  }
  return result;
}

PointCloudCarto AdaptivelyVoxelFiltered(
  const PointCloudCarto & point_cloud,
  const FilterParam & filter_param)
{
  if (point_cloud.points.size() <= static_cast<size_t>(filter_param.min_num_points)) {
    return point_cloud;
  }

  PointCloudCarto result = VoxelFilter(point_cloud, filter_param.max_length);
  if (result.points.size() >= static_cast<size_t>(filter_param.min_num_points)) {
    return result;
  }

  for (float high_length = filter_param.max_length;
    high_length > 1e-2f * filter_param.max_length; high_length /= 2.f)
  {
    float low_length = high_length / 2.f;
    result = VoxelFilter(point_cloud, low_length);
    if (result.points.size() >= static_cast<size_t>(filter_param.min_num_points)) {
      while (high_length - low_length / low_length > 1e-1f) {
        const float mid_length = (low_length + high_length) / 2.f;
        const PointCloudCarto candidate = VoxelFilter(point_cloud, mid_length);
        if (candidate.points.size() >= static_cast<size_t>(filter_param.min_num_points)) {
          low_length = mid_length;
          result = candidate;
        } else {
          high_length = mid_length;
        }
      }
      return result;
    }
    return result;
  }
}

MotionFilter::MotionFilter(const FilterParam & filter_params)
: filter_params_(filter_params)
{
  last_pose_ = transform::Rigid3d::Identity();
}

constexpr double kDogMaxLinearVelocity = 1.5f;  // m/s
constexpr double kDogMaxAngularVelocity = 1.5f;  // rad/s

bool MotionFilter::IsSimilar(
  const common::Time & time,
  const transform::Rigid3d & pose)
{
  const common::Duration max_time_seconds = common::FromSeconds(
    filter_params_.max_time_seconds * 1e-3);
  ++num_total_;
  common::Duration period = time - last_time_;
  double linear_velocity = (pose.translation() - last_pose_.translation()).norm() /
    (period.count() * 1e-7);
  double angular_velocity = std::fabs(transform::GetAngle(pose.inverse() * last_pose_)) /
    (period.count() * 1e-7);
  if (linear_velocity > kDogMaxLinearVelocity || angular_velocity > kDogMaxAngularVelocity) {
    return true;
  }
  if (num_total_ > 1 &&
    time - last_time_ <= max_time_seconds &&
    (pose.translation() - last_pose_.translation()).norm() <=
    filter_params_.max_distance_meters &&
    transform::GetAngle(pose.inverse() * last_pose_) <=
    filter_params_.max_angle_radians)
  {
    return true;
  }
  last_time_ = time;
  last_pose_ = pose;
  ++num_different_;
  return false;
}

void LRFlipMatrix(Eigen::MatrixXd & bel_data)
{
  Eigen::MatrixXd temp_data = bel_data;
  size_t row = temp_data.rows();
  size_t col = temp_data.cols();
  size_t NN;
  if (col % 2 == 0) {
    std::cout << "col%2=0\n";
    NN = col / 2;
  } else {
    std::cout << "col%2!=0\n";
    NN = (col - 1) / 2;
  }
  for (size_t ii = 0; ii < NN; ii++) {
    bel_data.block(0, col - ii - 1, row, 1) =
      temp_data.block(0, ii, row, 1);
    bel_data.block(0, ii, row, 1) =
      temp_data.block(0, col - ii - 1, row, 1);
  }
}

}  // namespace sensor
