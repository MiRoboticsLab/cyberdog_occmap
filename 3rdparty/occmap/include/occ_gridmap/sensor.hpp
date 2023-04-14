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

#ifndef OCC_GRIDMAP__SENSOR_HPP_
#define OCC_GRIDMAP__SENSOR_HPP_

#include <Eigen/Core>
#include <Eigen/Dense>

#include <vector>
#include <random>

#include "occ_gridmap/rigid_transform.hpp"
#include "occ_gridmap/param.hpp"
#include "occ_gridmap/common.hpp"

#include "proto/range_data.pb.h"
#include "proto/transform.pb.h"

namespace sensor
{

struct PointCloud
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  std::vector<Eigen::Vector2d> returns_points;
  transform::Rigid2d local_pose;
  int64_t time_stamp;

  size_t Size() const {return returns_points.size();}
};

struct RangeData
{
  RangeData();
  RangeData(
    const int64_t & time_stamp_,
    const transform::Rigid2d & submap_pose);
  explicit RangeData(const proto::RangeData & range_data);
  void clear();
  int64_t time_stamp;
  std::vector<PointCloud> point_clouds;
  transform::Rigid2d submap_pose;
};

struct PointCloudCarto
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  std::vector<Eigen::Vector2f> points;
};

// Transforms 'point_cloud' according to 'transform'.
PointCloudCarto TransformPointCloud(
  const PointCloudCarto & point_cloud_carto,
  const transform::Rigid2f & transform);

struct RangeDataCarto
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector2f origin;
  PointCloudCarto returns;
  PointCloudCarto misses;
};

RangeDataCarto TransformRangeData(
  const RangeDataCarto & range_data_carto,
  const transform::Rigid2f & transform);

proto::RangeData ToProto(const RangeData & range_data);

int64_t GetVoxelCellIndex(
  const Eigen::Vector2d & point,
  const double resolution);

int64_t GetVoxelCellIndex(
  const Eigen::Vector2f & point,
  const double resolution);

template<class T, class PointFunction>
std::vector<bool> RandomizedVoxelFilterIndices(
  const std::vector<T> & point_cloud, const float resolution,
  PointFunction && point_function);

PointCloud VoxelFilter(const PointCloud & point_cloud, const double resolution);

PointCloud AdaptivelyVoxelFiltered(
  const PointCloud & point_cloud,
  const FilterParam & filter_param);

PointCloudCarto VoxelFilter(const PointCloudCarto & point_cloud, const double resolution);

PointCloudCarto AdaptivelyVoxelFiltered(
  const PointCloudCarto & point_cloud,
  const FilterParam & filter_param);

class MotionFilter
{
public:
  explicit MotionFilter(const FilterParam & filter_params);
  // If the accumulated motion (linear, rotational, or time) is above the
  // threshold, returns false. Otherwise the relative motion is accumulated and
  // true is returned.
  bool IsSimilar(common::Time time, const transform::Rigid3d & pose);
  common::Time last_time() const {return last_time_;}

private:
  FilterParam filter_params_;
  int num_total_ = 0;
  int num_different_ = 0;
  common::Time last_time_;
  transform::Rigid3d last_pose_;
};  // calss MotionFilter

void TDFlipMatrix(Eigen::MatrixXd & bel_data);
void LRFlipMatrix(Eigen::MatrixXd & bel_data);

}  // namespace sensor
#endif  // OCC_GRIDMAP__SENSOR_HPP_
