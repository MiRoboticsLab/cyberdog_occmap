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
// limitations under the License..

#ifndef OCC_GRIDMAP__GRID_MAPPER_HPP_
#define OCC_GRIDMAP__GRID_MAPPER_HPP_

#include <Eigen/Core>
#include <Eigen/Dense>

#include <malloc.h>


#include <fstream>
#include <memory>
#include <string>
#include <vector>
#include <map>

#include "occ_gridmap/common.hpp"
#include "occ_gridmap/param.hpp"
#include "occ_gridmap/rigid_transform.hpp"
#include "occ_gridmap/sensor.hpp"
#include "occ_gridmap/proto_io.hpp"
#include "occ_gridmap/map_beauti.hpp"

#include "map2d/grid_2d.hpp"
#include "map2d/value_conversion_tables.hpp"
#include "map2d/grid_inserter_2d.hpp"
#include "map2d/submap_2d.hpp"

#include "scan_matching/ceres_scan_matching_2d.hpp"

#include "proto/range_data.pb.h"
#include "proto/transform.pb.h"

namespace mapping
{

struct InsertionResult
{
  transform::Rigid3d global_pose;
  sensor::RangeDataCarto glocal_range_data;
};

class GridMapper
{
public:
  GridMapper(
    const transform::Eigentf & eigentf,
    const ProbabilityParam & probability_param,
    const SubMapParam & submap_param,
    const FilesSaveParam & files_save_param,
    const FilterParam & filter_param,
    const CeresScanMatchingParam & ceres_param,
    const MapBeautiParam & mapbeauti_param);

  ~GridMapper();

  std::unique_ptr<sensor::RangeDataCarto> AddRangeData(
    const common::Time time,
    const sensor::PointCloudCarto & points,
    const transform::Rigid3d & laser_pose,
    const transform::Rigid3d & laser2baselink);

  void GenerateGrid(const std::string & name);

  std::unique_ptr<io::Image> GetOccGridImage(Eigen::Vector2d & origin)
  {
    return grid_2d_->ToOccupancyImage(origin);
  }

private:
  // void computeAngleCache(const sensor::RangeData& range_data);
  void RangeDataToProto(const sensor::RangeData & range_local_data);
  void InsertRangeData(const sensor::RangeData & range_local_data);
  std::unique_ptr<GridInserter2D> CreateGridInserter2D()
  {
    return std::make_unique<GridInserter2D>();
  }
  std::unique_ptr<transform::Rigid2d> ScanMatch(
    const transform::Rigid2d & pose_predicted,
    const sensor::PointCloudCarto & pcc);
  bool InsertIntoSubmap(
    const sensor::RangeDataCarto & range_data_in_global);
  std::unique_ptr<Grid2D> CreateGrid(const Eigen::Vector2d & origin);

private:
  transform::Eigentf eigentf_;
  ProbabilityParam probability_param_;
  SubMapParam submap_param_;
  FilesSaveParam files_save_param_;
  sensor::RangeData accumulated_data_;

private:
  std::vector<double> vcosangle_;
  std::vector<double> vsinangle_;
  int num_accumulated_ = 0;
  int sub_range_data_ = 0;
  int last_sub_range_data_ = 0;
  bool got_first_scan_ = true;
  std::map<int64_t, transform::Rigid3d> loop_pose_;

  std::unique_ptr<Grid2D> grid_2d_;
  std::unique_ptr<GridInserter2D> grid_inserter_2d_;

private:
  ValueConversionTables value_conversion_tables_;
  FilterParam filter_param_;
  std::unique_ptr<sensor::MotionFilter> motion_filter_;
  CeresScanMatchingParam ceres_param_;
  std::unique_ptr<mapping::CeresScanMatcher2D> ceres_scan_matcher_;
  std::shared_ptr<MapBeauti> map_beauti_;
  std::unique_ptr<mapping::ActiveSubmaps2D> active_submaps_;

private:
  Eigen::Vector2d first_pose;
  bool first_pose_flag = true;
  transform::Rigid3d current_submap_pose;
};  // class GridMapper

}  // namespace mapping

#endif  // OCC_GRIDMAP__GRID_MAPPER_HPP_
