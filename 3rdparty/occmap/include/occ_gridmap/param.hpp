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

#ifndef OCC_GRIDMAP__PARAM_HPP_
#define OCC_GRIDMAP__PARAM_HPP_

#include <string>

struct FilesSaveParam
{
  // from command line
  std::string create_map_path;  // map.pgm path
  std::string sub_range_data_path;  // .pbstream path
  std::string loop_pose_path;  // mivins_trajectory.txt path
};

struct ProbabilityParam
{
  ProbabilityParam() = default;
  double p_occ = 0.65;
  double p_free = 0.196;
  double p_prior = 0.5;
};

struct SubMapParam
{
  SubMapParam() = default;
  int num_accumulated = 35;  // nums laser frame accumulated
  int sizex = 100;
  int sizey = 100;
  int initx = 50;
  int inity = 50;
  int max_range = 12.0;
  int min_range = 0.01;
  double resolution = 0.05;
  double missing_data_ray_length = 6.0;
};

struct FilterParam
{
  FilterParam() = default;
  // motion filter
  int max_time_seconds = 160;  // ms
  double max_distance_meters = 0.2;
  double max_angle_radians = 0.0175;
  // voxel filter
  int min_num_points = 100;
  double max_length = 1.0;
  double voxel_filter_size = 0.025;
};

struct CeresScanMatchingParam
{
  bool use_nonmonotonic_steps;
  int max_num_iterations;
  int num_threads;
  double occupied_space_weight;
  double translation_weight;
  double rotation_weight;
};

struct RealTimeCorrelativeScanMatcherParam
{
  double linear_search_window;
  double angular_search_window;
  double translation_delta_cost_weight;
  double rotation_delta_cost_weight;
};

struct MapBeautiParam
{
  bool useMapBeauti;
  double sideFillThresh;
  double approxPolythresh;
  int dilateKernelSize;
  int noiseRemovalThresh;
};

typedef enum
{
  HANG_MODEL,
  CREATE_MAP_MODEL,
  START_NAV_MODEL
} GridMapStatus;

#endif  // OCC_GRIDMAP__PARAM_HPP_
