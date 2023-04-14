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

#ifndef SCAN_MATCHING__OCCUPIED_SPACE_COST_FUNCTION_2D_HPP_
#define SCAN_MATCHING__OCCUPIED_SPACE_COST_FUNCTION_2D_HPP_

#include <ceres/ceres.h>

#include "occ_gridmap/sensor.hpp"
#include "map2d/grid_2d.hpp"

namespace mapping
{

// Creates a cost function for matching the 'point_cloud' to the 'grid' with
// a 'pose'. The cost increases with poorer correspondence of the grid and the
// point observation (e.g. points falling into less occupied space).
ceres::CostFunction * CreateOccupiedSpaceCostFunction2D(
  const double scaling_factor, const sensor::PointCloudCarto & point_cloud_carto,
  const Grid2D & grid);

}  // namespace mapping

#endif  // SCAN_MATCHING__OCCUPIED_SPACE_COST_FUNCTION_2D_HPP_
