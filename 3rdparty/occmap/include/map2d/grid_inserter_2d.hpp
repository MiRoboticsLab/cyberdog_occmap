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

#ifndef MAP2D__GRID_INSERTER_2D_HPP_
#define MAP2D__GRID_INSERTER_2D_HPP_

#include <vector>
#include <cstdlib>

#include "occ_gridmap/sensor.hpp"
#include "map2d/xy_index.hpp"
#include "map2d/probability_value.hpp"
#include "map2d/grid_2d.hpp"
#include "map2d/ray_to_pixel_mask.hpp"

namespace mapping
{

class GridInserter2D
{
public:
  GridInserter2D();
  GridInserter2D(const GridInserter2D &) = delete;
  GridInserter2D & operator=(const GridInserter2D &) = delete;

  // Inserts 'range_data' into 'grid_2d'.
  void Insert(
    const sensor::RangeDataCarto & range_data,
    Grid2D * const grid) const;

private:
  const std::vector<uint16_t> hit_table_;
  const std::vector<uint16_t> miss_table_;
};  // class

}  // namespace mapping

#endif  // MAP2D__GRID_INSERTER_2D_HPP_
