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

#ifndef MAP2D__GRID_2D_HPP_
#define MAP2D__GRID_2D_HPP_

#include <stdint.h>

#include <vector>
#include <memory>
#include <string>

#include "occ_gridmap/common.hpp"
#include "occ_gridmap/map_beauti.hpp"

#include "map2d/map_limits.hpp"
#include "map2d/probability_value.hpp"
#include "map2d/value_conversion_tables.hpp"
#include "map2d/image.hpp"

namespace mapping
{

class Grid2D
{
public:
  Grid2D(
    const MapLimits & map_limit, float min_correspondence_cost,
    float max_correspondence_cost,
    ValueConversionTables * conversion_tables);
  // Grid2D(const MapLimits& map_limit, float min_correspondence_cost,
  //        float max_correspondence_cost);

public:
  //
  std::unique_ptr<io::Image> DrawProbabilityGrid(Eigen::Array2i * offset) const;
  // Sets the probability of the cell at 'cell_index' to the given
  // 'probability'. Only allowed if the cell was unknown before.
  void SetProbability(
    const Eigen::Array2i & cell_index,
    const float probability);
  // Applies the 'odds' specified when calling ComputeLookupTableToApplyOdds()
  // to the probability of the cell at 'cell_index' if the cell has not already
  // been updated. Multiple updates of the same cell will be ignored until
  // FinishUpdate() is called. Returns true if the cell was updated.
  //
  // If this is the first call to ApplyOdds() for the specified cell, its value
  // will be set to probability corresponding to 'odds'.
  bool ApplyLookupTable(
    const Eigen::Array2i & cell_index,
    const std::vector<uint16_t> & table, const std::vector<uint16_t> & hit_table);

  // Returns the probability of the cell with 'cell_index'.
  float GetProbability(const Eigen::Array2i & cell_index) const;

  std::unique_ptr<Grid2D> ComputeCroppedGrid() const;

  std::unique_ptr<io::Image> ToOccupancyImage(Eigen::Vector2d & origin);

  // bool ToRosOccupancyMsg(nav_msgs::msg::OccupancyGrid& occupancy_grid) const;

  bool SaveMap(const std::string & map_save_path, MapBeauti * const map_beauti);

  bool SaveMap(
    const std::string & map_save_path, const std::string & name,
    MapBeauti * const map_beauti);

public:
  const MapLimits & limits() const {return map_limit_;}

  // Finishes the update sequence.
  void FinishUpdate();

  // Returns the correspondence cost of the cell with 'cell_index'.
  float GetCorrespondenceCost(const Eigen::Array2i & cell_index) const
  {
    if (!limits().Contains(cell_index)) {return max_correspondence_cost_;}
    return (*value_to_correspondence_cost_table_)
           [correspondence_cost_cells()[ToFlatIndex(cell_index)]];
  }

  // Returns the minimum possible correspondence cost.
  float GetMinCorrespondenceCost() const {return min_correspondence_cost_;}

  // Returns the maximum possible correspondence cost.
  float GetMaxCorrespondenceCost() const {return max_correspondence_cost_;}

  // Returns true if the probability at the specified index is known.
  bool IsKnown(const Eigen::Array2i & cell_index) const
  {
    return map_limit_.Contains(cell_index) &&
           correspondence_cost_cells_[ToFlatIndex(cell_index)] !=
           kUnknownCorrespondenceValue;
  }

  // Fills in 'offset' and 'limits' to define a subregion of that contains all
  // known cells.
  void ComputeCroppedLimits(
    Eigen::Array2i * const offset,
    CellLimits * const limits) const;

  // Grows the map as necessary to include 'point'. This changes the meaning of
  // these coordinates going forward. This method must be called immediately
  // after 'FinishUpdate', before any calls to 'ApplyLookupTable'.
  virtual void GrowLimits(const Eigen::Vector2f & point);

protected:
  const std::vector<uint16_t> & correspondence_cost_cells() const
  {
    return correspondence_cost_cells_;
  }

  // Converts a 'cell_index' into an index into 'cells_'.
  int ToFlatIndex(const Eigen::Array2i & cell_index) const
  {
    CHECK(map_limit_.Contains(cell_index)) << cell_index.x() << " " << cell_index.y();
    return map_limit_.cell_limits().num_x_cells * cell_index.y() + cell_index.x();
  }

  void GrowLimits(
    const Eigen::Vector2f & point,
    const std::vector<std::vector<uint16_t> *> & grids,
    const std::vector<uint16_t> & grids_unknown_cell_values);

  const std::vector<int> & update_indices() const {return update_indices_;}

  const Eigen::AlignedBox2i & known_cells_box() const
  {
    return known_cells_box_;
  }

  std::vector<int> * mutable_update_indices() {return &update_indices_;}

  Eigen::AlignedBox2i * mutable_known_cells_box() {return &known_cells_box_;}

  std::vector<uint16_t> * mutable_correspondence_cost_cells()
  {
    return &correspondence_cost_cells_;
  }

private:
  MapLimits map_limit_;
  std::vector<uint16_t> correspondence_cost_cells_;
  float min_correspondence_cost_;
  float max_correspondence_cost_;
  std::vector<int> update_indices_;

private:
  Eigen::AlignedBox2i known_cells_box_;
  const std::vector<float> * value_to_correspondence_cost_table_;

  ValueConversionTables * conversion_tables_;
};

}  // namespace mapping

#endif  // MAP2D__GRID_2D_HPP_
