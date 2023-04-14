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

#include <vector>

#include "map2d/grid_inserter_2d.hpp"

namespace mapping
{

namespace
{
// Factor for subpixel accuracy of start and end point for ray casts.
constexpr int kSubpixelScale = 1000;
constexpr double max_beam_dis = 6.0;
constexpr double min_beam_dis = 0.01;
constexpr float missing_data_ray_length = 5.0;
void GrowAsNeeded(
  const sensor::RangeDataCarto & range_data,
  Grid2D * const grid)
{
  Eigen::AlignedBox2f bounding_box(range_data.origin);
  // Padding around bounding box to avoid numerical issues at cell boundaries.

  constexpr float kPadding = 1e-6f;

  for (const auto & hit : range_data.returns.points) {
    auto hit_delta = hit - range_data.origin;
    float beam_dis = hit_delta.norm();
    if (beam_dis < min_beam_dis) {continue;}
    if (beam_dis > max_beam_dis) {
      Eigen::Vector2f misses_end =
        range_data.origin + missing_data_ray_length * hit_delta.normalized();
      bounding_box.extend(misses_end);
    } else {
      bounding_box.extend(hit);
    }
  }
  grid->GrowLimits(
    bounding_box.min() -
    kPadding * Eigen::Vector2f::Ones());
  grid->GrowLimits(
    bounding_box.max() +
    kPadding * Eigen::Vector2f::Ones());
}

void CastRays(
  const sensor::RangeDataCarto & range_data,
  const std::vector<uint16_t> & hit_table,
  const std::vector<uint16_t> & miss_table,
  Grid2D * grid)
{
  GrowAsNeeded(range_data, grid);

  const MapLimits & limits = grid->limits();
  const float superscaled_resolution = limits.resolution() / kSubpixelScale;
  const MapLimits superscaled_limits(
    superscaled_resolution, limits.max(),
    CellLimits(
      limits.cell_limits().num_x_cells * kSubpixelScale,
      limits.cell_limits().num_y_cells * kSubpixelScale));

  const Eigen::Array2i begin =
    superscaled_limits.GetCellIndex(range_data.origin);
  if (begin.x() < 0 || begin.y() < 0) {return;}
  std::vector<Eigen::Array2i> ends;
  std::vector<Eigen::Array2i> misses_ends;
  ends.reserve(range_data.returns.points.size());
  for (const auto & hit : range_data.returns.points) {
    auto hit_delta = hit - range_data.origin;
    float beam_dis = hit_delta.norm();
    if (beam_dis < min_beam_dis) {continue;}
    if (beam_dis > max_beam_dis) {
      Eigen::Vector2f misses_end =
        range_data.origin + missing_data_ray_length * hit_delta.normalized();
      misses_ends.emplace_back(superscaled_limits.GetCellIndex(misses_end));
    } else {
      ends.emplace_back(superscaled_limits.GetCellIndex(hit));
      grid->ApplyLookupTable(ends.back() / kSubpixelScale, hit_table, hit_table);
    }
  }

  // Now add the misses.
  for (const Eigen::Array2i & end : ends) {
    std::vector<Eigen::Array2i> ray =
      RayToPixelMask(begin, end, kSubpixelScale);
    for (const Eigen::Array2i & cell_index : ray) {
      grid->ApplyLookupTable(cell_index, miss_table, hit_table);
    }
  }

  for (const Eigen::Array2i & end : misses_ends) {
    std::vector<Eigen::Array2i> ray =
      RayToPixelMask(begin, end, kSubpixelScale);
    for (const Eigen::Array2i & cell_index : ray) {
      grid->ApplyLookupTable(cell_index, miss_table, hit_table);
    }
  }
}

}  // namespace
constexpr int kValueCount = 32769;
GridInserter2D::GridInserter2D()
: hit_table_(ComputeLookupTableToApplyCorrespondenceCostOdds(
      Odds(0.65))),
  miss_table_(ComputeLookupTableToApplyCorrespondenceCostOdds(
      Odds(0.48)))
{}

void GridInserter2D::Insert(
  const sensor::RangeDataCarto & range_data, Grid2D * const grid) const
{
  // By not finishing the update after hits are inserted, we give hits priority
  // (i.e. no hits will be ignored because of a miss in the same cell).
  CHECK(grid != nullptr);
  CastRays(
    range_data, hit_table_, miss_table_, grid);
  grid->FinishUpdate();
}

}  // namespace mapping
