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
#include <utility>
#include <vector>

#include "map2d/submap_2d.hpp"

namespace mapping
{

Submap2D::Submap2D(
  const Eigen::Vector2f & origin, std::unique_ptr<Grid2D> grid,
  ValueConversionTables * conversion_tables)
: Submap(transform::Rigid3d::Translation(
      Eigen::Vector3d(origin.x(), origin.y(), 0.))),
  m_conversion_tables(conversion_tables)
{
  m_grid = std::move(grid);
}

void Submap2D::InsertRangeData(
  const sensor::RangeDataCarto & range_data_carto,
  const GridInserter2D * grid2d_inserter)
{
  CHECK(m_grid);
  CHECK(!insertion_finished());
  CHECK_NOTNULL(grid2d_inserter);
  grid2d_inserter->Insert(range_data_carto, m_grid.get());
  set_num_range_data(num_range_data() + 1);
}

void Submap2D::Finish()
{
  CHECK(m_grid);
  CHECK(!insertion_finished());
  m_grid = m_grid->ComputeCroppedGrid();
  set_insertion_finished(true);
}

std::vector<std::shared_ptr<const Submap2D>> ActiveSubmaps2D::submaps() const
{
  return std::vector<std::shared_ptr<const Submap2D>>(m_submaps.begin(), m_submaps.end());
}

std::vector<std::shared_ptr<const Submap2D>> ActiveSubmaps2D::InsertRangeData(
  const sensor::RangeDataCarto & range_data)
{
  if (m_submaps.empty() ||
    m_submaps.back()->num_range_data() == 35)
  {
    AddSubmap(range_data.origin.head<2>());
  }
  for (auto & submap : m_submaps) {
    submap->InsertRangeData(range_data, m_grid_inserter_2d.get());
  }
  if (m_submaps.front()->num_range_data() == 2 * 35) {
    m_submaps.front()->Finish();
  }
  return submaps();
}

std::unique_ptr<Grid2D> ActiveSubmaps2D::CreateGrid(
  const Eigen::Vector2f & origin)
{
  float resolution = m_param.resolution;
  return std::make_unique<Grid2D>(
    MapLimits(
      resolution,
      origin.cast<double>() + 0.5 * m_param.resolution *
      Eigen::Vector2d(static_cast<double>(m_param.sizex), static_cast<double>(m_param.sizey)),
      CellLimits(m_param.sizex, m_param.sizey)),
    kMinCorrespondenceCost, kMaxCorrespondenceCost,
    &m_conversion_tables);
}

void ActiveSubmaps2D::AddSubmap(const Eigen::Vector2f & origin)
{
  LOG(INFO) << "Add new submap";

  if (m_submaps.size() >= 2) {
    // This will crop the finished Submap before inserting a new Submap to
    // reduce peak memory usage a bit.
    CHECK(m_submaps.front()->insertion_finished());
    m_submaps.erase(m_submaps.begin());
  }
  m_submaps.push_back(
    std::make_unique<Submap2D>(
      origin,
      std::unique_ptr<Grid2D>(
        static_cast<Grid2D *>(CreateGrid(origin).release())),
      &m_conversion_tables));
}

}  // namespace mapping
