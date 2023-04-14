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

#ifndef MAP2D__SUBMAP_2D_HPP_
#define MAP2D__SUBMAP_2D_HPP_

#include <Eigen/Core>

#include <memory>
#include <vector>

#include "occ_gridmap/param.hpp"
#include "occ_gridmap/sensor.hpp"
#include "grid_2d.hpp"
#include "grid_inserter_2d.hpp"
#include "submaps.hpp"
#include "value_conversion_tables.hpp"

namespace mapping
{

class Submap2D : public Submap
{
public:
  Submap2D(
    const Eigen::Vector2f & origin, std::unique_ptr<Grid2D> grid,
    ValueConversionTables * conversion_tables);

  const Grid2D * grid() const {return m_grid.get();}

  // Insert 'range_data' into this submap using 'range_data_inserter'. The
  // submap must not be finished yet.
  void InsertRangeData(
    const sensor::RangeDataCarto & range_data_carto,
    const GridInserter2D * grid2d_inserter);
  void Finish();

private:
  std::unique_ptr<Grid2D> m_grid;
  ValueConversionTables * m_conversion_tables;
};

// The first active submap will be created on the insertion of the first range
// data. Except during this initialization when no or only one single submap
// exists, there are always two submaps into which range data is inserted: an
// old submap that is used for matching, and a new one, which will be used for
// matching next, that is being initialized.
//
// Once a certain number of range data have been inserted, the new submap is
// considered initialized: the old submap is no longer changed, the "new" submap
// is now the "old" submap and is used for scan-to-map matching. Moreover, a
// "new" submap gets created. The "old" submap is forgotten by this object.
class ActiveSubmaps2D
{
public:
  explicit ActiveSubmaps2D(const SubMapParam & param)
  : m_param(param), m_grid_inserter_2d(CreateGridInserter2D()) {}
  ActiveSubmaps2D(const ActiveSubmaps2D &) = delete;
  ActiveSubmaps2D & operator=(const ActiveSubmaps2D) = delete;

  bool Initialize();

  // Inserts 'range_data' into the Submap collection.
  std::vector<std::shared_ptr<const Submap2D>> InsertRangeData(
    const sensor::RangeDataCarto & range_data);

  std::vector<std::shared_ptr<const Submap2D>> submaps() const;

private:
  std::unique_ptr<GridInserter2D> CreateGridInserter2D()
  {
    return std::make_unique<GridInserter2D>();
  }

  std::unique_ptr<Grid2D> CreateGrid(const Eigen::Vector2f & origin);
  void FinishSubmap();
  void AddSubmap(const Eigen::Vector2f & origin);

  SubMapParam m_param;
  std::vector<std::shared_ptr<Submap2D>> m_submaps;
  std::unique_ptr<GridInserter2D> m_grid_inserter_2d;
  ValueConversionTables m_conversion_tables;
};  // class ActiveSubmaps2D

}  // namespace mapping

#endif  // MAP2D__SUBMAP_2D_HPP_
