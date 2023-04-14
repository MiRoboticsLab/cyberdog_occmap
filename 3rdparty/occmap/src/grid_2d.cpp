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
#include <memory>
#include <string>

#include "map2d/grid_2d.hpp"
#include "occ_gridmap/sensor.hpp"

namespace
{
uint8_t ProbabilityToColor(
  float probability_from_grid)
{
  const float probability = 1.f - probability_from_grid;
  return common::RoundToInt(
    255 * ((probability - mapping::kMinProbability) /
    (mapping::kMaxProbability - mapping::kMinProbability)));
}
}  // namespace

namespace mapping
{

Grid2D::Grid2D(
  const MapLimits & map_limit, float min_correspondence_cost,
  float max_correspondence_cost,
  ValueConversionTables * conversion_tables)
: map_limit_(map_limit),
  correspondence_cost_cells_(
    map_limit_.cell_limits().num_x_cells * map_limit_.cell_limits().num_y_cells,
    kUnknownCorrespondenceValue),
  min_correspondence_cost_(min_correspondence_cost),
  max_correspondence_cost_(max_correspondence_cost),
  value_to_correspondence_cost_table_(conversion_tables->GetConversionTable(
      max_correspondence_cost, min_correspondence_cost,
      max_correspondence_cost)),
  conversion_tables_(conversion_tables) {}

// Finishes the update sequence.
void Grid2D::FinishUpdate()
{
  while (!update_indices_.empty()) {
    DCHECK_GE(
      correspondence_cost_cells_[update_indices_.back()],
      kUpdateMarker);
    correspondence_cost_cells_[update_indices_.back()] -= kUpdateMarker;
    update_indices_.pop_back();
  }
}

// Grows the map as necessary to include 'point'. This changes the meaning of
// these coordinates going forward. This method must be called immediately
// after 'FinishUpdate', before any calls to 'ApplyLookupTable'.
void Grid2D::GrowLimits(const Eigen::Vector2f & point)
{
  GrowLimits(
    point, {mutable_correspondence_cost_cells()},
    {kUnknownCorrespondenceValue});
}

void Grid2D::GrowLimits(
  const Eigen::Vector2f & point,
  const std::vector<std::vector<uint16_t> *> & grids,
  const std::vector<uint16_t> & grids_unknown_cell_values)
{
  CHECK(update_indices_.empty());
  while (!map_limit_.Contains(map_limit_.GetCellIndex(point))) {
    LOG(INFO) << point[0] << " " << point[1];
    const int x_offset = map_limit_.cell_limits().num_x_cells / 2;
    const int y_offset = map_limit_.cell_limits().num_y_cells / 2;
    const MapLimits new_limits(
      map_limit_.resolution(),
      map_limit_.max() +
      map_limit_.resolution() * Eigen::Vector2d(y_offset, x_offset),
      CellLimits(
        2 * map_limit_.cell_limits().num_x_cells,
        2 * map_limit_.cell_limits().num_y_cells));
    const int stride = new_limits.cell_limits().num_x_cells;
    const int offset = x_offset + stride * y_offset;
    const int new_size = new_limits.cell_limits().num_x_cells *
      new_limits.cell_limits().num_y_cells;
    LOG(INFO) << "new_size = " << new_size;
    for (size_t grid_index = 0; grid_index < grids.size(); ++grid_index) {
      std::vector<uint16_t> new_cells(new_size,
        grids_unknown_cell_values[grid_index]);
      for (int i = 0; i < map_limit_.cell_limits().num_y_cells; ++i) {
        for (int j = 0; j < map_limit_.cell_limits().num_x_cells; ++j) {
          new_cells[offset + j + i * stride] =
            (*grids[grid_index])[j + i * map_limit_.cell_limits().num_x_cells];
        }
      }
      *grids[grid_index] = new_cells;
    }
    map_limit_ = new_limits;
    if (!known_cells_box_.isEmpty()) {
      known_cells_box_.translate(Eigen::Vector2i(x_offset, y_offset));
    }
  }
}

}  // namespace mapping

namespace mapping
{

// Sets the probability of the cell at 'cell_index' to the given
// 'probability'. Only allowed if the cell was unknown before.
void Grid2D::SetProbability(
  const Eigen::Array2i & cell_index,
  const float probability)
{
  uint16_t & cell =
    (*mutable_correspondence_cost_cells())[ToFlatIndex(cell_index)];
  CHECK_EQ(cell, kUnknownProbabilityValue);
  cell =
    CorrespondenceCostToValue(ProbabilityToCorrespondenceCost(probability));
  mutable_known_cells_box()->extend(cell_index.matrix());
}

// Applies the 'odds' specified when calling ComputeLookupTableToApplyOdds()
// to the probability of the cell at 'cell_index' if the cell has not already
// been updated. Multiple updates of the same cell will be ignored until
// FinishUpdate() is called. Returns true if the cell was updated.
//
// If this is the first call to ApplyOdds() for the specified cell, its value
// will be set to probability corresponding to 'odds'.
bool Grid2D::ApplyLookupTable(
  const Eigen::Array2i & cell_index,
  const std::vector<uint16_t> & table, const std::vector<uint16_t> & hit_table)
{
  DCHECK_EQ(table.size(), kUpdateMarker);
  const int flat_index = ToFlatIndex(cell_index);
  uint16_t * cell = &(*mutable_correspondence_cost_cells())[flat_index];

  if (*cell >= kUpdateMarker) {
    return false;
  }

  if (GetProbability(cell_index) >= 0.9) {
    mutable_update_indices()->push_back(flat_index);
    *cell = hit_table[*cell];
    DCHECK_GE(*cell, kUpdateMarker);
    mutable_known_cells_box()->extend(cell_index.matrix());
    return true;
  }

  mutable_update_indices()->push_back(flat_index);
  *cell = table[*cell];
  DCHECK_GE(*cell, kUpdateMarker);
  mutable_known_cells_box()->extend(cell_index.matrix());
  return true;
}

// Returns the probability of the cell with 'cell_index'.
float Grid2D::GetProbability(const Eigen::Array2i & cell_index) const
{
  if (!limits().Contains(cell_index)) {return kMinProbability;}
  return CorrespondenceCostToProbability(
    ValueToCorrespondenceCost(
      correspondence_cost_cells()[ToFlatIndex(cell_index)]));
}

std::unique_ptr<Grid2D> Grid2D::ComputeCroppedGrid() const
{
  Eigen::Array2i offset;
  CellLimits cell_limits;
  ComputeCroppedLimits(&offset, &cell_limits);
  const double resolution = limits().resolution();
  const Eigen::Vector2d max =
    limits().max() - resolution * Eigen::Vector2d(offset.y(), offset.x());
  std::unique_ptr<Grid2D> cropped_grid =
    std::make_unique<Grid2D>(
    MapLimits(resolution, max, cell_limits),
    kMinCorrespondenceCost, kMaxCorrespondenceCost,
    conversion_tables_);
  for (const Eigen::Array2i & xy_index : XYIndexRangeIterator(cell_limits)) {
    if (!IsKnown(xy_index + offset)) {continue;}
    cropped_grid->SetProbability(xy_index, GetProbability(xy_index + offset));
  }

  return std::unique_ptr<Grid2D>(cropped_grid.release());
}

void Grid2D::ComputeCroppedLimits(
  Eigen::Array2i * const offset,
  CellLimits * const limits) const
{
  if (known_cells_box_.isEmpty()) {
    *offset = Eigen::Array2i::Zero();
    *limits = CellLimits(1, 1);
    return;
  }
  *offset = known_cells_box_.min().array();
  *limits = CellLimits(
    known_cells_box_.sizes().x() + 1,
    known_cells_box_.sizes().y() + 1);
}

std::unique_ptr<io::Image> Grid2D::ToOccupancyImage(Eigen::Vector2d & origin)
{
  Eigen::Array2i offset;
  std::unique_ptr<io::Image> image = DrawProbabilityGrid(&offset);
  if (image == nullptr) {
    return image;
  }
  image->Rotate90DegreeClockWise();
  origin = Eigen::Vector2d(
    limits().max().x() - (offset.y() + image->width()) * limits().resolution(),
    limits().max().y() - (offset.x() + image->height()) * limits().resolution());

  return image;
}

bool Grid2D::SaveMap(const std::string & map_save_path, MapBeauti * const map_beauti)
{
  Eigen::Array2i offset;
  std::unique_ptr<io::Image> image = DrawProbabilityGrid(&offset);
  if (image == nullptr) {return false;}
  std::vector<uint8_t> pixel_data = map_beauti->FinalProcess(
    image->pixel_data(),
    image->width(), image->height());
  image->SetPixelData(pixel_data);
  image->Rotate90DegreeClockWise();
  const std::string pgm_file = map_save_path + "/map.pgm";
  auto file_writer = std::make_unique<io::StreamFileWriter>(pgm_file);
  image->WritePgm(limits().resolution(), file_writer.get());

  const Eigen::Vector2d origin(
    limits().max().x() - (offset.y() + image->width()) * limits().resolution(),
    limits().max().y() - (offset.x() + image->height()) * limits().resolution());

  /* 保存配置 */
  std::ofstream config_file;
  config_file.open(map_save_path + "/map.yaml");
  config_file << "image: map.pgm\n" <<
    "resolution: " << limits().resolution() << std::endl <<
    "origin: [" << origin.x() << "," << origin.y() << ",0]" << std::endl <<
    "negate: 0" << std::endl <<
    "occupied_thresh: 0.65" << std::endl <<
    "free_thresh: 0.196";
  config_file.close();

  // 保存map_server
  std::ofstream server_file;
  server_file.open(map_save_path + "/map_server_params.yaml");
  server_file << "map_server:\n" <<
    "  ros__parameters:\n" <<
    "    frame_id: map\n" <<
    "    topic_name: /miloc/grid_map\n" <<
    "    use_sim_time: false\n" <<
    "    yaml_filename: map.yaml";
  server_file.close();
  return true;
}

bool Grid2D::SaveMap(
  const std::string & map_save_path, const std::string & name,
  MapBeauti * const map_beauti)
{
  Eigen::Array2i offset;
  std::unique_ptr<io::Image> image = DrawProbabilityGrid(&offset);
  if (image == nullptr) {return false;}
  std::vector<uint8_t> pixel_data = map_beauti->FinalProcess(
    image->pixel_data(),
    image->width(), image->height());
  image->SetPixelData(pixel_data);
  image->Rotate90DegreeClockWise();
  const std::string pgm_file = map_save_path + "/" + name + ".pgm";
  auto file_writer = std::make_unique<io::StreamFileWriter>(pgm_file);
  image->WritePgm(limits().resolution(), file_writer.get());

  const Eigen::Vector2d origin(
    limits().max().x() - (offset.y() + image->width()) * limits().resolution(),
    limits().max().y() - (offset.x() + image->height()) * limits().resolution());

  std::ofstream config_file;
  config_file.open(map_save_path + "/" + name + ".yaml");
  config_file << "image: " << name << ".pgm" << std::endl <<
    "resolution: " << limits().resolution() << std::endl <<
    "origin: [" << origin.x() << "," << origin.y() << ",0]" << std::endl <<
    "negate: 0" << std::endl <<
    "occupied_thresh: 0.65" << std::endl <<
    "free_thresh: 0.196";
  config_file.close();

  std::ofstream server_file;
  server_file.open(map_save_path + "/map_server_params.yaml");
  server_file << "map_server:\n" <<
    "  ros__parameters:\n" <<
    "    frame_id: map\n" <<
    "    topic_name: /miloc/grid_map\n" <<
    "    use_sim_time: false\n" <<
    "    yaml_filename: map.yaml";
  server_file.close();

  return true;
}

std::unique_ptr<io::Image> Grid2D::DrawProbabilityGrid(Eigen::Array2i * offset) const
{
  mapping::CellLimits cell_limits;
  ComputeCroppedLimits(offset, &cell_limits);
  if (cell_limits.num_x_cells == 0 || cell_limits.num_y_cells == 0) {
    return nullptr;
  }
  auto image = std::make_unique<io::Image>(
    cell_limits.num_x_cells, cell_limits.num_y_cells);

  for (const Eigen::Array2i & xy_index :
    mapping::XYIndexRangeIterator(cell_limits))
  {
    const Eigen::Array2i index = xy_index + *offset;
    constexpr uint8_t kUnknownValue = 128;
    const uint8_t value = IsKnown(index) ?
      ProbabilityToColor(GetProbability(index)) :
      kUnknownValue;
    image->SetPixel(xy_index.x(), xy_index.y(), value);
  }
  return image;
}

}  // namespace mapping
