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

#include <ceres/cubic_interpolation.h>

#include "scan_matching/occupied_space_cost_function_2d.hpp"
#include "map2d/probability_value.hpp"

namespace mapping
{

// Computes a cost for matching the 'point_cloud' to the 'grid' with
// a 'pose'. The cost increases with poorer correspondence of the grid and the
// point observation (e.g. points falling into less occupied space).
class OccupiedSpaceCostFunction2D
{
public:
  OccupiedSpaceCostFunction2D(
    const double scaling_factor,
    const sensor::PointCloudCarto & point_cloud_carto,
    const Grid2D & grid)
  : m_scaling_factor(scaling_factor),
    m_point_cloud_carto(point_cloud_carto),
    m_grid(grid) {}

  template<typename T>
  bool operator()(const T * const pose, T * residual) const
  {
    Eigen::Matrix<T, 2, 1> translation(pose[0], pose[1]);
    Eigen::Rotation2D<T> rotation(pose[2]);
    Eigen::Matrix<T, 2, 2> rotation_matrix = rotation.toRotationMatrix();
    Eigen::Matrix<T, 3, 3> transform;
    transform << rotation_matrix, translation, T(0.), T(0.), T(1.);

    const GridArrayAdapter adapter(m_grid);
    ceres::BiCubicInterpolator<GridArrayAdapter> interpolator(adapter);
    const MapLimits & limits = m_grid.limits();

    for (size_t i = 0; i < m_point_cloud_carto.points.size(); ++i) {
      // Note that this is a 2D point. The third component is a scaling factor.
      const Eigen::Matrix<T, 3, 1> point((T(m_point_cloud_carto.points[i].x())),
        (T(m_point_cloud_carto.points[i].y())),
        T(1.));
      const Eigen::Matrix<T, 3, 1> world = transform * point;
      interpolator.Evaluate(
        (limits.max().x() - world[0]) / limits.resolution() - 0.5 +
        static_cast<double>(kPadding),
        (limits.max().y() - world[1]) / limits.resolution() - 0.5 +
        static_cast<double>(kPadding),
        &residual[i]);
      residual[i] = m_scaling_factor * residual[i];
    }
    return true;
  }

private:
  static constexpr int kPadding = INT_MAX / 4;
  class GridArrayAdapter
  {
public:
    enum { DATA_DIMENSION = 1 };

    explicit GridArrayAdapter(const Grid2D & grid)
    : m_grid(grid) {}

    void GetValue(const int row, const int column, double * const value) const
    {
      if (row < kPadding || column < kPadding || row >= NumRows() - kPadding ||
        column >= NumCols() - kPadding)
      {
        *value = kMaxCorrespondenceCost;
      } else {
        *value = static_cast<double>(m_grid.GetCorrespondenceCost(
            Eigen::Array2i(column - kPadding, row - kPadding)));
      }
    }

    int NumRows() const
    {
      return m_grid.limits().cell_limits().num_y_cells + 2 * kPadding;
    }

    int NumCols() const
    {
      return m_grid.limits().cell_limits().num_x_cells + 2 * kPadding;
    }

private:
    const Grid2D & m_grid;
  };  // class GridArrayAdapter

  OccupiedSpaceCostFunction2D(const OccupiedSpaceCostFunction2D &) = delete;
  OccupiedSpaceCostFunction2D & operator=(const OccupiedSpaceCostFunction2D &) =
  delete;

  const double m_scaling_factor;
  const sensor::PointCloudCarto & m_point_cloud_carto;
  const Grid2D & m_grid;
};  // class OccupiedSpaceCostFunction2D

ceres::CostFunction * CreateOccupiedSpaceCostFunction2D(
  const double scaling_factor, const sensor::PointCloudCarto & point_cloud_carto,
  const Grid2D & grid)
{
  return new ceres::AutoDiffCostFunction<OccupiedSpaceCostFunction2D,
           ceres::DYNAMIC /* residuals */,
           3 /* pose variables */>(
    new OccupiedSpaceCostFunction2D(scaling_factor, point_cloud_carto, grid),
    point_cloud_carto.points.size());
}

}  // namespace mapping
