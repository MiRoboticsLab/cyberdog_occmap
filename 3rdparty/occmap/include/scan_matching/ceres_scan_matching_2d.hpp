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

#ifndef SCAN_MATCHING__CERES_SCAN_MATCHING_2D_HPP_
#define SCAN_MATCHING__CERES_SCAN_MATCHING_2D_HPP_

#include <Eigen/Core>
#include <ceres/ceres.h>

#include <memory>
#include <vector>

#include "occ_gridmap/param.hpp"
#include "occ_gridmap/rigid_transform.hpp"
#include "occ_gridmap/sensor.hpp"
#include "map2d/grid_2d.hpp"

namespace mapping
{

class CeresScanMatcher2D
{
public:
  explicit CeresScanMatcher2D(const CeresScanMatchingParam & param);
  virtual ~CeresScanMatcher2D();

  CeresScanMatcher2D(const CeresScanMatcher2D &) = delete;
  CeresScanMatcher2D & operator=(const CeresScanMatcher2D &) = delete;

  void Match(
    const Eigen::Vector2d & target_translation,
    const transform::Rigid2d & initial_pose_estimate,
    const sensor::PointCloudCarto & point_cloud_carto,
    const Grid2D & grid,
    transform::Rigid2d * const pose_estimate,
    ceres::Solver::Summary * const summary) const;

private:
  ceres::Solver::Options CreateCeresSolverOptions()
  {
    ceres::Solver::Options options;
    options.use_nonmonotonic_steps = m_param.use_nonmonotonic_steps;
    options.max_num_iterations = m_param.max_num_iterations;
    options.num_threads = m_param.num_threads;
    return options;
  }

  CeresScanMatchingParam m_param;
  ceres::Solver::Options m_ceres_solver_options;
};  // class CeresScanMatcher2D
}  // namespace mapping

#endif  // SCAN_MATCHING__CERES_SCAN_MATCHING_2D_HPP_
