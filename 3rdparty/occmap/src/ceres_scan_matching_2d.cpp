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

#include <utility>
#include <vector>

#include "scan_matching/ceres_scan_matching_2d.hpp"
#include "scan_matching/occupied_space_cost_function_2d.hpp"
#include "scan_matching/rotation_delta_cost_function_2d.hpp"
#include "scan_matching/translation_delta_cost_functor_2d.hpp"

namespace mapping
{

CeresScanMatcher2D::CeresScanMatcher2D(
  const CeresScanMatchingParam & param)
: m_param(param),
  m_ceres_solver_options(CreateCeresSolverOptions())
{
  m_ceres_solver_options.linear_solver_type = ceres::DENSE_QR;
}

CeresScanMatcher2D::~CeresScanMatcher2D() {}

void CeresScanMatcher2D::Match(
  const Eigen::Vector2d & target_translation,
  const transform::Rigid2d & initial_pose_estimate,
  const sensor::PointCloudCarto & point_cloud_carto,
  const Grid2D & grid,
  transform::Rigid2d * const pose_estimate,
  ceres::Solver::Summary * const summary) const
{
  double ceres_pose_estimate[3] = {initial_pose_estimate.translation().x(),
    initial_pose_estimate.translation().y(),
    initial_pose_estimate.rotation().angle()};
  ceres::Problem problem;
  CHECK_GT(m_param.occupied_space_weight, 0.);
  problem.AddResidualBlock(
    CreateOccupiedSpaceCostFunction2D(
      m_param.occupied_space_weight /
      std::sqrt(static_cast<double>(point_cloud_carto.points.size())),
      point_cloud_carto, grid),
    nullptr /* loss function */, ceres_pose_estimate);

  CHECK_GT(m_param.rotation_weight, 0.);
  problem.AddResidualBlock(
    TranslationDeltaCostFunctor2D::CreateAutoDiffCostFunction(
      m_param.translation_weight, target_translation),
    nullptr /* loss function */, ceres_pose_estimate);
  CHECK_GT(m_param.rotation_weight, 0.);
  problem.AddResidualBlock(
    RotationDeltaCostFunctor2D::CreateAutoDiffCostFunction(
      m_param.rotation_weight, ceres_pose_estimate[2]),
    nullptr /* loss function */, ceres_pose_estimate);

  ceres::Solve(m_ceres_solver_options, &problem, summary);

  *pose_estimate = transform::Rigid2d(
    {ceres_pose_estimate[0], ceres_pose_estimate[1]}, ceres_pose_estimate[2]);
}

}  // namespace mapping
