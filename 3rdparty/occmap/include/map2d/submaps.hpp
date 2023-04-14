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

#ifndef MAP2D__SUBMAPS_HPP_
#define MAP2D__SUBMAPS_HPP_

#include <glog/logging.h>

#include <cmath>

#include "occ_gridmap/rigid_transform.hpp"

namespace mapping
{

// An individual submap, which has a 'local_pose' in the local map frame, keeps
// track of how many range data were inserted into it, and sets
// 'insertion_finished' when the map no longer changes and is ready for loop
// closing.
class Submap
{
public:
  explicit Submap(const transform::Rigid3d & local_submap_pose)
  : m_local_pose(local_submap_pose) {}
  virtual ~Submap() {}

  // Pose of this submap in the local map frame.
  transform::Rigid3d local_pose() const {return m_local_pose;}

  // Number of RangeData inserted.
  int num_range_data() const {return m_num_range_data;}
  void set_num_range_data(const int num_range_data)
  {
    m_num_range_data = num_range_data;
  }

  bool insertion_finished() const {return m_insertion_finished;}
  void set_insertion_finished(bool insertion_finished)
  {
    m_insertion_finished = insertion_finished;
  }

private:
  const transform::Rigid3d m_local_pose;
  int m_num_range_data = 0;
  bool m_insertion_finished = false;
};  // class Submap
}  // namespace mapping

#endif  // MAP2D__SUBMAPS_HPP_
