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

#ifndef MAP2D__XY_INDEX_HPP_
#define MAP2D__XY_INDEX_HPP_

#include <Eigen/Core>
#include <glog/logging.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <iterator>

namespace mapping
{

struct CellLimits
{
  CellLimits() = default;
  CellLimits(int init_num_x_cells, int init_num_y_cells)
  : num_x_cells(init_num_x_cells), num_y_cells(init_num_y_cells) {}

  int num_x_cells = 0;
  int num_y_cells = 0;
};

// Iterates in row-major order through a range of xy-indices.
class XYIndexRangeIterator
  : public std::iterator<std::input_iterator_tag, Eigen::Array2i>
{
public:
  // Constructs a new iterator for the specified range.
  XYIndexRangeIterator(
    const Eigen::Array2i & min_xy_index,
    const Eigen::Array2i & max_xy_index)
  : min_xy_index_(min_xy_index), max_xy_index_(max_xy_index), xy_index_(min_xy_index) {}

  // Constructs a new iterator for everything contained in 'cell_limits'.
  explicit XYIndexRangeIterator(const CellLimits & cell_limits)
  : XYIndexRangeIterator(
      Eigen::Array2i::Zero(),
      Eigen::Array2i(
        cell_limits.num_x_cells - 1,
        cell_limits.num_y_cells - 1)) {}

  XYIndexRangeIterator & operator++()
  {
    DCHECK(*this != end());
    if (xy_index_.x() < max_xy_index_.x()) {
      ++xy_index_.x();
    } else {
      xy_index_.x() = min_xy_index_.x();
      ++xy_index_.y();
    }
    return *this;
  }

  Eigen::Array2i & operator*() {return xy_index_;}

  bool operator==(const XYIndexRangeIterator & other) const
  {
    return (xy_index_ == other.xy_index_).all();
  }

  bool operator!=(const XYIndexRangeIterator & other) const
  {
    return !operator==(other);
  }

  XYIndexRangeIterator begin()
  {
    return XYIndexRangeIterator(min_xy_index_, max_xy_index_);
  }

  XYIndexRangeIterator end()
  {
    XYIndexRangeIterator it = begin();
    it.xy_index_ = Eigen::Array2i(min_xy_index_.x(), max_xy_index_.y() + 1);
    return it;
  }

private:
  Eigen::Array2i min_xy_index_;
  Eigen::Array2i max_xy_index_;
  Eigen::Array2i xy_index_;
};  // class XYIndexRangeIterator

}  // namespace mapping

#endif  // MAP2D__XY_INDEX_HPP_
