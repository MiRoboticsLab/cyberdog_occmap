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

#include <glog/logging.h>

#include <limits>
#include <memory>
#include <vector>
#include <tuple>

#include "map2d/value_conversion_tables.hpp"

namespace mapping
{

constexpr uint16_t kUpdateMarker = 1u << 15;

// 0 is unknown, [1, 32767] maps to [lower_bound, upper_bound].
float SlowValueToBoundedFloat(
  const uint16_t value, const uint16_t unknown_value,
  const float unknown_result,
  const float lower_bound,
  const float upper_bound)
{
  CHECK_LE(value, 32767);
  if (value == unknown_value) {return unknown_result;}
  const float kScale = (upper_bound - lower_bound) / 32766.f;
  return value * kScale + (lower_bound - kScale);
}

std::unique_ptr<std::vector<float>> PrecomputeValueToBoundedFloat(
  const uint16_t unknown_value, const float unknown_result,
  const float lower_bound, const float upper_bound)
{
  auto result = std::make_unique<std::vector<float>>();
  size_t num_values = std::numeric_limits<uint16_t>::max() + 1;
  result->reserve(num_values);
  for (size_t value = 0; value != num_values; ++value) {
    result->push_back(
      SlowValueToBoundedFloat(
        static_cast<uint16_t>(value) & ~kUpdateMarker, unknown_value,
        unknown_result, lower_bound, upper_bound));
  }
  return result;
}

const std::vector<float> * ValueConversionTables::GetConversionTable(
  float unknown_result, float lower_bound, float upper_bound)
{
  std::tuple<float, float, float> bounds =
    std::make_tuple(unknown_result, lower_bound, upper_bound);
  auto lookup_table_iterator = bounds_to_lookup_table_.find(bounds);
  if (lookup_table_iterator == bounds_to_lookup_table_.end()) {
    auto insertion_result = bounds_to_lookup_table_.emplace(
      bounds, PrecomputeValueToBoundedFloat(
        0, unknown_result, lower_bound,
        upper_bound));
    return insertion_result.first->second.get();
  } else {
    return lookup_table_iterator->second.get();
  }
}

}  // namespace mapping
