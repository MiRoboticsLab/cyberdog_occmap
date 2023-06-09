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

#ifndef MAP2D__VALUE_CONVERSION_TABLES_HPP_
#define MAP2D__VALUE_CONVERSION_TABLES_HPP_

#include <map>
#include <vector>
#include <memory>
#include <tuple>

namespace mapping
{

// Performs lazy computations of lookup tables for mapping from a uint16 value
// to a float in ['lower_bound', 'upper_bound']. The first element of the table
// is set to 'unknown_result'.
class ValueConversionTables
{
public:
  const std::vector<float> * GetConversionTable(
    float unknown_result,
    float lowe_bound,
    float upper_bound);

private:
  std::map<const std::tuple<float, float, float>,
    std::unique_ptr<const std::vector<float>>>
  bounds_to_lookup_table_;
};  // calss ValueConversionTables

}  // namespace mapping

#endif  // MAP2D__VALUE_CONVERSION_TABLES_HPP_
