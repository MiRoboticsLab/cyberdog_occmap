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

#ifndef MAP2D__PROBABILITY_VALUE_HPP_
#define MAP2D__PROBABILITY_VALUE_HPP_

#include <stdint.h>

#include <vector>

#include "occ_gridmap/common.hpp"

namespace mapping
{

namespace
{

inline uint16_t BoundedFloatToValue(
  const float float_value,
  const float lower_bound,
  const float upper_bound)
{
  const int value =
    common::RoundToInt(
    (common::Clamp(float_value, lower_bound, upper_bound) - lower_bound) *
    (32766.f / (upper_bound - lower_bound))) +
    1;
  // DCHECK for performance.
  DCHECK_GE(value, 1);
  DCHECK_LE(value, 32767);
  return value;
}

}  // namespace

inline float Odds(float probability)
{
  return probability / (1.f - probability);
}

inline float ProbabilityFromOdds(const float odds)
{
  return odds / (odds + 1.f);
}

inline float ProbabilityToCorrespondenceCost(const float probability)
{
  return 1.f - probability;
}

inline float CorrespondenceCostToProbability(const float correspondence_cost)
{
  return 1.f - correspondence_cost;
}

constexpr float kMinProbability = 0.1f;
constexpr float kMaxProbability = 1.f - kMinProbability;
constexpr float kMinCorrespondenceCost = 1.f - kMaxProbability;
constexpr float kMaxCorrespondenceCost = 1.f - kMinProbability;

// Clamps probability to be in the range [kMinProbability, kMaxProbability].
inline float ClampProbability(const float probability)
{
  return common::Clamp(probability, kMinProbability, kMaxProbability);
}

// Clamps correspondece cost to be in the range [kMinCorrespondenceCost,
// kMaxCorrespondenceCost].
inline float ClampCorrespondenceCost(const float correspondence_cost)
{
  return common::Clamp(
    correspondence_cost, kMinCorrespondenceCost,
    kMaxCorrespondenceCost);
}

constexpr uint16_t kUnknownProbabilityValue = 0;
constexpr uint16_t kUnknownCorrespondenceValue = kUnknownProbabilityValue;
constexpr uint16_t kUpdateMarker = 1u << 15;

// Converts a correspondence_cost to a uint16 in the [1, 32767] range.
inline uint16_t CorrespondenceCostToValue(const float correspondence_cost)
{
  return BoundedFloatToValue(
    correspondence_cost, kMinCorrespondenceCost,
    kMaxCorrespondenceCost);
}

// Converts a probability to a uint16 in the [1, 32767] range.
inline uint16_t ProbabilityToValue(const float probability)
{
  return BoundedFloatToValue(probability, kMinProbability, kMaxProbability);
}

extern const std::vector<float> * const kValueToProbability;
extern const std::vector<float> * const kValueToCorrespondenceCost;

// Converts a uint16 (which may or may not have the update marker set) to a
// probability in the range [kMinProbability, kMaxProbability].
inline float ValueToProbability(const uint16_t value)
{
  return (*kValueToProbability)[value];
}

// Converts a uint16 (which may or may not have the update marker set) to a
// correspondence cost in the range [kMinCorrespondenceCost,
// kMaxCorrespondenceCost].
inline float ValueToCorrespondenceCost(const uint16_t value)
{
  return (*kValueToCorrespondenceCost)[value];
}

inline uint16_t ProbabilityValueToCorrespondenceCostValue(
  uint16_t probability_value)
{
  if (probability_value == kUnknownProbabilityValue) {
    return kUnknownCorrespondenceValue;
  }
  bool update_carry = false;
  if (probability_value > kUpdateMarker) {
    probability_value -= kUpdateMarker;
    update_carry = true;
  }
  uint16_t result = CorrespondenceCostToValue(
    ProbabilityToCorrespondenceCost(ValueToProbability(probability_value)));
  if (update_carry) {result += kUpdateMarker;}
  return result;
}

inline uint16_t CorrespondenceCostValueToProbabilityValue(
  uint16_t correspondence_cost_value)
{
  if (correspondence_cost_value == kUnknownCorrespondenceValue) {
    return kUnknownProbabilityValue;
  }
  bool update_carry = false;
  if (correspondence_cost_value > kUpdateMarker) {
    correspondence_cost_value -= kUpdateMarker;
    update_carry = true;
  }
  uint16_t result = ProbabilityToValue(
    CorrespondenceCostToProbability(
      ValueToCorrespondenceCost(correspondence_cost_value)));
  if (update_carry) {result += kUpdateMarker;}
  return result;
}

std::vector<uint16_t> ComputeLookupTableToApplyOdds(float odds);
std::vector<uint16_t> ComputeLookupTableToApplyCorrespondenceCostOdds(float odds);

}  // namespace mapping

#endif  // MAP2D__PROBABILITY_VALUE_HPP_
