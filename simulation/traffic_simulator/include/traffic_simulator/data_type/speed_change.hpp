// Copyright 2015 TIER IV, Inc. All rights reserved.
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

#ifndef TRAFFIC_SIMULATOR__DATA_TYPE__SPEED_CHANGE_HPP_
#define TRAFFIC_SIMULATOR__DATA_TYPE__SPEED_CHANGE_HPP_

#include <iostream>
#include <scenario_simulator_exception/exception.hpp>
#include <traffic_simulator/data_type/entity_status.hpp>

namespace traffic_simulator
{
namespace speed_change
{
enum class Transition {
  /// @todo Add CUBIC transition.
  LINEAR,
  /// @todo Add SINUSOIDAL transition.
  STEP,
  AUTO
};

struct Constraint
{
  enum class Type {
    /// @todo Add DISTANCE constraint type.
    LONGITUDINAL_ACCELERATION,
    TIME,
    NONE
  };
  explicit constexpr Constraint(const Constraint::Type type, const double value)
  : type(type), value(value)
  {
  }
  Type type;
  double value;
};

struct RelativeTargetSpeed
{
  enum class Type {
    DELTA,
    FACTOR,
  };
  explicit RelativeTargetSpeed(
    const std::string & reference_entity_name, const RelativeTargetSpeed::Type type,
    const double value)
  : reference_entity_name(reference_entity_name), type(type), value(value)
  {
  }
  double getAbsoluteValue(
    const CanonicalizedEntityStatus & status,
    const std::unordered_map<std::string, CanonicalizedEntityStatus> & other_status) const;
  std::string reference_entity_name;
  Type type;
  double value;
};
}  // namespace speed_change
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__DATA_TYPE__SPEED_CHANGE_HPP_
