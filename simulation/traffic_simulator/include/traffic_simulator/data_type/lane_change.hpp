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

#ifndef TRAFFIC_SIMULATOR__DATA_TYPE__LANE_CHANGE_HPP_
#define TRAFFIC_SIMULATOR__DATA_TYPE__LANE_CHANGE_HPP_

#include <lanelet2_core/Forward.h>

#include <iostream>
#include <scenario_simulator_exception/exception.hpp>
#include <type_traits>
#include <unordered_map>

namespace traffic_simulator
{
namespace lane_change
{
enum class Direction { straight = 0, left = 1, right = 2 };

enum class TrajectoryShape { cubic = 0, linear = 1 };

struct AbsoluteTarget
{
  explicit constexpr AbsoluteTarget(const lanelet::Id lanelet_id = 0, const double offset = 0)
  : lanelet_id(lanelet_id), offset(offset)
  {
  }
  lanelet::Id lanelet_id;
  double offset;
};

struct Constraint
{
  enum class Type { none = 0, lateral_velocity = 1, longitudinal_distance = 2, time = 3 };
  enum class Policy { force = 0, best_effort = 1 };
  explicit constexpr Constraint(
    const Type type = Type::none, const double value = 0, const Policy policy = Policy::force)
  : type(type), value(value), policy(policy)
  {
  }
  Type type;
  double value;
  Policy policy;
};

struct RelativeTarget
{
  explicit RelativeTarget(
    const std::string & entity_name, const Direction direction, const std::uint8_t shift = 0,
    const double offset = 0)
  : entity_name(entity_name), direction(direction), shift(shift), offset(offset)
  {
  }
  std::string entity_name;
  Direction direction;
  std::uint8_t shift;
  double offset;
};

/**
 * @brief parameters for behavior plugin
 */
struct Parameter
{
  explicit Parameter(
    const AbsoluteTarget & target = AbsoluteTarget(),
    const TrajectoryShape trajectory_shape = TrajectoryShape::cubic,
    const Constraint & constraint = Constraint())
  : target(target), trajectory_shape(trajectory_shape), constraint(constraint)
  {
  }
  AbsoluteTarget target;
  TrajectoryShape trajectory_shape;
  Constraint constraint;
  static inline constexpr auto default_lanechange_distance = 20.0;
};

std::ostream & operator<<(std::ostream &, const Direction &);
std::ostream & operator<<(std::ostream &, const TrajectoryShape &);
std::ostream & operator<<(std::ostream &, const Constraint::Type &);
std::ostream & operator<<(std::ostream &, const Constraint::Policy &);
std::ostream & operator<<(std::ostream &, const Parameter &);
}  // namespace lane_change
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__DATA_TYPE__LANE_CHANGE_HPP_
