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

#ifndef TRAFFIC_SIMULATOR__DATA_TYPE__DATA_TYPES_HPP_
#define TRAFFIC_SIMULATOR__DATA_TYPE__DATA_TYPES_HPP_

#include <iostream>
#include <scenario_simulator_exception/exception.hpp>
#include <traffic_simulator_msgs/msg/entity_status.hpp>
#include <type_traits>
#include <unordered_map>

namespace traffic_simulator
{
namespace behavior
{
enum class Request {
  NONE,
  LANE_CHANGE,
  FOLLOW_LANE,
  WALK_STRAIGHT,
};
std::string getRequestString(const Request & value);
std::ostream & operator<<(std::ostream & stream, const Request & value);
}  // namespace behavior

namespace speed_change
{
enum class Transition {
  // @todo CUBIC,
  LINEAR,
  // @todo SINUSOIDAL,
  STEP
};

struct Constraint
{
  enum class Type {
    // @todo DISTANCE,
    LONGITUDINAL_ACCELERATION,
    // @todo TIME,
  };
  explicit Constraint(const Constraint::Type, const double);
  Constraint(const Constraint &) = default;
  Constraint(Constraint &&) = default;
  const Type type;
  const double value;
};

static_assert(std::is_copy_constructible_v<Constraint>);
static_assert(std::is_move_constructible_v<Constraint>);

struct RelativeTargetSpeed
{
  enum class Type {
    DELTA,
    FACTOR,
  };
  explicit RelativeTargetSpeed(const std::string &, const RelativeTargetSpeed::Type, const double);
  RelativeTargetSpeed(const RelativeTargetSpeed &) = default;
  RelativeTargetSpeed(RelativeTargetSpeed &&) = default;
  double getAbsoluteValue(
    const std::unordered_map<std::string, traffic_simulator_msgs::msg::EntityStatus> & other_status)
    const;
  const std::string reference_entity_name;
  const Type type;
  const double value;
};
}  // namespace speed_change

namespace lane_change
{
enum class Direction { STRAIGHT = 0, LEFT = 1, RIGHT = 2 };

enum class TrajectoryShape { CUBIC = 0, LINEAR = 1 };

struct AbsoluteTarget
{
  explicit constexpr AbsoluteTarget(const std::int64_t lanelet_id, const double offset = 0)
  : lanelet_id(lanelet_id), offset(offset)
  {
  }

  std::int64_t lanelet_id;
  double offset;
};

static_assert(std::is_copy_constructible_v<AbsoluteTarget>);
static_assert(std::is_move_constructible_v<AbsoluteTarget>);
static_assert(std::is_copy_assignable_v<AbsoluteTarget>);
static_assert(std::is_move_assignable_v<AbsoluteTarget>);

struct Constraint
{
  enum class Type { NONE = 0, LATERAL_VELOCITY = 1, LONGITUDINAL_DISTANCE = 2, TIME = 3 };
  enum class Policy { FORCE = 0, BEST_EFFORT = 1 };
  explicit Constraint() = default;
  Constraint(const Type & type, double value);
  Type type = Type::NONE;
  Policy policy = Policy::FORCE;
  double value = 0;
};

static_assert(std::is_copy_constructible_v<AbsoluteTarget>);
static_assert(std::is_move_constructible_v<AbsoluteTarget>);
static_assert(std::is_copy_assignable_v<AbsoluteTarget>);
static_assert(std::is_move_assignable_v<AbsoluteTarget>);

struct RelativeTarget
{
  explicit RelativeTarget(const std::string &, const Direction, const std::uint8_t, const double);
  const std::string entity_name;
  const Direction direction;
  const std::uint8_t shift = 0;
  const double offset = 0;
};

/**
 * @brief parameters for behavior plugin
 */
struct Parameter
{
  Parameter();
  Parameter(
    const AbsoluteTarget & target, const TrajectoryShape trajectory, const Constraint & constraint);
  AbsoluteTarget target;
  TrajectoryShape trajectory_shape;
  Constraint constraint;
  static inline constexpr auto default_lanechange_distance = 20.0;
};

static_assert(std::is_copy_constructible_v<Parameter>);
static_assert(std::is_move_constructible_v<Parameter>);
static_assert(std::is_copy_assignable_v<Parameter>);
static_assert(std::is_move_assignable_v<Parameter>);

std::ostream & operator<<(std::ostream & stream, const Direction & value);
std::ostream & operator<<(std::ostream & stream, const TrajectoryShape & value);
std::ostream & operator<<(std::ostream & stream, const AbsoluteTarget & value);
std::ostream & operator<<(std::ostream & stream, const Constraint::Type & value);
std::ostream & operator<<(std::ostream & stream, const Constraint::Policy & value);
std::ostream & operator<<(std::ostream & stream, const Constraint & value);
std::ostream & operator<<(std::ostream & stream, const RelativeTarget & value);
std::ostream & operator<<(std::ostream & stream, const Parameter & value);
}  // namespace lane_change
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__DATA_TYPE__DATA_TYPES_HPP_
