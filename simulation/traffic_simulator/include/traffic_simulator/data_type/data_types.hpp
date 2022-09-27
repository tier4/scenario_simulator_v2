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
  auto operator=(const RelativeTargetSpeed &) -> RelativeTargetSpeed & = default;
  auto operator=(RelativeTargetSpeed &&) -> RelativeTargetSpeed & = default;
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
  AbsoluteTarget(std::int64_t lanelet_id);
  AbsoluteTarget(std::int64_t lanelet_id, double offset);
  AbsoluteTarget(const AbsoluteTarget & other);
  AbsoluteTarget & operator=(const AbsoluteTarget & other);
  const std::int64_t lanelet_id;
  const double offset = 0;
};

struct Constraint
{
  enum class Type { NONE = 0, LATERAL_VELOCITY = 1, LONGITUDINAL_DISTANCE = 2, TIME = 3 };
  enum class Policy { FORCE = 0, BEST_EFFORT = 1 };
  Constraint();
  Constraint(const Type & type, double value);
  Constraint(const Type & type, const Policy & policy, double value);
  Constraint(const Constraint & other);
  Constraint & operator=(const Constraint & other);
  const Type type;
  const Policy policy;
  const double value = 0;
};

struct RelativeTarget
{
  RelativeTarget(
    const std::string & entity_name, const Direction direction, uint8_t shift, double offset);
  const std::string entity_name;
  const Direction direction;
  const uint8_t shift = 0;
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
  Parameter(const Parameter & other);
  Parameter & operator=(const Parameter & other);
  const AbsoluteTarget target;
  const TrajectoryShape trajectory_shape;
  const Constraint constraint;
  static double default_lanechange_distance;
};

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
