// Copyright 2015-2020 Tier IV, Inc. All rights reserved.
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
enum class SpeedChangeTransition {
  // @todo CUBIC,
  LINEAR,
  // @todo SINUSOIDAL,
  STEP
};

struct SpeedChangeConstraint
{
  enum class Type {
    // @todo DISTANCE,
    LONGITUDINAL_ACCELERATION,
    // @todo TIME,
  };
  SpeedChangeConstraint(Type type, double value) : type(type), value(value) {}

  const Type type;
  const double value;
};

struct RelativeTargetSpeed
{
  enum class Type {
    DELTA,
    FACTOR,
  };
  RelativeTargetSpeed(const std::string & reference_entity_name, Type type, double value)
  : reference_entity_name(reference_entity_name), type(type), value(value){};
  double getAbsoluteValue(
    const std::unordered_map<std::string, traffic_simulator_msgs::msg::EntityStatus> & other_status)
    const
  {
    if (other_status.find(reference_entity_name) == other_status.end()) {
      THROW_SEMANTIC_ERROR(
        "reference entity name : ", reference_entity_name,
        " is invalid. Please check entity : ", reference_entity_name,
        " exists and not a same entity you want to request changing target speed.");
    }
    double target_speed = 0;
    switch (type) {
      case Type::DELTA:
        target_speed =
          other_status.find(reference_entity_name)->second.action_status.twist.linear.x + value;
        break;
      case Type::FACTOR:
        target_speed =
          other_status.find(reference_entity_name)->second.action_status.twist.linear.x * value;
        break;
    }
    return target_speed;
  };
  RelativeTargetSpeed operator=(RelativeTargetSpeed val)
  {
    return RelativeTargetSpeed(val.reference_entity_name, val.type, val.value);
  }
  const std::string reference_entity_name;
  const Type type;
  const double value;
};

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
  enum class Type { NONE = 0, LATERAL_VELOCITY = 1 };
  Constraint();
  Constraint(const Type & type, double value);
  Constraint(const Constraint & other);
  Constraint & operator=(const Constraint & other);
  const Type type;
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
};

std::ostream & operator<<(std::ostream & stream, const Direction & value);
std::ostream & operator<<(std::ostream & stream, const TrajectoryShape & value);
std::ostream & operator<<(std::ostream & stream, const AbsoluteTarget & value);
std::ostream & operator<<(std::ostream & stream, const Constraint::Type & value);
std::ostream & operator<<(std::ostream & stream, const Constraint & value);
std::ostream & operator<<(std::ostream & stream, const RelativeTarget & value);
std::ostream & operator<<(std::ostream & stream, const Parameter & value);
}  // namespace lane_change
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__DATA_TYPE__DATA_TYPES_HPP_
