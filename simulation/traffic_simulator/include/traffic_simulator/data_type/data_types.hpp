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

std::ostream & operator<<(std::ostream & stream, const Direction & value)
{
  switch (value) {
    case Direction::STRAIGHT:
      stream << "direction : STRAIGHT" << std::endl;
      break;
    case Direction::LEFT:
      stream << "direction : LEFT" << std::endl;
      break;
    case Direction::RIGHT:
      stream << "direction : RIGHT" << std::endl;
      break;
  }
  return stream;
}

enum class Trajectory { CUBIC = 0, LINEAR = 1, STEP = 2 };

std::ostream & operator<<(std::ostream & stream, const Trajectory & value)
{
  switch (value) {
    case Trajectory::CUBIC:
      stream << "trajectory : CUBIC" << std::endl;
      break;
    case Trajectory::LINEAR:
      stream << "trajectory : LINEAR" << std::endl;
      break;
    case Trajectory::STEP:
      stream << "trajectory : STEP" << std::endl;
      break;
  }
  return stream;
}

struct AbsoluteTarget
{
  AbsoluteTarget(std::int64_t lanelet_id) : lanelet_id(lanelet_id), offset(0) {}
  AbsoluteTarget(std::int64_t lanelet_id, double offset) : lanelet_id(lanelet_id), offset(offset) {}
  const std::int64_t lanelet_id;
  const double offset = 0;
  AbsoluteTarget(const AbsoluteTarget & other) : lanelet_id(other.lanelet_id), offset(other.offset)
  {
  }
  AbsoluteTarget operator=(const AbsoluteTarget & other)
  {
    return AbsoluteTarget(other.lanelet_id, other.offset);
  }
};

std::ostream & operator<<(std::ostream & stream, const AbsoluteTarget & value)
{
  stream << "lanelet_id : " << value.lanelet_id << std::endl;
  stream << "offset : " << value.offset << std::endl;
  return stream;
}

struct Constraint
{
  enum class Type { NONE = 0, LATERAL_VELOCITY = 1 };
  Constraint() : type(Type::NONE), value(0) {}
  Constraint(const Type & type, double value) : type(type), value(value) {}
  const Type type;
  const double value = 0;
  Constraint(const Constraint & other) : type(other.type), value(other.value) {}
  Constraint operator=(const Constraint & other) { return Constraint(other.type, other.value); }
};

std::ostream & operator<<(std::ostream & stream, const Constraint::Type & value)
{
  switch (value) {
    case Constraint::Type::NONE:
      stream << "type : NONE" << std::endl;
      break;
    case Constraint::Type::LATERAL_VELOCITY:
      stream << "type : LATERAL_VELOCITY" << std::endl;
      break;
  }
  return stream;
}

std::ostream & operator<<(std::ostream & stream, const Constraint & value)
{
  stream << value.type;
  stream << value.value << std::endl;
}

struct RelativeTarget
{
  RelativeTarget(
    const std::string & entity_name, const Direction direction, uint8_t shift, double offset)
  : entity_name(entity_name), direction(direction), shift(shift), offset(offset)
  {
  }
  const std::string entity_name;
  const Direction direction;
  const uint8_t shift = 0;
  const double offset = 0;
};

std::ostream & operator<<(std::ostream & stream, const RelativeTarget & value)
{
  stream << "entity_name : " << value.entity_name << std::endl;
  stream << value.direction;
  stream << "shift : " << value.shift << std::endl;
  stream << "offset : " << value.offset << std::endl;
}

/**
 * @brief parameters for behavior plugin
 */
struct Parameter
{
  Parameter() : target(AbsoluteTarget(0)), trajectory(Trajectory::CUBIC), constraint(Constraint())
  {
  }
  Parameter(
    const AbsoluteTarget & target, const Trajectory trajectory, const Constraint & constraint)
  : target(target), trajectory(trajectory), constraint(constraint)
  {
  }
  Parameter(const Parameter & other)
  : target(other.target), trajectory(other.trajectory), constraint(other.constraint)
  {
  }
  Parameter operator=(const Parameter & other)
  {
    return Parameter(other.target, other.trajectory, other.constraint);
  }
  const AbsoluteTarget target;
  const Trajectory trajectory;
  const Constraint constraint;
};

std::ostream & operator<<(std::ostream & stream, const Parameter & value)
{
  std::cout << value.target;
  std::count << value.trajectory;
  std::count << value.constraint;
}

}  // namespace lane_change
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__DATA_TYPE__DATA_TYPES_HPP_
