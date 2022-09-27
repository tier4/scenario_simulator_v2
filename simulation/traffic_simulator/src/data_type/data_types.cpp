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

#include <traffic_simulator/data_type/data_types.hpp>

namespace traffic_simulator
{
namespace behavior
{
std::ostream & operator<<(std::ostream & stream, const Request & value)
{
  switch (value) {
    case Request::NONE:
      stream << "request : NONE" << std::endl;
      break;
    case Request::LANE_CHANGE:
      stream << "request : LANE_CHANGE" << std::endl;
      break;
    case Request::FOLLOW_LANE:
      stream << "request : FOLLOW_LANE" << std::endl;
      break;
    case Request::WALK_STRAIGHT:
      stream << "request : WALK_STRAIGHT" << std::endl;
      break;
  }
  return stream;
}

std::string getRequestString(const Request & value)
{
  switch (value) {
    case Request::NONE:
      return "none";
      break;
    case Request::LANE_CHANGE:
      return "lane_change";
      break;
    case Request::FOLLOW_LANE:
      return "follow_lane";
      break;
    case Request::WALK_STRAIGHT:
      return "walk_straight";
      break;
  }
  THROW_SEMANTIC_ERROR(value, " is invalid");
}
}  // namespace behavior

namespace speed_change
{
Constraint::Constraint(const Constraint::Type type, const double value) : type(type), value(value)
{
}

RelativeTargetSpeed::RelativeTargetSpeed(
  const std::string & reference_entity_name, const RelativeTargetSpeed::Type type,
  const double value)
: reference_entity_name(reference_entity_name), type(type), value(value)
{
}

double RelativeTargetSpeed::getAbsoluteValue(
  const std::unordered_map<std::string, traffic_simulator_msgs::msg::EntityStatus> & other_status)
  const
{
  if (const auto iter = other_status.find(reference_entity_name); iter == other_status.end()) {
    THROW_SEMANTIC_ERROR(
      "Reference entity name ", std::quoted(reference_entity_name),
      " is invalid. Please check entity ", std::quoted(reference_entity_name),
      " exists and not a same entity you want to request changing target speed.");
  } else {
    switch (type) {
      default:
      case Type::DELTA:
        return iter->second.action_status.twist.linear.x + value;
      case Type::FACTOR:
        return iter->second.action_status.twist.linear.x * value;
    }
  }
}
}  // namespace speed_change

namespace lane_change
{
AbsoluteTarget::AbsoluteTarget(std::int64_t lanelet_id) : lanelet_id(lanelet_id), offset(0) {}

AbsoluteTarget::AbsoluteTarget(std::int64_t lanelet_id, double offset)
: lanelet_id(lanelet_id), offset(offset)
{
}

AbsoluteTarget::AbsoluteTarget(const AbsoluteTarget & other)
: lanelet_id(other.lanelet_id), offset(other.offset)
{
}

AbsoluteTarget & AbsoluteTarget::operator=(const AbsoluteTarget & other)
{
  if (this == &other) {
    return *this;
  }
  this->~AbsoluteTarget();
  new (this) AbsoluteTarget(other);
  return *this;
}

Constraint::Constraint() : type(Type::NONE), policy(Policy::FORCE), value(0) {}

Constraint::Constraint(const Type & type, double value)
: type(type), policy(Policy::FORCE), value(value)
{
}

Constraint::Constraint(const Type & type, const Policy & policy, double value)
: type(type), policy(policy), value(value)
{
}

Constraint::Constraint(const Constraint & other)
: type(other.type), policy(other.policy), value(other.value)
{
}

Constraint & Constraint::operator=(const Constraint & other)
{
  if (this == &other) {
    return *this;
  }
  this->~Constraint();
  new (this) Constraint(other);
  return *this;
}

RelativeTarget::RelativeTarget(
  const std::string & entity_name, const Direction direction, uint8_t shift, double offset)
: entity_name(entity_name), direction(direction), shift(shift), offset(offset)
{
}

double Parameter::default_lanechange_distance = 20.0;

Parameter::Parameter()
: target(AbsoluteTarget(0)), trajectory_shape(TrajectoryShape::CUBIC), constraint(Constraint())
{
}

Parameter::Parameter(
  const AbsoluteTarget & target, const TrajectoryShape trajectory_shape,
  const Constraint & constraint)
: target(target), trajectory_shape(trajectory_shape), constraint(constraint)
{
}

Parameter::Parameter(const Parameter & other)
: target(other.target), trajectory_shape(other.trajectory_shape), constraint(other.constraint)
{
}

Parameter & Parameter::operator=(const Parameter & other)
{
  if (this == &other) {
    return *this;
  }
  this->~Parameter();
  new (this) Parameter(other);
  return *this;
}

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

std::ostream & operator<<(std::ostream & stream, const TrajectoryShape & value)
{
  switch (value) {
    case TrajectoryShape::CUBIC:
      stream << "trajectory shape : CUBIC" << std::endl;
      break;
    case TrajectoryShape::LINEAR:
      stream << "trajectory shape : LINEAR" << std::endl;
      break;
  }
  return stream;
}

std::ostream & operator<<(std::ostream & stream, const AbsoluteTarget & value)
{
  stream << "lanelet_id : " << value.lanelet_id << std::endl;
  stream << "offset : " << value.offset << std::endl;
  return stream;
}

std::ostream & operator<<(std::ostream & stream, const Constraint::Policy & value)
{
  switch (value) {
    case Constraint::Policy::FORCE:
      stream << "policy : FORCE" << std::endl;
      break;
    case Constraint::Policy::BEST_EFFORT:
      stream << "policy : BEST_EFFORT" << std::endl;
      break;
  }
  return stream;
}

std::ostream & operator<<(std::ostream & stream, const Constraint::Type & value)
{
  switch (value) {
    case Constraint::Type::NONE:
      stream << "type : NONE" << std::endl;
      break;
    case Constraint::Type::LATERAL_VELOCITY:
      stream << "type : LATERAL_VELOCITY" << std::endl;
      break;
    case Constraint::Type::LONGITUDINAL_DISTANCE:
      stream << "type : LONGITUDINAL_DISTANCE" << std::endl;
      break;
    case Constraint::Type::TIME:
      stream << "type : TIME" << std::endl;
      break;
  }
  return stream;
}

std::ostream & operator<<(std::ostream & stream, const Constraint & value)
{
  stream << value.type;
  stream << value.policy;
  stream << "value : " << value.value << std::endl;
  return stream;
}

std::ostream & operator<<(std::ostream & stream, const RelativeTarget & value)
{
  stream << "entity_name : " << value.entity_name << std::endl;
  stream << value.direction;
  stream << "shift : " << value.shift << std::endl;
  stream << "offset : " << value.offset << std::endl;
  return stream;
}

std::ostream & operator<<(std::ostream & stream, const Parameter & value)
{
  stream << value.target;
  stream << value.trajectory_shape;
  stream << value.constraint;
  return stream;
}
}  // namespace lane_change
}  // namespace traffic_simulator
