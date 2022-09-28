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

namespace speed_change
{
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
