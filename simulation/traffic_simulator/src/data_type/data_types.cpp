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

#include <traffic_simulator/data_type/data_types.hpp>

std::ostream & operator<<(
  std::ostream & stream, const traffic_simulator::lane_change::Direction & value)
{
  switch (value) {
    case traffic_simulator::lane_change::Direction::STRAIGHT:
      stream << "direction : STRAIGHT" << std::endl;
      break;
    case traffic_simulator::lane_change::Direction::LEFT:
      stream << "direction : LEFT" << std::endl;
      break;
    case traffic_simulator::lane_change::Direction::RIGHT:
      stream << "direction : RIGHT" << std::endl;
      break;
  }
  return stream;
}

std::ostream & operator<<(
  std::ostream & stream, const traffic_simulator::lane_change::Trajectory & value)
{
  switch (value) {
    case traffic_simulator::lane_change::Trajectory::CUBIC:
      stream << "trajectory : CUBIC" << std::endl;
      break;
    case traffic_simulator::lane_change::Trajectory::LINEAR:
      stream << "trajectory : LINEAR" << std::endl;
      break;
    case traffic_simulator::lane_change::Trajectory::STEP:
      stream << "trajectory : STEP" << std::endl;
      break;
  }
  return stream;
}

std::ostream & operator<<(
  std::ostream & stream, const traffic_simulator::lane_change::AbsoluteTarget & value)
{
  stream << "lanelet_id : " << value.lanelet_id << std::endl;
  stream << "offset : " << value.offset << std::endl;
  return stream;
}

std::ostream & operator<<(
  std::ostream & stream, const traffic_simulator::lane_change::Constraint::Type & value)
{
  switch (value) {
    case traffic_simulator::lane_change::Constraint::Type::NONE:
      stream << "type : NONE" << std::endl;
      break;
    case traffic_simulator::lane_change::Constraint::Type::LATERAL_VELOCITY:
      stream << "type : LATERAL_VELOCITY" << std::endl;
      break;
  }
  return stream;
}

std::ostream & operator<<(
  std::ostream & stream, const traffic_simulator::lane_change::Constraint & value)
{
  stream << value.type;
  stream << "value : " << value.value << std::endl;
  return stream;
}

std::ostream & operator<<(
  std::ostream & stream, const traffic_simulator::lane_change::RelativeTarget & value)
{
  stream << "entity_name : " << value.entity_name << std::endl;
  stream << value.direction;
  stream << "shift : " << value.shift << std::endl;
  stream << "offset : " << value.offset << std::endl;
  return stream;
}

std::ostream & operator<<(
  std::ostream & stream, const traffic_simulator::lane_change::Parameter & value)
{
  stream << value.target;
  stream << value.trajectory;
  stream << value.constraint;
  return stream;
}
