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

#include <simulation_interface/conversions.hpp>
#include <simulation_interface/operators.hpp>

namespace simulation_interface
{
std::ostream & operator<<(
  std::ostream & os, const traffic_simulator_msgs::msg::EntityType & entity_type)
{
  static const std::unordered_map<uint8_t, std::string> string_map = {
    {EntityType::EGO, "EGO"},
    {EntityType::VEHICLE, "VEHICLE"},
    {EntityType::PEDESTRIAN, "PEDESTRIAN"},
    {EntityType::MISC_OBJECT, "MISC_OBJECT"}};

  if (const auto & it = string_map.find(entity_type.type); it != string_map.end()) {
    os << it->second;
    return os;
  } else {
    throw common::Error(
      "Unknown traffic_simulator_msgs::msgs::EntityType enumeration: " +
      std::to_string(static_cast<int>(entity_type.type)));
  }
}

std::ostream & operator<<(std::ostream & os, const traffic_simulator_msgs::EntityType & entity_type)
{
  const traffic_simulator_msgs::msg::EntityType message;
  toMsg(entity_type, message);
  os << message;
  return os;
}

std::ostream & operator<<(
  std::ostream & os, const traffic_simulator_msgs::msg::EntitySubtype & entity_subtype)
{
  static const std::unordered_map<uint8_t, std::string> entity_names = {
    {EntitySubtype::UNKNOWN, "UNKNOWN"}, {EntitySubtype::CAR, "CAR"},
    {EntitySubtype::TRUCK, "TRUCK"},     {EntitySubtype::BUS, "BUS"},
    {EntitySubtype::TRAILER, "TRAILER"}, {EntitySubtype::MOTORCYCLE, "MOTORCYCLE"},
    {EntitySubtype::BICYCLE, "BICYCLE"}, {EntitySubtype::PEDESTRIAN, "PEDESTRIAN"}};

  if (const auto & it = entity_names.find(entity_subtype.value); it != entity_names.end()) {
    os << it->second;
    return os;
  } else {
    throw common::Error(
      "Unknown traffic_simulator_msgs::msgs::EntitySubtype enumeration: " +
      std::to_string(static_cast<int>(entity_subtype.value)));
  }
}

std::ostream & operator<<(
  std::ostream & os, const traffic_simulator_msgs::msg::EntitySubtype & entity_subtype)
{
  const traffic_simulator_msgs::msg::EntitySubtype message;
  toMsg(entity_subtype, message);
  os << message;
  return os;
}
}  // namespace simulation_interface
