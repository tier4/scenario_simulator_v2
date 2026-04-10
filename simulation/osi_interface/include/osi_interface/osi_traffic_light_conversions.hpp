// Copyright 2025 TIER IV, Inc. All rights reserved.
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

#ifndef OSI_INTERFACE__OSI_TRAFFIC_LIGHT_CONVERSIONS_HPP_
#define OSI_INTERFACE__OSI_TRAFFIC_LIGHT_CONVERSIONS_HPP_

#include <osi3/osi_trafficlight.pb.h>

#include <cstdint>
#include <osi_interface/entity_id_registry.hpp>
#include <string>
#include <vector>

namespace osi_interface
{
// Lightweight representation of a single traffic light bulb (matches simulation_api_schema)
struct TrafficLightBulb
{
  enum class Color { RED = 0, AMBER = 1, GREEN = 2, WHITE = 3, UNKNOWN = 4 };
  enum class Shape {
    CIRCLE = 0,
    LEFT_ARROW = 1,
    RIGHT_ARROW = 2,
    UP_ARROW = 3,
    UP_LEFT_ARROW = 4,
    UP_RIGHT_ARROW = 5,
    DOWN_ARROW = 6,
    DOWN_LEFT_ARROW = 7,
    DOWN_RIGHT_ARROW = 8,
    CROSS = 9,
    UNKNOWN = 10
  };
  enum class Status { SOLID_OFF = 0, SOLID_ON = 1, FLASHING = 2, UNKNOWN = 3 };

  Color color{Color::UNKNOWN};
  Shape shape{Shape::UNKNOWN};
  Status status{Status::UNKNOWN};
  float confidence{1.0f};
};

// Lightweight representation of a traffic signal group (matches simulation_api_schema)
struct TrafficSignalGroup
{
  int32_t lanelet_id{0};
  std::vector<TrafficLightBulb> bulbs;
  std::vector<int32_t> relation_ids;
};

// Convert a TrafficSignalGroup → vector of osi3::TrafficLight (one per bulb)
auto toOsiTrafficLights(const TrafficSignalGroup & signal, EntityIdRegistry & registry)
  -> std::vector<osi3::TrafficLight>;

// Convert a vector of osi3::TrafficLight (same signal group) → TrafficSignalGroup
auto fromOsiTrafficLights(
  const std::vector<osi3::TrafficLight> & lights, const EntityIdRegistry & registry)
  -> TrafficSignalGroup;

// Individual enum conversions
auto toOsiColor(TrafficLightBulb::Color color) -> osi3::TrafficLight::Classification::Color;
auto fromOsiColor(osi3::TrafficLight::Classification::Color color) -> TrafficLightBulb::Color;
auto toOsiIcon(TrafficLightBulb::Shape shape) -> osi3::TrafficLight::Classification::Icon;
auto fromOsiIcon(osi3::TrafficLight::Classification::Icon icon) -> TrafficLightBulb::Shape;
auto toOsiMode(TrafficLightBulb::Status status) -> osi3::TrafficLight::Classification::Mode;
auto fromOsiMode(osi3::TrafficLight::Classification::Mode mode) -> TrafficLightBulb::Status;

}  // namespace osi_interface

#endif  // OSI_INTERFACE__OSI_TRAFFIC_LIGHT_CONVERSIONS_HPP_
