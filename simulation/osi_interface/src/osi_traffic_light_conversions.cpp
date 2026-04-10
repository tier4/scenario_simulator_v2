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

#include <osi_interface/osi_traffic_light_conversions.hpp>

namespace osi_interface
{
auto toOsiColor(TrafficLightBulb::Color color) -> osi3::TrafficLight::Classification::Color
{
  using C = osi3::TrafficLight::Classification;
  switch (color) {
    case TrafficLightBulb::Color::RED:
      return C::COLOR_RED;
    case TrafficLightBulb::Color::AMBER:
      return C::COLOR_YELLOW;
    case TrafficLightBulb::Color::GREEN:
      return C::COLOR_GREEN;
    case TrafficLightBulb::Color::WHITE:
      return C::COLOR_WHITE;
    default:
      return C::COLOR_UNKNOWN;
  }
}

auto fromOsiColor(osi3::TrafficLight::Classification::Color color) -> TrafficLightBulb::Color
{
  using C = osi3::TrafficLight::Classification;
  switch (color) {
    case C::COLOR_RED:
      return TrafficLightBulb::Color::RED;
    case C::COLOR_YELLOW:
      return TrafficLightBulb::Color::AMBER;
    case C::COLOR_GREEN:
      return TrafficLightBulb::Color::GREEN;
    case C::COLOR_WHITE:
      return TrafficLightBulb::Color::WHITE;
    default:
      return TrafficLightBulb::Color::UNKNOWN;
  }
}

auto toOsiIcon(TrafficLightBulb::Shape shape) -> osi3::TrafficLight::Classification::Icon
{
  using I = osi3::TrafficLight::Classification;
  switch (shape) {
    case TrafficLightBulb::Shape::CIRCLE:
      return I::ICON_NONE;
    case TrafficLightBulb::Shape::LEFT_ARROW:
      return I::ICON_ARROW_LEFT;
    case TrafficLightBulb::Shape::RIGHT_ARROW:
      return I::ICON_ARROW_RIGHT;
    case TrafficLightBulb::Shape::UP_ARROW:
      return I::ICON_ARROW_STRAIGHT_AHEAD;
    case TrafficLightBulb::Shape::UP_LEFT_ARROW:
      return I::ICON_ARROW_STRAIGHT_AHEAD_LEFT;
    case TrafficLightBulb::Shape::UP_RIGHT_ARROW:
      return I::ICON_ARROW_STRAIGHT_AHEAD_RIGHT;
    case TrafficLightBulb::Shape::DOWN_ARROW:
      return I::ICON_ARROW_DIAG_LEFT;
    case TrafficLightBulb::Shape::DOWN_LEFT_ARROW:
      return I::ICON_ARROW_DIAG_LEFT;
    case TrafficLightBulb::Shape::DOWN_RIGHT_ARROW:
      return I::ICON_ARROW_DIAG_RIGHT;
    case TrafficLightBulb::Shape::CROSS:
      return I::ICON_PEDESTRIAN_AND_BICYCLE;
    default:
      return I::ICON_UNKNOWN;
  }
}

auto fromOsiIcon(osi3::TrafficLight::Classification::Icon icon) -> TrafficLightBulb::Shape
{
  using I = osi3::TrafficLight::Classification;
  switch (icon) {
    case I::ICON_NONE:
      return TrafficLightBulb::Shape::CIRCLE;
    case I::ICON_ARROW_LEFT:
      return TrafficLightBulb::Shape::LEFT_ARROW;
    case I::ICON_ARROW_RIGHT:
      return TrafficLightBulb::Shape::RIGHT_ARROW;
    case I::ICON_ARROW_STRAIGHT_AHEAD:
      return TrafficLightBulb::Shape::UP_ARROW;
    case I::ICON_ARROW_STRAIGHT_AHEAD_LEFT:
      return TrafficLightBulb::Shape::UP_LEFT_ARROW;
    case I::ICON_ARROW_STRAIGHT_AHEAD_RIGHT:
      return TrafficLightBulb::Shape::UP_RIGHT_ARROW;
    case I::ICON_ARROW_DIAG_LEFT:
      return TrafficLightBulb::Shape::DOWN_ARROW;
    case I::ICON_ARROW_DIAG_RIGHT:
      return TrafficLightBulb::Shape::DOWN_RIGHT_ARROW;
    case I::ICON_PEDESTRIAN_AND_BICYCLE:
      return TrafficLightBulb::Shape::CROSS;
    default:
      return TrafficLightBulb::Shape::UNKNOWN;
  }
}

auto toOsiMode(TrafficLightBulb::Status status) -> osi3::TrafficLight::Classification::Mode
{
  using M = osi3::TrafficLight::Classification;
  switch (status) {
    case TrafficLightBulb::Status::SOLID_OFF:
      return M::MODE_OFF;
    case TrafficLightBulb::Status::SOLID_ON:
      return M::MODE_CONSTANT;
    case TrafficLightBulb::Status::FLASHING:
      return M::MODE_FLASHING;
    default:
      return M::MODE_UNKNOWN;
  }
}

auto fromOsiMode(osi3::TrafficLight::Classification::Mode mode) -> TrafficLightBulb::Status
{
  using M = osi3::TrafficLight::Classification;
  switch (mode) {
    case M::MODE_OFF:
      return TrafficLightBulb::Status::SOLID_OFF;
    case M::MODE_CONSTANT:
      return TrafficLightBulb::Status::SOLID_ON;
    case M::MODE_FLASHING:
      return TrafficLightBulb::Status::FLASHING;
    default:
      return TrafficLightBulb::Status::UNKNOWN;
  }
}

auto toOsiTrafficLights(const TrafficSignalGroup & signal, EntityIdRegistry & registry)
  -> std::vector<osi3::TrafficLight>
{
  std::vector<osi3::TrafficLight> result;
  result.reserve(signal.bulbs.size());

  for (const auto & bulb : signal.bulbs) {
    osi3::TrafficLight tl;

    // Each bulb gets a unique ID based on "signal_{lanelet_id}_{index}"
    const auto bulb_name =
      "traffic_light_" + std::to_string(signal.lanelet_id) + "_" + std::to_string(result.size());
    *tl.mutable_id() = registry.assign(bulb_name);

    auto * cls = tl.mutable_classification();
    cls->set_color(toOsiColor(bulb.color));
    cls->set_icon(toOsiIcon(bulb.shape));
    cls->set_mode(toOsiMode(bulb.status));

    // Assigned lane IDs from relation_ids
    for (const auto & relation_id : signal.relation_ids) {
      osi3::Identifier lane_id;
      lane_id.set_value(static_cast<uint64_t>(relation_id));
      *cls->add_assigned_lane_id() = lane_id;
    }

    // Store lanelet ID as external reference
    auto * ref = tl.add_source_reference();
    ref->set_type("net.asam.lanelet2");
    ref->add_identifier(std::to_string(signal.lanelet_id));

    result.push_back(std::move(tl));
  }

  return result;
}

auto fromOsiTrafficLights(
  const std::vector<osi3::TrafficLight> & lights, const EntityIdRegistry & /*registry*/)
  -> TrafficSignalGroup
{
  TrafficSignalGroup signal;

  for (const auto & tl : lights) {
    // Extract lanelet ID from source reference
    if (signal.lanelet_id == 0 && tl.source_reference_size() > 0) {
      const auto & ref = tl.source_reference(0);
      if (ref.identifier_size() > 0) {
        signal.lanelet_id = std::stoi(ref.identifier(0));
      }
    }

    TrafficLightBulb bulb;
    if (tl.has_classification()) {
      const auto & cls = tl.classification();
      bulb.color = fromOsiColor(cls.color());
      bulb.shape = fromOsiIcon(cls.icon());
      bulb.status = fromOsiMode(cls.mode());

      // Extract relation IDs from first bulb only
      if (signal.relation_ids.empty()) {
        for (int i = 0; i < cls.assigned_lane_id_size(); ++i) {
          signal.relation_ids.push_back(static_cast<int32_t>(cls.assigned_lane_id(i).value()));
        }
      }
    }

    signal.bulbs.push_back(bulb);
  }

  return signal;
}

}  // namespace osi_interface
