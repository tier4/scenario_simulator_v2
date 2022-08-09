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

#include <limits>
#include <scenario_simulator_exception/exception.hpp>
#include <string>
#include <traffic_simulator/color_utils/color_utils.hpp>
#include <traffic_simulator/traffic_lights/traffic_light.hpp>
#include <unordered_map>
#include <utility>
#include <vector>

namespace traffic_simulator
{
auto TrafficLight::Color::make(const std::string & name) -> Color
{
  try {
    return table.at(name);
  } catch (const std::out_of_range &) {
    throw common::SyntaxError("Invalid traffic light color name ", std::quoted(name), " given.");
  }
}

auto operator>>(std::istream & is, TrafficLight::Color & color) -> std::istream &
{
  std::string name;
  is >> name;
  color.value = TrafficLight::Color::make(name);
  return is;
}

auto operator<<(std::ostream & os, const TrafficLight::Color & color) -> std::ostream &
{
  switch (color.value) {
    case TrafficLight::Color::green:
      return os << "green";
    case TrafficLight::Color::yellow:
      return os << "yellow";
    case TrafficLight::Color::red:
      return os << "red";
    case TrafficLight::Color::white:
      return os << "white";
    default:
      return os;
  }
}

auto TrafficLight::Status::make(const std::string & name) -> Status
{
  try {
    return table.at(name);
  } catch (const std::out_of_range &) {
    throw common::SyntaxError("Invalid traffic light status name ", std::quoted(name), " given.");
  }
}

auto operator>>(std::istream & is, TrafficLight::Status & status) -> std::istream &
{
  std::string name;
  is >> name;
  status.value = TrafficLight::Status::make(name);
  return is;
}

auto operator<<(std::ostream & os, const TrafficLight::Status & status) -> std::ostream &
{
  switch (status.value) {
    case TrafficLight::Status::solid_on:
      return os << "solidOn";
    case TrafficLight::Status::solid_off:
      return os << "solidOff";
    case TrafficLight::Status::flashing:
      return os << "flashing";
    default:
    case TrafficLight::Status::unknown:
      return os << "unknown";
  }
}

auto TrafficLight::Shape::make(const std::string & name) -> Shape
{
  try {
    return table.at(name);
  } catch (const std::out_of_range &) {
    throw common::SyntaxError("Invalid traffic light shape name ", std::quoted(name), " given.");
  }
}

auto operator>>(std::istream & is, TrafficLight::Shape & shape) -> std::istream &
{
  std::string name;
  is >> name;
  shape.value = TrafficLight::Shape::make(name);
  return is;
}

auto operator<<(std::ostream & os, const TrafficLight::Shape & shape) -> std::ostream &
{
  switch (shape.value) {
    case TrafficLight::Shape::circle:
      return os << "circle";
    case TrafficLight::Shape::cross:
      return os << "cross";
    case TrafficLight::Shape::left:
      return os << "left";
    case TrafficLight::Shape::down:
      return os << "down";
    case TrafficLight::Shape::up:
      return os << "up";
    case TrafficLight::Shape::right:
      return os << "right";
    case TrafficLight::Shape::lower_left:
      return os << "lowerLeft";
    case TrafficLight::Shape::upper_left:
      return os << "upperLeft";
    case TrafficLight::Shape::lower_right:
      return os << "lowerRight";
    case TrafficLight::Shape::upper_right:
      return os << "upperRight";
    default:
      return os;
  }
}

auto TrafficLight::Bulb::make(const std::string & s) -> Value
{
  auto make_pattern_from = [](auto && map) {
    std::stringstream ss;
    auto const * separator = "";
    for (auto && [name, value] : map) {
      ss << separator << name;
      separator = "|";
    }
    return "(" + ss.str() + ")";
  };

  static const auto pattern = std::regex(
    R"(^)" + make_pattern_from(Color::table) + R"(?\s*)" + make_pattern_from(Status::table) +
    R"(?\s*)" + make_pattern_from(Shape::table) + R"(?$)");

  if (std::smatch result; std::regex_match(s, result, pattern)) {
    auto color = [](auto && name) { return name.empty() ? Color() : Color(name); };
    auto status = [](auto && name) { return name.empty() ? Status() : Status(name); };
    auto shape = [](auto && name) { return name.empty() ? Shape() : Shape(name); };
    return std::make_tuple(color(result.str(1)), status(result.str(2)), shape(result.str(3)));
  } else {
    throw common::SyntaxError("Invalid traffic light state ", std::quoted(s), " given.");
  }
}

auto operator<<(std::ostream & os, const TrafficLight::Bulb & bulb) -> std::ostream &
{
  return os << std::get<TrafficLight::Color>(bulb.value) << " "
            << std::get<TrafficLight::Status>(bulb.value) << " "
            << std::get<TrafficLight::Shape>(bulb.value);
}

TrafficLight::Bulb::operator autoware_auto_perception_msgs::msg::TrafficLight() const
{
  auto color = [this]() {
    switch (std::get<Color>(value).value) {
      case Color::green:
        return autoware_auto_perception_msgs::msg::TrafficLight::GREEN;
      case Color::yellow:
        return autoware_auto_perception_msgs::msg::TrafficLight::AMBER;
      case Color::red:
        return autoware_auto_perception_msgs::msg::TrafficLight::RED;
      case Color::white:
        return autoware_auto_perception_msgs::msg::TrafficLight::WHITE;
      default:
        throw common::SyntaxError(
          std::get<Color>(value),
          " is not supported as a color for autoware_auto_perception_msgs::msg::TrafficLight.");
    }
  };

  auto status = [this]() {
    switch (std::get<Status>(value).value) {
      case Status::solid_on:
        return autoware_auto_perception_msgs::msg::TrafficLight::SOLID_ON;
      case Status::solid_off:
        return autoware_auto_perception_msgs::msg::TrafficLight::SOLID_OFF;
      case Status::flashing:
        return autoware_auto_perception_msgs::msg::TrafficLight::FLASHING;
      case Status::unknown:
        return autoware_auto_perception_msgs::msg::TrafficLight::UNKNOWN;
      default:
        throw common::SyntaxError(
          std::get<Status>(value),
          " is not supported as a status for "
          "autoware_auto_perception_msgs::msg::TrafficLight.");
    }
  };

  auto shape = [this]() {
    switch (std::get<Shape>(value).value) {
      case Shape::circle:
        return autoware_auto_perception_msgs::msg::TrafficLight::CIRCLE;
      case Shape::cross:
        return autoware_auto_perception_msgs::msg::TrafficLight::CROSS;
      case Shape::left:
        return autoware_auto_perception_msgs::msg::TrafficLight::LEFT_ARROW;
      case Shape::down:
        return autoware_auto_perception_msgs::msg::TrafficLight::DOWN_ARROW;
      case Shape::up:
        return autoware_auto_perception_msgs::msg::TrafficLight::UP_ARROW;
      case Shape::right:
        return autoware_auto_perception_msgs::msg::TrafficLight::RIGHT_ARROW;
      case Shape::lower_left:
        return autoware_auto_perception_msgs::msg::TrafficLight::DOWN_LEFT_ARROW;
      case Shape::lower_right:
        return autoware_auto_perception_msgs::msg::TrafficLight::DOWN_RIGHT_ARROW;
      default:
        throw common::SyntaxError(
          std::get<Shape>(value),
          " is not supported as a shape for autoware_auto_perception_msgs::msg::TrafficLight.");
    }
  };

  autoware_auto_perception_msgs::msg::TrafficLight traffic_light;
  traffic_light.color = color();
  traffic_light.status = status();
  traffic_light.shape = shape();
  traffic_light.confidence = 1.0;
  return traffic_light;
}

TrafficLight::TrafficLight(const std::int64_t id, hdmap_utils::HdMapUtils & map_manager)
: id(id),
  positions{
    std::make_pair(
      Bulb(Color::green, Status::solid_on, Shape::circle).hash(),
      map_manager.getTrafficLightBulbPosition(id, "green")),
    std::make_pair(
      Bulb(Color::yellow, Status::solid_on, Shape::circle).hash(),
      map_manager.getTrafficLightBulbPosition(id, "yellow")),
    std::make_pair(
      Bulb(Color::red, Status::solid_on, Shape::circle).hash(),
      map_manager.getTrafficLightBulbPosition(id, "red")),
  }
{
  if (not map_manager.isTrafficLight(id)) {
    throw common::scenario_simulator_exception::Error(
      "Given lanelet ID ", id, " is not a traffic light ID.");
  }
}

auto TrafficLight::set(const std::string & states) -> void
{
  auto split = [](auto && given) {
    static const auto pattern = std::regex(R"(^(\w[\w\s]+)(,\s*)?(.*)$)");
    if (std::smatch result; std::regex_match(given, result, pattern)) {
      return std::make_pair(result.str(1), result.str(3));
    } else {
      throw common::SyntaxError("Invalid traffic light state ", std::quoted(given), " given.");
    }
  };

  if (not states.empty()) {
    auto && [head, tail] = split(states);
    emplace(head);
    set(tail);
  }
}

TrafficLight::operator autoware_auto_perception_msgs::msg::TrafficSignal() const
{
  autoware_auto_perception_msgs::msg::TrafficSignal traffic_signal;
  traffic_signal.map_primitive_id = id;
  for (auto && bulb : bulbs) {
    traffic_signal.lights.push_back(
      static_cast<autoware_auto_perception_msgs::msg::TrafficLight>(bulb));
  }
  return traffic_signal;
}

auto operator<<(std::ostream & os, const TrafficLight & traffic_light) -> std::ostream &
{
  std::string separator = "";
  for (auto && bulb : traffic_light.bulbs) {
    os << separator << bulb;
    separator = ", ";
  }
  return os;
}
}  // namespace traffic_simulator
