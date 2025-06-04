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

#include <traffic_simulator/color_utils/color_utils.hpp>
#include <traffic_simulator/traffic_lights/traffic_light.hpp>
#include <traffic_simulator/utils/traffic_lights.hpp>

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

TrafficLight::TrafficLight(const lanelet::Id lanelet_id)
: way_id(traffic_lights::wayId(lanelet_id)),
  regulatory_elements_ids(
    traffic_lights::trafficLightRegulatoryElementIdsFromTrafficLightId(way_id)),
  positions{
    std::make_pair(Color::green, traffic_lights::bulbPosition(way_id, "green", true)),
    std::make_pair(Color::yellow, traffic_lights::bulbPosition(way_id, "yellow", true)),
    std::make_pair(Color::red, traffic_lights::bulbPosition(way_id, "red", true)),
  }
{
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
