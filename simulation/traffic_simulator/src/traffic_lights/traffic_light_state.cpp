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

#include <sstream>
#include <string>
#include <traffic_simulator/traffic_lights/traffic_light_state.hpp>
#include <unordered_map>

namespace traffic_simulator
{
std::istream & operator>>(std::istream & is, TrafficLightColor & datum)
{
  std::string value;

  is >> value;

  static const std::unordered_map<std::string, TrafficLightColor> conversions{
    // clang-format off
    std::make_pair("none",   TrafficLightColor::NONE),
    std::make_pair("red",    TrafficLightColor::RED),
    std::make_pair("green",  TrafficLightColor::GREEN),
    std::make_pair("yellow", TrafficLightColor::YELLOW),
    // clang-format on
  };

  try {
    datum = conversions.at(value);
  } catch (const std::out_of_range &) {
    std::stringstream what;
    what << "An invalid value '" << value
         << "' was specified for type 'traffic_simulator::TrafficLightColor'.";
    throw std::runtime_error(what.str());
  }

  return is;
}

std::ostream & operator<<(std::ostream & os, const TrafficLightColor & datum)
{
  // clang-format off
  switch (datum) {
    case TrafficLightColor::NONE:   return os << "none";
    case TrafficLightColor::RED:    return os << "red";
    case TrafficLightColor::GREEN:  return os << "green";
    case TrafficLightColor::YELLOW: return os << "yellow";
  }
  // clang-format on
}

std::istream & operator>>(std::istream & is, TrafficLightArrow & datum)
{
  std::string value;

  is >> value;

  static const std::unordered_map<std::string, TrafficLightArrow> conversions{
    // clang-format off
    std::make_pair("none",     TrafficLightArrow::NONE),
    std::make_pair("straight", TrafficLightArrow::STRAIGHT),
    std::make_pair("left",     TrafficLightArrow::LEFT),
    std::make_pair("right",    TrafficLightArrow::RIGHT),
    // clang-format on
  };

  try {
    datum = conversions.at(value);
  } catch (const std::out_of_range &) {
    std::stringstream what;
    what << "An invalid value '" << value
         << "' was specified for type 'traffic_simulator::TrafficLightArrow'.";
    throw std::runtime_error(what.str());
  }

  return is;
}

std::ostream & operator<<(std::ostream & os, const TrafficLightArrow & datum)
{
  // clang-format off
  switch (datum) {
    case TrafficLightArrow::NONE:     return os << "none";
    case TrafficLightArrow::STRAIGHT: return os << "straight";
    case TrafficLightArrow::LEFT:     return os << "left";
    case TrafficLightArrow::RIGHT:    return os << "right";
  }
  // clang-format on
}

}  // namespace traffic_simulator
