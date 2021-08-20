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

#include <scenario_simulator_exception/exception.hpp>
#include <sstream>
#include <stdexcept>
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
    THROW_SIMULATION_ERROR(
      "an invalid value : ", value,
      "was specified for type 'traffic_simulator::TrafficLightColor'");
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
    default:                        return os << "error";
  }
  // clang-format on
}

autoware_perception_msgs::msg::LampState makeLampState(const TrafficLightColor & datum)
{
  autoware_perception_msgs::msg::LampState lamp_state;
  {
    lamp_state.confidence = 1.0;

    switch (datum) {
      case TrafficLightColor::RED:
        lamp_state.type = autoware_perception_msgs::msg::LampState::RED;
        break;

      case TrafficLightColor::GREEN:
        lamp_state.type = autoware_perception_msgs::msg::LampState::GREEN;
        break;

      case TrafficLightColor::YELLOW:
        lamp_state.type = autoware_perception_msgs::msg::LampState::YELLOW;
        break;

      default:
        std::stringstream what;
        what << "Casting TrafficLightColor::" << datum
             << " to autoware_perception_msgs::msg::LampState is not supported.";
        throw std::out_of_range(what.str());
    }
  }

  return lamp_state;
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
    THROW_SIMULATION_ERROR(
      "An invalid value : ", value,
      " was specified for type 'traffic_simulator::TrafficLightArrow'");
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
    default:                          return os << "error";
  }
  // clang-format on
}

autoware_perception_msgs::msg::LampState makeLampState(const TrafficLightArrow & datum)
{
  autoware_perception_msgs::msg::LampState lamp_state;
  {
    lamp_state.confidence = 1.0;

    switch (datum) {
      case TrafficLightArrow::STRAIGHT:
        lamp_state.type = autoware_perception_msgs::msg::LampState::UP;
        break;

      case TrafficLightArrow::LEFT:
        lamp_state.type = autoware_perception_msgs::msg::LampState::LEFT;
        break;

      case TrafficLightArrow::RIGHT:
        lamp_state.type = autoware_perception_msgs::msg::LampState::RIGHT;
        break;

      default:
        std::stringstream what;
        what << "Casting TrafficLightArrow::" << datum
             << " to autoware_perception_msgs::msg::LampState is not supported.";
        throw std::out_of_range(what.str());
    }
  }

  return lamp_state;
}
}  // namespace traffic_simulator
