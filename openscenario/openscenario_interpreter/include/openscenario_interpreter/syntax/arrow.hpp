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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__ARROW_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__ARROW_HPP_

#include <iostream>
#include <traffic_simulator/traffic_lights/traffic_light_state.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
struct Arrow
{
  enum value_type {
    blank,
    left,
    right,
    straight,
  } value;

  explicit constexpr Arrow(value_type value = {}) : value(value) {}

  constexpr operator value_type() const noexcept { return value; }

  operator traffic_simulator::TrafficLightArrow() const
  {
    switch (value) {
      case left:
        return traffic_simulator::TrafficLightArrow::LEFT;

      case blank:
        return traffic_simulator::TrafficLightArrow::NONE;

      case right:
        return traffic_simulator::TrafficLightArrow::RIGHT;

      case straight:
        return traffic_simulator::TrafficLightArrow::STRAIGHT;

      default:
        THROW_IMPLEMENTATION_FAULT();
    }
  }
};

std::istream & operator>>(std::istream &, Arrow &);

std::ostream & operator<<(std::ostream &, const Arrow &);
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__ARROW_HPP_
