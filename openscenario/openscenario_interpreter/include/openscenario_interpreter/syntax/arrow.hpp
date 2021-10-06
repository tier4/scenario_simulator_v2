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

#include <boost/optional.hpp>
#include <iostream>
#include <traffic_simulator/traffic_lights/traffic_light_state.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
struct Arrow
{
  enum value_type {
    none = 0,

    // NOTE: Sorted lexicographically.
    left,
    right,
    straight,
  } value;

  constexpr Arrow(value_type value = none) : value(value) {}

  explicit Arrow(const traffic_simulator::TrafficLightArrow &);

  constexpr operator value_type() const noexcept { return value; }

  constexpr operator traffic_simulator::TrafficLightArrow() const
  {
    switch (value) {
      case none:
        return traffic_simulator::TrafficLightArrow::NONE;

      case left:
        return traffic_simulator::TrafficLightArrow::LEFT;

      case right:
        return traffic_simulator::TrafficLightArrow::RIGHT;

      case straight:
        return traffic_simulator::TrafficLightArrow::STRAIGHT;

      default:
        throw UNEXPECTED_ENUMERATION_VALUE_ASSIGNED(Arrow, *this);
    }
  }
};

auto operator>>(std::istream &, Arrow &) -> std::istream &;

auto operator>>(std::istream &, boost::optional<Arrow> &) -> std::istream &;

auto operator<<(std::ostream &, const Arrow &) -> std::ostream &;
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__ARROW_HPP_
