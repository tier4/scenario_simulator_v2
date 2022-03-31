// Copyright 2015-2021 Tier IV, Inc. All rights reserved.
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

#include <openscenario_interpreter/error.hpp>
#include <openscenario_interpreter/syntax/color.hpp>
#include <stdexcept>
#include <unordered_map>

namespace openscenario_interpreter
{
inline namespace syntax
{
static_assert(std::is_trivially_copy_assignable<Color>::value, "");

static_assert(std::is_trivially_copy_constructible<Color>::value, "");

static_assert(std::is_standard_layout<Color>::value, "");

Color::Color(const traffic_simulator::TrafficLightColor & color)
: value([](auto && color) {
    switch (color) {
      default:
      case traffic_simulator::TrafficLightColor::GREEN:
        return Color::green;

      case traffic_simulator::TrafficLightColor::RED:
        return Color::red;

      case traffic_simulator::TrafficLightColor::YELLOW:
        return Color::yellow;
    }
  }(color))
{
}

auto operator>>(std::istream & is, Color & datum) -> std::istream &
{
  std::string value;

  is >> value;

  static const std::unordered_map<std::string, Color::value_type> choice{
    // NOTE: Sorted lexicographically.
    std::make_pair("green", Color::green),
    std::make_pair("none", Color::green),
    std::make_pair("red", Color::red),
    std::make_pair("yellow", Color::yellow),
  };

  try {
    datum = choice.at(value);
  } catch (const std::out_of_range &) {
    throw UNEXPECTED_ENUMERATION_VALUE_SPECIFIED(Color, value);
  }

  return is;
}

auto operator>>(std::istream & is, boost::optional<Color> & datum) -> std::istream &
{
  std::string value;

  is >> value;

  static const std::unordered_map<std::string, Color::value_type> choice{
    // NOTE: Sorted lexicographically.
    std::make_pair("green", Color::green),
    std::make_pair("none", Color::green),
    std::make_pair("red", Color::red),
    std::make_pair("yellow", Color::yellow),
  };

  try {
    datum = choice.at(value);
  } catch (const std::out_of_range &) {
    datum = boost::none;
  }

  return is;
}

auto operator<<(std::ostream & os, const Color & datum) -> std::ostream &
{
#define BOILERPLATE(IDENTIFIER) \
  case Color::IDENTIFIER:       \
    return os << #IDENTIFIER

  switch (datum.value) {
    BOILERPLATE(green);
    BOILERPLATE(red);
    BOILERPLATE(yellow);

    default:
      throw UNEXPECTED_ENUMERATION_VALUE_ASSIGNED(Color, datum);
  }

#undef BOILERPLATE
}
}  // namespace syntax
}  // namespace openscenario_interpreter
