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
std::istream & operator>>(std::istream & is, Color & datum)
{
  std::string value;

  is >> value;

  static const std::unordered_map<std::string, Color::value_type> choice{
    // NOTE: Sorted lexicographically.
    std::make_pair("green", Color::green),
    std::make_pair("none", Color::none),
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

std::istream & operator>>(std::istream & is, boost::optional<Color> & datum)
{
  std::string value;

  is >> value;

  static const std::unordered_map<std::string, Color::value_type> choice{
    // NOTE: Sorted lexicographically.
    std::make_pair("green", Color::green),
    std::make_pair("none", Color::none),
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

std::ostream & operator<<(std::ostream & os, const Color & datum)
{
#define BOILERPLATE(IDENTIFIER) \
  case Color::IDENTIFIER:       \
    return os << #IDENTIFIER

  switch (datum.value) {
    BOILERPLATE(green);
    BOILERPLATE(none);
    BOILERPLATE(red);
    BOILERPLATE(yellow);

    default:
      throw UNEXPECTED_ENUMERATION_VALUE_ASSIGNED(Color, datum);
  }

#undef BOILERPLATE
}
}  // namespace syntax
}  // namespace openscenario_interpreter
