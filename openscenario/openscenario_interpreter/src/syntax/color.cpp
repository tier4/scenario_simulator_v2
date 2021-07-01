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

#include <openscenario_interpreter/error.hpp>
#include <openscenario_interpreter/syntax/color.hpp>
#include <unordered_map>

namespace openscenario_interpreter
{
inline namespace syntax
{
std::istream & operator>>(std::istream & is, Color & datum)
{
  std::string value;
  is >> value;

  if (value == "noColor") {
    datum = Color::noColor;
  } else if (value == "green") {
    datum = Color::green;
  } else if (value == "red") {
    datum = Color::red;
  } else if (value == "yellow") {
    datum = Color::yellow;
  } else {
    throw UNEXPECTED_ENUMERATION_VALUE_SPECIFIED(Color, value);
  }

  return is;
}

std::istream & operator>>(std::istream & is, boost::optional<Color> & datum)
{
  std::string value;
  is >> value;

  if (value == "noColor") {
    datum = Color::noColor;
  } else if (value == "green") {
    datum = Color::green;
  } else if (value == "red") {
    datum = Color::red;
  } else if (value == "yellow") {
    datum = Color::yellow;
  } else {
    datum = boost::none;
  }

  return is;
}

std::ostream & operator<<(std::ostream & os, const Color & datum)
{
  switch (datum.value) {
    case Color::noColor:
      return os << "noColor";

    case Color::green:
      return os << "green";

    case Color::red:
      return os << "red";

    case Color::yellow:
      return os << "yellow";

    default:
      throw UNEXPECTED_ENUMERATION_VALUE_ASSIGNED(Color, datum);
  }
}
}  // namespace syntax
}  // namespace openscenario_interpreter
