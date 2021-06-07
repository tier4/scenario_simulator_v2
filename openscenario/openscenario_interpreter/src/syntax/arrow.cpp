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
#include <openscenario_interpreter/syntax/arrow.hpp>
#include <unordered_map>

namespace openscenario_interpreter
{
inline namespace syntax
{
std::istream & operator>>(std::istream & is, Arrow & datum)
{
  std::string value;

  is >> value;

  static const std::unordered_map<std::string, Arrow::value_type> conversions{
    std::make_pair("noArrow", Arrow::noArrow),
    std::make_pair("left", Arrow::left),
    std::make_pair("right", Arrow::right),
    std::make_pair("straight", Arrow::straight),
  };

  try {
    datum.value = conversions.at(value);
  } catch (const std::out_of_range &) {
    throw UNEXPECTED_ENUMERATION_VALUE_SPECIFIED(Arrow, value);
  }

  return is;
}

std::ostream & operator<<(std::ostream & os, const Arrow & datum)
{
#define BOILERPLATE(IDENTIFIER) \
  case Arrow::IDENTIFIER:       \
    return os << #IDENTIFIER

  switch (datum.value) {
    BOILERPLATE(noArrow);
    BOILERPLATE(left);
    BOILERPLATE(right);
    BOILERPLATE(straight);

    default:
      throw UNEXPECTED_ENUMERATION_VALUE_ASSIGNED(Arrow, datum);
  }

#undef BOILERPLATE
}
}  // namespace syntax
}  // namespace openscenario_interpreter
