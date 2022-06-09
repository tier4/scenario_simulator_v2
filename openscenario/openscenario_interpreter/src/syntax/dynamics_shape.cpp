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

#include <openscenario_interpreter/error.hpp>
#include <openscenario_interpreter/syntax/dynamics_shape.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
auto operator>>(std::istream & is, DynamicsShape & datum) -> std::istream &
{
  std::string buffer;

  is >> buffer;

#define BOILERPLATE(IDENTIFIER)              \
  if (buffer == #IDENTIFIER) {               \
    datum.value = DynamicsShape::IDENTIFIER; \
    return is;                               \
  }                                          \
  static_assert(true, "")

  BOILERPLATE(cubic);
  BOILERPLATE(linear);
  BOILERPLATE(step);

#undef BOILERPLATE

#define BOILERPLATE(IDENTIFIER)                                           \
  if (buffer == #IDENTIFIER) {                                            \
    throw UNSUPPORTED_ENUMERATION_VALUE_SPECIFIED(DynamicsShape, buffer); \
  }                                                                       \
  static_assert(true, "")

  BOILERPLATE(sinusoidal);

#undef BOILERPLATE

  throw UNEXPECTED_ENUMERATION_VALUE_SPECIFIED(DynamicsShape, buffer);
}

auto operator<<(std::ostream & os, const DynamicsShape & datum) -> std::ostream &
{
#define BOILERPLATE(NAME)   \
  case DynamicsShape::NAME: \
    return os << #NAME;

  switch (datum) {
    BOILERPLATE(linear);
    BOILERPLATE(cubic);
    BOILERPLATE(sinusoidal);
    BOILERPLATE(step);

    default:
      throw UNEXPECTED_ENUMERATION_VALUE_ASSIGNED(DynamicsShape, datum);
  }

#undef BOILERPLATE
}
}  // namespace syntax
}  // namespace openscenario_interpreter
