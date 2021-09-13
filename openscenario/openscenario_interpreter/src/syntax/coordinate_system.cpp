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
#include <openscenario_interpreter/syntax/coordinate_system.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
auto operator>>(std::istream & is, CoordinateSystem & datum) -> std::istream &
{
  std::string buffer;

  is >> buffer;

#define BOILERPLATE(IDENTIFIER)                 \
  if (buffer == #IDENTIFIER) {                  \
    datum.value = CoordinateSystem::IDENTIFIER; \
    return is;                                  \
  }                                             \
  static_assert(true, "")

  BOILERPLATE(entity);
  BOILERPLATE(lane);
  BOILERPLATE(load);
  BOILERPLATE(trajectory);

#undef BOILERPLATE

  throw UNEXPECTED_ENUMERATION_VALUE_SPECIFIED(CoordinateSystem, buffer);
}

auto operator<<(std::ostream & os, const CoordinateSystem & datum) -> std::ostream &
{
#define BOILERPLATE(IDENTIFIER)      \
  case CoordinateSystem::IDENTIFIER: \
    return os << #IDENTIFIER

  switch (datum) {
    BOILERPLATE(entity);
    BOILERPLATE(lane);
    BOILERPLATE(load);
    BOILERPLATE(trajectory);

    default:
      throw UNEXPECTED_ENUMERATION_VALUE_ASSIGNED(CoordinateSystem, datum);
  }

#undef BOILERPLATE
}
}  // namespace syntax
}  // namespace openscenario_interpreter
