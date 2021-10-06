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
#include <openscenario_interpreter/syntax/route_strategy.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
static_assert(std::is_standard_layout<RouteStrategy>::value, "");

static_assert(std::is_trivial<RouteStrategy>::value, "");

auto operator>>(std::istream & is, RouteStrategy & datum) -> std::istream &
{
  std::string buffer;

  is >> buffer;

#define BOILERPLATE(IDENTIFIER)              \
  if (buffer == #IDENTIFIER) {               \
    datum.value = RouteStrategy::IDENTIFIER; \
    return is;                               \
  }                                          \
  static_assert(true, "")

  BOILERPLATE(shortest);

#undef BOILERPLATE

#define BOILERPLATE(IDENTIFIER)                                           \
  if (buffer == #IDENTIFIER) {                                            \
    throw UNSUPPORTED_ENUMERATION_VALUE_SPECIFIED(RouteStrategy, buffer); \
  }                                                                       \
  static_assert(true, "")

  BOILERPLATE(fastest);
  BOILERPLATE(leastIntersections);
  BOILERPLATE(random);

#undef BOILERPLATE

  throw UNEXPECTED_ENUMERATION_VALUE_SPECIFIED(RouteStrategy, buffer);
}

auto operator<<(std::ostream & os, const RouteStrategy & datum) -> std::ostream &
{
#define BOILERPLATE(NAME)   \
  case RouteStrategy::NAME: \
    return os << #NAME;

  switch (datum) {
    BOILERPLATE(fastest);
    BOILERPLATE(shortest);
    BOILERPLATE(leastIntersections);
    BOILERPLATE(random);

    default:
      throw UNEXPECTED_ENUMERATION_VALUE_ASSIGNED(RouteStrategy, datum);
  }

#undef BOILERPLATE
}
}  // namespace syntax
}  // namespace openscenario_interpreter
