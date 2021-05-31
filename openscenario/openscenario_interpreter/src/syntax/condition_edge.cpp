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

#include <openscenario_interpreter/syntax/condition_edge.hpp>
#include <sstream>
#include <string>

namespace openscenario_interpreter
{
inline namespace syntax
{
std::istream & operator>>(std::istream & is, ConditionEdge & edge)
{
  std::string buffer;

  is >> buffer;

#define BOILERPLATE(IDENTIFIER)             \
  if (buffer == #IDENTIFIER) {              \
    edge.value = ConditionEdge::IDENTIFIER; \
    return is;                              \
  }                                         \
  static_assert(true, "")

  BOILERPLATE(none);
  BOILERPLATE(sticky);

#undef BOILERPLATE

#define BOILERPLATE(IDENTIFIER)                                           \
  if (buffer == #IDENTIFIER) {                                            \
    throw UNSUPPORTED_ENUMERATION_VALUE_SPECIFIED(ConditionEdge, buffer); \
  }                                                                       \
  static_assert(true, "")

  BOILERPLATE(rising);
  BOILERPLATE(falling);
  BOILERPLATE(risingOrFalling);

#undef BOILERPLATE

  throw UNEXPECTED_ENUMERATION_VALUE_SPECIFIED(ConditionEdge, buffer);
}

std::ostream & operator<<(std::ostream & os, const ConditionEdge & datum)
{
#define BOILERPLATE(IDENTIFIER)   \
  case ConditionEdge::IDENTIFIER: \
    return os << #IDENTIFIER;

  switch (datum) {
    BOILERPLATE(rising);
    BOILERPLATE(falling);
    BOILERPLATE(risingOrFalling);
    BOILERPLATE(none);
    BOILERPLATE(sticky);

    default:
      throw UNEXPECTED_ENUMERATION_VALUE_ASSIGNED(ConditionEdge, datum);
  }

#undef BOILERPLATE
}
}  // namespace syntax
}  // namespace openscenario_interpreter
