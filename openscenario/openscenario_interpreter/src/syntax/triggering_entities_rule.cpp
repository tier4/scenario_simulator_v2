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
#include <openscenario_interpreter/syntax/triggering_entities_rule.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
static_assert(std::is_standard_layout<TriggeringEntitiesRule>::value, "");

static_assert(std::is_trivial<TriggeringEntitiesRule>::value, "");

auto TriggeringEntitiesRule::description() const -> std::string
{
  switch (value) {
    case all:
      return "Are all of";

    case any:
      return "Is any of";

    case none:
      return "Is none of";

    default:
      throw UNEXPECTED_ENUMERATION_VALUE_ASSIGNED(TriggeringEntitiesRule, *this);
  }
}

auto operator>>(std::istream & is, TriggeringEntitiesRule & rule) -> std::istream &
{
  std::string buffer;

  is >> buffer;

#define BOILERPLATE(IDENTIFIER)                      \
  if (buffer == #IDENTIFIER) {                       \
    rule.value = TriggeringEntitiesRule::IDENTIFIER; \
    return is;                                       \
  }                                                  \
  static_assert(true, "")

  BOILERPLATE(all);
  BOILERPLATE(any);

#undef BOILERPLATE

  throw UNEXPECTED_ENUMERATION_VALUE_SPECIFIED(TriggeringEntitiesRule, buffer);
}

auto operator<<(std::ostream & os, const TriggeringEntitiesRule & datum) -> std::ostream &
{
#define BOILERPLATE(ID)            \
  case TriggeringEntitiesRule::ID: \
    return os << #ID;

  switch (datum) {
    BOILERPLATE(all);
    BOILERPLATE(any);

    default:
      throw UNEXPECTED_ENUMERATION_VALUE_ASSIGNED(TriggeringEntitiesRule, datum);
  }

#undef BOILERPLATE
}
}  // namespace syntax
}  // namespace openscenario_interpreter
