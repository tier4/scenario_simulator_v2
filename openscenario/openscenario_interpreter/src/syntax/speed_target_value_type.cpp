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

#include <iostream>
#include <openscenario_interpreter/error.hpp>
#include <openscenario_interpreter/syntax/speed_target_value_type.hpp>
#include <string>

namespace openscenario_interpreter
{
inline namespace syntax
{
static_assert(std::is_standard_layout<SpeedTargetValueType>::value, "");

static_assert(std::is_trivial<SpeedTargetValueType>::value, "");

auto operator>>(std::istream & is, SpeedTargetValueType & datum) -> std::istream &
{
  std::string buffer;

  is >> buffer;

#define BOILERPLATE(IDENTIFIER)                     \
  if (buffer == #IDENTIFIER) {                      \
    datum.value = SpeedTargetValueType::IDENTIFIER; \
    return is;                                      \
  }                                                 \
  static_assert(true, "")

  BOILERPLATE(delta);
  BOILERPLATE(factor);

#undef BOILERPLATE

  throw UNEXPECTED_ENUMERATION_VALUE_SPECIFIED(SpeedTargetValueType, buffer);
}

auto operator<<(std::ostream & os, const SpeedTargetValueType & datum) -> std::ostream &
{
#define BOILERPLATE(NAME)          \
  case SpeedTargetValueType::NAME: \
    return os << #NAME;

  switch (datum) {
    BOILERPLATE(delta);
    BOILERPLATE(factor);

    default:
      throw UNEXPECTED_ENUMERATION_VALUE_ASSIGNED(SpeedTargetValueType, datum);
  }

#undef BOILERPLATE
}
}  // namespace syntax
}  // namespace openscenario_interpreter
