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

#include <openscenario_interpreter/syntax/reference_context.hpp>
#include <string>

namespace openscenario_interpreter
{
inline namespace syntax
{
static_assert(std::is_standard_layout<ReferenceContext>::value, "");

static_assert(std::is_trivial<ReferenceContext>::value, "");

auto operator>>(std::istream & is, ReferenceContext & context) -> std::istream &
{
  std::string buffer{};

  is >> buffer;

#define BOILERPLATE(IDENTIFIER)                   \
  if (buffer == #IDENTIFIER) {                    \
    context.value = ReferenceContext::IDENTIFIER; \
    return is;                                    \
  }                                               \
  static_assert(true, "")

  BOILERPLATE(relative);

#undef BOILERPLATE

#define BOILERPLATE(IDENTIFIER)                                              \
  if (buffer == #IDENTIFIER) {                                               \
    throw UNSUPPORTED_ENUMERATION_VALUE_SPECIFIED(ReferenceContext, buffer); \
  }                                                                          \
  static_assert(true, "")

  BOILERPLATE(absolute);

#undef BOILERPLATE

  throw UNEXPECTED_ENUMERATION_VALUE_SPECIFIED(ReferenceContext, buffer);
}

auto operator<<(std::ostream & os, const ReferenceContext & datum) -> std::ostream &
{
#define BOILERPLATE(ID)      \
  case ReferenceContext::ID: \
    return os << #ID;

  switch (datum) {
    BOILERPLATE(absolute);
    BOILERPLATE(relative);

    default:
      throw UNEXPECTED_ENUMERATION_VALUE_ASSIGNED(ReferenceContext, datum);
  }

#undef BOILERPLATE
}
}  // namespace syntax
}  // namespace openscenario_interpreter
