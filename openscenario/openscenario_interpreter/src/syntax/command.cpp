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
#include <openscenario_interpreter/syntax/command.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
static_assert(std::is_standard_layout<Command>::value, "");

static_assert(std::is_trivial<Command>::value, "");

auto Command::operator=(const value_type & rhs) -> Command &
{
  value = rhs;
  return *this;
}

auto operator>>(std::istream & is, Command & datum) -> std::istream &
{
  std::string buffer;

  is >> buffer;

#define BOILERPLATE(IDENTIFIER)  \
  if (buffer == #IDENTIFIER) {   \
    datum = Command::IDENTIFIER; \
    return is;                   \
  }

  BOILERPLATE(exitFailure);
  BOILERPLATE(exitSuccess);
  BOILERPLATE(nop);
  BOILERPLATE(print);

#undef BOILERPLATE

  throw UNEXPECTED_ENUMERATION_VALUE_SPECIFIED(Command, buffer);
}

auto operator<<(std::ostream & os, const Command & datum) -> std::ostream &
{
#define BOILERPLATE(NAME) \
  case Command::NAME:     \
    return os << #NAME;

  switch (datum) {
    BOILERPLATE(exitFailure);
    BOILERPLATE(exitSuccess);
    BOILERPLATE(nop);
    BOILERPLATE(print);

    default:
      throw UNEXPECTED_ENUMERATION_VALUE_ASSIGNED(Command, datum);
  }

#undef BOILERPLATE
}
}  // namespace syntax
}  // namespace openscenario_interpreter
