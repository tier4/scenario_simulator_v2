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
#include <openscenario_interpreter/syntax/command.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
std::istream & operator>>(std::istream & is, Command & datum)
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

std::ostream & operator<<(std::ostream & os, const Command & datum)
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
