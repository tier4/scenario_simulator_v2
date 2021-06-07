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

#include <openscenario_interpreter/syntax/priority.hpp>
#include <string>

namespace openscenario_interpreter
{
inline namespace syntax
{
std::istream & operator>>(std::istream & is, Priority & priority)
{
  std::string buffer;

  is >> buffer;

#define BOILERPLATE(IDENTIFIER)            \
  if (buffer == #IDENTIFIER) {             \
    priority.value = Priority::IDENTIFIER; \
    return is;                             \
  }                                        \
  static_assert(true, "")

  BOILERPLATE(overwrite);
  BOILERPLATE(parallel);

#undef BOILERPLATE

#define BOILERPLATE(IDENTIFIER)                                      \
  if (buffer == #IDENTIFIER) {                                       \
    throw UNSUPPORTED_ENUMERATION_VALUE_SPECIFIED(Priority, buffer); \
  }                                                                  \
  static_assert(true, "")

  BOILERPLATE(skip);

#undef BOILERPLATE

  throw UNEXPECTED_ENUMERATION_VALUE_SPECIFIED(Priority, buffer);
}

std::ostream & operator<<(std::ostream & os, const Priority & datum)
{
#define BOILERPLATE(NAME) \
  case Priority::NAME:    \
    return os << #NAME;

  switch (datum) {
    BOILERPLATE(overwrite);
    BOILERPLATE(skip);
    BOILERPLATE(parallel);

    default:
      throw UNEXPECTED_ENUMERATION_VALUE_ASSIGNED(Priority, datum);
  }

#undef BOILERPLATE
}
}  // namespace syntax
}  // namespace openscenario_interpreter
