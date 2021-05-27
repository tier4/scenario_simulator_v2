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
#include <sstream>
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

#define BOILERPLATE(IDENTIFIER)                                                       \
  if (buffer == #IDENTIFIER) {                                                        \
    std::stringstream ss{};                                                           \
    ss << "given value \'" << buffer                                                  \
       << "\' is valid OpenSCENARIO value of type Priority, but it is not supported"; \
    throw ImplementationFault{ss.str()};                                              \
  }                                                                                   \
  static_assert(true, "")

  BOILERPLATE(skip);

#undef BOILERPLATE

  std::stringstream ss{};
  ss << "unexpected value \'" << buffer << "\' specified as type Priority";
  throw SyntaxError{ss.str()};
}

std::ostream & operator<<(std::ostream & os, const Priority & priority)
{
  switch (priority) {
#define BOILERPLATE(NAME) \
  case Priority::NAME:    \
    return os << #NAME;

    BOILERPLATE(overwrite);
    BOILERPLATE(skip);
    BOILERPLATE(parallel);

#undef BOILERPLATE

    default:
      std::stringstream ss{};
      ss << "enum class Priority holds unexpected value "
         << static_cast<Priority::value_type>(priority);
      throw ImplementationFault{ss.str()};
  }
}
}  // namespace syntax
}  // namespace openscenario_interpreter
