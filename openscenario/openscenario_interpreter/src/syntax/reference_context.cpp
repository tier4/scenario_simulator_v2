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
#include <sstream>
#include <string>

namespace openscenario_interpreter
{
inline namespace syntax
{
std::istream & operator>>(std::istream & is, ReferenceContext & context)
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

#define BOILERPLATE(IDENTIFIER)                                                               \
  if (buffer == #IDENTIFIER) {                                                                \
    std::stringstream ss{};                                                                   \
    ss << "given value \'" << buffer                                                          \
       << "\' is valid OpenSCENARIO value of type ReferenceContext, but it is not supported"; \
    throw ImplementationFault{ss.str()};                                                      \
  }                                                                                           \
  static_assert(true, "")

  BOILERPLATE(absolute);

#undef BOILERPLATE

  std::stringstream ss{};
  ss << "unexpected value \'" << buffer << "\' specified as type ReferenceContext";
  throw SyntaxError{ss.str()};
}

std::ostream & operator<<(std::ostream & os, const ReferenceContext & context)
{
  switch (context) {
#define BOILERPLATE(ID)      \
  case ReferenceContext::ID: \
    return os << #ID;

    BOILERPLATE(absolute);
    BOILERPLATE(relative);

#undef BOILERPLATE

    default:
      std::stringstream ss{};
      ss << "enum class ReferenceContext holds unexpected value "
         << static_cast<ReferenceContext::value_type>(context.value);
      throw ImplementationFault{ss.str()};
  }
}
}  // namespace syntax
}  // namespace openscenario_interpreter
