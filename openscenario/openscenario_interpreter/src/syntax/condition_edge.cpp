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
  std::string buffer{};

  is >> buffer;

#define BOILERPLATE(IDENTIFIER)             \
  if (buffer == #IDENTIFIER) {              \
    edge.value = ConditionEdge::IDENTIFIER; \
    return is;                              \
  }                                         \
  static_assert(true, "")

  BOILERPLATE(rising);
  BOILERPLATE(falling);
  BOILERPLATE(risingOrFalling);
  BOILERPLATE(none);
  BOILERPLATE(sticky);

#undef BOILERPLATE

  std::stringstream ss{};
  ss << "unexpected value \'" << buffer << "\' specified as type ConditionEdge";
  throw SyntaxError(ss.str());
}

std::ostream & operator<<(std::ostream & os, const ConditionEdge & edge)
{
  switch (edge) {
#define BOILERPLATE(ID)   \
  case ConditionEdge::ID: \
    return os << #ID;

    BOILERPLATE(rising);
    BOILERPLATE(falling);
    BOILERPLATE(risingOrFalling);
    BOILERPLATE(none);
    BOILERPLATE(sticky);

#undef BOILERPLATE

    default:
      std::stringstream ss{};
      ss << "enum class ConditionEdge holds unexpected value "
         << static_cast<ConditionEdge::value_type>(edge.value);
      throw ImplementationFault(ss.str());
  }
}
}  // namespace syntax
}  // namespace openscenario_interpreter
