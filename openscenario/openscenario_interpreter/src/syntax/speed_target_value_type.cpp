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
#include <openscenario_interpreter/syntax/speed_target_value_type.hpp>
#include <sstream>
#include <string>

namespace openscenario_interpreter
{
inline namespace syntax
{
std::istream & operator>>(std::istream & is, SpeedTargetValueType & type)
{
  std::string buffer{};

  is >> buffer;

#define BOILERPLATE(IDENTIFIER)                    \
  if (buffer == #IDENTIFIER) {                     \
    type.value = SpeedTargetValueType::IDENTIFIER; \
    return is;                                     \
  }                                                \
  static_assert(true, "")

  BOILERPLATE(delta);

#undef BOILERPLATE

#define BOILERPLATE(IDENTIFIER)                                                                   \
  if (buffer == #IDENTIFIER) {                                                                    \
    std::stringstream ss{};                                                                       \
    ss << "given value \'" << buffer                                                              \
       << "\' is valid OpenSCENARIO value of type SpeedTargetValueType, but it is not supported"; \
    throw ImplementationFault{ss.str()};                                                          \
  }                                                                                               \
  static_assert(true, "")

  BOILERPLATE(factor);

#undef BOILERPLATE

  std::stringstream ss{};
  ss << "unexpected value \'" << buffer << "\' specified as type SpeedTargetValueType";
  throw SyntaxError{ss.str()};
}

std::ostream & operator<<(std::ostream & os, const SpeedTargetValueType & type)
{
  switch (type) {
#define BOILERPLATE(NAME)          \
  case SpeedTargetValueType::NAME: \
    return os << #NAME;

    BOILERPLATE(delta);
    BOILERPLATE(factor);

#undef BOILERPLATE

    default:
      std::stringstream ss{};
      ss << "enum class SpeedTargetValueType holds unexpected value "
         << static_cast<SpeedTargetValueType::value_type>(type);
      throw ImplementationFault{ss.str()};
  }
}
}  // namespace syntax
}  // namespace openscenario_interpreter
