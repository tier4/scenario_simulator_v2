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

#include <openscenario_interpreter/syntax/parameter_declaration.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
auto ParameterDeclaration::evaluate() const -> Element
{
  // clang-format off
  switch (parameter_type) {
    case ParameterType::INTEGER:        return make<Integer      >(value);
    case ParameterType::DOUBLE:         return make<Double       >(value);
    case ParameterType::STRING:         return make<String       >(value);
    case ParameterType::UNSIGNED_INT:   return make<UnsignedInt  >(value);
    case ParameterType::UNSIGNED_SHORT: return make<UnsignedShort>(value);
    case ParameterType::BOOLEAN:        return make<Boolean      >(value);
    case ParameterType::DATE_TIME:      return make<String       >(value);

    default:
      return unspecified;
  }
  // clang-format on
}

auto ParameterDeclaration::includes(const std::string & name, const std::vector<char> & chars)
  -> bool
{
  return std::any_of(std::begin(chars), std::end(chars), [&](const auto & each) {
    return name.find(each) != std::string::npos;
  });
}
}  // namespace syntax
}  // namespace openscenario_interpreter
