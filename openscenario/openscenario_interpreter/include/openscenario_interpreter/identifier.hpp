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

#ifndef OPENSCENARIO_INTERPRETER__IDENTIFIER_HPP_
#define OPENSCENARIO_INTERPRETER__IDENTIFIER_HPP_

#include <openscenario_interpreter/error.hpp>
#include <string>

namespace openscenario_interpreter
{
struct UnqualifiedIdentifier
{
  const std::string name;

  template <typename... Ts>
  UnqualifiedIdentifier(Ts &&... xs) : name(std::forward<decltype(xs)>(xs)...)
  {
    if (name.find(':') != std::string::npos) {
      throw SyntaxError("Invalid identifier ", std::quoted(name), ".");
    }
  }
};
}  // namespace openscenario_interpreter

namespace std
{
template <>
class hash<openscenario_interpreter::UnqualifiedIdentifier> : public hash<std::string>
{
};
}  // namespace std

#endif  // OPENSCENARIO_INTERPRETER__IDENTIFIER_HPP_
