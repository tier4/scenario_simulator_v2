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

#include <iostream>
#include <iterator>
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

struct QualifiedIdentifier  // NOTE: 1.4.5. Naming conventions for OpenSCENARIO references
{
  const std::vector<std::string> qualifiers;

  const std::string name;

  template <template <typename> typename Container>
  QualifiedIdentifier(const Container<std::string> given)
  : qualifiers(std::begin(given), std::prev(std::end(given))), name(given.back())
  {
  }

  QualifiedIdentifier(const std::string & given) : QualifiedIdentifier(separate(given)) {}

  static auto separate(const std::string & name) -> std::vector<std::string>
  {
    const std::string separator = "::";

    std::vector<std::string> result;

    std::size_t prev_pos = 0;

    std::size_t pos = 0;

    while ((pos = name.find(separator, prev_pos)) != std::string::npos) {
      result.push_back(name.substr(prev_pos, pos - prev_pos));
      prev_pos = pos + separator.size();
    }

    result.push_back(name.substr(prev_pos, pos));

    return result;
  }

  friend auto operator<<(std::ostream & os, const QualifiedIdentifier & identifier)
    -> std::ostream &
  {
    for (const auto & qualifier : identifier.qualifiers) {
      os << "::" << qualifier;
    }

    return os << "::" << identifier.name;
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
