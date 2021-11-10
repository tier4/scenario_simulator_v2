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

#ifndef OPENSCENARIO_INTERPRETER__NAME_HPP_
#define OPENSCENARIO_INTERPRETER__NAME_HPP_

#include <iostream>
#include <iterator>
#include <openscenario_interpreter/error.hpp>
#include <string>

namespace openscenario_interpreter
{
struct Name : public std::string
{
  template <typename... Ts>
  Name(Ts &&... xs) : std::string(std::forward<decltype(xs)>(xs)...)
  {
    for (const auto each : ":") {
      if (find(each) != std::string::npos) {
        throw SyntaxError("Invalid name reference ", std::quoted(*this), ".");
      }
    }
  }
};

struct PrefixedName  // NOTE: 1.4.5. Naming conventions for OpenSCENARIO references
{
  // TODO
  // const bool fully_prefixed;

  const std::vector<std::string> prefixes;

  const Name name;

  explicit PrefixedName(const std::vector<std::string> given)
  : prefixes(std::begin(given), std::prev(std::end(given))), name(given.back())
  // TODO
  // : fully_prefixed(not given.empty() and given.front().empty()),
  //   prefixes(
  //     fully_prefixed ? std::next(std::begin(given)) : std::begin(given),
  //     std::prev(std::end(given))),
  //   name(given.back())
  {
  }

  PrefixedName(const std::string & given) : PrefixedName(separate(given)) {}

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

  friend auto operator<<(std::ostream & os, const PrefixedName & prefixed_name) -> std::ostream &
  {
    // if (prefixed_name.fully_prefixed) {
    //   os << "{root}::";
    // }

    for (const auto & prefix : prefixed_name.prefixes) {
      os << (prefix.empty() ? "{annonymous}" : prefix.c_str()) << "::";
    }

    return os << prefixed_name.name;
  }
};
}  // namespace openscenario_interpreter

namespace std
{
template <>
class hash<openscenario_interpreter::Name> : public hash<std::string>
{
};
}  // namespace std

#endif  // OPENSCENARIO_INTERPRETER__NAME_HPP_
