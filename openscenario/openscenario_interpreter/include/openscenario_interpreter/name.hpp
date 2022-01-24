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
#include <list>
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

template <typename Name>
struct Prefixed  // NOTE: 1.4.5. Naming conventions for OpenSCENARIO references
{
  const bool absolute;

  const std::list<std::string> prefixes;

  const Name name;

  Prefixed() = delete;

  Prefixed(Prefixed &&) = default;

  Prefixed(const Prefixed &) = delete;

  explicit Prefixed(bool absolute, const std::list<std::string> & prefixes, const Name & name)
  : absolute(absolute), prefixes(prefixes), name(name)
  {
  }

  explicit Prefixed(const std::vector<std::string> given)
  : absolute(not given.empty() and given.front().empty()),
    prefixes(
      absolute ? std::next(std::begin(given)) : std::begin(given), std::prev(std::end(given))),
    name(given.back())
  {
  }

  Prefixed(const std::string & given) : Prefixed(separate(given)) {}

  template <std::size_t N>
  auto strip() const
  {
    return Prefixed(false, {std::next(std::begin(prefixes), N), std::end(prefixes)}, name);
  }

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

  friend auto operator<<(std::ostream & os, const Prefixed & prefixed_name) -> std::ostream &
  {
    if (prefixed_name.absolute) {
      os << "{root}::";
    }

    for (const auto & prefix : prefixed_name.prefixes) {
      os << (prefix.empty() ? "{anonymous}" : prefix.c_str()) << "::";
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
