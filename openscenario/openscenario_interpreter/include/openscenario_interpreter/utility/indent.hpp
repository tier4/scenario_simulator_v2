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

#ifndef OPENSCENARIO_INTERPRETER__UTILITY__INDENT_HPP_
#define OPENSCENARIO_INTERPRETER__UTILITY__INDENT_HPP_

#include <iostream>
#include <string>

namespace openscenario_interpreter
{
inline namespace utility
{
struct Indent
{
  std::size_t depth {0};

  std::size_t width {2};

  auto & reset()
  {
    depth = 0;
    return *this;
  }

  auto & operator++()
  {
    ++depth;
    return *this;
  }

  auto & operator--()
  {
    depth && --depth;
    return *this;
  }

  auto operator++(int)
  {
    Indent result {*this};
    ++depth;
    return result;
  }

  auto operator--(int)
  {
    Indent result {*this};
    depth && --depth;
    return result;
  }
} static indent;

template<typename ... Ts>
std::basic_ostream<Ts...> & operator<<(std::basic_ostream<Ts...> & os, const Indent & indent)
{
  return os << std::string(indent.depth * 2, ' ');
}
}
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__UTILITY__INDENT_HPP_
