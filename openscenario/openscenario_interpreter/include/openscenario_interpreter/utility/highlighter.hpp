// Copyright 2015-2020 Autoware Foundation. All rights reserved.
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

#ifndef OPENSCENARIO_INTERPRETER__UTILITY__HIGHLIGHTER_HPP_
#define OPENSCENARIO_INTERPRETER__UTILITY__HIGHLIGHTER_HPP_

#include <openscenario_interpreter/type_traits/has_stream_output_operator.hpp>

#include <iostream>
#include <string>
#include <utility>

namespace openscenario_interpreter
{
inline namespace utility
{
struct AttributeHighlighter
{
  const std::string name, value;

  template<typename ... Ts>
  decltype(auto) operator()(std::basic_ostream<Ts...>&os) const
  {
    return os << yellow << name << reset << "=" << cyan << "\"" << value << "\"" << reset;
  }
};

template<typename ... Ts>
decltype(auto) operator<<(std::basic_ostream<Ts...>&os, const AttributeHighlighter & highlight)
{
  return highlight(os);
}

template<typename T,
  typename = typename std::enable_if<HasStreamOutputOperator<T>::value>::type>
auto highlight(const std::string & name, const T & value)
{
  return AttributeHighlighter {name, boost::lexical_cast<std::string>(value)};
}
}
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__UTILITY__HIGHLIGHTER_HPP_
