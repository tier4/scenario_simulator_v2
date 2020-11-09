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

#ifndef OPENSCENARIO_INTERPRETER__TYPE_TRAITS__IF_HAS_STREAM_OUTPUT_OPERATOR_HPP_
#define OPENSCENARIO_INTERPRETER__TYPE_TRAITS__IF_HAS_STREAM_OUTPUT_OPERATOR_HPP_

#include <openscenario_interpreter/type_traits/has_stream_output_operator.hpp>
#include <openscenario_interpreter/console/escape_sequence.hpp>
#include <openscenario_interpreter/utility/indent.hpp>

namespace openscenario_interpreter
{
inline namespace type_traits
{
template<typename T, typename = void>
struct IfHasStreamOutputOperator
{
  static std::ostream & applyIt(std::ostream & os, const T &)
  {
    return os << indent << blue << "<" << typeid(T).name() << "/>" << reset;
  }
};

template<typename T>
struct IfHasStreamOutputOperator<T,
  typename std::enable_if<
    HasStreamOutputOperator<T>::value
  >::type>
{
  static std::ostream & applyIt(std::ostream & os, const T & rhs)
  {
    return os << rhs;
  }
};
}
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__TYPE_TRAITS__IF_HAS_STREAM_OUTPUT_OPERATOR_HPP_
