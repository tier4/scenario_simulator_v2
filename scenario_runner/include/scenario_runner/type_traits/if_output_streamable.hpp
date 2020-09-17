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

#ifndef SCENARIO_RUNNER__TYPE_TRAITS__IF_OUTPUT_STREAMABLE_HPP_
#define SCENARIO_RUNNER__TYPE_TRAITS__IF_OUTPUT_STREAMABLE_HPP_

#include <scenario_runner/concepts/output_streamable.hpp>
#include <scenario_runner/console/escape_sequence.hpp>
#include <scenario_runner/utility/indent.hpp>

namespace scenario_runner
{
inline namespace type_traits
{
template<typename T, typename = void>
struct IfOutputStreamable
{
  static std::ostream & invoke(std::ostream & os, const T &)
  {
    return os << indent << blue << "<" << typeid(T).name() << "/>" << reset;
  }
};

template<typename T>
struct IfOutputStreamable<T, typename std::enable_if<OutputStreamable<T>::value>::type>
{
  static std::ostream & invoke(std::ostream & os, const T & something)
  {
    return os << something;
  }
};
}
}  // namespace scenario_runner

#endif  // SCENARIO_RUNNER__TYPE_TRAITS__IF_OUTPUT_STREAMABLE_HPP_
