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

#ifndef SCENARIO_RUNNER__SYNTAX__IF_NOT_NOTHROW_DEFAULT_CONSTRUCTIBLE_HPP_
#define SCENARIO_RUNNER__SYNTAX__IF_NOT_NOTHROW_DEFAULT_CONSTRUCTIBLE_HPP_

#include <type_traits>

namespace scenario_runner
{inline namespace type_traits
{
template<typename T, typename = void>
struct IfNotNothrowDefaultConstructible
{
  static T error(const std::string & parent_name, const std::string & child_name)
  {
    std::stringstream ss {};
    ss << parent_name << " requires class " << child_name <<
      " as element, but there is no specification";
    throw SyntaxError {ss.str()};
  }
};

template<typename T>
struct IfNotNothrowDefaultConstructible<T,
  typename std::enable_if<std::is_nothrow_default_constructible<T>::value>::type>
{
  template<typename ... Ts>
  static T error(Ts && ...)
  {
    return T {};
  }
};
}}  // namespace scenario_runner::type_traits

#endif  // SCENARIO_RUNNER__SYNTAX__IF_NOT_NOTHROW_DEFAULT_CONSTRUCTIBLE_HPP_
