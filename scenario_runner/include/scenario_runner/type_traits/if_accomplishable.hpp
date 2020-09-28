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

#ifndef SCENARIO_RUNNER__TYPE_TRAITS__IF_ACCOMPLISHABLE_HPP_
#define SCENARIO_RUNNER__TYPE_TRAITS__IF_ACCOMPLISHABLE_HPP_

#include <scenario_runner/concepts/accomplishable.hpp>

namespace scenario_runner
{
inline namespace type_traits
{
template<typename T, typename = void>
struct IfAccomplishable
{
  static constexpr auto invoke(const T &) noexcept
  {
    return false;
  }
};

template<typename T>
struct IfAccomplishable<T, typename std::enable_if<Accomplishable<T>::value>::type>
{
  static decltype(auto) invoke(T & callee)
  {
    return callee.accomplished();
  }
};
}
}  // namespace scenario_runner

#endif  // SCENARIO_RUNNER__TYPE_TRAITS__IF_ACCOMPLISHABLE_HPP_
