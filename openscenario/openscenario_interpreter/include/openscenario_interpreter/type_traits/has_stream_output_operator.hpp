// Copyright 2015 TIER IV, Inc. All rights reserved.
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

#ifndef OPENSCENARIO_INTERPRETER__TYPE_TRAITS__HAS_STREAM_OUTPUT_OPERATOR_HPP_
#define OPENSCENARIO_INTERPRETER__TYPE_TRAITS__HAS_STREAM_OUTPUT_OPERATOR_HPP_

#include <iostream>
#include <type_traits>

namespace openscenario_interpreter
{
inline namespace concepts
{
template <typename T, typename = void>
struct HasStreamOutputOperator : public std::false_type
{
};

template <typename T>
struct HasStreamOutputOperator<
  T, std::void_t<decltype(std::declval<std::ostream &>() << std::declval<const T &>())>>
: public std::true_type
{
};
}  // namespace concepts
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__TYPE_TRAITS__HAS_STREAM_OUTPUT_OPERATOR_HPP_
