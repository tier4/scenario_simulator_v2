// Copyright 2015-2020 TierIV.inc. All rights reserved.
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

#ifndef OPENSCENARIO_INTERPRETER__ITERATOR__SIZE_HPP_
#define OPENSCENARIO_INTERPRETER__ITERATOR__SIZE_HPP_

#include <iterator>

namespace openscenario_interpreter
{
inline namespace iterator
{
template<typename T>
auto size(const T & range)
{
  return std::distance(std::begin(range), std::end(range));
}
}
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__ITERATOR__SIZE_HPP_
