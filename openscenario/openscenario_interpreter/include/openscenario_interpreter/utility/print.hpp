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

#ifndef OPENSCENARIO_INTERPRETER__UTILITY__PRINT_HPP_
#define OPENSCENARIO_INTERPRETER__UTILITY__PRINT_HPP_

#include <iomanip>
#include <iostream>

namespace openscenario_interpreter
{
inline namespace utility
{
template <typename SequenceContainer>
auto & print_to(std::ostream & os, const SequenceContainer & sequence_container)
{
  const auto * separator = "[";

  for (const auto & each : sequence_container) {
    os << separator << each;
    separator = ", ";
  }

  return os << "]";
}

inline auto print_keys_to = [](auto & os, const auto & xs) -> decltype(auto) {
  if (not xs.empty()) {
    for (auto iter = std::begin(xs); iter != std::end(xs); ++iter) {
      os << std::get<0>(*iter);
      switch (std::distance(iter, std::end(xs))) {
        case 1:
          return os;
        case 2:
          os << " and ";
          break;
        default:
          os << ", ";
          break;
      }
    }
  }
  return os;
};
}  // namespace utility
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__UTILITY__PRINT_HPP_
