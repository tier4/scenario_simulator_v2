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

#ifndef MAP_FRAGMENT__FILTER_HPP__
#define MAP_FRAGMENT__FILTER_HPP__

#include <algorithm>
#include <type_traits>
#include <vector>

namespace map_fragment
{
template <typename Predicate, typename Iterable, typename Continuation>
auto filter(Predicate && satisfy, Iterable && iterable, Continuation && current_continuation)
  -> decltype(auto)
{
  std::vector<typename std::decay_t<Iterable>::iterator::value_type> values;

  auto filter =
    [&](auto filter, auto begin, auto end) -> std::invoke_result_t<Continuation, decltype(values)> {
    if (auto first = std::find_if(begin, end, satisfy); first != end) {
      values.push_back(*first);
      return filter(filter, std::next(first), end);
    } else {
      return current_continuation(values);
    }
  };

  return filter(filter, iterable.begin(), iterable.end());
};
}  // namespace map_fragment

#endif  // MAP_FRAGMENT__FILTER_HPP__
