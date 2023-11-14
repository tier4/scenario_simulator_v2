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
inline auto identity = [](auto && x) constexpr { return std::forward<decltype(x)>(x); };

template <typename Predicate, typename Iterable, typename Continuation = decltype(identity)>
auto filter_(Predicate satisfy, Iterable && iterable, Continuation continuation = identity)
  -> decltype(auto)
{
  auto values = std::vector<typename std::decay_t<Iterable>::iterator::value_type>();

  auto filter = [&](
                  const auto filter, const auto begin,
                  const auto end) -> std::invoke_result_t<Continuation, decltype(values)> {
    if (auto first = std::find_if(begin, end, satisfy); first != end) {
      values.push_back(*first);
      return filter(filter, std::next(first), end);
    } else {
      return std::invoke(continuation, values);
    }
  };

  return filter(filter, iterable.begin(), iterable.end());
};

inline auto filter = [](auto &&... xs) -> decltype(auto) {
  return filter_(std::forward<decltype(xs)>(xs)...);
};

// cspell: ignore insertable

inline auto append_back_insertable = [](auto && v1, auto && v2) {
  v1.insert(v1.end(), v2.begin(), v2.end());
};

// cspell: ignore Appender

template <
  typename Function, typename Iterable, typename Continuation = decltype(identity),
  typename Appender = decltype(append_back_insertable)>
auto append_map_(
  Function function, Iterable && iterable, Continuation continuation = identity,
  Appender append = append_back_insertable) -> decltype(auto)
{
  auto values =
    std::invoke_result_t<Function, typename std::decay_t<Iterable>::iterator::reference>();

  for (auto && each : iterable) {
    append(values, std::invoke(function, std::forward<decltype(each)>(each)));
  }

  return std::invoke(continuation, values);
}

inline auto append_map = [](auto &&... xs) -> decltype(auto) {
  return append_map_(std::forward<decltype(xs)>(xs)...);
};

template <typename Invocable>
constexpr auto curry2(Invocable invocable) -> decltype(auto)
{
  return [invocable](auto &&... xs) {
    return [invocable, xs = std::forward_as_tuple(xs...)](auto &&... ys) {
      return std::apply(
        [&](auto &&... xs) {
          return std::invoke(
            invocable, std::forward<decltype(xs)>(xs)..., std::forward<decltype(ys)>(ys)...);
        },
        xs);
    };
  };
}

template <typename T, typename F, typename = std::enable_if_t<std::is_invocable_v<F, T>>>
auto operator|(T && x, F f) -> decltype(auto)
{
  return f(std::forward<decltype(x)>(x));
}
}  // namespace map_fragment

#endif  // MAP_FRAGMENT__FILTER_HPP__
