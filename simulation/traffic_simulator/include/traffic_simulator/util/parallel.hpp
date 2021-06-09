// Copyright 2015-2020 Tier IV, Inc. All rights reserved.
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

#ifndef TRAFFIC_SIMULATOR__UTIL__PARALLEL_HPP_
#define TRAFFIC_SIMULATOR__UTIL__PARALLEL_HPP_

#include <algorithm>
#include <utility>
#include <vector>

namespace util
{
namespace parallel
{
template <class Iterator, class Function>
inline Function forEach(Iterator begin, Iterator end, Function f)
{
  if (begin == end) {
    return std::move(f);
  }

  std::size_t num_threads = std::thread::hardware_concurrency();
  std::size_t step = std::max<std::size_t>(1, std::distance(begin, end) / num_threads);

  std::vector<std::thread> threads;

  for (; begin < end - step; begin += step) {
    threads.emplace_back([=, &f]() { std::for_each(begin, begin + step, f); });
  }

  threads.emplace_back([=, &f]() { std::for_each(begin, end, f); });

  for (auto && t : threads) {
    t.join();
  }

  return std::move(f);
}

template <class Container, class Function>
inline Function forEach(Container && c, Function f)
{
  return forEach(std::begin(c), std::end(c), f);
}
}
}  // namespace util

#endif  // TRAFFIC_SIMULATOR__UTIL__PARALLEL_FOR_EACH_HPP_
