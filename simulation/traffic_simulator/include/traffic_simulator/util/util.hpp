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

#ifndef TRAFFIC_SIMULATOR__UTIL__UTIL_HPP_
#define TRAFFIC_SIMULATOR__UTIL__UTIL_HPP_

template <typename T>
auto operator+(const std::vector<T> & v0, const std::vector<T> & v1) -> decltype(auto)
{
  auto result = v0;
  result.reserve(v0.size() + v1.size());
  result.insert(result.end(), v1.begin(), v1.end());
  return result;
}

template <typename T>
auto operator+=(std::vector<T> & v0, const std::vector<T> & v1) -> decltype(auto)
{
  v0.reserve(v0.size() + v1.size());
  v0.insert(v0.end(), v1.begin(), v1.end());
  return v0;
}

#endif  // TRAFFIC_SIMULATOR__UTIL__UTIL_HPP_
