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

#ifndef SCENARIO_SIMULATOR_EXCEPTION__CONCATENATE_HPP_
#define SCENARIO_SIMULATOR_EXCEPTION__CONCATENATE_HPP_

#include <scenario_simulator_exception/fold.hpp>
#include <sstream>
#include <string>
#include <utility>

namespace common
{
inline namespace scenario_simulator_exception
{
inline auto concatenate = [](auto &&... xs) {
  std::stringstream result;
  (result << ... << std::forward<decltype(xs)>(xs));
  return result.str();
};
}  // namespace scenario_simulator_exception
}  // namespace common

#endif  // SCENARIO_SIMULATOR_EXCEPTION__CONCATENATE_HPP_
