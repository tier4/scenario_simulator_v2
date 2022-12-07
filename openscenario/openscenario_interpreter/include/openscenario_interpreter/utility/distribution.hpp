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

#ifndef OPENSCENARIO_INTERPRETER__DISTRIBUTION_HPP_
#define OPENSCENARIO_INTERPRETER__DISTRIBUTION_HPP_

#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/double.hpp>
#include <random>

namespace openscenario_interpreter
{
inline namespace utility
{
// TODO : rename class name
template <typename DistributionT>
struct StochasticDistributionClass
{
  template <typename... Ts>
  StochasticDistributionClass(double random_seed, Ts... xs)
  : random_engine(random_seed), distribution(xs...)
  {
  }

  std::mt19937 random_engine;

  DistributionT distribution;

  auto generate() { return distribution(random_engine); }
};
}  // namespace utility
}  // namespace openscenario_interpreter
#endif  // OPENSCENARIO_INTERPRETER__DISTRIBUTION_HPP_
