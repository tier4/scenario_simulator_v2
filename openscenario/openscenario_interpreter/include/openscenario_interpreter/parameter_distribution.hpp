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

#ifndef OPENSCENARIO_INTERPRETER__PARAMETER_DISTRIBUTION_HPP_
#define OPENSCENARIO_INTERPRETER__PARAMETER_DISTRIBUTION_HPP_

#include <memory>
#include <openscenario_interpreter/object.hpp>
#include <scenario_simulator_exception/exception.hpp>
#include <string>
#include <unordered_map>
#include <vector>

namespace openscenario_interpreter
{
// parameter set like {a: 1.0, b: 2.0,...}
using ParameterSet = std::unordered_map<std::string, Object>;
using ParameterSetSharedPtr = std::shared_ptr<ParameterSet>;
// list of ParameterList
using ParameterDistribution = std::vector<ParameterSetSharedPtr>;
using SingleUnnamedParameterDistribution = std::vector<Object>;

// TODO(HansRobo): implement parallel derivable parameter distribution with this base struct
struct ParallelDerivableParameterValueDistributionBase
{
  virtual auto derive(
    std::size_t local_index, std::size_t local_size, std::size_t global_index,
    std::size_t global_size) -> ParameterSet = 0;

  virtual auto getNumberOfDeriveScenarios() const -> std::size_t
  {
    throw Error("getNumberOfDeriveScenarios() is not implemented");
  }
};

// generator types distribution
struct SingleParameterDistributionBase
{
  virtual auto derive() -> SingleUnnamedParameterDistribution = 0;
};

struct MultiParameterDistributionBase
{
  virtual auto derive() -> ParameterDistribution = 0;
};

struct StochasticParameterDistributionBase
{
  virtual auto derive() -> Object = 0;
};

// container types of distribution data generator
struct ParameterDistributionContainer
{
  virtual auto derive() -> ParameterDistribution = 0;
};

auto mergeParameterDistribution(
  const ParameterDistribution & distribution, const ParameterDistribution & additional_distribution)
  -> ParameterDistribution;

template <typename DistributionT>
ParameterDistribution mergeParameterDistributionList(
  const ParameterDistribution & base_distribution,
  const std::list<DistributionT> & distribution_list)
{
  ParameterDistribution merged_distribution{base_distribution};
  for (const auto & additional_distribution : distribution_list) {
    merged_distribution = mergeParameterDistribution(
      merged_distribution,
      apply(
        [](const auto & distribution) { return distribution.derive(); }, additional_distribution));
  }
  return merged_distribution;
}
}  // namespace openscenario_interpreter
#endif  // OPENSCENARIO_INTERPRETER__PARAMETER_DISTRIBUTION_HPP_
