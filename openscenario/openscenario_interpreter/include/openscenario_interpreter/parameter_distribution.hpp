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
// data container types of distribution
using ParameterList = std::unordered_map<std::string, Object>;
using ParameterListSharedPtr = std::shared_ptr<ParameterList>;
using ParameterDistribution = std::vector<ParameterListSharedPtr>;
using SingleUnnamedParameterDistribution = std::vector<Object>;

struct ParallelDerivableParameterDistributionBase
{
  virtual auto derive(
    std::size_t local_index, std::size_t local_size, std::size_t global_index,
    std::size_t global_size) -> ParameterList = 0;

  virtual auto getNumberOfDeriveScenarios() const -> std::size_t
  {
    throw Error("getNumberOfDeriveScenarios() is not implemented");
  }
};

// generator types distribution
struct SingleParameterDistributionBase : public ParallelDerivableParameterDistributionBase
{
  virtual auto derive() -> SingleUnnamedParameterDistribution = 0;
};

struct MultiParameterDistributionBase : public ParallelDerivableParameterDistributionBase
{
  virtual auto derive() -> ParameterDistribution = 0;
};

struct StochasticParameterDistributionBase : public ParallelDerivableParameterDistributionBase
{
  virtual auto derive() -> Object = 0;
};

// container types of distribution data generator
struct ParameterDistributionContainer : public ParallelDerivableParameterDistributionBase
{
  virtual auto derive() -> ParameterDistribution = 0;
};

auto mergeParameterDistribution(
  ParameterDistribution & distribution, ParameterDistribution && additional_distribution)
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
