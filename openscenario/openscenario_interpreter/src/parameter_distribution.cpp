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

#include <openscenario_interpreter/parameter_distribution.hpp>

namespace openscenario_interpreter
{
auto mergeParameterDistribution(
  const ParameterDistribution & distribution, const ParameterDistribution & additional_distribution)
  -> ParameterDistribution
{
  if (distribution.empty() and additional_distribution.empty()) {
    throw common::Error(
      "trying to merge two parameter distributions but failed because both are empty");
  } else if (distribution.empty()) {
    return additional_distribution;
  } else if (additional_distribution.empty()) {
    return distribution;
  } else {
    ParameterDistribution merged_distribution;
    merged_distribution.reserve(distribution.size() * additional_distribution.size());
    for (const ParameterSetSharedPtr & parameter_set : distribution) {
      for (const ParameterSetSharedPtr & additional_parameter_set : additional_distribution) {
        auto merged_set = ParameterSet{*parameter_set};
        merged_set.insert(additional_parameter_set->cbegin(), additional_parameter_set->cend());
        merged_distribution.emplace_back(std::make_shared<ParameterSet>(merged_set));
      }
    }
    return merged_distribution;
  }
}
}  // namespace openscenario_interpreter
