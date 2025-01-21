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

#include <openscenario_interpreter/reader/element.hpp>
#include <openscenario_interpreter/syntax/histogram.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
Histogram::Histogram(const pugi::xml_node & node, openscenario_interpreter::Scope & scope)
: Scope(scope), bins(readElements<HistogramBin, 1>("Bin", node, scope)), distribute([this]() {
    std::vector<double> intervals, densities;
    intervals.emplace_back(bins.front().range.lower_limit.data);
    for (const auto & bin : bins) {
      intervals.emplace_back(bin.range.upper_limit.data);
      densities.emplace_back(bin.weight.data);
    }
    return std::piecewise_constant_distribution<Double::value_type>::param_type(
      intervals.begin(), intervals.end(), densities.begin());
  }())
{
}

auto Histogram::evaluate() -> Object { return make<Double>(distribute(random_engine)); }
}  // namespace syntax
}  // namespace openscenario_interpreter
