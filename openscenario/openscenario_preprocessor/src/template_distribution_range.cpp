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

#include <openscenario_preprocessor/template_distributions.hpp>

namespace openscenario_preprocessor
{
const std::string_view template_distribution_trange = R"###(
<OpenSCENARIO>
  <FileHeader author="" date="2022-08-26T07:17:21.031Z" description="" revMajor="0" revMinor="0"/>
  <ParameterValueDistribution>
    <ScenarioFile filepath=""/>
    <Deterministic>
      <DeterministicSingleParameterDistribution parameterName="offset">
        <DistributionRange stepWidth="">
          <Range lowerLimit="" upperLimit=""/>
        </DistributionRange>
      </DeterministicSingleParameterDistribution>
    </Deterministic>
  </ParameterValueDistribution>
</OpenSCENARIO>)###";
}  // namespace openscenario_preprocessor
