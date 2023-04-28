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

#ifndef OPENSCENARIO_PREPROCESSOR_T4V2_HPP
#define OPENSCENARIO_PREPROCESSOR_T4V2_HPP

#include <boost/filesystem.hpp>
#include <openscenario_interpreter/parameter_distribution.hpp>
#include <openscenario_preprocessor/tojson.hpp>
#include <pugixml.hpp>

namespace openscenario_preprocessor
{
class T4V2
{
public:
  auto deriveToXoscStringScenarios(boost::filesystem::path path) -> std::vector<std::string>;

  auto generateParameterValueDistributionFromScenarioModifiers(std::string scenario_modifiers_str)
    -> pugi::xml_document;

  auto deriveScenarioWithScenarioModifiers(
    const pugi::xml_document & base_scenario_doc,
    const openscenario_interpreter::ParameterDistribution & scenario_modifier_distribution)
    -> std::vector<pugi::xml_document>;

  std::pair<std::string, std::string> splitScenarioModifiers(std::string scenario);
};
}  // namespace openscenario_preprocessor
#endif  //OPENSCENARIO_PREPROCESSOR_T4V2_HPP
