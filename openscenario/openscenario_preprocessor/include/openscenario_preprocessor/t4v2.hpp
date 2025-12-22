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
#include <pugixml.hpp>

namespace openscenario_preprocessor
{
class T4V2
{
  boost::filesystem::path output_directory;

public:
  explicit T4V2(boost::filesystem::path output_directory) : output_directory(output_directory)
  {
    boost::filesystem::create_directories(output_directory / "work" / "t4v2");
  }

  auto deriveScenarioWithScenarioModifiers(
    const pugi::xml_document & base_scenario_doc,
    const openscenario_interpreter::ParameterDistribution & scenario_modifier_distribution)
    -> std::vector<pugi::xml_document>;

  auto deriveToXoscStringScenarios(
    boost::filesystem::path modifiers_path, boost::filesystem::path scenario_path = {})
    -> std::vector<std::string>;

  auto generateParameterValueDistributionFromScenarioModifiers(std::string scenario_modifiers_str)
    -> pugi::xml_document;

  auto loadScenarioFile(boost::filesystem::path path) -> pugi::xml_document;

  std::pair<boost::filesystem::path, boost::filesystem::path> splitScenarioModifiers(
    boost::filesystem::path scenario_path);
};
}  // namespace openscenario_preprocessor
#endif  //OPENSCENARIO_PREPROCESSOR_T4V2_HPP
