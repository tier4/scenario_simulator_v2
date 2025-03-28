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

#ifndef OPENSCENARIO_PREPROCESSOR__DERIVER_HPP_
#define OPENSCENARIO_PREPROCESSOR__DERIVER_HPP_

#include <boost/filesystem.hpp>
#include <openscenario_interpreter/syntax/open_scenario.hpp>
#include <openscenario_interpreter/syntax/parameter_value_distribution.hpp>
#include <openscenario_interpreter/syntax/parameter_value_distribution_definition.hpp>
#include <openscenario_validator/validator.hpp>

namespace openscenario_preprocessor
{

class Deriver
{
public:
  explicit Deriver(boost::filesystem::path schema_path) : validate(schema_path) {}
  auto operator()(boost::filesystem::path path, bool check_scenario_path = true)
    -> openscenario_interpreter::ParameterDistribution
  {
    using openscenario_interpreter::OpenScenario;
    using openscenario_interpreter::ParameterValueDistribution;
    using openscenario_interpreter::ParameterValueDistributionDefinition;

    validate(path);

    if (OpenScenario script{path}; script.category.is<ParameterValueDistributionDefinition>()) {
      auto & parameter_value_distribution = script.category.as<ParameterValueDistribution>();
      scenario_file_path = parameter_value_distribution.scenario_file.filepath;

      if (not check_scenario_path || boost::filesystem::exists(scenario_file_path)) {
        if (check_scenario_path) {
          validate(scenario_file_path);
        }

        scenario_file_doc.load_file(scenario_file_path.c_str());

        return parameter_value_distribution.derive();
      } else {
        std::stringstream what;
        what << "Scenario file " << std::quoted(scenario_file_path.string())
             << " described in ParameterDistributionDefinition file " << std::quoted(path.string())
             << " does not exist";
        throw std::runtime_error(what.str());
      }
    } else {
      return {};
    }
  }

  auto get_doc() -> pugi::xml_document & { return scenario_file_doc; }

  auto get_scenario_path() -> boost::filesystem::path { return scenario_file_path; }

private:
  pugi::xml_document scenario_file_doc;

  openscenario_validator::OpenSCENARIOValidator validate;

  boost::filesystem::path scenario_file_path;
};
}  // namespace openscenario_preprocessor

#endif  //OPENSCENARIO_PREPROCESSOR__DERIVER_HPP_
