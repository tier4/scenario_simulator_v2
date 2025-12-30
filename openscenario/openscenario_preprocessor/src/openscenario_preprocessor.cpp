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

#include <algorithm>
#include <boost/lexical_cast.hpp>
#include <boost/range/adaptor/indexed.hpp>
#include <openscenario_interpreter/syntax/open_scenario.hpp>
#include <openscenario_interpreter/syntax/parameter_value_distribution_definition.hpp>
#include <openscenario_preprocessor/openscenario_preprocessor.hpp>

namespace openscenario_preprocessor
{
void Preprocessor::preprocessScenario(const Scenario & scenario)
{
  using openscenario_interpreter::OpenScenario;
  using openscenario_interpreter::ParameterValueDistribution;
  using openscenario_interpreter::ParameterValueDistributionDefinition;

  validate(scenario.path);

  if (OpenScenario script{scenario.path};
      script.category.is<ParameterValueDistributionDefinition>()) {
    auto & parameter_value_distribution = script.category.as<ParameterValueDistribution>();
    auto scenario_file_path = parameter_value_distribution.scenario_file.filepath;

    if (boost::filesystem::exists(scenario_file_path)) {
      validate(scenario_file_path);

      OpenScenario scenario_file{scenario_file_path};

      for (const auto & parameter_list :
           parameter_value_distribution.derive() | boost::adaptors::indexed()) {
        pugi::xml_document derived_script;

        derived_script.reset(scenario_file.script);  // deep copy

        auto parameter_declarations =
          derived_script.document_element()
            .select_node(pugi::xpath_query{"/OpenSCENARIO/ParameterDeclarations"})
            .node();

        // embedding parameter values
        for (const auto & [name, value] : *parameter_list.value()) {
          if (
            auto parameter_node =
              parameter_declarations.find_child_by_attribute("name", name.c_str())) {
            parameter_node.attribute("value").set_value(
              boost::lexical_cast<std::string>(value).c_str());
          } else {
            std::cerr << "Parameter " << std::quoted(name) << " is not declared in scenario "
                      << std::quoted(scenario_file_path.string()) << ", so ignore it." << std::endl;
          }
        }

        const auto derived_scenario_path =
          output_directory /
          (scenario.path.stem().string() + "." + std::to_string(parameter_list.index()) +
           scenario.path.extension().string());

        derived_script.save_file(derived_scenario_path.c_str());

        preprocessed_scenarios.emplace(derived_scenario_path, scenario.frame_rate);
      }
    } else {
      std::stringstream what;
      what << "Scenario file " << std::quoted(scenario_file_path.string())
           << " described in ParameterDistributionDefinition file "
           << std::quoted(scenario.path.string()) << " does not exist";
      throw std::runtime_error(what.str());
    }
  } else {
    preprocessed_scenarios.push(scenario);  // normal scenario
  }
}
}  // namespace openscenario_preprocessor
