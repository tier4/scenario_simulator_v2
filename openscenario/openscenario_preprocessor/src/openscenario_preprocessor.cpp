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
#include <boost/range/adaptor/indexed.hpp>
#include <openscenario_interpreter/syntax/open_scenario.hpp>
#include <openscenario_interpreter/syntax/parameter_value_distribution.hpp>
#include <openscenario_preprocessor/openscenario_preprocessor.hpp>

namespace openscenario_preprocessor
{
void Preprocessor::preprocessScenario(ScenarioSet & scenario)
{
  using openscenario_interpreter::OpenScenario;
  using openscenario_interpreter::ParameterValueDistribution;

  if (xml_validator.validate(scenario.path)) {
    if (auto script = std::make_shared<OpenScenario>(scenario.path);
        script->category.is_also<ParameterValueDistribution>()) {
      auto & parameter_value_distribution = script->category.as<ParameterValueDistribution>();
      auto base_scenario_path = parameter_value_distribution.scenario_file.filepath;

      if (boost::filesystem::exists(base_scenario_path)) {
        if (xml_validator.validate(base_scenario_path)) {
          auto base_scenario = std::make_shared<OpenScenario>(base_scenario_path);
          auto p = parameter_value_distribution.derive();

          for (const auto & parameter_list : p | boost::adaptors::indexed()) {
            pugi::xml_document derived_script;
            derived_script.reset(base_scenario->script);  // deep copy

            auto parameter_declarations =
              derived_script.document_element()
                .select_node(pugi::xpath_query{"/OpenSCENARIO/ParameterDeclarations"})
                .node();

            for (const auto & [name, parameter] : *parameter_list.value()) {
              if (auto parameter_node =
                    parameter_declarations.find_child_by_attribute("name", name.c_str());
                  parameter_node) {
                parameter_node.attribute("value").set_value(parameter.as<std::string>().c_str());
              } else {
                std::cout << "Parameter " << name << " not found in base scenario" << std::endl;
              }
            }

            ScenarioSet derived_scenario = scenario;
            derived_scenario.path += "." + std::to_string(parameter_list.index());

            derived_script.save_file(derived_scenario.path.c_str());

            preprocessed_scenarios.emplace_back(derived_scenario);
          }
        } else {
          throw common::Error("base scenario is not valid : " + base_scenario_path.string());
        }
      } else {
        throw common::Error("base scenario does not exist : " + base_scenario_path.string());
      }
    } else {
      // normal scenario
      preprocessed_scenarios.emplace_back(scenario);
    }
  } else {
    throw common::Error("the scenario file is not valid. Please check your scenario");
  }
}
}  // namespace openscenario_preprocessor
