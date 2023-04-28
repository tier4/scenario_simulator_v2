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
#include <openscenario_preprocessor/tojson.hpp>
#include <regex>

namespace openscenario_preprocessor
{
void Preprocessor::preprocessScenario(
  const boost::filesystem::path & scenario_path, ScenarioFormat output_format)
{
  using openscenario_interpreter::OpenScenario;
  using openscenario_interpreter::ParameterValueDistribution;
  using openscenario_interpreter::ParameterValueDistributionDefinition;

  auto distribution = derive(scenario_path);
  if (distribution.empty()) {
    preprocessed_scenarios.push(scenario_path);  // normal scenario
  } else {
    auto base_scenario_path = derive.get_scenario_path();
    generateDerivedScenarioFromDistribution(distribution, base_scenario_path, output_format);
  }
}

void Preprocessor::generateDerivedScenarioFromDistribution(
  openscenario_interpreter::ParameterDistribution & distribution,
  const boost::filesystem::path & path, ScenarioFormat output_format)
{
//  std::cout << "generateDerivedScenarioFromDistribution" << std::endl;
  for (const auto & parameter_list : distribution | boost::adaptors::indexed()) {
    pugi::xml_document derived_script;

    derived_script.reset(derive.get_doc());  // deep copy

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
                  << std::quoted(derive.get_scenario_path().string()) << ", so ignore it."
                  << std::endl;
      }
    }

    const auto derived_scenario_path_xosc =
      output_directory / (path.stem().string() + "." + std::to_string(parameter_list.index()) +
                          path.extension().string());

    derived_script.save_file(derived_scenario_path_xosc.c_str());

    try {
      auto derived_scenario_path = [&]() {
        if (output_format == ScenarioFormat::t4v2) {
          std::cout << "convert to json : " << derived_scenario_path_xosc.c_str() << std::endl;

          pugi::xml_document derived_scenario_xml;
          derived_scenario_xml.load_file(derived_scenario_path_xosc.c_str());
          auto root_xml = derived_scenario_xml.root();

          auto derived_scenario_json = tojson::pugixml2json(root_xml);
//          derived_scenario_xml.print(std::cout);
          std::cout << derived_scenario_json << std::endl;
          std::cout << "finish convert to json" << std::endl;
          const auto derived_scenario_path_t4v2 =
            output_directory /
            (path.stem().string() + "." + std::to_string(parameter_list.index()) + ".yaml");
//          std::cout << "save as t4v2 scenario" << std::endl;

          std::ofstream derived_scenario_yaml{derived_scenario_path_t4v2};
          derived_scenario_yaml << tojson::emitters::toyaml(derived_scenario_json);
//          std::cout << "Generated " << tojson::emitters::toyaml(derived_scenario_json) << std::endl;
          derived_scenario_yaml.close();
          return derived_scenario_path_t4v2;
        } else {
          return derived_scenario_path_xosc;
        }
      }();
      preprocessed_scenarios.emplace(derived_scenario_path);
    } catch (rapidxml::parse_error & ex) {
      std::cerr << "[Error] something went wrong during deriving scenario : " << ex.what() << ", "
                << ex.where<char>() << std::endl;
      //                return 1;
    }
  }
}

std::istream & operator>>(std::istream & is, ScenarioFormat & format)
{
  std::string token;
  is >> token;
  if (token == "t4v2") {
    format = ScenarioFormat::t4v2;
  } else if (token == "xosc") {
    format = ScenarioFormat::xosc;
  } else {
    std::stringstream what;
    what << token << " is invalid scenario format. Please specify t4v2 or xosc as scenario format";
    throw std::runtime_error(what.str());
  }
  return is;
}
}  // namespace openscenario_preprocessor
