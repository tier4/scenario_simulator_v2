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

    auto derived_scenario_path = [&]() {
      if (output_format == ScenarioFormat::t4v2) {
        YAML::Emitter yaml_emitter;
        convertXMLtoYAML(derived_script, yaml_emitter);

        const auto derived_scenario_path_t4v2 =
          output_directory /
          (path.stem().string() + "." + std::to_string(parameter_list.index()) + ".yaml");

        std::ofstream derived_scenario_yaml{derived_scenario_path_t4v2};
        derived_scenario_yaml << yaml_emitter.c_str();
        derived_scenario_yaml.close();

        return derived_scenario_path_t4v2;
      } else {
        derived_script.save_file(derived_scenario_path_xosc.c_str());
        return derived_scenario_path_xosc;
      }
    }();
    preprocessed_scenarios.emplace(derived_scenario_path);
  }
}

void Preprocessor::convertXMLtoYAML(const pugi::xml_node & xml, YAML::Emitter & emitter)
{
  if (xml.attributes().empty() && xml.children().empty()) {
    emitter << "";
    return;
  }

  // the map of node name and {total count, used count}
  std::map<std::string, std::pair<int, int>> count;
  for (const auto & child : xml.children()) {
    if (count.find(child.name()) == count.end()) {
      count[child.name()] = std::make_pair(1, 0);
    } else {
      count[child.name()].first++;
    }
  }

  // iterate attributes
  if (not xml.attributes().empty()) {
    emitter << YAML::BeginMap;

    for (const auto & attr : xml.attributes()) {
      emitter << YAML::Key << attr.name();
      emitter << YAML::Value << YAML::SingleQuoted << attr.as_string();
    }
    if (xml.children().empty()) {
      emitter << YAML::EndMap;
    }
  }

  // iterate child elements
  if (not xml.children().empty()) {
    if (xml.attributes().empty()) {
      emitter << YAML::BeginMap;
    }
    for (const auto & child : xml.children()) {
      // total count == 1, make single map in yaml
      if (count[child.name()].first == 1) {
        emitter << YAML::Key << child.name();
        emitter << YAML::Value;
        convertXMLtoYAML(child, emitter);
      } else {
        // total count > 1, make sequence in yaml
        if (count[child.name()].second == 0) {
          // first node
          emitter << YAML::Key << child.name();
          emitter << YAML::Value;
          emitter << YAML::BeginSeq;
          convertXMLtoYAML(child, emitter);
          // count up used count
          count[child.name()].second++;
        } else if (count[child.name()].second == count[child.name()].first - 1) {
          // last node
          convertXMLtoYAML(child, emitter);
          emitter << YAML::EndSeq;
        } else {
          // middle node
          convertXMLtoYAML(child, emitter);
          count[child.name()].second++;
        }
      }
    }
    emitter << YAML::EndMap;
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
