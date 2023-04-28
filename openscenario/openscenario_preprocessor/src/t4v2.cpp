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

#include <boost/range/adaptor/indexed.hpp>
#include <iostream>
#include <openscenario_interpreter/syntax/open_scenario.hpp>
#include <openscenario_interpreter/syntax/parameter_value_distribution.hpp>
#include <openscenario_interpreter/syntax/parameter_value_distribution_definition.hpp>
#include <openscenario_preprocessor/deriver.hpp>
#include <openscenario_preprocessor/t4v2.hpp>
#include <openscenario_preprocessor/template_distributions.hpp>
#include <pugixml.hpp>
#include <regex>

namespace openscenario_preprocessor
{
const std::string_view scenario_modifiers_distribution_base = R"###(
<OpenSCENARIO>
    <FileHeader author="" date="2022-08-26T07:17:21.031Z" description="" revMajor="0" revMinor="0"/>
    <ParameterValueDistribution>
        <ScenarioFile filepath=""/>
        <Deterministic>
        </Deterministic>
    </ParameterValueDistribution>
</OpenSCENARIO>)###";

auto T4V2::deriveToXoscStringScenarios(boost::filesystem::path path) -> std::vector<std::string>
{
  std::fstream file{path};
  std::stringstream scenario;
  scenario << file.rdbuf();
  file.close();

  auto split = splitScenarioModifiers(scenario.str());

  std::ofstream base_scenario_ofs("/tmp/openscenario_preprocessor/t4v2_openscenario.yaml");
  base_scenario_ofs << split.second;
  base_scenario_ofs.close();

  auto openscenario_json =
    tojson::loadyaml("/tmp/openscenario_preprocessor/t4v2_openscenario.yaml");
  auto openscenario_xml = tojson::emitters::toxml(openscenario_json);
  pugi::xml_document openscenario_doc;
  openscenario_doc.load_string(openscenario_xml.c_str());

  std::vector<std::string> derived_scenarios;
  if (not split.first.empty()) {
    auto parameter_value_distribution =
      generateParameterValueDistributionFromScenarioModifiers(split.first);
    boost::filesystem::path parameter_value_distribution_path =
      "/tmp/openscenario_preprocessor/scenario_modifiers_distribution.xosc";
    parameter_value_distribution.save_file(parameter_value_distribution_path.string().c_str());

    Deriver derive;
    auto distribution = derive(parameter_value_distribution_path, false);

    if (not distribution.empty()) {
      auto derived_scenario_docs =
        deriveScenarioWithScenarioModifiers(openscenario_doc, distribution);
      for (auto & derived_scenario_doc : derived_scenario_docs) {
        std::stringstream derived_scenario_stream;
        derived_scenario_doc.save(derived_scenario_stream);
        derived_scenarios.push_back(derived_scenario_stream.str());
      }
    } else {
      // no scenario modifiers
      std::stringstream scenario_stream;
      openscenario_doc.save(scenario_stream);
      derived_scenarios.push_back(scenario_stream.str());
    }
  } else {
    // no scenario modifiers
    std::stringstream scenario_stream;
    openscenario_doc.save(scenario_stream);
    derived_scenarios.push_back(scenario_stream.str());
  }
  return derived_scenarios;
}

std::pair<std::string, std::string> T4V2::splitScenarioModifiers(std::string scenario)
{
  std::regex re("(.*\n|^)(OpenSCENARIO.*)");
  std::smatch match;

  if (std::regex_search(scenario, match, re)) {
    return std::make_pair(
      scenario.substr(0, match.position(2)), scenario.substr(match.position(2)));
  } else {
    throw std::runtime_error(
      "No OpenSCENARIO element found in TIER IV 2.0 Format Scenario. Please check your "
      "scenario.");
  }
}

auto T4V2::generateParameterValueDistributionFromScenarioModifiers(
  std::string scenario_modifiers_str) -> pugi::xml_document
{
  pugi::xml_document doc;
  doc.load_string(scenario_modifiers_distribution_base.data());
  auto deterministic =
    doc.select_node(pugi::xpath_query{"OpenSCENARIO/ParameterValueDistribution/Deterministic"})
      .node();

  auto scenario_modifiers =
    YAML::Load(scenario_modifiers_str)["ScenarioModifiers"]["ScenarioModifier"];
  for (const auto & scenario_modifier : scenario_modifiers) {
    auto distribution = deterministic.append_child("DeterministicSingleParameterDistribution");

    struct
    {
      bool list_enabled = false;
      std::string name;
      double start;
      double step;
      double stop;
      std::vector<std::string> list;
    } data;

    for (auto modifier_element : scenario_modifier) {
      auto key = modifier_element.first.as<std::string>();

      if (key == "name") {
        data.name = modifier_element.second.as<std::string>();
      } else if (key == "list") {
        data.list_enabled = true;
        for (const auto & value : modifier_element.second.as<std::vector<std::string>>()) {
          data.list.push_back(value);
        }
      } else if (key == "start") {
        data.start = modifier_element.second.as<double>();
      } else if (key == "step") {
        data.step = modifier_element.second.as<double>();
      } else if (key == "stop") {
        data.stop = modifier_element.second.as<double>();
      } else {
        std::cout << "unknown key: " << key << std::endl;
      }
    }

    distribution.append_attribute("parameterName") = data.name.c_str();

    if (data.list_enabled) {
      // add distribution set
      auto distribution_set = distribution.append_child("DistributionSet");
      for (const auto & value : data.list) {
        auto distribution_set_element = distribution_set.append_child("Element");
        distribution_set_element.append_attribute("value") = value.c_str();
      }
    } else {
      // add distribution range
      auto range = distribution.append_child("DistributionRange");
      range.append_attribute("stepWidth") = data.step;
      auto range_range = range.append_child("Range");
      range_range.append_attribute("upperLimit") = data.stop;
      range_range.append_attribute("lowerLimit") = data.start;
    }
  }
  return doc;
}

auto T4V2::deriveScenarioWithScenarioModifiers(
  const pugi::xml_document & base_scenario_doc,
  const openscenario_interpreter::ParameterDistribution & scenario_modifier_distribution)
  -> std::vector<pugi::xml_document>
{
  std::vector<pugi::xml_document> derived_scripts;

  struct replace_walker : pugi::xml_tree_walker
  {
    openscenario_interpreter::ParameterListSharedPtr parameter_list;

    virtual bool for_each(pugi::xml_node & node)
    {
      for (pugi::xml_attribute_iterator attribute_iter = node.attributes_begin();
           attribute_iter != node.attributes_end(); ++attribute_iter) {
        for (const auto & parameter : *parameter_list) {
          attribute_iter->set_value(regex_replace(
                                      attribute_iter->as_string(), std::regex(parameter.first),
                                      parameter.second.as<openscenario_interpreter::String>())
                                      .c_str());
        }
      }
      return true;
    }
  };

  replace_walker walker;

  for (const auto & parameter_list : scenario_modifier_distribution | boost::adaptors::indexed()) {
    pugi::xml_document derived_script;

    derived_script.reset(base_scenario_doc);  // deep copy

    // overwrite with scenario modifiers
    walker.parameter_list = parameter_list.value();
    derived_script.traverse(walker);

    derived_scripts.push_back(std::forward<pugi::xml_document>(derived_script));
  }

  return derived_scripts;
}
}  // namespace openscenario_preprocessor
