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

#include <yaml-cpp/yaml.h>

#include <boost/range/adaptor/indexed.hpp>
#include <fstream>
#include <iostream>
#include <openscenario_preprocessor/deriver.hpp>
#include <openscenario_preprocessor/t4v2.hpp>
#include <openscenario_validator/validator.hpp>
#include <pugixml.hpp>
#include <regex>

namespace openscenario_preprocessor
{

// cspell: ignore isinstance etree

const std::string_view scenario_modifiers_distribution_base = R"###(
<OpenSCENARIO>
    <FileHeader author="" date="2022-08-26T07:17:21.031Z" description="" revMajor="0" revMinor="0"/>
    <ParameterValueDistribution>
        <ScenarioFile filepath=""/>
        <Deterministic>
        </Deterministic>
    </ParameterValueDistribution>
</OpenSCENARIO>)###";

const std::string_view load_yaml_to_xosc_with_encode_python_script = R"###(
import xmlschema
import yaml
import os
import sys
import xml.etree.ElementTree as ET
import cProfile
from yaml import CSafeLoader


def from_yaml(keyword, node):

  if isinstance(node, dict):
    #
    # ???: { ... }
    #
    result = {}

    for tag, value in node.items():

      if isinstance(value, list) and len(value) == 0:
        #
        # Tag: []
        #
        # => REMOVE
        #
        continue

      if str.islower(tag[0]):
        #
        # tag: { ... }
        #
        # => @tag: { ... }
        #
        result["@" + tag] = str(value)
      else:
        #
        # Tag: { ... }
        #
        # => NO CHANGES
        #
        result[tag] = from_yaml(tag, value)

    return result

  elif isinstance(node, list):
    #
    # ???: [ ... ]
    #
    return [from_yaml(keyword, item) for item in node]

  elif isinstance(node, str):
    return node

  else:
    return None


if __name__ == "__main__":
  # parse arguments here (input yaml path, output xosc path)
  input_yaml_path = sys.argv[1]
  output_xosc_path = sys.argv[2]

  xsd = open("/tmp/openscenario_preprocessor/schema.xsd")
  schema = xmlschema.XMLSchema(xsd)

  if os.path.exists(input_yaml_path):
    with open(input_yaml_path, "r") as file:
      openscenario_yaml = from_yaml("OpenSCENARIO", yaml.load(file, Loader=CSafeLoader))
      openscenario_yaml.pop("ScenarioModifiers", None)
      xosc, errors = schema.encode(
        openscenario_yaml,
        indent=2,
        preserve_root=True,
        unordered=True,  # Reorder elements
        # The "strict" mode is too strict than we would like.
        validation="lax",
      )
      xosc = xmlschema.XMLResource(xosc).tostring().replace("True", "true").replace("False", "false")
      with open(output_xosc_path, "w") as file:
        file.write(xosc)
)###";

auto T4V2::deriveToXoscStringScenarios(
  boost::filesystem::path scenario_path, boost::filesystem::path modifiers_path)
  -> std::vector<std::string>
{
  auto openscenario_doc = loadScenarioFile(scenario_path.string());

  std::vector<std::string> derived_scenarios;
  if (not modifiers_path.empty()) {
    std::fstream file{modifiers_path.string()};
    std::stringstream modifiers_ss;
    modifiers_ss << file.rdbuf();
    file.close();
    auto parameter_value_distribution =
      generateParameterValueDistributionFromScenarioModifiers(modifiers_ss.str());
    boost::filesystem::path parameter_value_distribution_path =
      "/tmp/openscenario_preprocessor/scenario_modifiers_distribution.xosc";
    parameter_value_distribution.save_file(parameter_value_distribution_path.string().c_str());

    Deriver derive("/tmp/openscenario_preprocessor/schema.xsd");
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

std::pair<boost::filesystem::path, boost::filesystem::path> T4V2::splitScenarioModifiers(
  boost::filesystem::path scenario_path)
{
  std::fstream file{scenario_path.string()};
  std::stringstream scenario_ss;
  scenario_ss << file.rdbuf();
  file.close();

  auto scenario_string = scenario_ss.str();

  ;
  if (auto modifiers_pos = scenario_string.find("ScenarioModifiers");
      modifiers_pos != std::string::npos) {
    auto openscenario_pos = scenario_string.find("OpenSCENARIO");

    if (openscenario_pos == std::string::npos) {
      throw std::runtime_error(
        "No OpenSCENARIO element found in TIER IV 2.0 Format Scenario. Please check your "
        "scenario.");
    }

    std::ofstream modifiers_ofs("/tmp/openscenario_preprocessor/t4v2_modifiers.yaml");
    std::ofstream base_scenario_ofs("/tmp/openscenario_preprocessor/t4v2_openscenario.yaml");

    if (openscenario_pos > modifiers_pos) {
      std::regex re("(.*\n|^)(OpenSCENARIO.*)");
      std::smatch match;
      std::regex_search(scenario_string, match, re);
      modifiers_ofs << scenario_string.substr(0, match.position(2));
      base_scenario_ofs << scenario_string.substr(match.position(2));
    } else {
      std::regex re("(.*\n|^)(ScenarioModifiers.*)");
      std::smatch match;
      std::regex_search(scenario_string, match, re);
      base_scenario_ofs << scenario_string.substr(0, match.position(2));
      modifiers_ofs << scenario_string.substr(match.position(2));
    }
    modifiers_ofs.close();
    base_scenario_ofs.close();
    return std::make_pair<boost::filesystem::path, boost::filesystem::path>(
      "/tmp/openscenario_preprocessor/t4v2_modifiers.yaml",
      "/tmp/openscenario_preprocessor/t4v2_openscenario.yaml");
  } else {
    std::ofstream base_scenario_ofs("/tmp/openscenario_preprocessor/t4v2_openscenario.yaml");
    base_scenario_ofs << scenario_string;
    base_scenario_ofs.close();
    return std::make_pair<boost::filesystem::path, boost::filesystem::path>(
      "", "/tmp/openscenario_preprocessor/t4v2_openscenario.yaml");
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
    openscenario_interpreter::ParameterSetSharedPtr parameter_set;

    virtual bool for_each(pugi::xml_node & node)
    {
      for (pugi::xml_attribute_iterator attribute_iter = node.attributes_begin();
           attribute_iter != node.attributes_end(); ++attribute_iter) {
        for (const auto & parameter : *parameter_set) {
          attribute_iter->set_value(std::regex_replace(
                                      attribute_iter->as_string(), std::regex(parameter.first),
                                      parameter.second.as<openscenario_interpreter::String>())
                                      .c_str());
        }
      }
      return true;
    }
  };

  replace_walker walker;

  for (const auto & parameter_set : scenario_modifier_distribution | boost::adaptors::indexed()) {
    pugi::xml_document derived_script;

    derived_script.reset(base_scenario_doc);  // deep copy

    // overwrite with scenario modifiers
    walker.parameter_set = parameter_set.value();
    derived_script.traverse(walker);

    derived_scripts.push_back(std::forward<pugi::xml_document>(derived_script));
  }

  return derived_scripts;
}

template <class XMLClass>
void convertYAMLtoXML(const YAML::Node & yaml, XMLClass & xml)
{
  switch (yaml.Type()) {
    case YAML::NodeType::Scalar:
      break;
    case YAML::NodeType::Sequence:
      break;
    case YAML::NodeType::Map:
      for (const auto & element : yaml) {
        auto key = element.first.as<std::string>();
        // skip empty sequence
        if (element.second.IsSequence() && element.second.size() == 0) {
          continue;
        }

        // add @ to tags that begin with lower charactor
        std::string processed_key = key;
        if (!key.empty() && std::islower(key[0])) {
          processed_key = "@" + key;
        }

        if (element.second.IsScalar()) {
          xml.append_attribute(key.c_str()).set_value(element.second.as<std::string>().c_str());
        } else if (element.second.IsSequence()) {
          for (const auto & sequence_element : element.second) {
            auto child = xml.append_child(key.c_str());
            convertYAMLtoXML(sequence_element, child);
          }
        } else {
          auto child = xml.append_child(key.c_str());
          convertYAMLtoXML(element.second, child);
        }
      }
      break;
    case YAML::NodeType::Null:
      break;
    default:
      throw std::runtime_error("Unknown YAML node type");
  }
}

auto convertYAMLtoXML(const boost::filesystem::path & path) -> pugi::xml_document
{
  auto yaml = YAML::LoadFile(path.string());
  pugi::xml_document xml;
  convertYAMLtoXML(yaml, xml);
  return xml;
}

auto T4V2::loadScenarioFile(boost::filesystem::path path) -> pugi::xml_document
{
  std::string script_path = "/tmp/openscenario_preprocessor/load_yaml_to_xosc_with_encode.py";
  std::ofstream ofs(script_path);
  ofs << load_yaml_to_xosc_with_encode_python_script;
  ofs.close();

  std::stringstream command_ss;
  command_ss << "python3 " << script_path << " " << path.string()
             << " /tmp/openscenario_preprocessor/normalized.xosc";

  if (system(command_ss.str().c_str()) != 0) {
    throw std::runtime_error("failed to execute python script : " + command_ss.str());
  }
  pugi::xml_document doc;
  doc.load_file("/tmp/openscenario_preprocessor/normalized.xosc");
  return doc;
}
}  // namespace openscenario_preprocessor
