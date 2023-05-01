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

#ifndef OPENSCENARIO_PREPROCESSOR__YAML_XML_HPP_
#define OPENSCENARIO_PREPROCESSOR__YAML_XML_HPP_

#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <fstream>
#include <pugixml.hpp>

namespace YAML
{
template <>
struct convert<pugi::xml_node>
{
  static Node encode(const pugi::xml_node & xml)
  {
    Node node;
    return node;
  }
  //
  //  static bool decode(const Node & yaml, const Node & parent_yaml, pugi::xml_node & xml)
  //  {
  //    switch (yaml.Type()) {
  //      case NodeType::Null:
  //        break;
  //      case NodeType::Scalar:
  //
  //        // TODO: put in attribute
  //        //        xml_node.append_attribute(pugi::node_)
  //        break;
  //      case NodeType::Sequence:
  //        for (const auto & node : yaml) {
  //          xml.append_child(parent_yaml.);
  //          convertYAMLtoXML(node, yaml, xml_node.append_child(pugi::node_element, "TODO"));
  //        }
  //        break;
  //      case NodeType::Map:
  //        for (YAML::const_iterator iter = yaml.begin(); iter != yaml.end(); ++iter) {
  //          auto value = iter->second;
  //          if (value.IsScalar()) {
  //            xml.append_attribute(iter->first.as<std::string>().c_str()) =
  //              value.as<std::string>().c_str();
  //          } else {
  //            decode(yaml, xml);
  //          }
  //        }
  //      case NodeType::Undefined:
  //        break;
  //    }
  //    rhs.x = node[0].as<double>();
  //    rhs.y = node[1].as<double>();
  //    rhs.z = node[2].as<double>();
  //    return true;
  //  }
};
}  // namespace YAML

namespace openscenario_preprocessor
{
namespace yaml_xml
{

class YamlXml
{
public:
  YamlXml() = default;

  ~YamlXml() = default;

  void loadYAMLfile(const std::filesystem::path & file_path)
  {
    std::ifstream ifs(file_path);
    *yaml_node_ = YAML::Load(ifs);
    xml_document_ = std::nullopt;
  }

  void loadXMLfile(const std::filesystem::path & file_path)
  {
    *xml_document_ = pugi::xml_document();
    xml_document_->load_file(file_path.c_str());
    yaml_node_ = std::nullopt;
  }

  void loadYAMLString(const std::string & yaml_string)
  {
    *yaml_node_ = YAML::Load(yaml_string);
    xml_document_ = std::nullopt;
  }

  void loadXMLString(const std::string & xml_string)
  {
    *xml_document_ = pugi::xml_document();
    xml_document_->load_string(xml_string.c_str());
    yaml_node_ = std::nullopt;
  }

  void convertYAMLtoXML()
  {
    if (yaml_node_) {
      *xml_document_ = pugi::xml_document();
      *xml_document_ = YAML::convert<YAML::Node>::encode(*yaml_node_);
    }
  }

  void convertXMLtoYAML()
  {
    if (xml_document_) {
      *yaml_node_ = YAML::Load(pugi::as_utf8(xml_document_->child_value()));
    }
  }

  void saveYAMLFile(const std::filesystem::path & file_path)
  {
    if (not yaml_node_) {
      convertXMLtoYAML();
    }
    std::ofstream ofs(file_path);
    ofs << *yaml_node_;
    ofs.close();
  }

  void saveXMLFile(const std::filesystem::path & file_path)
  {
    if (not xml_document_) {
      convertYAMLtoXML();
    }
    xml_document_->save_file(file_path.c_str());
  }

private:
  void convertXMLtoYAMLImpl(const pugi::xml_node & xml, YAML::Node & yaml)
  {
    std::map<std::string, int> count;
    for (const auto & child : xml.children()) {
      if (not count[child.name()]) {
        count[child.name()] = 1;
      } else {
        count[child.name()]++;
      }
    }

    YAML::Emitter out;
    for (const auto & child : xml.children()) {
      if (count[child.name()] > 1) {
        if (yaml[child.name()].IsNull()) {
          // first node
          out << YAML::BeginMap;
          out << YAML::Key << child.name();
          out << YAML::Value << YAML::BeginSeq;
          
        }else{

        }
      }
    }
  } &&
        std::find(yaml.begin(), yaml.end(), [&](YAML::const_iterator & iter) -> bool {
    return iter->first.as<std::string>() == child.name();
        }) == yaml.end())
  {
    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << child.name();
    out << YAML::Value << YAML::BeginSeq;
  }
  yaml[child.name()] = YAML::;
  convertXMLtoYAMLImpl(child, yaml[child.name()][count[child.name()] - 1]);
}

for (const auto & child : xml.children())
{
  if (child.type() == pugi::node_pcdata) {
    yaml = child.value();
  } else {
    yaml = YAML::Node();
    convertXMLtoYAMLImpl(child, yaml);
  }
}
}
void convertYAMLtoXMLImpl(const YAML::Node & yaml_node, pugi::xml_node & xml_node)
{
  //    using YAML::NodeType;
  //    switch (yaml_node.Type()) {
  //      case NodeType::Null:
  //        break;
  //      case NodeType::Scalar:
  //        // TODO: put in attribute
  //        //        xml_node.append_attribute(pugi::node_)
  //        break;
  //      case NodeType::Sequence:
  //        for (const auto & node : yaml_node) {
  //          convertYAMLtoXML(node, xml_node.append_child(pugi::node_element, "TODO"));
  //        }
  //        break;
  //      case NodeType::Map:
  //        yaml_node->fir break;
  //      case NodeType::Undefined:
  //        break;
  //    }
  //    if (yaml_node.IsScalar()) {
  //      xml_node.append_child(pugi::node_pcdata).set_value(yaml_node.as<std::string>().c_str());
  //    } else if (yaml_node.IsSequence()) {
  //      xml_node.append_child(yaml_node.)
  //
  //    } else if (yaml_node.IsMap()) {
  //      for (const auto & node : yaml_node) {
  //        convertYAMLtoXML(node.second, xml_node.append_child(node.first.as<std::string>().c_str()));
  //      }
  //    }
}
std::optional<YAML::Node> yaml_node_ = std::nullopt;

std::optional<pugi::xml_document> xml_document_ = std::nullopt;
};
}  // namespace yaml_xml
}  // namespace openscenario_preprocessor

#endif  // OPENSCENARIO_PREPROCESSOR__YAML_XML_HPP_
