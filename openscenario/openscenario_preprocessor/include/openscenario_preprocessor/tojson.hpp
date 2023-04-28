//  MIT License
//
//  Copyright (c) 2021 Mirco
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy
//  of this software and associated documentation files (the "Software"), to deal
//  in the Software without restriction, including without limitation the rights
//  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
//                                                          copies of the Software, and to permit persons to whom the Software is
//  furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all
//  copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
//  SOFTWARE.

#ifndef OPENSCENARIO_PREPROCESSOR_TOJSON_HPP
#define OPENSCENARIO_PREPROCESSOR_TOJSON_HPP

#include <yaml-cpp/yaml.h>

#include <boost/regex.hpp>
#include <fstream>
#include <nlohmann/json.hpp>
#include <rapidxml/rapidxml.hpp>

/* Adding declarations to make it compatible with gcc 4.7 and greater */
#if __GNUC__ * 10000 + __GNUC_MINOR__ * 100 + __GNUC_PATCHLEVEL__ > 40700
namespace rapidxml
{
namespace internal
{
template <class OutIt, class Ch>
inline OutIt print_children(OutIt out, const xml_node<Ch> * node, int flags, int indent);
template <class OutIt, class Ch>
inline OutIt print_attributes(OutIt out, const xml_node<Ch> * node, int flags);
template <class OutIt, class Ch>
inline OutIt print_data_node(OutIt out, const xml_node<Ch> * node, int flags, int indent);
template <class OutIt, class Ch>
inline OutIt print_cdata_node(OutIt out, const xml_node<Ch> * node, int flags, int indent);
template <class OutIt, class Ch>
inline OutIt print_element_node(OutIt out, const xml_node<Ch> * node, int flags, int indent);
template <class OutIt, class Ch>
inline OutIt print_declaration_node(OutIt out, const xml_node<Ch> * node, int flags, int indent);
template <class OutIt, class Ch>
inline OutIt print_comment_node(OutIt out, const xml_node<Ch> * node, int flags, int indent);
template <class OutIt, class Ch>
inline OutIt print_doctype_node(OutIt out, const xml_node<Ch> * node, int flags, int indent);
template <class OutIt, class Ch>
inline OutIt print_pi_node(OutIt out, const xml_node<Ch> * node, int flags, int indent);
}  // namespace internal
}  // namespace rapidxml
#include <rapidxml/rapidxml_print.hpp>
#endif

#if __has_cpp_attribute(nodiscard)
#define TOJSON_NODISCARD [[nodiscard]]
#else
#define TOJSON_NODISCARD
#endif

#include <nlohmann/ordered_map.hpp>
using json = nlohmann::ordered_json;

namespace tojson
{
namespace detail
{

/// \todo refactor and pass nlohmann::json down by reference instead of returning it
inline json xml2json(const rapidxml::xml_node<> * root)
{
  json j{};
  std::map<std::string, int> occurrence{};

  for (auto * node = root->first_node(); node; node = node->next_sibling()) {
    auto key = node->name();
    occurrence[key] += 1;

    // if seen current node, convert it into a list
    if (occurrence[key] == 2) j[key] = {j[key]};

    // if the node is a sequence
    if (occurrence[key] > 1) {
      json n{};
      if (node->first_node())
        n[key] = xml2json(node);
      else
        n["@text"] = node->value();
      // toyaml through the attributes
      for (auto * attr = node->first_attribute(); attr; attr = attr->next_attribute())
        n[key][attr->name()] = attr->value();
      j[key].emplace_back(n[key]);
    } else {
      if (node->first_node())
        j[key] = xml2json(node);
      else
        j["@text"] = node->value();
      // toyaml through the attributes
      for (auto * attr = node->first_attribute(); attr; attr = attr->next_attribute())
        j[key][attr->name()] = attr->value();
    }
  }

  return j;
}

/// \todo refactor and pass nlohmann::json down by reference instead of returning it
inline json pugixml2json(const pugi::xml_node & root)
{
  json j{};
  std::map<std::string, int> occurrence{};

  for (auto & child : root.children()) {
    auto key = child.name();
    occurrence[key] += 1;

    // if seen current node, convert it into a list
    if (occurrence[key] == 2) j[key] = json::array({j[key]});

    // if the node is a sequence
    if (occurrence[key] > 1) {
      json n{};
      if (child == child.parent().first_child())
        n[key].emplace_back(pugixml2json(child));
      else
//        n[key].emplace_back(child.value());

        n["@text"] = child.value();
      // toyaml through the attributes
      for (auto & attr : child.attributes()) {
        n[key][attr.name()] = attr.value();
      }

      j[key].emplace_back(n[key]);
    } else {
      if (child == child.parent().first_child())
        j[key] = pugixml2json(child);
      else
//        j[key].emplace_back(child.value());
        j["@text"] = child.value();
      // toyaml through the attributes
      for (auto & attr : child.attributes()) {
        j[key][attr.name()] = attr.value();
      }
    }
  }

  return j;
}

inline json parse_scalar(const YAML::Node & node)
{
  int i;
  double d;
  bool b;
  std::string s;

  if (YAML::convert<int>::decode(node, i)) return i;
  if (YAML::convert<double>::decode(node, d)) return d;
  if (YAML::convert<bool>::decode(node, b)) return b;
  if (YAML::convert<std::string>::decode(node, s)) return s;

  return nullptr;
}

/// \todo refactor and pass nlohmann::json down by reference instead of returning it
inline json yaml2json(const YAML::Node & root)
{
  json j{};

  switch (root.Type()) {
    case YAML::NodeType::Null:
      break;
    case YAML::NodeType::Scalar:
      return parse_scalar(root);
    case YAML::NodeType::Sequence:
      for (auto && node : root) j.emplace_back(yaml2json(node));
      break;
    case YAML::NodeType::Map:
      for (auto && it : root) j[it.first.as<std::string>()] = yaml2json(it.second);
      break;
    default:
      break;
  }
  return j;
}

/// \todo handle @text entries better
inline void toyaml(const json & j, YAML::Emitter & e)
{
  for (auto it = j.begin(); it != j.end(); ++it) {
    if (it->is_object()) {
      e << YAML::Key << it.key() << YAML::Value << YAML::BeginMap;
      toyaml(*it, e);
      e << YAML::EndMap;
    } else if (it->is_array()) {
      e << YAML::Key << it.key() << YAML::Value << YAML::BeginSeq;
      toyaml(it.value(), e);
      e << YAML::EndSeq;
    } else {
      if (it.key() == "@text") {
        e << YAML::Value << it.value().get<std::string>();
      } else {
        e << YAML::Key << it.key() << YAML::Value << it.value().get<std::string>();
      }
    }
  }
}

// Forward declaration required here for circular dipedency.
inline void toxml(const json & j, rapidxml::xml_document<> & doc, rapidxml::xml_node<> * parent);

inline std::string repr(const json & j)
{
  if (j.is_number()) return std::to_string(j.get<int>());
  if (j.is_boolean()) return j.get<bool>() ? "true" : "false";
  if (j.is_number_float()) return std::to_string(j.get<double>());
  if (j.is_string()) return j.get<std::string>();
  std::runtime_error("invalid type");
  return "";
}

/// \todo handle @text entries better
inline void toxml(
  const json & j, rapidxml::xml_document<> & doc, rapidxml::xml_node<> * parent,
  const std::string & key)
{
  //  std::cout << "toxml for array : " << key << std::endl;
  // Not the prettiest of designs, but it works fine.
  for (auto it = j.begin(); it != j.end(); ++it) {
    if (it->is_object()) {
      //      std::cout << "object in array : " << *it << std::endl;
      //      std::cout <<  it.key() << std::endl;
      //      auto * node = doc.allocate_node(rapidxml::node_element, doc.allocate_string(it.key().data()));
      auto * node = doc.allocate_node(rapidxml::node_element, doc.allocate_string(key.data()));
      //      auto * node = doc.allocate_node(rapidxml::node_element);
      detail::toxml(*it, doc, node);
      parent->append_node(node);
    } else if (it->is_array()) {
      //      std::cout << "array in array" << std::endl;
      detail::toxml(*it, doc, parent, key);
    } else {
      //      std::cout << "else in array : " << std::endl;
      //      std::cout <<  it.key() << std::endl;
      auto * node = doc.allocate_node(rapidxml::node_element, doc.allocate_string(key.data()));
      node->value(doc.allocate_string(repr(it.value()).data()));
      parent->append_node(node);
    }
  }
}

/// \todo handle @text entries better
inline void toxml(const json & j, rapidxml::xml_document<> & doc, rapidxml::xml_node<> * parent)
{
  //  std::cout << "detail::toxml 1" << std::endl;
  for (auto it = j.begin(); it != j.end(); ++it) {
    if (it->is_object()) {
      auto * node = doc.allocate_node(rapidxml::node_element, doc.allocate_string(it.key().data()));
      detail::toxml(*it, doc, node);
      parent->append_node(node);
    } else if (it->is_array()) {
      detail::toxml(*it, doc, parent, it.key());
    } else {
      //      std::cout << "detail::toxml else : "<< it.key().data() << std::endl;
      if (it->is_null()) {
        detail::toxml(*it, doc, parent, it.key());

      } else {
        auto * node = doc.allocate_attribute(doc.allocate_string(it.key().data()));
        node->value(doc.allocate_string(repr(it.value()).data()));
        parent->append_attribute(node);
      }
    }
  }
}

}  // namespace detail

/// \brief Convert XML string to JSON.
TOJSON_NODISCARD inline json xml2json(const std::string & str)
{
  std::string replacement = "\\/";
  boost::regex re("(?<!<)\\/(?!>)");
  std::string escaped_str = boost::regex_replace(str, re, replacement);

  json j{};
  rapidxml::xml_document<> doc{};
  doc.parse<0>(const_cast<char *>(escaped_str.data()));

  auto * root = doc.first_node();
  if (root) j[root->name()] = detail::xml2json(root);

  return j;
}

TOJSON_NODISCARD inline json pugixml2json(pugi::xml_node & root)
{
  json j{};
  j[root.name()] = detail::pugixml2json(root);

  return j;
}

/// \brief Convert YAML string to JSON.
TOJSON_NODISCARD inline json yaml2json(const std::string & str)
{
  YAML::Node root = YAML::Load(str);
  return detail::yaml2json(root);
}

/// \brief Load a YAML file to JSON.
TOJSON_NODISCARD inline json loadyaml(const std::string & filepath)
{
  YAML::Node root = YAML::LoadFile(filepath);
  return detail::yaml2json(root);
}

/// \brief Load XML file to JSON.
TOJSON_NODISCARD inline json loadxml(const std::string & filepath)
{
  std::ifstream file{filepath.data()};
  std::string str{std::istream_iterator<char>(file), std::istream_iterator<char>()};
  //  std::cout << "loaded : " <<  str << std::endl;
  return xml2json(str);
}

namespace emitters
{

/// \brief Generate string representation of json as an YAML document.
TOJSON_NODISCARD inline std::string toyaml(const json & j)
{
  YAML::Emitter e;
  e << YAML::BeginDoc;
  if (j.is_object()) {
    e << YAML::BeginMap;
    detail::toyaml(j, e);
    e << YAML::EndMap;
  } else if (j.is_array()) {
    e << YAML::BeginSeq;
    detail::toyaml(j, e);
    e << YAML::EndSeq;
  }
  e << YAML::EndDoc;
  return e.c_str();
}

/// \brief Generate string representation of json as an XML document.
/// \param j Json object to convert, must have a single root
/// \throws std::runtime_error if the object have more than one root
TOJSON_NODISCARD inline std::string toxml(const json & j)
{
  rapidxml::xml_document<> doc;
  auto * decl = doc.allocate_node(rapidxml::node_declaration);
  decl->append_attribute(doc.allocate_attribute("version", "1.0"));
  decl->append_attribute(doc.allocate_attribute("encoding", "utf-8"));
  doc.append_node(decl);

  if (j.is_object() && j.size() == 1) {
    auto * root =
      doc.allocate_node(rapidxml::node_element, doc.allocate_string(j.begin().key().data()));
    detail::toxml(j.begin().value(), doc, root);
    doc.append_node(root);
  } else {
    throw std::runtime_error("json must have a single root node");
  }

  std::string xml_as_string;
  rapidxml::print(std::back_inserter(xml_as_string), doc);
  return xml_as_string;
}

}  // namespace emitters
}  // namespace tojson

#endif  //OPENSCENARIO_PREPROCESSOR_TOJSON_HPP
