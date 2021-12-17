// Copyright 2015-2020 Tier IV, Inc. All rights reserved.
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

#ifndef OPENSCENARIO_INTERPRETER__READER__ATTRIBUTE_HPP_
#define OPENSCENARIO_INTERPRETER__READER__ATTRIBUTE_HPP_

#include "scenario_simulator_exception/exception.hpp"
#define OPENSCENARIO_INTERPRETER_ALLOW_ATTRIBUTES_TO_BE_BLANK

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <functional>
#include <openscenario_interpreter/reader/evaluate.hpp>
#include <openscenario_interpreter/syntax/parameter_type.hpp>
#include <openscenario_interpreter/utility/highlighter.hpp>
#include <pugixml.hpp>
#include <regex>
#include <string>
#include <unordered_map>

namespace openscenario_interpreter
{
inline namespace reader
{
/* ---- Dynamic Configuration --------------------------------------------------
 *
 *  See https://design.ros2.org/articles/roslaunch_xml.html#dynamic-configuration
 *
 * -------------------------------------------------------------------------- */
template <typename Scope>
auto substitute(std::string attribute, Scope & scope)
{
  static const std::regex substitution_syntax{R"((.*)\$\((([\w-]+)\s?([^\)]*))\)(.*))"};
  static const std::unordered_map<
    std::string, std::function<std::string(const std::string &, Scope &)> >
    substitutions{
      {"find-pkg-share",
       [](auto && package_name, auto &&) {
         return ament_index_cpp::get_package_share_directory(package_name);
       }},

      {"var",
       [](auto && name, auto && scope) -> String {
         const auto found = scope.findObject(name);
         if (found) {
           return boost::lexical_cast<String>(found);
         } else {
           return "";
         }
       }},
      {"dirname",
       [](auto &&, auto && scope) { return scope.global().pathname.parent_path().string(); }}};

  std::smatch match{};
  while (std::regex_match(attribute, match, substitution_syntax)) {
    const auto iter{substitutions.find(match.str(3))};

    if (iter != std::end(substitutions)) {
      attribute = match.str(1) + std::get<1>(*iter)(match.str(4), scope) + match.str(5);
    } else {
      throw SyntaxError("Unknown substitution ", std::quoted(match.str(3)), " specified");
    }
  }

  return attribute;
}

template <typename T, typename Node, typename Scope>
auto readAttribute(const std::string & name, const Node & node, const Scope & scope) -> T
{
  if (const auto & attribute{node.attribute(name.c_str())}) {
    std::string value{substitute(attribute.value(), scope)};

    if (value.empty()) {
#ifndef OPENSCENARIO_INTERPRETER_ALLOW_ATTRIBUTES_TO_BE_BLANK
      throw SyntaxError(
        "Blank is not allowed for the value of attribute ", std::quoted(name), " of class ",
        std::quoted(node.name()));
#else
      return T();
#endif
    }
    if (value.substr(0, 2) == "${" and value.back() == '}') {
      return boost::lexical_cast<T>(
        evaluate(std::string(value.begin() + 2, value.end() - 1), scope));
    }
    if (value.front() == '$') {
      const auto found = scope.findObject(value.substr(1));
      if (found) {
        return boost::lexical_cast<T>(boost::lexical_cast<String>(found));
      } else {
        throw SyntaxError(
          "There is no parameter named ", std::quoted(value.substr(1)), " (Attribute ",
          std::quoted(name), " of class ", std::quoted(node.name()), " references this parameter)");
      }
    } else {
      try {
        return boost::lexical_cast<T>(value);
      } catch (const boost::bad_lexical_cast &) {
        throw SyntaxError(
          "Value ", std::quoted(value), " specified for attribute ", std::quoted(name),
          " is invalid (Is not value of type ", typeid(T).name(), ")");
      }
    }
  } else {
    throw SyntaxError(
      "Required attribute ", std::quoted(name), " not specified for class ",
      std::quoted(node.name()));
  }
}

template <typename T, typename Node, typename Scope>
T readAttribute(const std::string & name, const Node & node, const Scope & scope, T && value)
{
  if (node.attribute(name.c_str())) {
    return readAttribute<T>(name, node, scope);
  } else {
    return value;
  }
}
}  // namespace reader
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__READER__ATTRIBUTE_HPP_
