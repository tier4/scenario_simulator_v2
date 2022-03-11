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

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <functional>
#include <openscenario_interpreter/reader/evaluate.hpp>
#include <openscenario_interpreter/syntax/parameter_type.hpp>
#include <openscenario_interpreter/utility/highlighter.hpp>
#include <pugixml.hpp>
#include <regex>
#include <scenario_simulator_exception/exception.hpp>
#include <string>
#include <unordered_map>

namespace openscenario_interpreter
{
inline namespace reader
{
template <typename Scope>
auto substitute(std::string attribute, Scope & scope)
{
  auto find_pkg_share = [](auto && package_name, auto &&) {
    return ament_index_cpp::get_package_share_directory(package_name);
  };

  auto var = [](auto && name, auto && scope) -> String {
    if (const auto found = scope.ref(name); found) {
      return boost::lexical_cast<String>(found);
    } else {
      return "";
    }
  };

  auto dirname = [](auto &&, auto && scope) {
    return scope.global().pathname.parent_path().string();
  };

  // NOTE: https://design.ros2.org/articles/roslaunch_xml.html#dynamic-configuration
  static const std::unordered_map<
    std::string, std::function<std::string(const std::string &, Scope &)> >
    substitutions{{"find-pkg-share", find_pkg_share}, {"var", var}, {"dirname", dirname}};

  static const auto pattern = std::regex(R"((.*)\$\((([\w-]+)\s?([^\)]*))\)(.*))");

  for (std::smatch result; std::regex_match(attribute, result, pattern);) {
    if (const auto iter = substitutions.find(result.str(3)); iter != std::end(substitutions)) {
      attribute = result.str(1) + std::get<1>(*iter)(result.str(4), scope) + result.str(5);
    } else {
      throw SyntaxError("Unknown substitution ", std::quoted(result.str(3)), " specified");
    }
  }

  return attribute;
}

template <typename T, typename Node, typename Scope>
auto readAttribute(const std::string & name, const Node & node, const Scope & scope) -> T
{
  if (const auto & attribute = node.attribute(name.c_str())) {
    if (std::string value = substitute(attribute.value(), scope); value.empty()) {
      return T();
    } else if (value.substr(0, 2) == "${" and value.back() == '}') {
      return boost::lexical_cast<T>(
        evaluate(std::string(value.begin() + 2, value.end() - 1), scope));
    } else if (value.front() == '$') {
      if (const auto found = scope.ref(value.substr(1)); found) {
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
auto readAttribute(const std::string & name, const Node & node, const Scope & scope, T && value)
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
