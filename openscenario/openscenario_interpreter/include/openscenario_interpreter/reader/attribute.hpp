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

#ifndef OPENSCENARIO_INTERPRETER__READER__ATTRIBUTE_HPP_
#define OPENSCENARIO_INTERPRETER__READER__ATTRIBUTE_HPP_

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <concealer/execute.hpp>
#include <functional>
#include <openscenario_interpreter/reader/evaluate.hpp>
#include <openscenario_interpreter/syntax/parameter_type.hpp>
#include <openscenario_interpreter/utility/highlighter.hpp>
#include <optional>
#include <pugixml.hpp>
#include <regex>
#include <scenario_simulator_exception/exception.hpp>
#include <string>
#include <unordered_map>

namespace openscenario_interpreter
{
struct Scope;

inline namespace syntax
{
struct Rule;
}  // namespace syntax

inline namespace reader
{
auto substitute(const std::string & attribute, const Scope & scope) -> String;

template <typename T, typename Node, typename Scope>
auto readAttribute(const std::string & name, const Node & node, const Scope & scope) -> T
{
  auto is_openscenario_standard_expression = [](const auto & s) {
    return s.substr(0, 2) == "${" and s.back() == '}';
  };

  auto read_openscenario_standard_expression = [&](const auto & s) {
    return boost::lexical_cast<T>(evaluate(std::string(std::begin(s) + 2, std::end(s) - 1), scope));
  };

  auto is_openscenario_standard_parameter_reference = [](const auto & s) {
    return s.front() == '$';
  };

  auto read_openscenario_standard_parameter_reference = [&](const auto & s) {
    // TODO Use `return scope.template ref<T>(s.substr(1));`
    if (auto && object = scope.ref(s.substr(1)); object) {
      return boost::lexical_cast<T>(boost::lexical_cast<String>(object));
    } else {
      throw SyntaxError(
        "There is no parameter named ", std::quoted(s.substr(1)), " (Attribute ", std::quoted(name),
        " of class ", std::quoted(node.name()), " references this parameter)");
    }
  };

  auto read_openscenario_standard_literal = [&](const auto & s) {
    try {
      return boost::lexical_cast<T>(s);
    } catch (const boost::bad_lexical_cast &) {
      throw SyntaxError(
        "Value ", std::quoted(s), " specified for attribute ", std::quoted(name),
        " is invalid (Is not value of type ", makeTypename(typeid(T)), ")");
    }
  };

  // NOTE: https://www.asam.net/index.php?eID=dumpFile&t=f&f=4092&token=d3b6a55e911b22179e3c0895fe2caae8f5492467#_parameters

  if (const auto & attribute = node.attribute(name.c_str())) {
    // NOTE: `substitute` is TIER IV extension (Non-OpenSCENARIO standard)
    if (std::string value = substitute(attribute.value(), scope); value.empty()) {
      return T();
    } else if (is_openscenario_standard_expression(value)) {
      return read_openscenario_standard_expression(value);
    } else if (is_openscenario_standard_parameter_reference(value)) {
      return read_openscenario_standard_parameter_reference(value);
    } else {
      return read_openscenario_standard_literal(value);
    }
  } else {
    throw SyntaxError(
      "Required attribute ", std::quoted(name), " not specified for class ",
      std::quoted(node.name()));
  }
}

extern template auto readAttribute(const std::string &, const pugi::xml_node &, const Scope &)
  -> Boolean;
extern template auto readAttribute(const std::string &, const pugi::xml_node &, const Scope &)
  -> UnsignedShort;
extern template auto readAttribute(const std::string &, const pugi::xml_node &, const Scope &)
  -> UnsignedInteger;
extern template auto readAttribute(const std::string &, const pugi::xml_node &, const Scope &)
  -> Double;
extern template auto readAttribute(const std::string &, const pugi::xml_node &, const Scope &)
  -> String;
extern template auto readAttribute(const std::string &, const pugi::xml_node &, const Scope &)
  -> syntax::Rule;

template <typename T, typename Node, typename Scope>
auto readAttribute(const std::string & name, const Node & node, const Scope & scope, T && value)
  -> T
{
  if (node.attribute(name.c_str())) {
    return readAttribute<T>(name, node, scope);
  } else {
    return value;
  }
}

template <typename T, typename Node, typename Scope>
auto readAttribute(const std::string & name, const Node & node, const Scope & scope, std::nullopt_t)
{
  if (node.attribute(name.c_str())) {
    return std::make_optional(readAttribute<T>(name, node, scope));
  } else {
    return std::optional<T>();
  }
}
}  // namespace reader
}  // namespace openscenario_interpreter
#endif  // OPENSCENARIO_INTERPRETER__READER__ATTRIBUTE_HPP_
