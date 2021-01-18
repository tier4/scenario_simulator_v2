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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__CUSTOM_COMMAND_ACTION_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__CUSTOM_COMMAND_ACTION_HPP_

#include <openscenario_interpreter/posix/fork_exec.hpp>
#include <openscenario_interpreter/reader/content.hpp>
#include <openscenario_interpreter/string/cat.hpp>

#include <memory>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- CustomCommandAction ----------------------------------------------------
 *
 *  <xsd:complexType name="CustomCommandAction">
 *    <xsd:simpleContent>
 *      <xsd:extension base="xsd:string">
 *        <xsd:attribute name="type" type="String" use="required"/>
 *      </xsd:extension>
 *    </xsd:simpleContent>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct CustomCommandAction
{
  const String type;

  const String content;

  const std::true_type accomplished {};

  static int exitSuccess()
  {
    throw EXIT_SUCCESS;
  }

  static int exitFailure()
  {
    throw EXIT_FAILURE;
  }

  static int error()
  {
    throw std::runtime_error(cat(__FILE__, ":", __LINE__));
  }

  static int segv()
  {
    return *reinterpret_cast<std::add_pointer<int>::type>(0);
  }

  const std::unordered_map<
    std::string, std::function<int(void)>
  >
  builtins
  {
    std::make_pair("error", error),

    std::make_pair("sigsegv", segv),

    std::make_pair("exitSuccess", exitSuccess),
    std::make_pair("exitFailure", exitFailure),
  };

  template
  <
    typename Node, typename Scope
  >
  explicit CustomCommandAction(const Node & node, Scope & scope)
  : type(
      readAttribute<String>("type", node, scope)),
    content(
      readContent<String>(node, scope))
  {}

  auto evaluate()
  {
    const auto iter = builtins.find(type);

    if (iter != std::end(builtins)) {
      std::get<1>(* iter)();
    } else {
      fork_exec(type, content);
    }

    return unspecified;
  }

  friend std::ostream & operator<<(std::ostream & os, const CustomCommandAction & action)
  {
    os << indent << blue << "<CustomCommandAction" << " " << highlight("type", action.type);

    if (action.content.empty()) {
      return os << blue << "/>" << reset;
    } else {
      return os << blue << ">" << reset << action.content << blue << "</CustomCommandAction>" <<
             reset;
    }
  }
};
}
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__CUSTOM_COMMAND_ACTION_HPP_
