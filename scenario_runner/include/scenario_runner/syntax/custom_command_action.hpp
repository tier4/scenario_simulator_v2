// Copyright 2015-2020 Autoware Foundation. All rights reserved.
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

#ifndef SCENARIO_RUNNER__SYNTAX__CUSTOM_COMMAND_ACTION_HPP_
#define SCENARIO_RUNNER__SYNTAX__CUSTOM_COMMAND_ACTION_HPP_

#include <scenario_runner/reader/content.hpp>
#include <scenario_runner/syntax/command.hpp>

namespace scenario_runner
{
inline namespace syntax
{
/* ==== CustomCommandAction ==================================================
 *
 * <xsd:complexType name="CustomCommandAction">
 *   <xsd:simpleContent>
 *     <xsd:extension base="xsd:string">
 *       <xsd:attribute name="type" type="String" use="required"/>
 *     </xsd:extension>
 *   </xsd:simpleContent>
 * </xsd:complexType>
 *
 * ======================================================================== */
struct CustomCommandAction
{
  const Command command;

  const String content;

  template<typename Node, typename Scope>
  explicit CustomCommandAction(const Node & node, Scope & scope)
  : command{readAttribute<Command>("type", node, scope)},
    content{readContent<String>(node, scope)}
  {}

  Element aux;

  auto accomplished()
  {
    switch (command) {
      case Command::debugAccomplishment:

        if (!aux) {
          aux = make(std::chrono::high_resolution_clock::now());
          return false;
        } else {
          const auto elapsed {
            std::chrono::duration_cast<std::chrono::seconds>(
              std::chrono::high_resolution_clock::now() -
              aux.as<decltype(std::chrono::high_resolution_clock::now())>(__FILE__, __LINE__))
          };

          return 3 < elapsed.count();
        }

      default:
        return false;
    }
  }

  auto evaluate()
  {
    switch (command) {
      case Command::debugAccomplishment:
        std::cout << *this << std::endl;
        return unspecified;

      case Command::print:
        std::cout << content << std::endl;
        return unspecified;

      default:
        throw command;
    }
  }

  template<typename ... Ts>
  friend std::basic_ostream<Ts...> & operator<<(
    std::basic_ostream<Ts...> & os,
    const CustomCommandAction & action)
  {
    os << indent << blue << "<CustomCommandAction" << " " << highlight("type", action.command);

    switch (action.command) {
      case Command::print:
        return os << blue << ">" << reset << action.content << blue << "</CustomCommandAction>" <<
               reset;

      case Command::debugAccomplishment:
      default:
        return os << blue << "/>" << reset;
    }
  }
};
}
}  // namespace scenario_runner

#endif  // SCENARIO_RUNNER__SYNTAX__CUSTOM_COMMAND_ACTION_HPP_
