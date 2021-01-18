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

#include <boost/algorithm/string.hpp>
#include <openscenario_interpreter/posix/fork_exec.hpp>
#include <openscenario_interpreter/reader/content.hpp>

#include <memory>
#include <string>
#include <unordered_map>
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

  const std::unordered_map<
    std::string,
    std::function<int(void)>
  >
  builtins
  {
    {
      "error", []() -> int
      {
        struct UnexpectedException {} it;
        throw it;
      }
    },

    {
      "sigsegv", []()
      {
        return *reinterpret_cast<std::add_pointer<int>::type>(0);
      }
    },

    {
      "exitSuccess", []() -> int
      {
        throw EXIT_SUCCESS;
      }
    },

    {
      "exitFailure", []() -> int
      {
        throw EXIT_FAILURE;
      }
    },
  };

  static auto split(const std::string & target)
  {
    std::vector<std::string> result {};

    boost::split(result, target, boost::is_space());

    return result;
  }

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

  static constexpr auto accomplished() noexcept
  {
    return true;
  }

  auto execute() const
  {
    const auto command = split(content.empty() ? type : type + " " + content);

    return fork_exec(command);
  }

  auto evaluate()
  {
    auto iter {
      builtins.find(type)
    };

    if (iter != std::end(builtins)) {
      std::get<1>(* iter)();
      return unspecified;
    } else {
      execute();
      return unspecified;
    }
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
