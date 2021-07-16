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

#include <iterator>  // std::distance
#include <openscenario_interpreter/error.hpp>
#include <openscenario_interpreter/posix/fork_exec.hpp>
#include <openscenario_interpreter/procedure.hpp>
#include <openscenario_interpreter/reader/content.hpp>
#include <stdexcept>  // std::runtime_error
#include <string>
#include <type_traits>  // std::true_type
#include <unordered_map>
#include <utility>  // std::make_pair
#include <vector>

namespace openscenario_interpreter
{
inline namespace syntax
{
template <int Value>
struct SpecialAction : public std::integral_constant<int, Value>
{
};

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
struct CustomCommandAction : private Scope
{
  const String type;

  const String content;

  template <typename Node>
  explicit CustomCommandAction(const Node & node, const Scope & outer_scope)
  : Scope(outer_scope),
    type(readAttribute<String>("type", node, localScope())),
    content(readContent<String>(node, localScope()))
  {
  }

  const std::true_type accomplished{};

  static int walkStraightAction(
    const std::vector<std::string> & actors, const Scope & current_scope)
  {
    for (const auto & actor : actors) {
      applyWalkStraightAction(actor);
    }

    for (const auto actor : current_scope.actors) {
      applyWalkStraightAction(actor);
    }

    return current_scope.actors.size();
  }

  static int exitSuccess(const std::vector<std::string> &, const Scope &)
  {
    throw SpecialAction<EXIT_SUCCESS>();
  }

  static int exitFailure(const std::vector<std::string> &, const Scope &)
  {
    throw SpecialAction<EXIT_FAILURE>();
  }

  static int error(const std::vector<std::string> &, const Scope &)
  {
    throw Error(__FILE__, ":", __LINE__);
  }

  static int segv(const std::vector<std::string> &, const Scope &)
  {
    return *reinterpret_cast<std::add_pointer<int>::type>(0);
  }

  static int test(const std::vector<std::string> & args, const Scope &)
  {
    std::cout << "test" << std::endl;

    for (auto iter = std::cbegin(args); iter != std::cend(args); ++iter) {
      std::cout << "  args[" << std::distance(std::cbegin(args), iter) << "] = " << *iter << "\n";
    }

    return args.size();
  }

  const std::unordered_map<
    std::string, std::function<int(const std::vector<std::string> &, const Scope &)>>
    builtins{
      std::make_pair("WalkStraightAction", walkStraightAction),
      std::make_pair("error", error),
      std::make_pair("exitFailure", exitFailure),
      std::make_pair("exitSuccess", exitSuccess),
      std::make_pair("sigsegv", segv),  // Deprecated
      std::make_pair("test", test),
    };

  static auto split(const std::string & s)
  {
    static const std::regex pattern{R"(([^\("\s,\)]+|\"[^"]*\"),?\s*)"};

    std::vector<std::string> args{};

    for (std::sregex_iterator iter{std::begin(s), std::end(s), pattern}, end; iter != end; ++iter) {
      args.emplace_back((*iter)[1]);
    }

    return args;
  }

  auto evaluate()
  {
    /* ---- NOTE ---------------------------------------------------------------
     *
     *  <CustomCommandAction type="function(hoge, &quot;hello, world!&quot;, 3.14)"/>
     *
     *  result[0] = function(hoge, "hello, world!", 3.14)
     *  result[1] = function
     *  result[2] = (hoge, "hello, world!", 3.14)
     *  result[3] = hoge, "hello, world!", 3.14
     *
     * ---------------------------------------------------------------------- */
    static const std::regex pattern{R"(^(\w+)(\(((?:(?:[^\("\s,\)]+|\"[^"]*\"),?\s*)*)\))?$)"};

    std::smatch result{};

    if (std::regex_match(type, result, pattern) && builtins.find(result[1]) != std::end(builtins)) {
      builtins.at(result[1])(split(result[3]), localScope());
    } else {
      fork_exec(type, content);
    }

    return unspecified;
  }

  friend std::ostream & operator<<(std::ostream & os, const CustomCommandAction & action)
  {
    os << indent << blue << "<CustomCommandAction"
       << " " << highlight("type", action.type);

    if (action.content.empty()) {
      return os << blue << "/>" << reset;
    } else {
      return os << blue << ">" << reset << action.content << blue << "</CustomCommandAction>"
                << reset;
    }
  }
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__CUSTOM_COMMAND_ACTION_HPP_
