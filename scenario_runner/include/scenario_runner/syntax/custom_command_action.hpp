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

#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <scenario_runner/reader/content.hpp>
#include <boost/algorithm/string.hpp>

#include <memory>
#include <string>
#include <system_error>
#include <unordered_map>
#include <vector>

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
  const String type;

  const String content;

  const std::unordered_map<std::string, std::function<int(void)>> builtins
  {
    std::make_pair(
      "exitSuccess",
      []() -> int
      {
        throw EXIT_SUCCESS;
      }),

    std::make_pair(
      "exitFailure",
      []() -> int
      {
        throw EXIT_FAILURE;
      }),
  };

  static auto split(const std::string & target)
  {
    std::vector<std::string> result {};

    boost::split(result, target, boost::is_space());

    return result;
  }

  template<typename Node, typename Scope>
  explicit CustomCommandAction(const Node & node, Scope & scope)
  : type
    {
      readAttribute<String>("type", node, scope)
    },
    content
    {
      readContent<String>(node, scope)
    }
  {}

  static constexpr auto accomplished() noexcept
  {
    return true;
  }

  auto execvp(const std::vector<std::string> & args) const
  {
    std::vector<std::vector<char>> buffer {};

    buffer.resize(args.size());

    std::vector<std::add_pointer<char>::type> argv {};

    argv.reserve(args.size());

    for (const auto & each : args) {
      std::cout << std::quoted(each) << std::endl;
      buffer.emplace_back(std::begin(each), std::end(each));
      buffer.back().push_back('\0');

      argv.push_back(buffer.back().data());
    }

    argv.emplace_back(static_cast<char *>(0));

    return ::execvp(argv[0], argv.data());
  }

  auto execute() const
  {
    const auto pid {fork()};

    int status {0};

    if (pid < 0) {
      throw std::system_error(errno, std::system_category());
    } else {
      switch (pid) {
        case 0:
          if (execvp(split(content.empty() ? type : type + " " + content)) < 0) {
            std::exit(EXIT_FAILURE);
          }
          break;

        default:
          do {
            ::waitpid(pid, &status, WUNTRACED);
          } while (!WIFEXITED(status) && !WIFSIGNALED(status));
      }

      return EXIT_SUCCESS;
    }
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

  template<typename ... Ts>
  friend std::basic_ostream<Ts...> & operator<<(
    std::basic_ostream<Ts...> & os, const CustomCommandAction & action)
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
}  // namespace scenario_runner

#endif  // SCENARIO_RUNNER__SYNTAX__CUSTOM_COMMAND_ACTION_HPP_
