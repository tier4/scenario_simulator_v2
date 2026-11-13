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

#include <openscenario_interpreter/reader/attribute.hpp>
#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/rule.hpp>

namespace openscenario_interpreter
{
inline namespace reader
{
auto substitute(const std::string & attribute, const Scope & scope) -> String
{
  auto dirname = [](auto &&, auto && scope) { return scope.dirname(); };

  auto find_pkg_share = [](auto && package_name, const auto &) {
    return ament_index_cpp::get_package_share_directory(package_name);
  };

  auto ros2 = [](auto && arguments, const auto &) {
    auto remove_trailing_newline = [](auto && s) {
      while (s.back() == '\n') {
        s.pop_back();
      }
      return s;
    };
    if (auto && result = remove_trailing_newline(concealer::dollar("ros2 " + arguments));
        result.find('\n') != std::string::npos) {
      throw SyntaxError(
        "The substitution result by `$(ros2 ...)` must not contain a newline character. "
        "You gave `$(ros2 ",
        arguments, ")` and the result was ",
        std::quoted(boost::replace_all_copy(result, "\n", "\\n")),
        ", which is unacceptable for the reasons stated above.");
    } else {
      return result;
    }
  };

  auto var = [](auto && name, const auto & scope) {
    // TODO: Return the value of the launch configuration variable instead of the OpenSCENARIO parameter.
    if (const auto found = scope.ref(name); found) {
      return boost::lexical_cast<String>(found);
    } else {
      return String();
    }
  };

  // NOTE: https://design.ros2.org/articles/roslaunch_xml.html#dynamic-configuration
  static const std::unordered_map<
    std::string, std::function<std::string(const std::string &, const Scope &)> >
    substitutions{
      {"dirname", dirname},
      // TODO {"env", env},
      // TODO {"eval", eval},
      // TODO {"exec-in-package", exec_in_package},
      // TODO {"find-exec", find_exec},
      // TODO {"find-pkg-prefix", find_pkg_prefix},
      {"find-pkg-share", find_pkg_share},
      {"ros2",
       ros2},  // NOTE: TIER IV extension (Not included in the ROS 2 Launch XML Substitution)
      {"var", var},
    };

  static const auto pattern = std::regex(R"((.*)\$\((([\w-]+)\s?([^\)]*))\)(.*))");

  auto substituted = attribute;
  for (std::smatch result; std::regex_match(substituted, result, pattern);) {
    if (const auto iter = substitutions.find(result.str(3)); iter != std::end(substitutions)) {
      substituted = result.str(1) + std::get<1>(*iter)(result.str(4), scope) + result.str(5);
    } else {
      throw SyntaxError("Unknown substitution ", std::quoted(result.str(3)), " specified");
    }
  }

  return substituted;
}

template auto readAttribute(const std::string &, const pugi::xml_node &, const Scope &) -> Boolean;
template auto readAttribute(const std::string &, const pugi::xml_node &, const Scope &) -> Double;
template auto readAttribute(const std::string &, const pugi::xml_node &, const Scope &)
  -> syntax::Rule;
template auto readAttribute(const std::string &, const pugi::xml_node &, const Scope &) -> String;
template auto readAttribute(const std::string &, const pugi::xml_node &, const Scope &)
  -> UnsignedShort;
template auto readAttribute(const std::string &, const pugi::xml_node &, const Scope &)
  -> UnsignedInteger;
}  // namespace reader
}  // namespace openscenario_interpreter
