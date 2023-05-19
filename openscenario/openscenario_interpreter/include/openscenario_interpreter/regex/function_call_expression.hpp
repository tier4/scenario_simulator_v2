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

#ifndef OPENSCENARIO_INTERPRETER__REGEX__FUNCTION_CALL_EXPRESSION_HPP_
#define OPENSCENARIO_INTERPRETER__REGEX__FUNCTION_CALL_EXPRESSION_HPP_

#include <regex>

namespace openscenario_interpreter
{
inline namespace regex
{
struct FunctionCallExpression
{
  /* ---- NOTE ---------------------------------------------------------------
   *
   *  function(foo, &quot;hello, world!&quot;, 3.14)
   *
   *    result[0] = function(foo, "hello, world!", 3.14)
   *    result[1] = function
   *    result[2] =         (foo, "hello, world!", 3.14)
   *    result[3] =          foo, "hello, world!", 3.14
   *
   * ---------------------------------------------------------------------- */
  static auto pattern() -> const auto &
  {
    static const auto pattern =
      std::regex(R"(^([\w@]+)(\(((?:(?:[^\("\s,\)]+|\"[^"]*\"),?\s*)*)\))?$)");
    return pattern;
  }

  static auto splitParameters(const std::string & s) -> std::vector<std::string>
  {
    static const auto pattern = std::regex(R"(([^\("\s,\)]+|\"[^"]*\"),?\s*)");

    std::vector<std::string> args;

    for (auto iter = std::sregex_iterator(std::begin(s), std::end(s), pattern);
         iter != std::sregex_iterator(); ++iter) {
      args.emplace_back((*iter)[1]);
    }

    return args;
  }
};
}  // namespace regex
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__REGEX__FUNCTION_CALL_EXPRESSION_HPP_
