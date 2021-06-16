// Copyright 2015-2021 Tier IV, Inc. All rights reserved.
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

#include <openscenario_interpreter/syntax/condition.hpp>
#include <openscenario_interpreter/utility/demangle.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
nlohmann::json & operator<<(nlohmann::json & json, const Condition & datum)
{
  json["currentEvaluation"] = boost::lexical_cast<std::string>(datum.current_evaluation);

  json["name"] = datum.name;

  // clang-format off
  static const std::unordered_map<
    std::type_index,
    std::function<std::string(const Condition &)>> table
  {
    { typeid(ByEntityCondition), [&](const Condition & condition) { return makeTypename(condition.as<ByEntityCondition>().type()); } },
    { typeid( ByValueCondition), [&](const Condition & condition) { return makeTypename(condition.as< ByValueCondition>().type()); } },
  };
  // clang-format on

  json["type"] = table.at(datum.type())(datum);

  json["description"] = datum.description();

  return json;
}
}  // namespace syntax
}  // namespace openscenario_interpreter
