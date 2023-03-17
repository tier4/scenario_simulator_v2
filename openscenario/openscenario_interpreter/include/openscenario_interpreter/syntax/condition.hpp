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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__CONDITION_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__CONDITION_HPP_

#include <algorithm>
#include <cstddef>
#include <functional>
#include <list>
#include <nlohmann/json.hpp>
#include <openscenario_interpreter/object.hpp>
#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/simulator_core.hpp>
#include <openscenario_interpreter/syntax/boolean.hpp>
#include <openscenario_interpreter/syntax/condition_edge.hpp>
#include <openscenario_interpreter/syntax/double.hpp>
#include <openscenario_interpreter/syntax/string.hpp>
#include <pugixml.hpp>
#include <tuple>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- Condition --------------------------------------------------------------
 *
 *  <xsd:complexType name="Condition">
 *    <xsd:choice>
 *      <xsd:element name="ByEntityCondition" type="ByEntityCondition"/>
 *      <xsd:element name="ByValueCondition" type="ByValueCondition"/>
 *    </xsd:choice>
 *    <xsd:attribute name="name" type="String" use="required"/>
 *    <xsd:attribute name="delay" type="Double" use="required"/>
 *    <xsd:attribute name="conditionEdge" type="ConditionEdge" use="required"/>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct Condition : public ComplexType, private SimulatorCore::ConditionEvaluation
{
  const String name;

  const Double delay;

  const ConditionEdge condition_edge;

  bool current_value;

private:
  struct History
  {
    double time;
    bool result;
  };

  std::list<History> histories;

public:
  explicit Condition(const pugi::xml_node & node, Scope & scope);

  auto evaluate() -> Object;

private:
  template <class R, class... Args>
  auto update_condition(std::function<R(Args...)> condition) -> Object
  {
    histories.push_back({evaluateSimulationTime(), ComplexType::evaluate().as<Boolean>()});
    if (auto canary_iterator = std::find_if(
          std::begin(histories), std::end(histories),
          [&](const auto & entry) { return entry.time > histories.back().time - delay; });
        std::ptrdiff_t(sizeof...(Args)) <= std::distance(std::begin(histories), canary_iterator)) {
      auto iterator = std::reverse_iterator(canary_iterator);
      auto results = std::tuple{[&] { return Args((iterator++)->result); }()...};
      histories.erase(std::begin(histories), iterator.base());
      return asBoolean(current_value = std::apply(condition, results));
    } else {
      return asBoolean(current_value = false);
    }
  }
};

auto operator<<(nlohmann::json &, const Condition &) -> nlohmann::json &;
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__CONDITION_HPP_
