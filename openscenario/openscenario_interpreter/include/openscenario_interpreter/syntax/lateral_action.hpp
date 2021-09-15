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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__LATERAL_ACTION_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__LATERAL_ACTION_HPP_

#include <openscenario_interpreter/syntax/lane_change_action.hpp>
#include <utility>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- LateralAction ----------------------------------------------------------
 *
 *  <xsd:complexType name="LateralAction">
 *    <xsd:choice>
 *      <xsd:element name="LaneChangeAction" type="LaneChangeAction"/>
 *      <xsd:element name="LaneOffsetAction" type="LaneOffsetAction"/>
 *      <xsd:element name="LateralDistanceAction" type="LateralDistanceAction"/>
 *    </xsd:choice>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct LateralAction : public ComplexType
{
  template <typename Node, typename Scope>
  explicit LateralAction(const Node & node, Scope & scope)
  // clang-format off
  : ComplexType(
      choice(node,
        std::make_pair(     "LaneChangeAction", [&](auto && node) { return make<LaneChangeAction>(node, scope); }),
        std::make_pair(     "LaneOffsetAction", [&](auto && node) { throw UNSUPPORTED_ELEMENT_SPECIFIED(node.name()); return unspecified; }),
        std::make_pair("LateralDistanceAction", [&](auto && node) { throw UNSUPPORTED_ELEMENT_SPECIFIED(node.name()); return unspecified; })))
  // clang-format on
  {
  }

  auto endsImmediately() const -> bool;

  auto run() -> void;
};

template <typename Result, typename Function, typename... Ts>
auto apply(Function && function, const LateralAction & action, Ts &&... xs) -> Result
{
  using functor = std::function<Result(Function &&, const LateralAction &, Ts &&...)>;

#define BOILERPLATE(TYPE)                                                               \
  std::make_pair<std::type_index, functor>(                                             \
    typeid(TYPE), [](Function && function, const LateralAction & action, Ts &&... xs) { \
      return function(action.as<TYPE>(), std::forward<decltype(xs)>(xs)...);            \
    })

  static const std::unordered_map<std::type_index, functor> overloads{
    // clang-format off
    BOILERPLATE(     LaneChangeAction),
    // BOILERPLATE(     LaneOffsetAction),
    // BOILERPLATE(LateralDistanceAction),
    // clang-format on
  };

#undef BOILERPLATE

  try {
    return overloads.at(action.type())(
      std::forward<decltype(function)>(function), action, std::forward<decltype(xs)>(xs)...);
  } catch (const std::out_of_range &) {
    throw UNSUPPORTED_SETTING_DETECTED(LateralAction, makeTypename(action.type().name()));
  }
}

template <typename Result, typename Function, typename... Ts>
auto apply(Function && function, LateralAction & action, Ts &&... xs) -> Result
{
  using functor = std::function<Result(Function &&, LateralAction &, Ts && ...)>;

#define BOILERPLATE(TYPE)                                                         \
  std::make_pair<std::type_index, functor>(                                       \
    typeid(TYPE), [](Function && function, LateralAction & action, Ts &&... xs) { \
      return function(action.as<TYPE>(), std::forward<decltype(xs)>(xs)...);      \
    })

  static const std::unordered_map<std::type_index, functor> overloads{
    // clang-format off
    BOILERPLATE(     LaneChangeAction),
    // BOILERPLATE(     LaneOffsetAction),
    // BOILERPLATE(LateralDistanceAction),
    // clang-format on
  };

#undef BOILERPLATE

  try {
    return overloads.at(action.type())(
      std::forward<decltype(function)>(function), action, std::forward<decltype(xs)>(xs)...);
  } catch (const std::out_of_range &) {
    throw UNSUPPORTED_SETTING_DETECTED(LateralAction, makeTypename(action.type().name()));
  }
}
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__LATERAL_ACTION_HPP_
