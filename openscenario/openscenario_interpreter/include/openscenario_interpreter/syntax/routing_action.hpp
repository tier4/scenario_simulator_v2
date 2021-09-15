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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__ROUTING_ACTION_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__ROUTING_ACTION_HPP_

#include <openscenario_interpreter/syntax/acquire_position_action.hpp>
#include <openscenario_interpreter/syntax/assign_route_action.hpp>
#include <utility>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- RoutingAction ----------------------------------------------------------
 *
 * <xsd:complexType name="RoutingAction">
 *   <xsd:choice>
 *     <xsd:element name="AssignRouteAction" type="AssignRouteAction"/>
 *     <xsd:element name="FollowTrajectoryAction" type="FollowTrajectoryAction"/>
 *     <xsd:element name="AcquirePositionAction" type="AcquirePositionAction"/>
 *   </xsd:choice>
 * </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct RoutingAction : public ComplexType
{
  template <typename Node, typename... Ts>
  explicit RoutingAction(const Node & node, Ts &&... xs)
  // clang-format off
  : ComplexType(
      choice(node,
        std::make_pair(     "AssignRouteAction", [&](auto && node) { return make<     AssignRouteAction>(node, std::forward<decltype(xs)>(xs)...); }),
        std::make_pair("FollowTrajectoryAction", [&](auto && node) { throw UNSUPPORTED_ELEMENT_SPECIFIED(node.name()); return unspecified; }),
        std::make_pair( "AcquirePositionAction", [&](auto && node) { return make< AcquirePositionAction>(node, std::forward<decltype(xs)>(xs)...); })))
  // clang-format on
  {
  }

  auto endsImmediately() const -> bool;

  auto run() -> void;
};

template <typename Result, typename Function, typename... Ts>
auto apply(Function && function, const RoutingAction & action, Ts &&... xs) -> Result
{
  using functor = std::function<Result(Function &&, const RoutingAction &, Ts &&...)>;

#define BOILERPLATE(TYPE)                                                               \
  std::make_pair<std::type_index, functor>(                                             \
    typeid(TYPE), [](Function && function, const RoutingAction & action, Ts &&... xs) { \
      return function(action.as<TYPE>(), std::forward<decltype(xs)>(xs)...);            \
    })

  static const std::unordered_map<std::type_index, functor> overloads{
    // clang-format off
    BOILERPLATE(     AssignRouteAction),
    // BOILERPLATE(FollowTrajectoryAction),
    BOILERPLATE( AcquirePositionAction),
    // clang-format on
  };

#undef BOILERPLATE

  try {
    return overloads.at(action.type())(
      std::forward<decltype(function)>(function), action, std::forward<decltype(xs)>(xs)...);
  } catch (const std::out_of_range &) {
    throw UNSUPPORTED_SETTING_DETECTED(RoutingAction, makeTypename(action.type().name()));
  }
}

template <typename Result, typename Function, typename... Ts>
auto apply(Function && function, RoutingAction & action, Ts &&... xs) -> Result
{
  using functor = std::function<Result(Function &&, RoutingAction &, Ts && ...)>;

#define BOILERPLATE(TYPE)                                                         \
  std::make_pair<std::type_index, functor>(                                       \
    typeid(TYPE), [](Function && function, RoutingAction & action, Ts &&... xs) { \
      return function(action.as<TYPE>(), std::forward<decltype(xs)>(xs)...);      \
    })

  static const std::unordered_map<std::type_index, functor> overloads{
    // clang-format off
    BOILERPLATE(     AssignRouteAction),
    // BOILERPLATE(FollowTrajectoryAction),
    BOILERPLATE( AcquirePositionAction),
    // clang-format on
  };

#undef BOILERPLATE

  try {
    return overloads.at(action.type())(
      std::forward<decltype(function)>(function), action, std::forward<decltype(xs)>(xs)...);
  } catch (const std::out_of_range &) {
    throw UNSUPPORTED_SETTING_DETECTED(RoutingAction, makeTypename(action.type().name()));
  }
}
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__ROUTING_ACTION_HPP_
