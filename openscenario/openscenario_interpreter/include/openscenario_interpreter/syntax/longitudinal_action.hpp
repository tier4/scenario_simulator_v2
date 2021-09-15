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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__LONGITUDINAL_ACTION_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__LONGITUDINAL_ACTION_HPP_

#include <openscenario_interpreter/syntax/speed_action.hpp>
#include <utility>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- LongitudinalAction -----------------------------------------------------
 *
 *  <xsd:complexType name="LongitudinalAction">
 *    <xsd:choice>
 *      <xsd:element name="SpeedAction" type="SpeedAction"/>
 *      <xsd:element name="LongitudinalDistanceAction" type="LongitudinalDistanceAction"/>
 *    </xsd:choice>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct LongitudinalAction : public Element
{
  template <typename Node, typename... Ts>
  explicit LongitudinalAction(const Node & node, Ts &&... xs)
  // clang-format off
  : Element(
      choice(node,
        std::make_pair(               "SpeedAction", [&](auto && node) { return make<SpeedAction>(node, std::forward<decltype(xs)>(xs)...); }),
        std::make_pair("LongitudinalDistanceAction", [&](auto && node) { throw UNSUPPORTED_ELEMENT_SPECIFIED(node.name()); return unspecified; })))
  // clang-format on
  {
  }

  auto endsImmediately() const -> bool;
};

template <typename Result = void, typename Function, typename... Ts>
auto apply(Function && function, const LongitudinalAction & action, Ts &&... xs) -> Result
{
  using functor = std::function<Result(Function &&, const LongitudinalAction &, Ts &&...)>;

#define BOILERPLATE(TYPE)                                                                    \
  std::make_pair<std::type_index, functor>(                                                  \
    typeid(TYPE), [](Function && function, const LongitudinalAction & action, Ts &&... xs) { \
      return function(action.as<TYPE>(), std::forward<decltype(xs)>(xs)...);                 \
    })

  static const std::unordered_map<std::type_index, functor> overloads{
    // clang-format off
    BOILERPLATE(               SpeedAction),
    // BOILERPLATE(LongitudinalDistanceAction),
    // clang-format on
  };

#undef BOILERPLATE

  try {
    return overloads.at(action.type())(
      std::forward<decltype(function)>(function), action, std::forward<decltype(xs)>(xs)...);
  } catch (const std::out_of_range &) {
    throw UNSUPPORTED_SETTING_DETECTED(LongitudinalAction, makeTypename(action.type().name()));
  }
}
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__LONGITUDINAL_ACTION_HPP_
