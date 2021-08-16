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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__SPEED_ACTION_TARGET_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__SPEED_ACTION_TARGET_HPP_

#include <openscenario_interpreter/syntax/absolute_target_speed.hpp>
#include <openscenario_interpreter/syntax/relative_target_speed.hpp>
#include <utility>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- SpeedActionTarget ------------------------------------------------------
 *
 *  <xsd:complexType name="SpeedActionTarget">
 *    <xsd:choice>
 *      <xsd:element name="RelativeTargetSpeed" type="RelativeTargetSpeed"/>
 *      <xsd:element name="AbsoluteTargetSpeed" type="AbsoluteTargetSpeed"/>
 *    </xsd:choice>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
#define ELEMENT(TYPE) \
  std::make_pair(     \
    #TYPE, [&](auto && node) { return make<TYPE>(node, std::forward<decltype(xs)>(xs)...); })

struct SpeedActionTarget : public Element
{
  template <typename Node, typename... Ts>
  explicit SpeedActionTarget(const Node & node, Ts &&... xs)
  : Element(choice(node, ELEMENT(RelativeTargetSpeed), ELEMENT(AbsoluteTargetSpeed)))
  {
  }

  std::function<double()> getCalculateAbsoluteTargetSpeed() const
  {
    if (is<AbsoluteTargetSpeed>()) {
      return as<AbsoluteTargetSpeed>().getCalculateAbsoluteTargetSpeed();
    } else if (is<RelativeTargetSpeed>()) {
      return as<RelativeTargetSpeed>().getCalculateAbsoluteTargetSpeed();
    } else {
      throw UNSUPPORTED_SETTING_DETECTED(SpeedActionTarget, this->type().name());
    }
  }

  std::function<bool(const EntityRef & actor)> getIsEnd() const
  {
    if (is<AbsoluteTargetSpeed>()) {
      return as<AbsoluteTargetSpeed>().getIsEnd();
    } else if (is<RelativeTargetSpeed>()) {
      return as<RelativeTargetSpeed>().getIsEnd();
    } else {
      throw UNSUPPORTED_SETTING_DETECTED(SpeedActionTarget, this->type().name());
    }
  }
};

#undef ELEMENT
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__SPEED_ACTION_TARGET_HPP_
