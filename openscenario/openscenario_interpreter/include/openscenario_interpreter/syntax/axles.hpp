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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__AXLES_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__AXLES_HPP_

#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/axle.hpp>
#include <pugixml.hpp>
#include <traffic_simulator_msgs/msg/axles.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- Axles ------------------------------------------------------------------
 *
 * <xsd:complexType name="Axles">
 *   <xsd:sequence>
 *     <xsd:element name="FrontAxle" type="Axle"/>
 *     <xsd:element name="RearAxle" type="Axle"/>
 *     <xsd:element name="AdditionalAxle" type="Axle" minOccurs="0" maxOccurs="unbounded"/>
 *   </xsd:sequence>
 * </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct Axles
{
  const FrontAxle front_axle;

  const RearAxle rear_axle;

  std::list<AdditionalAxle> additional_axles;

  Axles() = default;

  explicit Axles(const pugi::xml_node & node, Scope & scope);

  explicit operator traffic_simulator_msgs::msg::Axles() const;
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__AXLES_HPP_
