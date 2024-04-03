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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__CONTROLLER_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__CONTROLLER_HPP_

#include <openscenario_interpreter/simulator_core.hpp>
#include <openscenario_interpreter/syntax/parameter_declarations.hpp>
#include <openscenario_interpreter/syntax/pedestrian.hpp>
#include <openscenario_interpreter/syntax/properties.hpp>
#include <openscenario_interpreter/syntax/vehicle.hpp>
#include <pugixml.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- Controller -------------------------------------------------------------
 *
 *  Defines a controller type and parameters for the controller.
 *
 *  Used in:
 *    AssignControllerAction,
 *    Catalog
 *    ControllerDistributionEntry,
 *    ObjectController,
 *
 *  <xsd:complexType name="Controller">
 *    <xsd:all>
 *      <xsd:element name="ParameterDeclarations" type="ParameterDeclarations" minOccurs="0"/>
 *      <xsd:element name="Properties" type="Properties"/>
 *    </xsd:all>
 *    <xsd:attribute name="name" type="String" use="required"/>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct Controller : public Scope
{
  const ParameterDeclarations parameter_declarations;  // Definition of additional parameters.

  /*
     NOTE: The term "controller" in OpenSCENARIO is a concept equivalent to
     "the person driving the car. Here, Autoware is considered anthropomorphic.
     In other words, the sensor performance of Autoware in a simulation is
     described in ScenarioObject.ObjectController.Controller.Properties as
     "characteristics of the person driving the car.
  */
  Properties properties;  // Describing properties for the controller.

  explicit Controller(const pugi::xml_node &, Scope &);

  auto isAutoware() const & -> bool;

  template <typename T>
  auto of() const & -> const String &
  {
    if (properties.get<Boolean>("isEgo")) {
      return traffic_simulator::VehicleBehavior::autoware();
    } else if (name.empty()) {
      if constexpr (std::is_same_v<std::decay_t<T>, Vehicle>) {
        return traffic_simulator::VehicleBehavior::defaultBehavior();
      } else if constexpr (std::is_same_v<std::decay_t<T>, Pedestrian>) {
        return traffic_simulator::PedestrianBehavior::defaultBehavior();
      } else {
        return name;
      }
    } else {
      return name;
    }
  }
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__CONTROLLER_HPP_
