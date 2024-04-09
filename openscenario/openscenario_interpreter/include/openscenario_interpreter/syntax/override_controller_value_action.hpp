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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__OVERRIDE_CONTROLLER_VALUE_ACTION_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__OVERRIDE_CONTROLLER_VALUE_ACTION_HPP_

#include <openscenario_interpreter/scope.hpp>
#include <pugixml.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- OverrideControllerValueAction ------------------------------------------
 *
 *  Overrides entity controller values. Mostly suited for motor vehicles.
 *
 *  <xsd:complexType name="OverrideControllerValueAction">
 *    <xsd:all>
 *      <xsd:element name="Throttle" type="OverrideThrottleAction"/>
 *      <xsd:element name="Brake" type="OverrideBrakeAction"/>
 *      <xsd:element name="Clutch" type="OverrideClutchAction"/>
 *      <xsd:element name="ParkingBrake" type="OverrideParkingBrakeAction"/>
 *      <xsd:element name="SteeringWheel" type="OverrideSteeringWheelAction"/>
 *      <xsd:element name="Gear" type="OverrideGearAction"/>
 *    </xsd:all>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct OverrideControllerValueAction
{
  // New value for throttle pedal position or unset value.
  // const OverrideThrottleAction overrideThrottle;

  // New value for brake position or unset value.
  // const OverrideBrakeAction overrideBrake;

  // New value for clutch position or unset value.
  // const OverrideClutchAction overrideClutch;

  // New value for parking brake position or unset value.
  // const OverrideParkingBrakeAction overrideParkingBrake;

  // New value for steering wheel position or unset value.
  // const OverrideSteeringWheelAction overrideSteeringWheel;

  // New value for gear position or unset value.
  // const OverrideGearAction overrideGear;

  OverrideControllerValueAction() = default;

  explicit OverrideControllerValueAction(const pugi::xml_node &, Scope &);
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__OVERRIDE_CONTROLLER_VALUE_ACTION_HPP_
