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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__TIMING_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__TIMING_HPP_

#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/double.hpp>
#include <openscenario_interpreter/syntax/reference_context.hpp>
#include <pugixml.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- Timing 1.2 -------------------------------------------------------------
 *
 *
 *  <xsd:complexType name="Timing">
 *    <xsd:attribute name="domainAbsoluteRelative" type="ReferenceContext" use="required"/>
 *    <xsd:attribute name="offset" type="Double" use="required"/>
 *    <xsd:attribute name="scale" type="Double" use="required"/>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct Timing
{
  const ReferenceContext domain_absolute_relative;

  const Double offset;

  const Double scale;

  explicit Timing(const pugi::xml_node &, Scope &);
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__TIMING_HPP_
