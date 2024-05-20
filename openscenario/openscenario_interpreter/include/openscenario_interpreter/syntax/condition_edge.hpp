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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__CONDITION_EDGE_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__CONDITION_EDGE_HPP_

#include <iostream>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- ConditionEdge ----------------------------------------------------------
 *
 *  <xsd:simpleType name="ConditionEdge">
 *    <xsd:union>
 *      <xsd:simpleType>
 *        <xsd:restriction base="xsd:string">
 *          <xsd:enumeration value="rising"/>
 *          <xsd:enumeration value="falling"/>
 *          <xsd:enumeration value="risingOrFalling"/>
 *          <xsd:enumeration value="none"/>
 *        </xsd:restriction>
 *      </xsd:simpleType>
 *      <xsd:simpleType>
 *        <xsd:restriction base="parameter"/>
 *      </xsd:simpleType>
 *    </xsd:union>
 *  </xsd:simpleType>
 *
 *  TIER IV Extension:
 *    - Add enumeration <xsd.enumeration value="sticky"/>
 *
 * -------------------------------------------------------------------------- */
struct ConditionEdge
{
  enum value_type {
    /* ---- NOTE ---------------------------------------------------------------
     *
     *  A condition defined with a 'none' edge shall return true at discrete
     *  time t if its logical expression is true at discrete time t.
     *
     *  Default constructor select this.
     *
     * ---------------------------------------------------------------------- */
    none,

    /* ---- NOTE ---------------------------------------------------------------
     *
     *  A condition defined with a rising edge shall return true at discrete
     *  time t if its logical expression is true at discrete time t and its
     *  logical expression was false at discrete time t-ts, where ts is the
     *  simulation sampling time.
     *
     * ---------------------------------------------------------------------- */
    rising,

    /* ---- NOTE ---------------------------------------------------------------
     *
     *  A condition defined with a falling edge shall return true at discrete
     *  time t if its logical expression is false at discrete time t and its
     *  logical expression was true at discrete time t-ts, where ts is the
     *  simulation sampling time.
     *
     * ---------------------------------------------------------------------- */
    falling,

    /* ---- NOTE ---------------------------------------------------------------
     *
     *  A condition defined with a 'risingOrFalling' edge shall return true at
     *  discrete time t if its logical expression is true at discrete time t
     *  and its logical expression was false at discrete time t-ts OR if its
     *  logical expression is false at discrete time t and its logical
     *  expression was true at discrete time t-ts. ts is the simulation
     *  sampling time.
     *
     * ---------------------------------------------------------------------- */
    risingOrFalling,

    /* ---- NOTE ---------------------------------------------------------------
     *
     *  A condition defined by a 'sticky' edge returns true at discrete time
     *  t + k (0 < k) if its logical expression evaluates to true at discrete
     *  time t. This edge is provided for simply defining assertions such as
     *  "Did the Ego car pass over checkpoint X?
     *
     *  This is NOT an OpenSCENARIO 1.0.0 standard feature (TIER IV extension).
     *
     * ---------------------------------------------------------------------- */
    sticky,
  } value;

  ConditionEdge() = default;

  operator value_type() const noexcept { return value; }
};

auto operator>>(std::istream & is, ConditionEdge &) -> std::istream &;

auto operator<<(std::ostream & os, const ConditionEdge &) -> std::ostream &;
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__CONDITION_EDGE_HPP_
