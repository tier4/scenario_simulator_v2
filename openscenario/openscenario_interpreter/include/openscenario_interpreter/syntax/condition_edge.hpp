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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__CONDITION_EDGE_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__CONDITION_EDGE_HPP_

#include <openscenario_interpreter/reader/attribute.hpp>
#include <string>

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
 *  Tier IV Extension:
 *    - Add enumuration <xsd.enumeration value="sticky"/>
 *
 * -------------------------------------------------------------------------- */
struct ConditionEdge
{
  enum value_type {
    /* ---- Rising -------------------------------------------------------------
     *
     *  A condition defined with a rising edge shall return true at discrete
     *  time t if its logical expression is true at discrete time t and its
     *  logical expression was false at discrete time t-ts, where ts is the
     *  simulation sampling time.
     *
     * ---------------------------------------------------------------------- */
    rising,

    /* ---- Falling ------------------------------------------------------------
     *
     *  A condition defined with a falling edge shall return true at discrete
     *  time t if its logical expression is false at discrete time t and its
     *  logical expression was true at discrete time t-ts, where ts is the
     *  simulation sampling time.
     *
     * ---------------------------------------------------------------------- */
    falling,

    /* ---- Rising or Falling --------------------------------------------------
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

    /* ---- None ---------------------------------------------------------------
     *
     *  A condition defined with a 'none' edge shall return true at discrete
     *  time t if its logical expression is true at discrete time t.
     *
     * ---------------------------------------------------------------------- */
    none,

    /* ---- Sticky (Tier IV Extension) -----------------------------------------
     *
     *  A condition defined by a 'sticky' edge returns true at discrete time
     *  t + k (0 < k) if its logical expression evaluates to true at discrete
     *  time t. This edge is provided for simply defining assertions such as
     *  "Did the Ego car pass over checkpoint X?
     *
     *  This edge is a non-OpenSCEANRIO 1.0.0 standard feature.
     *
     * ---------------------------------------------------------------------- */
    sticky,
  } value;

  explicit constexpr ConditionEdge(const value_type value = {}) : value(value) {}

  operator value_type() const noexcept { return value; }
};

std::istream & operator>>(std::istream & is, ConditionEdge & edge)
{
  std::string buffer{};

  is >> buffer;

#define BOILERPLATE(IDENTIFIER)             \
  if (buffer == #IDENTIFIER) {              \
    edge.value = ConditionEdge::IDENTIFIER; \
    return is;                              \
  }                                         \
  static_assert(true, "")

  BOILERPLATE(rising);
  BOILERPLATE(falling);
  BOILERPLATE(risingOrFalling);
  BOILERPLATE(none);
  BOILERPLATE(sticky);

#undef BOILERPLATE

  std::stringstream ss{};
  ss << "unexpected value \'" << buffer << "\' specified as type ConditionEdge";
  throw SyntaxError(ss.str());
}

std::ostream & operator<<(std::ostream & os, const ConditionEdge & edge)
{
  switch (edge) {
#define BOILERPLATE(ID)   \
  case ConditionEdge::ID: \
    return os << #ID;

    BOILERPLATE(rising);
    BOILERPLATE(falling);
    BOILERPLATE(risingOrFalling);
    BOILERPLATE(none);
    BOILERPLATE(sticky);

#undef BOILERPLATE

    default:
      std::stringstream ss{};
      ss << "enum class ConditionEdge holds unexpected value "
         << static_cast<ConditionEdge::value_type>(edge.value);
      throw ImplementationFault(ss.str());
  }
}
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__CONDITION_EDGE_HPP_
