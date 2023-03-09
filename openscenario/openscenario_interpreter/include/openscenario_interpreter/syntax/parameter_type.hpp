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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__PARAMETER_TYPE_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__PARAMETER_TYPE_HPP_

#include <iostream>
#include <openscenario_interpreter/syntax/boolean.hpp>
#include <openscenario_interpreter/syntax/double.hpp>
#include <openscenario_interpreter/syntax/integer.hpp>
#include <openscenario_interpreter/syntax/string.hpp>
#include <openscenario_interpreter/syntax/unsigned_integer.hpp>
#include <openscenario_interpreter/syntax/unsigned_short.hpp>
#include <openscenario_interpreter/utility/variant.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- ParameterType ----------------------------------------------------------
 *
 *  <xsd:simpleType name="ParameterType">
 *    <xsd:union>
 *      <xsd:simpleType>
 *        <xsd:restriction base="xsd:string">
 *          <xsd:enumeration value="integer"/>
 *          <xsd:enumeration value="double"/>
 *          <xsd:enumeration value="string"/>
 *          <xsd:enumeration value="unsignedInt"/>
 *          <xsd:enumeration value="unsignedShort"/>
 *          <xsd:enumeration value="boolean"/>
 *          <xsd:enumeration value="dateTime"/>
 *        </xsd:restriction>
 *      </xsd:simpleType>
 *      <xsd:simpleType>
 *        <xsd:restriction base="parameter"/>
 *      </xsd:simpleType>
 *    </xsd:union>
 *  </xsd:simpleType>
 *
 * -------------------------------------------------------------------------- */
struct ParameterType
{
  enum value_type {
    BOOLEAN,
    DATE_TIME,
    DOUBLE,
    INTEGER,
    STRING,
    UNSIGNED_INT,
    UNSIGNED_SHORT,
  } value;

  explicit ParameterType() = default;

  constexpr operator value_type() const noexcept { return value; }
};

auto operator>>(std::istream &, ParameterType &) -> std::istream &;

auto operator<<(std::ostream &, const ParameterType &) -> std::ostream &;
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__PARAMETER_TYPE_HPP_
