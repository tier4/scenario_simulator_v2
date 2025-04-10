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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__DIRECTIONAL_DIMENSION_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__DIRECTIONAL_DIMENSION_HPP_

namespace openscenario_interpreter
{
inline namespace syntax
{
/*
   DirectionalDimension (OpenSCENARIO XML 1.3)

   Defines the directions in the entity coordinate system.

   <xsd:simpleType name="DirectionalDimension">
     <xsd:union>
       <xsd:simpleType>
         <xsd:restriction base="xsd:string">
           <xsd:enumeration value="longitudinal"/>
           <xsd:enumeration value="lateral"/>
           <xsd:enumeration value="vertical"/>
         </xsd:restriction>
       </xsd:simpleType>
       <xsd:simpleType>
         <xsd:restriction base="parameter"/>
       </xsd:simpleType>
     </xsd:union>
   </xsd:simpleType>
*/
struct DirectionalDimension
{
  enum value_type {
    /*
       Longitudinal direction (along the x-axis).
    */
    longitudinal,  // NOTE: DEFAULT VALUE

    /*
       Lateral direction (along the y-axis).
    */
    lateral,

    /*
       Vertical direction (along the z-axis).
    */
    vertical,
  };

  value_type value;

  DirectionalDimension() = default;

  constexpr DirectionalDimension(value_type value) : value(value) {}

  constexpr operator value_type() const noexcept { return value; }

  friend auto operator>>(std::istream & istream, DirectionalDimension & datum) -> std::istream &
  {
    if (auto token = std::string(); istream >> token) {
      if (token == "longitudinal") {
        datum.value = DirectionalDimension::longitudinal;
        return istream;
      } else if (token == "lateral") {
        datum.value = DirectionalDimension::lateral;
        return istream;
      } else if (token == "vertical") {
        datum.value = DirectionalDimension::vertical;
        return istream;
      } else {
        throw UNEXPECTED_ENUMERATION_VALUE_SPECIFIED(DirectionalDimension, token);
      }
    } else {
      datum.value = DirectionalDimension::value_type();
      return istream;
    }
  }

  friend auto operator<<(std::ostream & ostream, const DirectionalDimension & datum)
    -> std::ostream &
  {
    switch (datum) {
      case DirectionalDimension::longitudinal:
        return ostream << "longitudinal";
      case DirectionalDimension::lateral:
        return ostream << "lateral";
      case DirectionalDimension::vertical:
        return ostream << "vertical";
      default:
        throw UNEXPECTED_ENUMERATION_VALUE_ASSIGNED(DirectionalDimension, datum);
    }
  }
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__DIRECTIONAL_DIMENSION_HPP_
