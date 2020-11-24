// Copyright 2015-2020 TierIV.inc. All rights reserved.
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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__STORYBOARD_ELEMENT_TYPE_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__STORYBOARD_ELEMENT_TYPE_HPP_

#include <string>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ==== StoryboardElementType ==================================================
 *
 * <xsd:simpleType name="StoryboardElementType">
 *   <xsd:union>
 *     <xsd:simpleType>
 *       <xsd:restriction base="xsd:string">
 *         <xsd:enumeration value="story"/>
 *         <xsd:enumeration value="act"/>
 *         <xsd:enumeration value="maneuver"/>
 *         <xsd:enumeration value="event"/>
 *         <xsd:enumeration value="action"/>
 *         <xsd:enumeration value="maneuverGroup"/>
 *       </xsd:restriction>
 *     </xsd:simpleType>
 *     <xsd:simpleType>
 *       <xsd:restriction base="parameter"/>
 *     </xsd:simpleType>
 *   </xsd:union>
 * </xsd:simpleType>
 *
 * ========================================================================== */
struct StoryboardElementType
{
  enum value_type
  {
    act,
    action,
    event,
    maneuver,
    maneuverGroup,
    story,
  } value;

  explicit constexpr StoryboardElementType(value_type value = {})
  : value{value}
  {}

  constexpr operator value_type() const noexcept
  {
    return value;
  }
};

template<typename ... Ts>
std::basic_istream<Ts...> & operator>>(std::basic_istream<Ts...> & is, StoryboardElementType & type)
{
  std::string buffer {};

  is >> buffer;

  #define BOILERPLATE(IDENTIFIER) \
  if (buffer == #IDENTIFIER) { \
    type.value = StoryboardElementType::IDENTIFIER; \
    return is; \
  } static_assert(true, "")

  BOILERPLATE(act);
  BOILERPLATE(action);
  BOILERPLATE(event);
  BOILERPLATE(maneuver);
  BOILERPLATE(maneuverGroup);
  BOILERPLATE(story);

  #undef BOILERPLATE

  std::stringstream ss {};
  ss << "unexpected value \'" << buffer << "\' specified as type StoryboardElementType";
  throw SyntaxError {ss.str()};
}

template<typename ... Ts>
std::basic_ostream<Ts...> & operator<<(
  std::basic_ostream<Ts...> & os,
  const StoryboardElementType & type)
{
  switch (type) {
    #define BOILERPLATE(ID) case StoryboardElementType::ID: return os << #ID;

    BOILERPLATE(act);
    BOILERPLATE(action);
    BOILERPLATE(event);
    BOILERPLATE(maneuver);
    BOILERPLATE(maneuverGroup);
    BOILERPLATE(story);

    #undef BOILERPLATE

    default:
      std::stringstream ss {};
      ss << "enum class StoryboardElementType holds unexpected value " <<
        static_cast<StoryboardElementType::value_type>(type.value);
      throw ImplementationFault {ss.str()};
  }
}
}
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__STORYBOARD_ELEMENT_TYPE_HPP_
