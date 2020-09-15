// Copyright 2015-2020 Autoware Foundation. All rights reserved.
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

#ifndef SCENARIO_RUNNER__SYNTAX__PRIORITY_HPP_
#define SCENARIO_RUNNER__SYNTAX__PRIORITY_HPP_

#include <scenario_runner/object.hpp>

namespace scenario_runner
{inline namespace syntax
{
/* ==== Priority =============================================================
 *
 * <xsd:simpleType name="Priority">
 *   <xsd:union>
 *     <xsd:simpleType>
 *       <xsd:restriction base="xsd:string">
 *         <xsd:enumeration value="overwrite"/>
 *         <xsd:enumeration value="skip"/>
 *         <xsd:enumeration value="parallel"/>
 *       </xsd:restriction>
 *     </xsd:simpleType>
 *     <xsd:simpleType>
 *       <xsd:restriction base="parameter"/>
 *     </xsd:simpleType>
 *   </xsd:union>
 * </xsd:simpleType>
 *
 * ======================================================================== */
struct Priority
{
  enum value_type
  {
    // If a starting event has priority Overwrite, all events in running state,
    // within the same scope (maneuver) as the starting event, should be issued
    // a stop command (stop transition).
    overwrite,

    // If a starting event has priority Skip, then it will not be ran if there
    // is any other event in the same scope (maneuver) in the running state.
    skip,

    // Execute in parallel to other events.
    parallel,
  } value;

  explicit Priority() = default;

  explicit Priority(value_type value)
  : value{value}
  {}

  operator value_type() const noexcept
  {
    return value;
  }
};

template<typename ... Ts>
std::basic_istream<Ts...> & operator>>(std::basic_istream<Ts...> & is, Priority & priority)
{
  std::string buffer {};

  is >> buffer;

    #define SUPPORTED(IDENTIFIER) \
  if (buffer == #IDENTIFIER) do \
    { \
      priority.value = Priority::IDENTIFIER; \
      return is; \
    } while (false)

  SUPPORTED(overwrite);

    #undef SUPPORTED

    #define UNSUPPORTED(IDENTIFIER) \
  if (buffer == #IDENTIFIER) do \
    { \
      std::stringstream ss {}; \
      ss << "given value \'" << buffer << \
        "\' is valid OpenSCENARIO value of type Priority, but it is not supported"; \
      throw ImplementationFault {ss.str()}; \
    } while (false)

  UNSUPPORTED(skip);
  UNSUPPORTED(parallel);

    #undef UNSUPPORTED

  std::stringstream ss {};
  ss << "unexpected value \'" << buffer << "\' specified as type Priority";
  throw SyntaxError {ss.str()};
}

template<typename ... Ts>
std::basic_ostream<Ts...> & operator<<(std::basic_ostream<Ts...> & os, const Priority & priority)
{
  switch (priority) {
      #define BOILERPLATE(NAME) case Priority::NAME: return os << #NAME;

    BOILERPLATE(overwrite);
    BOILERPLATE(skip);
    BOILERPLATE(parallel);

      #undef BOILERPLATE

    default:
      std::stringstream ss {};
      ss << "enum class Priority holds unexpected value " <<
        static_cast<Priority::value_type>(priority);
      throw ImplementationFault {ss.str()};
  }
}
}}  // namespace scenario_runner::syntax


#endif  // SCENARIO_RUNNER__SYNTAX__PRIORITY_HPP_
