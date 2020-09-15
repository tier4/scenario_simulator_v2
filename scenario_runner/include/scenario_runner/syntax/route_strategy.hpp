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

#ifndef SCENARIO_RUNNER__SYNTAX__ROUTE_STRATEGY_HPP_
#define SCENARIO_RUNNER__SYNTAX__ROUTE_STRATEGY_HPP_

#include <scenario_runner/object.hpp>

namespace scenario_runner
{inline namespace syntax
{
/* ==== RouteStrategy ========================================================
 *
 * <xsd:simpleType name="RouteStrategy">
 *   <xsd:union>
 *     <xsd:simpleType>
 *       <xsd:restriction base="xsd:string">
 *         <xsd:enumeration value="fastest"/>
 *         <xsd:enumeration value="shortest"/>
 *         <xsd:enumeration value="leastIntersections"/>
 *         <xsd:enumeration value="random"/>
 *       </xsd:restriction>
 *     </xsd:simpleType>
 *     <xsd:simpleType>
 *       <xsd:restriction base="parameter"/>
 *     </xsd:simpleType>
 *   </xsd:union>
 * </xsd:simpleType>
 *
 * ======================================================================== */
enum class RouteStrategy
{
  // Fastest route.
  fastest,

  // Shortest route.
  shortest,

  // Route with least number of intersections.
  leastIntersections,

  // Random route.
  random,
};

template<typename ... Ts>
std::basic_istream<Ts...> & operator>>(std::basic_istream<Ts...> & is, RouteStrategy & strategy)
{
  std::string buffer {};

  is >> buffer;

    #define SUPPORTED(IDENTIFIER) \
  if (buffer == #IDENTIFIER) do \
    { \
      strategy = RouteStrategy::IDENTIFIER; \
      return is; \
    } while (false)

  SUPPORTED(shortest);

    #undef SUPPORTED

    #define UNSUPPORTED(IDENTIFIER) \
  if (buffer == #IDENTIFIER) do \
    { \
      std::stringstream ss {}; \
      ss << "given value \'" << buffer << \
        "\' is valid OpenSCENARIO value of type RouteStrategy, but it is not supported"; \
      throw ImplementationFault {ss.str()}; \
    } while (false)

  UNSUPPORTED(fastest);
  UNSUPPORTED(leastIntersections);
  UNSUPPORTED(random);

    #undef UNSUPPORTED

  std::stringstream ss {};
  ss << "unexpected value \'" << buffer << "\' specified as type RouteStrategy";
  throw SyntaxError {ss.str()};
}

template<typename ... Ts>
std::basic_ostream<Ts...> & operator<<(
  std::basic_ostream<Ts...> & os,
  const RouteStrategy & strategy)
{
  switch (strategy) {
      #define BOILERPLATE(NAME) case RouteStrategy::NAME: return os << #NAME;

    BOILERPLATE(fastest);
    BOILERPLATE(shortest);
    BOILERPLATE(leastIntersections);
    BOILERPLATE(random);

      #undef BOILERPLATE

    default:
      std::stringstream ss {};
      ss << "enum class RouteStrategy holds unexpected value " <<
        static_cast<std::underlying_type<RouteStrategy>::type>(strategy);
      throw ImplementationFault {ss.str()};
  }
}
}}  // namespace scenario_runner::syntax

#endif  // SCENARIO_RUNNER__SYNTAX__ROUTE_STRATEGY_HPP_
