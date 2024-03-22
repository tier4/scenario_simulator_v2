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

#include <openscenario_interpreter/error.hpp>
#include <openscenario_interpreter/syntax/routing_algorithm.hpp>
#include <string>
#include <type_traits>

namespace openscenario_interpreter
{
inline namespace syntax
{
static_assert(std::is_standard_layout<RoutingAlgorithm>::value, "");

static_assert(std::is_trivial<RoutingAlgorithm>::value, "");

auto operator>>(std::istream & is, RoutingAlgorithm & datum) -> std::istream &
{
  if (std::string token; is >> token) {
    if (token == "assignedRoute") {
      datum.value = RoutingAlgorithm::assigned_route;
      return is;
    } else if (token == "fastest") {
      datum.value = RoutingAlgorithm::fastest;
      return is;
    } else if (token == "leastIntersections") {
      datum.value = RoutingAlgorithm::least_intersections;
      return is;
    } else if (token == "shortest") {
      datum.value = RoutingAlgorithm::shortest;
      return is;
    } else if (token == "undefined") {
      datum.value = RoutingAlgorithm::undefined;
      return is;
    } else {
      throw UNEXPECTED_ENUMERATION_VALUE_SPECIFIED(RoutingAlgorithm, token);
    }
  } else {
    datum.value = RoutingAlgorithm::undefined;
    return is;
  }
}

auto operator<<(std::ostream & os, const RoutingAlgorithm & datum) -> std::ostream &
{
  switch (datum) {
    case RoutingAlgorithm::assigned_route:
      return os << "assignedRoute";
    case RoutingAlgorithm::fastest:
      return os << "fastest";
    case RoutingAlgorithm::least_intersections:
      return os << "leastIntersections";
    case RoutingAlgorithm::shortest:
      return os << "shortest";
    case RoutingAlgorithm::undefined:
      return os << "undefined";
    default:
      throw UNEXPECTED_ENUMERATION_VALUE_ASSIGNED(RoutingAlgorithm, datum);
  }
}
}  // namespace syntax
}  // namespace openscenario_interpreter
