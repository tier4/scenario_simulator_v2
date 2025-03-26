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

#include <openscenario_interpreter/reader/element.hpp>
#include <openscenario_interpreter/syntax/by_value_condition.hpp>
#include <openscenario_interpreter/syntax/parameter_condition.hpp>
#include <openscenario_interpreter/syntax/simulation_time_condition.hpp>
#include <openscenario_interpreter/syntax/storyboard_element_state_condition.hpp>
#include <openscenario_interpreter/syntax/traffic_signal_condition.hpp>
#include <openscenario_interpreter/syntax/traffic_signal_controller_condition.hpp>
#include <openscenario_interpreter/syntax/user_defined_value_condition.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
ByValueCondition::ByValueCondition(const pugi::xml_node & node, Scope & scope)
// clang-format off
: ComplexType(
    choice(node,
      std::make_pair(              "ParameterCondition", [&](const auto & node) { return make<              ParameterCondition>(node, scope); }),
      std::make_pair(              "TimeOfDayCondition", [&](const auto & node) { throw UNSUPPORTED_ELEMENT_SPECIFIED(node.name()); return unspecified; }),
      std::make_pair(         "SimulationTimeCondition", [&](const auto & node) { return make<         SimulationTimeCondition>(node, scope); }),
      std::make_pair( "StoryboardElementStateCondition", [&](const auto & node) { return make< StoryboardElementStateCondition>(node, scope); }),
      std::make_pair(       "UserDefinedValueCondition", [&](const auto & node) { return make<       UserDefinedValueCondition>(node, scope); }),
      std::make_pair(          "TrafficSignalCondition", [&](const auto & node) { return make<          TrafficSignalCondition>(node, scope); }),
      std::make_pair("TrafficSignalControllerCondition", [&](const auto & node) { return make<TrafficSignalControllerCondition>(node, scope); })))
// clang-format on
{
}
}  // namespace syntax
}  // namespace openscenario_interpreter
