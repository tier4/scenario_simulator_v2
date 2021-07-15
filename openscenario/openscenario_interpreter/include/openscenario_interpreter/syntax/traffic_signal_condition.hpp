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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__TRAFFIC_SIGNAL_CONDITION_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__TRAFFIC_SIGNAL_CONDITION_HPP_

#include <openscenario_interpreter/reader/attribute.hpp>
#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/string.hpp>
#include <openscenario_interpreter/syntax/traffic_signal_state.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- TrafficSignalCondition -------------------------------------------------
 *
 *  Considered true if a referenced traffic signal (e.g. from an OpenDRIVE
 *  file) reaches a specific states. Signal IDs are listed in the TrafficSignal
 *  list of the RoadNetwork together with their states and their controllers to
 *  enable dynamic signal modelling.
 *
 *  <xsd:complexType name="TrafficSignalCondition">
 *    <xsd:attribute name="name" type="String" use="required"/>
 *    <xsd:attribute name="state" type="String" use="required"/>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct TrafficSignalCondition : private Scope
{
  const String name;

  const String state;

  using LaneletId = TrafficSignalState::LaneletId;

  template <typename Node, typename Scope>
  explicit TrafficSignalCondition(const Node & node, Scope & scope)
  : Scope(scope),
    name(readAttribute<String>("name", node, localScope())),
    state(readAttribute<String>("state", node, localScope()))
  {
  }

  Arrow current_arrow;

  Color current_color;

  auto evaluate()
  {
    current_arrow = static_cast<Arrow>(getTrafficSignalArrow(boost::lexical_cast<LaneletId>(name)));
    current_color = static_cast<Color>(getTrafficSignalColor(boost::lexical_cast<LaneletId>(name)));

    if (state == "none") {
      return asBoolean(current_arrow == Arrow::none and current_color == Color::none);
    } else {
      return asBoolean(
        boost::lexical_cast<String>(current_arrow) == state or
        boost::lexical_cast<String>(current_color) == state);
    }
  }

  auto description() const
  {
    std::stringstream description;

    description << "Is TrafficSignal " << std::quoted(name) << " (Arrow = " << current_arrow
                << ", Color = " << current_color << ") in state " << std::quoted(state) << "?";

    return description.str();
  }
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__TRAFFIC_SIGNAL_CONDITION_HPP_
