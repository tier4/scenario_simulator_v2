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

#include <boost/algorithm/string.hpp>
#include <openscenario_interpreter/reader/attribute.hpp>
#include <openscenario_interpreter/simulator_core.hpp>
#include <openscenario_interpreter/syntax/traffic_signal_state.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
TrafficSignalState::TrafficSignalType::TrafficSignalType(const std::string & string)
: value([&]() {
    if (string == "conventional") {
      return conventional;
    } else if (string == "v2i") {
      return v2i;
    } else {
      throw Error(
        "TrafficSignalState: Invalid traffic signal type '", string,
        "'. Valid values are 'conventional' or 'v2i'.");
    }
  }())
{
}

TrafficSignalState::TrafficSignalState(const pugi::xml_node & node, Scope & scope)
: traffic_signal_id(readAttribute<String>("trafficSignalId", node, scope)),
  state(readAttribute<String>("state", node, scope)),
  parsed_traffic_signal_id(parseTrafficSignalId(traffic_signal_id))
{
}

auto TrafficSignalState::evaluate() const -> Object
{
  switch (trafficSignalType()) {
    case TrafficSignalType::conventional:
      addConventionalTrafficLightsState(id(), state);
      break;
    case TrafficSignalType::v2i:
      addV2ITrafficLightsState(id(), state);
      break;
    default:
      throw Error("Unknown traffic signal type has set to TrafficSignalState");
  }
  return unspecified;
}

auto TrafficSignalState::parseTrafficSignalId(const std::string & traffic_signal_id)
  -> std::pair<lanelet::Id, TrafficSignalType>
{
  std::vector<std::string> parts;
  boost::split(parts, traffic_signal_id, boost::is_space(), boost::token_compress_on);

  lanelet::Id id;
  try {
    id = boost::lexical_cast<lanelet::Id>(parts[0]);
  } catch (const boost::bad_lexical_cast &) {
    throw Error(
      "TrafficSignalState: Invalid traffic signal ID '", parts[0],
      "'. Expected a numeric lanelet ID.");
  }

  TrafficSignalType type = [&]() {
    switch (parts.size()) {
      case 1:
        return TrafficSignalType(TrafficSignalType::conventional);
      case 2:
        return TrafficSignalType(parts[1]);
      default:
        throw Error(
          "TrafficSignalState: trafficSignalId '", traffic_signal_id,
          "' has invalid format. Expected format: '<id>' or '<id> <type>'.");
    }
  }();

  return {id, type};
}
}  // namespace syntax
}  // namespace openscenario_interpreter
