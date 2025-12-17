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
#include <regex>

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
  if (not isDetected() and state.find("unknown") != std::string::npos) {
    throw Error(
      "TrafficSignalState: The state '", state,
      "' contains 'unknown' which is not allowed for ground truth traffic lights. "
      "'unknown' is reserved for traffic light perception simulation.");
  }
}

auto TrafficSignalState::clear() const -> void
{
  switch (trafficSignalType()) {
    case TrafficSignalType::conventional:
      if (isDetected()) {
        clearConventionalDetectedTrafficLightsState(id());
      } else {
        clearConventionalTrafficLightsState(id());
      }
      break;
    case TrafficSignalType::v2i:
      if (isDetected()) {
        clearV2IDetectedTrafficLightsState(id());
      } else {
        clearV2ITrafficLightsState(id());
      }
      break;
    default:
      throw Error("Unknown traffic signal type has set to TrafficSignalState");
  }
}

auto TrafficSignalState::evaluate() const -> Object
{
  switch (trafficSignalType()) {
    case TrafficSignalType::conventional:
      if (isDetected()) {
        // addConventionalDetectedTrafficLightsState(id(), state);
      } else {
        addConventionalTrafficLightsState(id(), state);
      }
      break;
    case TrafficSignalType::v2i:
      if (isDetected()) {
        // addV2IDetectedTrafficLightsState(id(), state);
      } else {
        addV2ITrafficLightsState(id(), state);
      }
      break;
    default:
      throw Error("Unknown traffic signal type has set to TrafficSignalState");
  }
  return unspecified;
}

auto TrafficSignalState::parseTrafficSignalId(const std::string & traffic_signal_id)
  -> ParsedTrafficSignalId
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

  auto [type, detected] = [&]() -> std::pair<TrafficSignalType, bool> {
    switch (parts.size()) {
      case 1:
        return {TrafficSignalType(TrafficSignalType::conventional), false};
      case 2: {
        const std::string & type_str = parts[1];
        static const std::regex pattern(R"(^(conventional|v2i)(_detected)?$)");
        std::smatch match;
        if (std::regex_match(type_str, match, pattern)) {
          bool detected = match[2].matched;
          return {TrafficSignalType(match[1].str()), detected};
        } else {
          throw Error(
            "TrafficSignalState: Invalid traffic signal type '", type_str,
            "'. Expected 'conventional', 'conventional_detected', 'v2i', or 'v2i_detected'.");
        }
      }
      default:
        throw Error(
          "TrafficSignalState: trafficSignalId '", traffic_signal_id,
          "' has invalid format. Expected format: '<id>' or '<id> <type>'.");
    }
  }();

  return {id, type, detected};
}
}  // namespace syntax
}  // namespace openscenario_interpreter
