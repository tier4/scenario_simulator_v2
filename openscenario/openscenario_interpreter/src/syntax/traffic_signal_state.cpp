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
TrafficSignalState::TrafficSignalChannelType::TrafficSignalChannelType(const std::string & string)
: value([&] {
    if (string == "conventional") {
      return conventional;
    } else if (string == "v2i") {
      return v2i;
    } else {
      throw Error(
        "TrafficSignalState: Invalid traffic signal channel type '", string,
        "'. Valid values are 'conventional' or 'v2i'.");
    }
  }())
{
}

TrafficSignalState::TrafficSignalState(const pugi::xml_node & node, Scope & scope)
: traffic_signal_id(readAttribute<String>("trafficSignalId", node, scope)),
  state(readAttribute<String>("state", node, scope)),
  target(traffic_signal_id)
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
  switch (channelType()) {
    case TrafficSignalChannelType::conventional:
      if (isDetected()) {
        clearConventionalDetectedTrafficLightsState(id());
      } else {
        clearConventionalTrafficLightsState(id());
      }
      break;
    case TrafficSignalChannelType::v2i:
      if (isDetected()) {
        clearV2IDetectedTrafficLightsState(id());
      } else {
        clearV2ITrafficLightsState(id());
      }
      break;
    default:
      throw Error("Unknown traffic signal channel type has set to TrafficSignalState");
  }
}

auto TrafficSignalState::evaluate() const -> Object
{
  switch (channelType()) {
    case TrafficSignalChannelType::conventional:
      if (isDetected()) {
        addConventionalDetectedTrafficLightsState(id(), state);
      } else {
        addConventionalTrafficLightsState(id(), state);
      }
      break;
    case TrafficSignalChannelType::v2i:
      if (isDetected()) {
        addV2IDetectedTrafficLightsState(id(), state);
      } else {
        addV2ITrafficLightsState(id(), state);
      }
      break;
    default:
      throw Error("Unknown traffic signal channel type has set to TrafficSignalState");
  }
  return unspecified;
}

TrafficSignalState::TargetTrafficSignalChannel::TargetTrafficSignalChannel(
  const std::string & traffic_signal_id)
: channel(TrafficSignalChannelType::conventional), detected(false)
{
  std::vector<std::string> parts;
  boost::split(parts, traffic_signal_id, boost::is_space(), boost::token_compress_on);

  try {
    id = boost::lexical_cast<lanelet::Id>(parts[0]);
  } catch (const boost::bad_lexical_cast &) {
    throw Error(
      "TrafficSignalState: Invalid traffic signal ID '", parts[0],
      "'. Expected a numeric lanelet ID.");
  }

  if (parts.size() == 1) {
    channel = TrafficSignalChannelType::conventional;
    detected = false;
  } else if (parts.size() == 2) {
    static const std::regex pattern(R"(^(conventional|v2i)(_detected)?$)");
    std::smatch match;
    if (std::regex_match(parts[1], match, pattern)) {
      channel = TrafficSignalChannelType(match[1].str());
      detected = match[2].matched;
    } else {
      throw Error(
        "TrafficSignalState: Invalid traffic signal channel type '", parts[1],
        "'. Expected 'conventional', 'conventional_detected', 'v2i', or 'v2i_detected'.");
    }
  } else {
    throw Error(
      "TrafficSignalState: trafficSignalId '", traffic_signal_id,
      "' has invalid format. Expected format: '<id>' or '<id> <channel_type>'.");
  }
}
}  // namespace syntax
}  // namespace openscenario_interpreter
