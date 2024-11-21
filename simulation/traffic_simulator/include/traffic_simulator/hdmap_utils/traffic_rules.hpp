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

#ifndef TRAFFIC_SIMULATOR__HDMAP_UTILS__TRAFFIC_RULES_HPP_
#define TRAFFIC_SIMULATOR__HDMAP_UTILS__TRAFFIC_RULES_HPP_

#include <lanelet2_traffic_rules/GermanTrafficRules.h>

namespace hdmap_utils
{
struct Locations
{
  /// @note DIRTY HACK!! Originally, a location code should be an ISO country code.
  /// @sa https://github.com/fzi-forschungszentrum-informatik/Lanelet2/blob/507033b82d9915f086f2539d56c3b62e71802438/lanelet2_traffic_rules/include/lanelet2_traffic_rules/TrafficRules.h#L97
  static constexpr char RoadShoulderPassableGermany[] = "de_road_shoulder_passable";
};

class GermanRoadShoulderPassableVehicle : public lanelet::traffic_rules::GermanVehicle
{
public:
  using lanelet::traffic_rules::GermanVehicle::GermanVehicle;

protected:
  lanelet::Optional<bool> canPass(
    const std::string & type, const std::string & /*location*/) const override
  {
    using ParticipantsMap = std::map<std::string, std::vector<std::string>>;
    using Value = lanelet::AttributeValueString;
    using Participants = lanelet::Participants;
    const static ParticipantsMap ParticipantMap{
      // clang-format off
      {"",                    {Participants::Vehicle}},
      {Value::Road,           {Participants::Vehicle, Participants::Bicycle}},
      {"road_shoulder",       {Participants::Vehicle, Participants::Bicycle}},  // add road_shoulder
      {Value::Highway,        {Participants::Vehicle}},
      {Value::BicycleLane,    {Participants::Bicycle}},
      {Value::PlayStreet,     {Participants::Pedestrian, Participants::Bicycle, Participants::Vehicle}},
      {Value::EmergencyLane,  {Participants::VehicleEmergency}},
      {Value::Exit,           {Participants::Pedestrian, Participants::Bicycle, Participants::Vehicle}},
      {Value::Walkway,        {Participants::Pedestrian}},
      {Value::Crosswalk,      {Participants::Pedestrian}},
      {Value::Stairs,         {Participants::Pedestrian}},
      {Value::SharedWalkway,  {Participants::Pedestrian, Participants::Bicycle}}
      // clang-format on
    };
    auto participants = ParticipantMap.find(type);
    if (participants == ParticipantMap.end()) {
      return {};
    }

    auto startswith = [](const std::string & str, const std::string & substr) {
      return str.compare(0, substr.size(), substr) == 0;
    };
    return lanelet::utils::anyOf(participants->second, [this, startswith](auto & participant) {
      return startswith(this->participant(), participant);
    });
  }
};
}  // namespace hdmap_utils
#endif  // TRAFFIC_SIMULATOR__HDMAP_UTILS__TRAFFIC_RULES_HPP_
