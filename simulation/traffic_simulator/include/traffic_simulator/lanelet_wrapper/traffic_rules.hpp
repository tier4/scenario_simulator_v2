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

namespace traffic_simulator
{
namespace lanelet_wrapper
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
  using lanelet::traffic_rules::GenericTrafficRules::canPass;
  using lanelet::traffic_rules::GermanVehicle::GermanVehicle;

protected:
  /// @note this function overrides and adds road shoulder handling to GenericTrafficRules::canPass
  auto canPass(const std::string & type, const std::string & /*location*/) const
    -> lanelet::Optional<bool> override
  {
    using lanelet::AttributeValueString;
    using lanelet::Participants;
    const static std::map<std::string, std::vector<std::string>, std::less<>> participants_map{
      // clang-format off
      {"",                                  {Participants::Vehicle}},
      {AttributeValueString::Road,          {Participants::Vehicle, Participants::Bicycle}},
      {"road_shoulder",                     {Participants::Vehicle, Participants::Bicycle}},  // add road_shoulder
      {AttributeValueString::Highway,       {Participants::Vehicle}},
      {AttributeValueString::BicycleLane,   {Participants::Bicycle}},
      {AttributeValueString::PlayStreet,    {Participants::Pedestrian, Participants::Bicycle, Participants::Vehicle}},
      {AttributeValueString::EmergencyLane, {Participants::VehicleEmergency}},
      {AttributeValueString::Exit,          {Participants::Pedestrian, Participants::Bicycle, Participants::Vehicle}},
      {AttributeValueString::Walkway,       {Participants::Pedestrian}},
      {AttributeValueString::Crosswalk,     {Participants::Pedestrian}},
      {AttributeValueString::Stairs,        {Participants::Pedestrian}},
      {AttributeValueString::SharedWalkway, {Participants::Pedestrian, Participants::Bicycle}}
      // clang-format on
    };
    auto participants = participants_map.find(type);
    if (participants == participants_map.end()) {
      return {};
    }

    auto startsWith = [](std::string_view str, std::string_view substr) {
      return str.compare(0, substr.size(), substr) == 0;
    };
    return lanelet::utils::anyOf(participants->second, [this, startsWith](auto & participant) {
      return startsWith(this->participant(), participant);
    });
  }

  lanelet::traffic_rules::LaneChangeType laneChangeType(
    const lanelet::ConstLineString3d &, bool) const override;
};
}  // namespace lanelet_wrapper
}  // namespace traffic_simulator
#endif  // TRAFFIC_SIMULATOR__HDMAP_UTILS__TRAFFIC_RULES_HPP_
