// Copyright 2024 TIER IV, Inc. All rights reserved.
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

#include <traffic_simulator/lanelet_map_core/traffic_lights.hpp>

namespace traffic_simulator
{
namespace lanelet_map_core
{
namespace traffic_lights
{
auto isTrafficLight(const lanelet::Id lanelet_id) -> bool
{
  if (LaneletMapCore::map()->lineStringLayer.exists(lanelet_id)) {
    if (auto && linestring = LaneletMapCore::map()->lineStringLayer.get(lanelet_id);
        linestring.hasAttribute(lanelet::AttributeName::Type)) {
      return linestring.attribute(lanelet::AttributeName::Type).value() == "traffic_light";
    }
  }
  return false;
}

auto isTrafficLightRegulatoryElement(const lanelet::Id lanelet_id) -> bool
{
  return LaneletMapCore::map()->regulatoryElementLayer.exists(lanelet_id) and
         std::dynamic_pointer_cast<lanelet::TrafficLight>(
           LaneletMapCore::map()->regulatoryElementLayer.get(lanelet_id));
}

auto getTrafficLightStopLinesPoints(const lanelet::Id traffic_light_id)
  -> std::vector<std::vector<Point>>
{
  std::vector<std::vector<Point>> ret;
  const auto traffic_lights = getTrafficLights(traffic_light_id);
  for (const auto & traffic_light : traffic_lights) {
    ret.emplace_back(std::vector<Point>{});
    const auto stop_line = traffic_light->stopLine();
    if (stop_line) {
      auto & current_stop_line = ret.back();
      for (const auto & point : stop_line.value()) {
        Point p;
        p.x = point.x();
        p.y = point.y();
        p.z = point.z();
        current_stop_line.emplace_back(p);
      }
    }
  }
  return ret;
}

auto getTrafficLights(const lanelet::Id traffic_light_id)
  -> std::vector<lanelet::AutowareTrafficLightConstPtr>
{
  std::vector<lanelet::AutowareTrafficLightConstPtr> ret;
  lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(LaneletMapCore::map());
  auto autoware_traffic_lights = lanelet::utils::query::autowareTrafficLights(all_lanelets);
  for (const auto & light : autoware_traffic_lights) {
    for (auto light_string : light->lightBulbs()) {
      if (light_string.hasAttribute("traffic_light_id")) {
        auto id = light_string.attribute("traffic_light_id").asId();
        if (id == traffic_light_id) {
          ret.emplace_back(light);
        }
      }
    }
  }
  if (ret.empty()) {
    THROW_SEMANTIC_ERROR("traffic_light_id does not match. ID : ", traffic_light_id);
  }
  return ret;
}

auto getTrafficLightBulbPosition(const lanelet::Id traffic_light_id, const std::string & color_name)
  -> std::optional<Point>
{
  lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(LaneletMapCore::map());
  auto autoware_traffic_lights = lanelet::utils::query::autowareTrafficLights(all_lanelets);

  auto areBulbsAssignedToTrafficLight = [traffic_light_id](auto red_yellow_green_bulbs) -> bool {
    return red_yellow_green_bulbs.hasAttribute("traffic_light_id") and
           red_yellow_green_bulbs.attribute("traffic_light_id").asId() and
           red_yellow_green_bulbs.attribute("traffic_light_id").asId().value() == traffic_light_id;
  };

  auto isBulbOfExpectedColor = [color_name](auto bulb) -> bool {
    return bulb.hasAttribute("color") and !bulb.hasAttribute("arrow") and
           bulb.attribute("color").value().compare(color_name) == 0;
  };

  for (const auto & light : autoware_traffic_lights) {
    for (auto three_light_bulbs : light->lightBulbs()) {
      if (areBulbsAssignedToTrafficLight(three_light_bulbs)) {
        for (auto bulb : static_cast<lanelet::ConstLineString3d>(three_light_bulbs)) {
          if (isBulbOfExpectedColor(bulb)) {
            Point point;
            point.x = bulb.x();
            point.y = bulb.y();
            point.z = bulb.z();
            return point;
          }
        }
      }
    }
  }
  return std::nullopt;
}

auto getTrafficLightRegulatoryElement(const lanelet::Id lanelet_id) -> lanelet::TrafficLight::Ptr
{
  assert(isTrafficLightRegulatoryElement(lanelet_id));
  return std::dynamic_pointer_cast<lanelet::TrafficLight>(
    LaneletMapCore::map()->regulatoryElementLayer.get(lanelet_id));
}

auto getTrafficLightRegulatoryElementIDsFromTrafficLight(const lanelet::Id traffic_light_way_id)
  -> lanelet::Ids
{
  assert(isTrafficLight(traffic_light_way_id));
  lanelet::Ids traffic_light_regulatory_element_ids;
  for (const auto & regulatory_element : LaneletMapCore::map()->regulatoryElementLayer) {
    if (regulatory_element->attribute(lanelet::AttributeName::Subtype).value() == "traffic_light") {
      for (const auto & ref_member :
           regulatory_element->getParameters<lanelet::ConstLineString3d>("refers")) {
        if (ref_member.id() == traffic_light_way_id) {
          traffic_light_regulatory_element_ids.push_back(regulatory_element->id());
        }
      }
    }
  }
  return traffic_light_regulatory_element_ids;
}

// On path
auto getTrafficLightIdsOnPath(const lanelet::Ids & route_lanelets) -> lanelet::Ids
{
  lanelet::Ids ids;
  for (const auto & traffic_light : getTrafficLightRegulatoryElementsOnPath(route_lanelets)) {
    for (auto light_string : traffic_light->lightBulbs()) {
      if (light_string.hasAttribute("traffic_light_id")) {
        if (auto id = light_string.attribute("traffic_light_id").asId(); id) {
          ids.push_back(id.value());
        }
      }
    }
  }
  return ids;
}

auto getTrafficSignRegulatoryElementsOnPath(const lanelet::Ids & lanelet_ids)
  -> std::vector<std::shared_ptr<const lanelet::TrafficSign>>
{
  std::vector<std::shared_ptr<const lanelet::TrafficSign>> ret;
  for (const auto & lanelet_id : lanelet_ids) {
    const auto lanelet = LaneletMapCore::map()->laneletLayer.get(lanelet_id);
    const auto traffic_signs = lanelet.regulatoryElementsAs<const lanelet::TrafficSign>();
    for (const auto & traffic_sign : traffic_signs) {
      ret.emplace_back(traffic_sign);
    }
  }
  return ret;
}
}  // namespace traffic_lights
}  // namespace lanelet_map_core
}  // namespace traffic_simulator
