// Copyright 2015 Tier IV, Inc. All rights reserved.
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

#include <traffic_simulator/lanelet_wrapper/traffic_lights.hpp>

namespace traffic_simulator
{
namespace lanelet_wrapper
{
namespace traffic_lights
{
auto isTrafficLight(const lanelet::Id lanelet_id) -> bool
{
  if (LaneletWrapper::map()->lineStringLayer.exists(lanelet_id)) {
    if (const auto && linestring = LaneletWrapper::map()->lineStringLayer.get(lanelet_id);
        linestring.hasAttribute(lanelet::AttributeName::Type)) {
      return linestring.attribute(lanelet::AttributeName::Type).value() == "traffic_light";
    }
  }
  return false;
}

auto isTrafficLightRegulatoryElement(const lanelet::Id lanelet_id) -> bool
{
  return LaneletWrapper::map()->regulatoryElementLayer.exists(lanelet_id) &&
         std::dynamic_pointer_cast<lanelet::TrafficLight>(
           LaneletWrapper::map()->regulatoryElementLayer.get(lanelet_id));
}

auto trafficLightIds() -> lanelet::Ids
{
  lanelet::Ids ids;
  const auto & all_lanelets = lanelet::utils::query::laneletLayer(LaneletWrapper::map());
  for (const auto & autoware_traffic_light :
       lanelet::utils::query::autowareTrafficLights(all_lanelets)) {
    for (auto & three_light_bulbs : autoware_traffic_light->lightBulbs()) {
      if (three_light_bulbs.hasAttribute("traffic_light_id")) {
        if (const auto & id = three_light_bulbs.attribute("traffic_light_id").asId()) {
          ids.emplace_back(id.value());
        }
      }
    }
  }
  return ids;
}

auto toTrafficLightRegulatoryElement(const lanelet::Id traffic_light_regulatory_element_id)
  -> lanelet::TrafficLight::Ptr
{
  if (isTrafficLightRegulatoryElement(traffic_light_regulatory_element_id)) {
    return std::dynamic_pointer_cast<lanelet::TrafficLight>(
      LaneletWrapper::map()->regulatoryElementLayer.get(traffic_light_regulatory_element_id));
  } else {
    THROW_SEMANTIC_ERROR(
      traffic_light_regulatory_element_id, " is not traffic light regulatory element!");
  }
}

auto toAutowareTrafficLights(const lanelet::Id traffic_light_id)
  -> std::vector<lanelet::AutowareTrafficLightConstPtr>
{
  auto areBulbsAssignedToTrafficLight =
    [traffic_light_id](const auto & red_yellow_green_bulbs) -> bool {
    return red_yellow_green_bulbs.hasAttribute("traffic_light_id") and
           red_yellow_green_bulbs.attribute("traffic_light_id").asId() and
           red_yellow_green_bulbs.attribute("traffic_light_id").asId().value() == traffic_light_id;
  };

  std::vector<lanelet::AutowareTrafficLightConstPtr> autoware_traffic_lights;

  const auto & all_lanelets = lanelet::utils::query::laneletLayer(LaneletWrapper::map());
  for (const auto & autoware_traffic_light :
       lanelet::utils::query::autowareTrafficLights(all_lanelets)) {
    if (const auto & light_bulbs = autoware_traffic_light->lightBulbs();
        std::any_of(light_bulbs.cbegin(), light_bulbs.cend(), areBulbsAssignedToTrafficLight)) {
      autoware_traffic_lights.push_back(autoware_traffic_light);
    }
  }

  if (!autoware_traffic_lights.empty()) {
    return autoware_traffic_lights;
  } else {
    THROW_SEMANTIC_ERROR(
      traffic_light_id,
      " lanelet does not have any regulatory elements with light bulbs assigned!");
  }
}

auto trafficLightBulbPosition(const lanelet::Id traffic_light_id, const std::string & color_name)
  -> std::optional<geometry_msgs::msg::Point>
{
  auto areBulbsAssignedToTrafficLight =
    [traffic_light_id](const auto & red_yellow_green_bulbs) -> bool {
    return red_yellow_green_bulbs.hasAttribute("traffic_light_id") and
           red_yellow_green_bulbs.attribute("traffic_light_id").asId() and
           red_yellow_green_bulbs.attribute("traffic_light_id").asId().value() == traffic_light_id;
  };

  auto isBulbOfExpectedColor = [&color_name](const auto & bulb) -> bool {
    return bulb.hasAttribute("color") and !bulb.hasAttribute("arrow") and
           bulb.attribute("color").value().compare(color_name) == 0;
  };

  const auto & all_lanelets = lanelet::utils::query::laneletLayer(LaneletWrapper::map());
  for (const auto & autoware_traffic_light :
       lanelet::utils::query::autowareTrafficLights(all_lanelets)) {
    for (auto three_light_bulbs : autoware_traffic_light->lightBulbs()) {
      if (areBulbsAssignedToTrafficLight(three_light_bulbs)) {
        for (auto bulb : static_cast<lanelet::ConstLineString3d>(three_light_bulbs)) {
          if (isBulbOfExpectedColor(bulb)) {
            return geometry_msgs::build<Point>().x(bulb.x()).y(bulb.y()).z(bulb.z());
          }
        }
      }
    }
  }
  return std::nullopt;
}

auto trafficLightStopLineIds(const lanelet::Id traffic_light_id) -> lanelet::Ids
{
  lanelet::Ids ids;
  for (const auto & traffic_light : toAutowareTrafficLights(traffic_light_id)) {
    if (traffic_light->stopLine()) {
      ids.push_back(traffic_light->stopLine()->id());
    }
  }
  return ids;
}

auto trafficLightStopLinesPoints(const lanelet::Id traffic_light_id)
  -> std::vector<std::vector<Point>>
{
  std::vector<std::vector<Point>> stop_lines;
  for (const auto & traffic_light : toAutowareTrafficLights(traffic_light_id)) {
    stop_lines.emplace_back(std::vector<Point>{});
    if (const auto & lanelet_stop_line = traffic_light->stopLine()) {
      auto & current_stop_line = stop_lines.back();
      for (const auto & point : lanelet_stop_line.value()) {
        current_stop_line.push_back(
          geometry_msgs::build<Point>().x(point.x()).y(point.y()).z(point.z()));
      }
    }
  }
  return stop_lines;
}

auto trafficLightRegulatoryElementIDsFromTrafficLight(const lanelet::Id traffic_light_id)
  -> lanelet::Ids
{
  if (isTrafficLight(traffic_light_id)) {
    lanelet::Ids traffic_light_regulatory_element_ids;
    for (const auto & regulatory_element : LaneletWrapper::map()->regulatoryElementLayer) {
      if (
        regulatory_element->attribute(lanelet::AttributeName::Subtype).value() == "traffic_light") {
        for (const auto & reference_traffic_light :
             regulatory_element->getParameters<lanelet::ConstLineString3d>("refers")) {
          if (reference_traffic_light.id() == traffic_light_id) {
            traffic_light_regulatory_element_ids.push_back(regulatory_element->id());
          }
        }
      }
    }
    return traffic_light_regulatory_element_ids;
  } else {
    THROW_SEMANTIC_ERROR(traffic_light_id, " is not traffic light!");
  }
}

// On path
auto autowareTrafficLightsOnPath(const lanelet::Ids & lanelet_ids)
  -> std::vector<lanelet::AutowareTrafficLightConstPtr>
{
  std::vector<lanelet::AutowareTrafficLightConstPtr> autoware_traffic_lights;
  for (const auto & lanelet_id : lanelet_ids) {
    const auto & lanelet = LaneletWrapper::map()->laneletLayer.get(lanelet_id);
    for (const auto & traffic_light :
         lanelet.regulatoryElementsAs<const lanelet::autoware::AutowareTrafficLight>()) {
      autoware_traffic_lights.push_back(traffic_light);
    }
  }
  return autoware_traffic_lights;
}

auto trafficLightIdsOnPath(const lanelet::Ids & route_lanelets) -> lanelet::Ids
{
  lanelet::Ids traffic_light_ids;
  for (const auto & autoware_traffic_lights : autowareTrafficLightsOnPath(route_lanelets)) {
    for (const auto & three_light_bulbs : autoware_traffic_lights->lightBulbs()) {
      if (three_light_bulbs.hasAttribute("traffic_light_id")) {
        if (const auto traffic_light_id = three_light_bulbs.attribute("traffic_light_id").asId();
            traffic_light_id) {
          traffic_light_ids.push_back(traffic_light_id.value());
        }
      }
    }
  }
  return traffic_light_ids;
}
}  // namespace traffic_lights
}  // namespace lanelet_wrapper
}  // namespace traffic_simulator
