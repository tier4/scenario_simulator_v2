// Copyright 2015-2019 Tier IV, Inc. All rights reserved.
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

#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_io/io_handlers/Factory.h>
#include <lanelet2_io/io_handlers/Writer.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <iostream>
#include <lanelet2_extension_psim/exception.hpp>
#include <lanelet2_extension_psim/projection/mgrs_projector.hpp>
#include <lanelet2_extension_psim/regulatory_elements/autoware_traffic_light.hpp>
#include <pugixml.hpp>
#include <string>

namespace lanelet
{
namespace validation
{
namespace keyword
{
constexpr const char * Id = "id";
constexpr const char * Osm = "osm";
constexpr const char * Tag = "tag";
constexpr const char * Key = "k";
constexpr const char * Node = "node";
constexpr const char * Elevation = "ele";
}  // namespace keyword

void validateElevationTag(const std::string filename)
{
  pugi::xml_document doc;
  auto result = doc.load_file(filename.c_str());
  if (!result) {
    throw lanelet::HdMapFormatException(result.description());
  }

  auto osmNode = doc.child("osm");
  for (auto node = osmNode.child(keyword::Node); node;  // NOLINT
       node = node.next_sibling(keyword::Node)) {
    const auto id = node.attribute(keyword::Id).as_llong(lanelet::InvalId);
    if (!node.find_child_by_attribute(keyword::Tag, keyword::Key, keyword::Elevation)) {
      std::stringstream sstream;
      sstream << "failed to find elevation tag for node: " << id;
      throw lanelet::HdMapFormatException(sstream.str());
    }
  }
}

void validateTrafficLight(const lanelet::LaneletMapPtr lanelet_map)
{
  if (!lanelet_map) {
    throw lanelet::HdMapFormatException("Missing map. Are you sure you set correct path for map?");
  }

  for (auto lanelet : lanelet_map->laneletLayer) {
    auto autoware_traffic_lights =
      lanelet.regulatoryElementsAs<lanelet::autoware::AutowareTrafficLight>();
    if (autoware_traffic_lights.empty()) {
      continue;
    }
    for (auto light : autoware_traffic_lights) {
      if (light->lightBulbs().empty()) {
        std::stringstream sstream;
        sstream << "regulatory element traffic light " << light->id()
                << " is missing optional light_bulb member. You won't "
                   "be able to use region_tlr node with this map";
        throw lanelet::HdMapFormatException(sstream.str());
      }
      for (auto light_string : light->lightBulbs()) {
        if (!light_string.hasAttribute("traffic_light_id")) {
          std::stringstream sstream;
          sstream << "light_bulb " << light_string.id() << " is missing traffic_light_id tag";
          throw lanelet::HdMapFormatException(sstream.str());
        }
      }
      for (auto base_string_or_poly : light->trafficLights()) {
        if (!base_string_or_poly.isLineString()) {
          std::stringstream sstream;
          sstream << "traffic_light " << base_string_or_poly.id()
                  << " is polygon, and only linestring class is currently supported for "
                     "traffic lights";
          throw lanelet::HdMapFormatException(sstream.str());
        }
        auto base_string = static_cast<lanelet::LineString3d>(base_string_or_poly);
        if (!base_string.hasAttribute("height")) {
          std::stringstream sstream;
          sstream << "traffic_light " << base_string.id() << " is missing height tag";
          throw lanelet::HdMapFormatException(sstream.str());
        }
      }
    }
  }
}

void validateTurnDirection(const lanelet::LaneletMapPtr lanelet_map)
{
  if (!lanelet_map) {
    throw lanelet::HdMapFormatException("Missing map. Are you sure you set correct path for map?");
  }

  lanelet::traffic_rules::TrafficRulesPtr traffic_rules =
    lanelet::traffic_rules::TrafficRulesFactory::create(
      lanelet::Locations::Germany, lanelet::Participants::Vehicle);
  lanelet::routing::RoutingGraphPtr vehicle_graph =
    lanelet::routing::RoutingGraph::build(*lanelet_map, *traffic_rules);

  for (const auto & lanelet : lanelet_map->laneletLayer) {
    if (!traffic_rules->canPass(lanelet)) {
      continue;
    }

    const auto conflicting_lanelets_or_areas = vehicle_graph->conflicting(lanelet);
    if (conflicting_lanelets_or_areas.empty()) {
      continue;
    }
    if (!lanelet.hasAttribute("turn_direction")) {
      std::stringstream sstream;
      sstream << "lanelet " << lanelet.id()
              << " seems to be intersecting other lanelet, but does "
                 "not have turn_direction tagging.";
      throw lanelet::HdMapFormatException(sstream.str());
    }
  }
}

void validateAll(std::string map_path)
{
  lanelet::LaneletMapPtr lanelet_map;
  lanelet::ErrorMessages errors;
  lanelet::projection::MGRSProjector projector;
  lanelet_map = lanelet::load(map_path, "autoware_osm_handler", projector, &errors);
  validateElevationTag(map_path);
  validateTrafficLight(lanelet_map);
  validateTurnDirection(lanelet_map);
}

}  // namespace validation
}  // namespace lanelet
