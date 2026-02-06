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

#include <lanelet2_core/primitives/Primitive.h>
#include <lanelet2_core/utility/Units.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_io/io_handlers/Serialize.h>
#include <lanelet2_projection/UTM.h>

#include <algorithm>
#include <autoware_lanelet2_extension/io/autoware_osm_parser.hpp>
#include <autoware_lanelet2_extension/utility/message_conversion.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <autoware_lanelet2_extension/visualization/visualization.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <deque>
#include <geometry/quaternion/euler_to_quaternion.hpp>
#include <geometry/quaternion/get_rotation.hpp>
#include <geometry/quaternion/operator.hpp>
#include <geometry/quaternion/quaternion_to_euler.hpp>
#include <geometry/spline/catmull_rom_spline.hpp>
#include <geometry/spline/hermite_curve.hpp>
#include <geometry/transform.hpp>
#include <geometry/vector3/inner_product.hpp>
#include <geometry/vector3/normalize.hpp>
#include <geometry/vector3/operator.hpp>
#include <memory>
#include <optional>
#include <scenario_simulator_exception/exception.hpp>
#include <set>
#include <string>
#include <traffic_simulator/color_utils/color_utils.hpp>
#include <traffic_simulator/hdmap_utils/hdmap_utils.hpp>
#include <traffic_simulator/helper/helper.hpp>
#include <traffic_simulator/lanelet_wrapper/lanelet_loader.hpp>
#include <traffic_simulator/lanelet_wrapper/lanelet_map.hpp>
#include <traffic_simulator/lanelet_wrapper/pose.hpp>
#include <traffic_simulator/lanelet_wrapper/traffic_rules.hpp>
#include <unordered_map>
#include <utility>
#include <vector>

namespace hdmap_utils
{
using namespace traffic_simulator::lanelet_wrapper;

HdMapUtils::HdMapUtils(
  const std::filesystem::path & lanelet2_map_path, const geographic_msgs::msg::GeoPoint &)
{
  lanelet_map_ptr_ = LaneletLoader::load(lanelet2_map_path.string());
  routing_graphs_ = std::make_unique<RoutingGraphs>(lanelet_map_ptr_);
}

auto HdMapUtils::getTrafficLightIds() const -> lanelet::Ids
{
  using namespace lanelet::utils::query;

  lanelet::Ids ids;

  for (auto && traffic_light : autowareTrafficLights(laneletLayer(lanelet_map_ptr_))) {
    for (auto && light_bulb : traffic_light->lightBulbs()) {
      if (light_bulb.hasAttribute("traffic_light_id")) {
        if (auto id = light_bulb.attribute("traffic_light_id").asId()) {
          ids.emplace_back(id.value());
        }
      }
    }
  }

  return ids;
}

auto HdMapUtils::getTrafficLightBulbPosition(
  const lanelet::Id traffic_light_id, const std::string & color_name,
  const bool allow_infer_position) const -> std::optional<geometry_msgs::msg::Point>
{
  lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(lanelet_map_ptr_);
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
            geometry_msgs::msg::Point point;
            point.x = bulb.x();
            point.y = bulb.y();
            point.z = bulb.z();
            return point;
          }
        }
      }
    }
  }

  if (!allow_infer_position) {
    return std::nullopt;
  }

  // In case of a traffic light without bulbs, we can check the base string
  // to get the position of the traffic light
  const auto position_scale = color_name == "red" ? 0.25 : (color_name == "yellow" ? 0.5 : 0.75);
  for (const auto & light : autoware_traffic_lights) {
    for (auto & traffic_light : light->trafficLights()) {
      if (traffic_light.id() != traffic_light_id) continue;
      if (!traffic_light.isLineString()) continue;
      const auto base_string = static_cast<lanelet::ConstLineString3d>(traffic_light);
      if (!base_string.hasAttribute("height")) continue;
      const auto height = base_string.attribute("height").asDouble();
      if (!height) continue;

      const auto x1 = base_string.front().x();
      const auto y1 = base_string.front().y();
      const auto z1 = base_string.front().z();

      const auto x2 = base_string.back().x();
      const auto y2 = base_string.back().y();

      geometry_msgs::msg::Point point;
      point.x = x1 * position_scale + x2 * (1 - position_scale);
      point.y = y1 * position_scale + y2 * (1 - position_scale);
      point.z = z1 + *height / 2.0;

      return point;
    }
  }

  return std::nullopt;
}

auto HdMapUtils::insertMarkerArray(
  visualization_msgs::msg::MarkerArray & a1, const visualization_msgs::msg::MarkerArray & a2) const
  -> void
{
  a1.markers.insert(a1.markers.end(), a2.markers.begin(), a2.markers.end());
}

auto HdMapUtils::generateMarker() const -> visualization_msgs::msg::MarkerArray
{
  visualization_msgs::msg::MarkerArray markers;
  lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(lanelet_map_ptr_);
  lanelet::ConstLanelets road_lanelets = lanelet::utils::query::roadLanelets(all_lanelets);
  lanelet::ConstLanelets crosswalk_lanelets =
    lanelet::utils::query::crosswalkLanelets(all_lanelets);
  lanelet::ConstLanelets walkway_lanelets = lanelet::utils::query::walkwayLanelets(all_lanelets);
  lanelet::ConstLineStrings3d stop_lines = lanelet::utils::query::stopLinesLanelets(road_lanelets);
  std::vector<lanelet::AutowareTrafficLightConstPtr> aw_tl_reg_elems =
    lanelet::utils::query::autowareTrafficLights(all_lanelets);
  std::vector<lanelet::DetectionAreaConstPtr> da_reg_elems =
    lanelet::utils::query::detectionAreas(all_lanelets);
  lanelet::ConstLineStrings3d parking_spaces =
    lanelet::utils::query::getAllParkingSpaces(lanelet_map_ptr_);
  lanelet::ConstPolygons3d parking_lots =
    lanelet::utils::query::getAllParkingLots(lanelet_map_ptr_);

  auto cl_ll_borders = color_utils::fromRgba(1.0, 1.0, 1.0, 0.999);
  auto cl_road = color_utils::fromRgba(0.2, 0.7, 0.7, 0.3);
  auto cl_cross = color_utils::fromRgba(0.2, 0.7, 0.2, 0.3);
  auto cl_stoplines = color_utils::fromRgba(1.0, 0.0, 0.0, 0.5);
  auto cl_trafficlights = color_utils::fromRgba(0.7, 0.7, 0.7, 0.8);
  auto cl_detection_areas = color_utils::fromRgba(0.7, 0.7, 0.7, 0.3);
  auto cl_parking_lots = color_utils::fromRgba(0.7, 0.7, 0.0, 0.3);
  auto cl_parking_spaces = color_utils::fromRgba(1.0, 0.647, 0.0, 0.6);
  auto cl_lanelet_id = color_utils::fromRgba(0.8, 0.2, 0.2, 0.999);

  insertMarkerArray(
    markers,
    lanelet::visualization::laneletsBoundaryAsMarkerArray(road_lanelets, cl_ll_borders, true));
  insertMarkerArray(
    markers,
    lanelet::visualization::laneletsAsTriangleMarkerArray("road_lanelets", road_lanelets, cl_road));
  insertMarkerArray(
    markers, lanelet::visualization::laneletsAsTriangleMarkerArray(
               "crosswalk_lanelets", crosswalk_lanelets, cl_cross));
  insertMarkerArray(
    markers, lanelet::visualization::laneletsAsTriangleMarkerArray(
               "walkway_lanelets", walkway_lanelets, cl_cross));
  insertMarkerArray(markers, lanelet::visualization::laneletDirectionAsMarkerArray(road_lanelets));
  insertMarkerArray(
    markers,
    lanelet::visualization::lineStringsAsMarkerArray(stop_lines, "stop_lines", cl_stoplines, 0.1));
  insertMarkerArray(
    markers,
    lanelet::visualization::autowareTrafficLightsAsMarkerArray(aw_tl_reg_elems, cl_trafficlights));
  insertMarkerArray(
    markers, lanelet::visualization::detectionAreasAsMarkerArray(da_reg_elems, cl_detection_areas));
  insertMarkerArray(
    markers, lanelet::visualization::parkingLotsAsMarkerArray(parking_lots, cl_parking_lots));
  insertMarkerArray(
    markers, lanelet::visualization::parkingSpacesAsMarkerArray(parking_spaces, cl_parking_spaces));
  insertMarkerArray(
    markers, lanelet::visualization::generateLaneletIdMarker(road_lanelets, cl_lanelet_id));
  insertMarkerArray(
    markers, lanelet::visualization::generateLaneletIdMarker(crosswalk_lanelets, cl_lanelet_id));
  return markers;
}

auto HdMapUtils::findNearestIndexPair(
  const std::vector<double> & accumulated_lengths, const double target_length) const
  -> std::pair<std::size_t, std::size_t>
{
  // List size
  const auto N = accumulated_lengths.size();
  // Front
  if (target_length < accumulated_lengths.at(1)) {
    return std::make_pair(0, 1);
  }
  // Back
  if (target_length > accumulated_lengths.at(N - 2)) {
    return std::make_pair(N - 2, N - 1);
  }

  // Middle
  for (size_t i = 1; i < N; ++i) {
    if (
      accumulated_lengths.at(i - 1) <= target_length &&
      target_length <= accumulated_lengths.at(i)) {
      return std::make_pair(i - 1, i);
    }
  }

  // Throw an exception because this never happens
  THROW_SEMANTIC_ERROR("findNearestIndexPair(): No nearest point found.");
}

auto HdMapUtils::getTrafficLightRegulatoryElementsOnPath(const lanelet::Ids & lanelet_ids) const
  -> std::vector<std::shared_ptr<const lanelet::autoware::AutowareTrafficLight>>
{
  std::vector<std::shared_ptr<const lanelet::autoware::AutowareTrafficLight>> ret;
  for (const auto & lanelet_id : lanelet_ids) {
    const auto lanelet = lanelet_map_ptr_->laneletLayer.get(lanelet_id);
    const auto traffic_lights =
      lanelet.regulatoryElementsAs<const lanelet::autoware::AutowareTrafficLight>();
    for (const auto & traffic_light : traffic_lights) {
      ret.emplace_back(traffic_light);
    }
  }
  return ret;
}

auto HdMapUtils::getTrafficLights(const lanelet::Id traffic_light_id) const
  -> std::vector<lanelet::AutowareTrafficLightConstPtr>
{
  std::vector<lanelet::AutowareTrafficLightConstPtr> ret;
  lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(lanelet_map_ptr_);
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

auto HdMapUtils::getTrafficLightStopLineIds(const lanelet::Id traffic_light_id) const
  -> lanelet::Ids
{
  lanelet::Ids ids;
  for (const auto & traffic_light : getTrafficLights(traffic_light_id)) {
    if (traffic_light->stopLine()) {
      ids.push_back(traffic_light->stopLine()->id());
    }
  }
  return ids;
}

auto HdMapUtils::getTrafficLightStopLinesPoints(const lanelet::Id traffic_light_id) const
  -> std::vector<std::vector<geometry_msgs::msg::Point>>
{
  std::vector<std::vector<geometry_msgs::msg::Point>> ret;
  const auto traffic_lights = getTrafficLights(traffic_light_id);
  for (const auto & traffic_light : traffic_lights) {
    ret.emplace_back(std::vector<geometry_msgs::msg::Point>{});
    const auto stop_line = traffic_light->stopLine();
    if (stop_line) {
      auto & current_stop_line = ret.back();
      for (const auto & point : stop_line.value()) {
        geometry_msgs::msg::Point p;
        p.x = point.x();
        p.y = point.y();
        p.z = point.z();
        current_stop_line.emplace_back(p);
      }
    }
  }
  return ret;
}

auto HdMapUtils::getTrafficLightIdsOnPath(const lanelet::Ids & route_lanelets) const -> lanelet::Ids
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

auto HdMapUtils::calculateSegmentDistances(const lanelet::ConstLineString3d & line_string) const
  -> std::vector<double>
{
  std::vector<double> segment_distances;
  segment_distances.reserve(line_string.size() - 1);
  for (size_t i = 1; i < line_string.size(); ++i) {
    const auto distance = lanelet::geometry::distance(line_string[i], line_string[i - 1]);
    segment_distances.push_back(distance);
  }
  return segment_distances;
}

auto HdMapUtils::calculateAccumulatedLengths(const lanelet::ConstLineString3d & line_string) const
  -> std::vector<double>
{
  const auto segment_distances = calculateSegmentDistances(line_string);

  std::vector<double> accumulated_lengths{0};
  accumulated_lengths.reserve(segment_distances.size() + 1);
  std::partial_sum(
    std::begin(segment_distances), std::end(segment_distances),
    std::back_inserter(accumulated_lengths));
  return accumulated_lengths;
}

auto HdMapUtils::resamplePoints(
  const lanelet::ConstLineString3d & line_string, const std::int32_t num_segments) const
  -> lanelet::BasicPoints3d
{
  // Calculate length
  const auto line_length = lanelet::geometry::length(line_string);

  // Calculate accumulated lengths
  const auto accumulated_lengths = calculateAccumulatedLengths(line_string);

  // Create each segment
  lanelet::BasicPoints3d resampled_points;
  for (auto i = 0; i <= num_segments; ++i) {
    // Find two nearest points
    const double target_length =
      (static_cast<double>(i) / num_segments) * static_cast<double>(line_length);
    const auto index_pair = findNearestIndexPair(accumulated_lengths, target_length);

    // Apply linear interpolation
    const lanelet::BasicPoint3d back_point = line_string[index_pair.first];
    const lanelet::BasicPoint3d front_point = line_string[index_pair.second];
    const auto direction_vector = (front_point - back_point);

    const auto back_length = accumulated_lengths.at(index_pair.first);
    const auto front_length = accumulated_lengths.at(index_pair.second);
    const auto segment_length = front_length - back_length;
    const auto target_point =
      back_point + (direction_vector * (target_length - back_length) / segment_length);

    // Add to list
    resampled_points.push_back(target_point);
  }
  return resampled_points;
}

auto HdMapUtils::generateFineCenterline(
  const lanelet::ConstLanelet & lanelet_obj, const double resolution) const -> lanelet::LineString3d
{
  // Get length of longer border
  const double left_length =
    static_cast<double>(lanelet::geometry::length(lanelet_obj.leftBound()));
  const double right_length =
    static_cast<double>(lanelet::geometry::length(lanelet_obj.rightBound()));
  const double longer_distance = (left_length > right_length) ? left_length : right_length;
  const int32_t num_segments =
    std::max(static_cast<int32_t>(ceil(longer_distance / resolution)), 1);

  // Resample points
  const auto left_points = resamplePoints(lanelet_obj.leftBound(), num_segments);
  const auto right_points = resamplePoints(lanelet_obj.rightBound(), num_segments);

  // Create centerline
  lanelet::LineString3d centerline(lanelet::utils::getId());
  for (size_t i = 0; i < static_cast<size_t>(num_segments + 1); i++) {
    // Add ID for the average point of left and right
    const auto center_basic_point = (right_points.at(i) + left_points.at(i)) / 2.0;
    const lanelet::Point3d center_point(
      lanelet::utils::getId(), center_basic_point.x(), center_basic_point.y(),
      center_basic_point.z());
    centerline.push_back(center_point);
  }
  return centerline;
}

auto HdMapUtils::isTrafficLight(const lanelet::Id lanelet_id) const -> bool
{
  using namespace lanelet;

  if (lanelet_map_ptr_->lineStringLayer.exists(lanelet_id)) {
    if (auto && linestring = lanelet_map_ptr_->lineStringLayer.get(lanelet_id);
        linestring.hasAttribute(AttributeName::Type)) {
      return linestring.attribute(AttributeName::Type).value() == "traffic_light";
    }
  }

  return false;
}

auto HdMapUtils::isTrafficLightRegulatoryElement(const lanelet::Id lanelet_id) const -> bool
{
  return lanelet_map_ptr_->regulatoryElementLayer.exists(lanelet_id) and
         std::dynamic_pointer_cast<lanelet::TrafficLight>(
           lanelet_map_ptr_->regulatoryElementLayer.get(lanelet_id));
}

auto HdMapUtils::getTrafficLightRegulatoryElement(const lanelet::Id lanelet_id) const
  -> lanelet::TrafficLight::Ptr
{
  assert(isTrafficLightRegulatoryElement(lanelet_id));
  return std::dynamic_pointer_cast<lanelet::TrafficLight>(
    lanelet_map_ptr_->regulatoryElementLayer.get(lanelet_id));
}

auto HdMapUtils::getTrafficLightRegulatoryElementIDsFromTrafficLight(
  const lanelet::Id traffic_light_way_id) const -> lanelet::Ids
{
  assert(isTrafficLight(traffic_light_way_id));
  lanelet::Ids traffic_light_regulatory_element_ids;
  for (const auto & regulatory_element : lanelet_map_ptr_->regulatoryElementLayer) {
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

HdMapUtils::RoutingGraphs::RoutingGraphs(const lanelet::LaneletMapPtr & lanelet_map)
{
  vehicle.rules = lanelet::traffic_rules::TrafficRulesFactory::create(
    lanelet::Locations::Germany, lanelet::Participants::Vehicle);
  vehicle.graph = lanelet::routing::RoutingGraph::build(*lanelet_map, *vehicle.rules);
  vehicle_with_road_shoulder.rules = lanelet::traffic_rules::TrafficRulesFactory::create(
    Locations::RoadShoulderPassableGermany, lanelet::Participants::Vehicle);
  vehicle_with_road_shoulder.graph =
    lanelet::routing::RoutingGraph::build(*lanelet_map, *vehicle_with_road_shoulder.rules);
  pedestrian.rules = lanelet::traffic_rules::TrafficRulesFactory::create(
    lanelet::Locations::Germany, lanelet::Participants::Pedestrian);
  pedestrian.graph = lanelet::routing::RoutingGraph::build(*lanelet_map, *pedestrian.rules);
}

lanelet::routing::RoutingGraphPtr HdMapUtils::RoutingGraphs::routing_graph(
  const traffic_simulator::RoutingGraphType type) const
{
  switch (type) {
    case traffic_simulator::RoutingGraphType::VEHICLE:
      return vehicle.graph;
    case traffic_simulator::RoutingGraphType::VEHICLE_WITH_ROAD_SHOULDER:
      return vehicle_with_road_shoulder.graph;
    case traffic_simulator::RoutingGraphType::PEDESTRIAN:
      return pedestrian.graph;
    default:
      std::stringstream what;
      what << "Invalid routing graph type: " << static_cast<int>(type);
      throw common::Error(what.str());
  }
}

lanelet::traffic_rules::TrafficRulesPtr HdMapUtils::RoutingGraphs::traffic_rule(
  const traffic_simulator::RoutingGraphType type) const
{
  switch (type) {
    case traffic_simulator::RoutingGraphType::VEHICLE:
      return vehicle.rules;
    case traffic_simulator::RoutingGraphType::VEHICLE_WITH_ROAD_SHOULDER:
      return vehicle_with_road_shoulder.rules;
    case traffic_simulator::RoutingGraphType::PEDESTRIAN:
      return pedestrian.rules;
    default:
      std::stringstream what;
      what << "Invalid routing graph type: " << static_cast<int>(type);
      throw common::Error(what.str());
  }
}

RouteCache & HdMapUtils::RoutingGraphs::route_cache(const traffic_simulator::RoutingGraphType type)
{
  switch (type) {
    case traffic_simulator::RoutingGraphType::VEHICLE:
      return vehicle.route_cache;
    case traffic_simulator::RoutingGraphType::VEHICLE_WITH_ROAD_SHOULDER:
      return vehicle_with_road_shoulder.route_cache;
    case traffic_simulator::RoutingGraphType::PEDESTRIAN:
      return pedestrian.route_cache;
    default:
      std::stringstream what;
      what << "Invalid routing graph type: " << static_cast<int>(type);
      throw common::Error(what.str());
  }
}

auto HdMapUtils::RoutingGraphs::getRoute(
  const lanelet::Id from_lanelet_id, const lanelet::Id to_lanelet_id,
  lanelet::LaneletMapPtr lanelet_map_ptr,
  const traffic_simulator::RoutingConfiguration & routing_configuration) -> lanelet::Ids
{
  auto & cache = route_cache(routing_configuration.routing_graph_type);
  if (cache.exists(from_lanelet_id, to_lanelet_id, routing_configuration.allow_lane_change)) {
    return cache.getRoute(from_lanelet_id, to_lanelet_id, routing_configuration.allow_lane_change);
  }
  lanelet::Ids ids;
  const auto lanelet = lanelet_map_ptr->laneletLayer.get(from_lanelet_id);
  const auto to_lanelet = lanelet_map_ptr->laneletLayer.get(to_lanelet_id);
  lanelet::Optional<lanelet::routing::Route> route =
    routing_graph(routing_configuration.routing_graph_type)
      ->getRoute(lanelet, to_lanelet, 0, routing_configuration.allow_lane_change);
  if (!route) {
    cache.appendData(from_lanelet_id, to_lanelet_id, routing_configuration.allow_lane_change, ids);
    return ids;
  }
  lanelet::routing::LaneletPath shortest_path = route->shortestPath();
  if (shortest_path.empty()) {
    cache.appendData(from_lanelet_id, to_lanelet_id, routing_configuration.allow_lane_change, ids);
    return ids;
  }
  for (auto lane_itr = shortest_path.begin(); lane_itr != shortest_path.end(); lane_itr++) {
    ids.push_back(lane_itr->id());
  }
  cache.appendData(from_lanelet_id, to_lanelet_id, routing_configuration.allow_lane_change, ids);
  return ids;
}
}  // namespace hdmap_utils
