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

#include <traffic_simulator/utils/lanelet_map.hpp>

namespace traffic_simulator
{
inline namespace lanelet_map
{
auto laneletLength(const lanelet::Id lanelet_id) -> double
{
  return lanelet_wrapper::lanelet_map::laneletLength(lanelet_id);
}

// auto laneletYaw(const Point & point, const lanelet::Id lanelet_id)
//   -> std::tuple<double, Point, Point>
// {
//   return lanelet_wrapper::lanelet_map::laneletYaw(lanelet_id, point);
// }

// auto nearbyLaneletIds(
//   const Pose & pose, const double distance_thresh, const bool include_crosswalk,
//   const std::size_t search_count) -> lanelet::Ids
// {
//   return lanelet_wrapper::lanelet_map::nearbyLaneletIds(
//     pose.position, distance_thresh, include_crosswalk, search_count);
// }

// auto borderlinePoses() -> std::vector<std::pair<lanelet::Id, Pose>>
// {
//   std::vector<std::pair<lanelet::Id, Pose>> borderline_poses;
//   for (const auto & lanelet_id : lanelet_wrapper::lanelet_map::laneletIds()) {
//     if (lanelet_wrapper::lanelet_map::nextLaneletIds(lanelet_id).empty()) {
//       LaneletPose lanelet_pose;
//       lanelet_pose.lanelet_id = lanelet_id;
//       lanelet_pose.s = lanelet_map::laneletLength(lanelet_id);
//       borderline_poses.push_back({lanelet_id, pose::toMapPose(lanelet_pose)});
//     }
//   }
//   return borderline_poses;
// }

// auto visualizationMarker() -> visualization_msgs::msg::MarkerArray
// {
//   visualization_msgs::msg::MarkerArray markers;
//   auto insertMarkerArray = [](
//                              visualization_msgs::msg::MarkerArray & a1,
//                              const visualization_msgs::msg::MarkerArray & a2) -> void {
//     a1.markers.insert(a1.markers.end(), a2.markers.begin(), a2.markers.end());
//   };

//   lanelet::ConstLanelets all_lanelets =
//     lanelet::utils::query::laneletLayer(lanelet_wrapper::LaneletWrapper::map());
//   lanelet::ConstLanelets road_lanelets = lanelet::utils::query::roadLanelets(all_lanelets);
//   lanelet::ConstLanelets crosswalk_lanelets =
//     lanelet::utils::query::crosswalkLanelets(all_lanelets);
//   lanelet::ConstLanelets walkway_lanelets = lanelet::utils::query::walkwayLanelets(all_lanelets);
//   lanelet::ConstLineStrings3d stop_lines = lanelet::utils::query::stopLinesLanelets(road_lanelets);
//   std::vector<lanelet::AutowareTrafficLightConstPtr> aw_tl_reg_elems =
//     lanelet::utils::query::autowareTrafficLights(all_lanelets);
//   std::vector<lanelet::DetectionAreaConstPtr> da_reg_elems =
//     lanelet::utils::query::detectionAreas(all_lanelets);
//   lanelet::ConstLineStrings3d parking_spaces =
//     lanelet::utils::query::getAllParkingSpaces(lanelet_wrapper::LaneletWrapper::map());
//   lanelet::ConstPolygons3d parking_lots =
//     lanelet::utils::query::getAllParkingLots(lanelet_wrapper::LaneletWrapper::map());

//   auto cl_ll_borders = color_utils::fromRgba(1.0, 1.0, 1.0, 0.999);
//   auto cl_road = color_utils::fromRgba(0.2, 0.7, 0.7, 0.3);
//   auto cl_cross = color_utils::fromRgba(0.2, 0.7, 0.2, 0.3);
//   auto cl_stoplines = color_utils::fromRgba(1.0, 0.0, 0.0, 0.5);
//   auto cl_trafficlights = color_utils::fromRgba(0.7, 0.7, 0.7, 0.8);
//   auto cl_detection_areas = color_utils::fromRgba(0.7, 0.7, 0.7, 0.3);
//   auto cl_parking_lots = color_utils::fromRgba(0.7, 0.7, 0.0, 0.3);
//   auto cl_parking_spaces = color_utils::fromRgba(1.0, 0.647, 0.0, 0.6);
//   auto cl_lanelet_id = color_utils::fromRgba(0.8, 0.2, 0.2, 0.999);

//   insertMarkerArray(
//     markers,
//     lanelet::visualization::laneletsBoundaryAsMarkerArray(road_lanelets, cl_ll_borders, true));
//   insertMarkerArray(
//     markers,
//     lanelet::visualization::laneletsAsTriangleMarkerArray("road_lanelets", road_lanelets, cl_road));
//   insertMarkerArray(
//     markers, lanelet::visualization::laneletsAsTriangleMarkerArray(
//                "crosswalk_lanelets", crosswalk_lanelets, cl_cross));
//   insertMarkerArray(
//     markers, lanelet::visualization::laneletsAsTriangleMarkerArray(
//                "walkway_lanelets", walkway_lanelets, cl_cross));
//   insertMarkerArray(markers, lanelet::visualization::laneletDirectionAsMarkerArray(road_lanelets));
//   insertMarkerArray(
//     markers,
//     lanelet::visualization::lineStringsAsMarkerArray(stop_lines, "stop_lines", cl_stoplines, 0.1));
//   insertMarkerArray(
//     markers,
//     lanelet::visualization::autowareTrafficLightsAsMarkerArray(aw_tl_reg_elems, cl_trafficlights));
//   insertMarkerArray(
//     markers, lanelet::visualization::detectionAreasAsMarkerArray(da_reg_elems, cl_detection_areas));
//   insertMarkerArray(
//     markers, lanelet::visualization::parkingLotsAsMarkerArray(parking_lots, cl_parking_lots));
//   insertMarkerArray(
//     markers, lanelet::visualization::parkingSpacesAsMarkerArray(parking_spaces, cl_parking_spaces));
//   insertMarkerArray(
//     markers, lanelet::visualization::generateLaneletIdMarker(road_lanelets, cl_lanelet_id));
//   insertMarkerArray(
//     markers, lanelet::visualization::generateLaneletIdMarker(crosswalk_lanelets, cl_lanelet_id));
//   return markers;
// }
}  // namespace lanelet_map
}  // namespace traffic_simulator
