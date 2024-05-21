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

#ifndef TRAFFIC_SIMULATOR__UTILS__LANELET_HPP_
#define TRAFFIC_SIMULATOR__UTILS__LANELET_HPP_
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_core/primitives/LaneletSequence.h>
#include <lanelet2_core/utility/Units.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_io/io_handlers/Serialize.h>
#include <lanelet2_matching/LaneletMatching.h>
#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingCost.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_routing/RoutingGraphContainer.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <geometry/linear_algebra.hpp>
#include <geometry/spline/catmull_rom_spline.hpp>
#include <geometry/spline/catmull_rom_spline_interface.hpp>
#include <geometry/spline/hermite_curve.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <lanelet2_extension/io/autoware_osm_parser.hpp>
#include <lanelet2_extension/projection/mgrs_projector.hpp>
#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <lanelet2_extension/visualization/visualization.hpp>
#include <optional>
#include <traffic_simulator/hdmap_utils/cache.hpp>
#include <traffic_simulator/helper/helper.hpp>
#include <traffic_simulator_msgs/msg/bounding_box.hpp>
#include <traffic_simulator_msgs/msg/entity_type.hpp>
#include <traffic_simulator_msgs/msg/lanelet_pose.hpp>

namespace traffic_simulator
{
inline namespace lanelet2
{

auto toLaneletPose(
  const geometry_msgs::msg::Pose &, const bool include_crosswalk,
  const double matching_distance = 1.0) -> std::optional<traffic_simulator_msgs::msg::LaneletPose>;

auto toLaneletPose(
  const geometry_msgs::msg::Pose &, const lanelet::Ids &, const double matching_distance = 1.0)
  -> std::optional<traffic_simulator_msgs::msg::LaneletPose>;

auto toLaneletPose(
  const geometry_msgs::msg::Pose &, const traffic_simulator_msgs::msg::BoundingBox &,
  const bool include_crosswalk, const double matching_distance = 1.0)
  -> std::optional<traffic_simulator_msgs::msg::LaneletPose>;

auto toLaneletPose(
  const geometry_msgs::msg::Pose &, const lanelet::Id, const double matching_distance = 1.0)
  -> std::optional<traffic_simulator_msgs::msg::LaneletPose>;

auto toLaneletPoses(
  const geometry_msgs::msg::Pose &, const lanelet::Id, const double matching_distance = 5.0,
  const bool include_opposite_direction = true)
  -> std::vector<traffic_simulator_msgs::msg::LaneletPose>;

auto getCenterPointsSpline(const lanelet::Id) -> std::shared_ptr<math::geometry::CatmullRomSpline>;

auto getNearbyLaneletIds(
  const geometry_msgs::msg::Point &, const double distance_threshold, const bool include_crosswalk,
  const std::size_t search_count = 5) -> lanelet::Ids;

auto getLeftLaneletIds(
  const lanelet::Id, const traffic_simulator_msgs::msg::EntityType &,
  const bool include_opposite_direction = true) -> lanelet::Ids;

auto getRightLaneletIds(
  lanelet::Id, traffic_simulator_msgs::msg::EntityType, bool include_opposite_direction = true)
  -> lanelet::Ids;

auto matchToLane(
  const geometry_msgs::msg::Pose &, const traffic_simulator_msgs::msg::BoundingBox &,
  const bool include_crosswalk, const double matching_distance = 1.0,
  const double reduction_ratio = 0.8) -> std::optional<lanelet::Id>;

auto getNextLaneletIds(const lanelet::Ids &) -> lanelet::Ids;

auto getNextLaneletIds(const lanelet::Ids &, const std::string & turn_direction) -> lanelet::Ids;

auto getNextLaneletIds(const lanelet::Id) -> lanelet::Ids;

auto getNextLaneletIds(const lanelet::Id, const std::string & turn_direction) -> lanelet::Ids;

auto getPreviousLaneletIds(const lanelet::Ids &) -> lanelet::Ids;

auto getPreviousLaneletIds(const lanelet::Ids &, const std::string & turn_direction)
  -> lanelet::Ids;

auto getPreviousLaneletIds(const lanelet::Id) -> lanelet::Ids;

auto getPreviousLaneletIds(const lanelet::Id, const std::string & turn_direction) -> lanelet::Ids;

auto getCenterPoints(const lanelet::Ids &) -> std::vector<geometry_msgs::msg::Point>;

auto getCenterPoints(const lanelet::Id) -> std::vector<geometry_msgs::msg::Point>;

auto getLaneletIds() -> lanelet::Ids;

template <typename Lanelet>
auto getLaneletIds(const std::vector<Lanelet> & lanelets) -> lanelet::Ids
{
  lanelet::Ids ids;
  std::transform(
    lanelets.begin(), lanelets.end(), std::back_inserter(ids),
    [](const auto & lanelet) { return lanelet.id(); });
  return ids;
}

auto getNextRoadShoulderLanelet(const lanelet::Id) -> lanelet::Ids;

auto getPreviousRoadShoulderLanelet(const lanelet::Id) -> lanelet::Ids;

auto excludeSubtypeLanelets(
  const std::vector<std::pair<double, lanelet::Lanelet>> &, const char subtype[])
  -> std::vector<std::pair<double, lanelet::Lanelet>>;

auto toPoint2d(const geometry_msgs::msg::Point &) -> lanelet::BasicPoint2d;

auto absoluteHull(const lanelet::BasicPolygon2d & relative_hull, const lanelet::matching::Pose2d &)
  -> lanelet::BasicPolygon2d;

auto overwriteLaneletsCenterline() -> void;

auto generateFineCenterline(const lanelet::ConstLanelet &, const double resolution)
  -> lanelet::LineString3d;

auto resamplePoints(const lanelet::ConstLineString3d &, const std::int32_t num_segments)
  -> lanelet::BasicPoints3d;

auto calculateAccumulatedLengths(const lanelet::ConstLineString3d &) -> std::vector<double>;

auto findNearestIndexPair(
  const std::vector<double> & accumulated_lengths, const double target_length)
  -> std::pair<std::size_t, std::size_t>;

auto calculateSegmentDistances(const lanelet::ConstLineString3d &) -> std::vector<double>;

////////////////////////////////////////
///////////////////////////////////////
/////////////////////////////////////
auto getAllCanonicalizedLaneletPoses(const traffic_simulator_msgs::msg::LaneletPose & lanelet_pose)
  -> std::vector<traffic_simulator_msgs::msg::LaneletPose>;

auto canonicalizeLaneletPose(const traffic_simulator_msgs::msg::LaneletPose & lanelet_pose)
  -> std::tuple<
    std::optional<traffic_simulator_msgs::msg::LaneletPose>, std::optional<lanelet::Id>>;

auto canonicalizeLaneletPose(
  const traffic_simulator_msgs::msg::LaneletPose & lanelet_pose,
  const lanelet::Ids & route_lanelets)
  -> std::tuple<
    std::optional<traffic_simulator_msgs::msg::LaneletPose>, std::optional<lanelet::Id>>;

auto getRoute(const lanelet::Id from, const lanelet::Id to, bool allow_lane_change = false)
  -> lanelet::Ids;

auto getLaneletLength(const lanelet::Id lanelet_id) -> double;

////////////////////////////////////////
///////////////////////////////////////
/////////////////////////////////////

auto getFollowingLanelets(
  const lanelet::Id lanelet_id, const lanelet::Ids & candidate_lanelet_ids,
  const double distance = 100, const bool include_self = true) -> lanelet::Ids;

auto getFollowingLanelets(
  const lanelet::Id, const double distance = 100, const bool include_self = true) -> lanelet::Ids;

auto isInRoute(const lanelet::Id lanelet_id, const lanelet::Ids & route) -> bool;

auto getTrafficLightRegulatoryElementIDsFromTrafficLight(const lanelet::Id traffic_light_way_id)
  -> lanelet::Ids;

auto getLateralDistance(
  const traffic_simulator_msgs::msg::LaneletPose & from,
  const traffic_simulator_msgs::msg::LaneletPose & to, bool allow_lane_change)
  -> std::optional<double>;

auto getLongitudinalDistance(
  const traffic_simulator_msgs::msg::LaneletPose & from,
  const traffic_simulator_msgs::msg::LaneletPose & to, bool allow_lane_change = false)
  -> std::optional<double>;

auto getLeftBound(const lanelet::Id lanelet_id) -> std::vector<geometry_msgs::msg::Point>;

auto getRightBound(const lanelet::Id lanelet_id) -> std::vector<geometry_msgs::msg::Point>;

auto getLaneletPolygon(const lanelet::Id lanelet_id) -> std::vector<geometry_msgs::msg::Point>;

auto getStopLinePolygon(const lanelet::Id lanelet_id) -> std::vector<geometry_msgs::msg::Point>;

auto toMapPose(const traffic_simulator_msgs::msg::LaneletPose &, const bool fill_pitch = true)
  -> geometry_msgs::msg::PoseStamped;

auto toPolygon(const lanelet::ConstLineString3d & line_string)
  -> std::vector<geometry_msgs::msg::Point>;

auto getLaneChangeableLaneletId(const lanelet::Id, const traffic_simulator::lane_change::Direction)
  -> std::optional<lanelet::Id>;

auto getLaneChangeableLaneletId(
  const lanelet::Id, const traffic_simulator::lane_change::Direction, const std::uint8_t shift)
  -> std::optional<lanelet::Id>;

auto generateMarker() -> visualization_msgs::msg::MarkerArray;

auto insertMarkerArray(
  visualization_msgs::msg::MarkerArray &, const visualization_msgs::msg::MarkerArray &) -> void;

auto getDistanceToStopLine(
  const lanelet::Ids & route_lanelets, const std::vector<geometry_msgs::msg::Point> & waypoints)
  -> std::optional<double>;

auto getDistanceToStopLine(
  const lanelet::Ids & route_lanelets, const math::geometry::CatmullRomSplineInterface & spline)
  -> std::optional<double>;

auto getSpeedLimit(const lanelet::Ids & lanelet_ids) -> double;

auto canChangeLane(const lanelet::Id from_lanelet_id, const lanelet::Id to_lanelet_id) -> bool;

auto getAlongLaneletPose(
  const traffic_simulator_msgs::msg::LaneletPose & from_pose, const double along)
  -> traffic_simulator_msgs::msg::LaneletPose;

auto getStopLinesOnPath(const lanelet::Ids & lanelet_ids) -> lanelet::ConstLineStrings3d;

auto getTrafficSignRegulatoryElementsOnPath(const lanelet::Ids & lanelet_ids)
  -> std::vector<std::shared_ptr<const lanelet::TrafficSign>>;

auto getPreviousLanelets(const lanelet::Id, const double distance = 100) -> lanelet::Ids;

auto getRightOfWayLaneletIds(const lanelet::Ids & lanelet_ids)
  -> std::unordered_map<lanelet::Id, lanelet::Ids>;

auto getRightOfWayLaneletIds(const lanelet::Id lanelet_id) -> lanelet::Ids;

auto getLaneChangeTrajectory(
  const traffic_simulator_msgs::msg::LaneletPose & from_pose,
  const traffic_simulator::lane_change::Parameter & lane_change_parameter)
  -> std::optional<std::pair<math::geometry::HermiteCurve, double>>;

auto getLaneChangeTrajectory(
  const geometry_msgs::msg::Pose & from_pose,
  const traffic_simulator::lane_change::Parameter & lane_change_parameter,
  const double maximum_curvature_threshold, const double target_trajectory_length,
  const double forward_distance_threshold)
  -> std::optional<std::pair<math::geometry::HermiteCurve, double>>;

auto getLaneChangeTrajectory(
  const geometry_msgs::msg::Pose & from_pose,
  const traffic_simulator_msgs::msg::LaneletPose & to_pose,
  const traffic_simulator::lane_change::TrajectoryShape trajectory_shape,
  const double tangent_vector_size) -> math::geometry::HermiteCurve;

auto getTangentVector(const lanelet::Id lanelet_id, const double s)
  -> std::optional<geometry_msgs::msg::Vector3>;

auto getVectorFromPose(const geometry_msgs::msg::Pose & pose, const double magnitude)
  -> geometry_msgs::msg::Vector3;

auto getTrafficLightIdsOnPath(const lanelet::Ids & route_lanelets) -> lanelet::Ids;

auto getDistanceToTrafficLightStopLine(
  const lanelet::Ids & route_lanelets, const std::vector<geometry_msgs::msg::Point> & waypoints)
  -> std::optional<double>;

auto getDistanceToTrafficLightStopLine(
  const lanelet::Ids & route_lanelets, const math::geometry::CatmullRomSplineInterface & spline)
  -> std::optional<double>;

auto getDistanceToTrafficLightStopLine(
  const std::vector<geometry_msgs::msg::Point> & waypoints, const lanelet::Id traffic_light_id)
  -> std::optional<double>;

auto getDistanceToTrafficLightStopLine(
  const math::geometry::CatmullRomSplineInterface & spline, const lanelet::Id traffic_light_id)
  -> std::optional<double>;

auto getConflictingCrosswalkIds(const lanelet::Ids & lanelet_ids) -> lanelet::Ids;

auto getConflictingLaneIds(const lanelet::Ids & lanelet_ids) -> lanelet::Ids;

auto getTrafficLightStopLinesPoints(const lanelet::Id traffic_light_id)
  -> std::vector<std::vector<geometry_msgs::msg::Point>>;

auto getTrafficLightRegulatoryElementsOnPath(const lanelet::Ids & lanelet_ids)
  -> std::vector<std::shared_ptr<const lanelet::autoware::AutowareTrafficLight>>;

auto getTrafficLights(const lanelet::Id traffic_light_id)
  -> std::vector<lanelet::AutowareTrafficLightConstPtr>;
}  // namespace lanelet2
}  // namespace traffic_simulator
#endif  // TRAFFIC_SIMULATOR__UTILS__LANELET_HPP_
