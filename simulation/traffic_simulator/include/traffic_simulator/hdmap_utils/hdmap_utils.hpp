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

#ifndef TRAFFIC_SIMULATOR__HDMAP_UTILS__HDMAP_UTILS_HPP_
#define TRAFFIC_SIMULATOR__HDMAP_UTILS__HDMAP_UTILS_HPP_

#include <geometry_msgs/msg/vector3.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_core/primitives/LaneletSequence.h>
#include <lanelet2_matching/LaneletMatching.h>
#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingCost.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_routing/RoutingGraphContainer.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <autoware_lanelet2_extension/utility/message_conversion.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <boost/filesystem.hpp>
#include <geographic_msgs/msg/geo_point.hpp>
#include <geometry/spline/catmull_rom_spline.hpp>
#include <geometry/spline/catmull_rom_spline_interface.hpp>
#include <geometry/spline/hermite_curve.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <map>
#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <traffic_simulator/data_type/lane_change.hpp>
#include <traffic_simulator/hdmap_utils/cache.hpp>
#include <traffic_simulator_msgs/msg/bounding_box.hpp>
#include <traffic_simulator_msgs/msg/entity_status.hpp>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>
#include <visualization_msgs/msg/marker_array.hpp>

namespace hdmap_utils
{
enum class LaneletType { LANE, CROSSWALK };

class HdMapUtils
{
public:
  explicit HdMapUtils(const boost::filesystem::path &, const geographic_msgs::msg::GeoPoint &);

  auto canChangeLane(const lanelet::Id from, const lanelet::Id to) const -> bool;

  auto canonicalizeLaneletPose(const traffic_simulator_msgs::msg::LaneletPose &) const
    -> std::tuple<
      std::optional<traffic_simulator_msgs::msg::LaneletPose>, std::optional<lanelet::Id>>;

  auto canonicalizeLaneletPose(
    const traffic_simulator_msgs::msg::LaneletPose &, const lanelet::Ids & route_lanelets) const
    -> std::tuple<
      std::optional<traffic_simulator_msgs::msg::LaneletPose>, std::optional<lanelet::Id>>;

  auto clipTrajectoryFromLaneletIds(
    const lanelet::Id, const double s, const lanelet::Ids &,
    const double forward_distance = 20) const -> std::vector<geometry_msgs::msg::Point>;

  auto filterLaneletIds(const lanelet::Ids &, const char subtype[]) const -> lanelet::Ids;

  auto generateMarker() const -> visualization_msgs::msg::MarkerArray;

  auto getAllCanonicalizedLaneletPoses(const traffic_simulator_msgs::msg::LaneletPose &) const
    -> std::vector<traffic_simulator_msgs::msg::LaneletPose>;

  auto getAlongLaneletPose(
    const traffic_simulator_msgs::msg::LaneletPose & from, const double along) const
    -> traffic_simulator_msgs::msg::LaneletPose;

  auto getCenterPoints(const lanelet::Ids &) const -> std::vector<geometry_msgs::msg::Point>;

  auto getCenterPoints(const lanelet::Id) const -> std::vector<geometry_msgs::msg::Point>;

  auto getCenterPointsSpline(const lanelet::Id) const
    -> std::shared_ptr<math::geometry::CatmullRomSpline>;

  auto getClosestLaneletId(
    const geometry_msgs::msg::Pose &, const double distance_thresh = 30.0,
    const bool include_crosswalk = false) const -> std::optional<lanelet::Id>;

  auto getCollisionPointInLaneCoordinate(
    const lanelet::Id lanelet_id, const lanelet::Id crossing_lanelet_id) const
    -> std::optional<double>;

  auto getConflictingCrosswalkIds(const lanelet::Ids &) const -> lanelet::Ids;

  auto getConflictingLaneIds(const lanelet::Ids &) const -> lanelet::Ids;

  auto getDistanceToStopLine(
    const lanelet::Ids & route_lanelets,
    const math::geometry::CatmullRomSplineInterface & spline) const -> std::optional<double>;

  auto getDistanceToStopLine(
    const lanelet::Ids & route_lanelets,
    const std::vector<geometry_msgs::msg::Point> & waypoints) const -> std::optional<double>;

  auto getDistanceToTrafficLightStopLine(
    const lanelet::Ids & route_lanelets,
    const math::geometry::CatmullRomSplineInterface & spline) const -> std::optional<double>;

  auto getDistanceToTrafficLightStopLine(
    const lanelet::Ids & route_lanelets,
    const std::vector<geometry_msgs::msg::Point> & waypoints) const -> std::optional<double>;

  auto getDistanceToTrafficLightStopLine(
    const math::geometry::CatmullRomSplineInterface & spline,
    const lanelet::Id traffic_light_id) const -> std::optional<double>;

  auto getDistanceToTrafficLightStopLine(
    const std::vector<geometry_msgs::msg::Point> & waypoints,
    const lanelet::Id traffic_light_id) const -> std::optional<double>;

  auto getFollowingLanelets(
    const lanelet::Id lanelet_id, const lanelet::Ids & candidate_lanelet_ids,
    const double distance = 100, const bool include_self = true) const -> lanelet::Ids;

  auto getFollowingLanelets(
    const lanelet::Id, const double distance = 100, const bool include_self = true) const
    -> lanelet::Ids;

  auto getHeight(const traffic_simulator_msgs::msg::LaneletPose &) const -> double;

  auto getLaneChangeTrajectory(
    const geometry_msgs::msg::Pose & from,
    const traffic_simulator::lane_change::Parameter & lane_change_parameter,
    const double maximum_curvature_threshold, const double target_trajectory_length,
    const double forward_distance_threshold) const
    -> std::optional<std::pair<math::geometry::HermiteCurve, double>>;

  auto getLaneChangeTrajectory(
    const traffic_simulator_msgs::msg::LaneletPose & from,
    const traffic_simulator::lane_change::Parameter & lane_change_parameter) const
    -> std::optional<std::pair<math::geometry::HermiteCurve, double>>;

  auto getLaneChangeableLaneletId(
    const lanelet::Id, const traffic_simulator::lane_change::Direction) const
    -> std::optional<lanelet::Id>;

  auto getLaneChangeableLaneletId(
    const lanelet::Id, const traffic_simulator::lane_change::Direction,
    const std::uint8_t shift) const -> std::optional<lanelet::Id>;

  auto getLaneletIds() const -> lanelet::Ids;

  auto getLaneletLength(const lanelet::Id) const -> double;

  auto getLaneletPolygon(const lanelet::Id) const -> std::vector<geometry_msgs::msg::Point>;

  auto getLanelets(const lanelet::Ids &) const -> lanelet::Lanelets;

  auto getLateralDistance(
    const traffic_simulator_msgs::msg::LaneletPose & from,
    const traffic_simulator_msgs::msg::LaneletPose & to, bool allow_lane_change = false) const
    -> std::optional<double>;

  auto getLeftBound(const lanelet::Id) const -> std::vector<geometry_msgs::msg::Point>;

  auto getLeftLaneletIds(
    const lanelet::Id, const traffic_simulator_msgs::msg::EntityType &,
    const bool include_opposite_direction = true) const -> lanelet::Ids;

  auto getLongitudinalDistance(
    const traffic_simulator_msgs::msg::LaneletPose & from,
    const traffic_simulator_msgs::msg::LaneletPose & to, bool allow_lane_change = false) const
    -> std::optional<double>;

  auto getNearbyLaneletIds(
    const geometry_msgs::msg::Point &, const double distance_threshold,
    const bool include_crosswalk, const std::size_t search_count = 5) const -> lanelet::Ids;

  auto getNearbyLaneletIds(
    const geometry_msgs::msg::Point &, const double distance_threshold,
    const std::size_t search_count = 5) const -> lanelet::Ids;

  auto getNextLaneletIds(const lanelet::Ids &) const -> lanelet::Ids;

  auto getNextLaneletIds(const lanelet::Ids &, const std::string & turn_direction) const
    -> lanelet::Ids;

  auto getNextLaneletIds(const lanelet::Id) const -> lanelet::Ids;

  auto getNextLaneletIds(const lanelet::Id, const std::string & turn_direction) const
    -> lanelet::Ids;

  auto getPreviousLaneletIds(const lanelet::Ids &) const -> lanelet::Ids;

  auto getPreviousLaneletIds(const lanelet::Ids &, const std::string & turn_direction) const
    -> lanelet::Ids;

  auto getPreviousLaneletIds(const lanelet::Id) const -> lanelet::Ids;

  auto getPreviousLaneletIds(const lanelet::Id, const std::string & turn_direction) const
    -> lanelet::Ids;

  auto getPreviousLanelets(const lanelet::Id, const double distance = 100) const -> lanelet::Ids;

  auto getRightBound(const lanelet::Id) const -> std::vector<geometry_msgs::msg::Point>;

  auto getRightLaneletIds(
    lanelet::Id, traffic_simulator_msgs::msg::EntityType,
    bool include_opposite_direction = true) const -> lanelet::Ids;

  auto getRightOfWayLaneletIds(const lanelet::Ids &) const
    -> std::unordered_map<lanelet::Id, lanelet::Ids>;

  auto getRightOfWayLaneletIds(const lanelet::Id) const -> lanelet::Ids;

  auto getRoute(const lanelet::Id from, const lanelet::Id to, bool allow_lane_change = false) const
    -> lanelet::Ids;

  auto getSpeedLimit(const lanelet::Ids &) const -> double;

  auto getStopLineIds() const -> lanelet::Ids;

  auto getStopLineIdsOnPath(const lanelet::Ids & route_lanelets) const -> lanelet::Ids;

  auto getStopLinePolygon(const lanelet::Id) const -> std::vector<geometry_msgs::msg::Point>;

  auto getTangentVector(const lanelet::Id, const double s) const
    -> std::optional<geometry_msgs::msg::Vector3>;

  auto getTrafficLightBulbPosition(const lanelet::Id traffic_light_id, const std::string &) const
    -> std::optional<geometry_msgs::msg::Point>;

  auto getTrafficLightIds() const -> lanelet::Ids;

  auto getTrafficLightIdsOnPath(const lanelet::Ids & route_lanelets) const -> lanelet::Ids;

  auto getTrafficLightRegulatoryElement(const lanelet::Id) const -> lanelet::TrafficLight::Ptr;

  auto getTrafficLightRegulatoryElementIDsFromTrafficLight(const lanelet::Id) const -> lanelet::Ids;

  auto getTrafficLightStopLineIds(const lanelet::Id traffic_light_id) const -> lanelet::Ids;

  auto getTrafficLightStopLinesPoints(const lanelet::Id traffic_light_id) const
    -> std::vector<std::vector<geometry_msgs::msg::Point>>;

  auto insertMarkerArray(
    visualization_msgs::msg::MarkerArray &, const visualization_msgs::msg::MarkerArray &) const
    -> void;

  auto isInLanelet(const lanelet::Id, const double s) const -> bool;

  auto isInRoute(const lanelet::Id, const lanelet::Ids & route) const -> bool;

  auto isTrafficLight(const lanelet::Id) const -> bool;

  auto isTrafficLightRegulatoryElement(const lanelet::Id) const -> bool;

  auto matchToLane(
    const geometry_msgs::msg::Pose &, const traffic_simulator_msgs::msg::BoundingBox &,
    const bool include_crosswalk, const double matching_distance = 1.0,
    const double reduction_ratio = 0.8) const -> std::optional<lanelet::Id>;

  auto toLaneletPose(
    const geometry_msgs::msg::Pose &, const bool include_crosswalk,
    const double matching_distance = 1.0) const
    -> std::optional<traffic_simulator_msgs::msg::LaneletPose>;

  auto toLaneletPose(
    const geometry_msgs::msg::Pose &, const lanelet::Ids &,
    const double matching_distance = 1.0) const
    -> std::optional<traffic_simulator_msgs::msg::LaneletPose>;

  auto toLaneletPose(
    const geometry_msgs::msg::Point &, const traffic_simulator_msgs::msg::BoundingBox &,
    const bool include_crosswalk, const double matching_distance = 1.0) const
    -> std::optional<traffic_simulator_msgs::msg::LaneletPose>;

  auto toLaneletPose(
    const geometry_msgs::msg::Pose &, const traffic_simulator_msgs::msg::BoundingBox &,
    const bool include_crosswalk, const double matching_distance = 1.0) const
    -> std::optional<traffic_simulator_msgs::msg::LaneletPose>;

  auto toLaneletPose(
    const geometry_msgs::msg::Pose &, const lanelet::Id, const double matching_distance = 1.0) const
    -> std::optional<traffic_simulator_msgs::msg::LaneletPose>;

  auto toLaneletPoses(
    const geometry_msgs::msg::Pose &, const lanelet::Id, const double matching_distance = 5.0,
    const bool include_opposite_direction = true) const
    -> std::vector<traffic_simulator_msgs::msg::LaneletPose>;

  auto toMapBin() const -> autoware_auto_mapping_msgs::msg::HADMapBin;

  auto toMapPoints(const lanelet::Id, const std::vector<double> & s) const
    -> std::vector<geometry_msgs::msg::Point>;

  auto toMapPose(const traffic_simulator_msgs::msg::LaneletPose &, const bool fill_pitch = true)
    const -> geometry_msgs::msg::PoseStamped;

private:
  /** @defgroup cache
   *  Declared mutable for caching
   */
  // @{
  mutable RouteCache route_cache_;
  mutable CenterPointsCache center_points_cache_;
  mutable LaneletLengthCache lanelet_length_cache_;
  // @}

  lanelet::LaneletMapPtr lanelet_map_ptr_;
  lanelet::routing::RoutingGraphConstPtr vehicle_routing_graph_ptr_;
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules_vehicle_ptr_;
  lanelet::routing::RoutingGraphConstPtr pedestrian_routing_graph_ptr_;
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules_pedestrian_ptr_;
  lanelet::ConstLanelets shoulder_lanelets_;

  template <typename Lanelet>
  auto getLaneletIds(const std::vector<Lanelet> & lanelets) const -> lanelet::Ids
  {
    lanelet::Ids ids;
    std::transform(
      lanelets.begin(), lanelets.end(), std::back_inserter(ids),
      [](const auto & lanelet) { return lanelet.id(); });
    return ids;
  }

  auto absoluteHull(
    const lanelet::BasicPolygon2d & relative_hull, const lanelet::matching::Pose2d &) const
    -> lanelet::BasicPolygon2d;

  auto calcEuclidDist(
    const std::vector<double> & x, const std::vector<double> & y,
    const std::vector<double> & z) const -> std::vector<double>;

  auto calculateAccumulatedLengths(const lanelet::ConstLineString3d &) const -> std::vector<double>;

  auto calculateSegmentDistances(const lanelet::ConstLineString3d &) const -> std::vector<double>;

  auto excludeSubtypeLanelets(
    const std::vector<std::pair<double, lanelet::Lanelet>> &, const char subtype[]) const
    -> std::vector<std::pair<double, lanelet::Lanelet>>;

  auto filterLanelets(const lanelet::Lanelets &, const char subtype[]) const -> lanelet::Lanelets;

  auto findNearestIndexPair(
    const std::vector<double> & accumulated_lengths, const double target_length) const
    -> std::pair<std::size_t, std::size_t>;

  auto generateFineCenterline(const lanelet::ConstLanelet &, const double resolution) const
    -> lanelet::LineString3d;

  auto getLaneChangeTrajectory(
    const geometry_msgs::msg::Pose & from, const traffic_simulator_msgs::msg::LaneletPose & to,
    const traffic_simulator::lane_change::TrajectoryShape,
    const double tangent_vector_size = 100) const -> math::geometry::HermiteCurve;

  auto getNextRoadShoulderLanelet(const lanelet::Id) const -> lanelet::Ids;

  auto getPreviousRoadShoulderLanelet(const lanelet::Id) const -> lanelet::Ids;

  auto getStopLines() const -> lanelet::ConstLineStrings3d;

  auto getStopLinesOnPath(const lanelet::Ids &) const -> lanelet::ConstLineStrings3d;

  auto getTrafficLightRegulatoryElementsOnPath(const lanelet::Ids &) const
    -> std::vector<std::shared_ptr<const lanelet::autoware::AutowareTrafficLight>>;

  auto getTrafficLights(const lanelet::Id traffic_light_id) const
    -> std::vector<lanelet::AutowareTrafficLightConstPtr>;

  auto getTrafficSignRegulatoryElementsOnPath(const lanelet::Ids &) const
    -> std::vector<std::shared_ptr<const lanelet::TrafficSign>>;

  auto getTrafficSignRegulatoryElements() const
    -> std::vector<std::shared_ptr<const lanelet::TrafficSign>>;

  auto getVectorFromPose(const geometry_msgs::msg::Pose &, const double magnitude) const
    -> geometry_msgs::msg::Vector3;

  auto mapCallback(const autoware_auto_mapping_msgs::msg::HADMapBin &) const -> void;

  auto overwriteLaneletsCenterline() -> void;

  auto resamplePoints(const lanelet::ConstLineString3d &, const std::int32_t num_segments) const
    -> lanelet::BasicPoints3d;

  auto toPoint2d(const geometry_msgs::msg::Point &) const -> lanelet::BasicPoint2d;

  auto toPolygon(const lanelet::ConstLineString3d &) const
    -> std::vector<geometry_msgs::msg::Point>;
};
}  // namespace hdmap_utils

#endif  // TRAFFIC_SIMULATOR__HDMAP_UTILS__HDMAP_UTILS_HPP_
