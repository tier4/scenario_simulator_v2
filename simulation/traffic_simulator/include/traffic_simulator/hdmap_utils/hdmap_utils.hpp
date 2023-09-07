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
#ifdef USE_TF2_GEOMETRY_MSGS_DEPRECATED_HEADER
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <boost/filesystem.hpp>
#include <geographic_msgs/msg/geo_point.hpp>
#include <geometry/spline/catmull_rom_spline.hpp>
#include <geometry/spline/catmull_rom_spline_interface.hpp>
#include <geometry/spline/hermite_curve.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <map>
#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <string>
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

  auto gelAllCanonicalizedLaneletPoses(
    const traffic_simulator_msgs::msg::LaneletPose & lanelet_pose) const
    -> std::vector<traffic_simulator_msgs::msg::LaneletPose>;
  auto canonicalizeLaneletPose(const traffic_simulator_msgs::msg::LaneletPose & lanelet_pose) const
    -> std::tuple<
      std::optional<traffic_simulator_msgs::msg::LaneletPose>, std::optional<lanelet::Id>>;
  auto canonicalizeLaneletPose(
    const traffic_simulator_msgs::msg::LaneletPose & lanelet_pose,
    const lanelet::Ids & route_lanelets) const
    -> std::tuple<
      std::optional<traffic_simulator_msgs::msg::LaneletPose>, std::optional<lanelet::Id>>;

  autoware_auto_mapping_msgs::msg::HADMapBin toMapBin() const;
  void insertMarkerArray(
    visualization_msgs::msg::MarkerArray & a1,
    const visualization_msgs::msg::MarkerArray & a2) const;
  std::vector<geometry_msgs::msg::Point> toMapPoints(
    lanelet::Id lanelet_id, const std::vector<double> & s) const;
  std::optional<traffic_simulator_msgs::msg::LaneletPose> toLaneletPose(
    const geometry_msgs::msg::Pose & pose, bool include_crosswalk,
    double matching_distance = 1.0) const;
  std::optional<traffic_simulator_msgs::msg::LaneletPose> toLaneletPose(
    const geometry_msgs::msg::Pose & pose, const traffic_simulator_msgs::msg::BoundingBox & bbox,
    bool include_crosswalk, double matching_distance = 1.0) const;
  std::optional<traffic_simulator_msgs::msg::LaneletPose> toLaneletPose(
    const geometry_msgs::msg::Pose & pose, lanelet::Id lanelet_id,
    double matching_distance = 1.0) const;
  std::optional<traffic_simulator_msgs::msg::LaneletPose> toLaneletPose(
    const geometry_msgs::msg::Pose & pose, const lanelet::Ids & lanelet_ids,
    double matching_distance = 1.0) const;
  std::vector<traffic_simulator_msgs::msg::LaneletPose> toLaneletPoses(
    const geometry_msgs::msg::Pose & pose, lanelet::Id lanelet_id, double matching_distance = 5.0,
    bool include_opposite_direction = true) const;
  std::optional<lanelet::Id> matchToLane(
    const geometry_msgs::msg::Pose & pose, const traffic_simulator_msgs::msg::BoundingBox & bbox,
    bool include_crosswalk, double reduction_ratio = 0.8) const;
  geometry_msgs::msg::PoseStamped toMapPose(
    const traffic_simulator_msgs::msg::LaneletPose & lanelet_pose) const;
  double getHeight(const traffic_simulator_msgs::msg::LaneletPose & lanelet_pose) const;
  lanelet::Ids getLaneletIds() const;
  lanelet::Ids getNextLaneletIds(lanelet::Id lanelet_id, const std::string & turn_direction) const;
  lanelet::Ids getNextLaneletIds(lanelet::Id lanelet_id) const;
  lanelet::Ids getNextLaneletIds(
    const lanelet::Ids & lanelet_id, const std::string & turn_direction) const;
  lanelet::Ids getNextLaneletIds(const lanelet::Ids & lanelet_id) const;
  lanelet::Ids getPreviousLaneletIds(
    lanelet::Id lanelet_id, const std::string & turn_direction) const;
  lanelet::Ids getPreviousLaneletIds(lanelet::Id lanelet_id) const;
  lanelet::Ids getPreviousLaneletIds(
    const lanelet::Ids & lanelet_ids, const std::string & turn_direction) const;
  lanelet::Ids getPreviousLaneletIds(const lanelet::Ids & lanelet_ids) const;
  std::optional<int64_t> getLaneChangeableLaneletId(
    lanelet::Id lanelet_id, traffic_simulator::lane_change::Direction direction) const;
  std::optional<int64_t> getLaneChangeableLaneletId(
    lanelet::Id lanelet_id, traffic_simulator::lane_change::Direction direction,
    uint8_t shift) const;
  std::optional<double> getDistanceToStopLine(
    const lanelet::Ids & route_lanelets,
    const std::vector<geometry_msgs::msg::Point> & waypoints) const;
  std::optional<double> getDistanceToStopLine(
    const lanelet::Ids & route_lanelets,
    const math::geometry::CatmullRomSplineInterface & spline) const;
  double getLaneletLength(lanelet::Id lanelet_id) const;
  bool isInLanelet(lanelet::Id lanelet_id, double s) const;
  std::optional<double> getLateralDistance(
    const traffic_simulator_msgs::msg::LaneletPose & from,
    const traffic_simulator_msgs::msg::LaneletPose & to) const;
  std::optional<double> getLongitudinalDistance(
    const traffic_simulator_msgs::msg::LaneletPose & from,
    const traffic_simulator_msgs::msg::LaneletPose & to) const;
  double getSpeedLimit(const lanelet::Ids & lanelet_ids) const;
  bool isInRoute(lanelet::Id lanelet_id, const lanelet::Ids & route) const;
  lanelet::Ids getFollowingLanelets(
    lanelet::Id lanelet_id, double distance = 100, bool include_self = true) const;
  lanelet::Ids getFollowingLanelets(
    lanelet::Id lanelet_id, const lanelet::Ids & candidate_lanelet_ids, double distance = 100,
    bool include_self = true) const;
  lanelet::Ids getPreviousLanelets(lanelet::Id lanelet_id, double distance = 100) const;
  std::vector<geometry_msgs::msg::Point> getCenterPoints(lanelet::Id lanelet_id) const;
  std::vector<geometry_msgs::msg::Point> getCenterPoints(const lanelet::Ids & lanelet_ids) const;
  std::shared_ptr<math::geometry::CatmullRomSpline> getCenterPointsSpline(
    lanelet::Id lanelet_id) const;
  std::vector<geometry_msgs::msg::Point> clipTrajectoryFromLaneletIds(
    lanelet::Id lanelet_id, double s, const lanelet::Ids & lanelet_ids,
    double forward_distance = 20) const;
  bool canChangeLane(lanelet::Id from_lanelet_id, lanelet::Id to_lanelet_id) const;
  std::optional<std::pair<math::geometry::HermiteCurve, double>> getLaneChangeTrajectory(
    const traffic_simulator_msgs::msg::LaneletPose & from_pose,
    const traffic_simulator::lane_change::Parameter & lane_change_parameter) const;
  std::optional<std::pair<math::geometry::HermiteCurve, double>> getLaneChangeTrajectory(
    const geometry_msgs::msg::Pose & from_pose,
    const traffic_simulator::lane_change::Parameter & lane_change_parameter,
    double maximum_curvature_threshold, double target_trajectory_length,
    double forward_distance_threshold) const;
  std::optional<geometry_msgs::msg::Vector3> getTangentVector(
    lanelet::Id lanelet_id, double s) const;
  lanelet::Ids getRoute(lanelet::Id from_lanelet_id, lanelet::Id to_lanelet_id) const;
  lanelet::Ids getConflictingCrosswalkIds(const lanelet::Ids & lanelet_ids) const;
  lanelet::Ids getConflictingLaneIds(const lanelet::Ids & lanelet_ids) const;
  std::optional<double> getCollisionPointInLaneCoordinate(
    lanelet::Id lanelet_id, lanelet::Id crossing_lanelet_id) const;
  visualization_msgs::msg::MarkerArray generateMarker() const;
  lanelet::Ids getRightOfWayLaneletIds(lanelet::Id lanelet_id) const;
  std::unordered_map<lanelet::Id, lanelet::Ids> getRightOfWayLaneletIds(
    const lanelet::Ids & lanelet_ids) const;
  std::optional<lanelet::Id> getClosestLaneletId(
    const geometry_msgs::msg::Pose & pose, double distance_thresh = 30.0,
    bool include_crosswalk = false) const;

  auto getNearbyLaneletIds(
    const geometry_msgs::msg::Point & point, double distance_threshold,
    std::size_t search_count = 5) const -> lanelet::Ids;

  auto getNearbyLaneletIds(
    const geometry_msgs::msg::Point & point, double distance_threshold, bool include_crosswalk,
    std::size_t search_count = 5) const -> lanelet::Ids;

  lanelet::Ids filterLaneletIds(const lanelet::Ids & lanelet_ids, const char subtype[]) const;
  std::vector<geometry_msgs::msg::Point> getLaneletPolygon(lanelet::Id lanelet_id) const;
  std::vector<geometry_msgs::msg::Point> getStopLinePolygon(lanelet::Id lanelet_id) const;
  std::vector<int64_t> getStopLineIdsOnPath(const lanelet::Ids & route_lanelets) const;
  lanelet::Ids getTrafficLightIds() const;
  std::optional<geometry_msgs::msg::Point> getTrafficLightBulbPosition(
    lanelet::Id traffic_light_id, const std::string &) const;
  lanelet::Ids getTrafficLightStopLineIds(const lanelet::Id traffic_light_id) const;
  std::vector<std::vector<geometry_msgs::msg::Point>> getTrafficLightStopLinesPoints(
    lanelet::Id traffic_light_id) const;
  std::optional<double> getDistanceToTrafficLightStopLine(
    const std::vector<geometry_msgs::msg::Point> & waypoints,
    const lanelet::Id traffic_light_id) const;
  std::optional<double> getDistanceToTrafficLightStopLine(
    const math::geometry::CatmullRomSplineInterface & spline,
    const lanelet::Id traffic_light_id) const;
  std::optional<double> getDistanceToTrafficLightStopLine(
    const lanelet::Ids & route_lanelets,
    const std::vector<geometry_msgs::msg::Point> & waypoints) const;
  std::optional<double> getDistanceToTrafficLightStopLine(
    const lanelet::Ids & route_lanelets,
    const math::geometry::CatmullRomSplineInterface & spline) const;
  lanelet::Ids getTrafficLightIdsOnPath(const lanelet::Ids & route_lanelets) const;
  traffic_simulator_msgs::msg::LaneletPose getAlongLaneletPose(
    const traffic_simulator_msgs::msg::LaneletPose & from_pose, double along) const;
  std::vector<geometry_msgs::msg::Point> getLeftBound(lanelet::Id lanelet_id) const;
  std::vector<geometry_msgs::msg::Point> getRightBound(lanelet::Id lanelet_id) const;

  auto getLeftLaneletIds(
    lanelet::Id lanelet_id, traffic_simulator_msgs::msg::EntityType type,
    bool include_opposite_direction = true) const -> lanelet::Ids;

  auto getRightLaneletIds(
    lanelet::Id lanelet_id, traffic_simulator_msgs::msg::EntityType type,
    bool include_opposite_direction = true) const -> lanelet::Ids;

  auto isTrafficLight(const lanelet::Id) const -> bool;

  auto isTrafficLightRegulatoryElement(const lanelet::Id) const -> bool;

  auto getTrafficLightRegulatoryElement(const lanelet::Id) const -> lanelet::TrafficLight::Ptr;

  auto getTrafficLightRegulatoryElementIDsFromTrafficLight(const lanelet::Id) const -> lanelet::Ids;

private:
  math::geometry::HermiteCurve getLaneChangeTrajectory(
    const geometry_msgs::msg::Pose & from_pose,
    const traffic_simulator_msgs::msg::LaneletPose & to_pose,
    const traffic_simulator::lane_change::TrajectoryShape trajectory_shape,
    double tangent_vector_size = 100) const;
  /** @defgroup cache
   *  Declared mutable for caching
   */
  // @{
  mutable RouteCache route_cache_;
  mutable CenterPointsCache center_points_cache_;
  mutable LaneletLengthCache lanelet_length_cache_;
  // @}

  template <typename Lanelet>
  lanelet::Ids getLaneletIds(const std::vector<Lanelet> & lanelets) const
  {
    lanelet::Ids ids;
    std::transform(
      lanelets.begin(), lanelets.end(), std::back_inserter(ids),
      [](const Lanelet & lanelet) { return lanelet.id(); });
    return ids;
  }
  std::vector<lanelet::AutowareTrafficLightConstPtr> getTrafficLights(
    const lanelet::Id traffic_light_id) const;
  std::vector<std::pair<double, lanelet::Lanelet>> excludeSubtypeLanelets(
    const std::vector<std::pair<double, lanelet::Lanelet>> & lls, const char subtype[]) const;
  std::vector<lanelet::Lanelet> filterLanelets(
    const std::vector<lanelet::Lanelet> & lanelets, const char subtype[]) const;
  std::vector<std::shared_ptr<const lanelet::autoware::AutowareTrafficLight>>
  getTrafficLightRegElementsOnPath(const lanelet::Ids & lanelet_ids) const;
  std::vector<std::shared_ptr<const lanelet::TrafficSign>> getTrafficSignRegElementsOnPath(
    const lanelet::Ids & lanelet_ids) const;
  std::vector<lanelet::ConstLineString3d> getStopLinesOnPath(
    const lanelet::Ids & lanelet_ids) const;
  geometry_msgs::msg::Vector3 getVectorFromPose(
    const geometry_msgs::msg::Pose & pose, double magnitude) const;
  void mapCallback(const autoware_auto_mapping_msgs::msg::HADMapBin & msg) const;
  lanelet::LaneletMapPtr lanelet_map_ptr_;
  lanelet::routing::RoutingGraphConstPtr vehicle_routing_graph_ptr_;
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules_vehicle_ptr_;
  lanelet::routing::RoutingGraphConstPtr pedestrian_routing_graph_ptr_;
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules_pedestrian_ptr_;
  std::vector<double> calcEuclidDist(
    const std::vector<double> & x, const std::vector<double> & y,
    const std::vector<double> & z) const;
  void overwriteLaneletsCenterline();
  lanelet::LineString3d generateFineCenterline(
    const lanelet::ConstLanelet & lanelet_obj, const double resolution) const;
  std::vector<lanelet::BasicPoint3d> resamplePoints(
    const lanelet::ConstLineString3d & line_string, const int32_t num_segments) const;
  std::pair<size_t, size_t> findNearestIndexPair(
    const std::vector<double> & accumulated_lengths, const double target_length) const;
  std::vector<double> calculateAccumulatedLengths(
    const lanelet::ConstLineString3d & line_string) const;
  std::vector<double> calculateSegmentDistances(
    const lanelet::ConstLineString3d & line_string) const;
  std::vector<lanelet::Lanelet> getLanelets(const lanelet::Ids & lanelet_ids) const;
  lanelet::BasicPoint2d toPoint2d(const geometry_msgs::msg::Point & point) const;
  lanelet::BasicPolygon2d absoluteHull(
    const lanelet::BasicPolygon2d & relativeHull, const lanelet::matching::Pose2d & pose) const;
  std::vector<geometry_msgs::msg::Point> toPolygon(
    const lanelet::ConstLineString3d & line_string) const;
  lanelet::ConstLanelets shoulder_lanelets_;
  lanelet::Ids getNextRoadShoulderLanelet(lanelet::Id lanelet_id) const;
  lanelet::Ids getPreviousRoadShoulderLanelet(lanelet::Id lanelet_id) const;
};
}  // namespace hdmap_utils

#endif  // TRAFFIC_SIMULATOR__HDMAP_UTILS__HDMAP_UTILS_HPP_
