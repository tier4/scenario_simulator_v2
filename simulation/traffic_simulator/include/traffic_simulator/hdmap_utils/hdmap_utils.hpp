// Copyright 2015-2020 Tier IV, Inc. All rights reserved.
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
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <boost/filesystem.hpp>
#include <boost/optional.hpp>
#include <geographic_msgs/msg/geo_point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <lanelet2_extension_psim/utility/message_conversion.hpp>
#include <lanelet2_extension_psim/utility/query.hpp>
#include <lanelet2_extension_psim/utility/utilities.hpp>
#include <map>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <traffic_simulator/data_type/data_types.hpp>
#include <traffic_simulator/hdmap_utils/cache.hpp>
#include <traffic_simulator/math/catmull_rom_interface.hpp>
#include <traffic_simulator/math/catmull_rom_spline.hpp>
#include <traffic_simulator/math/hermite_curve.hpp>
#include <traffic_simulator/traffic_lights/traffic_light_state.hpp>
#include <traffic_simulator_msgs/msg/bounding_box.hpp>
#include <traffic_simulator_msgs/msg/entity_status.hpp>
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

  const autoware_auto_mapping_msgs::msg::HADMapBin toMapBin();
  void insertMarkerArray(
    visualization_msgs::msg::MarkerArray & a1,
    const visualization_msgs::msg::MarkerArray & a2) const;
  std::vector<geometry_msgs::msg::Point> toMapPoints(
    std::int64_t lanelet_id, std::vector<double> s);
  boost::optional<traffic_simulator_msgs::msg::LaneletPose> toLaneletPose(
    geometry_msgs::msg::Pose pose, bool include_crosswalk, double matching_distance = 1.0);
  boost::optional<traffic_simulator_msgs::msg::LaneletPose> toLaneletPose(
    geometry_msgs::msg::Pose pose, const traffic_simulator_msgs::msg::BoundingBox & bbox,
    bool include_crosswalk, double matching_distance = 1.0);
  boost::optional<traffic_simulator_msgs::msg::LaneletPose> toLaneletPose(
    geometry_msgs::msg::Pose pose, std::int64_t lanelet_id, double matching_distance = 1.0);
  boost::optional<traffic_simulator_msgs::msg::LaneletPose> toLaneletPose(
    geometry_msgs::msg::Pose pose, std::vector<std::int64_t> lanelet_ids,
    double matching_distance = 1.0);
  boost::optional<std::int64_t> matchToLane(
    const geometry_msgs::msg::Pose & pose, const traffic_simulator_msgs::msg::BoundingBox & bbox,
    bool include_crosswalk, double reduction_ratio = 0.8);
  geometry_msgs::msg::PoseStamped toMapPose(
    std::int64_t lanelet_id, double s, double offset, geometry_msgs::msg::Quaternion quat);
  geometry_msgs::msg::PoseStamped toMapPose(traffic_simulator_msgs::msg::LaneletPose lanelet_pose);
  geometry_msgs::msg::PoseStamped toMapPose(std::int64_t lanelet_id, double s, double offset);
  double getHeight(const traffic_simulator_msgs::msg::LaneletPose & lanelet_pose);
  const std::vector<std::int64_t> getLaneletIds();
  std::vector<std::int64_t> getNextLaneletIds(std::int64_t lanelet_id, std::string turn_direction);
  std::vector<std::int64_t> getNextLaneletIds(std::int64_t lanelet_id) const;
  std::vector<std::int64_t> getPreviousLaneletIds(
    std::int64_t lanelet_id, std::string turn_direction);
  std::vector<std::int64_t> getPreviousLaneletIds(std::int64_t lanelet_id) const;
  boost::optional<int64_t> getLaneChangeableLaneletId(
    std::int64_t lanelet_id, traffic_simulator::lane_change::Direction direction);
  boost::optional<int64_t> getLaneChangeableLaneletId(
    std::int64_t lanelet_id, traffic_simulator::lane_change::Direction direction, uint8_t shift);
  boost::optional<double> getDistanceToStopLine(
    const std::vector<std::int64_t> & route_lanelets,
    const std::vector<geometry_msgs::msg::Point> & waypoints);
  boost::optional<double> getDistanceToStopLine(
    const std::vector<std::int64_t> & route_lanelets,
    const traffic_simulator::math::CatmullRomInterface & spline);
  double getLaneletLength(std::int64_t lanelet_id);
  bool isInLanelet(std::int64_t lanelet_id, double s);
  boost::optional<double> getLongitudinalDistance(
    traffic_simulator_msgs::msg::LaneletPose from, traffic_simulator_msgs::msg::LaneletPose to);
  boost::optional<double> getLongitudinalDistance(
    std::int64_t from_lanelet_id, double from_s, std::int64_t to_lanelet_id, double to_s);
  double getSpeedLimit(std::vector<std::int64_t> lanelet_ids);
  bool isInRoute(std::int64_t lanelet_id, std::vector<std::int64_t> route) const;
  std::vector<std::int64_t> getFollowingLanelets(
    std::int64_t lanelet_id, double distance = 100, bool include_self = true);
  std::vector<std::int64_t> getFollowingLanelets(
    std::int64_t lanelet_id, std::vector<std::int64_t> candidate_lanelet_ids, double distance = 100,
    bool include_self = true);
  std::vector<std::int64_t> getPreviousLanelets(std::int64_t lanelet_id, double distance = 100);
  std::vector<geometry_msgs::msg::Point> getCenterPoints(std::int64_t lanelet_id);
  std::vector<geometry_msgs::msg::Point> getCenterPoints(std::vector<std::int64_t> lanelet_ids);
  std::shared_ptr<traffic_simulator::math::CatmullRomSpline> getCenterPointsSpline(
    std::int64_t lanelet_id);
  std::vector<geometry_msgs::msg::Point> clipTrajectoryFromLaneletIds(
    std::int64_t lanelet_id, double s, std::vector<std::int64_t> lanelet_ids,
    double forward_distance = 20);
  bool canChangeLane(std::int64_t from_lanelet_id, std::int64_t to_lanelet_id);
  boost::optional<std::pair<traffic_simulator::math::HermiteCurve, double>> getLaneChangeTrajectory(
    const traffic_simulator_msgs::msg::LaneletPose & from_pose,
    const traffic_simulator::lane_change::Parameter & lane_change_parameter);
  boost::optional<std::pair<traffic_simulator::math::HermiteCurve, double>> getLaneChangeTrajectory(
    const geometry_msgs::msg::Pose & from_pose,
    const traffic_simulator::lane_change::Parameter & lane_change_parameter,
    double maximum_curvature_threshold, double target_trajectory_length,
    double forward_distance_threshold);
  boost::optional<geometry_msgs::msg::Vector3> getTangentVector(std::int64_t lanelet_id, double s);
  std::vector<std::int64_t> getRoute(std::int64_t from_lanelet_id, std::int64_t to_lanelet_id);
  std::vector<std::int64_t> getConflictingCrosswalkIds(
    const std::vector<std::int64_t> & lanelet_ids) const;
  std::vector<std::int64_t> getConflictingLaneIds(
    const std::vector<std::int64_t> & lanelet_ids) const;
  boost::optional<double> getCollisionPointInLaneCoordinate(
    std::int64_t lanelet_id, std::int64_t crossing_lanelet_id);
  const visualization_msgs::msg::MarkerArray generateMarker() const;
  const std::vector<std::int64_t> getRightOfWayLaneletIds(std::int64_t lanelet_id) const;
  const std::unordered_map<std::int64_t, std::vector<std::int64_t>> getRightOfWayLaneletIds(
    std::vector<std::int64_t> lanelet_ids) const;
  boost::optional<std::int64_t> getClosestLaneletId(
    geometry_msgs::msg::Pose pose, double distance_thresh = 30.0, bool include_crosswalk = false);
  std::vector<std::int64_t> getNearbyLaneletIds(
    const geometry_msgs::msg::Point & point, double distance_threshold) const;
  std::vector<std::int64_t> getNearbyLaneletIds(
    const geometry_msgs::msg::Point & point, double distance_threshold,
    bool include_crosswalk) const;
  std::vector<std::int64_t> filterLaneletIds(
    const std::vector<std::int64_t> & lanelet_ids, const char subtype[]) const;
  const std::vector<geometry_msgs::msg::Point> getLaneletPolygon(std::int64_t lanelet_id);
  const std::vector<geometry_msgs::msg::Point> getStopLinePolygon(std::int64_t lanelet_id);
  std::vector<std::int64_t> getTrafficLightIds() const;
  const boost::optional<geometry_msgs::msg::Point> getTrafficLightBulbPosition(
    std::int64_t traffic_light_id, traffic_simulator::TrafficLightColor color) const;
  std::vector<std::int64_t> getTrafficLightStopLineIds(const std::int64_t & traffic_light_id) const;
  std::vector<std::vector<geometry_msgs::msg::Point>> getTrafficLightStopLinesPoints(
    std::int64_t traffic_light_id) const;
  const boost::optional<double> getDistanceToTrafficLightStopLine(
    const std::vector<geometry_msgs::msg::Point> & waypoints,
    const std::int64_t & traffic_light_id) const;
  const boost::optional<double> getDistanceToTrafficLightStopLine(
    const traffic_simulator::math::CatmullRomInterface & spline,
    const std::int64_t & traffic_light_id) const;
  const boost::optional<double> getDistanceToTrafficLightStopLine(
    const std::vector<std::int64_t> & route_lanelets,
    const std::vector<geometry_msgs::msg::Point> & waypoints) const;
  const boost::optional<double> getDistanceToTrafficLightStopLine(
    const std::vector<std::int64_t> & route_lanelets,
    const traffic_simulator::math::CatmullRomInterface & spline) const;
  const std::vector<std::int64_t> getTrafficLightIdsOnPath(
    const std::vector<std::int64_t> & route_lanelets) const;
  traffic_simulator_msgs::msg::LaneletPose getAlongLaneletPose(
    const traffic_simulator_msgs::msg::LaneletPose & from_pose, double along);
  auto isTrafficRelationId(const std::int64_t) const -> bool;
  auto getTrafficLight(const std::int64_t) const -> lanelet::TrafficLight::Ptr;

private:
  traffic_simulator::math::HermiteCurve getLaneChangeTrajectory(
    const geometry_msgs::msg::Pose & from_pose,
    const traffic_simulator_msgs::msg::LaneletPose & to_pose,
    const traffic_simulator::lane_change::TrajectoryShape trajectory_shape,
    double tangent_vector_size = 100);
  RouteCache route_cache_;
  CenterPointsCache center_points_cache_;
  LaneletLengthCache lanelet_length_cache_;
  std::vector<lanelet::AutowareTrafficLightConstPtr> getTrafficLights(
    const std::int64_t traffic_light_id) const;
  std::vector<std::pair<double, lanelet::Lanelet>> excludeSubtypeLanelets(
    const std::vector<std::pair<double, lanelet::Lanelet>> & lls, const char subtype[]) const;
  std::vector<lanelet::Lanelet> filterLanelets(
    const std::vector<lanelet::Lanelet> & lanelets, const char subtype[]) const;
  std::vector<std::shared_ptr<const lanelet::autoware::AutowareTrafficLight>>
  getTrafficLightRegElementsOnPath(const std::vector<std::int64_t> & lanelet_ids) const;
  std::vector<std::shared_ptr<const lanelet::TrafficSign>> getTrafficSignRegElementsOnPath(
    std::vector<std::int64_t> lanelet_ids) const;
  std::vector<lanelet::ConstLineString3d> getStopLinesOnPath(std::vector<std::int64_t> lanelet_ids);
  geometry_msgs::msg::Vector3 getVectorFromPose(geometry_msgs::msg::Pose pose, double magnitude);
  void mapCallback(const autoware_auto_mapping_msgs::msg::HADMapBin & msg);
  lanelet::LaneletMapPtr lanelet_map_ptr_;
  lanelet::routing::RoutingGraphConstPtr vehicle_routing_graph_ptr_;
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules_vehicle_ptr_;
  lanelet::routing::RoutingGraphConstPtr pedestrian_routing_graph_ptr_;
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules_pedestrian_ptr_;
  std::vector<double> calcEuclidDist(
    const std::vector<double> & x, const std::vector<double> & y, const std::vector<double> & z);
  void overwriteLaneletsCenterline();
  lanelet::LineString3d generateFineCenterline(
    const lanelet::ConstLanelet & lanelet_obj, const double resolution);
  std::vector<lanelet::BasicPoint3d> resamplePoints(
    const lanelet::ConstLineString3d & line_string, const int32_t num_segments);
  std::pair<size_t, size_t> findNearestIndexPair(
    const std::vector<double> & accumulated_lengths, const double target_length);
  std::vector<double> calculateAccumulatedLengths(const lanelet::ConstLineString3d & line_string);
  std::vector<double> calculateSegmentDistances(const lanelet::ConstLineString3d & line_string);
  std::vector<lanelet::Lanelet> getLanelets(const std::vector<std::int64_t> & lanelet_ids) const;
  std::vector<std::int64_t> getLaneletIds(const std::vector<lanelet::Lanelet> & lanelets) const;
  lanelet::BasicPoint2d toPoint2d(const geometry_msgs::msg::Point & point) const;
  lanelet::BasicPolygon2d absoluteHull(
    const lanelet::BasicPolygon2d & relativeHull, const lanelet::matching::Pose2d & pose) const;
};
}  // namespace hdmap_utils

#endif  // TRAFFIC_SIMULATOR__HDMAP_UTILS__HDMAP_UTILS_HPP_
