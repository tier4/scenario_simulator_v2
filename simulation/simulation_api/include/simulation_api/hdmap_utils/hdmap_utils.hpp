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

#ifndef SIMULATION_API__HDMAP_UTILS__HDMAP_UTILS_HPP_
#define SIMULATION_API__HDMAP_UTILS__HDMAP_UTILS_HPP_

#include <simulation_api/math/hermite_curve.hpp>
#include <simulation_api/traffic_lights/traffic_light_state.hpp>

#include <rclcpp/rclcpp.hpp>
#include <openscenario_msgs/msg/entity_status.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <autoware_auto_msgs/msg/had_map_bin.hpp>
#include <geometry_msgs/msg/vector3.h>
#include <geographic_msgs/msg/geo_point.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_core/primitives/LaneletSequence.h>
#include <lanelet2_extension_psim/regulatory_elements/autoware_traffic_light.hpp>
#include <lanelet2_extension_psim/utility/message_conversion.hpp>
#include <lanelet2_extension_psim/utility/utilities.hpp>
#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingCost.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_routing/RoutingGraphContainer.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <boost/optional.hpp>

#include <map>
#include <unordered_map>
#include <string>
#include <utility>
#include <vector>
#include <memory>


namespace hdmap_utils
{
class HdMapError : public std::runtime_error
{
public:
  explicit HdMapError(const char * message)
  : runtime_error(message) {}
};

class HdMapUtils
{
public:
  explicit HdMapUtils(std::string lanelet_path, geographic_msgs::msg::GeoPoint origin);
  const autoware_auto_msgs::msg::HADMapBin toMapBin();
  void insertMarkerArray(
    visualization_msgs::msg::MarkerArray & a1,
    const visualization_msgs::msg::MarkerArray & a2) const;
  std::vector<geometry_msgs::msg::Point> toMapPoints(
    std::int64_t lanelet_id,
    std::vector<double> s);
  boost::optional<openscenario_msgs::msg::LaneletPose> toLaneletPose(geometry_msgs::msg::Pose pose);
  geometry_msgs::msg::PoseStamped toMapPose(
    std::int64_t lanelet_id, double s,
    double offset,
    geometry_msgs::msg::Quaternion quat);
  geometry_msgs::msg::PoseStamped toMapPose(
    openscenario_msgs::msg::LaneletPose lanlet_pose);
  geometry_msgs::msg::PoseStamped toMapPose(
    std::int64_t lanelet_id, double s,
    double offset);
  std::vector<std::int64_t> getNextLaneletIds(std::int64_t lanelet_id, std::string turn_direction);
  std::vector<std::int64_t> getNextLaneletIds(std::int64_t lanelet_id) const;
  std::vector<std::int64_t> getPreviousLaneletIds(
    std::int64_t lanelet_id,
    std::string turn_direction);
  std::vector<std::int64_t> getPreviousLaneletIds(std::int64_t lanelet_id) const;
  boost::optional<int> getLaneChangeableLenletId(std::int64_t lanelet_id, std::string direction);
  boost::optional<double> getDistanceToStopLine(
    std::vector<std::int64_t> following_lanelets, std::int64_t lanelet_id,
    double s);
  boost::optional<double> getDistanceToStopLine(
    std::vector<std::int64_t> following_lanelets, openscenario_msgs::msg::LaneletPose lanlet_pose);
  double getLaneletLength(std::int64_t lanelet_id) const;
  bool isInLanelet(std::int64_t lanelet_id, double s);
  boost::optional<double> getLongitudinalDistance(
    openscenario_msgs::msg::LaneletPose from,
    openscenario_msgs::msg::LaneletPose to
  );
  boost::optional<double> getLongitudinalDistance(
    std::int64_t from_lanelet_id, double from_s,
    std::int64_t to_lanelet_id, double to_s);
  double getSpeedLimit(std::vector<std::int64_t> lanelet_ids);
  bool isInRoute(std::int64_t lanelet_id, std::vector<std::int64_t> route) const;
  std::vector<std::int64_t> getFollowingLanelets(
    std::int64_t lanelet_id, double distance = 100,
    bool include_self = true);
  std::vector<std::int64_t> getFollowingLanelets(
    std::int64_t lanelet_id,
    std::vector<std::int64_t> candidate_lanelet_ids, double distance = 100,
    bool include_self = true);
  std::vector<std::int64_t> getPreviousLanelets(std::int64_t lanelet_id, double distance = 100);
  std::vector<geometry_msgs::msg::Point> getCenterPoints(std::int64_t lanelet_id);
  std::vector<geometry_msgs::msg::Point> getCenterPoints(std::vector<std::int64_t> lanelet_ids);
  std::vector<geometry_msgs::msg::Point> clipTrajectoryFromLaneletIds(
    std::int64_t lanelet_id, double s,
    std::vector<std::int64_t> lanelet_ids, double foward_distance = 20);
  bool canChangeLane(std::int64_t from_lanelet_id, std::int64_t to_lanelet_id);
  boost::optional<std::pair<simulation_api::math::HermiteCurve,
    double>> getLaneChangeTrajectory(
    geometry_msgs::msg::Pose from_pose,
    std::int64_t to_lanelet_id);
  boost::optional<simulation_api::math::HermiteCurve> getLaneChangeTrajectory(
    geometry_msgs::msg::Pose from_pose,
    std::int64_t to_lanelet_id, double to_s, double tangent_vector_size = 100);
  boost::optional<geometry_msgs::msg::Vector3> getTangentVector(std::int64_t lanelet_id, double s);
  std::vector<std::int64_t> getRoute(std::int64_t from_lanelet_id, std::int64_t to_lanelet_id);
  std::vector<std::int64_t> getConflictingCrosswalkIds(std::vector<std::int64_t> lanelet_ids) const;
  boost::optional<double> getCollisionPointInLaneCoordinate(
    std::int64_t lanelet_id,
    std::int64_t crossing_lanelet_id);
  const visualization_msgs::msg::MarkerArray generateMarker() const;
  const std::vector<std::int64_t> getRightOfWayLaneletIds(std::int64_t lanelet_id) const;
  const std::unordered_map<std::int64_t, std::vector<std::int64_t>> getRightOfWayLaneletIds(
    std::vector<std::int64_t> lanelet_ids) const;
  int64_t getClosetLanletId(geometry_msgs::msg::Pose pose, double distance_thresh = 30.0);
  const std::vector<geometry_msgs::msg::Point> getLaneletPolygon(std::int64_t lanelet_id);
  const std::vector<geometry_msgs::msg::Point> getStopLinesPolygon(std::int64_t lanelet_id);
  const std::vector<std::int64_t> getTrafficLightIds() const;
  const boost::optional<geometry_msgs::msg::Point> getTrafficLightBulbPosition(
    std::int64_t traffic_light_id, simulation_api::TrafficLightColor color) const;
  const std::int64_t getStopLineId(std::int64_t traffic_light_id) const;
  const std::vector<geometry_msgs::msg::Point> getStopLinePoints(std::int64_t traffic_light_id)
  const;

private:
  std::vector<std::pair<double, lanelet::Lanelet>> excludeSubtypeLaneletsWithDistance(
    const std::vector<std::pair<double, lanelet::Lanelet>> & lls, const char subtype[]);
  std::vector<std::shared_ptr<const lanelet::TrafficSign>> getTrafficSignRegElementsOnPath(
    std::vector<std::int64_t> lanelet_ids);
  std::vector<std::shared_ptr<const lanelet::autoware::AutowareTrafficLight>> getTrafficLightRegElementsOnPath(
    std::vector<std::int64_t> lanelet_ids);
  std::vector<lanelet::ConstLineString3d> getStopLinesOnPath(std::vector<std::int64_t> lanelet_ids);
  geometry_msgs::msg::Vector3 getVectorFromPose(geometry_msgs::msg::Pose pose, double magnitude);
  void mapCallback(const autoware_auto_msgs::msg::HADMapBin & msg);
  lanelet::LaneletMapPtr lanelet_map_ptr_;
  lanelet::routing::RoutingGraphConstPtr vehicle_routing_graph_ptr_;
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules_vehicle_ptr_;
  lanelet::routing::RoutingGraphConstPtr pedestrian_routing_graph_ptr_;
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules_pedestrian_ptr_;
  std::vector<double> calcEuclidDist(
    const std::vector<double> & x, const std::vector<double> & y,
    const std::vector<double> & z);
  void overwriteLaneletsCenterline();
  lanelet::LineString3d generateFineCenterline(
    const lanelet::ConstLanelet & lanelet_obj, const double resolution);
  std::vector<lanelet::BasicPoint3d> resamplePoints(
    const lanelet::ConstLineString3d & line_string, const int32_t num_segments);
  std::pair<size_t, size_t> findNearestIndexPair(
    const std::vector<double> & accumulated_lengths, const double target_length);
  std::vector<double> calculateAccumulatedLengths(const lanelet::ConstLineString3d & line_string);
  std::vector<double> calculateSegmentDistances(const lanelet::ConstLineString3d & line_string);
};
}  // namespace hdmap_utils

#endif  // SIMULATION_API__HDMAP_UTILS__HDMAP_UTILS_HPP_
