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

#include <autoware_lanelet2_extension/utility/message_conversion.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
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
#include <traffic_simulator/data_type/routing_configuration.hpp>
#include <traffic_simulator/data_type/routing_graph_type.hpp>
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

  auto canChangeLane(
    const lanelet::Id from, const lanelet::Id to,
    const traffic_simulator::RoutingGraphType type =
      traffic_simulator::RoutingConfiguration().routing_graph_type) const -> bool;

  auto countLaneChanges(
    const traffic_simulator_msgs::msg::LaneletPose & from,
    const traffic_simulator_msgs::msg::LaneletPose & to,
    const traffic_simulator::RoutingConfiguration & routing_configuration =
      traffic_simulator::RoutingConfiguration()) const -> std::optional<std::pair<int, int>>;

  auto generateMarker() const -> visualization_msgs::msg::MarkerArray;

  auto getCenterPoints(const lanelet::Ids &) const -> std::vector<geometry_msgs::msg::Point>;

  auto getCenterPoints(const lanelet::Id) const -> std::vector<geometry_msgs::msg::Point>;

  auto getCenterPointsSpline(const lanelet::Id) const
    -> std::shared_ptr<math::geometry::CatmullRomSpline>;

  auto getFollowingLanelets(
    const lanelet::Id current_lanelet_id, const lanelet::Ids & route, const double horizon = 100,
    const bool include_current_lanelet_id = true,
    const traffic_simulator::RoutingGraphType type =
      traffic_simulator::RoutingConfiguration().routing_graph_type) const -> lanelet::Ids;

  auto getFollowingLanelets(
    const lanelet::Id, const double distance = 100, const bool include_self = true,
    const traffic_simulator::RoutingGraphType type =
      traffic_simulator::RoutingConfiguration().routing_graph_type) const -> lanelet::Ids;

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
    const lanelet::Id, const traffic_simulator::lane_change::Direction,
    const traffic_simulator::RoutingGraphType type =
      traffic_simulator::RoutingConfiguration().routing_graph_type) const
    -> std::optional<lanelet::Id>;

  auto getLaneChangeableLaneletId(
    const lanelet::Id, const traffic_simulator::lane_change::Direction, const std::uint8_t shift,
    const traffic_simulator::RoutingGraphType type =
      traffic_simulator::RoutingConfiguration().routing_graph_type) const
    -> std::optional<lanelet::Id>;

  auto getLaneletIds() const -> lanelet::Ids;

  auto getLanelets(const lanelet::Ids &) const -> lanelet::Lanelets;

  auto getPreviousLanelets(
    const lanelet::Id, const double backward_horizon = 100,
    const traffic_simulator::RoutingGraphType type =
      traffic_simulator::RoutingConfiguration().routing_graph_type) const -> lanelet::Ids;

  auto getRoute(
    const lanelet::Id from, const lanelet::Id to,
    const traffic_simulator::RoutingConfiguration & routing_configuration =
      traffic_simulator::RoutingConfiguration()) const -> lanelet::Ids;

  auto getSpeedLimit(
    const lanelet::Ids &, const traffic_simulator::RoutingGraphType type =
                            traffic_simulator::RoutingConfiguration().routing_graph_type) const
    -> double;

  auto getTangentVector(const lanelet::Id, const double s) const
    -> std::optional<geometry_msgs::msg::Vector3>;

  auto getTrafficLightBulbPosition(
    const lanelet::Id traffic_light_id, const std::string &,
    const bool allow_infer_position = false) const -> std::optional<geometry_msgs::msg::Point>;

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

  auto isInRoute(const lanelet::Id, const lanelet::Ids & route) const -> bool;

  auto isTrafficLight(const lanelet::Id) const -> bool;

  auto isTrafficLightRegulatoryElement(const lanelet::Id) const -> bool;

private:
  /// @note This value was determined experimentally by @hakuturu583 and not theoretically.
  /// @sa https://github.com/tier4/scenario_simulator_v2/commit/4c8e9f496b061b00bec799159d59c33f2ba46b3a
  constexpr static double DEFAULT_MATCH_TO_LANE_REDUCTION_RATIO = 0.8;

  /** @defgroup cache
   *  Declared mutable for caching
   */
  // @{
  mutable CenterPointsCache center_points_cache_;
  mutable LaneletLengthCache lanelet_length_cache_;
  // @}

  lanelet::LaneletMapPtr lanelet_map_ptr_;

  class RoutingGraphs
  {
  public:
    explicit RoutingGraphs(const lanelet::LaneletMapPtr & lanelet_map);

    struct RuleWithGraph
    {
      lanelet::traffic_rules::TrafficRulesPtr rules;
      lanelet::routing::RoutingGraphPtr graph;
      RouteCache route_cache;
    };

    [[nodiscard]] lanelet::routing::RoutingGraphPtr routing_graph(
      const traffic_simulator::RoutingGraphType type) const;

    [[nodiscard]] lanelet::traffic_rules::TrafficRulesPtr traffic_rule(
      const traffic_simulator::RoutingGraphType type) const;

    [[nodiscard]] auto getRoute(
      const lanelet::Id from_lanelet_id, const lanelet::Id to_lanelet_id,
      lanelet::LaneletMapPtr lanelet_map_ptr,
      const traffic_simulator::RoutingConfiguration & routing_configuration =
        traffic_simulator::RoutingConfiguration()) -> lanelet::Ids;

  private:
    [[nodiscard]] RouteCache & route_cache(const traffic_simulator::RoutingGraphType type);

    RuleWithGraph vehicle;

    RuleWithGraph vehicle_with_road_shoulder;

    RuleWithGraph pedestrian;
  };

  std::unique_ptr<RoutingGraphs> routing_graphs_;

  template <typename Lanelet>
  auto getLaneletIds(const std::vector<Lanelet> & lanelets) const -> lanelet::Ids
  {
    lanelet::Ids ids;
    std::transform(
      lanelets.begin(), lanelets.end(), std::back_inserter(ids),
      [](const auto & lanelet) { return lanelet.id(); });
    return ids;
  }

  auto calculateAccumulatedLengths(const lanelet::ConstLineString3d &) const -> std::vector<double>;

  auto calculateSegmentDistances(const lanelet::ConstLineString3d &) const -> std::vector<double>;

  auto excludeSubtypeLanelets(
    const std::vector<std::pair<double, lanelet::Lanelet>> &, const char subtype[]) const
    -> std::vector<std::pair<double, lanelet::Lanelet>>;

  auto findNearestIndexPair(
    const std::vector<double> & accumulated_lengths, const double target_length) const
    -> std::pair<std::size_t, std::size_t>;

  auto generateFineCenterline(const lanelet::ConstLanelet &, const double resolution) const
    -> lanelet::LineString3d;

  auto getLaneChangeTrajectory(
    const geometry_msgs::msg::Pose & from, const traffic_simulator_msgs::msg::LaneletPose & to,
    const traffic_simulator::lane_change::TrajectoryShape,
    const double tangent_vector_size = 100) const -> math::geometry::HermiteCurve;

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

  auto overwriteLaneletsCenterline() -> void;

  auto resamplePoints(const lanelet::ConstLineString3d &, const std::int32_t num_segments) const
    -> lanelet::BasicPoints3d;
};
}  // namespace hdmap_utils

#endif  // TRAFFIC_SIMULATOR__HDMAP_UTILS__HDMAP_UTILS_HPP_
