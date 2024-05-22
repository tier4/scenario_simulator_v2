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
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <lanelet2_extension/io/autoware_osm_parser.hpp>
#include <lanelet2_extension/projection/mgrs_projector.hpp>
#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <lanelet2_extension/visualization/visualization.hpp>
#include <optional>
#include <traffic_simulator/data_type/lane_change.hpp>
#include <traffic_simulator/data_type/lanelet_pose.hpp>
#include <traffic_simulator/hdmap_utils/cache.hpp>
#include <traffic_simulator_msgs/msg/bounding_box.hpp>
#include <traffic_simulator_msgs/msg/entity_type.hpp>
#include <traffic_simulator_msgs/msg/lanelet_pose.hpp>

namespace traffic_simulator
{
inline namespace lanelet2
{
class Memory
{
private:
  inline static std::unique_ptr<Memory> instance{nullptr};
  inline static std::string lanelet_map_path_{""};
  inline static std::mutex mutex_;

  Memory(const boost::filesystem::path & lanelet2_map_path)
  {
    lanelet::projection::MGRSProjector projector;

    lanelet::ErrorMessages errors;

    lanelet_map_ptr_ = lanelet::load(lanelet2_map_path.string(), projector, &errors);

    if (not errors.empty()) {
      std::stringstream ss;
      const auto * separator = "";
      for (const auto & error : errors) {
        ss << separator << error;
        separator = "\n";
      }
      THROW_SIMULATION_ERROR("Failed to load lanelet map (", ss.str(), ")");
    }
    overwriteLaneletsCenterline();
    traffic_rules_vehicle_ptr_ = lanelet::traffic_rules::TrafficRulesFactory::create(
      lanelet::Locations::Germany, lanelet::Participants::Vehicle);
    vehicle_routing_graph_ptr_ =
      lanelet::routing::RoutingGraph::build(*lanelet_map_ptr_, *traffic_rules_vehicle_ptr_);
    traffic_rules_pedestrian_ptr_ = lanelet::traffic_rules::TrafficRulesFactory::create(
      lanelet::Locations::Germany, lanelet::Participants::Pedestrian);
    pedestrian_routing_graph_ptr_ =
      lanelet::routing::RoutingGraph::build(*lanelet_map_ptr_, *traffic_rules_pedestrian_ptr_);
    std::vector<lanelet::routing::RoutingGraphConstPtr> all_graphs;
    all_graphs.push_back(vehicle_routing_graph_ptr_);
    all_graphs.push_back(pedestrian_routing_graph_ptr_);
    shoulder_lanelets_ = lanelet::utils::query::shoulderLanelets(
      lanelet::utils::query::laneletLayer(lanelet_map_ptr_));
  }

  auto overwriteLaneletsCenterline() -> void
  {
    for (auto & lanelet_obj : lanelet_map_ptr_->laneletLayer) {
      if (!lanelet_obj.hasCustomCenterline()) {
        const auto fine_center_line = generateFineCenterline(lanelet_obj, 2.0);
        lanelet_obj.setCenterline(fine_center_line);
      }
    }
  }

  auto generateFineCenterline(const lanelet::ConstLanelet & lanelet_obj, const double resolution)
    -> lanelet::LineString3d
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

  auto resamplePoints(
    const lanelet::ConstLineString3d & line_string, const std::int32_t num_segments)
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

  auto findNearestIndexPair(
    const std::vector<double> & accumulated_lengths, const double target_length)
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

  auto calculateAccumulatedLengths(const lanelet::ConstLineString3d & line_string)
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

  auto calculateSegmentDistances(const lanelet::ConstLineString3d & line_string)
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

public:
  Memory(const Memory &) = delete;
  Memory & operator=(const Memory &) = delete;
  Memory() = delete;
  ~Memory() = default;

  static auto activate(const std::string & lanelet_map_path) -> void
  {
    lanelet_map_path_ = lanelet_map_path;
  }

  static Memory & getInstance()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    // if(lanelet_map_path_.empty())
    // {
    //   throw std::runtime_error("Memory is not activate! Please call Memory::activate(map_path) in this node.");
    // }
    // else
    if (!instance) {
      std::cout << std::endl
                << std::endl
                << " ###### Make Memory ###### " << lanelet_map_path_ << std::endl
                << std::endl
                << std::endl;
      instance.reset(new Memory(lanelet_map_path_));
    }
    return *instance;
  }

  mutable hdmap_utils::RouteCache route_cache_;
  mutable hdmap_utils::CenterPointsCache center_points_cache_;
  mutable hdmap_utils::LaneletLengthCache lanelet_length_cache_;

  lanelet::LaneletMapPtr lanelet_map_ptr_;
  lanelet::routing::RoutingGraphConstPtr vehicle_routing_graph_ptr_;
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules_vehicle_ptr_;
  lanelet::routing::RoutingGraphConstPtr pedestrian_routing_graph_ptr_;
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules_pedestrian_ptr_;
  lanelet::ConstLanelets shoulder_lanelets_;
};

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

auto isTrafficLightRegulatoryElement(const lanelet::Id lanelet_id) -> bool;

auto getTrafficLightRegulatoryElement(const lanelet::Id lanelet_id) -> lanelet::TrafficLight::Ptr;

auto isTrafficLight(const lanelet::Id lanelet_id) -> bool;

auto getTrafficLightBulbPosition(const lanelet::Id traffic_light_id, const std::string & color_name)
  -> std::optional<geometry_msgs::msg::Point>;

auto isInLanelet(const lanelet::Id lanelet_id, const double s) -> bool;
}  // namespace lanelet2
}  // namespace traffic_simulator
#endif  // TRAFFIC_SIMULATOR__UTILS__LANELET_HPP_
