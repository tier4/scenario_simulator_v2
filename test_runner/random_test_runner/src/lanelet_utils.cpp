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
//
// Co-developed by TIER IV, Inc. and Robotec.AI sp. z o.o.

#include "random_test_runner/lanelet_utils.hpp"

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_routing/RoutingCost.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <autoware_lanelet2_extension/projection/mgrs_projector.hpp>
#include <geographic_msgs/msg/geo_point.hpp>
#include <geometry/vector3/normalize.hpp>
#include <geometry/vector3/operator.hpp>
#include <optional>
#include <traffic_simulator/hdmap_utils/hdmap_utils.hpp>

LaneletUtils::LaneletUtils(const boost::filesystem::path & filename)
{
  lanelet::projection::MGRSProjector projector;
  lanelet::ErrorMessages errors;
  lanelet_map_ptr_ = lanelet::load(filename.string(), projector, &errors);

  double arbitraryLaneChangeCostUsedOnlyForRouteFeasibilityTest = 2;
  lanelet::routing::RoutingCostPtrs costPtrs{
    std::make_shared<lanelet::routing::RoutingCostDistance>(
      arbitraryLaneChangeCostUsedOnlyForRouteFeasibilityTest)};
  auto traffic_rules_vehicle_ptr = lanelet::traffic_rules::TrafficRulesFactory::create(
    lanelet::Locations::Germany, lanelet::Participants::Vehicle);
  vehicle_routing_graph_ptr_ =
    lanelet::routing::RoutingGraph::build(*lanelet_map_ptr_, *traffic_rules_vehicle_ptr, costPtrs);

  hdmap_utils_ptr_ =
    std::make_shared<hdmap_utils::HdMapUtils>(filename, geographic_msgs::msg::GeoPoint());
}

std::vector<int64_t> LaneletUtils::getLaneletIds() { return hdmap_utils_ptr_->getLaneletIds(); }

geometry_msgs::msg::PoseStamped LaneletUtils::toMapPose(
  const traffic_simulator_msgs::msg::LaneletPose & lanelet_pose, const bool fill_pitch)
{
  return hdmap_utils_ptr_->toMapPose(lanelet_pose, fill_pitch);
}

std::vector<int64_t> LaneletUtils::getRoute(int64_t from_lanelet_id, int64_t to_lanelet_id)
{
  return hdmap_utils_ptr_->getRoute(from_lanelet_id, to_lanelet_id);
}

double LaneletUtils::getLaneletLength(int64_t lanelet_id)
{
  return hdmap_utils_ptr_->getLaneletLength(lanelet_id);
}

double LaneletUtils::computeDistance(
  const traffic_simulator_msgs::msg::LaneletPose & p1,
  const traffic_simulator_msgs::msg::LaneletPose & p2)
{
  auto p1_g = hdmap_utils_ptr_->toMapPose(p1).pose.position;
  auto p2_g = hdmap_utils_ptr_->toMapPose(p2).pose.position;
  geometry_msgs::msg::Point d;
  d.x = p1_g.x - p2_g.x;
  d.y = p1_g.y - p2_g.y;
  d.z = p1_g.z - p2_g.z;
  return std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
}

bool LaneletUtils::isInLanelet(int64_t lanelet_id, double s)
{
  return hdmap_utils_ptr_->isInLanelet(lanelet_id, s);
}

std::optional<traffic_simulator_msgs::msg::LaneletPose> LaneletUtils::getOppositeLaneLet(
  const traffic_simulator_msgs::msg::LaneletPose & pose)
{
  // Assumptions:
  // - chosen lane is right most and close to opposite one
  // - traffic is left-hand
  // - lane width is approximately constant on all the length

  // TODO: Multiple same-direction lane support
  // TODO: Find lane width for current s value

  using math::geometry::operator*;
  using math::geometry::operator+;

  if (!lanelet_map_ptr_->laneletLayer.exists(pose.lanelet_id)) {
    return {};
  }

  auto tangent_vector = hdmap_utils_ptr_->getTangentVector(pose.lanelet_id, pose.s);
  if (!tangent_vector) {
    return {};
  }
  lanelet::ConstLanelet current_lanelet = lanelet_map_ptr_->laneletLayer.get(pose.lanelet_id);
  auto left_point = current_lanelet.leftBound().front();
  auto right_point = current_lanelet.rightBound().front();

  double lane_width = lanelet::geometry::distance3d(left_point, right_point);

  if (!vehicle_routing_graph_ptr_->rights(current_lanelet).empty()) {
    // unsupported situation
    return {};
  }

  geometry_msgs::msg::Vector3 perpendicular_vector;
  perpendicular_vector.x = tangent_vector->y;
  perpendicular_vector.y = -tangent_vector->x;
  perpendicular_vector.z = 0.0;
  perpendicular_vector = math::geometry::normalize(perpendicular_vector);

  geometry_msgs::msg::Point global_position_p = toMapPose(pose, false).pose.position;
  geometry_msgs::msg::Vector3 global_position;
  global_position.x = global_position_p.x;
  global_position.y = global_position_p.y;
  global_position.z = global_position_p.z;

  geometry_msgs::msg::Vector3 opposite_lane_global_position;
  opposite_lane_global_position = global_position + perpendicular_vector * lane_width;

  geometry_msgs::msg::Pose global_pose;
  global_pose.position.x = opposite_lane_global_position.x;
  global_pose.position.y = opposite_lane_global_position.y;
  global_pose.position.z = opposite_lane_global_position.z;

  return hdmap_utils_ptr_->toLaneletPose(global_pose, false);
}

enum SearchDirection { FORWARD, BACKWARD, INVALID };

struct LaneletPartForRouting
{
  LaneletPartForRouting(
    lanelet::ConstLanelet lanelet_, double min_distance_remaining_, double max_distance_remaining_,
    SearchDirection direction_, double pose_s_)
  : lanelet(std::move(lanelet_)),
    min_distance_remaining(min_distance_remaining_),
    max_distance_remaining(max_distance_remaining_),
    pose_s(pose_s_),
    direction(direction_)
  {
  }

  LaneletPartForRouting() = delete;
  LaneletPartForRouting(const LaneletPartForRouting &) = default;
  ~LaneletPartForRouting() = default;

  lanelet::ConstLanelet lanelet;
  double start_s = -1.0;  // -1 means from the beginning
  double end_s = -1.0;    // -1 means to the end

  double min_distance_remaining;
  double max_distance_remaining;

  double pose_s;

  SearchDirection direction = SearchDirection::INVALID;
};

std::vector<LaneletPart> LaneletUtils::getLanesWithinDistance(
  const traffic_simulator_msgs::msg::LaneletPose & pose, double min_distance, double max_distance)
{
  // TODO: Implement verification that lanelet part was already added (lanelet
  // part merging)
  if (max_distance < min_distance) {
    throw std::runtime_error("Min distance cannot be greater than max distance");
  }

  std::multimap<int64_t, LaneletPartForRouting> lanelets_within_distance;
  std::queue<LaneletPartForRouting> lanelets_to_test;

  lanelet::ConstLanelet starting_lanelet = lanelet_map_ptr_->laneletLayer.get(pose.lanelet_id);
  lanelets_to_test.emplace(
    starting_lanelet, min_distance, max_distance, SearchDirection::FORWARD, pose.s);
  lanelets_to_test.emplace(
    starting_lanelet, min_distance, max_distance, SearchDirection::BACKWARD, pose.s);

  std::optional<traffic_simulator_msgs::msg::LaneletPose> opposite_lane_pose =
    getOppositeLaneLet(pose);
  if (opposite_lane_pose) {
    lanelet::ConstLanelet opposite_lanelet =
      lanelet_map_ptr_->laneletLayer.get(opposite_lane_pose->lanelet_id);
    lanelets_to_test.emplace(
      opposite_lanelet, min_distance, max_distance, SearchDirection::FORWARD,
      opposite_lane_pose->s);
    lanelets_to_test.emplace(
      opposite_lanelet, min_distance, max_distance, SearchDirection::BACKWARD,
      opposite_lane_pose->s);
  }

  lanelet::ConstLanelets next_lanelets = vehicle_routing_graph_ptr_->besides(starting_lanelet);
  for (const auto & lanelet : next_lanelets) {
    if (lanelet.id() == starting_lanelet.id()) {
      continue;
    }
    lanelets_to_test.emplace(lanelet, min_distance, max_distance, SearchDirection::FORWARD, pose.s);
    lanelets_to_test.emplace(
      lanelet, min_distance, max_distance, SearchDirection::BACKWARD, pose.s);
  }

  while (!lanelets_to_test.empty()) {
    auto & current_lanelet = lanelets_to_test.front();
    double current_lanelet_length = getLaneletLength(current_lanelet.lanelet.id());
    switch (current_lanelet.direction) {
      case SearchDirection::FORWARD:
        current_lanelet.start_s = current_lanelet.pose_s + current_lanelet.min_distance_remaining;
        current_lanelet.end_s = current_lanelet.pose_s + current_lanelet.max_distance_remaining;

        if (current_lanelet.end_s > current_lanelet_length) {
          current_lanelet.end_s = current_lanelet_length;

          double remaining_length_forward = current_lanelet_length - current_lanelet.pose_s;

          double remaining_min_distance_forward =
            std::max(0.0, current_lanelet.min_distance_remaining - remaining_length_forward);
          double remaining_max_distance_forward =
            std::max(0.0, current_lanelet.max_distance_remaining - remaining_length_forward);

          lanelet::ConstLanelets next_lanelets =
            vehicle_routing_graph_ptr_->following(current_lanelet.lanelet);
          for (const auto & lanelet : next_lanelets) {
            lanelets_to_test.emplace(
              lanelet, remaining_min_distance_forward, remaining_max_distance_forward,
              SearchDirection::FORWARD, 0.0);
          }
        }

        if (current_lanelet.start_s < current_lanelet_length) {
          lanelets_within_distance.emplace(current_lanelet.lanelet.id(), current_lanelet);
        }

        break;
      case SearchDirection::BACKWARD:
        current_lanelet.start_s = current_lanelet.pose_s - current_lanelet.max_distance_remaining;
        current_lanelet.end_s = current_lanelet.pose_s - current_lanelet.min_distance_remaining;

        if (current_lanelet.start_s < 0.0) {
          current_lanelet.start_s = 0.0;

          double remaining_length_backward = current_lanelet.pose_s;

          double remaining_min_distance_backward =
            std::max(0.0, current_lanelet.min_distance_remaining - remaining_length_backward);
          double remaining_max_distance_backward =
            std::max(0.0, current_lanelet.max_distance_remaining - remaining_length_backward);

          lanelet::ConstLanelets previous_lanelets =
            vehicle_routing_graph_ptr_->previous(current_lanelet.lanelet);
          for (const auto & lanelet : previous_lanelets) {
            lanelets_to_test.emplace(
              lanelet, remaining_min_distance_backward, remaining_max_distance_backward,
              SearchDirection::BACKWARD, getLaneletLength(lanelet.id()));
          }
        }

        if (current_lanelet.end_s > 0.0) {
          lanelets_within_distance.emplace(current_lanelet.lanelet.id(), current_lanelet);
        }
        break;
      default:
        throw std::runtime_error("No tested lanelet can have INVALID search directions");
    }

    lanelets_to_test.pop();
  }

  std::vector<LaneletPart> ret;
  for (const auto & lanelet_part_key_value : lanelets_within_distance) {
    const auto & lanelet_part = lanelet_part_key_value.second;
    ret.emplace_back(
      LaneletPart{lanelet_part.lanelet.id(), lanelet_part.start_s, lanelet_part.end_s});
  }
  return ret;
}
