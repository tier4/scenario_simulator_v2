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

#include <algorithm>
#include <behavior_tree_plugin/action_node.hpp>
#include <geometry/bounding_box.hpp>
#include <geometry/distance.hpp>
#include <geometry/quaternion/euler_to_quaternion.hpp>
#include <geometry/quaternion/get_rotation.hpp>
#include <geometry/quaternion/get_rotation_matrix.hpp>
#include <geometry/quaternion/quaternion_to_euler.hpp>
#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <scenario_simulator_exception/exception.hpp>
#include <set>
#include <string>
#include <traffic_simulator/behavior/longitudinal_speed_planning.hpp>
#include <traffic_simulator/helper/helper.hpp>
#include <traffic_simulator/lanelet_wrapper/lanelet_map.hpp>
#include <traffic_simulator/lanelet_wrapper/pose.hpp>
#include <traffic_simulator/utils/pose.hpp>
#include <unordered_map>
#include <utility>
#include <vector>

namespace entity_behavior
{
BT::PortsList operator+(const BT::PortsList & ports_1, const BT::PortsList & ports_2)
{
  BT::PortsList ports = ports_1;
  ports.insert(ports_2.begin(), ports_2.end());
  return ports;
}

ActionNode::ActionNode(const std::string & name, const BT::NodeConfiguration & config)
: BT::ActionNodeBase(name, config)
{
}

auto ActionNode::executeTick() -> BT::NodeStatus { return BT::ActionNodeBase::executeTick(); }

auto ActionNode::getBlackBoardValues() -> void
{
  if (!getInput<traffic_simulator::behavior::Request>("request", request)) {
    THROW_SIMULATION_ERROR("failed to get input request in ActionNode");
  }
  if (!getInput<double>("step_time", step_time)) {
    THROW_SIMULATION_ERROR("failed to get input step_time in ActionNode");
  }
  if (!getInput<double>("current_time", current_time)) {
    THROW_SIMULATION_ERROR("failed to get input current_time in ActionNode");
  }
  if (!getInput<std::shared_ptr<hdmap_utils::HdMapUtils>>("hdmap_utils", hdmap_utils)) {
    THROW_SIMULATION_ERROR("failed to get input hdmap_utils in ActionNode");
  }
  if (!getInput<std::shared_ptr<traffic_simulator::TrafficLightsBase>>(
        "traffic_lights", traffic_lights)) {
    THROW_SIMULATION_ERROR("failed to get input traffic_lights in ActionNode");
  }
  if (!getInput<std::shared_ptr<traffic_simulator::CanonicalizedEntityStatus>>(
        "canonicalized_entity_status", canonicalized_entity_status)) {
    THROW_SIMULATION_ERROR("failed to get input canonicalized_entity_status in ActionNode");
  }

  if (!getInput<std::optional<double>>("target_speed", target_speed)) {
    target_speed = std::nullopt;
  }

  if (!getInput<double>(
        "matching_distance_for_lanelet_pose_calculation",
        default_matching_distance_for_lanelet_pose_calculation)) {
    THROW_SIMULATION_ERROR(
      "failed to get input matching_distance_for_lanelet_pose_calculation in ActionNode");
  }

  if (!getInput<EntityStatusDict>("other_entity_status", other_entity_status)) {
    THROW_SIMULATION_ERROR("failed to get input other_entity_status in ActionNode");
  }
  if (!getInput<lanelet::Ids>("route_lanelets", route_lanelets)) {
    THROW_SIMULATION_ERROR("failed to get input route_lanelets in ActionNode");
  }
}

auto ActionNode::getOtherEntitiesCanonicalizedLaneletPoses() const
  -> std::vector<traffic_simulator::CanonicalizedLaneletPose>
{
  std::vector<traffic_simulator::CanonicalizedLaneletPose> other_canonicalized_lanelet_poses;
  for (const auto & [name, status] : other_entity_status) {
    if (auto const & canonicalized_lanelet_pose = status.getCanonicalizedLaneletPose()) {
      other_canonicalized_lanelet_poses.push_back(canonicalized_lanelet_pose.value());
    }
  }
  return other_canonicalized_lanelet_poses;
}

auto ActionNode::getOtherEntitiesCanonicalizedEntityStatuses() const
  -> std::vector<traffic_simulator::CanonicalizedEntityStatus>
{
  std::vector<traffic_simulator::CanonicalizedEntityStatus> other_status;
  for (const auto & [entity_name, entity_status] : other_entity_status) {
    other_status.push_back(entity_status);
  }
  return other_status;
}

auto ActionNode::getHorizon() const -> double
{
  return std::clamp(canonicalized_entity_status->getTwist().linear.x * 5.0, 20.0, 50.0);
}

auto ActionNode::stopEntity() const -> void
{
  canonicalized_entity_status->setTwist(geometry_msgs::msg::Twist());
  canonicalized_entity_status->setAccel(geometry_msgs::msg::Accel());
  canonicalized_entity_status->setLinearJerk(0);
}

auto ActionNode::setCanonicalizedEntityStatus(const traffic_simulator::EntityStatus & entity_status)
  -> void
{
  canonicalized_entity_status->set(
    entity_status, default_matching_distance_for_lanelet_pose_calculation);
}

auto ActionNode::getYieldStopDistance(const lanelet::Ids & following_lanelets) const
  -> std::optional<double>
{
  if (
    const auto canonicalized_lanelet_pose =
      canonicalized_entity_status->getCanonicalizedLaneletPose()) {
    if (const auto other_canonicalized_lanelet_poses = getOtherEntitiesCanonicalizedLaneletPoses();
        !other_canonicalized_lanelet_poses.empty()) {
      traffic_simulator::distance::distanceToYieldStop(
        canonicalized_lanelet_pose.value(), following_lanelets, other_canonicalized_lanelet_poses);
    }
  }
  return std::nullopt;
}

/// @todo it will be moved to traffic_simulator::route::isNeedToRightOfWay(...)
auto ActionNode::isNeedToRightOfWay(const lanelet::Ids & following_lanelets) const -> bool
{
  auto isTheSameRightOfWay =
    [&](const std::int64_t & lanelet_id, const std::int64_t & following_lanelet) {
      const auto right_of_way_lanelet_ids =
        traffic_simulator::lanelet_wrapper::lanelet_map::rightOfWayLaneletIds(lanelet_id);
      const auto the_same_right_of_way_it = std::find(
        right_of_way_lanelet_ids.begin(), right_of_way_lanelet_ids.end(), following_lanelet);
      return the_same_right_of_way_it != std::end(right_of_way_lanelet_ids);
    };

  const auto lanelet_ids_list =
    traffic_simulator::lanelet_wrapper::lanelet_map::rightOfWayLaneletIds(following_lanelets);
  for (const auto & pose : getOtherEntitiesCanonicalizedLaneletPoses()) {
    for (const auto & following_lanelet : following_lanelets) {
      for (const lanelet::Id lanelet_id : lanelet_ids_list.at(following_lanelet)) {
        if (
          isSameLaneletId(pose, lanelet_id) &&
          not isTheSameRightOfWay(lanelet_id, following_lanelet)) {
          return true;
        }
      }
    }
  }
  return false;
}

auto ActionNode::getDistanceToFrontEntity(
  const math::geometry::CatmullRomSplineInterface & spline) const -> std::optional<double>
{
  if (not canonicalized_entity_status->isInLanelet()) {
    return std::nullopt;
  }
  if (const auto front_entity_name = getFrontEntityName(spline)) {
    const auto & front_entity_status = getEntityStatus(front_entity_name.value());
    if (
      const auto & front_entity_canonicalized_lanelet_pose =
        front_entity_status.getCanonicalizedLaneletPose()) {
      return traffic_simulator::distance::splineDistanceToBoundingBox(
        spline, canonicalized_entity_status->getCanonicalizedLaneletPose().value(),
        canonicalized_entity_status->getBoundingBox(),
        front_entity_canonicalized_lanelet_pose.value(), front_entity_status.getBoundingBox());
    }
  }
  return std::nullopt;
}

auto ActionNode::getFrontEntityName(const math::geometry::CatmullRomSplineInterface & spline) const
  -> std::optional<std::string>
{
  if (not canonicalized_entity_status->isInLanelet()) {
    return std::nullopt;
  }
  /**
   * @note hard-coded parameter, if the Yaw value of RPY is in ~1.5708 -> 1.5708, entity is a
   * candidate of front entity.
   */
  constexpr double front_entity_angle_threshold{boost::math::constants::half_pi<double>()};
  constexpr double front_entity_horizon{40.0};

  std::vector<std::pair<std::string, double>> entities_with_distances;
  for (const auto & [other_entity_name, other_entity_status] : other_entity_status) {
    if (
      auto const other_canonicalized_lanelet_pose =
        other_entity_status.getCanonicalizedLaneletPose()) {
      const auto distance = traffic_simulator::distance::splineDistanceToBoundingBox(
        spline, canonicalized_entity_status->getCanonicalizedLaneletPose().value(),
        canonicalized_entity_status->getBoundingBox(), other_canonicalized_lanelet_pose.value(),
        other_entity_status.getBoundingBox());
      if (distance && distance.value() < front_entity_horizon) {
        const auto quaternion = math::geometry::getRotation(
          canonicalized_entity_status->getMapPose().orientation,
          other_entity_status.getMapPose().orientation);
        const auto yaw = math::geometry::convertQuaternionToEulerAngle(quaternion).z;
        if (std::fabs(yaw) <= front_entity_angle_threshold) {
          entities_with_distances.push_back({other_entity_name, distance.value()});
        }
      }
    }
  }

  if (!entities_with_distances.empty()) {
    auto min_entity_with_distance_it = std::min_element(
      entities_with_distances.begin(), entities_with_distances.end(),
      [](const auto & lhs, const auto & rhs) { return lhs.second < rhs.second; });
    return min_entity_with_distance_it->first;
  } else {
    return std::nullopt;
  }
}

auto ActionNode::getEntityStatus(const std::string & target_name) const
  -> const traffic_simulator::CanonicalizedEntityStatus &
{
  if (auto it = other_entity_status.find(target_name); it != other_entity_status.end()) {
    return it->second;
  } else {
    THROW_SEMANTIC_ERROR("Other entity ", std::quoted(target_name), " does not exist.");
  }
}

auto ActionNode::calculateUpdatedEntityStatus(
  const double local_target_speed,
  const traffic_simulator_msgs::msg::DynamicConstraints & constraints) const
  -> traffic_simulator::EntityStatus
{
  const auto speed_planner =
    traffic_simulator::longitudinal_speed_planning::LongitudinalSpeedPlanner(
      step_time, canonicalized_entity_status->getName());
  const auto dynamics = speed_planner.getDynamicStates(
    local_target_speed, constraints, canonicalized_entity_status->getTwist(),
    canonicalized_entity_status->getAccel());

  const double linear_jerk_new = std::get<2>(dynamics);
  const geometry_msgs::msg::Accel accel_new = std::get<1>(dynamics);
  const geometry_msgs::msg::Twist twist_new = std::get<0>(dynamics);
  if (
    const auto canonicalized_lanelet_pose =
      canonicalized_entity_status->getCanonicalizedLaneletPose()) {
    const auto distance =
      (twist_new.linear.x + canonicalized_entity_status->getTwist().linear.x) / 2.0 * step_time;
    auto entity_status_updated =
      static_cast<traffic_simulator::EntityStatus>(*canonicalized_entity_status);
    entity_status_updated.time = current_time + step_time;
    entity_status_updated.action_status.twist = twist_new;
    entity_status_updated.action_status.accel = accel_new;
    entity_status_updated.action_status.linear_jerk = linear_jerk_new;
    /// @todo it will be moved to route::moveAlongLaneletPose(...)
    entity_status_updated.lanelet_pose = traffic_simulator::lanelet_wrapper::pose::alongLaneletPose(
      static_cast<traffic_simulator::LaneletPose>(canonicalized_lanelet_pose.value()),
      route_lanelets, distance);
    entity_status_updated.lanelet_pose_valid = true;
    entity_status_updated.pose =
      traffic_simulator::pose::toMapPose(entity_status_updated.lanelet_pose);
    return entity_status_updated;
  } else {
    THROW_SIMULATION_ERROR(
      "Cannot move along lanelet - entity ", std::quoted(canonicalized_entity_status->getName()),
      " has invalid lanelet pose.");
  }
}

auto ActionNode::calculateUpdatedEntityStatusInWorldFrame(
  const double local_target_speed,
  const traffic_simulator_msgs::msg::DynamicConstraints & constraints) const
  -> traffic_simulator::EntityStatus
{
  using math::geometry::operator*;
  const auto speed_planner =
    traffic_simulator::longitudinal_speed_planning::LongitudinalSpeedPlanner(
      step_time, canonicalized_entity_status->getName());
  const auto dynamics = speed_planner.getDynamicStates(
    local_target_speed, constraints, canonicalized_entity_status->getTwist(),
    canonicalized_entity_status->getAccel());
  double linear_jerk_new = std::get<2>(dynamics);
  geometry_msgs::msg::Accel accel_new = std::get<1>(dynamics);
  geometry_msgs::msg::Twist twist_new = std::get<0>(dynamics);
  geometry_msgs::msg::Pose pose_new;
  geometry_msgs::msg::Vector3 angular_trans_vec;
  angular_trans_vec.z = twist_new.angular.z * step_time;
  geometry_msgs::msg::Quaternion angular_trans_quat =
    math::geometry::convertEulerAngleToQuaternion(angular_trans_vec);
  pose_new.orientation = canonicalized_entity_status->getMapPose().orientation * angular_trans_quat;
  Eigen::Vector3d trans_vec;
  trans_vec(0) = twist_new.linear.x * step_time;
  trans_vec(1) = twist_new.linear.y * step_time;
  trans_vec(2) = 0;
  Eigen::Matrix3d rotation_mat = math::geometry::getRotationMatrix(pose_new.orientation);
  trans_vec = rotation_mat * trans_vec;
  pose_new.position.x = trans_vec(0) + canonicalized_entity_status->getMapPose().position.x;
  pose_new.position.y = trans_vec(1) + canonicalized_entity_status->getMapPose().position.y;
  pose_new.position.z = trans_vec(2) + canonicalized_entity_status->getMapPose().position.z;
  auto entity_status_updated =
    static_cast<traffic_simulator::EntityStatus>(*canonicalized_entity_status);
  entity_status_updated.time = current_time + step_time;
  entity_status_updated.lanelet_pose = traffic_simulator::LaneletPose();
  entity_status_updated.lanelet_pose_valid = false;
  entity_status_updated.pose = pose_new;
  entity_status_updated.action_status.twist = twist_new;
  entity_status_updated.action_status.accel = accel_new;
  entity_status_updated.action_status.linear_jerk = linear_jerk_new;
  return entity_status_updated;
}

auto ActionNode::calculateStopDistance(
  const traffic_simulator_msgs::msg::DynamicConstraints & constraints) const -> double
{
  return traffic_simulator::longitudinal_speed_planning::LongitudinalSpeedPlanner(
           step_time, canonicalized_entity_status->getName())
    .getRunningDistance(
      0, constraints, canonicalized_entity_status->getTwist(),
      canonicalized_entity_status->getAccel(), canonicalized_entity_status->getLinearJerk());
}
}  // namespace entity_behavior
