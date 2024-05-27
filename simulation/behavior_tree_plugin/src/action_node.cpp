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

#include <quaternion_operation/quaternion_operation.h>

#include <algorithm>
#include <behavior_tree_plugin/action_node.hpp>
#include <geometry/bounding_box.hpp>
#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <scenario_simulator_exception/exception.hpp>
#include <set>
#include <string>
#include <traffic_simulator/behavior/longitudinal_speed_planning.hpp>
#include <traffic_simulator/helper/helper.hpp>
#include <traffic_simulator/utils/distance.hpp>
#include <traffic_simulator/utils/lanelet/other.hpp>
#include <traffic_simulator/utils/lanelet/pose.hpp>
#include <traffic_simulator/utils/lanelet/traffic_lights.hpp>
#include <traffic_simulator/utils/route.hpp>
#include <unordered_map>
#include <utility>
#include <vector>

namespace entity_behavior
{
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
  if (!getInput<std::shared_ptr<traffic_simulator::TrafficLightManager>>(
        "traffic_light_manager", traffic_light_manager)) {
    THROW_SIMULATION_ERROR("failed to get input traffic_light_manager in ActionNode");
  }
  if (!getInput<std::shared_ptr<traffic_simulator::CanonicalizedEntityStatus>>(
        "entity_status", entity_status)) {
    THROW_SIMULATION_ERROR("failed to get input entity_status in ActionNode");
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

auto ActionNode::getOtherEntitiesPoses() const
  -> std::vector<traffic_simulator::CanonicalizedLaneletPose>
{
  std::vector<traffic_simulator::CanonicalizedLaneletPose> other_poses;
  for (const auto & other_status_pair : other_entity_status) {
    if (
      auto const canonicalized_lanelet_pose =
        other_status_pair.second.getCanonicalizedLaneletPose()) {
      other_poses.push_back(canonicalized_lanelet_pose.value());
    }
  }
  return other_poses;
}

auto ActionNode::getOtherEntities() const
  -> std::vector<traffic_simulator::CanonicalizedEntityStatus>
{
  std::vector<traffic_simulator::CanonicalizedEntityStatus> other_poses;
  for (const auto & other_status_pair : other_entity_status) {
    other_poses.push_back(other_status_pair.second);
  }
  return other_poses;
}

auto ActionNode::getHorizon() const -> double
{
  return std::clamp(entity_status->getTwist().linear.x * 5.0, 20.0, 50.0);
}

auto ActionNode::stopEntity() const -> void
{
  entity_status->setTwist(geometry_msgs::msg::Twist());
  entity_status->setAccel(geometry_msgs::msg::Accel());
  entity_status->setLinearJerk(0);
}

auto ActionNode::getOtherEntityStatus(lanelet::Id lanelet_id) const
  -> std::vector<traffic_simulator::CanonicalizedEntityStatus>
{
  std::vector<traffic_simulator::CanonicalizedEntityStatus> ret;
  for (const auto & status : other_entity_status) {
    if (
      status.second.laneMatchingSucceed() &&
      traffic_simulator::isSameLaneletId(status.second, lanelet_id)) {
      ret.emplace_back(status.second);
    }
  }
  return ret;
}

auto ActionNode::getYieldStopDistance(const lanelet::Ids & following_lanelets) const
  -> std::optional<double>
{
  if (const auto other_poses = getOtherEntitiesPoses(); !other_poses.empty()) {
    if (auto const canonicalized_lanelet_pose = entity_status->getCanonicalizedLaneletPose())
      traffic_simulator::distance::distanceToYieldStop(
        canonicalized_lanelet_pose.value(), following_lanelets, other_poses);
  } else {
    return std::nullopt;
  }
}

auto ActionNode::getDistanceToStopLine(
  const lanelet::Ids & route_lanelets,
  const std::vector<geometry_msgs::msg::Point> & waypoints) const -> std::optional<double>
{
  return traffic_simulator::distance::distanceToStopLine(route_lanelets, waypoints);
}

auto ActionNode::getDistanceToFrontEntity(
  const math::geometry::CatmullRomSplineInterface & spline) const -> std::optional<double>
{
  if (const auto name = getFrontEntityName(spline)) {
    const auto & status = getEntityStatus(name.value());
    if (auto const canonicalized_lanelet_pose = status.getCanonicalizedLaneletPose()) {
      return traffic_simulator::distance::splineDistanceToBoundingBox(
        spline, canonicalized_lanelet_pose.value(), status.getBoundingBox());
    }
  }
  return std::nullopt;
}

auto ActionNode::getFrontEntityName(const math::geometry::CatmullRomSplineInterface & spline) const
  -> std::optional<std::string>
{
  std::vector<double> distances;
  std::vector<std::string> entities;
  for (const auto & each : other_entity_status) {
    if (auto const canonicalized_lanelet_pose = each.second.getCanonicalizedLaneletPose()) {
      const auto distance = traffic_simulator::distance::splineDistanceToBoundingBox(
        spline, canonicalized_lanelet_pose.value(), each.second.getBoundingBox());
      const auto quat = quaternion_operation::getRotation(
        entity_status->getMapPose().orientation, each.second.getMapPose().orientation);
      /**
     * @note hard-coded parameter, if the Yaw value of RPY is in ~1.5708 -> 1.5708, entity is a candidate of front entity.
     */
      if (
        std::fabs(quaternion_operation::convertQuaternionToEulerAngle(quat).z) <=
        boost::math::constants::half_pi<double>()) {
        if (distance && distance.value() < 40) {
          entities.emplace_back(each.first);
          distances.emplace_back(distance.value());
        }
      }
    }
  }
  if (entities.size() != distances.size()) {
    THROW_SIMULATION_ERROR("size of entities and distances vector does not match.");
  }
  if (distances.empty()) {
    return std::nullopt;
  }
  std::vector<double>::iterator iter = std::min_element(distances.begin(), distances.end());
  size_t index = std::distance(distances.begin(), iter);
  return entities[index];
}

auto ActionNode::getEntityStatus(const std::string & target_name) const
  -> const traffic_simulator::CanonicalizedEntityStatus &
{
  if (auto it = other_entity_status.find(target_name); it != other_entity_status.end()) {
    return it->second;
  } else {
    THROW_SEMANTIC_ERROR("other entity : ", target_name, " does not exist.");
  }
}

auto ActionNode::calculateUpdatedEntityStatus(
  double target_speed, const traffic_simulator_msgs::msg::DynamicConstraints & constraints) const
  -> traffic_simulator::EntityStatus
{
  const auto speed_planner =
    traffic_simulator::longitudinal_speed_planning::LongitudinalSpeedPlanner(
      step_time, getEntityName());
  const auto dynamics = speed_planner.getDynamicStates(
    target_speed, constraints, entity_status->getTwist(), entity_status->getAccel());

  double linear_jerk_new = std::get<2>(dynamics);
  geometry_msgs::msg::Accel accel_new = std::get<1>(dynamics);
  geometry_msgs::msg::Twist twist_new = std::get<0>(dynamics);
  if (const auto canonicalized_lanelet_pose = entity_status->getCanonicalizedLaneletPose()) {
    auto entity_status_updated = static_cast<traffic_simulator::EntityStatus>(*entity_status);
    const auto distance =
      (twist_new.linear.x + entity_status->getTwist().linear.x) / 2.0 * step_time;
    entity_status_updated.time = current_time + step_time;
    entity_status_updated.action_status.twist = twist_new;
    entity_status_updated.action_status.accel = accel_new;
    entity_status_updated.action_status.linear_jerk = linear_jerk_new;
    entity_status_updated.lanelet_pose =
      moveAlongLanelet(canonicalized_lanelet_pose.value(), route_lanelets, distance);
    entity_status_updated.lanelet_pose_valid = true;
    entity_status_updated.pose =
      traffic_simulator::pose::toMapPose(entity_status_updated.lanelet_pose);
    return entity_status_updated;
  } else {
    THROW_SIMULATION_ERROR("Cannot move along lanelet - there is invalid lanelet pose.");
  }
}

auto ActionNode::moveAlongLanelet(
  const traffic_simulator::CanonicalizedLaneletPose & canonicalized_lanelet_pose,
  const lanelet::Ids & route_lanelets, const auto distance) const -> traffic_simulator::LaneletPose
{
  auto lanelet_pose = static_cast<traffic_simulator::LaneletPose>(canonicalized_lanelet_pose);
  lanelet_pose.s = lanelet_pose.s + distance;
  const auto canonicalized =
    traffic_simulator::lanelet2::pose::canonicalizeLaneletPose(lanelet_pose, route_lanelets);
  if (
    const auto canonicalized_lanelet_pose =
      std::get<std::optional<traffic_simulator::LaneletPose>>(canonicalized)) {
    // If canonicalize succeed, just return canonicalized pose
    return canonicalized_lanelet_pose.value();
  } else {
    // If canonicalize failed, return lanelet pose as end of road
    if (const auto end_of_road_lanelet_id = std::get<std::optional<lanelet::Id>>(canonicalized)) {
      traffic_simulator::LaneletPose end_of_road_lanelet_pose;
      end_of_road_lanelet_pose.lanelet_id = end_of_road_lanelet_id.value();
      end_of_road_lanelet_pose.offset = lanelet_pose.offset;
      end_of_road_lanelet_pose.rpy = lanelet_pose.rpy;
      /// @note here was condition: .s < 0, now try to use .s <= 0
      end_of_road_lanelet_pose.s =
        lanelet_pose.s <= 0
          ? 0
          : traffic_simulator::lanelet2::other::getLaneletLength(end_of_road_lanelet_id.value());
      return end_of_road_lanelet_pose;
    } else {
      THROW_SIMULATION_ERROR("Failed to find trailing lanelet_id.");
    }
  }
}

auto ActionNode::calculateUpdatedEntityStatusInWorldFrame(
  double target_speed, const traffic_simulator_msgs::msg::DynamicConstraints & constraints) const
  -> traffic_simulator::EntityStatus
{
  const auto speed_planner =
    traffic_simulator::longitudinal_speed_planning::LongitudinalSpeedPlanner(
      step_time, getEntityName());
  const auto dynamics = speed_planner.getDynamicStates(
    target_speed, constraints, entity_status->getTwist(), entity_status->getAccel());
  double linear_jerk_new = std::get<2>(dynamics);
  geometry_msgs::msg::Accel accel_new = std::get<1>(dynamics);
  geometry_msgs::msg::Twist twist_new = std::get<0>(dynamics);
  geometry_msgs::msg::Pose pose_new;
  geometry_msgs::msg::Vector3 angular_trans_vec;
  angular_trans_vec.z = twist_new.angular.z * step_time;
  geometry_msgs::msg::Quaternion angular_trans_quat =
    quaternion_operation::convertEulerAngleToQuaternion(angular_trans_vec);
  pose_new.orientation =
    quaternion_operation::rotation(entity_status->getMapPose().orientation, angular_trans_quat);
  Eigen::Vector3d trans_vec;
  trans_vec(0) = twist_new.linear.x * step_time;
  trans_vec(1) = twist_new.linear.y * step_time;
  trans_vec(2) = 0;
  Eigen::Matrix3d rotation_mat = quaternion_operation::getRotationMatrix(pose_new.orientation);
  trans_vec = rotation_mat * trans_vec;
  pose_new.position.x = trans_vec(0) + entity_status->getMapPose().position.x;
  pose_new.position.y = trans_vec(1) + entity_status->getMapPose().position.y;
  pose_new.position.z = trans_vec(2) + entity_status->getMapPose().position.z;
  auto entity_status_updated = static_cast<traffic_simulator::EntityStatus>(*entity_status);
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
           step_time, getEntityName())
    .getRunningDistance(
      0, constraints, entity_status->getTwist(), entity_status->getAccel(),
      entity_status->getLinearJerk());
}

auto ActionNode::getActionStatus() const noexcept -> traffic_simulator_msgs::msg::ActionStatus
{
  return static_cast<traffic_simulator::EntityStatus>(*entity_status).action_status;
}

auto ActionNode::getEntityName() const noexcept -> std::string
{
  return static_cast<traffic_simulator::EntityStatus>(*entity_status).name;
}
}  // namespace entity_behavior
