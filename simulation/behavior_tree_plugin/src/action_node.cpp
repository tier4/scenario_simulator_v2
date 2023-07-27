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
  if (!getInput<std::shared_ptr<hdmap_utils::HdMapUtils>>("hdmap_utils", hdmap_utils)) {
    THROW_SIMULATION_ERROR("failed to get input hdmap_utils in ActionNode");
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

  if (!getInput<EntityStatusDict>("other_entity_status", other_entity_status)) {
    THROW_SIMULATION_ERROR("failed to get input other_entity_status in ActionNode");
  }
  if (!getInput<std::unordered_map<std::string, traffic_simulator_msgs::msg::EntityType>>(
        "entity_type_list", entity_type_list)) {
    THROW_SIMULATION_ERROR("failed to get input entity_type_list in ActionNode");
  }
  if (!getInput<std::vector<std::int64_t>>("route_lanelets", route_lanelets)) {
    THROW_SIMULATION_ERROR("failed to get input route_lanelets in ActionNode");
  }
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

auto ActionNode::getOtherEntityStatus(std::int64_t lanelet_id) const
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

auto ActionNode::getYieldStopDistance(const std::vector<std::int64_t> & following_lanelets) const
  -> std::optional<double>
{
  std::set<double> distances;
  for (const auto & lanelet : following_lanelets) {
    const auto right_of_way_ids = hdmap_utils->getRightOfWayLaneletIds(lanelet);
    for (const auto right_of_way_id : right_of_way_ids) {
      const auto other_status = getOtherEntityStatus(right_of_way_id);
      if (!other_status.empty() && entity_status->laneMatchingSucceed()) {
        const auto lanelet_pose = entity_status->getLaneletPose();
        auto distance = hdmap_utils->getLongitudinalDistance(
          lanelet_pose, traffic_simulator::helper::constructLaneletPose(lanelet, 0));
        if (distance) {
          distances.insert(distance.value());
        }
      }
    }
    if (distances.size() != 0) {
      return *distances.begin();
    }
  }
  return std::nullopt;
}

auto ActionNode::getRightOfWayEntities(const std::vector<std::int64_t> & following_lanelets) const
  -> std::vector<traffic_simulator::CanonicalizedEntityStatus>
{
  std::vector<traffic_simulator::CanonicalizedEntityStatus> ret;
  const auto lanelet_ids_list = hdmap_utils->getRightOfWayLaneletIds(following_lanelets);
  for (const auto & status : other_entity_status) {
    for (const auto & following_lanelet : following_lanelets) {
      for (const std::int64_t & lanelet_id : lanelet_ids_list.at(following_lanelet)) {
        if (
          status.second.laneMatchingSucceed() &&
          traffic_simulator::isSameLaneletId(status.second, lanelet_id)) {
          ret.emplace_back(status.second);
        }
      }
    }
  }
  return ret;
}

auto ActionNode::getRightOfWayEntities() const
  -> std::vector<traffic_simulator::CanonicalizedEntityStatus>
{
  if (!entity_status->laneMatchingSucceed()) {
    return {};
  }
  std::vector<traffic_simulator::CanonicalizedEntityStatus> ret;
  const auto lanelet_ids =
    hdmap_utils->getRightOfWayLaneletIds(entity_status->getLaneletPose().lanelet_id);
  if (lanelet_ids.empty()) {
    return ret;
  }
  for (const auto & status : other_entity_status) {
    for (const std::int64_t & lanelet_id : lanelet_ids) {
      if (
        status.second.laneMatchingSucceed() &&
        traffic_simulator::isSameLaneletId(status.second, lanelet_id)) {
        ret.emplace_back(status.second);
      }
    }
  }
  return ret;
}

auto ActionNode::getDistanceToTrafficLightStopLine(
  const std::vector<std::int64_t> & route_lanelets,
  const math::geometry::CatmullRomSplineInterface & spline) const -> std::optional<double>
{
  const auto traffic_light_ids = hdmap_utils->getTrafficLightIdsOnPath(route_lanelets);
  if (traffic_light_ids.empty()) {
    return std::nullopt;
  }
  std::set<double> collision_points = {};
  for (const auto id : traffic_light_ids) {
    using Color = traffic_simulator::TrafficLight::Color;
    using Status = traffic_simulator::TrafficLight::Status;
    using Shape = traffic_simulator::TrafficLight::Shape;
    if (auto && traffic_light = traffic_light_manager->getTrafficLight(id);
        traffic_light.contains(Color::red, Status::solid_on, Shape::circle) or
        traffic_light.contains(Color::yellow, Status::solid_on, Shape::circle)) {
      const auto collision_point = hdmap_utils->getDistanceToTrafficLightStopLine(spline, id);
      if (collision_point) {
        collision_points.insert(collision_point.value());
      }
    }
  }
  if (collision_points.empty()) {
    return std::nullopt;
  }
  return *collision_points.begin();
}

auto ActionNode::getDistanceToStopLine(
  const std::vector<std::int64_t> & route_lanelets,
  const std::vector<geometry_msgs::msg::Point> & waypoints) const -> std::optional<double>
{
  return hdmap_utils->getDistanceToStopLine(route_lanelets, waypoints);
}

auto ActionNode::getDistanceToFrontEntity(
  const math::geometry::CatmullRomSplineInterface & spline) const -> std::optional<double>
{
  auto name = getFrontEntityName(spline);
  if (!name) {
    return std::nullopt;
  }
  return getDistanceToTargetEntityPolygon(spline, name.value());
}

auto ActionNode::getFrontEntityName(const math::geometry::CatmullRomSplineInterface & spline) const
  -> std::optional<std::string>
{
  std::vector<double> distances;
  std::vector<std::string> entities;
  for (const auto & each : other_entity_status) {
    const auto distance = getDistanceToTargetEntityPolygon(spline, each.first);
    const auto quat = quaternion_operation::getRotation(
      entity_status->getMapPose().orientation,
      other_entity_status.at(each.first).getMapPose().orientation);
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

auto ActionNode::getDistanceToTargetEntityOnCrosswalk(
  const math::geometry::CatmullRomSplineInterface & spline,
  const traffic_simulator::CanonicalizedEntityStatus & status) const -> std::optional<double>
{
  if (status.laneMatchingSucceed()) {
    return spline.getCollisionPointIn2D(
      hdmap_utils->getLaneletPolygon(status.getLaneletPose().lanelet_id), false, true);
  }
  return std::nullopt;
}

auto ActionNode::getEntityStatus(const std::string & target_name) const
  -> traffic_simulator::CanonicalizedEntityStatus
{
  if (other_entity_status.find(target_name) != other_entity_status.end()) {
    return traffic_simulator::CanonicalizedEntityStatus(other_entity_status.at(target_name));
  }
  THROW_SIMULATION_ERROR("other entity : ", target_name, " does not exist.");
}

auto ActionNode::getDistanceToTargetEntityPolygon(
  const math::geometry::CatmullRomSplineInterface & spline, const std::string target_name,
  double width_extension_right, double width_extension_left, double length_extension_front,
  double length_extension_rear) const -> std::optional<double>
{
  const auto status = getEntityStatus(target_name);
  if (status.laneMatchingSucceed()) {
    return getDistanceToTargetEntityPolygon(
      spline, status, width_extension_right, width_extension_left, length_extension_front,
      length_extension_rear);
  }
  return std::nullopt;
}

auto ActionNode::getDistanceToTargetEntityPolygon(
  const math::geometry::CatmullRomSplineInterface & spline,
  const traffic_simulator::CanonicalizedEntityStatus & status, double width_extension_right,
  double width_extension_left, double length_extension_front, double length_extension_rear) const
  -> std::optional<double>
{
  if (status.laneMatchingSucceed()) {
    const auto polygon = math::geometry::transformPoints(
      status.getMapPose(), math::geometry::getPointsFromBbox(
                             status.getBoundingBox(), width_extension_right, width_extension_left,
                             length_extension_front, length_extension_rear));
    return spline.getCollisionPointIn2D(polygon, false, true);
  }
  return std::nullopt;
}

auto ActionNode::getDistanceToConflictingEntity(
  const std::vector<std::int64_t> & route_lanelets,
  const math::geometry::CatmullRomSplineInterface & spline) const -> std::optional<double>
{
  auto crosswalk_entity_status = getConflictingEntityStatusOnCrossWalk(route_lanelets);
  auto lane_entity_status = getConflictingEntityStatusOnLane(route_lanelets);
  std::set<double> distances;
  for (const auto & status : crosswalk_entity_status) {
    const auto s = getDistanceToTargetEntityOnCrosswalk(spline, status);
    if (s) {
      distances.insert(s.value());
    }
  }
  for (const auto & status : lane_entity_status) {
    const auto s = getDistanceToTargetEntityPolygon(spline, status, 0.0, 0.0, 0.0, 1.0);
    if (s) {
      distances.insert(s.value());
    }
  }
  if (distances.empty()) {
    return std::nullopt;
  }
  return *distances.begin();
}

auto ActionNode::getConflictingEntityStatusOnCrossWalk(
  const std::vector<std::int64_t> & route_lanelets) const
  -> std::vector<traffic_simulator::CanonicalizedEntityStatus>
{
  std::vector<traffic_simulator::CanonicalizedEntityStatus> conflicting_entity_status;
  auto conflicting_crosswalks = hdmap_utils->getConflictingCrosswalkIds(route_lanelets);
  for (const auto & status : other_entity_status) {
    if (
      status.second.laneMatchingSucceed() &&
      std::count(
        conflicting_crosswalks.begin(), conflicting_crosswalks.end(),
        status.second.getLaneletPose().lanelet_id) >= 1) {
      conflicting_entity_status.emplace_back(status.second);
    }
  }
  return conflicting_entity_status;
}

auto ActionNode::getConflictingEntityStatusOnLane(const std::vector<std::int64_t> & route_lanelets)
  const -> std::vector<traffic_simulator::CanonicalizedEntityStatus>
{
  std::vector<traffic_simulator::CanonicalizedEntityStatus> conflicting_entity_status;
  auto conflicting_lanes = hdmap_utils->getConflictingLaneIds(route_lanelets);
  for (const auto & status : other_entity_status) {
    if (
      status.second.laneMatchingSucceed() && std::count(
                                               conflicting_lanes.begin(), conflicting_lanes.end(),
                                               status.second.getLaneletPose().lanelet_id) >= 1) {
      conflicting_entity_status.emplace_back(status.second);
    }
  }
  return conflicting_entity_status;
}

auto ActionNode::foundConflictingEntity(const std::vector<std::int64_t> & following_lanelets) const
  -> bool
{
  auto conflicting_crosswalks = hdmap_utils->getConflictingCrosswalkIds(following_lanelets);
  auto conflicting_lanes = hdmap_utils->getConflictingLaneIds(following_lanelets);
  for (const auto & status : other_entity_status) {
    if (
      status.second.laneMatchingSucceed() &&
      std::count(
        conflicting_crosswalks.begin(), conflicting_crosswalks.end(),
        status.second.getLaneletPose().lanelet_id) >= 1) {
      return true;
    }
    if (
      status.second.laneMatchingSucceed() && std::count(
                                               conflicting_lanes.begin(), conflicting_lanes.end(),
                                               status.second.getLaneletPose().lanelet_id) >= 1) {
      return true;
    }
  }
  return false;
}

auto ActionNode::calculateUpdatedEntityStatus(
  double target_speed, const traffic_simulator_msgs::msg::DynamicConstraints & constraints) const
  -> traffic_simulator::CanonicalizedEntityStatus
{
  const auto speed_planner =
    traffic_simulator::longitudinal_speed_planning::LongitudinalSpeedPlanner(
      step_time, getEntityName());
  const auto dynamics = speed_planner.getDynamicStates(
    target_speed, constraints, entity_status->getTwist(), entity_status->getAccel());

  double linear_jerk_new = std::get<2>(dynamics);
  geometry_msgs::msg::Accel accel_new = std::get<1>(dynamics);
  geometry_msgs::msg::Twist twist_new = std::get<0>(dynamics);

  auto lanelet_pose = entity_status->getLaneletPose();
  lanelet_pose.s =
    lanelet_pose.s + (twist_new.linear.x + entity_status->getTwist().linear.x) / 2.0 * step_time;
  const auto canonicalized = hdmap_utils->canonicalizeLaneletPose(lanelet_pose, route_lanelets);
  if (
    const auto canonicalized_lanelet_pose =
      std::get<std::optional<traffic_simulator::LaneletPose>>(canonicalized)) {
    // If canonicalize succeed, set canonicalized pose and set other values.
    traffic_simulator::EntityStatus entity_status_updated;
    {
      entity_status_updated.time = current_time + step_time;
      entity_status_updated.lanelet_pose = canonicalized_lanelet_pose.value();
      entity_status_updated.action_status.twist = twist_new;
      entity_status_updated.action_status.accel = accel_new;
      entity_status_updated.action_status.linear_jerk = linear_jerk_new;
      entity_status_updated.pose = hdmap_utils->toMapPose(canonicalized_lanelet_pose.value()).pose;
    }
    return traffic_simulator::CanonicalizedEntityStatus(entity_status_updated, hdmap_utils);
  } else {
    // If canonicalize failed, set end of road lanelet pose.
    if (const auto end_of_road_lanelet_id = std::get<std::optional<std::int64_t>>(canonicalized)) {
      if (lanelet_pose.s < 0) {
        traffic_simulator::LaneletPose end_of_road_lanelet_pose;
        {
          end_of_road_lanelet_pose.lanelet_id = end_of_road_lanelet_id.value();
          end_of_road_lanelet_pose.s = 0;
          end_of_road_lanelet_pose.offset = lanelet_pose.offset;
          end_of_road_lanelet_pose.rpy = lanelet_pose.rpy;
        }
        traffic_simulator::EntityStatus entity_status_updated;
        {
          entity_status_updated.time = current_time + step_time;
          entity_status_updated.lanelet_pose = end_of_road_lanelet_pose;
          entity_status_updated.action_status.twist = twist_new;
          entity_status_updated.action_status.accel = accel_new;
          entity_status_updated.action_status.linear_jerk = linear_jerk_new;
          entity_status_updated.pose = hdmap_utils->toMapPose(end_of_road_lanelet_pose).pose;
        }
        return traffic_simulator::CanonicalizedEntityStatus(entity_status_updated, hdmap_utils);
      } else {
        traffic_simulator::LaneletPose end_of_road_lanelet_pose;
        {
          end_of_road_lanelet_pose.lanelet_id = end_of_road_lanelet_id.value();
          end_of_road_lanelet_pose.s =
            hdmap_utils->getLaneletLength(end_of_road_lanelet_id.value());
          end_of_road_lanelet_pose.offset = lanelet_pose.offset;
          end_of_road_lanelet_pose.rpy = lanelet_pose.rpy;
        }
        traffic_simulator::EntityStatus entity_status_updated;
        {
          entity_status_updated.time = current_time + step_time;
          entity_status_updated.lanelet_pose = end_of_road_lanelet_pose;
          entity_status_updated.action_status.twist = twist_new;
          entity_status_updated.action_status.accel = accel_new;
          entity_status_updated.action_status.linear_jerk = linear_jerk_new;
          entity_status_updated.pose = hdmap_utils->toMapPose(end_of_road_lanelet_pose).pose;
        }
        return traffic_simulator::CanonicalizedEntityStatus(entity_status_updated, hdmap_utils);
      }
    } else {
      THROW_SIMULATION_ERROR("Failed to find trailing lanelet_id.");
    }
  }
}

auto ActionNode::calculateUpdatedEntityStatusInWorldFrame(
  double target_speed, const traffic_simulator_msgs::msg::DynamicConstraints & constraints) const
  -> traffic_simulator::CanonicalizedEntityStatus
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
  traffic_simulator::EntityStatus entity_status_updated;
  entity_status_updated.time = current_time + step_time;
  entity_status_updated.pose = pose_new;
  entity_status_updated.action_status.twist = twist_new;
  entity_status_updated.action_status.accel = accel_new;
  entity_status_updated.action_status.linear_jerk = linear_jerk_new;
  entity_status_updated.lanelet_pose_valid = false;
  entity_status_updated.lanelet_pose = traffic_simulator::LaneletPose();
  return traffic_simulator::CanonicalizedEntityStatus(entity_status_updated, hdmap_utils);
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
