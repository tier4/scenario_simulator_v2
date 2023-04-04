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
#include <rclcpp/rclcpp.hpp>
#include <scenario_simulator_exception/exception.hpp>
#include <set>
#include <string>
#include <traffic_simulator/behavior/longitudinal_speed_planning.hpp>
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
  if (!getInput<std::shared_ptr<traffic_simulator::TrafficLightManagerBase>>(
        "traffic_light_manager", traffic_light_manager)) {
    THROW_SIMULATION_ERROR("failed to get input traffic_light_manager in ActionNode");
  }
  if (!getInput<std::shared_ptr<traffic_simulator::CanonicalizedEntityStatusType>>(
        "entity_status", entity_status)) {
    THROW_SIMULATION_ERROR("failed to get input entity_status in ActionNode");
  }

  if (!getInput<boost::optional<double>>("target_speed", target_speed)) {
    target_speed = boost::none;
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
  return std::clamp(getCurrentTwist().linear.x * 5.0, 20.0, 50.0);
}

auto ActionNode::stopEntity() const -> void
{
  entity_status->setTime(current_time + step_time);
  entity_status->setTwist();
  entity_status->setAccel();
}

auto ActionNode::getOtherEntityStatus(std::int64_t lanelet_id) const
  -> std::vector<traffic_simulator::CanonicalizedEntityStatusType>
{
  std::vector<traffic_simulator::CanonicalizedEntityStatusType> ret;
  for (const auto & status : other_entity_status) {
    if (
      status.second.laneMatchingSucceed() &&
      static_cast<traffic_simulator::LaneletPoseType>(status.second).lanelet_id == lanelet_id) {
      ret.emplace_back(status.second);
    }
  }
  return ret;
}

auto ActionNode::getYieldStopDistance(const std::vector<std::int64_t> & following_lanelets) const
  -> boost::optional<double>
{
  std::set<double> distances;
  for (const auto & lanelet : following_lanelets) {
    const auto right_of_way_ids = hdmap_utils->getRightOfWayLaneletIds(lanelet);
    for (const auto right_of_way_id : right_of_way_ids) {
      const auto other_status = getOtherEntityStatus(right_of_way_id);
      if (other_status.size() != 0 && entity_status->laneMatchingSucceed()) {
        const auto lanelet_pose = getLaneletPose();
        auto distance =
          hdmap_utils->getLongitudinalDistance(lanelet_pose.lanelet_id, lanelet_pose.s, lanelet, 0);
        if (distance) {
          distances.insert(distance.get());
        }
      }
    }
    if (distances.size() != 0) {
      return *distances.begin();
    }
  }
  return boost::none;
}

auto ActionNode::getRightOfWayEntities(const std::vector<std::int64_t> & following_lanelets) const
  -> std::vector<traffic_simulator::CanonicalizedEntityStatusType>
{
  std::vector<traffic_simulator::CanonicalizedEntityStatusType> ret;
  const auto lanelet_ids_list = hdmap_utils->getRightOfWayLaneletIds(following_lanelets);
  for (const auto & status : other_entity_status) {
    for (const auto & following_lanelet : following_lanelets) {
      for (const std::int64_t & lanelet_id : lanelet_ids_list.at(following_lanelet)) {
        if (
          status.second.laneMatchingSucceed() &&
          static_cast<traffic_simulator::LaneletPoseType>(status.second).lanelet_id == lanelet_id) {
          ret.emplace_back(status.second);
        }
      }
    }
  }
  return ret;
}

auto ActionNode::getRightOfWayEntities() const
  -> std::vector<traffic_simulator::CanonicalizedEntityStatusType>
{
  if (!entity_status->laneMatchingSucceed()) {
    return {};
  }
  std::vector<traffic_simulator::CanonicalizedEntityStatusType> ret;
  const auto lanelet_ids = hdmap_utils->getRightOfWayLaneletIds(getLaneletPose().lanelet_id);
  if (lanelet_ids.empty()) {
    return ret;
  }
  for (const auto & status : other_entity_status) {
    for (const std::int64_t & lanelet_id : lanelet_ids) {
      if (
        status.second.laneMatchingSucceed() &&
        static_cast<traffic_simulator::LaneletPoseType>(status.second).lanelet_id == lanelet_id) {
        ret.emplace_back(status.second);
      }
    }
  }
  return ret;
}

auto ActionNode::getDistanceToTrafficLightStopLine(
  const std::vector<std::int64_t> & route_lanelets,
  const math::geometry::CatmullRomSplineInterface & spline) const -> boost::optional<double>
{
  const auto traffic_light_ids = hdmap_utils->getTrafficLightIdsOnPath(route_lanelets);
  if (traffic_light_ids.empty()) {
    return boost::none;
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
        collision_points.insert(collision_point.get());
      }
    }
  }
  if (collision_points.empty()) {
    return boost::none;
  }
  return *collision_points.begin();
}

auto ActionNode::getDistanceToStopLine(
  const std::vector<std::int64_t> & route_lanelets,
  const std::vector<geometry_msgs::msg::Point> & waypoints) const -> boost::optional<double>
{
  return hdmap_utils->getDistanceToStopLine(route_lanelets, waypoints);
}

auto ActionNode::getDistanceToFrontEntity(
  const math::geometry::CatmullRomSplineInterface & spline) const -> boost::optional<double>
{
  auto name = getFrontEntityName(spline);
  if (!name) {
    return boost::none;
  }
  return getDistanceToTargetEntityPolygon(spline, name.get());
}

auto ActionNode::getFrontEntityName(const math::geometry::CatmullRomSplineInterface & spline) const
  -> boost::optional<std::string>
{
  std::vector<double> distances;
  std::vector<std::string> entities;
  for (const auto & each : other_entity_status) {
    const auto distance = getDistanceToTargetEntityPolygon(spline, each.first);
    const auto quat = quaternion_operation::getRotation(
      getEntityOrientation(),
      static_cast<geometry_msgs::msg::Pose>(other_entity_status.at(each.first)).orientation);
    /**
     * @note hard-coded parameter, if the Yaw value of RPY is in ~1.5708 -> 1.5708, entity is a candidate of front entity.
     */
    if (
      std::fabs(quaternion_operation::convertQuaternionToEulerAngle(quat).z) <=
      boost::math::constants::half_pi<double>()) {
      if (distance && distance.get() < 40) {
        entities.emplace_back(each.first);
        distances.emplace_back(distance.get());
      }
    }
  }
  if (entities.size() != distances.size()) {
    THROW_SIMULATION_ERROR("size of entities and distances vector does not match.");
  }
  if (distances.empty()) {
    return boost::none;
  }
  std::vector<double>::iterator iter = std::min_element(distances.begin(), distances.end());
  size_t index = std::distance(distances.begin(), iter);
  return entities[index];
}

auto ActionNode::getDistanceToTargetEntityOnCrosswalk(
  const math::geometry::CatmullRomSplineInterface & spline,
  const traffic_simulator::CanonicalizedEntityStatusType & status) const -> boost::optional<double>
{
  if (status.laneMatchingSucceed()) {
    return spline.getCollisionPointIn2D(
      hdmap_utils->getLaneletPolygon(
        static_cast<traffic_simulator::LaneletPoseType>(status).lanelet_id),
      false, true);
  }
  return boost::none;
}

auto ActionNode::getEntityStatus(const std::string & target_name) const
  -> traffic_simulator::CanonicalizedEntityStatusType
{
  if (other_entity_status.find(target_name) != other_entity_status.end()) {
    return traffic_simulator::CanonicalizedEntityStatusType(other_entity_status.at(target_name));
  }
  THROW_SIMULATION_ERROR("other entity : ", target_name, " does not exist.");
}

auto ActionNode::getDistanceToTargetEntityPolygon(
  const math::geometry::CatmullRomSplineInterface & spline, const std::string target_name,
  double width_extension_right, double width_extension_left, double length_extension_front,
  double length_extension_rear) const -> boost::optional<double>
{
  const auto status = getEntityStatus(target_name);
  if (status.laneMatchingSucceed()) {
    return getDistanceToTargetEntityPolygon(
      spline, status, width_extension_right, width_extension_left, length_extension_front,
      length_extension_rear);
  }
  return boost::none;
}

auto ActionNode::getDistanceToTargetEntityPolygon(
  const math::geometry::CatmullRomSplineInterface & spline,
  const traffic_simulator::CanonicalizedEntityStatusType & status, double width_extension_right,
  double width_extension_left, double length_extension_front, double length_extension_rear) const
  -> boost::optional<double>
{
  if (status.laneMatchingSucceed()) {
    const auto polygon = math::geometry::transformPoints(
      static_cast<traffic_simulator::EntityStatusType>(status).pose,
      math::geometry::getPointsFromBbox(
        static_cast<traffic_simulator::EntityStatusType>(status).bounding_box,
        width_extension_right, width_extension_left, length_extension_front,
        length_extension_rear));
    return spline.getCollisionPointIn2D(polygon, false, true);
  }
  return boost::none;
}

auto ActionNode::getDistanceToConflictingEntity(
  const std::vector<std::int64_t> & route_lanelets,
  const math::geometry::CatmullRomSplineInterface & spline) const -> boost::optional<double>
{
  auto crosswalk_entity_status = getConflictingEntityStatusOnCrossWalk(route_lanelets);
  auto lane_entity_status = getConflictingEntityStatusOnLane(route_lanelets);
  std::set<double> distances;
  for (const auto & status : crosswalk_entity_status) {
    const auto s = getDistanceToTargetEntityOnCrosswalk(spline, status);
    if (s) {
      distances.insert(s.get());
    }
  }
  for (const auto & status : lane_entity_status) {
    const auto s = getDistanceToTargetEntityPolygon(spline, status, 0.0, 0.0, 0.0, 1.0);
    if (s) {
      distances.insert(s.get());
    }
  }
  if (distances.empty()) {
    return boost::none;
  }
  return *distances.begin();
}

auto ActionNode::getConflictingEntityStatusOnCrossWalk(
  const std::vector<std::int64_t> & route_lanelets) const
  -> std::vector<traffic_simulator::CanonicalizedEntityStatusType>
{
  std::vector<traffic_simulator::CanonicalizedEntityStatusType> conflicting_entity_status;
  auto conflicting_crosswalks = hdmap_utils->getConflictingCrosswalkIds(route_lanelets);
  for (const auto & status : other_entity_status) {
    if (
      status.second.laneMatchingSucceed() &&
      std::count(
        conflicting_crosswalks.begin(), conflicting_crosswalks.end(),
        static_cast<traffic_simulator::LaneletPoseType>(status.second).lanelet_id) >= 1) {
      conflicting_entity_status.emplace_back(status.second);
    }
  }
  return conflicting_entity_status;
}

auto ActionNode::getConflictingEntityStatusOnLane(const std::vector<std::int64_t> & route_lanelets)
  const -> std::vector<traffic_simulator::CanonicalizedEntityStatusType>
{
  std::vector<traffic_simulator::CanonicalizedEntityStatusType> conflicting_entity_status;
  auto conflicting_lanes = hdmap_utils->getConflictingLaneIds(route_lanelets);
  for (const auto & status : other_entity_status) {
    if (
      status.second.laneMatchingSucceed() &&
      std::count(
        conflicting_lanes.begin(), conflicting_lanes.end(),
        static_cast<traffic_simulator::LaneletPoseType>(status.second).lanelet_id) >= 1) {
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
        static_cast<traffic_simulator::LaneletPoseType>(status.second).lanelet_id) >= 1) {
      return true;
    }
    if (
      status.second.laneMatchingSucceed() &&
      std::count(
        conflicting_lanes.begin(), conflicting_lanes.end(),
        static_cast<traffic_simulator::LaneletPoseType>(status.second).lanelet_id) >= 1) {
      return true;
    }
  }
  return false;
}

auto ActionNode::calculateUpdatedEntityStatus(
  double target_speed, const traffic_simulator_msgs::msg::DynamicConstraints & constraints) const
  -> traffic_simulator::CanonicalizedEntityStatusType
{
  const auto speed_planner =
    traffic_simulator::longitudinal_speed_planning::LongitudinalSpeedPlanner(
      step_time, getEntityName());
  const auto dynamics =
    speed_planner.getDynamicStates(target_speed, constraints, getCurrentTwist(), getCurrentAccel());

  double linear_jerk_new = std::get<2>(dynamics);
  geometry_msgs::msg::Accel accel_new = std::get<1>(dynamics);
  geometry_msgs::msg::Twist twist_new = std::get<0>(dynamics);

  auto lanelet_pose = getLaneletPose();
  lanelet_pose.s =
    lanelet_pose.s + (twist_new.linear.x + getCurrentTwist().linear.x) / 2.0 * step_time;
  const auto canonicalized = hdmap_utils->canonicalizeLaneletPose(lanelet_pose, route_lanelets);
  if (
    const auto canonicalized_lanelet_pose =
      std::get<boost::optional<traffic_simulator_msgs::msg::LaneletPose>>(canonicalized)) {
    // If canonicalize succeed, set canonicalized pose and set other values.
    traffic_simulator::EntityStatusType entity_status_updated;
    {
      entity_status_updated.time = current_time + step_time;
      entity_status_updated.lanelet_pose = canonicalized_lanelet_pose.get();
      entity_status_updated.action_status.twist = twist_new;
      entity_status_updated.action_status.accel = accel_new;
      entity_status_updated.action_status.linear_jerk = linear_jerk_new;
      entity_status_updated.pose = hdmap_utils->toMapPose(canonicalized_lanelet_pose.get()).pose;
    }
    return traffic_simulator::CanonicalizedEntityStatusType(entity_status_updated, hdmap_utils);
  } else {
    // If canonicalize failed, set end of road lanelet pose.
    if (
      const auto end_of_road_lanelet_id = std::get<boost::optional<std::int64_t>>(canonicalized)) {
      if (lanelet_pose.s < 0) {
        traffic_simulator_msgs::msg::LaneletPose end_of_road_lanelet_pose;
        {
          end_of_road_lanelet_pose.lanelet_id = end_of_road_lanelet_id.get();
          end_of_road_lanelet_pose.s = 0;
          end_of_road_lanelet_pose.offset = lanelet_pose.offset;
          end_of_road_lanelet_pose.rpy = lanelet_pose.rpy;
        }
        traffic_simulator::EntityStatusType entity_status_updated;
        {
          entity_status_updated.time = current_time + step_time;
          entity_status_updated.lanelet_pose = end_of_road_lanelet_pose;
          entity_status_updated.action_status.twist = twist_new;
          entity_status_updated.action_status.accel = accel_new;
          entity_status_updated.action_status.linear_jerk = linear_jerk_new;
          entity_status_updated.pose = hdmap_utils->toMapPose(end_of_road_lanelet_pose).pose;
        }
        return traffic_simulator::CanonicalizedEntityStatusType(entity_status_updated, hdmap_utils);
      } else {
        traffic_simulator_msgs::msg::LaneletPose end_of_road_lanelet_pose;
        {
          end_of_road_lanelet_pose.lanelet_id = end_of_road_lanelet_id.get();
          end_of_road_lanelet_pose.s = hdmap_utils->getLaneletLength(end_of_road_lanelet_id.get());
          end_of_road_lanelet_pose.offset = lanelet_pose.offset;
          end_of_road_lanelet_pose.rpy = lanelet_pose.rpy;
        }
        traffic_simulator::EntityStatusType entity_status_updated;
        {
          entity_status_updated.time = current_time + step_time;
          entity_status_updated.lanelet_pose = end_of_road_lanelet_pose;
          entity_status_updated.action_status.twist = twist_new;
          entity_status_updated.action_status.accel = accel_new;
          entity_status_updated.action_status.linear_jerk = linear_jerk_new;
          entity_status_updated.pose = hdmap_utils->toMapPose(end_of_road_lanelet_pose).pose;
        }
        return traffic_simulator::CanonicalizedEntityStatusType(entity_status_updated, hdmap_utils);
      }
    } else {
      THROW_SIMULATION_ERROR("Failed to find trailing laenlet_id.");
    }
  }
}

auto ActionNode::calculateUpdatedEntityStatusInWorldFrame(
  double target_speed, const traffic_simulator_msgs::msg::DynamicConstraints & constraints) const
  -> traffic_simulator::CanonicalizedEntityStatusType
{
  const auto speed_planner =
    traffic_simulator::longitudinal_speed_planning::LongitudinalSpeedPlanner(
      step_time, getEntityName());
  const auto dynamics =
    speed_planner.getDynamicStates(target_speed, constraints, getCurrentTwist(), getCurrentAccel());
  double linear_jerk_new = std::get<2>(dynamics);
  geometry_msgs::msg::Accel accel_new = std::get<1>(dynamics);
  geometry_msgs::msg::Twist twist_new = std::get<0>(dynamics);
  geometry_msgs::msg::Pose pose_new;
  geometry_msgs::msg::Vector3 angular_trans_vec;
  angular_trans_vec.z = twist_new.angular.z * step_time;
  geometry_msgs::msg::Quaternion angular_trans_quat =
    quaternion_operation::convertEulerAngleToQuaternion(angular_trans_vec);
  pose_new.orientation = quaternion_operation::rotation(getEntityOrientation(), angular_trans_quat);
  Eigen::Vector3d trans_vec;
  trans_vec(0) = twist_new.linear.x * step_time;
  trans_vec(1) = twist_new.linear.y * step_time;
  trans_vec(2) = 0;
  Eigen::Matrix3d rotation_mat = quaternion_operation::getRotationMatrix(pose_new.orientation);
  trans_vec = rotation_mat * trans_vec;
  pose_new.position.x = trans_vec(0) + getEntityPosition().x;
  pose_new.position.y = trans_vec(1) + getEntityPosition().y;
  pose_new.position.z = trans_vec(2) + getEntityPosition().z;
  traffic_simulator::EntityStatusType entity_status_updated;
  entity_status_updated.time = current_time + step_time;
  entity_status_updated.pose = pose_new;
  entity_status_updated.action_status.twist = twist_new;
  entity_status_updated.action_status.accel = accel_new;
  entity_status_updated.action_status.linear_jerk = linear_jerk_new;
  entity_status_updated.lanelet_pose_valid = false;
  entity_status_updated.lanelet_pose = traffic_simulator::LaneletPoseType();
  return traffic_simulator::CanonicalizedEntityStatusType(entity_status_updated, hdmap_utils);
}

auto ActionNode::calculateStopDistance(
  const traffic_simulator_msgs::msg::DynamicConstraints & constraints) const -> double
{
  return traffic_simulator::longitudinal_speed_planning::LongitudinalSpeedPlanner(
           step_time, getEntityName())
    .getRunningDistance(
      0, constraints, getCurrentTwist(), getCurrentAccel(), getCurrentLinearJerk());
}

auto ActionNode::getCurrentTwist() const noexcept -> geometry_msgs::msg::Twist
{
  return static_cast<traffic_simulator::EntityStatusType>(*entity_status).action_status.twist;
}

auto ActionNode::getCurrentAccel() const noexcept -> geometry_msgs::msg::Accel
{
  return static_cast<traffic_simulator::EntityStatusType>(*entity_status).action_status.accel;
}

auto ActionNode::getCurrentLinearJerk() const noexcept -> double
{
  return static_cast<traffic_simulator::EntityStatusType>(*entity_status).action_status.linear_jerk;
}

auto ActionNode::getBoundingBox() const noexcept -> traffic_simulator_msgs::msg::BoundingBox
{
  return static_cast<traffic_simulator::EntityStatusType>(*entity_status).bounding_box;
}

auto ActionNode::getEntityPose() const noexcept -> geometry_msgs::msg::Pose
{
  return static_cast<geometry_msgs::msg::Pose>(*entity_status);
}

auto ActionNode::getEntityPosition() const noexcept -> geometry_msgs::msg::Point
{
  return getEntityPose().position;
}

auto ActionNode::getEntityOrientation() const noexcept -> geometry_msgs::msg::Quaternion
{
  return getEntityPose().orientation;
}

auto ActionNode::getActionStatus() const noexcept -> traffic_simulator_msgs::msg::ActionStatus
{
  return static_cast<traffic_simulator::EntityStatusType>(*entity_status).action_status;
}

auto ActionNode::getEntityName() const noexcept -> std::string
{
  return static_cast<traffic_simulator::EntityStatusType>(*entity_status).name;
}

auto ActionNode::getLaneletPose() const -> traffic_simulator::LaneletPoseType
{
  return static_cast<traffic_simulator::LaneletPoseType>(*entity_status);
}
}  // namespace entity_behavior
