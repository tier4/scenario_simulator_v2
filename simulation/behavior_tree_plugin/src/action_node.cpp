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
#include <geometry/vector3/operator.hpp>
#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <scenario_simulator_exception/exception.hpp>
#include <set>
#include <string>
#include <traffic_simulator/behavior/longitudinal_speed_planning.hpp>
#include <traffic_simulator/helper/helper.hpp>
#include <traffic_simulator/utils/distance.hpp>
#include <traffic_simulator/utils/pose.hpp>
#include <traffic_simulator/utils/route.hpp>
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
  if (!getInput<std::shared_ptr<EuclideanDistancesMap>>(
        "euclidean_distances_map", euclidean_distances_map)) {
    euclidean_distances_map = std::make_shared<EuclideanDistancesMap>();
  }
  if (!getInput<traffic_simulator_msgs::msg::BehaviorParameter>(
        "behavior_parameter", behavior_parameter)) {
    behavior_parameter = traffic_simulator_msgs::msg::BehaviorParameter();
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
  other_status.reserve(other_entity_status.size());
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

  if (euclidean_distances_map != nullptr) {
    std::map<double, std::string> local_euclidean_distances_map;
    const double stop_distance = calculateStopDistance(behavior_parameter.dynamic_constraints);
    const double horizon = spline.getLength() > stop_distance ? spline.getLength() : stop_distance;
    for (const auto & [name_pair, euclidean_distance] : *euclidean_distances_map) {
      /**
       * @note Euclidean distance is here used as a "rough" distance to filter only NPCs which possibly are in range of current horizon. Because euclidean distance is the shortest possible distance comparing it with horizon will never omit NPCs for which actual lane distance is in range of horizon.
       */
      if (euclidean_distance < horizon) {
        if (name_pair.first == canonicalized_entity_status->getName()) {
          local_euclidean_distances_map.emplace(euclidean_distance, name_pair.second);
        } else if (name_pair.second == canonicalized_entity_status->getName()) {
          local_euclidean_distances_map.emplace(euclidean_distance, name_pair.first);
        }
      }
    }

    for (const auto & [euclidean_distance, name] : local_euclidean_distances_map) {
      const auto & other_status = other_entity_status.at(name);
      if (
        const auto & other_canonicalized_lanelet_pose =
          other_status.getCanonicalizedLaneletPose()) {
        const auto quaternion = math::geometry::getRotation(
          canonicalized_entity_status->getMapPose().orientation,
          other_status.getMapPose().orientation);
        const auto yaw = math::geometry::convertQuaternionToEulerAngle(quaternion).z;

        if (std::fabs(yaw) <= front_entity_angle_threshold) {
          const auto longitudinal_distance =
            traffic_simulator::distance::splineDistanceToBoundingBox(
              spline, canonicalized_entity_status->getCanonicalizedLaneletPose().value(),
              canonicalized_entity_status->getBoundingBox(),
              other_canonicalized_lanelet_pose.value(), other_status.getBoundingBox());

          if (longitudinal_distance && longitudinal_distance.value() < horizon) {
            return name;
          }
        }
      }
    }
  }
  return std::nullopt;
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
    entity_status_updated.lanelet_pose = traffic_simulator::route::moveAlongLaneletPose(
      canonicalized_lanelet_pose.value(), route_lanelets, distance);
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
  using math::geometry::operator+;
  using math::geometry::operator+=;

  constexpr bool desired_velocity_is_global{false};

  const auto include_crosswalk = [](const auto & entity_type) {
    return (traffic_simulator_msgs::msg::EntityType::PEDESTRIAN == entity_type.type) ||
           (traffic_simulator_msgs::msg::EntityType::MISC_OBJECT == entity_type.type);
  }(canonicalized_entity_status->getType());

  const auto matching_distance = default_matching_distance_for_lanelet_pose_calculation;

  const auto build_updated_pose =
    [&include_crosswalk, &matching_distance](
      const std::shared_ptr<traffic_simulator::CanonicalizedEntityStatus> & status,
      const geometry_msgs::msg::Twist & desired_twist, const double time_step) {
      geometry_msgs::msg::Pose updated_pose;

      /// @note Apply yaw change (delta rotation) in radians: yaw_angular_speed (rad/s) * time_step (s)
      geometry_msgs::msg::Vector3 delta_rotation;
      delta_rotation = desired_twist.angular * time_step;
      const auto delta_quaternion = math::geometry::convertEulerAngleToQuaternion(delta_rotation);
      updated_pose.orientation = status->getMapPose().orientation * delta_quaternion;

      /// @note Apply position change
      /// @todo first determine global desired_velocity, calculate position change using it
      /// then pass the same global desired_velocity to updatePositionForLaneletTransition()
      const Eigen::Matrix3d rotation_matrix =
        math::geometry::getRotationMatrix(updated_pose.orientation);
      const auto translation = Eigen::Vector3d(
        desired_twist.linear.x * time_step, desired_twist.linear.y * time_step,
        desired_twist.linear.z * time_step);
      const Eigen::Vector3d delta_position = rotation_matrix * translation;
      updated_pose.position = status->getMapPose().position + delta_position;

      /// @note If it is the transition between lanelets: overwrite position to improve precision
      if (const auto canonicalized_lanelet_pose = status->getCanonicalizedLaneletPose()) {
        const auto estimated_next_canonicalized_lanelet_pose =
          traffic_simulator::pose::toCanonicalizedLaneletPose(
            updated_pose, status->getBoundingBox(), include_crosswalk, matching_distance);
        if (estimated_next_canonicalized_lanelet_pose) {
          const auto next_lanelet_id = static_cast<traffic_simulator::LaneletPose>(
                                         estimated_next_canonicalized_lanelet_pose.value())
                                         .lanelet_id;
          if (  /// @note Handle lanelet transition
            const auto updated_position =
              traffic_simulator::pose::updatePositionForLaneletTransition(
                canonicalized_lanelet_pose.value(), next_lanelet_id, desired_twist.linear,
                desired_velocity_is_global, time_step)) {
            updated_pose.position = updated_position.value();
          }
        }
      }
      return updated_pose;
    };

  const auto speed_planner =
    traffic_simulator::longitudinal_speed_planning::LongitudinalSpeedPlanner(
      step_time, canonicalized_entity_status->getName());
  const auto dynamics = speed_planner.getDynamicStates(
    local_target_speed, constraints, canonicalized_entity_status->getTwist(),
    canonicalized_entity_status->getAccel());
  const auto linear_jerk_new = std::get<2>(dynamics);
  const auto & accel_new = std::get<1>(dynamics);
  const auto & twist_new = std::get<0>(dynamics);
  const auto pose_new = build_updated_pose(canonicalized_entity_status, twist_new, step_time);

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
