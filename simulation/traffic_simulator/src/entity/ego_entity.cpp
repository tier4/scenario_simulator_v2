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

#include <boost/lexical_cast.hpp>
#include <concealer/field_operator_application.hpp>
#include <concealer/launch.hpp>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <system_error>
#include <thread>
#include <traffic_simulator/entity/ego_entity.hpp>
#include <traffic_simulator/utils/pose.hpp>
#include <traffic_simulator_msgs/msg/waypoints_array.hpp>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

namespace traffic_simulator
{
namespace entity
{
EgoEntity::EgoEntity(
  const std::string & name, const CanonicalizedEntityStatus & entity_status,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr,
  const traffic_simulator_msgs::msg::VehicleParameters & parameters,
  const Configuration & configuration,
  const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & node_parameters)
: VehicleEntity(name, entity_status, hdmap_utils_ptr, parameters), FieldOperatorApplication([&]() {
    if (const auto architecture_type =
          getParameter<std::string>(node_parameters, "architecture_type", "awf/universe/20240605");
        architecture_type.find("awf/universe") != std::string::npos) {
      auto parameters = getParameter<std::vector<std::string>>(node_parameters, "autoware.", {});

      // clang-format off
      parameters.push_back("map_path:=" + configuration.map_path.string());
      parameters.push_back("lanelet2_map_file:=" + configuration.getLanelet2MapFile());
      parameters.push_back("pointcloud_map_file:=" + configuration.getPointCloudMapFile());
      parameters.push_back("sensor_model:=" + getParameter<std::string>(node_parameters, "sensor_model"));
      parameters.push_back("vehicle_model:=" + getParameter<std::string>(node_parameters, "vehicle_model"));
      parameters.push_back("rviz_config:=" + getParameter<std::string>(node_parameters, "rviz_config"));
      parameters.push_back("scenario_simulation:=true");
      parameters.push_back("use_foa:=false");
      parameters.push_back("perception/enable_traffic_light:=" + std::string(architecture_type >= "awf/universe/20230906" ? "true" : "false"));
      parameters.push_back("use_sim_time:=" + std::string(getParameter<bool>(node_parameters, "use_sim_time", false) ? "true" : "false"));
      parameters.push_back("localization_sim_mode:=" + std::string(getParameter<bool>(node_parameters, "simulate_localization") ? "api" : "pose_twist_estimator"));
      // clang-format on

      return getParameter<bool>(node_parameters, "launch_autoware", true)
               ? concealer::ros2_launch(
                   getParameter<std::string>(node_parameters, "autoware_launch_package"),
                   getParameter<std::string>(node_parameters, "autoware_launch_file"), parameters)
               : 0;
    } else {
      throw common::SemanticError(
        "Unexpected architecture_type ", std::quoted(architecture_type), " was given.");
    }
  }())
{
}

auto EgoEntity::engage() -> void { FieldOperatorApplication::engage(); }

auto EgoEntity::isEngaged() const -> bool { return engaged(); }

auto EgoEntity::isEngageable() const -> bool { return engageable(); }

auto EgoEntity::sendCooperateCommand(const std::string & module_name, const std::string & command)
  -> void
{
  FieldOperatorApplication::sendCooperateCommand(module_name, command);
}

auto EgoEntity::requestAutoModeForCooperation(const std::string & module_name, bool enable) -> void
{
  FieldOperatorApplication::requestAutoModeForCooperation(module_name, enable);
}

auto EgoEntity::getMinimumRiskManeuverBehaviorName() const -> std::string
{
  return minimum_risk_maneuver_behavior;
}

auto EgoEntity::getMinimumRiskManeuverStateName() const -> std::string
{
  return minimum_risk_maneuver_state;
}

auto EgoEntity::getEmergencyStateName() const -> std::string { return minimum_risk_maneuver_state; }

auto EgoEntity::getTurnIndicatorsCommandName() const -> std::string
{
  switch (getTurnIndicatorsCommand().command) {
    case autoware_vehicle_msgs::msg::TurnIndicatorsCommand::DISABLE:
      return "DISABLE";
    case autoware_vehicle_msgs::msg::TurnIndicatorsCommand::ENABLE_LEFT:
      return "ENABLE_LEFT";
    case autoware_vehicle_msgs::msg::TurnIndicatorsCommand::ENABLE_RIGHT:
      return "ENABLE_RIGHT";
    case autoware_vehicle_msgs::msg::TurnIndicatorsCommand::NO_COMMAND:
      return "NO_COMMAND";
    default:
      return "";
  }
}

auto EgoEntity::getCurrentAction() const -> std::string { return autoware_state; }

auto EgoEntity::getBehaviorParameter() const -> traffic_simulator_msgs::msg::BehaviorParameter
{
  /**
   * @brief TODO, Input values get from autoware.
   */
  return behavior_parameter_;
}

auto EgoEntity::getEntityTypename() const -> const std::string &
{
  static const std::string result = "EgoEntity";
  return result;
}

auto EgoEntity::getObstacle() -> std::optional<traffic_simulator_msgs::msg::Obstacle>
{
  return std::nullopt;
}

auto EgoEntity::getRouteLanelets(double /*unused horizon*/) -> lanelet::Ids
{
  lanelet::Ids ids{};

  for (const auto & point : getPathWithLaneId().points) {
    ids += point.lane_ids;
  }

  return ids;
}

auto EgoEntity::getCurrentPose() const -> const geometry_msgs::msg::Pose &
{
  return status_->getMapPose();
}

auto EgoEntity::getWaypoints() -> const traffic_simulator_msgs::msg::WaypointsArray
{
  return FieldOperatorApplication::getWaypoints();
}

void EgoEntity::updateFieldOperatorApplication()
{
  rethrow();
  spinSome();
}

void EgoEntity::onUpdate(double current_time, double step_time)
{
  EntityBase::onUpdate(current_time, step_time);

  if (is_controlled_by_simulator_) {
    if (
      const auto non_canonicalized_updated_status =
        traffic_simulator::follow_trajectory::makeUpdatedStatus(
          static_cast<traffic_simulator::EntityStatus>(*status_), *polyline_trajectory_,
          behavior_parameter_, hdmap_utils_ptr_, step_time,
          getDefaultMatchingDistanceForLaneletPoseCalculation(),
          target_speed_ ? target_speed_.value() : status_->getTwist().linear.x)) {
      // prefer current lanelet on ss2 side
      setStatus(non_canonicalized_updated_status.value(), status_->getLaneletIds());
    } else {
      enableAutowareControl();
      is_controlled_by_simulator_ = false;
    }
  }
  if (not is_controlled_by_simulator_) {
    updateEntityStatusTimestamp(current_time + step_time);
  }
  updateFieldOperatorApplication();

  EntityBase::onPostUpdate(current_time, step_time);
}

void EgoEntity::requestAcquirePosition(const CanonicalizedLaneletPose & lanelet_pose)
{
  requestAssignRoute({lanelet_pose});
}

void EgoEntity::requestAcquirePosition(const geometry_msgs::msg::Pose & map_pose)
{
  requestAssignRoute({map_pose});
}

void EgoEntity::requestAssignRoute(const std::vector<CanonicalizedLaneletPose> & waypoints)
{
  std::vector<geometry_msgs::msg::Pose> route;
  for (const auto & waypoint : waypoints) {
    route.push_back(static_cast<geometry_msgs::msg::Pose>(waypoint));
  }
  requestAssignRoute(route);
}

void EgoEntity::requestAssignRoute(const std::vector<geometry_msgs::msg::Pose> & waypoints)
{
  std::vector<geometry_msgs::msg::PoseStamped> route;
  for (const auto & waypoint : waypoints) {
    geometry_msgs::msg::PoseStamped pose_stamped;
    {
      pose_stamped.header.frame_id = "map";
      pose_stamped.pose = waypoint;
    }
    route.push_back(pose_stamped);
  }

  requestClearRoute();
  if (not initialized) {
    initialize(getMapPose());
    plan(route);
    // NOTE: engage() will be executed at simulation-time 0.
  } else {
    plan(route);
    FieldOperatorApplication::engage();
  }
}

auto EgoEntity::isControlledBySimulator() const -> bool { return is_controlled_by_simulator_; }

auto EgoEntity::requestFollowTrajectory(
  const std::shared_ptr<traffic_simulator_msgs::msg::PolylineTrajectory> & parameter) -> void
{
  polyline_trajectory_ = parameter;
  VehicleEntity::requestFollowTrajectory(parameter);
  is_controlled_by_simulator_ = true;
}

auto EgoEntity::requestLaneChange(const lanelet::Id) -> void
{
  THROW_SEMANTIC_ERROR(
    "From scenario, a lane change was requested to Ego type entity ", std::quoted(name),
    " In general, such a request is an error, since Ego cars make autonomous decisions about "
    "everything but their destination");
}

auto EgoEntity::requestLaneChange(const traffic_simulator::lane_change::Parameter &) -> void
{
  THROW_SEMANTIC_ERROR(
    "From scenario, a lane change was requested to Ego type entity ", std::quoted(name),
    " In general, such a request is an error, since Ego cars make autonomous decisions about "
    "everything but their destination");
}

auto EgoEntity::requestSpeedChange(
  const double target_speed, const speed_change::Transition, const speed_change::Constraint,
  const bool) -> void
{
  requestSpeedChange(target_speed, false);
}

auto EgoEntity::requestSpeedChange(
  const speed_change::RelativeTargetSpeed &, const speed_change::Transition,
  const speed_change::Constraint, const bool) -> void
{
  THROW_SEMANTIC_ERROR(
    "The traffic_simulator's request to set speed to the Ego type entity is for initialization "
    "purposes only.");
}

auto EgoEntity::requestClearRoute() -> void { clearRoute(); }

auto EgoEntity::requestReplanRoute(const std::vector<geometry_msgs::msg::PoseStamped> & route)
  -> void
{
  clearRoute();
  plan(route);
  enableAutowareControl();
  FieldOperatorApplication::engage();
}

auto EgoEntity::getDefaultDynamicConstraints() const
  -> const traffic_simulator_msgs::msg::DynamicConstraints &
{
  THROW_SEMANTIC_ERROR("getDefaultDynamicConstraints function does not support EgoEntity");
}

auto EgoEntity::setBehaviorParameter(
  const traffic_simulator_msgs::msg::BehaviorParameter & behavior_parameter) -> void
{
  behavior_parameter_ = behavior_parameter;
}

auto EgoEntity::requestSpeedChange(double value, bool /* continuous */) -> void
{
  if (status_->getTime() > 0.0) {
    THROW_SEMANTIC_ERROR("You cannot set target speed to the ego vehicle after starting scenario.");
  } else {
    target_speed_ = value;
  }
}

auto EgoEntity::requestSpeedChange(
  const speed_change::RelativeTargetSpeed & /*target_speed*/, bool /*continuous*/) -> void
{
  THROW_SEMANTIC_ERROR(
    "The traffic_simulator's request to set speed to the Ego type entity is for initialization "
    "purposes only.");
}

auto EgoEntity::setVelocityLimit(double value) -> void  //
{
  behavior_parameter_.dynamic_constraints.max_speed = value;
  FieldOperatorApplication::setVelocityLimit(value);
}

auto EgoEntity::setMapPose(const geometry_msgs::msg::Pose & map_pose) -> void
{
  auto entity_status = static_cast<EntityStatus>(*status_);
  entity_status.pose = map_pose;
  entity_status.lanelet_pose_valid = false;
  // prefer current lanelet on Autoware side
  status_->set(
    entity_status, helper::getUniqueValues(getRouteLanelets()),
    getDefaultMatchingDistanceForLaneletPoseCalculation());
}
}  // namespace entity
}  // namespace traffic_simulator
