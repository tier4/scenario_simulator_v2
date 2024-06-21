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
#include <concealer/autoware_universe.hpp>
#include <concealer/field_operator_application_for_autoware_universe.hpp>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <system_error>
#include <thread>
#include <traffic_simulator/entity/ego_entity.hpp>
#include <traffic_simulator_msgs/msg/waypoints_array.hpp>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

namespace traffic_simulator
{
namespace entity
{
auto EgoEntity::makeFieldOperatorApplication(
  const Configuration & configuration,
  const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & node_parameters)
  -> std::unique_ptr<concealer::FieldOperatorApplication>
{
  if (const auto architecture_type =
        getParameter<std::string>(node_parameters, "architecture_type", "awf/universe");
      architecture_type.find("awf/universe") != std::string::npos) {
    std::string rviz_config = getParameter<std::string>(node_parameters, "rviz_config", "");
    return getParameter<bool>(node_parameters, "launch_autoware", true)
             ? std::make_unique<
                 concealer::FieldOperatorApplicationFor<concealer::AutowareUniverse>>(
                 getParameter<std::string>(node_parameters, "autoware_launch_package"),
                 getParameter<std::string>(node_parameters, "autoware_launch_file"),
                 "map_path:=" + configuration.map_path.string(),
                 "lanelet2_map_file:=" + configuration.getLanelet2MapFile(),
                 "pointcloud_map_file:=" + configuration.getPointCloudMapFile(),
                 "sensor_model:=" + getParameter<std::string>(node_parameters, "sensor_model"),
                 "vehicle_model:=" + getParameter<std::string>(node_parameters, "vehicle_model"),
                 "rviz_config:=" + ((rviz_config == "")
                                      ? configuration.rviz_config_path.string()
                                      : Configuration::Pathname(rviz_config).string()),
                 "scenario_simulation:=true", "use_foa:=false",
                 "perception/enable_traffic_light:=" +
                   std::string((architecture_type >= "awf/universe/20230906") ? "true" : "false"))
             : std::make_unique<
                 concealer::FieldOperatorApplicationFor<concealer::AutowareUniverse>>();
  } else {
    throw common::SemanticError(
      "Unexpected architecture_type ", std::quoted(architecture_type), " was given.");
  }
}

EgoEntity::EgoEntity(
  const std::string & name, const CanonicalizedEntityStatus & entity_status,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr,
  const traffic_simulator_msgs::msg::VehicleParameters & parameters,
  const Configuration & configuration,
  const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & node_parameters)
: VehicleEntity(name, entity_status, hdmap_utils_ptr, parameters),
  field_operator_application(makeFieldOperatorApplication(configuration, node_parameters))
{
}

auto EgoEntity::asFieldOperatorApplication() const -> concealer::FieldOperatorApplication &
{
  assert(field_operator_application);
  return *field_operator_application;
}

auto EgoEntity::getCurrentAction() const -> std::string
{
  const auto state = field_operator_application->getAutowareStateName();
  return state.empty() ? "Launching" : state;
}

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

auto EgoEntity::getEntityType() const -> const traffic_simulator_msgs::msg::EntityType &
{
  static traffic_simulator_msgs::msg::EntityType type;
  type.type = traffic_simulator_msgs::msg::EntityType::EGO;
  return type;
}

auto EgoEntity::getObstacle() -> std::optional<traffic_simulator_msgs::msg::Obstacle>
{
  return std::nullopt;
}

auto EgoEntity::getRouteLanelets(double /*unused horizon*/) -> lanelet::Ids
{
  lanelet::Ids ids{};

  if (const auto universe =
        dynamic_cast<concealer::FieldOperatorApplicationFor<concealer::AutowareUniverse> *>(
          field_operator_application.get());
      universe) {
    for (const auto & point : universe->getPathWithLaneId().points) {
      ids += point.lane_ids;
    }
  }

  return ids;
}

auto EgoEntity::getCurrentPose() const -> geometry_msgs::msg::Pose { return status_.getMapPose(); }

auto EgoEntity::getWaypoints() -> const traffic_simulator_msgs::msg::WaypointsArray
{
  return field_operator_application->getWaypoints();
}

void EgoEntity::onUpdate(double current_time, double step_time)
{
  EntityBase::onUpdate(current_time, step_time);

  if (is_controlled_by_simulator_ && npc_logic_started_) {
    if (
      const auto updated_status = traffic_simulator::follow_trajectory::makeUpdatedStatus(
        static_cast<traffic_simulator::EntityStatus>(status_), *polyline_trajectory_,
        behavior_parameter_, hdmap_utils_ptr_, step_time,
        getDefaultMatchingDistanceForLaneletPoseCalculation(),
        target_speed_ ? target_speed_.value() : status_.getTwist().linear.x)) {
      setStatus(CanonicalizedEntityStatus(*updated_status, hdmap_utils_ptr_));
    } else {
      is_controlled_by_simulator_ = false;
    }
  }

  updateStandStillDuration(step_time);
  updateTraveledDistance(step_time);

  field_operator_application->rethrow();
  field_operator_application->spinSome();

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

  if (not field_operator_application->initialized()) {
    field_operator_application->initialize(getMapPose());
    field_operator_application->plan(route);
    // NOTE: engage() will be executed at simulation-time 0.
  } else {
    field_operator_application->plan(route);
    field_operator_application->engage();
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

void EgoEntity::requestLaneChange(const lanelet::Id)
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

void EgoEntity::requestClearRoute() { field_operator_application->clearRoute(); }

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

void EgoEntity::requestSpeedChange(double value, bool)
{
  target_speed_ = value;
  field_operator_application->restrictTargetSpeed(value);
}

void EgoEntity::requestSpeedChange(
  const speed_change::RelativeTargetSpeed & /*target_speed*/, bool /*continuous*/)
{
}

auto EgoEntity::setVelocityLimit(double value) -> void  //
{
  behavior_parameter_.dynamic_constraints.max_speed = value;
  field_operator_application->setVelocityLimit(value);
}

auto EgoEntity::fillLaneletPose(CanonicalizedEntityStatus & status) -> void
{
  EntityBase::fillLaneletPose(status, false);
}

auto EgoEntity::setMapPose(const geometry_msgs::msg::Pose & map_pose) -> void
{
  const auto unique_route_lanelets = traffic_simulator::helper::getUniqueValues(getRouteLanelets());
  std::optional<traffic_simulator_msgs::msg::LaneletPose> lanelet_pose;
  if (unique_route_lanelets.empty()) {
    lanelet_pose = hdmap_utils_ptr_->toLaneletPose(
      map_pose, getBoundingBox(), false, getDefaultMatchingDistanceForLaneletPoseCalculation());
  } else {
    lanelet_pose = hdmap_utils_ptr_->toLaneletPose(
      map_pose, unique_route_lanelets, getDefaultMatchingDistanceForLaneletPoseCalculation());
    if (!lanelet_pose) {
      lanelet_pose = hdmap_utils_ptr_->toLaneletPose(
        map_pose, getBoundingBox(), false, getDefaultMatchingDistanceForLaneletPoseCalculation());
    }
  }
  geometry_msgs::msg::Pose map_pose_z_fixed = map_pose;
  auto status = static_cast<EntityStatus>(status_);
  if (lanelet_pose) {
    math::geometry::CatmullRomSpline spline(
      hdmap_utils_ptr_->getCenterPoints(lanelet_pose->lanelet_id));
    if (const auto s_value = spline.getSValue(map_pose)) {
      map_pose_z_fixed.position.z = spline.getPoint(s_value.value()).z;
    }
    status.pose = map_pose_z_fixed;
    status.lanelet_pose_valid = true;
    status.lanelet_pose = lanelet_pose.value();
  } else {
    status.pose = map_pose;
    status.lanelet_pose_valid = false;
    status.lanelet_pose = LaneletPose();
  }
  status_ = CanonicalizedEntityStatus(status, hdmap_utils_ptr_);
}
}  // namespace entity
}  // namespace traffic_simulator
