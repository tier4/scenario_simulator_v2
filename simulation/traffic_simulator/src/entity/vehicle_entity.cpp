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

#include <boost/algorithm/clamp.hpp>
#include <memory>
#include <string>
#include <traffic_simulator/entity/vehicle_entity.hpp>
#include <traffic_simulator_msgs/msg/vehicle_parameters.hpp>
#include <vector>

namespace traffic_simulator
{
namespace entity
{
VehicleEntity::VehicleEntity(
  const std::string & name, const traffic_simulator_msgs::msg::VehicleParameters & params,
  const std::string & plugin_name)
: EntityBase(name, params.subtype),
  parameters(params),
  plugin_name(plugin_name),
  loader_(pluginlib::ClassLoader<entity_behavior::BehaviorPluginBase>(
    "traffic_simulator", "entity_behavior::BehaviorPluginBase")),
  behavior_plugin_ptr_(loader_.createSharedInstance(plugin_name))
{
  entity_type_.type = traffic_simulator_msgs::msg::EntityType::VEHICLE;
  behavior_plugin_ptr_->configure(rclcpp::get_logger(name));
  behavior_plugin_ptr_->setVehicleParameters(parameters);
  behavior_plugin_ptr_->setDebugMarker({});
  behavior_plugin_ptr_->setDriverModel(traffic_simulator_msgs::msg::DriverModel());
}

void VehicleEntity::appendDebugMarker(visualization_msgs::msg::MarkerArray & marker_array)
{
  const auto marker = behavior_plugin_ptr_->getDebugMarker();
  std::copy(marker.begin(), marker.end(), std::back_inserter(marker_array.markers));
}

void VehicleEntity::requestAssignRoute(
  const std::vector<traffic_simulator_msgs::msg::LaneletPose> & waypoints)
{
  if (status_ and status_->lanelet_pose_valid) {
    route_planner_ptr_->getRouteLanelets(status_->lanelet_pose, waypoints);
  }
}

void VehicleEntity::requestAssignRoute(const std::vector<geometry_msgs::msg::Pose> & waypoints)
{
  std::vector<traffic_simulator_msgs::msg::LaneletPose> route;
  for (const auto & waypoint : waypoints) {
    const auto lanelet_waypoint =
      hdmap_utils_ptr_->toLaneletPose(waypoint, getBoundingBox(), false);
    if (lanelet_waypoint) {
      route.emplace_back(lanelet_waypoint.get());
    } else {
      THROW_SEMANTIC_ERROR("Waypoint of pedestrian entity should be on lane.");
    }
  }
  requestAssignRoute(route);
}

void VehicleEntity::requestAcquirePosition(
  const traffic_simulator_msgs::msg::LaneletPose & lanelet_pose)
{
  if (status_ and status_->lanelet_pose_valid) {
    route_planner_ptr_->getRouteLanelets(status_->lanelet_pose, lanelet_pose);
  }
}

void VehicleEntity::requestAcquirePosition(const geometry_msgs::msg::Pose & map_pose)
{
  const auto lanelet_pose = hdmap_utils_ptr_->toLaneletPose(map_pose, getBoundingBox(), false);
  if (lanelet_pose) {
    requestAcquirePosition(lanelet_pose.get());
  } else {
    THROW_SEMANTIC_ERROR("Goal of the vehicle entity should be on lane.");
  }
}

void VehicleEntity::requestLaneChange(const std::int64_t to_lanelet_id)
{
  behavior_plugin_ptr_->setRequest(behavior::Request::LANE_CHANGE);
  const auto parameter = lane_change::Parameter(
    lane_change::AbsoluteTarget(to_lanelet_id), lane_change::TrajectoryShape::CUBIC,
    lane_change::Constraint());
  behavior_plugin_ptr_->setLaneChangeParameters(parameter);
}

void VehicleEntity::requestLaneChange(const traffic_simulator::lane_change::Parameter & parameter)
{
  behavior_plugin_ptr_->setRequest(behavior::Request::LANE_CHANGE);
  behavior_plugin_ptr_->setLaneChangeParameters(parameter);
}

void VehicleEntity::cancelRequest()
{
  behavior_plugin_ptr_->setRequest(behavior::Request::NONE);
  route_planner_ptr_->cancelGoal();
}

auto VehicleEntity::getDriverModel() const -> traffic_simulator_msgs::msg::DriverModel
{
  return behavior_plugin_ptr_->getDriverModel();
}

void VehicleEntity::onUpdate(double current_time, double step_time)
{
  EntityBase::onUpdate(current_time, step_time);
  if (!status_) {
    return;
  }
  if (current_time_ < 0) {
    updateEntityStatusTimestamp(current_time_);
  } else {
    behavior_plugin_ptr_->setOtherEntityStatus(other_status_);
    behavior_plugin_ptr_->setEntityTypeList(entity_type_list_);
    behavior_plugin_ptr_->setEntityStatus(status_.get());
    behavior_plugin_ptr_->setTargetSpeed(target_speed_);

    std::vector<std::int64_t> route_lanelets = {};
    if (status_->lanelet_pose_valid) {
      route_lanelets = route_planner_ptr_->getRouteLanelets(status_->lanelet_pose);
    }
    behavior_plugin_ptr_->setRouteLanelets(route_lanelets);

    // recalculate spline only when input data changes
    if (previous_route_lanelets_ != route_lanelets) {
      previous_route_lanelets_ = route_lanelets;
      try {
        spline_ = std::make_shared<math::geometry::CatmullRomSpline>(
          hdmap_utils_ptr_->getCenterPoints(route_lanelets));
      } catch (const common::scenario_simulator_exception::SemanticError & error) {
        // reset the ptr when spline cannot be calculated
        spline_.reset();
      }
    }
    behavior_plugin_ptr_->setReferenceTrajectory(spline_);

    behavior_plugin_ptr_->update(current_time_, step_time_);
    auto status_updated = behavior_plugin_ptr_->getUpdatedStatus();
    if (status_updated.lanelet_pose_valid) {
      auto following_lanelets =
        hdmap_utils_ptr_->getFollowingLanelets(status_updated.lanelet_pose.lanelet_id);
      auto l = hdmap_utils_ptr_->getLaneletLength(status_updated.lanelet_pose.lanelet_id);
      if (following_lanelets.size() == 1 && l <= status_updated.lanelet_pose.s) {
        stopAtEndOfRoad();
        return;
      }
    }
    if (!status_) {
      linear_jerk_ = 0;
    } else {
      linear_jerk_ =
        (status_updated.action_status.accel.linear.x - status_->action_status.accel.linear.x) /
        step_time_;
    }
    setStatus(status_updated);
  }
}

void VehicleEntity::setAccelerationLimit(double acceleration)
{
  if (acceleration <= 0.0) {
    THROW_SEMANTIC_ERROR("Acceleration limit should be over zero.");
  }
  auto driver_model = getDriverModel();
  driver_model.acceleration = acceleration;
  setDriverModel(driver_model);
}

void VehicleEntity::setDecelerationLimit(double deceleration)
{
  if (deceleration <= 0.0) {
    THROW_SEMANTIC_ERROR("Deceleration limit should be over zero.");
  }
  auto driver_model = getDriverModel();
  driver_model.deceleration = deceleration;
  setDriverModel(driver_model);
}
}  // namespace entity
}  // namespace traffic_simulator
