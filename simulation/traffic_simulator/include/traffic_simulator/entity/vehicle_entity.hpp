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

#ifndef TRAFFIC_SIMULATOR__ENTITY__VEHICLE_ENTITY_HPP_
#define TRAFFIC_SIMULATOR__ENTITY__VEHICLE_ENTITY_HPP_

#include <memory>
#include <optional>
#include <pluginlib/class_loader.hpp>
#include <pugixml.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <traffic_simulator/behavior/behavior_plugin_base.hpp>
#include <traffic_simulator/behavior/route_planner.hpp>
#include <traffic_simulator/entity/entity_base.hpp>
#include <traffic_simulator_msgs/msg/behavior_parameter.hpp>
#include <traffic_simulator_msgs/msg/vehicle_parameters.hpp>
#include <traffic_simulator_msgs/msg/waypoints_array.hpp>
#include <vector>

namespace traffic_simulator
{
namespace entity
{
class VehicleEntity : public EntityBase
{
public:
  struct BuiltinBehavior
  {
    static auto behaviorTree() -> const std::string &
    {
      static const std::string name = "behavior_tree_plugin/VehicleBehaviorTree";
      return name;
    }

    static auto contextGamma() -> const std::string &
    {
      static const std::string name = "context_gamma_planner/VehiclePlugin";
      return name;
    }

    static auto doNothing() noexcept -> const std::string &
    {
      static const std::string name = "do_nothing_plugin/DoNothingPlugin";
      return name;
    }

    static auto defaultBehavior() -> const std::string & { return behaviorTree(); }
  };

  explicit VehicleEntity(
    const std::string & name, const CanonicalizedEntityStatus &,
    const std::shared_ptr<hdmap_utils::HdMapUtils> &,
    const traffic_simulator_msgs::msg::VehicleParameters &,
    const std::string & plugin_name = BuiltinBehavior::defaultBehavior());

  ~VehicleEntity() override = default;

  void appendDebugMarker(visualization_msgs::msg::MarkerArray & marker_array) override;

  void cancelRequest() override;

  auto getCurrentAction() const -> std::string override;

  auto getDefaultDynamicConstraints() const
    -> const traffic_simulator_msgs::msg::DynamicConstraints & override;

  auto getDefaultMatchingDistanceForLaneletPoseCalculation() const -> double override;

  auto getBehaviorParameter() const -> traffic_simulator_msgs::msg::BehaviorParameter override;

  auto getMaxAcceleration() const -> double override;

  auto getMaxDeceleration() const -> double override;

  auto getEntityTypename() const -> const std::string & override;

  auto getGoalPoses() -> std::vector<CanonicalizedLaneletPose> override;

  auto getObstacle() -> std::optional<traffic_simulator_msgs::msg::Obstacle> override;

  auto getRouteLanelets(double horizon = 100) -> lanelet::Ids override;

  auto getWaypoints() -> const traffic_simulator_msgs::msg::WaypointsArray override;

  auto onUpdate(const double current_time, const double step_time) -> void override;

  void requestAcquirePosition(const CanonicalizedLaneletPose &);

  void requestAcquirePosition(const geometry_msgs::msg::Pose & map_pose) override;

  void requestAssignRoute(const std::vector<geometry_msgs::msg::Pose> &) override;

  void requestAssignRoute(const std::vector<CanonicalizedLaneletPose> &) override;

  auto requestFollowTrajectory(
    const std::shared_ptr<traffic_simulator_msgs::msg::PolylineTrajectory> &) -> void override;

  void requestLaneChange(const lanelet::Id to_lanelet_id) override;

  void requestLaneChange(const traffic_simulator::lane_change::Parameter &) override;

  void setVelocityLimit(double linear_velocity) override;

  void setAccelerationLimit(double acceleration) override;

  void setAccelerationRateLimit(double acceleration_rate) override;

  void setDecelerationLimit(double deceleration) override;

  void setDecelerationRateLimit(double deceleration_rate) override;

  void setBehaviorParameter(const traffic_simulator_msgs::msg::BehaviorParameter &) override;

  void setTrafficLightManager(
    const std::shared_ptr<traffic_simulator::TrafficLightManager> &) override;

  const traffic_simulator_msgs::msg::VehicleParameters vehicle_parameters;

private:
  pluginlib::ClassLoader<entity_behavior::BehaviorPluginBase> loader_;

  const std::shared_ptr<entity_behavior::BehaviorPluginBase> behavior_plugin_ptr_;

  traffic_simulator::RoutePlanner route_planner_;

  std::shared_ptr<math::geometry::CatmullRomSpline> spline_;

  lanelet::Ids previous_route_lanelets_;
};
}  // namespace entity
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__ENTITY__VEHICLE_ENTITY_HPP_
