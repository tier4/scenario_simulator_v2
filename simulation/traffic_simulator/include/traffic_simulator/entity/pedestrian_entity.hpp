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

#ifndef TRAFFIC_SIMULATOR__ENTITY__PEDESTRIAN_ENTITY_HPP_
#define TRAFFIC_SIMULATOR__ENTITY__PEDESTRIAN_ENTITY_HPP_

#include <memory>
#include <optional>
#include <pluginlib/class_loader.hpp>
#include <pugixml.hpp>
#include <string>
#include <traffic_simulator/behavior/behavior_plugin_base.hpp>
#include <traffic_simulator/behavior/route_planner.hpp>
#include <traffic_simulator/entity/entity_base.hpp>
#include <traffic_simulator_msgs/msg/pedestrian_parameters.hpp>
#include <vector>

namespace traffic_simulator
{
namespace entity
{
class PedestrianEntity : public EntityBase
{
public:
  struct BuiltinBehavior
  {
    static auto behaviorTree() noexcept -> const std::string &
    {
      static const std::string name = "behavior_tree_plugin/PedestrianBehaviorTree";
      return name;
    }

    static auto contextGamma() noexcept -> const std::string &
    {
      static const std::string name = "context_gamma_planner/PedestrianPlugin";
      return name;
    }

    static auto doNothing() noexcept -> const std::string &
    {
      static const std::string name = "do_nothing_plugin/DoNothingPlugin";
      return name;
    }

    static auto defaultBehavior() noexcept -> const std::string & { return behaviorTree(); }
  };

  explicit PedestrianEntity(
    const std::string & name, const CanonicalizedEntityStatus &,
    const std::shared_ptr<hdmap_utils::HdMapUtils> &,
    const traffic_simulator_msgs::msg::PedestrianParameters &,
    const std::string & plugin_name = BuiltinBehavior::defaultBehavior());

  ~PedestrianEntity() override = default;

  void appendDebugMarker(visualization_msgs::msg::MarkerArray & marker_array) override;

  auto getEntityTypename() const -> const std::string & override;

  auto onUpdate(const double current_time, const double step_time) -> void override;

  void requestAcquirePosition(const CanonicalizedLaneletPose & lanelet_pose) override;

  void requestAcquirePosition(const geometry_msgs::msg::Pose & map_pose) override;

  void requestWalkStraight() override;

  void cancelRequest() override;

  void setTrafficLightManager(
    const std::shared_ptr<traffic_simulator::TrafficLightManager> & ptr) override;

  auto getBehaviorParameter() const -> traffic_simulator_msgs::msg::BehaviorParameter;

  auto getMaxAcceleration() const -> double override;

  auto getMaxDeceleration() const -> double override;

  void setBehaviorParameter(const traffic_simulator_msgs::msg::BehaviorParameter &);

  void setVelocityLimit(double linear_velocity) override;

  void setAccelerationLimit(double acceleration) override;

  void setAccelerationRateLimit(double acceleration_rate) override;

  void setDecelerationLimit(double deceleration) override;

  void setDecelerationRateLimit(double deceleration) override;

  void requestAssignRoute(const std::vector<CanonicalizedLaneletPose> & waypoints) override;

  void requestAssignRoute(const std::vector<geometry_msgs::msg::Pose> &) override;

  auto requestFollowTrajectory(
    const std::shared_ptr<traffic_simulator_msgs::msg::PolylineTrajectory> &) -> void override;

  std::string getCurrentAction() const override;

  auto getDefaultDynamicConstraints() const
    -> const traffic_simulator_msgs::msg::DynamicConstraints & override;

  auto getRouteLanelets(double horizon = 100) -> lanelet::Ids override;

  auto getObstacle() -> std::optional<traffic_simulator_msgs::msg::Obstacle> override;

  auto getGoalPoses() -> std::vector<CanonicalizedLaneletPose> override;

  auto getWaypoints() -> const traffic_simulator_msgs::msg::WaypointsArray override;

  const std::string plugin_name;

  const traffic_simulator_msgs::msg::PedestrianParameters pedestrian_parameters;

private:
  pluginlib::ClassLoader<entity_behavior::BehaviorPluginBase> loader_;
  const std::shared_ptr<entity_behavior::BehaviorPluginBase> behavior_plugin_ptr_;
  traffic_simulator::RoutePlanner route_planner_;
};
}  // namespace entity
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__ENTITY__PEDESTRIAN_ENTITY_HPP_
