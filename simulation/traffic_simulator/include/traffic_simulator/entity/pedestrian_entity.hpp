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

#include <boost/optional.hpp>
#include <memory>
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

    static auto defaultBehavior() noexcept -> const std::string & { return behaviorTree(); }
  };

  explicit PedestrianEntity(
    const std::string & name, const CanonicalizedEntityStatusType &,
    const std::shared_ptr<hdmap_utils::HdMapUtils> &,
    const traffic_simulator_msgs::msg::PedestrianParameters &,
    const std::string & plugin_name = BuiltinBehavior::defaultBehavior());

  ~PedestrianEntity() override = default;

  void appendDebugMarker(visualization_msgs::msg::MarkerArray & marker_array) override;

  auto getEntityType() const -> const traffic_simulator_msgs::msg::EntityType & override;

  auto getEntityTypename() const -> const std::string & override;

  void onUpdate(double current_time, double step_time) override;

  void requestAcquirePosition(const CanonicalizedLaneletPoseType & lanelet_pose) override;

  void requestAcquirePosition(const geometry_msgs::msg::Pose & map_pose) override;

  void requestWalkStraight() override;

  void cancelRequest() override;

  void setTrafficLightManager(
    const std::shared_ptr<traffic_simulator::TrafficLightManagerBase> & ptr) override;

  auto getBehaviorParameter() const -> traffic_simulator_msgs::msg::BehaviorParameter;

  void setBehaviorParameter(const traffic_simulator_msgs::msg::BehaviorParameter &);

  void setAccelerationLimit(double acceleration) override;

  void setAccelerationRateLimit(double acceleration_rate) override;

  void setDecelerationLimit(double deceleration) override;

  void setDecelerationRateLimit(double deceleration) override;

  void requestAssignRoute(const std::vector<CanonicalizedLaneletPoseType> & waypoints) override;

  void requestAssignRoute(const std::vector<geometry_msgs::msg::Pose> &) override;

  std::string getCurrentAction() const override;

  auto getDefaultDynamicConstraints() const
    -> const traffic_simulator_msgs::msg::DynamicConstraints & override;

  auto getRouteLanelets(double horizon = 100) -> std::vector<std::int64_t> override;

  auto getObstacle() -> boost::optional<traffic_simulator_msgs::msg::Obstacle> override;

  auto getGoalPoses() -> std::vector<CanonicalizedLaneletPoseType> override;

  auto getWaypoints() -> const traffic_simulator_msgs::msg::WaypointsArray override;

  const std::string plugin_name;

private:
  pluginlib::ClassLoader<entity_behavior::BehaviorPluginBase> loader_;
  const std::shared_ptr<entity_behavior::BehaviorPluginBase> behavior_plugin_ptr_;
  traffic_simulator::RoutePlanner route_planner_;
};
}  // namespace entity
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__ENTITY__PEDESTRIAN_ENTITY_HPP_
