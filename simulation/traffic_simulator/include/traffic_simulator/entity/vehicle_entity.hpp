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

#include <boost/optional.hpp>
#include <memory>
#include <pluginlib/class_loader.hpp>
#include <pugixml.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <traffic_simulator/behavior/behavior_plugin_base.hpp>
#include <traffic_simulator/behavior/route_planner.hpp>
#include <traffic_simulator/entity/entity_base.hpp>
#include <traffic_simulator_msgs/msg/driver_model.hpp>
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
      static const std::string name = "TODO";
      return name;
    }

    static auto defaultBehavior() -> const std::string & { return behaviorTree(); }
  };

  template <typename Pose>
  explicit VehicleEntity(
    const std::string & name, const Pose & pose,
    const traffic_simulator_msgs::msg::VehicleParameters & parameters,
    const std::string & plugin_name = BuiltinBehavior::defaultBehavior())
  : EntityBase(name, pose, parameters.subtype),
    parameters(parameters),
    loader_(pluginlib::ClassLoader<entity_behavior::BehaviorPluginBase>(
      "traffic_simulator", "entity_behavior::BehaviorPluginBase")),
    behavior_plugin_ptr_(loader_.createSharedInstance(plugin_name))
  {
    behavior_plugin_ptr_->configure(rclcpp::get_logger(name));
    behavior_plugin_ptr_->setVehicleParameters(parameters);
    behavior_plugin_ptr_->setDebugMarker({});
    behavior_plugin_ptr_->setDriverModel(traffic_simulator_msgs::msg::DriverModel());
  }

  ~VehicleEntity() override = default;

  void appendDebugMarker(visualization_msgs::msg::MarkerArray & marker_array) override;

  void cancelRequest() override;

  auto getBoundingBox() const -> const traffic_simulator_msgs::msg::BoundingBox override;

  auto getCurrentAction() const -> std::string override;

  auto getDriverModel() const -> traffic_simulator_msgs::msg::DriverModel override;

  auto getEntityType() const -> const traffic_simulator_msgs::msg::EntityType & override;

  auto getEntityTypename() const -> const std::string & override;

  auto getGoalPoses() -> std::vector<traffic_simulator_msgs::msg::LaneletPose> override;

  auto getObstacle() -> boost::optional<traffic_simulator_msgs::msg::Obstacle> override;

  auto getRouteLanelets(double horizon = 100) -> std::vector<std::int64_t> override;

  auto getVehicleParameters() const
    -> boost::optional<traffic_simulator_msgs::msg::VehicleParameters>;

  auto getWaypoints() -> const traffic_simulator_msgs::msg::WaypointsArray override;

  void onUpdate(double current_time, double step_time) override;

  void requestAcquirePosition(const traffic_simulator_msgs::msg::LaneletPose &);

  void requestAcquirePosition(const geometry_msgs::msg::Pose & map_pose) override;

  void requestAssignRoute(const std::vector<geometry_msgs::msg::Pose> &) override;

  void requestAssignRoute(const std::vector<traffic_simulator_msgs::msg::LaneletPose> &) override;

  void requestLaneChange(const std::int64_t to_lanelet_id) override;

  void requestLaneChange(const traffic_simulator::lane_change::Parameter &) override;

  void setAccelerationLimit(double acceleration) override;

  void setDecelerationLimit(double deceleration) override;

  void setDriverModel(const traffic_simulator_msgs::msg::DriverModel &) override;

  void setHdMapUtils(const std::shared_ptr<hdmap_utils::HdMapUtils> &) override;

  void setTrafficLightManager(
    const std::shared_ptr<traffic_simulator::TrafficLightManagerBase> &) override;

  const traffic_simulator_msgs::msg::VehicleParameters parameters;

private:
  pluginlib::ClassLoader<entity_behavior::BehaviorPluginBase> loader_;

  const std::shared_ptr<entity_behavior::BehaviorPluginBase> behavior_plugin_ptr_;

  std::shared_ptr<traffic_simulator::RoutePlanner> route_planner_ptr_;

  std::shared_ptr<math::geometry::CatmullRomSpline> spline_;

  std::vector<std::int64_t> previous_route_lanelets_;
};
}  // namespace entity
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__ENTITY__VEHICLE_ENTITY_HPP_
