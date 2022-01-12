// Copyright 2015-2020 Tier IV, Inc. All rights reserved.
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

#ifndef TRAFFIC_SIMULATOR__BEHAVIOR__BEHAVIOR_PLUGIN_BASE_HPP_
#define TRAFFIC_SIMULATOR__BEHAVIOR__BEHAVIOR_PLUGIN_BASE_HPP_

#include <boost/optional.hpp>
#include <string>
#include <traffic_simulator/data_type/data_types.hpp>
#include <traffic_simulator/hdmap_utils/hdmap_utils.hpp>
#include <traffic_simulator/traffic_lights/traffic_light_manager.hpp>
#include <traffic_simulator_msgs/msg/driver_model.hpp>
#include <traffic_simulator_msgs/msg/entity_status.hpp>
#include <traffic_simulator_msgs/msg/entity_type.hpp>
#include <traffic_simulator_msgs/msg/obstacle.hpp>
#include <traffic_simulator_msgs/msg/pedestrian_parameters.hpp>
#include <traffic_simulator_msgs/msg/vehicle_parameters.hpp>
#include <traffic_simulator_msgs/msg/waypoints_array.hpp>
#include <unordered_map>
#include <visualization_msgs/msg/marker_array.hpp>

namespace entity_behavior
{
class BehaviorPluginBase
{
public:
  virtual ~BehaviorPluginBase() = default;
  virtual void configure(const rclcpp::Logger & logger) = 0;
  virtual void update(double current_time, double step_time) = 0;
  virtual const std::string & getCurrentAction() const = 0;

  typedef std::unordered_map<std::string, traffic_simulator_msgs::msg::EntityType> EntityTypeDict;
  typedef std::unordered_map<std::string, traffic_simulator_msgs::msg::EntityStatus>
    EntityStatusDict;

#define DEFINE_GETTER_SETTER(NAME, KEY, TYPE)     \
  virtual TYPE get##NAME() = 0;                   \
  virtual void set##NAME(const TYPE & value) = 0; \
  const std::string get##NAME##Key() const { return KEY; };

  // clang-format off
  DEFINE_GETTER_SETTER(CurrentTime, "current_time", double)
  DEFINE_GETTER_SETTER(DebugMarker, "debug_marker", std::vector<visualization_msgs::msg::Marker>)
  DEFINE_GETTER_SETTER(DriverModel, "driver_model", traffic_simulator_msgs::msg::DriverModel)
  DEFINE_GETTER_SETTER(EntityStatus, "entity_status", traffic_simulator_msgs::msg::EntityStatus)
  DEFINE_GETTER_SETTER(EntityTypeList, "entity_type_list", EntityTypeDict)
  DEFINE_GETTER_SETTER(GoalPoses, "goal_poses", std::vector<geometry_msgs::msg::Pose>)
  DEFINE_GETTER_SETTER(HdMapUtils, "hdmap_utils", std::shared_ptr<hdmap_utils::HdMapUtils>)
  DEFINE_GETTER_SETTER(Obstacle, "obstacle", boost::optional<traffic_simulator_msgs::msg::Obstacle>)
  DEFINE_GETTER_SETTER(OtherEntityStatus, "other_entity_status", EntityStatusDict)
  DEFINE_GETTER_SETTER(PedestrianParameters, "pedestrian_parameters", traffic_simulator_msgs::msg::PedestrianParameters)
  DEFINE_GETTER_SETTER(Request, "request", std::string)
  DEFINE_GETTER_SETTER(RouteLanelets, "route_lanelets", std::vector<std::int64_t>)
  DEFINE_GETTER_SETTER(StepTime, "step_time", double)
  DEFINE_GETTER_SETTER(TargetSpeed, "target_speed", boost::optional<double>)
  DEFINE_GETTER_SETTER(LaneChangeParameters, "lane_change_parameters", traffic_simulator::lane_change::Parameter)
  DEFINE_GETTER_SETTER(TrafficLightManager, "traffic_light_manager", std::shared_ptr<traffic_simulator::TrafficLightManagerBase>)
  DEFINE_GETTER_SETTER(UpdatedStatus, "updated_status", traffic_simulator_msgs::msg::EntityStatus)
  DEFINE_GETTER_SETTER(VehicleParameters, "vehicle_parameters", traffic_simulator_msgs::msg::VehicleParameters)
  DEFINE_GETTER_SETTER(Waypoints, "waypoints", traffic_simulator_msgs::msg::WaypointsArray)
  // clang-format on
#undef DEFINE_GETTER_SETTER
};
}  // namespace entity_behavior

#endif  // TRAFFIC_SIMULATOR__BEHAVIOR__BEHAVIOR_PLUGIN_BASE_HPP_
