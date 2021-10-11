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
#include <openscenario_msgs/msg/driver_model.hpp>
#include <openscenario_msgs/msg/entity_status.hpp>
#include <openscenario_msgs/msg/entity_type.hpp>
#include <openscenario_msgs/msg/obstacle.hpp>
#include <openscenario_msgs/msg/pedestrian_parameters.hpp>
#include <openscenario_msgs/msg/vehicle_parameters.hpp>
#include <openscenario_msgs/msg/waypoints_array.hpp>
#include <string>
#include <traffic_simulator/behavior/black_board.hpp>
#include <traffic_simulator/hdmap_utils/hdmap_utils.hpp>
#include <traffic_simulator/traffic_lights/traffic_light_manager.hpp>
#include <unordered_map>

namespace entity_behavior
{
class BehaviorPluginBase
{
private:
  BlackBoard black_board_;
  std::string current_action_;

public:
  virtual void update(double current_time, double step_time) = 0;
  const std::string getCurrentAction() const { return current_action_; }

  typedef std::unordered_map<std::string, openscenario_msgs::msg::EntityType> EntityTypeDict;
  typedef std::unordered_map<std::string, openscenario_msgs::msg::EntityStatus> EntityStatusDict;
#define DEFINE_GETTER(GETTER, KEY, TYPE) \
  TYPE GETTER() const                    \
  {                                      \
    TYPE value;                          \
    black_board_.get(KEY, value);        \
    return value;                        \
  }
  DEFINE_GETTER(getOtherEntityStatus, "other_entity_status", EntityStatusDict)
  DEFINE_GETTER(getToLaneletId, "to_lanelet_id", std::int64_t)
  DEFINE_GETTER(getEntityStatus, "entity_status", openscenario_msgs::msg::EntityStatus)
  DEFINE_GETTER(getTargetSpeed, "target_speed", boost::optional<double>)
  DEFINE_GETTER(getRouteLanelets, "route_lanelets", std::vector<std::int64_t>)
#undef DEFINE_GETTER

#define DEFINE_SETTER(SETTER, KEY, TYPE) \
  void SETTER(const TYPE & value) { black_board_.set(KEY, value); }
  DEFINE_SETTER(setOtherEntityStatus, "other_entity_status", EntityStatusDict)
  DEFINE_SETTER(setToLaneletId, "to_lanelet_id", std::int64_t)
  DEFINE_SETTER(setEntityStatus, "entity_status", openscenario_msgs::msg::EntityStatus)
  DEFINE_SETTER(setTargetSpeed, "target_speed", boost::optional<double>)
  DEFINE_SETTER(setRouteLanelets, "route_lanelets", std::vector<std::int64_t>)
#undef DEFINE_SETTER

#define DEFINE_GETTER_SETTER(GETTER, SETTER, KEY, TYPE)             \
  TYPE GETTER() const                                               \
  {                                                                 \
    TYPE value;                                                     \
    black_board_.get(KEY, value);                                   \
    return value;                                                   \
  }                                                                 \
  void SETTER(const TYPE & value) { black_board_.set(KEY, value); } \
  const std::string GETTER##Key() const { return KEY; }
  DEFINE_GETTER_SETTER(
    getWaypoints, setWaypoints, "waypoints", openscenario_msgs::msg::WaypointsArray)
  DEFINE_GETTER_SETTER(
    getObstacle, setObstacle, "obstacle", boost::optional<openscenario_msgs::msg::Obstacle>)
  DEFINE_GETTER_SETTER(
    getUpdatedStatus, setUpdatedStatus, "updated_status", openscenario_msgs::msg::EntityStatus)
  DEFINE_GETTER_SETTER(getRequest, setRequest, "request", std::string)
  DEFINE_GETTER_SETTER(
    getHdMapUtils, setHdMapUtils, "hdmap_utils", std::shared_ptr<hdmap_utils::HdMapUtils>)
  DEFINE_GETTER_SETTER(getEntityTypeList, setEntityTypeList, "entity_type_list", EntityTypeDict)
  DEFINE_GETTER_SETTER(
    getTrafficLightManager, setTrafficLightManager, "traffic_light_manager",
    std::shared_ptr<traffic_simulator::TrafficLightManager>)
  DEFINE_GETTER_SETTER(
    getPedestrianParameters, setPedestrianParameters, "pedestrian_parameters",
    openscenario_msgs::msg::PedestrianParameters)
  DEFINE_GETTER_SETTER(
    getDriverModel, setDriverModel, "driver_model", openscenario_msgs::msg::DriverModel)
  DEFINE_GETTER_SETTER(
    getVehicleParameters, setVehicleParameters, "vehicle_parameters",
    openscenario_msgs::msg::VehicleParameters)
#undef DEFINE_GETTER_SETTER
};
}  // namespace entity_behavior

#endif  // TRAFFIC_SIMULATOR__BEHAVIOR__BEHAVIOR_PLUGIN_BASE_HPP_
