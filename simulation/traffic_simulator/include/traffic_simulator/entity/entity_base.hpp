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

#ifndef TRAFFIC_SIMULATOR__ENTITY__ENTITY_BASE_HPP_
#define TRAFFIC_SIMULATOR__ENTITY__ENTITY_BASE_HPP_

#include <boost/optional.hpp>
#include <openscenario_msgs/msg/bounding_box.hpp>
#include <openscenario_msgs/msg/driver_model.hpp>
#include <openscenario_msgs/msg/entity_status.hpp>
#include <openscenario_msgs/msg/entity_type.hpp>
#include <openscenario_msgs/msg/obstacle.hpp>
#include <openscenario_msgs/msg/vehicle_parameters.hpp>
#include <openscenario_msgs/msg/waypoints_array.hpp>
#include <traffic_simulator/hdmap_utils/hdmap_utils.hpp>
#include <traffic_simulator/traffic_lights/traffic_light_manager.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// headers in STL
#include <memory>
#include <queue>
#include <string>
#include <unordered_map>
#include <vector>

namespace traffic_simulator
{
namespace entity
{
enum class Direction { STRAIGHT = 0, LEFT = 1, RIGHT = 2 };

class EntityBase
{
public:
  EntityBase(
    std::string type, std::string name, const openscenario_msgs::msg::EntityStatus & initial_state);
  EntityBase(std::string type, std::string name);
  virtual ~EntityBase() = default;
  const std::string type;
  const std::string name;
  const openscenario_msgs::msg::EntityStatus getStatus() const;
  bool setStatus(const openscenario_msgs::msg::EntityStatus & status);
  bool setVisibility(bool visibility);
  bool getVisibility();
  void setHdMapUtils(std::shared_ptr<hdmap_utils::HdMapUtils> ptr) { hdmap_utils_ptr_ = ptr; }
  void setTrafficLightManager(std::shared_ptr<traffic_simulator::TrafficLightManager> ptr)
  {
    traffic_light_manager_ = ptr;
  }
  void setDriverModel(const openscenario_msgs::msg::DriverModel &) { return; }
  virtual void onUpdate(double current_time, double step_time) = 0;
  bool statusSet() const
  {
    if (status_) {
      return true;
    }
    return false;
  }
  const boost::optional<openscenario_msgs::msg::VehicleParameters> getVehicleParameters() const
  {
    return boost::none;
  }
  void updateEntityStatusTimestamp(double current_time);
  void setVerbose(bool verbose) { verbose_ = verbose; }
  void setEntityTypeList(
    const std::unordered_map<std::string, openscenario_msgs::msg::EntityType> & entity_type_list)
  {
    entity_type_list_ = entity_type_list;
  }
  void setOtherStatus(
    const std::unordered_map<std::string, openscenario_msgs::msg::EntityStatus> & status);
  void updateStandStillDuration(double step_time);
  boost::optional<double> getStandStillDuration() const;
  virtual const openscenario_msgs::msg::BoundingBox getBoundingBox() const = 0;
  virtual const std::string getCurrentAction() const = 0;
  void stopAtEndOfRoad();
  boost::optional<double> getLinearJerk() const { return linear_jerk_; }
  virtual void requestAssignRoute(
    const std::vector<openscenario_msgs::msg::LaneletPose> & waypoints) = 0;
  void requestLaneChange(const std::int64_t) { return; };
  virtual void requestAcquirePosition(const openscenario_msgs::msg::LaneletPose & lanelet_pose) = 0;
  virtual void requestWalkStraight() = 0;
  const openscenario_msgs::msg::EntityType getEntityType() const { return entity_type_; }
  virtual std::vector<std::int64_t> getRouteLanelets(const double horizon = 100) = 0;
  virtual void setTargetSpeed(const double target_speed, const bool continuous) = 0;
  boost::optional<openscenario_msgs::msg::Obstacle> getObstacle() { return boost::none; }
  const openscenario_msgs::msg::WaypointsArray getWaypoints()
  {
    return openscenario_msgs::msg::WaypointsArray();
  }

protected:
  bool visibility_;
  boost::optional<openscenario_msgs::msg::LaneletPose> next_waypoint_;
  std::queue<openscenario_msgs::msg::LaneletPose> waypoints_;
  boost::optional<openscenario_msgs::msg::EntityStatus> status_;
  boost::optional<double> linear_jerk_;
  std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_utils_ptr_;
  std::shared_ptr<traffic_simulator::TrafficLightManager> traffic_light_manager_;
  bool verbose_;
  std::unordered_map<std::string, openscenario_msgs::msg::EntityStatus> other_status_;
  std::unordered_map<std::string, openscenario_msgs::msg::EntityType> entity_type_list_;
  boost::optional<double> stand_still_duration_;
  visualization_msgs::msg::MarkerArray current_marker_;
  openscenario_msgs::msg::EntityType entity_type_;
};
}  // namespace entity
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__ENTITY__ENTITY_BASE_HPP_
