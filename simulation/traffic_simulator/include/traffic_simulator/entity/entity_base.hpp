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

#include <autoware_vehicle_msgs/msg/vehicle_command.hpp>
#include <boost/optional.hpp>
#include <memory>
#include <openscenario_msgs/msg/bounding_box.hpp>
#include <openscenario_msgs/msg/driver_model.hpp>
#include <openscenario_msgs/msg/entity_status.hpp>
#include <openscenario_msgs/msg/entity_type.hpp>
#include <openscenario_msgs/msg/obstacle.hpp>
#include <openscenario_msgs/msg/vehicle_parameters.hpp>
#include <openscenario_msgs/msg/waypoints_array.hpp>
#include <queue>
#include <string>
#include <traffic_simulator/hdmap_utils/hdmap_utils.hpp>
#include <traffic_simulator/traffic_lights/traffic_light_manager.hpp>
#include <unordered_map>
#include <vector>
#include <visualization_msgs/msg/marker_array.hpp>

namespace traffic_simulator
{
namespace entity
{
enum class Direction { STRAIGHT = 0, LEFT = 1, RIGHT = 2 };

class EntityBase
{
public:
  const std::string type;

  const std::string name;

  EntityBase(const std::string & type, const std::string & name);

  EntityBase(
    const std::string & type, const std::string & name,
    const openscenario_msgs::msg::EntityStatus & initial_state);

  virtual ~EntityBase() = default;

public:
  virtual void engage() {}

  virtual auto getBoundingBox() const -> const openscenario_msgs::msg::BoundingBox = 0;

  virtual auto getCurrentAction() const -> const std::string = 0;

  /*   */ auto getEntityStatusBeforeUpdate() const
    -> const boost::optional<openscenario_msgs::msg::EntityStatus>
  {
    return status_before_update_;
  }

  /*   */ auto getEntityType() const -> const auto & { return entity_type_; }

  virtual auto getEntityTypename() const -> const std::string & = 0;

  /*   */ auto getLinearJerk() const { return linear_jerk_; }

  virtual auto getObstacle() -> boost::optional<openscenario_msgs::msg::Obstacle> = 0;

  virtual auto getRouteLanelets(const double horizon = 100) -> std::vector<std::int64_t> = 0;

  /*   */ auto getStatus() const -> const openscenario_msgs::msg::EntityStatus;

  /*   */ auto getStandStillDuration() const -> boost::optional<double>;

  /*   */ auto getVisibility() { return visibility_; }

  virtual auto getVehicleCommand() -> const autoware_vehicle_msgs::msg::VehicleCommand;

  /*   */ auto getVehicleParameters() const
    -> const boost::optional<openscenario_msgs::msg::VehicleParameters>
  {
    return boost::none;
  }

  virtual auto getWaypoints() -> const openscenario_msgs::msg::WaypointsArray = 0;

  virtual void setDriverModel(const openscenario_msgs::msg::DriverModel &) {}

  /*   */ void setEntityTypeList(
    const std::unordered_map<std::string, openscenario_msgs::msg::EntityType> & entity_type_list)
  {
    entity_type_list_ = entity_type_list;
  }

  virtual void setHdMapUtils(const std::shared_ptr<hdmap_utils::HdMapUtils> & ptr)
  {
    hdmap_utils_ptr_ = ptr;
  }

  /*   */ void setOtherStatus(
    const std::unordered_map<std::string, openscenario_msgs::msg::EntityStatus> & status);

  virtual auto setStatus(const openscenario_msgs::msg::EntityStatus & status) -> bool;

  virtual void setTargetSpeed(double target_speed, bool continuous) = 0;

  virtual void setTrafficLightManager(
    const std::shared_ptr<traffic_simulator::TrafficLightManager> & ptr)
  {
    traffic_light_manager_ = ptr;
  }

  /*   */ void setVerbose(bool verbose) { verbose_ = verbose; }

  /*   */ auto setVisibility(const bool visibility) { return visibility_ = visibility; }

  virtual void onUpdate(double current_time, double step_time);

  virtual auto ready() const -> bool { return static_cast<bool>(status_); }

  virtual void requestAcquirePosition(const openscenario_msgs::msg::LaneletPose & lanelet_pose) = 0;

  virtual void requestAssignRoute(
    const std::vector<openscenario_msgs::msg::LaneletPose> & waypoints) = 0;

  virtual void requestLaneChange(const std::int64_t){};

  virtual void requestWalkStraight()
  {
    THROW_SEMANTIC_ERROR(getEntityTypename(), " type entities do not support WalkStraightAction");
  }

  /*   */ auto statusSet() const noexcept { return static_cast<bool>(status_); }

  /*   */ void stopAtEndOfRoad();

  /*   */ void updateEntityStatusTimestamp(const double current_time);

  /*   */ void updateStandStillDuration(const double step_time);

protected:
  boost::optional<openscenario_msgs::msg::LaneletPose> next_waypoint_;
  boost::optional<openscenario_msgs::msg::EntityStatus> status_;
  boost::optional<openscenario_msgs::msg::EntityStatus> status_before_update_;

  std::queue<openscenario_msgs::msg::LaneletPose> waypoints_;

  std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_utils_ptr_;
  std::shared_ptr<traffic_simulator::TrafficLightManager> traffic_light_manager_;

  bool verbose_;
  bool visibility_;

  std::unordered_map<std::string, openscenario_msgs::msg::EntityStatus> other_status_;
  std::unordered_map<std::string, openscenario_msgs::msg::EntityType> entity_type_list_;

  boost::optional<double> linear_jerk_;
  boost::optional<double> stand_still_duration_;

  visualization_msgs::msg::MarkerArray current_marker_;
  openscenario_msgs::msg::EntityType entity_type_;
};
}  // namespace entity
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__ENTITY__ENTITY_BASE_HPP_
