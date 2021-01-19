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

#ifndef SIMULATION_API__ENTITY__EGO_ENTITY_HPP_
#define SIMULATION_API__ENTITY__EGO_ENTITY_HPP_

#include <simulation_api/entity/vehicle_entity.hpp>
#include <simulation_api/vehicle_model/sim_model.hpp>
#include <autoware_auto_msgs/msg/vehicle_control_command.hpp>
#include <autoware_auto_msgs/msg/vehicle_state_command.hpp>
#include <autoware_auto_msgs/msg/vehicle_kinematic_state.hpp>
#include <autoware_auto_msgs/msg/complex32.hpp>

// headers in pugixml
#include <pugixml.hpp>
#include <boost/optional.hpp>

#include <string>
#include <memory>

namespace simulation_api
{
namespace entity
{
class EgoEntity : public VehicleEntity
{
public:
  EgoEntity(
    std::string name, const openscenario_msgs::msg::EntityStatus & initial_state,
    const pugi::xml_node & xml);
  EgoEntity(
    std::string name, const openscenario_msgs::msg::EntityStatus & initial_state,
    VehicleParameters parameters);
  EgoEntity(std::string name, const pugi::xml_node & xml);
  EgoEntity(std::string name, VehicleParameters parameters);
  void setVehicleCommands(
    boost::optional<autoware_auto_msgs::msg::VehicleControlCommand> control_cmd,
    boost::optional<autoware_auto_msgs::msg::VehicleStateCommand> state_cmd);
  const std::string getCurrentAction() const
  {
    return "none";
  }
  void onUpdate(double current_time, double step_time) override;
  bool setStatus(const openscenario_msgs::msg::EntityStatus & status);
  boost::optional<autoware_auto_msgs::msg::VehicleKinematicState>
  getCurrentKinematicState() const
  {
    return current_kinematic_state_;
  }

private:
  boost::optional<geometry_msgs::msg::Pose> origin_;
  autoware_auto_msgs::msg::Complex32 toHeading(const double yaw);
  const openscenario_msgs::msg::EntityStatus getEntityStatus(double time) const;
  boost::optional<autoware_auto_msgs::msg::VehicleControlCommand> control_cmd_;
  boost::optional<autoware_auto_msgs::msg::VehicleStateCommand> state_cmd_;
  boost::optional<autoware_auto_msgs::msg::VehicleKinematicState> current_kinematic_state_;
  std::shared_ptr<SimModelInterface> vehicle_model_ptr_;
};
}      // namespace entity
}  // namespace simulation_api

#endif  // SIMULATION_API__ENTITY__EGO_ENTITY_HPP_
