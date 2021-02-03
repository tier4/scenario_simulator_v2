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

#include <autoware_auto_msgs/msg/complex32.hpp>
#include <autoware_auto_msgs/msg/vehicle_control_command.hpp>
#include <autoware_auto_msgs/msg/vehicle_kinematic_state.hpp>
#include <autoware_auto_msgs/msg/vehicle_state_command.hpp>
#include <awapi_accessor/accessor.hpp>
#include <simulation_api/entity/vehicle_entity.hpp>
#include <simulation_api/vehicle_model/sim_model.hpp>

// headers in pugixml
#include <pugixml.hpp>
#include <boost/optional.hpp>

#include <memory>
#include <string>
#include <utility>

namespace simulation_api
{
namespace entity
{
class EgoEntity : public VehicleEntity
{
  const std::shared_ptr<autoware_api::Accessor> autoware;  // TODO(yamacir-kit): Use unique_ptr

public:
  /* ---------------------------------------------------------------------------
   *
   *  NOTE: from yamacir-kit
   *
   *  This constructor makes an Ego type entity with the proper initial state.
   *  It is mainly used when writing scenarios in C++.
   *
   * ------------------------------------------------------------------------ */
  template
  <
    typename ... Ts  // Maybe, VehicleParameters or pugi::xml_node
  >
  explicit EgoEntity(
    const std::string & name,
    const openscenario_msgs::msg::EntityStatus & initial_state, Ts && ... xs)
  : VehicleEntity(name, initial_state, std::forward<decltype(xs)>(xs)...)
  {
    setStatus(initial_state);
  }

  // using VehicleEntity::VehicleEntity;

  /* ---------------------------------------------------------------------------
   *
   *  NOTE: from yamacir-kit
   *
   *  This constructor builds an Ego-type entity with an ambiguous initial
   *  state. In this case, the values for status_ and current_kinematic_state_
   *  are boost::none, respectively.
   *
   *  This constructor is used for the purpose of delaying the transmission of
   *  the initial position from the entity's spawn. If you build an ego-type
   *  entity with this constructor, you must explicitly call setStatus at least
   *  once before the first onUpdate call to establish location and kinematic
   *  state.
   *
   *  For OpenSCENARIO, setStatus before the onUpdate call is called by
   *  TeleportAction in the Storyboard.Init section.
   *
   * ------------------------------------------------------------------------ */
  template
  <
    typename ... Ts
  >
  explicit EgoEntity(Ts && ... xs)
  : VehicleEntity(std::forward<decltype(xs)>(xs)...),
    autoware(std::make_shared<autoware_api::Accessor>(rclcpp::NodeOptions()))
  {
    std::thread(
      [autoware]()
      {
        rclcpp::executors::MultiThreadedExecutor executor {};
        executor.add_node((*autoware).get_node_base_interface());
        executor.spin();
      }).detach();
  }

  void setVehicleCommands(
    const boost::optional<autoware_auto_msgs::msg::VehicleControlCommand> & control_cmd,
    const boost::optional<autoware_auto_msgs::msg::VehicleStateCommand> & state_cmd);

  const std::string getCurrentAction() const
  {
    return "none";
  }

  void onUpdate(double current_time, double step_time) override;

  bool setStatus(const openscenario_msgs::msg::EntityStatus & status);

  const auto & getCurrentKinematicState() const noexcept
  {
    return current_kinematic_state_;
  }

private:
  autoware_auto_msgs::msg::Complex32 toHeading(const double yaw);

  const openscenario_msgs::msg::EntityStatus getEntityStatus(
    const double time,
    const double step_time) const;

  boost::optional<geometry_msgs::msg::Pose> origin_;
  boost::optional<autoware_auto_msgs::msg::VehicleControlCommand> control_cmd_;
  boost::optional<autoware_auto_msgs::msg::VehicleStateCommand> state_cmd_;
  boost::optional<autoware_auto_msgs::msg::VehicleKinematicState> current_kinematic_state_;

  std::shared_ptr<SimModelInterface> vehicle_model_ptr_;

  boost::optional<double> previous_velocity_;
  boost::optional<double> previous_angular_velocity_;
};
}      // namespace entity
}  // namespace simulation_api

#endif  // SIMULATION_API__ENTITY__EGO_ENTITY_HPP_
