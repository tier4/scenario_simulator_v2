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

#ifndef TRAFFIC_SIMULATOR__ENTITY__EGO_ENTITY_HPP_
#define TRAFFIC_SIMULATOR__ENTITY__EGO_ENTITY_HPP_

#include <traffic_simulator/entity/vehicle_entity.hpp>
#include <traffic_simulator/vehicle_model/sim_model_ideal.hpp>
#include <traffic_simulator/vehicle_model/sim_model_time_delay.hpp>

#undef TRAFFIC_SIMULATOR_ISOLATE_STANDARD_OUTPUT_FROM_AUTOWARE

#ifdef TRAFFIC_SIMULATOR_ISOLATE_STANDARD_OUTPUT_FROM_AUTOWARE
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#endif

#include <sys/wait.h>  // for EgoEntity::~EgoEntity
#include <tf2/utils.h>

#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware_auto_msgs/msg/complex32.hpp>
#include <autoware_auto_msgs/msg/vehicle_control_command.hpp>
#include <autoware_auto_msgs/msg/vehicle_kinematic_state.hpp>
#include <autoware_auto_msgs/msg/vehicle_state_command.hpp>
#include <awapi_accessor/accessor.hpp>
#include <boost/filesystem.hpp>
#include <boost/optional.hpp>
#include <chrono>
#include <cstdlib>
#include <future>
#include <memory>
#include <pugixml.hpp>
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#define DEBUG_VALUE(...) \
  std::cout << "\x1b[32m" #__VA_ARGS__ " = " << (__VA_ARGS__) << "\x1b[0m" << std::endl

#define DEBUG_LINE() \
  std::cout << "\x1b[32m" << __FILE__ << ":" << __LINE__ << "\x1b[0m" << std::endl

namespace traffic_simulator
{
namespace entity
{
class EgoEntity : public VehicleEntity
{
  // NOTE: One day we will have to do simultaneous simulations of multiple Autowares.
  static std::unordered_map<
    std::string, std::shared_ptr<autoware_api::Accessor>  // TODO(yamacir-kit): virtualize accessor.
    >
    autowares;

  decltype(fork()) autoware_process_id = 0;

  // XXX DIRTY HACK: The EntityManager terribly requires Ego to be Copyable.
  std::shared_ptr<std::promise<void>> accessor_status;

  // XXX DIRTY HACK: The EntityManager terribly requires Ego to be Copyable.
  std::shared_ptr<std::thread> accessor_spinner;

public:
  EgoEntity() = delete;

  // TODO(yamacir-kit): EgoEntity(EgoEntity &&) = delete;
  // TODO(yamacir-kit): EgoEntity(const EgoEntity &) = delete;
  // TODO(yamacir-kit): EgoEntity & operator=(EgoEntity &&) = delete;
  // TODO(yamacir-kit): EgoEntity & operator=(const EgoEntity &) = delete;

  /* ---- NOTE -----------------------------------------------------------------
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
  explicit EgoEntity(
    const std::string & name, const boost::filesystem::path & lanelet2_map_osm,
    const double step_time, const openscenario_msgs::msg::VehicleParameters & parameters);

  ~EgoEntity() override;

  bool autoware_initialized = false;

  void initializeAutoware();

  void requestAcquirePosition(
    const geometry_msgs::msg::PoseStamped &,
    const std::vector<geometry_msgs::msg::PoseStamped> & = {});

  void requestAssignRoute(
    const std::vector<openscenario_msgs::msg::LaneletPose> & waypoints) override;

  void setTargetSpeed(const double value, const bool)
  {
    const auto current = getStatus();

    Eigen::VectorXd v(5);
    {
      v << 0, 0, 0, value, 0;
    }

    (*vehicle_model_ptr_).setState(v);
  }

  const std::string getCurrentAction() const
  {
    return std::atomic_load(&autowares.at(name))->getAutowareStatus().autoware_state;
  }

  void onUpdate(double current_time, double step_time);

  bool setStatus(const openscenario_msgs::msg::EntityStatus & status);

  openscenario_msgs::msg::WaypointsArray getWaypoints() const;

private:
// TODO(yamacir-kit): Define AutowareError type as struct based on std::runtime_error
#define DEFINE_WAIT_FOR_AUTOWARE_STATE_TO_BE(STATE)                                             \
  template <typename Thunk, typename Seconds = std::chrono::seconds>                            \
  void waitForAutowareStateToBe##STATE(Thunk thunk, Seconds interval = std::chrono::seconds(1)) \
    const                                                                                       \
  {                                                                                             \
    static const auto duration_max = std::chrono::seconds(30);                                  \
    Seconds duration{0};                                                                        \
    for (rclcpp::WallRate rate{interval}; !std::atomic_load(&autowares.at(name))->is##STATE();  \
         rate.sleep()) {                                                                        \
      if ((duration += interval) < duration_max) {                                              \
        thunk();                                                                                \
      } else {                                                                                  \
        const auto current_state =                                                              \
          std::atomic_load(&autowares.at(name))->getAutowareStatus().autoware_state;            \
        std::stringstream ss{};                                                                 \
        ss << "The simulator waited " << duration_max.count()                                   \
           << " seconds, expecting the Autoware state to transitioning to " << #STATE           \
           << ", but there was no change. The current Autoware state is "                       \
           << (current_state.empty() ? "NOT PUBLISHED YET" : current_state)                     \
           << ". This error is most likely due to the Autoware state transition "               \
           << "conditions changing with the update. Please report this error to "               \
           << "the developer. This error message was written by @yamacir-kit.";                 \
        using AutowareError = std::runtime_error;                                               \
        throw AutowareError(ss.str());                                                          \
      }                                                                                         \
    }                                                                                           \
    RCLCPP_INFO_STREAM(                                                                         \
      std::atomic_load(&autowares.at(name))->get_logger(), "Autoware is " #STATE " now.");      \
  }                                                                                             \
  static_assert(true, "")

  DEFINE_WAIT_FOR_AUTOWARE_STATE_TO_BE(InitializingVehicle);
  DEFINE_WAIT_FOR_AUTOWARE_STATE_TO_BE(WaitingForRoute);
  DEFINE_WAIT_FOR_AUTOWARE_STATE_TO_BE(Planning);
  DEFINE_WAIT_FOR_AUTOWARE_STATE_TO_BE(WaitingForEngage);
  DEFINE_WAIT_FOR_AUTOWARE_STATE_TO_BE(Driving);
  DEFINE_WAIT_FOR_AUTOWARE_STATE_TO_BE(ArrivedGoal);
  DEFINE_WAIT_FOR_AUTOWARE_STATE_TO_BE(Emergency);
  DEFINE_WAIT_FOR_AUTOWARE_STATE_TO_BE(Finalizing);

#undef DEFINE_WAIT_FOR_AUTOWARE_STATE_TO_BE

  void updateAutoware(const geometry_msgs::msg::Pose & current_pose)
  {
    geometry_msgs::msg::Twist current_twist;
    {
      current_twist.linear.x = (*vehicle_model_ptr_).getVx();
      current_twist.angular.z = (*vehicle_model_ptr_).getWz();
    }

    std::atomic_load(&autowares.at(name))->setCurrentControlMode();
    std::atomic_load(&autowares.at(name))->setCurrentPose(current_pose);
    std::atomic_load(&autowares.at(name))->setCurrentShift(current_twist);
    std::atomic_load(&autowares.at(name))->setCurrentSteering(current_twist);
    std::atomic_load(&autowares.at(name))->setCurrentTurnSignal();
    std::atomic_load(&autowares.at(name))->setCurrentTwist(current_twist);
    std::atomic_load(&autowares.at(name))->setCurrentVelocity(current_twist);
    std::atomic_load(&autowares.at(name))->setLaneChangeApproval();
    std::atomic_load(&autowares.at(name))->setLocalizationTwist(current_twist);
    std::atomic_load(&autowares.at(name))->setTransform(current_pose);
    std::atomic_load(&autowares.at(name))->setVehicleVelocity(parameters.performance.max_speed);
  }

private:
  const openscenario_msgs::msg::EntityStatus getEntityStatus(
    const double time, const double step_time) const;

  boost::optional<geometry_msgs::msg::Pose> initial_pose_;

  const std::shared_ptr<SimModelInterface> vehicle_model_ptr_;

  boost::optional<double> previous_linear_velocity_;
  boost::optional<double> previous_angular_velocity_;
};
}  // namespace entity
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__ENTITY__EGO_ENTITY_HPP_
