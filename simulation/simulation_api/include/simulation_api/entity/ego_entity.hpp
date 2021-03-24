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

#undef TRAFFIC_SIMULATOR_ISOLATE_STANDARD_OUTPUT_FROM_AUTOWARE

#ifdef TRAFFIC_SIMULATOR_ISOLATE_STANDARD_OUTPUT_FROM_AUTOWARE
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#endif

#include <autoware_auto_msgs/msg/complex32.hpp>
#include <autoware_auto_msgs/msg/vehicle_control_command.hpp>
#include <autoware_auto_msgs/msg/vehicle_kinematic_state.hpp>
#include <autoware_auto_msgs/msg/vehicle_state_command.hpp>
#include <awapi_accessor/accessor.hpp>
#include <boost/filesystem.hpp>
#include <boost/optional.hpp>
#include <pugixml.hpp>
#include <simulation_api/entity/vehicle_entity.hpp>
#include <simulation_api/vehicle_model/sim_model_ideal.hpp>  // NOTE: Copy from ArchitectureProposal.iv
#include <sys/wait.h>  // for EgoEntity::~EgoEntity

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <future>
#include <memory>
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

namespace simulation_api
{
namespace entity
{
class EgoEntity : public VehicleEntity
{
  // NOTE: One day we will have to do simultaneous simulations of multiple Autowares.
  static std::unordered_map<
    std::string, std::shared_ptr<autoware_api::Accessor>  // TODO(yamacir-kit): virtualize accessor.
  > autowares;

  decltype(fork()) autoware_process_id = 0;

  // XXX DIRTY HACK: The EntityManager terribly requires Ego to be Copyable.
  std::shared_ptr<std::promise<void>> accessor_status;

  // XXX DIRTY HACK: The EntityManager terribly requires Ego to be Copyable.
  std::shared_ptr<std::thread> accessor_spinner;

  /* ---- NOTE -----------------------------------------------------------------
   *
   *  If you can't explain the difference between char * and char [], don't
   *  edit this function even if it looks strange.
   *
   * ------------------------------------------------------------------------ */
  auto execute(const std::vector<std::string> & f_xs)
  {
    std::vector<std::vector<char>> buffer {};

    buffer.resize(f_xs.size());

    std::vector<std::add_pointer<char>::type> argv {};

    argv.reserve(f_xs.size());

    for (const auto & each : f_xs) {
      buffer.emplace_back(std::begin(each), std::end(each));
      buffer.back().push_back('\0');
      argv.push_back(buffer.back().data());
    }

    argv.emplace_back(static_cast<std::add_pointer<char>::type>(0));

    return ::execvp(argv[0], argv.data());
  }

public:
  EgoEntity() = delete;

  // TODO(yamacir-kit): EgoEntity(EgoEntity &&) = delete;
  // TODO(yamacir-kit): EgoEntity(const EgoEntity &) = delete;
  // TODO(yamacir-kit): EgoEntity & operator=(EgoEntity &&) = delete;
  // TODO(yamacir-kit): EgoEntity & operator=(const EgoEntity &) = delete;

  /* ---- NOTE -----------------------------------------------------------------
   *
   *  This constructor makes an Ego type entity with the proper initial state.
   *  It is mainly used when writing scenarios in C++.
   *
   * ------------------------------------------------------------------------ */
  template<typename ... Ts>
  explicit EgoEntity(
    const std::string & name,
    const openscenario_msgs::msg::EntityStatus & initial_state, Ts && ... xs)
  : VehicleEntity(name, initial_state, std::forward<decltype(xs)>(xs)...)
    // ,
    // vehicle_model_ptr_(
    //   std::make_shared<SimModelIdealSteerVel>(
    //     parameters.axles.front_axle.position_x - parameters.axles.rear_axle.position_x))
  {
    setStatus(initial_state);
  }

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
    const boost::filesystem::path & lanelet2_map_osm,
    const std::string & name,
    const openscenario_msgs::msg::VehicleParameters & parameters)
  : VehicleEntity(name, parameters)
    // ,
    // vehicle_model_ptr_(
    //   std::make_shared<SimModelIdealSteerVel>(
    //     parameters.axles.front_axle.position_x - parameters.axles.rear_axle.position_x))
  {
    auto launch_autoware =
      [&]()
      {
        auto get_parameter =
          [](const std::string & name, const auto & alternate)
          {
            rclcpp::Node node {
              "get_parameter", "simulation"
            };

            auto value = alternate;

            using value_type = typename std::decay<decltype(value)>::type;

            node.declare_parameter<value_type>(name, value);
            node.get_parameter<value_type>(name, value);

            return value;
          };

        /* ---- NOTE -----------------------------------------------------------
         *
         *  The actual values of these parameters are set by
         *  scenario_test_runner.launch.py as parameters of
         *  openscenario_interpreter_node.
         *
         * ------------------------------------------------------------------ */
        const auto autoware_launch_package =
          get_parameter("autoware_launch_package", std::string(""));
        const auto autoware_launch_file =
          get_parameter("autoware_launch_file", std::string(""));

        auto child =
          [&]()
          {
            DEBUG_VALUE(lanelet2_map_osm);

            const std::vector<std::string> argv {
              "python3",
              "/opt/ros/foxy/bin/ros2",  // NOTE: The command 'ros2' is a Python script.
              "launch",
              autoware_launch_package,
              autoware_launch_file,
              std::string("map_path:=") += lanelet2_map_osm.parent_path().string(),
              std::string("lanelet2_map_file:=") += lanelet2_map_osm.filename().string()
            };

            for (const auto & each : argv) {
              std::cout << each << (&each != &argv.back() ? ' ' : '\n');
            }

            #ifdef TRAFFIC_SIMULATOR_ISOLATE_STANDARD_OUTPUT_FROM_AUTOWARE
            const std::string name = "/tmp/scenario_test_runner/autoware-output.txt";
            const auto fd = ::open(name.c_str(), O_RDWR | O_CREAT, S_IRUSR | S_IWUSR);
            ::dup2(fd, STDOUT_FILENO);
            ::dup2(fd, STDERR_FILENO);
            ::close(fd);
            #endif

            if (execute(argv) < 0) {
              std::cout << std::system_error(errno, std::system_category()).what() << std::endl;
              std::exit(EXIT_FAILURE);
            }
          };

        if ((autoware_process_id = fork()) < 0) {
          throw std::system_error(errno, std::system_category());
        } else if (autoware_process_id == 0) {
          return child();
        }
      };

    if (autowares.find(name) == std::end(autowares)) {
      auto my_name = name;
      std::replace(std::begin(my_name), std::end(my_name), ' ', '_');
      autowares.emplace(
        name,
        std::make_shared<autoware_api::Accessor>(
          "awapi_accessor",
          "simulation/" + my_name,  // NOTE: Specified in scenario_test_runner.launch.py
          rclcpp::NodeOptions().use_global_arguments(false)));

      launch_autoware();
    }

    /* ---- NOTE ---------------------------------------------------------------
     *
     *  The simulator needs to run in a fixed-cycle loop, but the communication
     *  part with Autoware needs to run at a higher frequency (e.g. the
     *  transform in map -> base_link needs to be updated at a higher frequency
     *  even if the value does not change). We also need to keep collecting the
     *  latest values of topics from Autoware, independently of the simulator.
     *
     *  For this reason, autoware_api::Accessor, which is responsible for
     *  communication with Autoware, should run in an independent thread. This
     *  is probably an EXTREMELY DIRTY HACK.
     *
     *  Ideally, the constructor caller of traffic_simulator::API should
     *  provide a std::shared_ptr to autoware_api::Accessor and spin that node
     *  with MultiThreadedExecutor.
     *
     *  If you have a nice idea to solve this, and are interested in improving
     *  the quality of the Tier IV simulator, please contact @yamacir-kit.
     *
     * ---------------------------------------------------------------------- */
    if (autowares.at(name).use_count() < 2) {
      accessor_status = std::make_shared<std::promise<void>>();
      accessor_spinner = std::make_shared<std::thread>(
        [](const auto node, auto status)  // NOTE: This copy increments use_count to 2 from 1.
        {
          while (
            rclcpp::ok() &&
            status.wait_for(std::chrono::milliseconds(1)) == std::future_status::timeout)
          {
            rclcpp::spin_some(node);
          }
        }, autowares.at(name), std::move(accessor_status->get_future()));
    }
  }

  ~EgoEntity() override
  {
    if (accessor_spinner && accessor_spinner.use_count() < 2 && accessor_spinner->joinable()) {
      accessor_status->set_value();
      accessor_spinner->join();
      autowares.erase(name);
      int status = 0;
      if (
        ::kill(autoware_process_id, SIGINT) < 0 ||
        ::waitpid(autoware_process_id, &status, WUNTRACED) < 0)
      {
        std::cout << std::system_error(errno, std::system_category()).what() << std::endl;
        std::exit(EXIT_FAILURE);
      }
    }
  }

  bool autoware_initialized = false;

  auto initializeAutoware()
  {
    const auto current_entity_status = getStatus();

    if (!std::exchange(autoware_initialized, true)) {
      std::atomic_load(&autowares.at(name))->setInitialPose(current_entity_status.pose);

      waitForAutowareStateToBeInitializingVehicle(
        [&]()
        {
          return updateAutoware(current_entity_status.pose);
        });

      /* ---- NOTE ---------------------------------------------------------------
       *
       *  awapi_awiv_adapter requires at least 'initialpose' and 'initialtwist'
       *  and tf to be published. Member function EgoEntity::waitForAutowareToBe*
       *  are depends a topic '/awapi/autoware/get/status' published by
       *  awapi_awiv_adapter.
       *
       * ---------------------------------------------------------------------- */
      waitForAutowareStateToBeWaitingForRoute(
        [&]()
        {
          return updateAutoware(current_entity_status.pose);
        });
    }
  }

  void requestAcquirePosition(
    const geometry_msgs::msg::PoseStamped & map_pose)
  {
    initializeAutoware();

    const auto current_pose = getStatus().pose;

    waitForAutowareStateToBeWaitingForRoute(
      [&]()  // NOTE: This is assertion.
      {
        return updateAutoware(current_pose);
      });

    waitForAutowareStateToBePlanning(
      [&]()
      {
        std::atomic_load(&autowares.at(name))->setGoalPose(map_pose);
        return updateAutoware(current_pose);
      });

    waitForAutowareStateToBeWaitingForEngage(
      [&]()
      {
        return updateAutoware(current_pose);
      });

    waitForAutowareStateToBeDriving(
      [&]()
      {
        std::atomic_load(&autowares.at(name))->setAutowareEngage(true);
        return updateAutoware(current_pose);
      });
  }

  decltype(auto) setTargetSpeed(const double value, const bool)
  {
    std::cout << "\x1b[31mEgo::setTargetSpeed " << value << "\x1b[0m" << std::endl;
    // std::atomic_load(&autowares.at(name))->setInitialTwist(value);
  }

  const std::string getCurrentAction() const
  {
    return "none";
  }

  void onUpdate(double current_time, double step_time) override;

  bool setStatus(const openscenario_msgs::msg::EntityStatus & status);

  // const auto & getCurrentKinematicState() const noexcept
  // {
  //   return current_kinematic_state_;
  // }

  openscenario_msgs::msg::WaypointsArray getWaypoints() const;

private:
  // TODO(yamacir-kit): Define AutowareError type as struct based on std::runtime_error
  #define DEFINE_WAIT_FOR_AUTOWARE_STATE_TO_BE(STATE) \
  template<typename Thunk> \
  void waitForAutowareStateToBe ## STATE(Thunk thunk, std::size_t count_max = 300) const \
  { \
    std::size_t count = 0; \
    for ( \
      rclcpp::WallRate rate {std::chrono::milliseconds(100)}; \
      !std::atomic_load(&autowares.at(name))->is ## STATE(); \
      rate.sleep()) \
    { \
      if (count++ < count_max) { \
        thunk(); \
      } else { \
        const auto current_state = \
          std::atomic_load(&autowares.at(name))->getAutowareStatus().autoware_state; \
        std::stringstream ss {}; \
        ss << "The simulator waited " \
           << (count / 10) \
           << " seconds, expecting the Autoware state to transitioning to " \
           << #STATE \
           << ", but there was no change. The current Autoware state is " \
           << (current_state.empty() ? "NOT PUBLISHED YET" : current_state) \
           << ". This error is most likely due to the Autoware state transition " \
           << "conditions changing with the update. Please report this error to " \
           << "the developer. This error message was written by @yamacir-kit."; \
        using AutowareError = std::runtime_error; \
        throw AutowareError(ss.str()); \
      } \
    } \
    RCLCPP_INFO_STREAM( \
      std::atomic_load(&autowares.at(name))->get_logger(), \
      "Autoware is " #STATE " now."); \
  } static_assert(true, "")

  DEFINE_WAIT_FOR_AUTOWARE_STATE_TO_BE(InitializingVehicle);
  DEFINE_WAIT_FOR_AUTOWARE_STATE_TO_BE(WaitingForRoute);
  DEFINE_WAIT_FOR_AUTOWARE_STATE_TO_BE(Planning);
  DEFINE_WAIT_FOR_AUTOWARE_STATE_TO_BE(WaitingForEngage);
  DEFINE_WAIT_FOR_AUTOWARE_STATE_TO_BE(Driving);
  DEFINE_WAIT_FOR_AUTOWARE_STATE_TO_BE(ArrivedGoal);
  DEFINE_WAIT_FOR_AUTOWARE_STATE_TO_BE(Emergency);
  DEFINE_WAIT_FOR_AUTOWARE_STATE_TO_BE(Finalizing);

  #undef DEFINE_WAIT_FOR_AUTOWARE_STATE_TO_BE

  void updateAutoware(
    const geometry_msgs::msg::Pose & current_pose)
  {
    geometry_msgs::msg::Twist current_twist;
    {
      current_twist.linear.x =
        std::atomic_load(&autowares.at(name))->getVehicleCommand().control.velocity;
      current_twist.angular.z =
        std::atomic_load(&autowares.at(name))->getVehicleCommand().control.steering_angle;
    }

    // DEBUG_VALUE(current_twist.linear.x);
    // DEBUG_VALUE(current_twist.angular.z);
    //
    // DEBUG_VALUE(current_pose.position.x);
    // DEBUG_VALUE(current_pose.position.y);
    // DEBUG_VALUE(current_pose.position.z);

    std::atomic_load(&autowares.at(name))->setCurrentControlMode();
    std::atomic_load(&autowares.at(name))->setCurrentPose(current_pose);
    std::atomic_load(&autowares.at(name))->setCurrentShift(current_twist);
    std::atomic_load(&autowares.at(name))->setCurrentSteering(current_twist);
    std::atomic_load(&autowares.at(name))->setCurrentTurnSignal();
    std::atomic_load(&autowares.at(name))->setCurrentTwist(current_twist);
    std::atomic_load(&autowares.at(name))->setCurrentVelocity(current_twist);
    std::atomic_load(&autowares.at(name))->setLaneChangeApproval();
    std::atomic_load(&autowares.at(name))->setTransform(current_pose);
    std::atomic_load(&autowares.at(name))->setVehicleVelocity(50);  // 50[m/s] = 180[km/h]
  }

private:
  const openscenario_msgs::msg::EntityStatus getEntityStatus(
    const double time,
    const double step_time) const;

  boost::optional<geometry_msgs::msg::Pose> origin_;

  // boost::optional<autoware_auto_msgs::msg::VehicleControlCommand> control_cmd_;
  // boost::optional<autoware_auto_msgs::msg::VehicleStateCommand> state_cmd_;
  // boost::optional<autoware_auto_msgs::msg::VehicleKinematicState> current_kinematic_state_;

  std::shared_ptr<SimModelInterface> vehicle_model_ptr_;

  boost::optional<double> previous_velocity_;
  boost::optional<double> previous_angular_velocity_;
};
}      // namespace entity
}  // namespace simulation_api

#endif  // SIMULATION_API__ENTITY__EGO_ENTITY_HPP_
