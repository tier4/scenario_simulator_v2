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

#ifndef AWAPI_ACCESSOR__ACCESSOR_HPP_
#define AWAPI_ACCESSOR__ACCESSOR_HPP_

// NOTE: headers are lexicographically sorted.

#include <autoware_api_msgs/msg/awapi_autoware_status.hpp>
#include <autoware_api_msgs/msg/awapi_vehicle_status.hpp>
#include <autoware_control_msgs/msg/engage_mode.hpp>
#include <autoware_perception_msgs/msg/traffic_light_state_array.hpp>
#include <autoware_planning_msgs/msg/route.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_system_msgs/msg/autoware_state.hpp>
#include <autoware_vehicle_msgs/msg/control_mode.hpp>
#include <autoware_vehicle_msgs/msg/shift_stamped.hpp>
#include <autoware_vehicle_msgs/msg/steering.hpp>
#include <autoware_vehicle_msgs/msg/turn_signal.hpp>
#include <autoware_vehicle_msgs/msg/vehicle_command.hpp>
#include <awapi_accessor/conversion.hpp>
#include <awapi_accessor/define_macro.hpp>
#include <awapi_accessor/utility/visibility.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <openscenario_msgs/msg/waypoints_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <mutex>
#include <utility>

namespace autoware_api
{

struct AutowareError
{
};

class Accessor : public rclcpp::Node
{
  std::mutex mutex;

public:
#ifndef NDEBUG
  /** ---- DummyData -----------------------------------------------------------
   *
   *  Topic: ~/dummy
   *
   * ------------------------------------------------------------------------ */
  using DebugString = std_msgs::msg::String;

  DEFINE_PUBLISHER(DebugString);
  DEFINE_SUBSCRIPTION(DebugString);
#endif

  /** ---- AutowareEngage ------------------------------------------------------
   *
   *  Topic: /awapi/autoware/put/engage
   *
   *  Overloads:
   *    setAutowareEngage(const AutowareEngage &) const
   *    setAutowareEngage(const bool) const
   *
   * ------------------------------------------------------------------------ */
  using AutowareEngage = autoware_control_msgs::msg::EngageMode;

  DEFINE_PUBLISHER(AutowareEngage);

  decltype(auto) setAutowareEngage(const bool value = true)
  {
    return setAutowareEngage(convertTo<AutowareEngage>(value));
  }

  /** ---- AutowareRoute -------------------------------------------------------
   *
   *  Topic: /awapi/autoware/put/route
   *
   * ------------------------------------------------------------------------ */
  using AutowareRoute = autoware_planning_msgs::msg::Route;

  DEFINE_PUBLISHER(AutowareRoute);

  /** ---- LaneChangeApproval --------------------------------------------------
   *
   *  Topic: /awapi/lane_change/put/approval
   *
   * ------------------------------------------------------------------------ */
  using LaneChangeApproval = std_msgs::msg::Bool;

  DEFINE_PUBLISHER(LaneChangeApproval);

  decltype(auto) setLaneChangeApproval(const bool approve = true)
  {
    return setLaneChangeApproval(convertTo<LaneChangeApproval>(approve));
  }

  /** ---- LaneChangeForce -----------------------------------------------------
   *
   *  Topic: /awapi/lane_change/put/force
   *
   * ------------------------------------------------------------------------ */
  using LaneChangeForce = std_msgs::msg::Bool;

  DEFINE_PUBLISHER(LaneChangeForce);

  /** ---- TrafficLightStateArray ----------------------------------------------
   *
   *  Overwrite the recognition result of traffic light.
   *
   *  Topic: /awapi/traffic_light/put/traffic_light
   *
   * ------------------------------------------------------------------------ */
  using TrafficLightStateArray = autoware_perception_msgs::msg::TrafficLightStateArray;

  DEFINE_PUBLISHER(TrafficLightStateArray);

  /** ---- VehicleVelocity -----------------------------------------------------
   *
   *  Set upper bound of velocity.
   *
   *  Topic: /awapi/vehicle/put/velocity
   *
   * ------------------------------------------------------------------------ */
  using VehicleVelocity = std_msgs::msg::Float32;

  DEFINE_PUBLISHER(VehicleVelocity);

  template<
    typename T,
    REQUIRES(std::is_floating_point<T>)>
  decltype(auto) setVehicleVelocity(const T value)
  {
    return setVehicleVelocity(convertTo<VehicleVelocity>(value));
  }

  /** ---- AutowareStatus ------------------------------------------------------
   *
   *  Topic: /awapi/autoware/get/status
   *
   * ------------------------------------------------------------------------ */
  using AutowareStatus = autoware_api_msgs::msg::AwapiAutowareStatus;

  DEFINE_SUBSCRIPTION(AutowareStatus);

  /** ---- TrafficLightStatus --------------------------------------------------
   *
   *  Topic: /awapi/traffic_light/get/status
   *
   * ------------------------------------------------------------------------ */
  using TrafficLightStatus = autoware_perception_msgs::msg::TrafficLightStateArray;

  DEFINE_SUBSCRIPTION(TrafficLightStatus);

  /** ---- VehicleStatus -------------------------------------------------------
   *
   *  Topic: /awapi/vehicle/get/status
   *
   * ------------------------------------------------------------------------ */
  using VehicleStatus = autoware_api_msgs::msg::AwapiVehicleStatus;

  DEFINE_SUBSCRIPTION(VehicleStatus);

  /** ---- Trajectory ----------------------------------------------------------
   *
   *  Topic: /planning/scenario_planning/trajectory
   *
   * ------------------------------------------------------------------------ */
  using Trajectory = autoware_planning_msgs::msg::Trajectory;

  DEFINE_SUBSCRIPTION(Trajectory);

public:
  /** ---- Checkpoint ----------------------------------------------------------
   *
   *  Set goal pose of Autoware.
   *
   *  Topic: /planning/mission_planning/checkpoint
   *
   * ------------------------------------------------------------------------ */
  using Checkpoint = geometry_msgs::msg::PoseStamped;

  DEFINE_PUBLISHER(Checkpoint);

  /* ---- CurrentControlMode ---------------------------------------------------
   *
   *  Topic: /vehicle/status/control_mode
   *
   * ------------------------------------------------------------------------ */
  using CurrentControlMode = autoware_vehicle_msgs::msg::ControlMode;

  DEFINE_PUBLISHER(CurrentControlMode);

  decltype(auto) setCurrentControlMode(const std::uint8_t mode = CurrentControlMode::AUTO)
  {
    CurrentControlMode current_control_mode {};
    {
      current_control_mode.data = mode;
    }

    return setCurrentControlMode(current_control_mode);
  }

  /* ---- CurrentPose ----------------------------------------------------------
   *
   *  Topic: /current_pose
   *
   * ------------------------------------------------------------------------ */
  using CurrentPose = geometry_msgs::msg::PoseStamped;

  DEFINE_PUBLISHER(CurrentPose);

  decltype(auto) setCurrentPose(const geometry_msgs::msg::Pose & pose)
  {
    geometry_msgs::msg::PoseStamped current_pose {};
    {
      current_pose.header.stamp = get_clock()->now();
      current_pose.header.frame_id = "map";
      current_pose.pose = pose;
    }

    return setCurrentPose(current_pose);
  }

  /* ---- CurrentShift ---------------------------------------------------------
   *
   *  Topic: /vehicle/status/shift
   *
   *  Overloads:
   *    setCurrentShift(const autoware_vehicle_msgs::msg::ShiftStamped &);
   *    setCurrentShift(const double);
   *
   * ------------------------------------------------------------------------ */
  using CurrentShift = autoware_vehicle_msgs::msg::ShiftStamped;

  DEFINE_PUBLISHER(CurrentShift);

  template<
    typename T,
    REQUIRES(std::is_floating_point<T>)>
  decltype(auto) setCurrentShift(const T twist_linear_x)
  {
    CurrentShift current_shift {};
    {
      using autoware_vehicle_msgs::msg::Shift;

      current_shift.header.stamp = get_clock()->now();
      current_shift.header.frame_id = "map";
      current_shift.shift.data = twist_linear_x >= 0 ? Shift::DRIVE : Shift::REVERSE;
    }

    return setCurrentShift(current_shift);
  }

  decltype(auto) setCurrentShift(const geometry_msgs::msg::Twist & twist)
  {
    return setCurrentShift(twist.linear.x);
  }

  /* ---- CurrentSteering ------------------------------------------------------
   *
   *  Topic: /vehicle/status/steering
   *
   * ------------------------------------------------------------------------ */
  using CurrentSteering = std_msgs::msg::Float32;

  DEFINE_PUBLISHER(CurrentSteering);

  template<
    typename T,
    REQUIRES(std::is_floating_point<T>)>
  decltype(auto) setCurrentSteering(const T value)
  {
    return setCurrentSteering(convertTo<CurrentSteering>(value));
  }

  decltype(auto) setCurrentSteering(const geometry_msgs::msg::Twist & twist)
  {
    return setCurrentSteering(twist.angular.z);
  }

  /* ---- CurrentTurnSignal ----------------------------------------------------
   *
   *  Topic: /vehicle/status/turn_signal
   *
   * ------------------------------------------------------------------------ */
  using CurrentTurnSignal = autoware_vehicle_msgs::msg::TurnSignal;

  DEFINE_PUBLISHER(CurrentTurnSignal);

  decltype(auto) setCurrentTurnSignal()
  {
    CurrentTurnSignal current_turn_signal {};
    {
      current_turn_signal.header.stamp = get_clock()->now();
      current_turn_signal.header.frame_id = "map";
      current_turn_signal.data = autoware_vehicle_msgs::msg::TurnSignal::NONE;
    }

    return setCurrentTurnSignal(current_turn_signal);

    // return setCurrentTurnSignal(getTurnSignalCommand());
  }

  /* ---- CurrentTwist ---------------------------------------------------------
   *
   *  Topic: /vehicle/status/twist
   *
   * ------------------------------------------------------------------------ */
  using CurrentTwist = geometry_msgs::msg::TwistStamped;

  DEFINE_PUBLISHER(CurrentTwist);

  decltype(auto) setCurrentTwist(const geometry_msgs::msg::Twist & twist)
  {
    geometry_msgs::msg::TwistStamped current_twist {};
    {
      current_twist.header.stamp = get_clock()->now();
      current_twist.header.frame_id = "map";
      current_twist.twist = twist;
    }

    return setCurrentTwist(current_twist);
  }

  /* ---- CurrentVelocity ------------------------------------------------------
   *
   *  Topic: /vehicle/status/velocity
   *
   * ------------------------------------------------------------------------ */
  using CurrentVelocity = std_msgs::msg::Float32;

  DEFINE_PUBLISHER(CurrentVelocity);

  template<
    typename T,
    REQUIRES(std::is_floating_point<T>)>
  decltype(auto) setCurrentVelocity(const T twist_linear_x)
  {
    return setCurrentVelocity(convertTo<CurrentVelocity>(twist_linear_x));
  }

  decltype(auto) setCurrentVelocity(const geometry_msgs::msg::Twist & twist)
  {
    return setCurrentVelocity(twist.linear.x);
  }

  /** ---- GoalPose ------------------------------------------------------------
   *
   *  Set goal pose of Autoware.
   *
   *  Topic: /planning/mission_planning/goal
   *
   * ------------------------------------------------------------------------ */
  using GoalPose = geometry_msgs::msg::PoseStamped;

  DEFINE_PUBLISHER(GoalPose);

  /** ---- InitialPose ---------------------------------------------------------
   *
   *  Set initial pose of Autoware.
   *
   *  Topic: /initialpose
   *
   * ------------------------------------------------------------------------ */
  using InitialPose = geometry_msgs::msg::PoseWithCovarianceStamped;

  DEFINE_PUBLISHER(InitialPose);

  decltype(auto) setInitialPose(const geometry_msgs::msg::Pose & pose)
  {
    autoware_api::Accessor::InitialPose initial_pose {};
    {
      initial_pose.header.stamp = get_clock()->now();
      initial_pose.header.frame_id = "map";
      initial_pose.pose.pose = pose;
    }

    return setInitialPose(initial_pose);
  }

  /** ---- InitialTwist --------------------------------------------------------
   *
   *  Set initial velocity of Autoware.
   *
   *  Topic: /initialtwist
   *
   * ------------------------------------------------------------------------ */
  using InitialTwist = geometry_msgs::msg::TwistStamped;

  DEFINE_PUBLISHER(InitialTwist);

  decltype(auto) setInitialTwist()
  {
    autoware_api::Accessor::InitialTwist initial_twist {};
    {
      initial_twist.header.stamp = get_clock()->now();
      initial_twist.header.frame_id = "map";
      initial_twist.twist = geometry_msgs::msg::Twist();
    }

    return setInitialTwist(initial_twist);
  }

  /* ---- Turn Signal Command --------------------------------------------------
   *
   *  Topic: /control/turn_signal_cmd
   *
   * ------------------------------------------------------------------------ */
  using TurnSignalCommand = autoware_vehicle_msgs::msg::TurnSignal;

  DEFINE_SUBSCRIPTION(TurnSignalCommand);

  /** ---- InitialTwist --------------------------------------------------------
   *
   *  Set initial velocity of Autoware.
   *
   *  Topic: /initialtwist
   *
   * ------------------------------------------------------------------------ */
  using VehicleCommand = autoware_vehicle_msgs::msg::VehicleCommand;

  DEFINE_SUBSCRIPTION(VehicleCommand);

public:
  auto isWaitingForRoute() const noexcept
  {
    using autoware_system_msgs::msg::AutowareState;

    return CURRENT_VALUE_OF(AutowareStatus).autoware_state == AutowareState::WAITING_FOR_ROUTE;
  }

  auto isEmergency() const noexcept
  {
    using autoware_system_msgs::msg::AutowareState;

    return CURRENT_VALUE_OF(AutowareStatus).autoware_state == AutowareState::EMERGENCY;
  }

  auto isWaitingForEngage() const noexcept
  {
    using autoware_system_msgs::msg::AutowareState;

    return CURRENT_VALUE_OF(AutowareStatus).autoware_state == AutowareState::WAITING_FOR_ENGAGE;
  }

  auto isReady() const noexcept
  {
    static auto ready = false;

    return ready || (ready = isWaitingForRoute());
  }

  auto isNotReady() const noexcept
  {
    return !isReady();
  }

  void checkAutowareState() const
  {
    if (isReady() && isEmergency()) {
      throw AutowareError();
    }
  }

  tf2_ros::Buffer transform_buffer;
  tf2_ros::TransformBroadcaster transform_broadcaster;

  geometry_msgs::msg::TransformStamped current_transform;

  const auto & setTransform(const geometry_msgs::msg::Pose & pose)
  {
    current_transform.header.stamp = get_clock()->now();
    current_transform.header.frame_id = "map";
    current_transform.child_frame_id = "base_link";
    current_transform.transform.translation.x = pose.position.x;
    current_transform.transform.translation.y = pose.position.y;
    current_transform.transform.translation.z = pose.position.z;
    current_transform.transform.rotation = pose.orientation;

    return current_transform;
  }

  const rclcpp::TimerBase::SharedPtr timer;

  decltype(auto) updateTransform()
  {
    current_transform.header.stamp = get_clock()->now();
    return transform_broadcaster.sendTransform(current_transform);
  }

public:
  template<
    typename ... Ts>
  AWAPI_ACCESSOR_PUBLIC
  explicit Accessor(Ts && ... xs)
  : rclcpp::Node("awapi_accessor_node", std::forward<decltype(xs)>(xs)...),
#ifndef NDEBUG
    INIT_PUBLISHER(DebugString, "debug/string"),
    INIT_SUBSCRIPTION(DebugString, "debug/string", []() {}),
#endif
    // AWAPI topics (lexicographically sorted)
    INIT_PUBLISHER(AutowareEngage, "/autoware/put/engage"),
    INIT_PUBLISHER(AutowareRoute, "/autoware/put/route"),
    INIT_PUBLISHER(LaneChangeApproval, "/lane_change/put/approval"),
    INIT_PUBLISHER(LaneChangeForce, "/lane_change/put/force"),
    INIT_PUBLISHER(TrafficLightStateArray, "/traffic_light/put/traffic_light"),
    INIT_PUBLISHER(VehicleVelocity, "/vehicle/put/velocity"),
    INIT_SUBSCRIPTION(AutowareStatus, "/autoware/get/status", checkAutowareState),
    INIT_SUBSCRIPTION(TrafficLightStatus, "/traffic_light/get/status", []() {}),
    INIT_SUBSCRIPTION(VehicleStatus, "/vehicle/get/status", []() {}),

    // Simulation specific topics (lexicographically sorted)
    INIT_PUBLISHER(Checkpoint, "/planning/mission_planning/checkpoint"),
    INIT_PUBLISHER(CurrentControlMode, "/vehicle/status/control_mode"),
    INIT_PUBLISHER(CurrentPose, "/current_pose"),
    INIT_PUBLISHER(CurrentShift, "/vehicle/status/shift"),
    INIT_PUBLISHER(CurrentSteering, "/vehicle/status/steering"),
    INIT_PUBLISHER(CurrentTurnSignal, "/vehicle/status/turn_signal"),
    INIT_PUBLISHER(CurrentTwist, "/vehicle/status/twist"),
    INIT_PUBLISHER(CurrentVelocity, "/vehicle/status/velocity"),
    INIT_PUBLISHER(GoalPose, "/planning/mission_planning/goal"),
    INIT_PUBLISHER(InitialPose, "/initialpose"),
    INIT_PUBLISHER(InitialTwist, "/initialtwist"),
    INIT_SUBSCRIPTION(TurnSignalCommand, "/control/turn_signal_cmd", []() {}),
    INIT_SUBSCRIPTION(VehicleCommand, "/control/vehicle_cmd", []() {}),

    transform_buffer(get_clock()),
    transform_broadcaster(std::shared_ptr<rclcpp::Node>(this, [](auto && ...) {})),

    timer(
      create_wall_timer(
        std::chrono::milliseconds(5),
        [this]()
        {
          return updateTransform();
        }))
  {}

public:
  const openscenario_msgs::msg::WaypointsArray getWaypoints()
  {
    openscenario_msgs::msg::WaypointsArray waypoints;
    for (const auto trajectory_point : trajectory.points) {
      geometry_msgs::msg::Point p = trajectory_point.pose.position;
      waypoints.waypoints.emplace_back(p);
    }
    return waypoints;
  }

private:
  Trajectory trajectory;
};

}  // namespace autoware_api

#include <awapi_accessor/undefine_macro.hpp>

#endif  // AWAPI_ACCESSOR__ACCESSOR_HPP_
