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

#ifndef CONCEALER__AUTOWARE_HPP_
#define CONCEALER__AUTOWARE_HPP_

#include <concealer/autoware_def.hpp>

// #define CONCEALER_ISOLATE_STANDARD_OUTPUT

#include <sys/wait.h>

#include <chrono>
#include <concealer/continuous_transform_broadcaster.hpp>
#include <concealer/fundamental_api.hpp>
#include <concealer/launch.hpp>
#include <concealer/miscellaneous_api.hpp>
#include <concealer/task_queue.hpp>
#include <concealer/transition_assertion.hpp>
#include <concealer/utility/visibility.hpp>
#include <exception>
#include <future>
#include <mutex>
#include <thread>
#include <utility>

namespace concealer
{
/* ---- NOTE -------------------------------------------------------------------
 *
 *  The magic class 'Autoware' is a class that makes it easy to work with
 *  Autoware from C++. The main features of this class are as follows
 *
 *    (1) Launch Autoware in an independent process upon instantiation of the
 *        class.
 *    (2) Properly terminates the Autoware process started by the constructor
 *        upon destruction of the class.
 *    (3) Probably the simplest instructions to Autoware, consisting of
 *        initialize, plan, and engage.
 *
 * -------------------------------------------------------------------------- */
class Autoware : public rclcpp::Node,
                 public FundamentalAPI<Autoware>,
                 public MiscellaneousAPI<Autoware>,
                 public ContinuousTransformBroadcaster<Autoware>,
                 public TransitionAssertion<Autoware>
{
  friend class ContinuousTransformBroadcaster<Autoware>;
  friend class FundamentalAPI<Autoware>;
  friend class MiscellaneousAPI<Autoware>;
  friend class TransitionAssertion<Autoware>;

  mutable std::mutex mutex;

  const pid_t process_id;

  std::promise<void> promise;

  std::thread spinner;

  const rclcpp::TimerBase::SharedPtr updater;

  /* ---- NOTE -----------------------------------------------------------------
   *
   *  For a simple simulator, I believe that most of the information that needs
   *  to be published could be easily calculated from Twist and Pose. However,
   *  if the information that the update function wants to use cannot be
   *  created only from Twist and Pose, you can add it after separating it with
   *  '#if AUTOWARE_AUTO ... #endif'.
   *
   * ------------------------------------------------------------------------ */
  geometry_msgs::msg::Pose current_pose;

  geometry_msgs::msg::Twist current_twist;

  bool initialize_was_called = false;

  TaskQueue task_queue;

  std::exception_ptr thrown;

public:
  template <typename... Ts>
  CONCEALER_PUBLIC explicit Autoware(Ts &&... xs)
  : rclcpp::Node("concealer", "simulation", rclcpp::NodeOptions().use_global_arguments(false)),
    process_id(ros2_launch(std::forward<decltype(xs)>(xs)...)),
    spinner(
      [this](auto future) {
        while (rclcpp::ok() and
               future.wait_for(std::chrono::milliseconds(1)) == std::future_status::timeout) {
          try {
            rclcpp::spin_some(get_node_base_interface());
          } catch (...) {
            thrown = std::current_exception();
          }
        }
        RCLCPP_INFO_STREAM(
          get_logger(),
          "\x1b[32mShutting down Autoware: (1/3) Stoped publlishing/subscribing.\x1b[0m");
      },
      std::move(promise.get_future())),
    updater(create_wall_timer(std::chrono::milliseconds(5), [this]() { return update(); }))
  {
#ifdef AUTOWARE_ARCHITECTURE_PROPOSAL
    // Lane change is not implemented in Autoware.Auto
    setLaneChangeApproval();
#endif
  }

  virtual ~Autoware();

  void engage();

  void initialize(const geometry_msgs::msg::Pose &);

  auto initialized() const noexcept { return initialize_was_called; }

  auto lock() const { return std::unique_lock<std::mutex>(mutex); }

  void plan(const std::vector<geometry_msgs::msg::PoseStamped> &);

  void rethrow() const noexcept(false);

  bool ready() const noexcept(false);

  auto set(const geometry_msgs::msg::Pose & pose) -> const auto & { return current_pose = pose; }

  auto set(const geometry_msgs::msg::Twist & twist) -> const auto &
  {
    return current_twist = twist;
  }

  void update();
};
}  // namespace concealer

#include <concealer/undefine_macro.hpp>

#endif  // CONCEALER__AUTOWARE_HPP_
