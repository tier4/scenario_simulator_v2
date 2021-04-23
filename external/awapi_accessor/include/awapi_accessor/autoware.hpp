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

#ifndef AWAPI_ACCESSOR__AUTOWARE_HPP_
#define AWAPI_ACCESSOR__AUTOWARE_HPP_

#define AUTOWARE_AUTO false
#define AUTOWARE_IV true

// #define AUTOWARE_CONCEALER_ISOLATE_STANDARD_OUTPUT

#include <sys/wait.h>

#include <awapi_accessor/continuous_transform_broadcaster.hpp>
#include <awapi_accessor/fundamental_api.hpp>
#include <awapi_accessor/launch.hpp>
#include <awapi_accessor/miscellaneous_api.hpp>
#include <awapi_accessor/task_queue.hpp>
#include <awapi_accessor/transition_assertion.hpp>
#include <awapi_accessor/utility/visibility.hpp>
#include <chrono>
#include <future>
#include <mutex>
#include <thread>
#include <utility>

namespace awapi
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

                 /* ---- NOTE --------------------------------------------------
                  *
                  *
                  * --------------------------------------------------------- */
                 public ContinuousTransformBroadcaster<Autoware>,

                 /* ---- NOTE --------------------------------------------------
                  *
                  *
                  * --------------------------------------------------------- */
                 public FundamentalAPI<Autoware>,

                 /* ---- NOTE --------------------------------------------------
                  *
                  *
                  * --------------------------------------------------------- */
                 public MiscellaneousAPI<Autoware>,

                 /* ---- NOTE --------------------------------------------------
                  *
                  *
                  * --------------------------------------------------------- */
                 public TransitionAssertion<Autoware>
{
  friend class ContinuousTransformBroadcaster<Autoware>;
  friend class FundamentalAPI<Autoware>;
  friend class MiscellaneousAPI<Autoware>;
  friend class TransitionAssertion<Autoware>;

  std::mutex mutex;

  decltype(auto) lock() { return std::unique_lock<std::mutex>(mutex); }

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

  /* ---- NOTE -----------------------------------------------------------------
   *
   *  This update function is responsible for mass updates of topics that must
   *  be continuously updated.
   *
   * ------------------------------------------------------------------------ */
  void update();

  TaskQueue task_queue;

public:
  template <std::size_t Rate = 5, typename... Ts>
  AWAPI_ACCESSOR_PUBLIC explicit Autoware(Ts &&... xs)
  : rclcpp::Node("awapi_accessor", "simulation", rclcpp::NodeOptions().use_global_arguments(false)),
    process_id(ros2_launch(std::forward<decltype(xs)>(xs)...)),
    spinner(
      [this](auto future) {
        while (rclcpp::ok() and
               future.wait_for(std::chrono::milliseconds(1)) == std::future_status::timeout) {
          rclcpp::spin_some(get_node_base_interface());
        }
      },
      std::move(promise.get_future())),
    updater(create_wall_timer(std::chrono::milliseconds(Rate), [this]() { return update(); }))
  {
  }

  virtual ~Autoware();

  auto readyToEngage() const { return task_queue.exhausted(); }

  const auto & set(const geometry_msgs::msg::Pose & pose) { return current_pose = pose; }

  const auto & set(const geometry_msgs::msg::Twist & twist) { return current_twist = twist; }

  void initialize(const geometry_msgs::msg::Pose &);

  void plan(const std::vector<geometry_msgs::msg::PoseStamped> &);

  void engage();
};
}  // namespace awapi

#include <awapi_accessor/undefine_macro.hpp>

#endif  // AWAPI_ACCESSOR__AUTOWARE_HPP_
