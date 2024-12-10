// Copyright 2015 TIER IV, Inc. All rights reserved.
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

#include <autoware_system_msgs/msg/autoware_state.hpp>
#include <boost/lexical_cast.hpp>
#include <rclcpp/rclcpp.hpp>

#if __has_include(<tier4_simulation_msgs/msg/user_defined_value.hpp>)
#include <tier4_simulation_msgs/msg/user_defined_value.hpp>
#endif

int main(const int argc, char const * const * const argv)
{
  /* ---- NOTE -----------------------------------------------------------------
   *
   *  The scenario simulator can be started by the following command
   *
   *      ros2 launch scenario_test_runner scenario_test_runner.launch.py \
   *        scenario:=</path/to/scenario.yaml> \
   *        initialize_duration:=<unsigned integer (default: 30 [sec])>
   *
   *  The launched scenario simulator will setup the simulation with the
   *  following sequence.
   *
   *  (1) Load the given <scenario.yaml>.
   *
   *    (1a) At this time, if the scenario defines an entity of Vehicle type
   *         with `isEgo: true` specified, Autoware is launched
   *         (planning_simulator.launch.xml by default).
   *
   *    (1b) At this time, if `UserDefinedValueCondition` is defined in the
   *         scenario, it starts to subscribe to the topic name specified in
   *         `UserDefinedValueCondition.name`. The scenario simulator does not
   *         care about the activation of the node (the file you are looking at
   *         right now) that publishes the message corresponding to the topic
   *         name.
   *
   *  (2) Send initial coordinates, destination coordinates, etc. to Autoware
   *      sequentially according to the scenario definition.
   *
   *    (2a) At this time, the simulation time starts from minus
   *         `initialize_duration` seconds (t = -30 [sec] by default).
   *
   *    (2b) Here, the scenario simulator is hard-coded with the correct state
   *         transitions for Autoware. The simulation time starts from -30 sec
   *         is the grace time for the initialization of Autoware. If Autoware
   *         has not reached the WaitingForRoute state at simulation time zero,
   *         the scenario is terminated immediately as a failure.
   *
   *  (3) Send an engagement to `Autoware` to start the simulation.
   *
   *  The important thing to note in the above sequence is that "this node
   *  starts working well before the simulation starts". This behavior is
   *  especially problematic when you do your own time management within this
   *  node (using std::chrono, etc.). For example, if this node is supposed to
   *  take some specific action at 100 seconds after startup, it will actually
   *  take place roughly at simulation time 100 - initialize_duration seconds.
   *  Therefore, this node should not use absolute time as a trigger for its
   *  actions. A possible workaround is for this node to subscribe to its own
   *  `AutowareState` and manage its time starting from the point when the
   *  `AutowareState` transitions to Driving.
   *
   *  In short, this node should be purely dedicated to eavesdropping on
   *  Autoware topics and sending higher-order information computed from them
   *  to the simulator.
   *
   * ------------------------------------------------------------------------ */

  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("count_up");

#if __has_include(<tier4_simulation_msgs/msg/user_defined_value.hpp>)
  using tier4_simulation_msgs::msg::UserDefinedValue;
  using tier4_simulation_msgs::msg::UserDefinedValueType;

  autoware_system_msgs::msg::AutowareState status;

  auto subscription = node->create_subscription<autoware_system_msgs::msg::AutowareState>(
    "/autoware/state", rclcpp::QoS(1).reliable(),
    [&](const autoware_system_msgs::msg::AutowareState::SharedPtr message) { status = *message; });

  auto publisher = node->create_publisher<UserDefinedValue>("/timeout", rclcpp::QoS(1).reliable());

  auto make_message = [&](const auto & status) mutable  //
  {
    static auto duration_since_autoware_engaged = std::chrono::high_resolution_clock::now();

    if (status.state != autoware_system_msgs::msg::AutowareState::DRIVING) {
      duration_since_autoware_engaged = std::chrono::high_resolution_clock::now();
    }

    UserDefinedValue message;
    {
      message.type.data = UserDefinedValueType::BOOLEAN;
      message.value =
        (10 < std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::high_resolution_clock::now() - duration_since_autoware_engaged)
                .count())
          ? "true"
          : "false";
    }

    std::cout << "message.value = " << message.value << std::endl;

    return message;
  };

  auto timer = node->create_wall_timer(
    std::chrono::milliseconds(100), [&]() { publisher->publish(make_message(status)); });
#else
  std::cout
    << "The ability to have ROS 2 topics as values for `UserDefinedValueCondition` is enabled only "
       "when the `UserDefinedValue` type is present in the `tier4_simulation_msgs` package."
    << std::endl;
#endif

  rclcpp::executors::SingleThreadedExecutor executor;

  executor.add_node(node);

  executor.spin();

  return rclcpp::shutdown() ? EXIT_SUCCESS : EXIT_FAILURE;
}
