// Copyright 2015-2020 Autoware Foundation. All rights reserved.
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

#include <awapi_accessor/accessor.hpp>

#include <cstdlib>
#include <memory>

class Example
  : public rclcpp::Node, private autoware_api::Accessor
{
  std::shared_ptr<rclcpp::TimerBase> timer;

public:
  explicit Example(const rclcpp::NodeOptions & options)
  : rclcpp::Node("awapi_accessor_example", options),
    autoware_api::Accessor(static_cast<rclcpp::Node &>(*this)),
    timer(
      create_wall_timer(
        std::chrono::seconds(1),
        [this]()
        {
          std::cout << ">>> TIMER CALLBACK!" << std::endl;

          {
            std_msgs::msg::Bool message {};
            message.data = true;
            setAutowareEngage(message);
          }

          {
            autoware_planning_msgs::msg::Route message {};
            setAutowareRoute(message);
          }

          {
            std_msgs::msg::Bool message {};
            setLaneChangeApproval(message);
          }

          {
            std_msgs::msg::Bool message {};
            setLaneChangeForce(message);
          }

          {
            autoware_perception_msgs::msg::TrafficLightStateArray message {};
            setTrafficLightStateArray(message);
          }

          {
            std_msgs::msg::Float32 message {};
            setVehicleVelocity(message);
          }

          {
            autoware_api_msgs::msg::AwapiAutowareStatus current {
              getAutowareStatus()
            };
          }

          {
            autoware_perception_msgs::msg::TrafficLightStateArray current {
              getTrafficLightStatus()
            };
          }

          {
            autoware_api_msgs::msg::AwapiVehicleStatus current {
              getVehicleStatus()
            };
          }

          {
            static auto value = 0;
            std_msgs::msg::String message {};
            message.data = "loop " + std::to_string(++value);
            setDebugString(message);
          }

          {
            std_msgs::msg::String current {
              getDebugString()
            };

            std::cout << current.data << std::endl;
          }

          std::cout << "<<< TIMER CALLBACK!" << std::endl;
        }))
  {}
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor executor {};

  rclcpp::NodeOptions options {};

  auto example {
    std::make_shared<Example>(options)
  };

  executor.add_node(example);

  executor.spin();

  rclcpp::shutdown();

  return EXIT_SUCCESS;
}
