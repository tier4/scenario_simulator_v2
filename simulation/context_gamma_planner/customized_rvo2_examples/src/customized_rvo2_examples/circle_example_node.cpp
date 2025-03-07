// Copyright 2021 Tier IV, Inc All rights reserved.
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

#include <memory>

#include "customized_rvo2_examples/example_scenarios.hpp"
#include "customized_rvo2_examples/rvo2_ros2.hpp"
#include "customized_rvo2_examples/scenario.hpp"
#include "rclcpp/rclcpp.hpp"

class CircleNode : public RVO2ROS2Component
{
public:
  explicit CircleNode(const rclcpp::NodeOptions & options) : RVO2ROS2Component(options, 1.0f)
  {
    RVO::Scenario scenario;
    setupCircleScenario(scenario);
    scenario.setupScenario(sim_);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto component = std::make_shared<CircleNode>(options);
  rclcpp::spin(component);
  rclcpp::shutdown();
  return 0;
}
