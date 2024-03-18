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

/**
 * @mainpage ROS 2 visualization node for OpenSCENARIO entities
 * @image html images/rviz.png width=1280px height=540px
 * @author Masaya Kataoka
 * @date 2020-11-19
 * @section interface
  <table>
    <caption id="multi_row">ROS 2 Topic interface</caption>
    <tr>
      <th>Name</th>
      <th>Type</th>
      <th>Pub/Sub</th>
      <th>description</th>
    </tr>
    <tr>
      <td>/entity/marker</td>
      <td>visualization_msgs/msg/MarkerArray</td>
      <td>Publish</td>
      <td>Visualization results of the marker.</td>
    </tr>
    <tr>
      <td>/entity/status</td>
      <td>traffic_simulator_msgs/msg/EntityStatusArray</td>
      <td>Subscribe</td>
      <td>Topics for publishing entity status in simulation.</td>
    </tr>
  </table>
 */

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <traffic_simulator/visualization/visualization_component.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto component = std::make_shared<traffic_simulator::VisualizationComponent>(options);
  rclcpp::spin(component->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
