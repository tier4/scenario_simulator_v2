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

#ifndef SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__TRAFFIC_LIGHTS__TRAFFIC_LIGHTS_DETECTOR_HPP_
#define SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__TRAFFIC_LIGHTS__TRAFFIC_LIGHTS_DETECTOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <simple_sensor_simulator/sensor_simulation/traffic_lights/traffic_lights_publisher.hpp>
#include <simulation_interface/conversions.hpp>
#include <string>

namespace simple_sensor_simulator
{
namespace traffic_lights
{
/** @brief Implements traffic lights detector mechanism simulation
 * Currently it only allows to set traffic lights state and publish them on predefined topic
 * Future implementations might, for example, publish only traffic lights that are in a specific FoV
 * of a camera sensor Further refactoring would be required, however, to achieve this.
 */
class TrafficLightsDetector
{
public:
  template <typename NodeType>
  explicit TrafficLightsDetector(NodeType & node, const std::string & architecture_type)
  : publisher_ptr_(makePublisher(node, architecture_type))
  {
  }

  auto updateFrame(
    const rclcpp::Time & current_ros_time,
    const simulation_api_schema::UpdateTrafficLightsRequest & request) -> void
  {
    publisher_ptr_->publish(current_ros_time, request);
  }

private:
  template <typename NodeType>
  auto makePublisher(NodeType & node, const std::string & architecture_type)
    -> std::unique_ptr<TrafficLightsPublisherBase>
  {
    /*
       TrafficLightsDetector in SimpleSensorSimulator publishes using architecture-dependent topics:
       "/perception/traffic_light_recognition/internal/traffic_signals" for >= "awf/universe/20230906"
       "/perception/traffic_light_recognition/traffic_signals" for "awf/universe"

       V2ITrafficLights in TrafficSimulator publishes publishes using architecture-independent topics ("awf/universe..."): 
       "/v2x/traffic_signals" and "/perception/traffic_light_recognition/external/traffic_signals"
    */
    if (architecture_type == "awf/universe") {
      using Message = autoware_auto_perception_msgs::msg::TrafficSignalArray;
      return std::make_unique<TrafficLightsPublisher<Message>>(
        &node, "/perception/traffic_light_recognition/traffic_signals");
    } else if (architecture_type >= "awf/universe/20230906") {
      using Message = autoware_perception_msgs::msg::TrafficSignalArray;
      return std::make_unique<TrafficLightsPublisher<Message>>(
        &node, "/perception/traffic_light_recognition/internal/traffic_signals");
    } else {
      std::stringstream ss;
      ss << "Unexpected architecture_type " << std::quoted(architecture_type) << " given.";
      throw std::invalid_argument(ss.str());
    }
  }

  const std::unique_ptr<TrafficLightsPublisherBase> publisher_ptr_;
};
}  // namespace traffic_lights
}  // namespace simple_sensor_simulator

#endif  // SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__TRAFFIC_LIGHTS__TRAFFIC_LIGHTS_DETECTOR_HPP_
