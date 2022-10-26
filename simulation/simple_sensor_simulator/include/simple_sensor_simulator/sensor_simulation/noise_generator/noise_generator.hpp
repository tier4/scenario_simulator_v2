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

#ifndef SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__NOISE_GENERATOR__NOISE_GENERATOR_HPP_
#define SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__NOISE_GENERATOR__NOISE_GENERATOR_HPP_

#include <simulation_api_schema.pb.h>

#include <autoware_auto_perception_msgs/msg/detected_objects.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
#include <random>

namespace simple_sensor_simulator
{
class NoiseGeneratorBase
{
protected:
  double last_update_stamp_;

  simulation_api_schema::NoiseGeneratorConfiguration configuration_; //TODO add NoiseGeneratorConfiguration

  NoiseGeneratorBase(
    const double last_update_stamp,
    const simulation_api_schema::NoiseGeneratorConfiguration & configuration)
  : last_update_stamp_(last_update_stamp), configuration_(configuration)
  {
  }

public:
  virtual ~NoiseGeneratorBase() = default;
};

template <typename T>
class NoiseGenerator : public NoiseGeneratorBase
{
  const typename rclcpp::Publisher<T>::SharedPtr publisher_ptr_;

public:
  explicit NoiseGenerator(
    const double current_time,
    const simulation_api_schema::NoiseGeneratorConfiguration & configuration,
    const typename rclcpp::Publisher<T>::SharedPtr & publisher)
  : NoiseGeneratorBase(current_time, configuration), publisher_ptr_(publisher)
  {
  }

  auto publish_with_noise(T);

template <>
void NoiseGenerator<autoware_auto_perception_msgs::msg::DetectedObjects>::publish_with_noise(
  autoware_auto_perception_msgs::msg::DetectedObjects msg);
}  // namespace simple_sensor_simulator

#endif  // SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__NOISE_GENERATOR__NOISE_GENERATOR_HPP_
