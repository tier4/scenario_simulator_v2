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

#include <simple_sensor_simulator/exception.hpp>
#include <simple_sensor_simulator/sensor_simulation/noise_generator/noise_generator.hpp>
#include <simulation_interface/conversions.hpp>

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

namespace simple_sensor_simulator
{

template <>
void NoiseGenerator<autoware_auto_perception_msgs::msg::DetectedObjects>::publish_with_noise(
  const autoware_auto_perception_msgs::msg::DetectedObjects msg)
{
  std::random_device seed;
  std::shared_ptr<std::mt19937> rand_engine = std::make_shared<std::mt19937>(seed());
  double & pos_noise_stddev = configuration_.pos_noise_stddev;
  std::shared_ptr<std::normal_distribution<>> pos_noise_dist =
    std::make_shared<std::normal_distribution<>>(0.0, pos_noise_stddev);
  auto detected_objects = msg;
  autoware_auto_perception_msgs::msg::DetectedObjects detected_objects_with_noise;
  for (auto & object : detected_objects.objects) {
    object.kinematics.pose_with_covariance.pose.position.x += (*pos_noise_dist)(*rand_engine);
    object.kinematics.pose_with_covariance.pose.position.y += (*pos_noise_dist)(*rand_engine);
  }
  detected_objects_with_noise.header = detected_objects.header;
  detected_objects_with_noise.objects = detected_objects.objects;
  publisher_ptr_->publish(detected_objects_with_noise);
}
}  // namespace simple_sensor_simulator
