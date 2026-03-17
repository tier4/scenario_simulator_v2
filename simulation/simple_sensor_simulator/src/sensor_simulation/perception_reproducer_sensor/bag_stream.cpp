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

#include <autoware_perception_msgs/msg/detected_objects.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <simple_sensor_simulator/sensor_simulation/perception_reproducer_sensor/bag_stream.hpp>

namespace simple_sensor_simulator
{
inline namespace experimental
{

template <typename T>
auto BagStream<T>::pushMessage(double time_s, const std::shared_ptr<rcutils_uint8_array_t> & data)
  -> void
{
  auto message = this->deserialize(data);
  if (message.header.frame_id == "base_link") {
    message.header.frame_id = "replay_base_link";
  }
  this->data_.emplace_back(time_s, message);
}

template class BagStream<autoware_perception_msgs::msg::DetectedObjects>;
template class BagStream<autoware_planning_msgs::msg::Trajectory>;

}  // namespace experimental
}  // namespace simple_sensor_simulator
