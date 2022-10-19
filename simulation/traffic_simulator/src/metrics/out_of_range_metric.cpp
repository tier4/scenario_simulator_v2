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

#include <string>
#include <traffic_simulator/metrics/out_of_range_metric.hpp>

namespace metrics
{
void OutOfRangeMetric::setEntityManager(
  std::shared_ptr<traffic_simulator::entity::EntityManager> entity_manager_ptr)
{
  entity_manager_ptr_ = std::move(entity_manager_ptr);
  if (jerk_topic) {
    jerk_callback_ptr_ = entity_manager_ptr_->createSubscription<JerkMessageType>(
      *jerk_topic, rclcpp::SensorDataQoS(),
      [this](const JerkMessageType::ConstSharedPtr msg) { linear_jerk_ = msg->data; });
  }
}

void OutOfRangeMetric::update()
{
  const auto status = entity_manager_ptr_->getEntityStatus(target_entity);

  linear_velocity_ = status.action_status.twist.linear.x;
  linear_acceleration_ = status.action_status.accel.linear.x;

  if (!(min_velocity <= linear_velocity_ && linear_velocity_ <= max_velocity)) {
    failure(SPECIFICATION_VIOLATION(
      "current velocity (which is ", linear_velocity_, ") is out of range (which is [",
      min_velocity, ", ", max_velocity, "])"));
    return;
  }

  if (!(min_acceleration <= linear_acceleration_ && linear_acceleration_ <= max_acceleration)) {
    failure(SPECIFICATION_VIOLATION(
      "current acceleration (which is ", linear_acceleration_, ") is out of range (which is [",
      min_acceleration, ", ", max_acceleration, "])"));
    return;
  }

  if (!jerk_callback_ptr_) {
    const auto jerk_opt = entity_manager_ptr_->getLinearJerk(target_entity);
    if (jerk_opt) {
      linear_jerk_ = jerk_opt.get();
    }
  }

  if (!(min_jerk <= linear_jerk_ && linear_jerk_ <= max_jerk)) {
    failure(SPECIFICATION_VIOLATION(
      "current jerk (which is ", linear_jerk_, ") is out of range (which is [", min_jerk, ", ",
      max_jerk, "])"));
    return;
  }
}

nlohmann::json OutOfRangeMetric::toJson()
{
  nlohmann::json json = MetricBase::toBaseJson();
  if (getLifecycle() != MetricLifecycle::INACTIVE) {
    json["linear_velocity"] = linear_velocity_;
    json["linear_acceleration"] = linear_acceleration_;
    json["linear_jerk"] = linear_jerk_;
  }
  return json;
}
}  // namespace metrics
