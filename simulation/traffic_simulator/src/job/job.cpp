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

#include <traffic_simulator/job/job.hpp>
#include <rclcpp/rclcpp.hpp>

namespace traffic_simulator
{
namespace job
{
Job::Job(
  const std::function<bool(const double)> & func_on_update,
  const std::function<void()> & func_on_cleanup, job::Type type, bool exclusive, job::Event event)
: func_on_update_(func_on_update),
  func_on_cleanup_(func_on_cleanup),
  job_duration_(0.0),
  type(type),
  exclusive(exclusive),
  event(event)
{
  status_ = Status::ACTIVE;
}

void Job::inactivate()
{
  status_ = Status::INACTIVE;
  RCLCPP_ERROR_STREAM(rclcpp::get_logger("JobList"), "inactivate start");
  if(!func_on_cleanup_){
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("JobList"), "func_on_cleanup_ is null");
    return;
  }
  func_on_cleanup_();
  RCLCPP_ERROR_STREAM(rclcpp::get_logger("JobList"), "done inactivate");
}

Status Job::getStatus() const { return status_; }

void Job::onUpdate(const double step_time)
{
  RCLCPP_ERROR_STREAM(rclcpp::get_logger("JobList"), "onupdate job type: " << (int)type);
  RCLCPP_ERROR_STREAM(rclcpp::get_logger("JobList"), "onupdate job status: " << (int)status_);
  switch (status_) {
    case Status::ACTIVE:
      if (func_on_update_(job_duration_)) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("JobList"), "inactivate");
        inactivate();
      }
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("JobList"), "victoiry");
      job_duration_ = job_duration_ + step_time;
      return;
    case Status::INACTIVE:
      return;
  }
}
}  // namespace job
}  // namespace traffic_simulator
