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

#include <rclcpp/rclcpp.hpp>
#include <traffic_simulator/job/job_list.hpp>

namespace traffic_simulator
{
namespace job
{
void JobList::append(
  const std::function<bool(double)> & func_on_update, const std::function<void()> & func_on_cleanup,
  job::Type type, bool exclusive, const job::Event event)
{
  for (auto & job : list_) {
    if (job.type == type && job.exclusive == exclusive && job.getStatus() == job::Status::ACTIVE) {
      job.inactivate();
    }
  }
  list_.emplace_back(Job(func_on_update, func_on_cleanup, type, exclusive, event));
}

void JobList::update(const double step_time, const job::Event event)
{
  // show numper of jobs in the list
  RCLCPP_ERROR_STREAM(rclcpp::get_logger("JobList"), "job list size: " << list_.size());

  for (auto & job : list_) {
    if (job.event == event) {
      job.onUpdate(step_time);
    }
  }
}
}  // namespace job
}  // namespace traffic_simulator
