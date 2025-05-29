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
  status_ = Status::active;
}

void Job::inactivate()
{
  status_ = Status::inactive;
  func_on_cleanup_();
}

Status Job::getStatus() const { return status_; }

void Job::onUpdate(const double step_time)
{
  switch (status_) {
    case Status::active:
      if (func_on_update_(job_duration_)) {
        inactivate();
      }
      job_duration_ = job_duration_ + step_time;
      return;
    case Status::inactive:
      return;
  }
}
}  // namespace job
}  // namespace traffic_simulator
