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
  const std::function<bool()> & func_on_update, const std::function<void()> & func_on_cleanup,
  job::Type type, bool exclusive)
: func_on_update_(func_on_update),
  func_on_cleanup_(func_on_cleanup),
  type(type),
  exclusive(exclusive)
{
  status_ = Status::ACTIVE;
}

void Job::inactivate()
{
  status_ = Status::INACTIVE;
  func_on_cleanup_();
}

void Job::onUpdate()
{
  switch (status_) {
    case Status::ACTIVE:
      if (func_on_update_()) {
        inactivate();
      }
      return;
    case Status::INACTIVE:
      return;
  }
}
}  // namespace job
}  // namespace traffic_simulator
