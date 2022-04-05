// Copyright 2015-2020 Tier IV, Inc. All rights reserved.
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
  const std::function<bool()> & func_condition, const std::function<void()> & func_execution,
  job::Type type, bool exclusive)
: func_condition_(func_condition), func_execution_(func_execution), type(type), exclusive(exclusive)
{
  status_ = Status::ACTIVE;
}

void Job::inactivate() { status_ = Status::INACTIVE; }

bool Job::checkCondition()
{
  if (func_condition_()) {
    inactivate();
    return true;
  }
  return false;
}

void Job::execute()
{
  switch (status_) {
    case Status::ACTIVE:
      func_execution_();
      return;
    case Status::INACTIVE:
      return;
  }
}
}  // namespace job
}  // namespace traffic_simulator
