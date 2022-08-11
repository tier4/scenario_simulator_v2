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

#include <traffic_simulator/job/job_list.hpp>

namespace traffic_simulator
{
namespace job
{
void JobList::append(
  const std::function<bool()> & func_on_update, const std::function<void()> & func_on_cleanup,
  job::Type type, bool exclusive, job::Trigger trigger)
{
  for (auto & job : list_) {
    if (exclusive) {
      if (job.type == type && job.exclusive) {
        job.inactivate();
      }
    }
  }
  list_.emplace_back(Job(func_on_update, func_on_cleanup, type, exclusive, trigger));
}

void JobList::update()
{
  for (auto & job : list_) {
    if (job.trigger == job::Trigger::ON_UPDATE) {
      job.update();
    }
  }
}

void JobList::measure()
{
  for (auto & job : list_) {
    if (job.trigger == job::Trigger::ON_MEASURE) {
      job.update();
    }
  }
}
}  // namespace job
}  // namespace traffic_simulator
