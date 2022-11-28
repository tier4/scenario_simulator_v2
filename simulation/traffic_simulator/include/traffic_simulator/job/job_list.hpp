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

#ifndef TRAFFIC_SIMULATOR__JOB__JOB_LIST_HPP_
#define TRAFFIC_SIMULATOR__JOB__JOB_LIST_HPP_

#include <traffic_simulator/job/job.hpp>
#include <vector>

namespace traffic_simulator
{
namespace job
{
class JobList
{
public:
  void append(
    const std::function<bool(const double)> & func_on_update,
    const std::function<void()> & func_on_cleanup, job::Type type, bool exclusive,
    const job::Event event = Event::POST_UPDATE);
  void update(const double step_time, const job::Event event);

private:
  std::vector<Job> list_;
};
}  // namespace job
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__JOB__JOB_LIST_HPP_
