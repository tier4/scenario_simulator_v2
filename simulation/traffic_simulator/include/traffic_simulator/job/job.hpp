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

#ifndef TRAFFIC_SIMULATOR__JOB__JOB_HPP_
#define TRAFFIC_SIMULATOR__JOB__JOB_HPP_

#include <functional>
#include <memory>

namespace traffic_simulator
{
namespace job
{
enum class Type {
  UNKOWN = 0,
  LINEAR_VELOCITY = 1,
  LINEAR_ACCELERATION = 2,
  STAND_STILL_DURATION = 3,
  TRAVELED_DISTANCE = 4,
  OUT_OF_RANGE = 5
};

enum class Status {
  ACTIVE = 0,
  INACTIVE = 1,
};

enum class Event {
  PRE_UPDATE = 0,
  POST_UPDATE = 1,
};

class Job
{
public:
  /**
   * @brief Construct a new Job object
   *
   * @param func_on_update If func_on_update function returns true, runs func_on_update function.
   * @param func_on_cleanup If func_on_update function returns true, runs func_on_update function.
   * @param type Type of the Job
   * @param exclusive If true, the Job works exclusively by type.
   */
  Job(
    const std::function<bool(double)> & func_on_update,
    const std::function<void()> & func_on_cleanup, job::Type type, bool exclusive, Event event);
  void onUpdate(const double step_time);
  void inactivate();
  Status getStatus() const;

private:
  std::function<bool(double)> func_on_update_;
  std::function<void()> func_on_cleanup_;
  Status status_;
  double job_duration_;

public:
  const job::Type type;
  const bool exclusive;
  const Event event;
};
}  // namespace job
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__JOB__JOB_HPP_
