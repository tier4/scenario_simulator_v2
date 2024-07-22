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
//
// Co-developed by TIER IV, Inc. and Robotec.AI sp. z o.o.

#ifndef RANDOM_TEST_RUNNER__ALMOST_STANDSTILL_METRIC_H
#define RANDOM_TEST_RUNNER__ALMOST_STANDSTILL_METRIC_H

#include <iostream>
#include <optional>

#include "random_test_runner/data_types.hpp"

class AlmostStandstillMetric
{
public:
  bool isAlmostStandingStill(const traffic_simulator::CanonicalizedEntityStatus & status)
  {
    if (!last_status_) {
      last_status_.emplace(status);
      return false;
    }

    if (status.getTime() - last_status_->getTime() > last_data_timeout_) {
      last_status_.reset();
      return false;
    }

    if (status.getTwist().linear.x < linear_velocity_threshold_) {
      almost_standstill_time_ += status.getTime() - last_status_->getTime();
    } else {
      almost_standstill_time_ = 0.0;
    }

    last_status_->set(status);

    if (almost_standstill_time_ > almost_standstill_timeout_) {
      return true;
    }
    return false;
  }

private:
  const double linear_velocity_threshold_ = 0.01;
  const double almost_standstill_timeout_ = 10.0;

  const double last_data_timeout_ = 2.0;

  double almost_standstill_time_ = 0.0;
  std::optional<traffic_simulator::CanonicalizedEntityStatus> last_status_;
};

#endif  // RANDOM_TEST_RUNNER__ALMOST_STANDSTILL_METRIC_H
