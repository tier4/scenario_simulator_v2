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

#ifndef SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__MEASURE_TIME_HPP_
#define SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__MEASURE_TIME_HPP_

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <vector>
#include <numeric>

using namespace std::chrono;

namespace simple_sensor_simulator
{
class MeasureTime
{
public:
  MeasureTime() : is_started(false) {}
  ~MeasureTime() { getAverageMeasurements(); };

  void start()
  {
    start_time = steady_clock::now();
    is_started = true;
  }

  void stop()
  {
    if (!is_started)
    {
      std::cerr << "[MeasureTime] The clock cannot be stopped since it was not started before" << std::endl;
      return;
    }
    // loop_durations.push_back(duration<double>(duration_cast<nanoseconds>(steady_clock::now() - start_time)).count());
    // auto duration_chrono = duration<double>(duration_cast<seconds>(steady_clock::now() - start_time)).count();
    auto duration_std = duration<double, std::milli>(steady_clock::now() - start_time).count();
    loop_durations.push_back(duration_std);
    // std::cerr << "[MeasureTime] Measured duration_chrono: " << duration_chrono << std::endl;
    // std::cerr << "[MeasureTime] Measured duration_std: " << loop_durations.back() << "[ms]" << std::endl;
      std::cerr << "[MeasureTime] Measured average time: " << getAverageMeasurements() << "[ms]" << std::endl;
    is_started = false;
  }

  double getAverageMeasurements()
  {
    if (!loop_durations.empty())
    {
      auto average = std::accumulate(loop_durations.begin(), loop_durations.end(), 0.0) / loop_durations.size();
      // loop_durations.clear();
      return average;
    }
    return 0.0;
  }

private:
  std::vector<double> loop_durations;
  steady_clock::time_point start_time;
  bool is_started;
};

}  // namespace simple_sensor_simulator

#endif  // SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__MEASURE_TIME_HPP_
