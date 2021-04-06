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

#ifndef TRAFFIC_SIMULATOR__HELPER__STOP_WATCH_HPP_
#define TRAFFIC_SIMULATOR__HELPER__STOP_WATCH_HPP_

#include <boost/optional.hpp>
#include <chrono>
#include <string>
#include <iostream>

namespace traffic_simulator
{
namespace helper
{
template<typename T>
class StopWatch
{
public:
  explicit StopWatch(const std::string & name)
  : name(name) {}
  void start()
  {
    end_time = boost::none;
    start_time = std::chrono::system_clock::now();
  }
  void stop()
  {
    end_time = std::chrono::system_clock::now();
  }
  void print()
  {
    if (start_time && end_time) {
      double elapsed = std::chrono::duration_cast<T>(end_time.get() - start_time.get()).count();
      if (typeid(T) == typeid(std::chrono::microseconds)) {
        std::cout << "elapsed time in stop watch " << name << " : " << elapsed << " microseconds" <<
          std::endl;
      }
      if (typeid(T) == typeid(std::chrono::milliseconds)) {
        std::cout << "elapsed time in stop watch " << name << " : " << elapsed << " milliseconds" <<
          std::endl;
      }
      if (typeid(T) == typeid(std::chrono::seconds)) {
        std::cout << "elapsed time in stop watch " << name << " : " << elapsed << " seconds" <<
          std::endl;
      }
    }
  }

private:
  const std::string name;
  boost::optional<std::chrono::system_clock::time_point> start_time, end_time;
};
}  // namespace helper
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__HELPER__STOP_WATCH_HPP_
