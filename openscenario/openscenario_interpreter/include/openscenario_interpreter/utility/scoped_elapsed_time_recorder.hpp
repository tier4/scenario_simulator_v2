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

#ifndef OPENSCENARIO_INTERPRETER__UTILITY__SCOPED_ELAPSED_TIME_RECORDER_HPP_
#define OPENSCENARIO_INTERPRETER__UTILITY__SCOPED_ELAPSED_TIME_RECORDER_HPP_

#include <chrono>
#include <cmath>

template <typename TClock = std::chrono::high_resolution_clock>
class ScopedElapsedTimeRecorder
{
public:
  explicit ScopedElapsedTimeRecorder(double & output_seconds) : output_seconds(output_seconds) {}

  ~ScopedElapsedTimeRecorder()
  {
    output_seconds = std::abs(std::chrono::duration<double>(TClock::now() - start).count());
  }

private:
  std::chrono::time_point<TClock> start = TClock::now();

  double & output_seconds;
};

#endif  // OPENSCENARIO_INTERPRETER__UTILITY__SCOPED_ELAPSED_TIME_RECORDER_HPP_
