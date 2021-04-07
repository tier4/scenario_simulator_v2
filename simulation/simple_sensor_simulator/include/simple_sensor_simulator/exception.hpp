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

#ifndef SIMPLE_SENSOR_SIMULATOR__EXCEPTION_HPP_
#define SIMPLE_SENSOR_SIMULATOR__EXCEPTION_HPP_

#include <stdexcept>

namespace simple_sensor_simulator
{
class SimulationRuntimeError : public std::runtime_error
{
public:
  explicit SimulationRuntimeError(const char * message) : runtime_error(message) {}

private:
};
}  // namespace simple_sensor_simulator

#endif  // SIMPLE_SENSOR_SIMULATOR__EXCEPTION_HPP_
