// Copyright 2015-2020 Autoware Foundation. All rights reserved.
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

#ifndef SIMULATION_API__ENTITY__EXCEPTION_HPP_
#define SIMULATION_API__ENTITY__EXCEPTION_HPP_

#include <string>
#include <exception>

namespace simulation_api
{
class SimulationRuntimeError : public std::runtime_error
{
public:
  explicit SimulationRuntimeError(const char * message)
  : runtime_error(message) {}
  explicit SimulationRuntimeError(std::string message)
  : runtime_error(message.c_str()) {}
};

class SplineInterpolationError : public std::runtime_error
{
public:
  explicit SplineInterpolationError(const char * message)
  : runtime_error(message) {}
};
}  // namespace simulation_api


#endif  // SIMULATION_API__ENTITY__EXCEPTION_HPP_
