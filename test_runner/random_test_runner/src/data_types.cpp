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

#include "random_test_runner/data_types.hpp"

SimulatorType simulatorTypeFromString(const std::string & simulator_type_str)
{
  if (simulator_type_str == "simple_sensor_simulator") {
    return SimulatorType::SIMPLE_SENSOR_SIMULATOR;
  } else if (simulator_type_str == "awsim") {
    return SimulatorType::AWSIM;
  }
  throw std::runtime_error(
    fmt::format("Failed to convert {} to simulation type", simulator_type_str));
}

ArchitectureType architectureTypeFromString(const std::string & architecture_type_str)
{
  if (architecture_type_str == "awf/auto") {
    return ArchitectureType::AWF_AUTO;
  } else if (architecture_type_str.find("awf/universe") != std::string::npos) {
    return ArchitectureType::AWF_UNIVERSE;
  } else if (architecture_type_str == "tier4/proposal") {
    return ArchitectureType::TIER4_PROPOSAL;
  }
  throw std::runtime_error(
    fmt::format("Failed to convert {} to architecture type", architecture_type_str));
}

std::string stringFromArchitectureType(const ArchitectureType architecture_type)
{
  switch (architecture_type) {
    case ArchitectureType::AWF_AUTO:
      return "awf/auto";
    case ArchitectureType::AWF_UNIVERSE:
      return "awf/universe";
    case ArchitectureType::TIER4_PROPOSAL:
      return "tier4/proposal";
    default:
      throw std::runtime_error(fmt::format("Unknown ArchitectureType {}.", architecture_type));
  }
}
