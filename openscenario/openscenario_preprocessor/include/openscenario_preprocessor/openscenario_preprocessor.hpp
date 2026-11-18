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

#ifndef OPENSCENARIO_PREPROCESSOR__OPENSCENARIO_PREPROCESSOR_HPP_
#define OPENSCENARIO_PREPROCESSOR__OPENSCENARIO_PREPROCESSOR_HPP_

#include <filesystem>
#include <memory>
#include <openscenario_interpreter/syntax/open_scenario.hpp>
#include <openscenario_validator/validator.hpp>
#include <queue>

namespace openscenario_preprocessor
{
struct Scenario
{
  Scenario() = default;

  explicit Scenario(const std::filesystem::path & path, double frame_rate)
  : path(path), frame_rate(frame_rate)
  {
  }

  std::filesystem::path path;

  float frame_rate;
};

class Preprocessor
{
public:
  explicit Preprocessor(const std::filesystem::path & output_directory)
  : validate(), output_directory(output_directory)
  {
    if (not std::filesystem::exists(output_directory)) {
      std::filesystem::create_directories(output_directory);
    }
  }

protected:
  void preprocessScenario(const Scenario &);

  std::queue<Scenario> preprocessed_scenarios;

  std::mutex preprocessed_scenarios_mutex;

  openscenario_validator::OpenSCENARIOValidator validate;

  std::filesystem::path output_directory;
};
}  // namespace openscenario_preprocessor

#endif  // OPENSCENARIO_PREPROCESSOR__OPENSCENARIO_PREPROCESSOR_HPP_
