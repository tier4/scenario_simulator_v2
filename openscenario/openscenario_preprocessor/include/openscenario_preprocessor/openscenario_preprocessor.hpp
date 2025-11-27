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

#include <memory>
#include <openscenario_interpreter/syntax/open_scenario.hpp>
#include <openscenario_validator/validator.hpp>
#include <queue>

namespace openscenario_preprocessor
{
struct Scenario
{
  Scenario() = default;

  explicit Scenario(const boost::filesystem::path & path, double frame_rate)
  : path(path), frame_rate(frame_rate)
  {
  }

  boost::filesystem::path path;

  float frame_rate;
};

class Preprocessor
{
public:
  explicit Preprocessor(const boost::filesystem::path & output_directory)
  : validate(), output_directory(output_directory)
  {
    if (not boost::filesystem::exists(output_directory)) {
      boost::filesystem::create_directories(output_directory);
    }
  }

protected:
  void preprocessScenario(const Scenario &);

  std::queue<Scenario> preprocessed_scenarios;

  std::mutex preprocessed_scenarios_mutex;

  openscenario_validator::OpenSCENARIOValidator validate;

  boost::filesystem::path output_directory;
};
}  // namespace openscenario_preprocessor

#endif  // OPENSCENARIO_PREPROCESSOR__OPENSCENARIO_PREPROCESSOR_HPP_
