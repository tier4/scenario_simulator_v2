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

#include <deque>
#include <memory>
#include <openscenario_interpreter/syntax/open_scenario.hpp>
#include <openscenario_utility/xml_validator.hpp>

namespace openscenario_preprocessor
{
struct ScenarioSet
{
  ScenarioSet() = default;

  explicit ScenarioSet(std::string path, int expect, float frame_rate)
  : path(path), expect(expect), frame_rate(frame_rate)
  {
  }

  std::string path;

  int expect;

  float frame_rate;
};

class Preprocessor
{
public:
  explicit Preprocessor() : xml_validator("") {}

protected:
  void preprocessScenario(ScenarioSet &);

  [[nodiscard]] bool validateXOSC(
    const boost::filesystem::path & target_file, const boost::filesystem::path & xsd_file,
    bool verbose = false);

  std::deque<ScenarioSet> preprocessed_scenarios;

  std::mutex preprocessed_scenarios_mutex;

  openscenario_utility::XMLValidator xml_validator;
};

}  // namespace openscenario_preprocessor

#endif  // OPENSCENARIO_PREPROCESSOR__OPENSCENARIO_PREPROCESSOR_HPP_
