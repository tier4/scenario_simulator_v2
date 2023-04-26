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

#include <boost/program_options.hpp>
#include <openscenario_preprocessor/openscenario_preprocessor.hpp>

int main(const int argc, char const * const * const argv)
{
  using namespace boost::program_options;

  options_description description("openscenario_preprocessor_command");

  //  -o <directory> -p param.json -s scenario.yaml
  description.add_options()(
    "output-directory,o", value<std::string>()->default_value("/tmp/openscenario_preprocessor"),
    "path of output directory")(
    "parameters,p", value<std::string>()->default_value(""), "parameters in json format")(
    "scenario,s", value<std::string>(), "path of scenario file")("help,H", "help");

  std::cout << "setup description" << std::endl;

  variables_map vm;
  store(parse_command_line(argc, argv, description), vm);
  notify(vm);

  std::cout << "setup variables map" << std::endl;

  auto output_directory_option = boost::filesystem::path(vm["output-directory"].as<std::string>());
  auto parameters_option = boost::filesystem::path(vm["parameters"].as<std::string>());
  auto scenario_option = boost::filesystem::path(vm["scenario"].as<std::string>());

  std::cout << "get options" << std::endl;

  openscenario_preprocessor::Preprocessor preprocessor(output_directory_option);

  std::cout << "create preprocessor" << std::endl;

  openscenario_preprocessor::Scenario scenario;
  scenario.path = scenario_option;
  scenario.expect = 1;
  scenario.frame_rate = 30.0;

  preprocessor.preprocessScenario(scenario);

  std::cout << "preprocess scenario" << std::endl;

  return 0;
}
