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

#include "gtest/gtest.h"

#include <boost/lexical_cast.hpp>

auto dollar(const std::string & command) -> std::string
{
  std::array<char, 128> buffer;

  std::string result;

  std::unique_ptr<FILE, decltype(&pclose)> pipe{::popen(command.c_str(), "r"), pclose};

  if (not pipe) {
    throw std::system_error(errno, std::system_category());
  } else {
    while (std::fgets(buffer.data(), buffer.size(), pipe.get())) {
      result += buffer.data();
    }
    return result;
  }
}

TEST(getYawFromQuaternion, returnValue)
{
  std::stringstream preprocessor_command;
  preprocessor_command << "ros2 run openscenario_preprocessor openscenario_preprocessor_command "
                          "-s /home/hans/Documents/sample.yaml "
                          "--parameters '{\"random_offset\": true}' "
                          "-f t4v2 -o /home/hans/Documents/cmd "
                          "--skip-full-derivation";
  dollar(preprocessor_command.str());

  std::stringstream diff_command;
  diff_command << " diff "
               << "-bB "
               << "/tmp/scenario_test_runner/0.0/0.0_0.xosc"
               << " /tmp/openscenario_preprocessor/normalized.xosc"
               << " | sed '/^[^>].*/d' | wc -l";
  std::string diff_result = dollar(diff_command.str());

  EXPECT_EQ(1, boost::lexical_cast<int>(diff_result));
}

auto main(int argc, char ** argv) -> int
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
