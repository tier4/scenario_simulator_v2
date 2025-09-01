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

#ifndef CONCEALER__LAUNCH_HPP_
#define CONCEALER__LAUNCH_HPP_

#include <boost/algorithm/string.hpp>
#include <concealer/execute.hpp>
#include <cstdlib>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <system_error>
#include <type_traits>
#include <vector>

namespace concealer
{
template <typename Parameters>
auto ros2_launch(
  const std::string & package, const std::string & file, const Parameters & parameters)
{
  const auto argv = [&]() {
    auto argv = std::vector<std::string>();

    argv.push_back("python3");
    argv.push_back(boost::algorithm::replace_all_copy(dollar("which ros2"), "\n", ""));
    argv.push_back("launch");
    argv.push_back(package);
    argv.push_back(file);

    for (const auto & parameter : parameters) {
      argv.push_back(parameter);
    }

    return argv;
  }();

  if (const auto process_id = fork(); process_id < 0) {
    throw std::system_error(errno, std::system_category());
  } else if (process_id == 0 and execute(argv) < 0) {
    std::cout << std::system_error(errno, std::system_category()).what() << std::endl;
    std::exit(EXIT_FAILURE);
  } else {
    RCLCPP_WARN(
      rclcpp::get_logger("DEBUG/concealer::ros2_launch"), "Autoware launched pid=%d", process_id);
    return process_id;
  }
}
}  // namespace concealer

#endif  // CONCEALER__LAUNCH_HPP_
