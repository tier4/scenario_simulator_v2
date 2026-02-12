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

#include <boost/lexical_cast.hpp>
#include <random>
#include <rclcpp/rclcpp.hpp>

auto main(const int argc, char const * const * const argv) -> int
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("uniform_distribution");

  auto number = [&]() {
    auto engine = std::default_random_engine(std::random_device()());
    switch (argc) {
      case 3:
        return std::uniform_real_distribution<double>(
          boost::lexical_cast<double>(argv[1]), boost::lexical_cast<double>(argv[2]))(engine);
      default:
        return std::uniform_real_distribution<double>(0, 1)(engine);
    }
  };

  RCLCPP_INFO_STREAM(node->get_logger(), number());

  return EXIT_SUCCESS;
}
