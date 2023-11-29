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

#include <filesystem>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <string>
#include <vector>

auto main(const int argc, char const * const * const argv) -> int
try {
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node(std::filesystem::path(argv[0]).stem());

  node.declare_parameter("input", std::vector<int>());
  node.declare_parameter("index", std::numeric_limits<int>::max());

  const auto integers = node.get_parameter("input").as_integer_array();

  if (const auto index = node.get_parameter("index").as_int();
      0 <= index and static_cast<std::size_t>(index) < integers.size()) {
    std::cout << integers[index] << std::endl;
  } else {
    std::stringstream what;
    what << "It is not possible to refer to index " << index << " of list [";
    for (auto && integer : integers) {
      what << integer << (&integer != &integers.back() ? ", " : "");
    }
    what << "] of size " << integers.size();
    throw std::out_of_range(what.str());
  }

  return EXIT_SUCCESS;
} catch (const std::exception & exception) {
  std::cerr << exception.what() << std::endl;
  return EXIT_FAILURE;
}
