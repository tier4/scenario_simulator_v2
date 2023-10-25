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

#ifndef MAP_FRAGMENT__PRINTER_HPP__
#define MAP_FRAGMENT__PRINTER_HPP__

#include <random>

namespace map_fragment
{
template <typename Node>
auto loadBasicPrinter(const Node & node)
{
  const auto select =
    node.has_parameter("select") ? node.get_parameter("select").as_string() : "any";

  return [select](auto && results) -> decltype(auto) {
    if (1 < results.size()) {
      auto sort = [&]() {
        std::sort(results.begin(), results.end(), [](const auto & a, const auto & b) {
          return a.id() < b.id();
        });
      };

      auto shuffle = [&]() {
        auto device = std::random_device();
        auto engine = std::mt19937(device());
        std::shuffle(results.begin(), results.end(), engine);
      };

      auto all = [&]() {
        for (const auto & result : results) {
          std::cout << result.id() << std::endl;
        }
      };

      auto first = [&]() { std::cout << results.front().id() << std::endl; };

      if (select == "all") {
        sort();
        all();
      } else if (select == "any") {
        shuffle();
        first();
      } else if (select == "first") {
        sort();
        first();
      } else {
        std::stringstream what;
        what << "There are " << results.size() << " candidates that satisfy the constraints.";
        throw std::runtime_error(what.str());
      }
    } else if (0 < results.size()) {
      std::cout << results.front().id() << std::endl;
    } else {
      throw std::runtime_error("There is no candidate that satisfies the constraints.");
    }
  };
}
}  // namespace map_fragment

#endif  // MAP_FRAGMENT__PRINTER_HPP__
