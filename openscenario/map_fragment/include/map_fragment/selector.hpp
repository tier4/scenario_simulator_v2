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

#ifndef MAP_FRAGMENT__SELECTOR_HPP__
#define MAP_FRAGMENT__SELECTOR_HPP__

#include <algorithm>
#include <iostream>
#include <random>
#include <rclcpp/rclcpp.hpp>

namespace map_fragment
{
inline auto print = [](auto && x) -> std::ostream & {
  return std::cout << std::forward<decltype(x)>(x) << std::endl;
};

template <typename Node, typename Comparator = std::less<void>, typename Receiver = decltype(print)>
auto loadBasicSelector(
  const Node & node, const std::string & prefix = "", const std::string & suffix = "",
  Receiver receive = print, Comparator compare = {})
{
  const auto select = node.has_parameter(prefix + "select" + suffix)
                        ? node.get_parameter(prefix + "select" + suffix).as_string()
                        : "any";

  return [select, compare, receive](auto && results) -> decltype(auto) {
    if (1 < results.size()) {
      // cspell: ignore swappables

      auto sort = [&](auto && value_swappables) {
        std::sort(value_swappables.begin(), value_swappables.end(), compare);
      };

      auto shuffle = [&](auto && value_swappables) {
        auto device = std::random_device();
        auto engine = std::mt19937(device());
        std::shuffle(value_swappables.begin(), value_swappables.end(), engine);
      };

      auto receive_for_each_of = [&](auto && iterable) {
        for (auto && each : iterable) {
          receive(std::forward<decltype(each)>(each));
        }
      };

      auto receive_first_of = [&](auto && sequence) { receive(sequence.front()); };

      auto receive_last_of = [&](auto && sequence) { receive(sequence.back()); };

      if (select == "all") {
        if constexpr (std::is_const_v<decltype(results)>) {
          auto copy_of_results = results;
          sort(copy_of_results);
          receive_for_each_of(copy_of_results);
        } else {
          shuffle(results);
          receive_for_each_of(results);
        }
      } else if (select == "any") {
        if constexpr (std::is_const_v<decltype(results)>) {
          auto copy_of_results = results;
          shuffle(copy_of_results);
          receive_first_of(copy_of_results);
        } else {
          shuffle(results);
          receive_first_of(results);
        }
      } else if (select == "first") {
        receive_first_of(results);
      } else if (select == "last") {
        receive_last_of(results);
      } else {
        std::stringstream what;
        what << "There are " << results.size() << " candidates that satisfy the constraints.";
        throw std::runtime_error(what.str());
      }
    } else if (0 < results.size()) {
      receive(results.front());
    } else {
      throw std::runtime_error("There is no candidate that satisfies the constraints.");
    }
  };
}

inline auto compare_lanelet_id = [](const auto & a, const auto & b) { return a.id() < b.id(); };

inline auto print_lanelet_id = [](const auto & lanelet) -> std::ostream & {
  return std::cout << lanelet.id() << std::endl;
};

template <
  typename Node, typename Receiver = decltype(print_lanelet_id),
  typename Comparator = decltype(compare_lanelet_id)>
auto loadLaneletSelector(
  const Node & node, const std::string & prefix = "", const std::string & suffix = "",
  Receiver receive = print_lanelet_id, Comparator compare = compare_lanelet_id)
{
  return loadBasicSelector(node, prefix, suffix, receive, compare);
}

inline auto compare_lanelet_path = [](const auto & a, const auto & b) {
  return std::lexicographical_compare(a.begin(), a.end(), b.begin(), b.end(), compare_lanelet_id);
};

inline auto print_lanelet_path = [](const auto & lanelet_path) -> std::ostream & {
  for (auto && lanelet : lanelet_path) {
    std::cout << lanelet.id() << " ";
  }
  return std::cout << std::endl;
};

template <
  typename Node, typename Receiver = decltype(print_lanelet_path),
  typename Comparator = decltype(compare_lanelet_path)>
auto loadLaneletPathSelector(
  const Node & node, const std::string & prefix = "", const std::string & suffix = "",
  Receiver receive = print_lanelet_path, Comparator compare = compare_lanelet_path)
{
  return loadBasicSelector(node, prefix, suffix, receive, compare);
}
}  // namespace map_fragment

#endif  // MAP_FRAGMENT__SELECTOR_HPP__
