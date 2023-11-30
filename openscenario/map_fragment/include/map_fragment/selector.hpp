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
auto print(std::ostream & os, const lanelet::ConstLanelet & lanelet) -> auto &
{
  return os << lanelet.id();
}

auto print(std::ostream & os, const lanelet::Lanelets & lanelets) -> auto &
{
  for (auto && lanelet : lanelets) {
    print(os, lanelet) << std::endl;
  }
  return os;
}

auto print(std::ostream & os, const lanelet::ConstLanelets & lanelets) -> auto &
{
  for (auto && lanelet : lanelets) {
    print(os, lanelet) << std::endl;
  }
  return os;
}

auto print(std::ostream & os, const lanelet::routing::LaneletPath & path) -> auto &
{
  os << "[";
  for (auto && lanelet : path) {
    print(os, lanelet) << (&lanelet != &path.back() ? ", " : "]");
  }
  return os;
}

auto print(std::ostream & os, const lanelet::routing::LaneletPaths & paths) -> auto &
{
  for (auto && path : paths) {
    print(os, path) << std::endl;
  }
  return os;
}

inline auto default_print = [](auto && x) -> std::ostream & {
  return print(std::cout, std::forward<decltype(x)>(x)) << std::endl;
};

auto operator<(const lanelet::ConstLanelet & a, const lanelet::ConstLanelet & b)
{
  return a.id() < b.id();
}

auto operator<(const lanelet::routing::LaneletPath & a, const lanelet::routing::LaneletPath & b)
{
  return std::lexicographical_compare(
    a.begin(), a.end(), b.begin(), b.end(), [](auto && a, auto && b) { return a < b; });
}

inline auto less = [](auto && x, auto && y) { return x < y; };

template <
  typename Node, typename Receiver = decltype(default_print), typename Comparator = decltype(less)>
auto loadBasicSelector(
  const Node & node, const std::string & prefix = "", const std::string & suffix = "",
  Receiver receive = default_print, Comparator compare = less)
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
        auto engine = std::default_random_engine(device());
        std::shuffle(value_swappables.begin(), value_swappables.end(), engine);
      };

      if (select == "all") {
        if constexpr (std::is_const_v<decltype(results)>) {
          auto copy_of_results = results;
          sort(copy_of_results);
          receive(copy_of_results);
        } else {
          sort(results);
          receive(results);
        }
      } else if (select == "any") {
        if constexpr (std::is_const_v<decltype(results)>) {
          auto copy_of_results = results;
          shuffle(copy_of_results);
          receive(copy_of_results.front());
        } else {
          shuffle(results);
          receive(results.front());
        }
      } else if (select == "first") {
        if constexpr (std::is_const_v<decltype(results)>) {
          auto copy = results;
          sort(copy);
          receive(copy.front());
        } else {
          sort(results);
          receive(results.front());
        }
      } else if (select == "last") {
        if constexpr (std::is_const_v<decltype(results)>) {
          auto copy = results;
          sort(copy);
          receive(copy.back());
        } else {
          sort(results);
          receive(results.back());
        }
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
}  // namespace map_fragment

#endif  // MAP_FRAGMENT__SELECTOR_HPP__
