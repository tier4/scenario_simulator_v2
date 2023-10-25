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

#include <map_fragment/filter.hpp>
#include <map_fragment/map_fragment.hpp>
#include <random>
#include <rclcpp/rclcpp.hpp>

auto main(const int argc, char const * const * const argv) -> int
try {
  using namespace map_fragment;

  rclcpp::init(argc, argv);

  auto node = rclcpp::Node(std::filesystem::path(argv[0]).stem());

  const auto input_directory = [&]() {
    node.declare_parameter("input_directory", default_value::directory());
    return std::filesystem::path(node.get_parameter("input_directory").as_string());
  }();

  const auto lanelet2_map = [&]() {
    node.declare_parameter("lanelet2_map", input_directory / "lanelet2_map.osm");
    return node.get_parameter("lanelet2_map").as_string();
  }();

  const auto map = lanelet::load(lanelet2_map, map_fragment::projector());

  const auto routing_graph = lanelet::routing::RoutingGraph::build(*map, vehicleTrafficRules());

  auto is_leftmost = [&](const auto & lanelet) {
    return routing_graph->leftRelations(lanelet).empty();
  };

  auto is_rightmost = [&](const auto & lanelet) {
    auto right_relations = routing_graph->rightRelations(lanelet);
    return right_relations.empty() or
           std::any_of(
             right_relations.begin(), right_relations.end(), [&](const auto & right_relation) {
               /*
                  NOTE: The Lanelet returned by rightRelations is not
                  useful because the directions are aligned regardless of
                  the original orientation.
               */
               return lanelet.rightBound() ==
                      map->laneletLayer.find(right_relation.lanelet.id())->rightBound().invert();
             });
  };

  const std::list<std::function<bool(const lanelet::Lanelet &)>> constraints{
    [&]() {
      node.declare_parameter("type", "lanelet");
      const auto type = node.get_parameter("type").as_string();
      return [type](const auto & lanelet) {
        auto discard = [&]() {
          std::cerr << "[" << lanelet.id() << "] discarded: The type is not " << type << "."
                    << std::endl;
          return false;
        };
        return lanelet.attribute("type") == type or discard();
      };
    }(),

    [&]() {
      node.declare_parameter("subtype", "road");
      const auto subtype = node.get_parameter("subtype").as_string();
      return [subtype](const auto & lanelet) {
        auto discard = [&]() {
          std::cerr << "[" << lanelet.id() << "] discarded: The subtype is not " << subtype << "."
                    << std::endl;
          return false;
        };
        return lanelet.attribute("subtype") == subtype or discard();
      };
    }(),

    [&]() {
      node.declare_parameter("id_greater_than", lanelet::Id());
      const auto lower_bound = node.get_parameter("id_greater_than").as_int();
      return [lower_bound](const auto & lanelet) {
        auto discard = [&]() {
          std::cerr << "[" << lanelet.id() << "] discarded: The ID is not greater than "
                    << lower_bound << "." << std::endl;
          return false;
        };
        return lower_bound < lanelet.id() or discard();
      };
    }(),

    [&]() {
      node.declare_parameter("id_less_than", std::numeric_limits<lanelet::Id>::max());
      const auto upper_bound = node.get_parameter("id_less_than").as_int();
      return [upper_bound](const auto & lanelet) {
        auto discard = [&]() {
          std::cerr << "[" << lanelet.id() << "] discarded: The ID is not less than " << upper_bound
                    << "." << std::endl;
          return false;
        };
        return lanelet.id() < upper_bound or discard();
      };
    }(),

    [&]() {
      node.declare_parameter("left_of", lanelet::Id());
      const auto right_id = node.get_parameter("left_of").as_int();
      return [&, right_id](const auto & lanelet) {
        auto is_left_of = [&](auto right_id) {
          auto right = map->laneletLayer.find(right_id);
          return right != map->laneletLayer.end() and lanelet::geometry::leftOf(lanelet, *right);
        };
        auto discard = [&]() {
          std::cerr << "[" << lanelet.id()
                    << "] discarded: The lanelet is not to the left of the lanelet with ID "
                    << right_id << "." << std::endl;
          return false;
        };
        return not right_id or is_left_of(right_id) or discard();
      };
    }(),

    [&]() {
      node.declare_parameter("right_of", lanelet::Id());
      const auto left_id = node.get_parameter("right_of").as_int();
      return [&, left_id](const auto & lanelet) {
        auto is_right_of = [&](auto left_id) {
          auto left = map->laneletLayer.find(left_id);
          return left != map->laneletLayer.end() and lanelet::geometry::rightOf(lanelet, *left);
        };
        auto discard = [&]() {
          std::cerr << "[" << lanelet.id()
                    << "] discarded: The lanelet is not to the right of the lanelet with ID "
                    << left_id << "." << std::endl;
          return false;
        };
        return not left_id or is_right_of(left_id) or discard();
      };
    }(),

    [&]() {
      node.declare_parameter("leftmost", false);
      const auto constrained = node.get_parameter("leftmost").as_bool();
      return [&, constrained](const auto & lanelet) {
        auto discard = [&]() {
          std::cerr << "[" << lanelet.id()
                    << "] discarded: There are lanelets to the left of the lanelet." << std::endl;
          return false;
        };
        return not constrained or is_leftmost(lanelet) or discard();
      };
    }(),

    [&]() {
      node.declare_parameter("not_leftmost", false);
      const auto constrained = node.get_parameter("not_leftmost").as_bool();
      return [&, constrained](const auto & lanelet) {
        auto discard = [&]() {
          std::cerr << "[" << lanelet.id()
                    << "] discarded: There are lanelets to the left of the lanelet." << std::endl;
          return false;
        };
        return not constrained or not is_leftmost(lanelet) or discard();
      };
    }(),

    [&]() {
      node.declare_parameter("rightmost", false);
      const auto constrained = node.get_parameter("rightmost").as_bool();
      return [&, constrained](const auto & lanelet) {
        auto discard = [&]() {
          std::cerr << "[" << lanelet.id()
                    << "] discarded: There are lanelets to the right of the lanelet." << std::endl;
          return false;
        };
        return not constrained or is_rightmost(lanelet) or discard();
      };
    }(),

    [&]() {
      node.declare_parameter("not_rightmost", false);
      const auto constrained = node.get_parameter("not_rightmost").as_bool();
      return [&, constrained](const auto & lanelet) {
        auto discard = [&]() {
          std::cerr << "[" << lanelet.id()
                    << "] discarded: There are lanelets to the right of the lanelet." << std::endl;
          return false;
        };
        return not constrained or not is_rightmost(lanelet) or discard();
      };
    }(),

    [&]() {
      node.declare_parameter("route_length_greater_than", 0.0);
      const auto lower_bound = node.get_parameter("route_length_greater_than").as_double();
      return [&, lower_bound](const auto & lanelet) {
        auto discard = [&]() {
          std::cerr
            << "[" << lanelet.id()
            << "] discarded: The length of the path routable from the lanelet is not greater than "
            << lower_bound << std::endl;
          return false;
        };
        return 0 < routing_graph->possiblePaths(lanelet, lower_bound).size() or discard();
      };
    }(),
  };

  auto satisfy = [&](const auto & lanelet) {
    return std::all_of(
      constraints.begin(), constraints.end(), [&](const auto satisfy) { return satisfy(lanelet); });
  };

  auto receive = [&](auto && results) -> decltype(auto) {
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

      node.declare_parameter("select", "only");
      const auto select = node.get_parameter("select").as_string();

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

  filter(satisfy, map->laneletLayer, receive);
} catch (const std::exception & exception) {
  std::cerr << exception.what() << std::endl;
  return EXIT_FAILURE;
}
