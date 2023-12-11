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
#include <limits>
#include <map_fragment/print.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

namespace map_fragment
{
inline namespace tuple
{
auto head = [](auto tuple) -> decltype(auto) { return std::get<0>(tuple); };

auto rest = [](auto && tuple) -> decltype(auto) {
  return std::apply(
    [](auto &&, auto &&... xs) { return std::forward_as_tuple(std::forward<decltype(xs)>(xs)...); },
    std::forward<decltype(tuple)>(tuple));
};

template <typename F, typename... Ts>
auto apply(const std::tuple<F> & invocable, Ts &&... xs) -> decltype(auto)
{
  return head(invocable)(std::forward<decltype(xs)>(xs)...);
}

template <typename F, typename G, typename... Fs, typename... Ts>
auto apply(const std::tuple<F, G, Fs...> & invocables, Ts &&... xs) -> decltype(auto)
{
  auto thunk = [&]() -> decltype(auto) {
    return map_fragment::apply(rest(invocables), std::forward<decltype(xs)>(xs)...);
  };

  return head(invocables)(thunk, xs...);
}
}  // namespace tuple
}  // namespace map_fragment

auto main(const int argc, char const * const * const argv) -> int
try {
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node(
    std::filesystem::path(argv[0]).stem(),
    rclcpp::NodeOptions()
      .allow_undeclared_parameters(true)
      .automatically_declare_parameters_from_overrides(true));

  auto at = [&](auto thunk, auto && array) -> decltype(auto) {
    switch (const auto parameter = node.get_parameter("index"); parameter.get_type()) {
      case rclcpp::ParameterType::PARAMETER_INTEGER:
        if (const auto index = parameter.as_int();
            0 <= index and static_cast<std::size_t>(index) < array.size()) {
          return map_fragment::print(std::cout, array[index]) << std::endl;
        } else {
          std::stringstream what;
          what << "It is not possible to refer to index " << index << " of array ";
          map_fragment::print(what, array);
          what << " of size " << array.size() << ".";
          throw std::out_of_range(what.str());
        }

      case rclcpp::ParameterType::PARAMETER_NOT_SET:
        return thunk();

      default:
        throw std::invalid_argument("index type-error!");
    }
  };

  auto select = [&](auto thunk, auto && array) -> decltype(auto) {
    switch (const auto parameter = node.get_parameter("select"); parameter.get_type()) {
      case rclcpp::ParameterType::PARAMETER_STRING:
        if (const auto select = parameter.as_string(); select == "first") {
          return std::cout << array.front() << std::endl;
        } else if (select == "last") {
          return std::cout << array.back() << std::endl;
        } else {
          std::stringstream what;
          what << "An unknown value " << std::quoted(select) << " was given for parameter "
               << std::quoted(parameter.get_name()) << ".";
          throw std::out_of_range(what.str());
        }

      case rclcpp::ParameterType::PARAMETER_NOT_SET:
        return thunk();

      default:
        throw std::invalid_argument("select type-error!");
    }
  };

  auto otherwise = [](auto && array) -> auto & {
    return map_fragment::print(std::cout, std::forward<decltype(array)>(array)) << std::endl;
  };

  switch (const auto parameter = node.get_parameter("reference"); parameter.get_type()) {
    case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY:
      map_fragment::apply(std::make_tuple(at, select, otherwise), parameter.as_integer_array());
      break;

    case rclcpp::ParameterType::PARAMETER_STRING_ARRAY:
      map_fragment::apply(std::make_tuple(at, select, otherwise), parameter.as_string_array());
      break;

    case rclcpp::ParameterType::PARAMETER_NOT_SET:
      std::cout << "[]" << std::endl;
      break;

    default:
      throw std::invalid_argument("reference type-error!");
  }

  return EXIT_SUCCESS;
} catch (const std::exception & exception) {
  std::cerr << exception.what() << std::endl;
  return EXIT_FAILURE;
}
