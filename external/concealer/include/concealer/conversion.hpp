// Copyright 2015-2020 Tier IV, Inc. All rights reserved.
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

#ifndef CONCEALER__CONVERSION_HPP_
#define CONCEALER__CONVERSION_HPP_

// NOTE: headers are lexicographically sorted.

#ifdef AUTOWARE_ARCHITECTURE_PROPOSAL
#include <autoware_vehicle_msgs/msg/engage.hpp>
#endif

#ifdef AUTOWARE_AUTO
// TODO(yamacir-kit)
#endif

#include <boost/mpl/and.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <type_traits>
#include <utility>

namespace concealer
{
#define REQUIRES(...) typename = typename std::enable_if<__VA_ARGS__::value>::type

#define REQUIRES_VA(...) \
  typename = typename std::enable_if<boost::mpl::and_<__VA_ARGS__>::value>::type

template <typename From, typename To>
struct converter
{
};

template <typename To, typename From>
decltype(auto) convertTo(From && from)
{
  return converter<typename std::decay<From>::type, typename std::decay<To>::type>()(
    std::forward<decltype(from)>(from));
}

template <typename From>
struct converter<From, std_msgs::msg::Bool>
{
  using result_type = std_msgs::msg::Bool;

  template <REQUIRES(std::is_convertible<From, bool>)>
  result_type operator()(const From & from)
  {
    result_type to;
    {
      to.data = from;
    }

    return to;
  }
};

template <typename From>
struct converter<From, std_msgs::msg::Float32>
{
  using result_type = std_msgs::msg::Float32;

  template <REQUIRES(std::is_floating_point<From>)>
  result_type operator()(const From from)
  {
    result_type to;
    {
      to.data = from;
    }

    return to;
  }
};

#ifdef AUTOWARE_ARCHITECTURE_PROPOSAL
template <typename From>
struct converter<From, autoware_vehicle_msgs::msg::Engage>
{
  using result_type = autoware_vehicle_msgs::msg::Engage;

  template <REQUIRES(std::is_convertible<From, bool>)>
  result_type operator()(const From & from)
  {
    result_type to;
    {
      to.engage = from;
    }

    return to;
  }
};
#endif

}  // namespace concealer

#endif  // CONCEALER__CONVERSION_HPP_
