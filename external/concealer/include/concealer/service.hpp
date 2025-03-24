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

#ifndef CONCEALER__SERVICE_HPP_
#define CONCEALER__SERVICE_HPP_

#include <autoware_adapi_v1_msgs/msg/response_status.hpp>
#include <chrono>
#include <concealer/member_detector.hpp>
#include <rclcpp/rclcpp.hpp>
#include <scenario_simulator_exception/exception.hpp>
#include <string>
#include <tier4_external_api_msgs/msg/response_status.hpp>
#include <tier4_rtc_msgs/msg/cooperate_response.hpp>
#include <tier4_rtc_msgs/srv/auto_mode_with_module.hpp>
#include <type_traits>

namespace concealer
{
template <typename T>
class Service
{
  const std::string name;

  typename rclcpp::Client<T>::SharedPtr client;

  rclcpp::WallRate interval;

public:
  template <typename Node>
  explicit Service(
    const std::string & name, Node & node,
    const std::chrono::nanoseconds & interval = std::chrono::seconds(1))
  : name(name),
    client(node.template create_client<T>(name, rmw_qos_profile_default)),
    interval(interval)
  {
  }

  auto operator()(const typename T::Request::SharedPtr & request, std::size_t attempts_count)
  {
    while (!client->service_is_ready()) {
      interval.sleep();
    }

    auto receive = [this](const auto & response) {
      if constexpr (DetectMember_status<typename T::Response>::value) {
        if constexpr (std::is_same_v<
                        tier4_external_api_msgs::msg::ResponseStatus,
                        decltype(T::Response::status)>) {
          return response->status.code == tier4_external_api_msgs::msg::ResponseStatus::SUCCESS;
        } else if constexpr (std::is_same_v<
                               autoware_adapi_v1_msgs::msg::ResponseStatus,
                               decltype(T::Response::status)>) {
          return response->status.success;
        } else {
          static_assert([]() { return false; });
        }
      } else if constexpr (DetectMember_success<typename T::Response>::value) {
        if constexpr (std::is_same_v<bool, decltype(T::Response::success)>) {
          return response->success;
        } else {
          static_assert([]() { return false; });
        }
      } else if constexpr (DetectMember_responses<typename T::Response>::value) {
        if constexpr (std::is_same_v<
                        std::vector<tier4_rtc_msgs::msg::CooperateResponse>,
                        decltype(T::Response::responses)>) {
          return std::all_of(
            response->responses.begin(), response->responses.end(),
            [](const auto & response) { return response.success; });
        } else {
          static_assert([]() { return false; });
        }
      } else {
        static_assert([]() { return false; });
      }
    };

    for (std::size_t attempt = 0; attempt < attempts_count; ++attempt, interval.sleep()) {
      if (auto future = client->async_send_request(request);
          future.wait_for(interval.period()) == std::future_status::ready and
          receive(future.get())) {
        return;
      }
    }

    throw common::scenario_simulator_exception::AutowareError(
      "Requested the service ", std::quoted(name), " ", attempts_count,
      " times, but was not successful.");
  }
};
}  // namespace concealer

#endif  //CONCEALER__SERVICE_HPP_
