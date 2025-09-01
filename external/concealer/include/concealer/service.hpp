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
    const std::chrono::nanoseconds & interval = std::chrono::seconds(3))
  : name(name),
    client(node.template create_client<T>(name, rmw_qos_profile_default)),
    interval(interval)
  {
  }

  auto operator()(const typename T::Request::SharedPtr & request, std::size_t attempts_count)
  {
    RCLCPP_WARN(
      rclcpp::get_logger("DEBUG/concealer::Service"), "Service request: %s",
      name.c_str());

    constexpr auto max_wait_time = std::chrono::seconds(15);
    const auto max_response_timestamp = std::chrono::steady_clock::now() + max_wait_time;
    const auto start_time = std::chrono::steady_clock::now();
    while (rclcpp::ok() and not client->service_is_ready()) {
      if (std::chrono::steady_clock::now() > max_response_timestamp) {
        throw common::scenario_simulator_exception::AutowareError(
          "Service ", std::quoted(name), " not ready ", max_wait_time.count(),
          " seconds after the request");
      }
      const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start_time).count();
      RCLCPP_WARN(
        rclcpp::get_logger("DEBUG/concealer::Service"),
        "Waiting for service %s to be ready... (elapsed: %ld ms)", name.c_str(), elapsed_ms);
      
      interval.sleep();
    }

    const auto total_wait_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start_time).count();
    RCLCPP_WARN(
      rclcpp::get_logger("DEBUG/concealer::Service"),
      "Service %s is now ready (total wait time: %ld ms)", name.c_str(), total_wait_time_ms);

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
      RCLCPP_WARN(
        rclcpp::get_logger("DEBUG/concealer::Service"),
        "Sending request to service %s (attempt %zu/%zu)", name.c_str(), attempt, attempts_count);
      
      if (auto future = client->async_send_request(request);
          future.wait_for(interval.period()) == std::future_status::ready and
          receive(future.get())) {
        RCLCPP_WARN(
          rclcpp::get_logger("DEBUG/concealer::Service"), 
          "Service %s responded successfully on attempt %zu", name.c_str(), attempt);
        return;
      } else {
        RCLCPP_WARN(
          rclcpp::get_logger("DEBUG/concealer::Service"),
          "Service %s request failed on attempt %zu", name.c_str(), attempt);
      }
    }

    throw common::scenario_simulator_exception::AutowareError(
      "Requested the service ", std::quoted(name), " ", attempts_count,
      " times, but was not successful.");
  }
};
}  // namespace concealer

#endif  //CONCEALER__SERVICE_HPP_
