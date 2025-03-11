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
#include <tier4_rtc_msgs/srv/auto_mode_with_module.hpp>
#include <type_traits>

namespace concealer
{
template <typename T>
class Service
{
  const std::string service_name;

  rclcpp::Logger logger;

  typename rclcpp::Client<T>::SharedPtr client;

  rclcpp::WallRate validation_rate;

public:
  template <typename Node>
  explicit Service(
    const std::string & service_name, Node & node,
    const std::chrono::nanoseconds validation_interval = std::chrono::seconds(1))
  : service_name(service_name),
    logger(node.get_logger()),
    client(node.template create_client<T>(service_name, rmw_qos_profile_default)),
    validation_rate(validation_interval)
  {
  }

  auto operator()(const typename T::Request::SharedPtr & request, std::size_t attempts_count)
    -> void
  {
    while (!client->service_is_ready()) {
      RCLCPP_INFO_STREAM(logger, service_name << " service is not ready.");
      validation_rate.sleep();
    }

    auto send = [this](const auto & request) {
      if (auto future = client->async_send_request(request);
          future.wait_for(validation_rate.period()) == std::future_status::ready) {
        return std::optional<typename rclcpp::Client<T>::SharedFuture>(future);
      } else {
        RCLCPP_ERROR_STREAM(logger, service_name << " service request has timed out.");
        return std::optional<typename rclcpp::Client<T>::SharedFuture>();
      }
    };

    auto receive = [this](const auto & response) {
      if constexpr (DetectMember_status<typename T::Response>::value) {
        if constexpr (std::is_same_v<
                        tier4_external_api_msgs::msg::ResponseStatus,
                        decltype(T::Response::status)>) {
          if (const auto & status = response->get()->status;
              status.code == tier4_external_api_msgs::msg::ResponseStatus::SUCCESS) {
            // clang-format off
            RCLCPP_INFO_STREAM(logger, service_name << " service request has been accepted" << (status.message.empty() ? "." : " (" + status.message + ")."));
            // clang-format on
            return true;
          } else {
            // clang-format off
            RCLCPP_ERROR_STREAM(logger, service_name << " service request was accepted, but ResponseStatus is FAILURE" << (status.message.empty() ? "." : " (" + status.message + ")"));
            // clang-format on
            return false;
          }
        } else if constexpr (std::is_same_v<
                               autoware_adapi_v1_msgs::msg::ResponseStatus,
                               decltype(T::Response::status)>) {
          if (const auto & status = response->get()->status; status.success) {
            // clang-format off
            RCLCPP_INFO_STREAM(logger, service_name << " service request has been accepted" << (status.message.empty() ? "." : " (" + status.message + ")."));
            // clang-format on
            return true;
          } else {
            // clang-format off
            RCLCPP_ERROR_STREAM(logger, service_name << " service request was accepted, but " "ResponseStatus::success is false with error code: " << status.code << ", and message: " << (status.message.empty() ? "" : " (" + status.message + ")"));
            // clang-format on
            return false;
          }
        } else {
          RCLCPP_INFO_STREAM(logger, service_name << " service request has been accepted.");
          return true;
        }
      } else if constexpr (DetectMember_success<typename T::Response>::value) {
        if constexpr (std::is_same_v<bool, decltype(T::Response::success)>) {
          if (response->get()->success) {
            RCLCPP_INFO_STREAM(logger, service_name << " service request has been accepted.");
            return true;
          } else {
            // clang-format off
            RCLCPP_ERROR_STREAM(logger, service_name << " service request has been accepted, but Response::success is false.");
            // clang-format off
            return false;
          }
        } else {
          RCLCPP_INFO_STREAM(logger, service_name << " service request has been accepted.");
          return true;
        }
      } else {
        RCLCPP_INFO_STREAM(logger, service_name << " service request has been accepted.");
        return true;
      }
    };

    for (std::size_t attempt = 0; attempt < attempts_count; ++attempt, validation_rate.sleep()) {
      if (receive(send(request))) {
        return;
      }
    }

    throw common::scenario_simulator_exception::AutowareError(
      "Requested the service ", std::quoted(service_name), " ", attempts_count,
      " times, but was not successful.");
  }
};
}  // namespace concealer

#endif  //CONCEALER__SERVICE_HPP_
