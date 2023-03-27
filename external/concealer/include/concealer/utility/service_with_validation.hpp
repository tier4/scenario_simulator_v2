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

#ifndef CONCEALER__SERVICE_WITH_VALIDATION_HPP_
#define CONCEALER__SERVICE_WITH_VALIDATION_HPP_

#include <chrono>
#include <concealer/autoware.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <tier4_external_api_msgs/msg/response_status.hpp>

namespace concealer
{
template <typename T>
class ServiceWithValidation
{
public:
  explicit ServiceWithValidation(const std::string & service_name, Autoware & autoware)
  : service_name(service_name),
    logger(autoware.get_logger()),
    client(autoware.create_client<T>(service_name, rmw_qos_profile_default)),
    validation_rate(std::chrono::seconds(1))
  {
  }

  auto operator()(const typename T::Request::SharedPtr & request) -> void
  {
    validateAvailability();
    callWithTimeoutValidation(request);
  }

  auto operator()(const typename T::Request::SharedPtr & request, std::size_t attempts_count)
    -> void
  {
    validateAvailability();
    for (std::size_t attempt = 0; attempt < attempts_count; ++attempt, validation_rate.sleep()) {
      if (const auto & service_call_result = callWithTimeoutValidation(request)) {
        if (const auto & service_call_status = service_call_result->get()->status;
            service_call_status.code == tier4_external_api_msgs::msg::ResponseStatus::SUCCESS) {
          RCLCPP_INFO_STREAM(
            logger, service_name << " service request has been accepted "
                                 << (service_call_status.message.empty()
                                       ? "."
                                       : " (" + service_call_status.message + ")."));
          return;
        } else {
          RCLCPP_ERROR_STREAM(
            logger, service_name << " service request was accepted, but ResponseStatus is FAILURE "
                                 << (service_call_status.message.empty()
                                       ? ""
                                       : " (" + service_call_status.message + ")"));
        }
      }
    }
    throw common::AutowareError(
      "Requested the service ", service_name, " ", attempts_count,
      " times, but was not successful.");
  }

private:
  auto validateAvailability() -> void
  {
    while (!client->service_is_ready()) {
      RCLCPP_INFO_STREAM(logger, service_name << " service is not ready.");
      validation_rate.sleep();
    }
  }

  auto callWithTimeoutValidation(const typename T::Request::SharedPtr & request)
    -> std::optional<typename rclcpp::Client<T>::SharedFuture>
  {
    if (auto future = client->async_send_request(request);
        future.wait_for(std::chrono::seconds(1)) == std::future_status::ready) {
      return future;
    } else {
      RCLCPP_ERROR_STREAM(logger, service_name << " service request has timed out.");
      return std::nullopt;
    }
  }

  const std::string service_name;
  rclcpp::Logger logger;
  typename rclcpp::Client<T>::SharedPtr client;
  rclcpp::WallRate validation_rate;
};
}  // namespace concealer

#endif  //CONCEALER__SERVICE_WITH_VALIDATION_HPP_
