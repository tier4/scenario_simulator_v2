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
#include <string>

#include "concealer/autoware.hpp"
#include "rclcpp/rclcpp.hpp"

namespace concealer
{
template <class T>
class ServiceWithValidation
{
public:
  ServiceWithValidation(std::string serviceName, Autoware & autoware)
  : serviceName(serviceName),
    logger(autoware.get_logger()),
    client(autoware.create_client<T>(serviceName, rmw_qos_profile_default)),
    validation_rate(std::chrono::seconds(1))
  {
  }

  void operator()(const typename T::Request::SharedPtr & request)
  {
    validateAvailability();
    callWithTimeoutValidation(request);
  }

  void operator()(const typename T::Request::SharedPtr & request, size_t attempts_count)
  {
    validateAvailability();
    for (size_t attempt = 0; attempt < attempts_count; ++attempt, validation_rate.sleep()) {
      std::optional<typename rclcpp::Client<T>::SharedFuture> future_opt =
        callWithTimeoutValidation(request);
      if (!future_opt) {
        continue;
      }

      auto result = future_opt->get();
      if (result->status.code == tier4_external_api_msgs::msg::ResponseStatus::SUCCESS) {
        RCLCPP_INFO_STREAM(
          logger, serviceName << " service request has been accepted "
                              << (result->status.message.empty()
                                    ? "."
                                    : " (" + result.get()->status.message + ")."));
        break;
      }
      RCLCPP_ERROR_STREAM(
        logger,
        serviceName << " service request was accepted, but ResponseStatus is FAILURE "
                    << (result->status.message.empty() ? "" : " (" + result->status.message + ")"));
    }
  }

private:
  void validateAvailability()
  {
    while (!client->service_is_ready()) {
      RCLCPP_INFO_STREAM(logger, serviceName << " service is not ready.");
      validation_rate.sleep();
    }
  }

  std::optional<typename rclcpp::Client<T>::SharedFuture> callWithTimeoutValidation(
    const typename T::Request::SharedPtr & request)
  {
    auto future = client->async_send_request(request);
    if (future.wait_for(std::chrono::seconds(1)) != std::future_status::ready) {
      RCLCPP_ERROR_STREAM(logger, serviceName << " service request has timed out.");
      return std::nullopt;
    }
    return future;
  }

  std::string serviceName;
  rclcpp::Logger logger;
  typename rclcpp::Client<T>::SharedPtr client;
  rclcpp::WallRate validation_rate;
};
}  // namespace concealer

#endif  //CONCEALER__SERVICE_WITH_VALIDATION_HPP_
