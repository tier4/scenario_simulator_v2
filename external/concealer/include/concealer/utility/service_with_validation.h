#ifndef SRC_SERVICE_INTERFACE_HPP
#define SRC_SERVICE_INTERFACE_HPP

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
    autoware(autoware),
    client(autoware.create_client<T>(serviceName, rmw_qos_profile_default)),
    validation_rate(std::chrono::seconds(1))
  {
  }

  void requestOnce(const typename T::Request::SharedPtr & request)
  {
    validateAvailability();
    callWithTimeoutValidation(request);
  }

  void requestUntilSuccess(const typename T::Request::SharedPtr & request)
  {
    validateAvailability();
    for (int attempt = 0; attempt < attempts_count; ++attempt, validation_rate.sleep()) {
      autoware.rethrow();
      std::optional<typename rclcpp::Client<T>::SharedFuture> future_opt =
        callWithTimeoutValidation(request);
      if (!future_opt) {
        continue;
      }

      auto result = future_opt->get();
      if (result->status.code == tier4_external_api_msgs::msg::ResponseStatus::SUCCESS) {
        RCLCPP_INFO_STREAM(
          autoware.get_logger(), serviceName << " service request has been accepted "
                                             << (result->status.message.empty()
                                                   ? "."
                                                   : " (" + result.get()->status.message + ")."));
        break;
      }
      RCLCPP_ERROR_STREAM(
        autoware.get_logger(),
        serviceName << " service request was accepted, but ResponseStatus is FAILURE "
                    << (result->status.message.empty() ? "" : " (" + result->status.message + ")"));
    }
  }

private:
  void validateAvailability()
  {
    autoware.rethrow();
    while (!client->service_is_ready()) {
      autoware.rethrow();
      RCLCPP_INFO_STREAM(autoware.get_logger(), serviceName << " service is not ready.");
      validation_rate.sleep();
    }
  }

  std::optional<typename rclcpp::Client<T>::SharedFuture> callWithTimeoutValidation(
    const typename T::Request::SharedPtr & request)
  {
    auto future = client->async_send_request(request);
    if (future.wait_for(std::chrono::seconds(1)) != std::future_status::ready) {
      RCLCPP_ERROR_STREAM(autoware.get_logger(), serviceName << " service request has timed out.");
      return std::nullopt;
    }
    return future;
  }

  std::string serviceName;
  Autoware & autoware;
  typename rclcpp::Client<T>::SharedPtr client;
  rclcpp::WallRate validation_rate;
  static constexpr int attempts_count = 10;
};
}  // namespace concealer

#endif  //SRC_SERVICE_INTERFACE_HPP
