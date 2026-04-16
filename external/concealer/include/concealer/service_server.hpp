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

#ifndef CONCEALER__SERVICE_SERVER_HPP_
#define CONCEALER__SERVICE_SERVER_HPP_

#include <get_parameter/get_parameter.hpp>
#include <rclcpp/rclcpp.hpp>

namespace concealer
{
template <typename Service>
class ServiceServer
{
  typename rclcpp::Service<Service>::SharedPtr server_;

  bool enabled_;

public:
  template <typename Node, typename Callback>
  explicit ServiceServer(const std::string & name, Node & node, Callback && callback)
  : enabled_(
      common::getParameter<bool>(node.get_node_parameters_interface(), name + ".enabled", true))
  {
    if (enabled_) {
      server_ = node.template create_service<Service>(name, std::forward<Callback>(callback));
    }
  }

  auto isEnabled() const noexcept -> bool { return enabled_; }
};
}  // namespace concealer

#endif  // CONCEALER__SERVICE_SERVER_HPP_
