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

#ifndef CONCEALER__AUTOWARE_UNIVERSE_HPP_
#define CONCEALER__AUTOWARE_UNIVERSE_HPP_

#include <concealer/autoware.hpp>
#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <concealer/utility/subscriber_wrapper.hpp>

namespace concealer {


class AutowareUniverse : public Autoware
{
  using AckermannControlCommand = autoware_auto_control_msgs::msg::AckermannControlCommand;
  SubscriberWrapper<AckermannControlCommand> getAckermannControlCommand;
public:

  CONCEALER_PUBLIC explicit AutowareUniverse()
  : getAckermannControlCommand("/control/command/control_cmd", *this)
  {}

  auto getAcceleration() const -> double override;

};

}

#endif //CONCEALER__AUTOWARE_UNIVERSE_HPP_
