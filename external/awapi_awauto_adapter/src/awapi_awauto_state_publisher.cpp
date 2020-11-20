// Copyright 2015-2020 Autoware Foundation. All rights reserved.
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

#include <awapi_awauto_state_publisher/awapi_awauto_state_publisher.hpp>

namespace autoware_api
{
AutowareIvAutowareStatePublisher::AutowareIvAutowareStatePublisher(const rclcpp::NodeOptions & options)
: rclcpp::Node("awapi_awauto_state_publisher", options)
{
  // publisher
  pub_state_ = this->create_publisher<AwapiAutowareStatus>("/awapi/autoware/get/status", 1);
}

void AutowareIvAutowareStatePublisher::statePublisher(const AutowareInfo & aw_info)
{
  AwapiAutowareStatus status;
  autoware_api_msgs::AwapiAutowareStatus status;

  //input header
  status.header.frame_id = "base_link";
  status.header.stamp = ros::Time::now();
}
void AutowareIvAutowareStatePublisher::getAutowareStateInfo(){
  // get autoware status
}

}
