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

#ifndef AWAPI_ACCESSOR__ACCESSOR_HPP_
#define AWAPI_ACCESSOR__ACCESSOR_HPP_

#include <awapi_accessor/utility/visibility.h>
#include <rclcpp/rclcpp.hpp>

namespace autoware_api
{

class Accessor
  : public rclcpp::Node
{
public:
  AWAPI_ACCESSOR_PUBLIC
  explicit Accessor(const rclcpp::NodeOptions &);
};

}  // namespace autoware_api

#endif  // AWAPI_ACCESSOR__ACCESSOR_HPP_
