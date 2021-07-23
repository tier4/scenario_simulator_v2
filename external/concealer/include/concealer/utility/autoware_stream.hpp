// Copyright 2015-2021 Tier IV, Inc. All rights reserved.
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

#ifndef CONCEALER__UTILITY__AUTOWARE_STREAM_HPP_
#define CONCEALER__UTILITY__AUTOWARE_STREAM_HPP_

#define AUTOWARE_INFO_STREAM(...) \
  RCLCPP_INFO_STREAM(get_logger(), "\x1b[32m" << __VA_ARGS__ << "\x1b[0m")

#define AUTOWARE_WARN_STREAM(...) \
  RCLCPP_WARN_STREAM(get_logger(), "\x1b[33m" << __VA_ARGS__ << "\x1b[0m")

#define AUTOWARE_ERROR_STREAM(...) \
  RCLCPP_ERROR_STREAM(get_logger(), "\x1b[1;31m" << __VA_ARGS__ << "\x1b[0m")

#define AUTOWARE_SYSTEM_ERROR(FROM) \
  AUTOWARE_ERROR_STREAM(            \
    "Error on calling " FROM ": " << std::system_error(errno, std::system_category()).what())

#endif  // CONCEALER__UTILITY__AUTOWARE_STREAM_HPP_
