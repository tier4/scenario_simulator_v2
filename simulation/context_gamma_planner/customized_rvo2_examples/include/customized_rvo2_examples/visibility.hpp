// Copyright 2021 Tier IV, Inc All rights reserved.
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

#ifndef CUSTOMIZED_RVO2_EXAMPLES__VISIBILITY_HPP_
#define CUSTOMIZED_RVO2_EXAMPLES__VISIBILITY_HPP_

#ifdef __cplusplus
extern "C" {
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__

#ifdef __GNUC__
#define RVO2_ROS2_EXPORT __attribute__((dllexport))
#define RVO2_ROS2_IMPORT __attribute__((dllimport))
#else
#define RVO2_ROS2_EXPORT __declspec(dllexport)
#define RVO2_ROS2_IMPORT __declspec(dllimport)
#endif

#ifdef RVO2_ROS2_DLL
#define RVO2_ROS2_PUBLIC RVO2_ROS2_EXPORT
#else
#define RVO2_ROS2_PUBLIC RVO2_ROS2_IMPORT
#endif

#define RVO2_ROS2_PUBLIC_TYPE RVO2_ROS2_PUBLIC

#define RVO2_ROS2_LOCAL

#else

#define RVO2_ROS2_EXPORT __attribute__((visibility("default")))
#define RVO2_ROS2_IMPORT

#if __GNUC__ >= 4
#define RVO2_ROS2_PUBLIC __attribute__((visibility("default")))
#define RVO2_ROS2_LOCAL __attribute__((visibility("hidden")))
#else
#define RVO2_ROS2_PUBLIC
#define RVO2_ROS2_LOCAL
#endif

#define RVO2_ROS2_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // CUSTOMIZED_RVO2_EXAMPLES__VISIBILITY_HPP_
