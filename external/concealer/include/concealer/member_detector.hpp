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

#ifndef CONCEALER__MEMBER_DETECTOR_HPP_
#define CONCEALER__MEMBER_DETECTOR_HPP_

#include <type_traits>

namespace concealer
{
#define DEFINE_MEMBER_DETECTOR(IDENTIFIER)                                                 \
  template <typename T, typename = void>                                                   \
  struct DetectMember_##IDENTIFIER : public std::false_type                                \
  {                                                                                        \
  };                                                                                       \
                                                                                           \
  template <typename T>                                                                    \
  struct DetectMember_##IDENTIFIER<T, std::void_t<decltype(std::declval<T>().IDENTIFIER)>> \
  : public std::true_type                                                                  \
  {                                                                                        \
  }

DEFINE_MEMBER_DETECTOR(allow_goal_modification);
DEFINE_MEMBER_DETECTOR(distance);
DEFINE_MEMBER_DETECTOR(option);
DEFINE_MEMBER_DETECTOR(responses);
DEFINE_MEMBER_DETECTOR(status);
DEFINE_MEMBER_DETECTOR(success);

#undef DEFINE_MEMBER_DETECTOR

#define DEFINE_STATIC_MEMBER_DETECTOR(IDENTIFIER)                                 \
  template <typename T, typename = void>                                          \
  struct DetectStaticMember_##IDENTIFIER : public std::false_type                 \
  {                                                                               \
  };                                                                              \
                                                                                  \
  template <typename T>                                                           \
  struct DetectStaticMember_##IDENTIFIER<T, std::void_t<decltype(T::IDENTIFIER)>> \
  : public std::true_type                                                         \
  {                                                                               \
  }

DEFINE_STATIC_MEMBER_DETECTOR(AVOIDANCE_BY_LC_LEFT);
DEFINE_STATIC_MEMBER_DETECTOR(AVOIDANCE_BY_LC_RIGHT);
DEFINE_STATIC_MEMBER_DETECTOR(AVOIDANCE_LEFT);
DEFINE_STATIC_MEMBER_DETECTOR(AVOIDANCE_RIGHT);
DEFINE_STATIC_MEMBER_DETECTOR(BLIND_SPOT);
DEFINE_STATIC_MEMBER_DETECTOR(COMFORTABLE_STOP);
DEFINE_STATIC_MEMBER_DETECTOR(CROSSWALK);
DEFINE_STATIC_MEMBER_DETECTOR(DETECTION_AREA);
DEFINE_STATIC_MEMBER_DETECTOR(EMERGENCY_STOP);
DEFINE_STATIC_MEMBER_DETECTOR(EXT_REQUEST_LANE_CHANGE_LEFT);
DEFINE_STATIC_MEMBER_DETECTOR(EXT_REQUEST_LANE_CHANGE_RIGHT);
DEFINE_STATIC_MEMBER_DETECTOR(GOAL_PLANNER);
DEFINE_STATIC_MEMBER_DETECTOR(INTERSECTION);
DEFINE_STATIC_MEMBER_DETECTOR(INTERSECTION_OCCLUSION);
DEFINE_STATIC_MEMBER_DETECTOR(LANE_CHANGE_LEFT);
DEFINE_STATIC_MEMBER_DETECTOR(LANE_CHANGE_RIGHT);
DEFINE_STATIC_MEMBER_DETECTOR(NONE);
DEFINE_STATIC_MEMBER_DETECTOR(NO_DRIVABLE_LANE);
DEFINE_STATIC_MEMBER_DETECTOR(NO_STOPPING_AREA);
DEFINE_STATIC_MEMBER_DETECTOR(OCCLUSION_SPOT);
DEFINE_STATIC_MEMBER_DETECTOR(PULL_OUT);
DEFINE_STATIC_MEMBER_DETECTOR(PULL_OVER);
DEFINE_STATIC_MEMBER_DETECTOR(START_PLANNER);
DEFINE_STATIC_MEMBER_DETECTOR(TRAFFIC_LIGHT);
DEFINE_STATIC_MEMBER_DETECTOR(UNKNOWN);

#undef DEFINE_STATIC_MEMBER_DETECTOR
}  // namespace concealer

#endif  // CONCEALER__MEMBER_DETECTOR_HPP_
