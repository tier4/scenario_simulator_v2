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

#ifndef CONCEALER__HAS_DATA_MEMBER_OPTION_HPP_
#define CONCEALER__HAS_DATA_MEMBER_OPTION_HPP_

#include <type_traits>

namespace concealer
{
template <typename T, typename = void>
struct has_data_member_option : public std::false_type
{
};

template <typename T>
struct has_data_member_option<T, std::void_t<decltype(std::declval<T>().option)>>
: public std::true_type
{
};

template <typename... Ts>
inline constexpr auto has_data_member_option_v = has_data_member_option<Ts...>::value;
}  // namespace concealer

#endif  // CONCEALER__HAS_DATA_MEMBER_OPTION_HPP_
