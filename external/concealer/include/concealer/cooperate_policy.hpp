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

#ifndef CONCEALER__COOPERATE_POCILY_HPP_
#define CONCEALER__COOPERATE_POCILY_HPP_

#include <boost/lexical_cast.hpp>
#include <istream>
#include <unordered_map>

namespace concealer
{
enum class CooperatePolicy {
  automatic,  // DEFAULT
  manual,
};

auto operator>>(std::istream & is, CooperatePolicy & cooperate_policy) -> std::istream &;

auto operator<<(std::ostream & os, const CooperatePolicy & cooperate_policy) -> std::ostream &;
}  // namespace concealer

#endif  // CONCEALER__COOPERATE_POCILY_HPP_
