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

#include <concealer/cooperate_policy.hpp>

namespace concealer
{
auto operator>>(std::istream & is, CooperatePolicy & cooperate_policy) -> std::istream &
{
  std::string token;

  is >> token;

  static const std::unordered_map<std::string, CooperatePolicy> table{
    {"automatic", CooperatePolicy::automatic},
    {"manual", CooperatePolicy::manual},
  };

  if (auto iter = table.find(token); iter != std::end(table)) {
    cooperate_policy = (*iter).second;
  } else {
    cooperate_policy = CooperatePolicy::automatic;
  }

  return is;
}

auto operator<<(std::ostream & os, const CooperatePolicy & cooperate_policy) -> std::ostream &
{
  switch (cooperate_policy) {
    default:
    case CooperatePolicy::automatic:
      return os << "automatic";

    case CooperatePolicy::manual:
      return os << "manual";
  }
}
}  // namespace concealer
