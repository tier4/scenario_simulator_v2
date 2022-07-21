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

#include <concealer/cooperator.hpp>

namespace concealer
{
// auto operator>>(std::istream & is, RTC::Cooperator & cooperator) -> std::istream &
// {
//   std::string token;
//
//   is >> token;
//
//   static const std::unordered_map<std::string, RTC::Cooperator> table{
//     {"simulator", RTC::Cooperator::simulator},
//     {"scenario", RTC::Cooperator::scenario},
//   };
//
//   if (auto iter = table.find(token); iter != std::end(table)) {
//     cooperator = (*iter).second;
//   } else {
//     cooperator = RTC::Cooperator::simulator;
//   }
//
//   return is;
// }
//
// auto operator<<(std::ostream & os, const RTC::Cooperator & cooperator) -> std::ostream &
// {
//   switch (cooperator) {
//     default:
//     case RTC::Cooperator::simulator:
//       return os << "simulator";
//
//     case RTC::Cooperator::scenario:
//       return os << "scenario";
//   }
// }
}  // namespace concealer
