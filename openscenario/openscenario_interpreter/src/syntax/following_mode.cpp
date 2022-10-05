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

#include <openscenario_interpreter/error.hpp>
#include <openscenario_interpreter/syntax/following_mode.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
static_assert(std::is_standard_layout<FollowingMode>::value);

static_assert(std::is_trivial<FollowingMode>::value);

auto operator>>(std::istream & is, FollowingMode & datum) -> std::istream &
{
  if (std::string buffer; is >> buffer and buffer == "follow") {
    datum.value = FollowingMode::follow;
    return is;
  } else if (buffer == "position") {
    datum.value = FollowingMode::position;
    return is;
  } else {
    throw UNEXPECTED_ENUMERATION_VALUE_SPECIFIED(FollowingMode, buffer);
  }
}

auto operator<<(std::ostream & os, const FollowingMode & datum) -> std::ostream &
{
  switch (datum) {
    default:
    case FollowingMode::follow:
      return os << "follow";
    case FollowingMode::position:
      return os << "position";
  }
}
}  // namespace syntax
}  // namespace openscenario_interpreter
