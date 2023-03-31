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
#include <openscenario_interpreter/syntax/fractional_cloud_cover.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
static_assert(std::is_standard_layout<FractionalCloudCover>::value, "");

static_assert(std::is_trivial<FractionalCloudCover>::value, "");

auto operator>>(std::istream & is, FractionalCloudCover & datum) -> std::istream &
{
  std::string buffer;

  is >> buffer;

#define BOILERPLATE(IDENTIFIER)                     \
  if (buffer == #IDENTIFIER) {                      \
    datum.value = FractionalCloudCover::IDENTIFIER; \
    return is;                                      \
  }                                                 \
  static_assert(true, "")

  BOILERPLATE(zeroOktas);
  BOILERPLATE(oneOktas);
  BOILERPLATE(twoOktas);
  BOILERPLATE(threeOktas);
  BOILERPLATE(fourOktas);
  BOILERPLATE(fiveOktas);
  BOILERPLATE(sixOktas);
  BOILERPLATE(sevenOktas);
  BOILERPLATE(eightOktas);
  BOILERPLATE(nineOktas);

#undef BOILERPLATE

  throw UNEXPECTED_ENUMERATION_VALUE_SPECIFIED(FractionalCloudCover, buffer);
}

auto operator<<(std::ostream & os, const FractionalCloudCover & datum) -> std::ostream &
{
  switch (datum) {
#define BOILERPLATE(ID)          \
  case FractionalCloudCover::ID: \
    return os << #ID;

    BOILERPLATE(zeroOktas);
    BOILERPLATE(oneOktas);
    BOILERPLATE(twoOktas);
    BOILERPLATE(threeOktas);
    BOILERPLATE(fourOktas);
    BOILERPLATE(fiveOktas);
    BOILERPLATE(sixOktas);
    BOILERPLATE(sevenOktas);
    BOILERPLATE(eightOktas);
    BOILERPLATE(nineOktas);

#undef BOILERPLATE

    default:
      throw UNEXPECTED_ENUMERATION_VALUE_ASSIGNED(FractionalCloudCover, datum);
  }
}
}  // namespace syntax
}  // namespace openscenario_interpreter
