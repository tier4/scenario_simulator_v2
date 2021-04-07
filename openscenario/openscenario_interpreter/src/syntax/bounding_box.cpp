// Copyright 2015-2020 Tier IV, Inc. All rights reserved.
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

#include <openscenario_interpreter/syntax/bounding_box.hpp>
#include <string>

namespace openscenario_interpreter
{
inline namespace syntax
{
static_assert(IsOptionalElement<BoundingBox>::value, "BoundingBox must be an optional element");

std::ostream & operator<<(std::ostream & os, const BoundingBox & datum)
{
  // clang-format off

  return os << (indent++) << blue << "<BoundingBox>\n" << reset
            << datum.center << "\n"
            << datum.dimensions << "\n"
            << (--indent) << blue << "</BoundingBox>" << reset;

  // clang-format on
}
}  // namespace syntax
}  // namespace openscenario_interpreter
