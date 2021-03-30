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

#include <openscenario_interpreter/syntax/dimensions.hpp>

#include <string>

namespace openscenario_interpreter
{
inline namespace syntax
{
static_assert(IsOptionalElement<Dimensions>::value, "Dimensions must be an optional element");

std::ostream & operator<<(std::ostream & os, const Dimensions & datum)
{
  os << indent;
  os << blue << "<Dimensions";
  os << " " << highlight("width", datum.width);
  os << " " << highlight("length", datum.length);
  os << " " << highlight("height", datum.height);
  os << blue << "/>" << reset;

  return os;
}
}  // namespace syntax
}  // namespace openscenario_interpreter
