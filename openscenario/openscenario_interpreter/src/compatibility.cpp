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

#include <openscenario_interpreter/compatibility.hpp>

namespace openscenario_interpreter
{
auto operator>>(std::istream & input, Compatibility & compatiblity) -> std::istream &
{
  if (auto token = std::string(); input >> token) {
    if (token == "legacy") {
      compatiblity = Compatibility::legacy;
      return input;
    } else if (token == "standard") {
      compatiblity = Compatibility::standard;
      return input;
    } else {
      throw Error(
        "Unknown compatiblity ", std::quoted(token),
        " was specified. It must be \"legacy\" or \"standard\".");
    }
  } else {
    compatiblity = Compatibility::legacy;
    return input;
  }
}
}  // namespace openscenario_interpreter
