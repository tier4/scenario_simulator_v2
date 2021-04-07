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

#include <openscenario_interpreter/syntax/boolean.hpp>
#include <string>

namespace openscenario_interpreter
{
inline namespace syntax
{
std::ostream & operator<<(std::ostream & os, const Boolean & datum)
{
  boost::io::ios_flags_saver saver{os};
  return os << std::boolalpha << datum.data;
}

std::istream & operator>>(std::istream & is, Boolean & datum)
{
  boost::io::ios_flags_saver saver{is};
  return is >> std::boolalpha >> datum.data;
}

const Element true_v = make<Boolean>(true);
const Element false_v = make<Boolean>(false);

const Element & asBoolean(bool value) { return value ? true_v : false_v; }
}  // namespace syntax
}  // namespace openscenario_interpreter
