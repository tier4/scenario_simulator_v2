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

#include <openscenario_interpreter/console/escape_sequence.hpp>
#include <openscenario_interpreter/utility/pugi_extension.hpp>

#include <iomanip>
#include <string>

namespace openscenario_interpreter
{
std::ostream & operator<<(std::ostream & os, const pugi::xml_node & datum)
{
  static std::size_t depth = 0;

  const auto indent = std::string(depth * 2, ' ');

  os << indent << blue << "<" << bold << datum.name() << reset;

  for (const auto & attribute : datum.attributes()) {
    os << " " << yellow << attribute.name() << reset;
    os << "=" << cyan << std::quoted(attribute.value()) << reset;
  }

  if (datum.first_child().empty()) {
    return os << blue << "/>" << reset << std::endl;
  } else {
    os << blue << ">" << reset << std::endl;

    ++depth;

    for (const auto & each : datum.children()) {
      os << each;
    }

    --depth;

    os << indent;
    os << blue << "</" << bold << datum.name() << reset << blue << ">" << reset << std::endl;

    return os;
  }
}
}  // namespace openscenario_interpreter
