// Copyright 2015-2020 Autoware Foundation. All rights reserved.
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

#ifndef SCENARIO_RUNNER__UTILITY__PUGI_EXTENSION_HPP_
#define SCENARIO_RUNNER__UTILITY__PUGI_EXTENSION_HPP_

#include <pugixml.hpp>

std::ostream & operator<<(std::ostream & os, const pugi::xml_node & node)
{
  static std::size_t depth {0};

  const auto indent {std::string(depth * 2, ' ')};

  os << indent << "\x1b[34m<\x1b[1m" << node.name() << "\x1b[0m";

  for (const auto & attribute : node.attributes()) {
    os << " \x1b[33m" << attribute.name() << "\x1b[0m=\x1b[36m\"" << attribute.value() <<
      "\"\x1b[0m";
  }

  if (node.first_child().empty()) {
    return os << "\x1b[34m/>\x1b[0m" << std::endl;
  } else {
    os << "\x1b[34m>\x1b[0m" << std::endl;

    ++depth;
    for (const auto & each : node.children()) {
      os << each;
    }
    --depth;

    return os << indent << "\x1b[34m</\x1b[1m" << node.name() << "\x1b[34m>\x1b[0m" << std::endl;
  }
}

#endif  // SCENARIO_RUNNER__UTILITY__PUGI_EXTENSION_HPP_
