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

#include <openscenario_interpreter/reader/attribute.hpp>
#include <openscenario_interpreter/reader/element.hpp>
#include <openscenario_interpreter/syntax/file_header.hpp>
#include <openscenario_interpreter/syntax/license.hpp>
#include <openscenario_interpreter/syntax/properties.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
FileHeader::FileHeader(const pugi::xml_node & tree, Scope & scope)
: license(readElement<License>("License", tree, scope)),
  properties(readElement<Properties>("Properties", tree, scope)),
  revMajor(readAttribute<UnsignedShort>("revMajor", tree, scope)),
  revMinor(readAttribute<UnsignedShort>("revMinor", tree, scope)),
  date(readAttribute<String>("date", tree, scope)),
  description(readAttribute<String>("description", tree, scope)),
  author(readAttribute<String>("author", tree, scope))
{
}
}  // namespace syntax
}  // namespace openscenario_interpreter
