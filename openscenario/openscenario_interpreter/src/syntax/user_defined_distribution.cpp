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
#include <openscenario_interpreter/reader/content.hpp>
#include <openscenario_interpreter/syntax/user_defined_distribution.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
UserDefinedDistribution::UserDefinedDistribution(const pugi::xml_node & node, const Scope & scope)
: Scope(scope), type(readAttribute<String>("type", node, local()))
{
}

auto UserDefinedDistribution::evaluate() -> Object
{
  throw common::Error("UserDefinedDistribution is not implemented yet");
}
}  // namespace syntax
}  // namespace openscenario_interpreter
