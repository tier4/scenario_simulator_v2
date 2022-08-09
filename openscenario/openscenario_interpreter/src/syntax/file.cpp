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

#include <boost/filesystem/operations.hpp>  // boost::filesystem::is_directory
#include <openscenario_interpreter/reader/attribute.hpp>
#include <openscenario_interpreter/syntax/file.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
File::File() : filepath("./") {}

File::File(const std::string & filepath) : filepath(filepath) {}

File::File(const pugi::xml_node & node, Scope & scope)
: filepath(readAttribute<String>("filepath", node, scope))
{
}

auto File::isDirectory() const -> bool { return boost::filesystem::is_directory(filepath); }

File::operator boost::filesystem::path() const { return filepath; }

File::operator String() const { return filepath.string(); }
}  // namespace syntax
}  // namespace openscenario_interpreter
