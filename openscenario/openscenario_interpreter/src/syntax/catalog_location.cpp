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

#include <openscenario_interpreter/reader/element.hpp>
#include <openscenario_interpreter/syntax/catalog.hpp>
#include <openscenario_interpreter/syntax/catalog_location.hpp>
#include <openscenario_interpreter/syntax/directory.hpp>
#include <openscenario_interpreter/syntax/open_scenario.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
auto convertScenario(
  const std::filesystem::path & yaml_path, const std::filesystem::path & output_dir)
{
  std::stringstream command;

  command << "python3 -c \"from openscenario_utility import conversion; conversion.main()\""
          << " --input " << yaml_path  //
          << " --output " << output_dir;

  if (std::system(command.str().c_str()) != 0) {
    THROW_SYNTAX_ERROR("failed to convert scenario: " + yaml_path.string());
  } else {
    return output_dir / yaml_path.filename().stem().replace_extension(".xosc");
  }
}

CatalogLocation::CatalogLocation(const pugi::xml_node & node, Scope & scope)
: directory(readElement<Directory>("Directory", node, scope))
{
  if (not std::filesystem::exists(directory.path)) {
    THROW_SYNTAX_ERROR(directory.path.string() + " is not found");
  }
  if (not std::filesystem::is_directory(directory.path)) {
    THROW_SYNTAX_ERROR(directory.path.string() + " is not directory");
  }

  for (auto path : Directory::ls(directory)) {
    if (path.extension() == ".yaml") {
      path = convertScenario(
        path, std::filesystem::path("/tmp/converted_scenario") / directory.path.filename());
    } else if (path.extension() != ".xosc") {
      continue;
    }
    auto new_document = std::make_shared<pugi::xml_document>();
    new_document->load_file(path.string().c_str());
    catalog_files.emplace_back(std::move(new_document));
  }

  for (auto && xml : catalog_files) {
    auto open_scenario = xml->child("OpenSCENARIO");
    if (open_scenario) {
      auto catalog = open_scenario.child("Catalog");
      if (catalog) {
        auto name = catalog.attribute("name");
        if (name) {
          emplace(name.as_string(), catalog);
        }
      }
    }
  }
}
}  // namespace syntax
}  // namespace openscenario_interpreter
