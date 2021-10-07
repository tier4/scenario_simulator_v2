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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__CATALOG_LOCATION_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__CATALOG_LOCATION_HPP_

#include <openscenario_interpreter/reader/element.hpp>
#include <openscenario_interpreter/syntax/catalog.hpp>
#include <openscenario_interpreter/syntax/directory.hpp>
#include <openscenario_interpreter/syntax/openscenario.hpp>>

#include <boost/filesystem.hpp>
#include <memory>
#include <pugixml.hpp>
#include <unordered_map>
#include <vector>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- CatalogLocation --------------------------------------------------------
 *
 *
 * -------------------------------------------------------------------------- */
struct CatalogLocation : std::unordered_map<std::string, pugi::xml_node>
{
  const Directory directory;

  template <typename Node>
  explicit CatalogLocation(const Node & node, Scope & outer_scope)
  : directory(readElement<Directory>("Directory", node, outer_scope))
  {
    if (not boost::filesystem::exists(directory.path)) {
      THROW_SYNTAX_ERROR(directory.path.string() + " is not found.");
    }
    if (not boost::filesystem::is_directory(directory.path)) {
      THROW_SYNTAX_ERROR(directory.path.string() + " is not directory.");
    }

    for (auto path : Directory::ls(directory)) {
      if (path.extension() == ".yaml") {
        path = convertScenario(
          path, boost::filesystem::path("/tmp/converted_scenario") / directory.path.filename());
      } else if (path.extension() != ".xosc") {
        continue;
      }
      auto new_document = std::make_shared<pugi::xml_document>();
      new_document->load_file(path.string().c_str());
      catalog_files.emplace_back(std::move(new_document));
    }

    for (auto & xml : catalog_files) {
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

private:
  std::vector<std::shared_ptr<pugi::xml_document>> catalog_files;

  static boost::filesystem::path convertScenario(
    const boost::filesystem::path & yaml_path, const boost::filesystem::path & output_dir)
  {
    static auto conversion_py_path =
      "src/simulator/scenario_simulator/openscenario/openscenario_utility/openscenario_utility/"
      "conversion.py";

    std::stringstream command;
    command << "python3 " << conversion_py_path << " --input " << yaml_path << " --output "
            << output_dir;

    if (std::system(command.str().c_str()) != 0) {
      THROW_SYNTAX_ERROR("failed to convert sceanrio: " + yaml_path.string());
    }

    return output_dir / yaml_path.filename().stem().replace_extension(".xosc");
  }
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__CATALOG_LOCATION_HPP_
