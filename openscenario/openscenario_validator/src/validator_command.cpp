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

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <openscenario_validator/validator.hpp>

int main(int argc, char * argv[])
{
  assert(argc == 2);
  std::string file_path = argv[1];
  std::string schema_path = ament_index_cpp::get_package_share_directory("openscenario_validator") +
                            "/schema/OpenSCENARIO-1.3.xsd";
  openscenario_validator::OpenSCENARIOValidator validate(schema_path);
  try {
    validate(file_path);
    return 0;
  } catch (const std::exception & e) {
    std::cerr << e.what() << std::endl;
    return 1;
  }
}
