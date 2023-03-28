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

#include <boost/math/constants/constants.hpp>
#include <openscenario_interpreter/reader/attribute.hpp>
#include <openscenario_interpreter/reader/element.hpp>
#include <openscenario_interpreter/syntax/dome_image.hpp>
#include <openscenario_interpreter/syntax/file.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
DomeImage::DomeImage(const pugi::xml_node & node, Scope & scope)
: azimuth_offset(readAttribute<Double>("azimuthOffset", node, scope, 0)),
  dome_file(readElement<File>("DomeFile", node, scope))
{
  // Valid range ref:
  // https://www.asam.net/static_downloads/ASAM_OpenSCENARIO_V1.2.0_Model_Documentation/modelDocumentation/content/DomeImage.html
  auto azimuth_offset_valid =
    0 <= azimuth_offset and azimuth_offset <= boost::math::constants::pi<Double>();
  if (!azimuth_offset_valid) {
    THROW_SYNTAX_ERROR(std::quoted("azimuthOffset"), "is out of range [0..2*PI]");
  }
}
}  // namespace syntax
}  // namespace openscenario_interpreter
