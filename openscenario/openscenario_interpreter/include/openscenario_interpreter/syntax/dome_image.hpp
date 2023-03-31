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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__DOME_IMAGE_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__DOME_IMAGE_HPP_

#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/double.hpp>
#include <openscenario_interpreter/syntax/file.hpp>
#include <pugixml.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- DomeImage 1.2 --------------------------------------------------------
 *
 * <xsd:complexType name="DomeImage">
 *   <xsd:sequence>
 *     <xsd:element name="DomeFile" type="File"/>
 *   </xsd:sequence>
 *   <xsd:attribute name="azimuthOffset" type="Double"/>
 * </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct DomeImage
{
  const Double azimuth_offset;  // Offset to north / y-axis of world coordinate system
                                // (counter-clockwise). Unit: [rad]. Range: [0..2*PI]. 0 means the
                                // left and right borders of the image are aligned with the y-axis
                                // of the world coordinate system. Default if omitted: 0.

  const File dome_file;  // Filepath to the dome file.

  DomeImage() = default;

  explicit DomeImage(const pugi::xml_node &, Scope &);
};

}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__DOME_IMAGE_HPP_
