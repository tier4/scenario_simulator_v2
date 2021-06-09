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

#ifndef OPENSCENARIO_INTERPRETER__SCOPE_HPP_
#define OPENSCENARIO_INTERPRETER__SCOPE_HPP_

#include <boost/filesystem.hpp>
#include <functional>  // std::reference_wrapper
#include <limits>
#include <memory>
#include <openscenario_interpreter/syntax/entity_ref.hpp>
#include <openscenario_interpreter/syntax/traffic_signal_controller.hpp>
#include <unordered_map>
#include <utility>
#include <vector>

namespace openscenario_interpreter
{
/* ---- NOTE -------------------------------------------------------------------
 *
 *  This structure provides resource access during the scenario at each level of
 *  Storyboard.
 *
 *  Typically, in programming language implementations, scopes are implemented
 *  as linked lists that hold pointers to outer scopes. However, OpenSCENARIO
 *  does not have features that require dynamic frame construction like function
 *  calls, and the structure is fixed at the time of parsing, so it is okay to
 *  build the child scope as a copy of the parent scope.
 *
 *  In other words, this structure is not elegant, but I believe it is a simple
 *  structure that even beginners of programming language implementations can
 *  understand.
 *
 * -------------------------------------------------------------------------- */
struct Scope
{
  using Actor = EntityRef;

  using Actors = std::list<Actor>;

  /* ---- GLOBAL ------------------------------------------------------------ */

  const boost::filesystem::path pathname;  // for substituation syntax '$(dirname)'

  boost::filesystem::path logic_file;  // NOTE: Assigned by RoadNetwork's constructor.

  boost::filesystem::path scene_graph_file;  // NOTE: Assigned by RoadNetwork's constructor.

  /* ---- NOTE -----------------------------------------------------------------
   *
   *  for TrafficSignalControllerAction.trafficSignalControllerRef
   *
   *  Be careful not to use the TrafficSignalController as a dangling reference.
   *  Normally, the destruction of all scopes occurs at the same time as the
   *  destruction of the scenario itself.
   *
   * ------------------------------------------------------------------------ */
  std::unordered_map<String, std::reference_wrapper<TrafficSignalController>>
    traffic_signal_controller_refs;

  std::unordered_map<String, Element> entities;

  /* ---- LEXICAL ----------------------------------------------------------- */

  std::unordered_map<String, Element> parameters;

  std::unordered_map<String, Element> storyboard_elements;

  Actors actors;

  /* ---- CONSTRUCTORS ------------------------------------------------------ */

  Scope() = delete;

  explicit Scope(Scope &) = default;

  explicit Scope(const Scope &) = default;

  explicit Scope(const boost::filesystem::path & pathname) : pathname(pathname) {}
};
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SCOPE_HPP_
