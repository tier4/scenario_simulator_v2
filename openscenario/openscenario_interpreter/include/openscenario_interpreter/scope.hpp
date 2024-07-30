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

#ifndef OPENSCENARIO_INTERPRETER__SCOPE_HPP_
#define OPENSCENARIO_INTERPRETER__SCOPE_HPP_

#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/range/algorithm.hpp>
#include <functional>
#include <memory>
#include <openscenario_interpreter/name.hpp>
#include <openscenario_interpreter/syntax/catalog_locations.hpp>
#include <openscenario_interpreter/syntax/entity.hpp>
#include <openscenario_interpreter/utility/demangle.hpp>
#include <unordered_map>
#include <utility>
#include <vector>

namespace openscenario_interpreter
{
class EnvironmentFrame
{
  friend struct Scope;

  std::multimap<std::string, Object> variables;  // NOTE: must be ordered.

  EnvironmentFrame * const outer_frame = nullptr;

  std::multimap<std::string, EnvironmentFrame *> inner_frames;  // NOTE: must be ordered.

  std::vector<EnvironmentFrame *> unnamed_inner_frames;

#define DEFINE_SYNTAX_ERROR(TYPENAME, ...)                                                       \
  template <typename T>                                                                          \
  struct TYPENAME : public SyntaxError                                                           \
  {                                                                                              \
    explicit TYPENAME(const std::string & variable)                                              \
    : SyntaxError(__VA_ARGS__, std::quoted(variable), " of type ", makeTypename(typeid(T)), ".") \
    {                                                                                            \
    }                                                                                            \
  }

  DEFINE_SYNTAX_ERROR(AmbiguousReferenceTo, "Ambiguous reference to ");
  DEFINE_SYNTAX_ERROR(NoSuchVariableNamed, "No such variable named ");

#undef DEFINE_SYNTAX_ERROR

  EnvironmentFrame() = default;

  explicit EnvironmentFrame(EnvironmentFrame &, const std::string &);

public:
  explicit EnvironmentFrame(const EnvironmentFrame &) = delete;

  explicit EnvironmentFrame(EnvironmentFrame &&) = delete;

  auto define(const Name &, const Object &) -> void;

  template <typename T>
  auto find(const Name & name) const -> Object
  {
    // NOTE: breadth first search
    for (std::vector<const EnvironmentFrame *> frames{this}; not frames.empty();) {
      auto objects = [&]() {
        std::vector<Object> result;
        for (auto && frame : frames) {
          boost::range::for_each(frame->variables.equal_range(name), [&](auto && name_and_value) {
            return result.push_back(name_and_value.second);
          });
        }
        return result;
      }();

      switch (boost::range::count_if(objects, is_also<T>())) {
        case 0:
          frames = [&]() {
            std::vector<const EnvironmentFrame *> result;
            for (auto && current_frame : frames) {
              boost::range::copy(current_frame->unnamed_inner_frames, std::back_inserter(result));
            }
            return result;
          }();
          break;
        case 1:
          return *boost::range::find_if(objects, is_also<T>());
        default:
          throw AmbiguousReferenceTo<T>(name);
      }
    }

    return isOutermost() ? throw NoSuchVariableNamed<T>(name) : outer_frame->find<T>(name);
  }

  template <typename T>
  auto find(const Prefixed<Name> & prefixed_name) const -> Object
  {
    if (not prefixed_name.prefixes.empty()) {
      const auto found = resolvePrefix(prefixed_name);
      switch (found.size()) {
        case 0:
          throw NoSuchVariableNamed<T>(boost::lexical_cast<std::string>(prefixed_name));
        case 1:
          return found.front()->find<T>(prefixed_name.strip<1>());
        default:
          throw AmbiguousReferenceTo<T>(boost::lexical_cast<std::string>(prefixed_name));
      }
    } else {
      return find<T>(prefixed_name.name);
    }
  }

  template <typename T>
  auto ref(const Prefixed<Name> & prefixed_name) const -> Object
  {
    if (prefixed_name.absolute) {
      return outermostFrame().find<T>(prefixed_name);
    } else if (prefixed_name.prefixes.empty()) {
      return find<T>(prefixed_name.name);
    } else {
      return lookupFrame(prefixed_name)->find<T>(prefixed_name.strip<1>());
    }
  }

  auto isOutermost() const noexcept -> bool;

private:
  auto resolvePrefix(const Prefixed<Name> &) const -> std::list<const EnvironmentFrame *>;

  auto lookupFrame(const Prefixed<Name> &) const -> const EnvironmentFrame *;

  auto outermostFrame() const noexcept -> const EnvironmentFrame &;
};

inline namespace syntax
{
struct Entities;

struct OpenScenario;
}  // namespace syntax

class Scope
{
  /*
     In OpenSCENARIO, global resources are FileHeader, top-level
     ParameterDeclaration, CatalogLocations, RoadNetwork, and Entities (These
     are located directly under the `OpenSCENARIO` tag).

     The `Scope` data member `open_scenario` is provided to reference those
     global resources. However, some constructors need access to global
     resources during the construction of the OpenScenario class (corresponding
     to the OpenSCENARIO tag) for processing order reasons. Here, the data
     member `open_scenario` is a null pointer until the
     `syntax::ScenarioDefinition` construction is complete. The inner class
     `Scope::ScenarioDefinition` is provided to deal with this problem, which
     implementors understand to be a DIRTY HACK.

     A fundamental solution to this problem will require a reworking of
     CatalogReference, which is still in the pilot implementation stage at this
     time, but the status quo will be maintained for the time being due to its
     wide impact. If you want to access global resources via `Scope`, we
     recommend that you go through the data member `open_scenario` instead of
     `Scope::ScenarioDefinition` whenever possible.
  */

  struct ScenarioDefinition
  {
    const Entities * entities = nullptr;

    const CatalogLocations * catalog_locations = nullptr;
  };

  const OpenScenario * const open_scenario;

  const std::shared_ptr<EnvironmentFrame> frame;

  const std::shared_ptr<ScenarioDefinition> scenario_definition;

public:
  const std::string name;

  std::list<Entity> actors;

  double seed;  // NOTE: `seed` is used only for sharing randomSeed in Stochastic now

  Scope() = delete;

  Scope(const Scope &) = default;  // NOTE: shallow copy

  Scope(Scope &&) = default;

  explicit Scope(const OpenScenario * const);

  explicit Scope(const std::string &, const Scope &);

  auto dirname() const -> std::string;

  template <typename... Ts>
  auto ref(Ts &&... xs) const -> decltype(auto)
  {
    return frame->ref<Object>(std::forward<decltype(xs)>(xs)...);
  }

  template <typename T, typename... Ts>
  auto ref(Ts &&... xs) const -> decltype(auto)
  {
    return frame->ref<T>(std::forward<decltype(xs)>(xs)...).template as<T>();
  }

  auto global() const -> const ScenarioDefinition &;

  auto global() -> ScenarioDefinition &;

  auto local() const noexcept -> const Scope &;

  auto local() noexcept -> Scope &;

  auto insert(const Name &, const Object &) -> void;
};
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SCOPE_HPP_
