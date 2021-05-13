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

#ifndef OPENSCENARIO_INTERPRETER__ERROR_HPP_
#define OPENSCENARIO_INTERPRETER__ERROR_HPP_

#include <openscenario_interpreter/string/cat.hpp>
#include <stdexcept>
#include <string>
#include <utility>

namespace openscenario_interpreter
{
/* ---- NOTE -------------------------------------------------------------------
 *
 *  -- Error
 *      |-- SyntaxError
 *      |    `-- InvalidEnumeration
 *      |-- SemanticError
 *      `-- ImplementationFault
 *
 * -------------------------------------------------------------------------- */
struct Error : public std::runtime_error
{
  template <typename... Ts>
  explicit constexpr Error(Ts &&... xs)
  : std::runtime_error(cat(std::forward<decltype(xs)>(xs)..., "."))
  {
  }
};

struct SyntaxError : public Error
{
  template <typename... Ts>
  explicit constexpr SyntaxError(Ts &&... xs)
  : Error("SyntaxError: ", std::forward<decltype(xs)>(xs)...)
  {
  }

  static decltype(auto) invalidValue(const std::string & type, const std::string & value)
  {
    return SyntaxError("An invalid value '", value, "' was specified for type '", type);
  };
};

struct SemanticError : public Error
{
  template <typename... Ts>
  explicit SemanticError(Ts &&... xs) : Error("SemanticError: ", std::forward<decltype(xs)>(xs)...)
  {
  }
};

struct [[deprecated]] ConnectionError : public Error{
  template <typename... Ts>
  explicit ConnectionError(Ts && ... xs) :
    Error("connection-error: ", std::forward<decltype(xs)>(xs)...){}
};

struct ImplementationFault : public Error
{
  template <typename... Ts>
  explicit ImplementationFault(Ts &&... xs)
  : Error("ImplementationFault: ", std::forward<decltype(xs)>(xs)...)
  {
  }
};

#define THROW(TYPENAME)                \
  do {                                 \
    std::stringstream ss{};            \
    ss << __FILE__ << ":" << __LINE__; \
    throw TYPENAME{ss.str()};          \
  } while (false)

#define THROW_IMPLEMENTATION_FAULT() THROW(ImplementationFault)

#define UNIMPLEMENTED(NAME)                                               \
  do {                                                                    \
    std::stringstream ss{};                                               \
    ss << "given class \'" << NAME                                        \
       << "\' is valid OpenSCENARIO element, but is not yet implemented"; \
    throw ImplementationFault{ss.str()};                                  \
  } while (false)

#define THROW_UNSUPPORTED_ERROR(PARENT)                                                  \
  [&](auto && child) {                                                                   \
    std::stringstream ss{};                                                              \
    ss << "given class \'" << child.name() << "\' (element of class \'" << PARENT.name() \
       << "\') is valid OpenSCENARIO element, but is not supported";                     \
    throw SyntaxError(ss.str());                                                         \
    return unspecified;                                                                  \
  }

#define UNSUPPORTED()                                             \
  [&](auto && node) {                                             \
    std::stringstream ss{};                                       \
    ss << "given class \'" << node.name()                         \
       << " is valid OpenSCENARIO element, but is not supported"; \
    throw SyntaxError(ss.str());                                  \
    return unspecified;                                           \
  }
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__ERROR_HPP_
