// Copyright 2015-2020 TierIV.inc. All rights reserved.
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

#include <sstream>
#include <stdexcept>
#include <string>

namespace openscenario_interpreter
{
/* ==== Error ================================================================
 *
 * -- Error
 *     |-- SyntaxError
 *     |-- ConnectionError
 *     |-- SemanticError
 *     `-- ImplementationFault
 *
 * ======================================================================== */
struct Error
  : public std::runtime_error
{
  using std::runtime_error::runtime_error;
};

struct SyntaxError
  : public Error
{
  explicit SyntaxError(const std::string & s)
  : Error{"syntax-error: " + s}
  {}
};

struct SemanticError
  : public Error
{
  explicit SemanticError(const std::string & s)
  : Error{"semantic-error: " + s}
  {}
};

struct ConnectionError
  : public Error
{
  explicit ConnectionError(const std::string & s)
  : Error{"connection-error: " + s}
  {}
};

struct ImplementationFault
  : public Error
{
  explicit ImplementationFault(const std::string & s)
  : Error{"implementation-fault: " + s}
  {}
};

#define THROW(TYPENAME) \
  do { \
    std::stringstream ss {}; \
    ss << __FILE__ << ":" << __LINE__; \
    throw TYPENAME {ss.str()}; \
  } while (false)

#define THROW_IMPLEMENTATION_FAULT() THROW(ImplementationFault)

#define UNIMPLEMENTED(NAME) \
  do { \
    std::stringstream ss {}; \
    ss << "given class \'" << NAME << \
      "\' is valid OpenSCENARIO element, but is not yet implemented"; \
    throw ImplementationFault {ss.str()}; \
  } while (false)

#define THROW_UNSUPPORTED_ERROR(PARENT) \
  [&](auto && child) \
  { \
    std::stringstream ss {}; \
    ss << "given class \'" << child.name() << "\' (element of class \'" << PARENT.name() << \
      "\') is valid OpenSCENARIO element, but is not supported"; \
    throw SyntaxError(ss.str()); \
    return unspecified; \
  }

#define UNSUPPORTED() \
  [&](auto && node) \
  { \
    std::stringstream ss {}; \
    ss << "given class \'" << node.name() << \
      " is valid OpenSCENARIO element, but is not supported"; \
    throw SyntaxError(ss.str()); \
    return unspecified; \
  }
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__ERROR_HPP_
