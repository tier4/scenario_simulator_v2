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

#include <iomanip>
#include <openscenario_interpreter/utility/demangle.hpp>
#include <scenario_simulator_exception/exception.hpp>
#include <stdexcept>

#define LINE() \
  std::cout << "; \x1b[33m" __FILE__ "\x1b[31m:\x1b[36m" << __LINE__ << "\x1b[0m" << std::endl

#define PRINT(...) \
  std::cout << "; " #__VA_ARGS__ " = " << std::boolalpha << (__VA_ARGS__) << std::endl

namespace openscenario_interpreter
{
using InternalError = std::exception;

using common::AutowareError;
using common::Error;
using common::SemanticError;
using common::SimulationError;
using common::SyntaxError;

#define INVALID_NUMERIC_LITERAL_SPECIFIED(VALUE)                                       \
  SyntaxError(                                                                         \
    "Given value ", std::quoted(VALUE), " is not an external representation of type ", \
    demangle(typeid(*this)))

#define UNSUPPORTED_ENUMERATION_VALUE_SPECIFIED(TYPE, VALUE) \
  SyntaxError(                                               \
    "Given value ", std::quoted(VALUE),                      \
    " is valid OpenSCENARIO value of type " #TYPE ", but it is not supported yet")

#define UNEXPECTED_ENUMERATION_VALUE_SPECIFIED(TYPE, VALUE) \
  SyntaxError("Unexpected value ", std::quoted(VALUE), " of type " #TYPE " was specified")

#define UNEXPECTED_ENUMERATION_VALUE_ASSIGNED(TYPE, VALUE) \
  SyntaxError(                                             \
    "Unexpected value ", static_cast<TYPE::value_type>(VALUE), " was assigned to type " #TYPE)

#define UNSUPPORTED_ELEMENT_SPECIFIED(ELEMENT)                                                    \
  SyntaxError(                                                                                    \
    "Given class ", ELEMENT, " is valid OpenSCENARIO element of class ", demangle(typeid(*this)), \
    ", but is not supported yet")

#define UNSUPPORTED_CONVERSION_DETECTED(FROM, TO) \
  SyntaxError("Converting " #FROM " to " #TO      \
              ". This is valid in OpenSCENARIO standard, but is not yet supported")

#define UNSUPPORTED_SETTING_DETECTED(ACTION_OR_CONDITION, ELEMENT) \
  SyntaxError(#ACTION_OR_CONDITION " does not yet support ", ELEMENT)
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__ERROR_HPP_
