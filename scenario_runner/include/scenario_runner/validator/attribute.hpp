// Copyright 2015-2020 Autoware Foundation. All rights reserved.
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

#ifndef SCENARIO_RUNNER__VALIDATOR__ATTRIBUTE_HPP_
#define SCENARIO_RUNNER__VALIDATOR__ATTRIBUTE_HPP_

#include <scenario_runner/syntax/parameter_type.hpp>
#include <scenario_runner/utility/pugi_extension.hpp>

namespace scenario_runner
{inline namespace validator
{
template<typename T>
T readUnsupportedAttribute(
  const pugi::xml_node & node, const std::string & name,
  const T & value = {})
{
  if (node.attribute(name.c_str())) {
    std::stringstream ss {};
    ss << "attribute \'" << name << "\' specified for class \'" << node.name() <<
      "\' is valid OpenSCENARIO element, but is not supported";
    throw SyntaxError {ss.str()};
  } else {
    return value;
  }
}

template<typename T>
T readRequiredAttribute(const pugi::xml_node & node, const std::string & name)
{
  if (const auto x {node.attribute(name.c_str())}) {
    return boost::lexical_cast<T>(x.value());
  } else {
    std::stringstream ss {};
    ss << "required attribute \'" << name << "\' not specified for class \'" << node.name() << "\'";
    throw SyntaxError {ss.str()};
  }
}

template<typename T>
T readOptionalAttribute(const pugi::xml_node & node, const std::string & name, const T & value = {})
{
  if (const auto & x {node.attribute(name.c_str())}) {
    return boost::lexical_cast<T>(x.value());
  } else {
    return value;
  }
}
}}  // namespace scenario_runner::validator

#endif  // SCENARIO_RUNNER__VALIDATOR__ATTRIBUTE_HPP_
