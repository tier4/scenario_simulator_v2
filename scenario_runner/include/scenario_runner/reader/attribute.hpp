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

#ifndef SCENARIO_RUNNER__READER__ATTRIBUTE_HPP_
#define SCENARIO_RUNNER__READER__ATTRIBUTE_HPP_

#include <scenario_runner/syntax/parameter_type.hpp>
#include <scenario_runner/utility/highlighter.hpp>
#include <scenario_runner/utility/pugi_extension.hpp>

#include <string>

namespace scenario_runner
{
inline namespace reader
{
template<typename T, typename Node, typename Scope>
T readAttribute(const std::string & name, const Node & node, const Scope & scope)
{
  if (const auto & attribute {node.attribute(name.c_str())}) {
    const std::string value {attribute.value()};

    if (value.empty()) {
      #ifndef SCENARIO_RUNNER_ALLOW_ATTRIBUTES_TO_BE_BLANK
      std::stringstream ss {};
      ss << "blank is not allowed for the value of attribute \'" << name << "\' of class \'" <<
        node.name() << "\'";
      throw SyntaxError {ss.str()};
      #else
      return T {};
      #endif
    } else if (value.front() == '$') {
      const auto iter {scope.parameters.find(value.substr(1))};

      if (iter != std::end(scope.parameters)) {
        return std::get<1>(*iter).template as<T>();
      } else {
        std::stringstream ss {};
        ss << "there is no parameter named '" << value.substr(1) << "' (attribute \'" << name <<
          "\' of class \'" << node.name() << "\' references this parameter)";
        throw SyntaxError {ss.str()};
      }
    } else {
      try {
        return boost::lexical_cast<T>(value);
      } catch (const boost::bad_lexical_cast &) {
        std::stringstream ss {};
        ss << "value \"" << value << "\" specified for attribute \'" << name <<
          "\' is invalid (not value of type " << typeid(T).name() << ")";
        throw SyntaxError {ss.str()};
      }
    }
  } else {
    std::stringstream ss {};
    ss << "required attribute \'" << name << "\' not specified for class \'" << node.name() << "\'";
    throw SyntaxError {ss.str()};
  }
}

template<typename T, typename Node, typename Scope>
T readAttribute(const std::string & name, const Node & node, const Scope & scope, T && value)
{
  if (node.attribute(name.c_str())) {
    return readAttribute<T>(name, node, scope);
  } else {
    return value;
  }
}
}
}  // namespace scenario_runner

#endif  // SCENARIO_RUNNER__READER__ATTRIBUTE_HPP_
