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

#ifndef OPENSCENARIO_INTERPRETER__READER__ATTRIBUTE_HPP_
#define OPENSCENARIO_INTERPRETER__READER__ATTRIBUTE_HPP_

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <openscenario_interpreter/syntax/parameter_type.hpp>
#include <openscenario_interpreter/utility/highlighter.hpp>
#include <openscenario_interpreter/utility/pugi_extension.hpp>

#include <functional>
#include <regex>
#include <string>
#include <unordered_map>

namespace openscenario_interpreter
{
inline namespace reader
{
/* ---- Dynamic Configuration --------------------------------------------------
 *
 * See https://design.ros2.org/articles/roslaunch_xml.html#dynamic-configuration
 *
 * -------------------------------------------------------------------------- */
template<typename Scope>
auto substitute(std::string attribute, Scope & scope)
{
  static const std::regex substitution_syntax {
    R"((.*)\$\((([\w-]+)\s?([^\)]*))\)(.*))"
  };

  std::smatch match {};

  while (std::regex_match(attribute, match, substitution_syntax)) {
    // std::cout << "match 1: " << match.str(1) << std::endl;
    // std::cout << "match 2: " << match.str(2) << std::endl;
    // std::cout << "match 3: " << match.str(3) << std::endl;
    // std::cout << "match 4: " << match.str(4) << std::endl;
    // std::cout << "match 5: " << match.str(5) << std::endl;

    static const std::unordered_map<
      std::string,
      std::function<std::string(const std::string &, Scope &)>
    >
    substitutions
    {
      {
        "find-pkg-share", [](auto && package_name, auto &&)
        {
          return ament_index_cpp::get_package_share_directory(package_name);
        }
      },

      {
        "var", [](auto && name, auto && scope) -> String
        {
          const auto iter {
            scope.parameters.find(name)
          };

          if (iter != std::end(scope.parameters)) {
            return boost::lexical_cast<String>(std::get<1>(*iter));
          } else {
            return "";
          }
        }
      },

      {
        "dirname", [](auto &&, auto && scope)
        {
          return scope.scenario.parent_path().string();
        }
      }
    };

    // std::cout << "Substitute: " << cyan << attribute << reset << " => ";

    const auto iter {
      substitutions.find(match.str(3))
    };

    if (iter != std::end(substitutions)) {
      attribute = match.str(1) + std::get<1>(* iter)(match.str(4), scope) + match.str(5);
    } else {
      std::stringstream ss {};
      ss << "Unknown substitution '" << match.str(3) << "' specified.";
      throw SyntaxError(ss.str());
    }

    // std::cout << cyan << attribute << reset << std::endl;
  }

  return attribute;
}

template
<
  typename T,
  typename Node,
  typename Scope
>
T readAttribute(const std::string & name, const Node & node, const Scope & scope)
{
  if (const auto & attribute {
        node.attribute(name.c_str())
      })
  {
    std::string value {
      substitute(attribute.value(), scope)
    };

    if (value.empty()) {
      #ifndef OPENSCENARIO_INTERPRETER_ALLOW_ATTRIBUTES_TO_BE_BLANK
      std::stringstream ss {};
      ss << "Blank is not allowed for the value of attribute \'" << name << "\' of class \'" <<
        node.name() << "\'";
      throw SyntaxError {ss.str()};
      #else
      return T {};
      #endif
    } else if (value.front() == '$') {
      const auto iter {
        scope.parameters.find(value.substr(1))
      };
      if (iter != std::end(scope.parameters)) {
        return boost::lexical_cast<T>(boost::lexical_cast<String>(cdr(*iter)));
      } else {
        std::stringstream ss {};
        ss << "There is no parameter named '" << value.substr(1) << "' (Attribute \'" << name;
        ss << "\' of class \'" << node.name() << "\' references this parameter)";
        throw SyntaxError(ss.str());
      }
    } else {
      try {
        return boost::lexical_cast<T>(value);
      } catch (const boost::bad_lexical_cast &) {
        std::stringstream ss {};
        ss << "Value \"" << value << "\" specified for attribute \'" << name;
        ss << "\' is invalid (Is not value of type " << typeid(T).name() << ")";
        throw SyntaxError(ss.str());
      }
    }
  } else {
    std::stringstream ss {};
    ss << "Required attribute \'" << name << "\' not specified for class \'" << node.name() << "\'";
    throw SyntaxError {ss.str()};
  }
}

template
<
  typename T,
  typename Node,
  typename Scope
>
T readAttribute(const std::string & name, const Node & node, const Scope & scope, T && value)
{
  if (node.attribute(name.c_str())) {
    return readAttribute<T>(name, node, scope);
  } else {
    return value;
  }
}
}
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__READER__ATTRIBUTE_HPP_
