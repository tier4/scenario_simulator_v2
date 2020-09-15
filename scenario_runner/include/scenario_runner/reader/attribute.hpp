#ifndef SCENARIO_RUNNER__READER__ATTRIBUTE_HPP_
#define SCENARIO_RUNNER__READER__ATTRIBUTE_HPP_

#include <scenario_runner/syntax/parameter_type.hpp>
#include <scenario_runner/utility/highlighter.hpp>
#include <scenario_runner/utility/pugi_extension.hpp>

namespace scenario_runner { inline namespace reader
{
  // TODO
  // T readAttribute(const std::string& name, const Node& node, const Scope& scope)
  template <typename T, typename Node, typename Scope>
  T readAttribute(const Node& node, const Scope& scope, const std::string& name)
  {
    if (const auto& attribute { node.attribute(name.c_str()) })
    {
      const std::string value { attribute.value() };

      if (value.empty())
      {
      #ifndef SCENARIO_RUNNER_ALLOW_ATTRIBUTES_TO_BE_BLANK
        std::stringstream ss {};
        ss << "blank is not allowed for the value of attribute \'" << name << "\' of class \'" << node.name() << "\'";
        throw SyntaxError { ss.str() };
      #else
        return T {};
      #endif
      }
      else if (value.front() == '$')
      {
        const auto iter { scope.parameters.find(value.substr(1)) };

        if (iter != std::end(scope.parameters))
        {
          return std::get<1>(*iter).template as<T>();
        }
        else
        {
          std::stringstream ss {};
          ss << "there is no parameter named '" << value.substr(1) << "' (attribute \'" << name << "\' of class \'" << node.name() << "\' references this parameter)";
          throw SyntaxError { ss.str() };
        }
      }
      else try
      {
        return boost::lexical_cast<T>(value);
      }
      catch (const boost::bad_lexical_cast&)
      {
        std::stringstream ss {};
        ss << "value \"" << value << "\" specified for attribute \'" << name << "\' is invalid (not value of type " << typeid(T).name() << ")";
        throw SyntaxError { ss.str() };
      }
    }
    else
    {
      std::stringstream ss {};
      ss << "required attribute \'" << name << "\' not specified for class \'" << node.name() << "\'";
      throw SyntaxError { ss.str() };
    }
  }

  // TODO
  // T readAttribute(const std::string& name, const Node& node, const Scope& scope)
  template <typename T, typename Node, typename Scope>
  T readAttribute(const Node& node, const Scope& scope, const std::string& name, T&& value)
  {
    if (node.attribute(name.c_str()))
    {
      return readAttribute<T>(node, scope, name);
    }
    else
    {
      return value;
    }
  }
}}  // namespace scenario_runner::reader

#endif  // SCENARIO_RUNNER__READER__ATTRIBUTE_HPP_
