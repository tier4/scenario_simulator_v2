#ifndef SCENARIO_RUNNER__VALIDATOR__CHOICE_HPP_
#define SCENARIO_RUNNER__VALIDATOR__CHOICE_HPP_

#include <scenario_runner/iterator/size.hpp>
#include <scenario_runner/validator/element.hpp>

namespace scenario_runner { inline namespace validator
{
  struct Choice
    : public Elements
  {
    template <typename... Ts>
    explicit Choice(Ts&&... xs)
      : Elements { std::forward<decltype(xs)>(xs)... }
    {}

    Object validate(const pugi::xml_node& node, Scope& scope)
    {
      if (empty())
      {
        std::stringstream ss {};
        ss << node.name() << " is defined as choice of elements, but no elements defined.";
        throw ImplementationFault { ss.str() };
      }
      else if (scenario_runner::iterator::size(node.children()) < 1)
      {
        std::stringstream ss {};

        ss << "class " << node.name() << " must have one of following classes as an element: (";

        for (auto iter { std::begin(*this) }; iter != std::end(*this); ++iter)
        {
          ss << std::get<0>(*iter) << (std::next(iter) != std::end(*this) ? " " : ")");
        }

        throw SyntaxError { ss.str() };
      }
      else if (1 < scenario_runner::iterator::size(node.children()))
      {
        std::stringstream ss {};

        ss << node.name() << " cannot have more than one of ";

        for (auto iter { std::begin(*this) }; iter != std::end(*this); ++iter)
        {
          ss << std::get<0>(*iter) << (std::next(iter) != std::end(*this) ? ", " : "");
        }

        ss << " at the same node";

        throw SyntaxError { ss.str() };
      }
      else
      {
        const std::string name { node.first_child().name() };

        const auto iter { find(name) };

        if (iter != std::end(*this))
        {
          return std::get<1>(*iter).emplace_back(node.first_child(), scope);
        }
        else
        {
          std::stringstream ss {};
          ss << "unknown class \'" << name << "\' specified for property of class " << node.name();
          throw SyntaxError { ss.str() };
        }
      }
    }
  };
}}  // namespace scenario_runner::validator

#endif  // SCENARIO_RUNNER__VALIDATOR__CHOICE_HPP_
