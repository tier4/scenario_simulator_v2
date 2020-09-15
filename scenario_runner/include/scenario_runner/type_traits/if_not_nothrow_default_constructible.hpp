#ifndef SCENARIO_RUNNER__SYNTAX__IF_NOT_NOTHROW_DEFAULT_CONSTRUCTIBLE_HPP_
#define SCENARIO_RUNNER__SYNTAX__IF_NOT_NOTHROW_DEFAULT_CONSTRUCTIBLE_HPP_

#include <type_traits>

namespace scenario_runner { inline namespace type_traits
{
  template <typename T, typename = void>
  struct IfNotNothrowDefaultConstructible
  {
    static T error(const std::string& parent_name, const std::string& child_name)
    {
      std::stringstream ss {};
      ss << parent_name << " requires class " << child_name << " as element, but there is no specification";
      throw SyntaxError { ss.str() };
    }
  };

  template <typename T>
  struct IfNotNothrowDefaultConstructible<T, typename std::enable_if<std::is_nothrow_default_constructible<T>::value>::type>
  {
    template <typename... Ts>
    static T error(Ts&&...)
    {
      return T {};
    }
  };
}}  // namespace scenario_runner::type_traits

#endif  // SCENARIO_RUNNER__SYNTAX__IF_NOT_NOTHROW_DEFAULT_CONSTRUCTIBLE_HPP_
