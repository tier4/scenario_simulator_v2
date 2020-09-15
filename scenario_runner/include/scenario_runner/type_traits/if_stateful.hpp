#ifndef SCENARIO_RUNNER__TYPE_TRAITS__IF_STATEFUL_HPP_
#define SCENARIO_RUNNER__TYPE_TRAITS__IF_STATEFUL_HPP_

#include <scenario_runner/concepts/stateful.hpp>

namespace scenario_runner { inline namespace type_traits
{
  template <typename T, typename = void>
  struct IfStateful
  {
    template <typename Result>
    static const Result& currentState(const T&)
    {
      std::stringstream ss {};
      ss << "class " << typeid(T).name() << " is not stateful";
      throw ImplementationFault { ss.str() };
    }
  };

  template <typename T>
  struct IfStateful<T, typename std::enable_if<Stateful<T>::value>::type>
  {
    template <typename Result>
    static const Result& currentState(const T& callee)
    {
      return callee.currentState();
    }
  };
}}  // namespace scenario_runner::type_traits

#endif  // SCENARIO_RUNNER__TYPE_TRAITS__IF_STATEFUL_HPP_
