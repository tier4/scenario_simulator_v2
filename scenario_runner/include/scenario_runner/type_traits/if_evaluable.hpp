#ifndef SCENARIO_RUNNER__TYPE_TRAITS__IF_EVALUABLE_HPP_
#define SCENARIO_RUNNER__TYPE_TRAITS__IF_EVALUABLE_HPP_

#include <scenario_runner/concepts/evaluable.hpp>

namespace scenario_runner { inline namespace type_traits
{
  template <typename T, typename = void>
  struct IfEvaluable
  {
    template <typename Result, typename U, typename... Us>
    static Result invoke(const Result& as_is, U&&, Us&&...)
    {
      return as_is;
    }
  };

  template <typename T>
  struct IfEvaluable<T, typename std::enable_if<Evaluable<T>::value>::type>
  {
    template <typename Result, typename U, typename... Us>
    static Result invoke(const Result&, U& object, Us&&... xs)
    {
      return object.evaluate(std::forward<decltype(xs)>(xs)...);
    }
  };
}}  // namespace scenario_runner::type_traits

#endif  // SCENARIO_RUNNER__TYPE_TRAITS__IF_EVALUABLE_HPP_


