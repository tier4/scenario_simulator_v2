#ifndef SCENARIO_RUNNER__TYPE_TRAITS__IF_ACCOMPLISHABLE_HPP_
#define SCENARIO_RUNNER__TYPE_TRAITS__IF_ACCOMPLISHABLE_HPP_

#include <scenario_runner/concepts/accomplishable.hpp>

namespace scenario_runner { inline namespace type_traits
{
  template <typename T, typename = void>
  struct IfAccomplishable
  {
    static constexpr auto invoke(const T&) noexcept
    {
      return false;
    }
  };

  template <typename T>
  struct IfAccomplishable<T, typename std::enable_if<Accomplishable<T>::value>::type>
  {
    static decltype(auto) invoke(T& callee)
    {
      return callee.accomplished();
    }
  };
}}  // namespace scenario_runner::type_traits

#endif  // SCENARIO_RUNNER__TYPE_TRAITS__IF_ACCOMPLISHABLE_HPP_
