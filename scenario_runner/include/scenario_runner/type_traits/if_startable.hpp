#ifndef SCENARIO_RUNNER__TYPE_TRAITS__IF_STARTABLE_HPP_
#define SCENARIO_RUNNER__TYPE_TRAITS__IF_STARTABLE_HPP_

#include <scenario_runner/concepts/startable.hpp>

namespace scenario_runner
{inline namespace type_traits
{
template<typename T, typename = void>
struct IfStartable
{
  static constexpr void invoke(const T &) noexcept
  {}
};

template<typename T>
struct IfStartable<T, typename std::enable_if<Startable<T>::value>::type>
{
  static decltype(auto) invoke(T & callee)
  {
    return callee.start();
  }
};
}}  // namespace scenario_runner::type_traits

#endif  // SCENARIO_RUNNER__TYPE_TRAITS__IF_STARTABLE_HPP_
