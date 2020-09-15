#ifndef SCENARIO_RUNNER__TYPE_TRAITS__VOID_T_HPP
#define SCENARIO_RUNNER__TYPE_TRAITS__VOID_T_HPP

#include <type_traits>

namespace scenario_runner { inline namespace type_traits
{
  template <typename...>
  using void_t = void;
}}  // namespace scenario_runner::type_traits

#endif  // SCENARIO_RUNNER__TYPE_TRAITS__VOID_T_HPP

