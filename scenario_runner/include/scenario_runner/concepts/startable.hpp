#ifndef SCENARIO_RUNNER__CONCEPTS__STARTABLE_HPP_
#define SCENARIO_RUNNER__CONCEPTS__STARTABLE_HPP_

#include <scenario_runner/type_traits/void_t.hpp>

namespace scenario_runner { inline namespace concepts
{
  template <typename T, typename = void>
  struct Startable
    : public std::false_type
  {};

  template <typename T>
  struct Startable<T, void_t<decltype(std::declval<T>().start())>>
    : public std::true_type
  {};
}}  // namespace scenario_runner::concepts

#endif  // SCENARIO_RUNNER__CONCEPTS__STARTABLE_HPP_
