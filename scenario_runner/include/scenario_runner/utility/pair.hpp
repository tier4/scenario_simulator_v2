#ifndef SCENARIO_RUNNER__UTILITY__PAIR_HPP_
#define SCENARIO_RUNNER__UTILITY__PAIR_HPP_

#include <utility>

namespace scenario_runner
{inline namespace utility
{
auto car = [] (auto && pare) noexcept->decltype(auto)
{
  return std::get<0>(pare);
};

auto cdr = [] (auto && pare) noexcept->decltype(auto)
{
  return std::get<1>(pare);
};

  #define COMPOSE(NAME, F, G) \
  template<typename ... Ts> \
  constexpr decltype(auto) NAME(Ts && ... xs) \
  { \
    return F(G(std::forward<decltype(xs)>(xs)...)); \
  } static_assert(true, "")

COMPOSE(caar, car, car);
COMPOSE(cadr, car, cdr);
COMPOSE(cdar, cdr, car);
COMPOSE(cddr, cdr, cdr);

  #undef COMPOSE
}}  // namespace scenario_runner::utility

#endif  // SCENARIO_RUNNER__UTILITY__PAIR_HPP_
