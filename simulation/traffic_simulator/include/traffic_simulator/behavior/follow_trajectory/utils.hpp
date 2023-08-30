#ifndef TRAFFIC_SIMULATOR__BEHAVIOR__UTILS_HPP_
#define TRAFFIC_SIMULATOR__BEHAVIOR__UTILS_HPP_

#include <geometry/vector3/is_like_vector3.hpp>

#include <cmath>

namespace traffic_simulator
{
namespace follow_trajectory
{
template <typename F, typename T, typename... Ts>
auto any(F f, T && x, Ts &&... xs)
{
  if constexpr (math::geometry::IsLikeVector3<std::decay_t<decltype(x)>>::value) {
    return any(f, x.x, x.y, x.z);
  } else if constexpr (0 < sizeof...(xs)) {
    return f(x) or any(f, std::forward<decltype(xs)>(xs)...);
  } else {
    return f(x);
  }
}

inline auto is_infinity_or_nan = [](auto x) constexpr { return std::isinf(x) or std::isnan(x); };

}  // namespace follow_trajectory
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__BEHAVIOR__UTILS_HPP_
