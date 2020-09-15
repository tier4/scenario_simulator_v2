#ifndef SCNARIO_RUNNER__OBJECT_HPP_
#define SCNARIO_RUNNER__OBJECT_HPP_

#include <scenario_runner/expression.hpp>
#include <vector>

namespace scenario_runner
{
  using Object = Pointer<Expression>;

  using Objects = std::vector<Object>;

  static const Object unit { nullptr };

  template <typename T, typename... Ts>
  inline constexpr decltype(auto) make(Ts&&... xs)
  {
    return Object::bind<T>(std::forward<decltype(xs)>(xs)...);
  }

  template <typename T>
  inline constexpr decltype(auto) make(T&& x)
  {
    return Object::bind<typename std::decay<decltype(x)>::type>(std::forward<decltype(x)>(x));
  }

  extern const Object unspecified;

  struct Unspecified
  {
    decltype(auto) evaluate() const noexcept
    {
      return unspecified;
    }
  };

  std::ostream& operator <<(std::ostream&, const Unspecified&);
}  // namespace scenario_runner

#endif  // SCENARIO_RUNNER__OBJECT_HPP_
