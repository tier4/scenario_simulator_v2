// Copyright 2015-2020 Tier IV, Inc. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef OPENSCENARIO_INTERPRETER__VARIANT__VISIBILITY_H_
#define OPENSCENARIO_INTERPRETER__VARIANT__VISIBILITY_H_

#if __cplusplus >= 201606
#include <variant>
#else
#include <boost/variant.hpp>
#endif

namespace openscenario_interpreter
{
#if __cplusplus >= 201606

template <typename... Ts>
using Variant = std::variant<Ts...>;

using std::visit;

#else

template <typename... Ts>
using Variant = boost::variant<Ts...>;

template <typename Lambda, typename... Ts>
struct lambda_visitor
: boost::static_visitor<
    typename std::common_type<decltype(std::declval<Lambda>()(std::declval<Ts>()))...>::type>
{
  explicit lambda_visitor(const Lambda & visitor) : visitor(visitor) {}
  explicit lambda_visitor(Lambda && visitor) : visitor(std::move(visitor)) {}

  template <typename T>
  auto operator()(T && arg)
  {
    return visitor(std::forward<T>(arg));
  }

  template <typename T>
  auto operator()(T && arg) const
  {
    return visitor(std::forward<T>(arg));
  }

private:
  Lambda visitor;
};

template <typename Visitor, typename... Ts>
auto visit(Visitor && visitor, const boost::variant<Ts...> & arg0)
{
  lambda_visitor<std::decay_t<Visitor>, Ts...> f{std::forward<Visitor>(visitor)};
  return boost::apply_visitor(f, arg0);
}

template <typename Visitor, typename... Ts>
auto visit(Visitor && visitor, boost::variant<Ts...> && arg0)
{
  lambda_visitor<std::decay_t<Visitor>, Ts...> f{std::forward<Visitor>(visitor)};
  return boost::apply_visitor(f, std::move(arg0));
}

template <typename Visitor, typename... Ts, typename... Variants>
auto visit(Visitor && visitor, const boost::variant<Ts...> & var0, Variants &&... vars)
{
  return visit(
    [&visitor, &vars...](auto && arg0) {
      return visit(
        [&visitor, &arg0](auto &&... args) {
          return visitor(arg0, std::forward<decltype(args)>(args)...);
        },
        std::forward<decltype(vars)>(vars)...);
    },
    var0);
}

template <typename Visitor, typename... Ts, typename... Variants>
auto visit(Visitor && visitor, boost::variant<Ts...> && var0, Variants &&... vars)
{
  return visit(
    [&visitor, &vars...](auto && arg0) {
      return visit(
        [&visitor, &arg0](auto &&... args) {
          return visitor(arg0, std::forward<decltype(args)>(args)...);
        },
        std::forward<decltype(vars)>(vars)...);
    },
    std::move(var0));
}
#endif
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__VARIANT__VISIBILITY_H_
