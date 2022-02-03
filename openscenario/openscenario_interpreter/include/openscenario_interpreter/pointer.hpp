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

#ifndef OPENSCENARIO_INTERPRETER__POINTER_HPP_
#define OPENSCENARIO_INTERPRETER__POINTER_HPP_

#include <cstddef>
#include <memory>
#include <openscenario_interpreter/error.hpp>
#include <openscenario_interpreter/type_traits/if_has_member_function_accomplished.hpp>
#include <openscenario_interpreter/type_traits/if_has_member_function_current_state.hpp>
#include <openscenario_interpreter/type_traits/if_has_member_function_description.hpp>
#include <openscenario_interpreter/type_traits/if_has_member_function_evaluate.hpp>
#include <openscenario_interpreter/type_traits/if_has_stream_output_operator.hpp>
#include <openscenario_interpreter/utility/pair.hpp>
#include <type_traits>
#include <typeinfo>
#include <utility>

namespace openscenario_interpreter
{
template <typename T>
class Pointer : public std::shared_ptr<T>
{
  template <typename Bound>
  struct Binder : public T, public Bound
  {
    template <typename... Ts>
    explicit constexpr Binder(Ts &&... xs) : Bound(std::forward<decltype(xs)>(xs)...)
    {
    }

    virtual ~Binder() = default;

    auto type() const noexcept -> const std::type_info & override { return typeid(Bound); }

  private:
    bool accomplished() override  //
    {
      return IfHasMemberFunctionAccomplished<Bound>::invoke(*this);
    }

    auto description() const -> std::string override
    {
      return IfHasMemberFunctionDescription<Bound>::invoke(*this);
    }

    auto evaluate(const Pointer & else_) -> Pointer override
    {
      return IfHasMemberFunctionEvaluate<Bound>::invoke(static_cast<Bound &>(*this), else_);
    }

    auto currentState() const -> const Pointer & override
    {
      return IfHasMemberFunctionCurrentState<Bound>::template invoke<Pointer>(*this);
    }

    auto write(std::ostream & os) const -> std::ostream & override
    {
      return IfHasStreamOutputOperator<Bound>::invoke(os, *this);
    }
  };

public:
  using std::shared_ptr<T>::shared_ptr;

  template <typename U, typename... Ts>
  static Pointer bind(Ts &&... xs)
  {
    using Binding = Binder<U>;
    return static_cast<Pointer>(std::make_shared<Binding>(std::forward<decltype(xs)>(xs)...));
  }

  template <typename U, typename... Ts>
  decltype(auto) rebind(Ts &&... xs)
  {
    return *this = bind<U>(std::forward<decltype(xs)>(xs)...);
  }

  decltype(auto) binding() const
  {
    if (*this) {
      return std::shared_ptr<T>::operator*();
    } else {
      throw SemanticError(
        "Dereferencing null-pointer. This is likely due to improper implementation");
    }
  }

  auto type() const -> const std::type_info & { return *this ? binding().type() : typeid(nullptr); }

  template <typename U>
  auto is() const -> bool
  {
    return type() == typeid(U);
  }

  template <typename U>
  auto is_also() const
  {
    return static_cast<bool>(std::dynamic_pointer_cast<U>(*this));
  }

  template <typename U>
  auto as() const -> U &
  {
    if (const auto bound = std::dynamic_pointer_cast<U>(*this)) {
      return *bound;
    } else {
      throw SemanticError("Can't treat ", binding().type().name(), " as ", typeid(U).name());
    }
  }

  template <typename... Ts>
  decltype(auto) evaluate(Ts &&... xs) const
  {
    return binding().evaluate(*this, std::forward<decltype(xs)>(xs)...);
  }

  template <typename... Ts>
  decltype(auto) accomplished(Ts &&... xs) const
  {
    return binding().accomplished(std::forward<decltype(xs)>(xs)...);
  }

  template <typename... Ts>
  decltype(auto) currentState(Ts &&... xs) const
  {
    return binding().currentState(std::forward<decltype(xs)>(xs)...);
  }

  template <typename... Ts>
  decltype(auto) description(Ts &&... xs) const
  {
    return binding().description(std::forward<decltype(xs)>(xs)...);
  }
};

template <typename T>
std::ostream & operator<<(std::ostream & os, const Pointer<T> & pointer)
{
  return (pointer ? pointer.binding().write(os) : (os << faint << "<TODO/>")) << reset;
}
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__POINTER_HPP_
