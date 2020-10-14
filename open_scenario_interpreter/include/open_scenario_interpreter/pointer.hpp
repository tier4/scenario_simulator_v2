// Copyright 2015-2020 Autoware Foundation. All rights reserved.
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

#ifndef OPEN_SCENARIO_INTERPRETER__POINTER_HPP_
#define OPEN_SCENARIO_INTERPRETER__POINTER_HPP_

#include <open_scenario_interpreter/error.hpp>
#include <open_scenario_interpreter/type_traits/if_has_member_function_accomplished.hpp>
#include <open_scenario_interpreter/type_traits/if_has_member_function_evaluate.hpp>
#include <open_scenario_interpreter/type_traits/if_has_member_function_start.hpp>
#include <open_scenario_interpreter/type_traits/if_has_member_function_state.hpp>
#include <open_scenario_interpreter/type_traits/if_has_stream_output_operator.hpp>
#include <open_scenario_interpreter/utility/pair.hpp>

#include <memory>
#include <typeinfo>
#include <utility>

#define DEBUG() std::cout << green << __FILE__ << magenta << ":" << cyan << __LINE__ << reset << \
    std::endl

namespace open_scenario_interpreter
{
template<typename T>
class Pointer
  : public std::shared_ptr<T>
{
  template<typename Bound>
  struct Binder
    : public T,
    public Bound
  {
    using top = T;

    template<typename ... Ts>
    explicit constexpr Binder(Ts && ... xs)
    : Bound{std::forward<decltype(xs)>(xs)...}
    {}

    virtual ~Binder() = default;

    const std::type_info & type() const noexcept override
    {
      return typeid(Bound);
    }

private:
// ^ NOTE This broken indent was forced by ament_uncrustify.
    std::ostream & write(std::ostream & os) const override
    {
      return IfHasStreamOutputOperator<Bound>::applyIt(os, *this);
    }

    Pointer evaluate(const Pointer & else_) override
    {
      return IfHasMemberFunctionEvaluate<Bound>::callIt(static_cast<Bound &>(*this), else_);
    }

    bool accomplished() override
    {
      return IfHasMemberFunctionAccomplished<Bound>::callIt(*this);
    }

    const Pointer & state() const override
    {
      return IfHasMemberFunctionState<Bound>::template callIt<Pointer>(*this);
    }

    void start() override  // corresponds to startTransition
    {
      IfHasMemberFunctionStart<Bound>::callIt(*this);
    }
  };

public:
  template<typename ... Ts>
  explicit constexpr Pointer(Ts && ... xs)
  : std::shared_ptr<T>{std::forward<decltype(xs)>(xs)...}
  {}

  template<typename U, typename ... Ts>
  static Pointer bind(Ts && ... xs)
  {
    using Binding = Binder<U>;
    return static_cast<Pointer>(std::make_shared<Binding>(std::forward<decltype(xs)>(xs)...));
  }

  template<typename U, typename ... Ts>
  decltype(auto) rebind(Ts && ... xs)
  {
    return *this = bind<U>(std::forward<decltype(xs)>(xs)...);
  }

  decltype(auto) binding() const
  {
    if (*this) {
      return std::shared_ptr<T>::operator*();
    } else {
      std::stringstream ss {};
      ss << "dereferencing nullptr";
      throw ImplementationFault {ss.str()};
    }
  }

  decltype(auto) type() const
  {
    return binding().type();
  }

  template<typename U>
  decltype(auto) is() const
  {
    return type() == typeid(U);
  }

  template<typename U>
  decltype(auto) as() const
  {
    const auto bound {std::dynamic_pointer_cast<U>(*this)};

    if (bound) {
      return *bound;
    } else {
      std::stringstream ss {};
      ss << "type-error: can't treat " << binding().type().name() << " as type " <<
        typeid(U).name();
      throw std::runtime_error {ss.str()};
    }
  }

  template<typename U>
  decltype(auto) as(const char * const file, int line) const try {
    return as<U>();
  } catch (const std::runtime_error & error) {
    std::stringstream ss {};
    ss << error.what() << " (call from " << file << ":" << line << ")";
    throw std::runtime_error {ss.str()};
  }

public:
  template<typename ... Ts>
  decltype(auto) evaluate(Ts && ... xs) const
  {
    return binding().evaluate(*this, std::forward<decltype(xs)>(xs)...);
  }

  template<typename ... Ts>
  decltype(auto) accomplished(Ts && ... xs) const
  {
    return binding().accomplished(std::forward<decltype(xs)>(xs)...);
  }

  template<typename ... Ts>
  decltype(auto) state(Ts && ... xs) const
  {
    return binding().state(std::forward<decltype(xs)>(xs)...);
  }

  template<typename ... Ts>
  decltype(auto) start(Ts && ... xs) const
  {
    return binding().start(std::forward<decltype(xs)>(xs)...);
  }
};

template<typename T, typename ... Ts>
std::basic_ostream<Ts...> & operator<<(std::basic_ostream<Ts...> & os, const Pointer<T> & pointer)
{
  return (pointer ? pointer.binding().write(os) : (os << faint << "<Null/>")) << reset;
}
}  // namespace open_scenario_interpreter

#endif  // OPEN_SCENARIO_INTERPRETER__POINTER_HPP_
