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

#ifndef SCENARIO_RUNNER__POINTER_HPP_
#define SCENARIO_RUNNER__POINTER_HPP_

#include <memory>
#include <typeinfo>
#include <utility>

#include <scenario_runner/error.hpp>
#include <scenario_runner/type_traits/if_accomplishable.hpp>
#include <scenario_runner/type_traits/if_evaluable.hpp>
#include <scenario_runner/type_traits/if_output_streamable.hpp>
#include <scenario_runner/type_traits/if_startable.hpp>
#include <scenario_runner/type_traits/if_stateful.hpp>
#include <scenario_runner/utility/pair.hpp>

#define DEBUG() std::cout << green << __FILE__ << magenta << ":" << cyan << __LINE__ << reset << \
    std::endl

namespace scenario_runner
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
    std::ostream & write(std::ostream & os) const override
    {
      return IfOutputStreamable<Bound>::invoke(os, *this);
    }

    Pointer evaluate(const Pointer & as_is) override
    {
      return IfEvaluable<Bound>::invoke(as_is, static_cast<Bound &>(*this));
    }

    bool accomplished() override
    {
      return IfAccomplishable<Bound>::invoke(*this);
    }

    const Pointer & currentState() const override
    {
      return IfStateful<Bound>::template currentState<Pointer>(*this);
    }

    void start() override
    {
      IfStartable<Bound>::invoke(*this);
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
  decltype(auto) currentState(Ts && ... xs) const
  {
    return binding().currentState(std::forward<decltype(xs)>(xs)...);
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
}  // namespace scenario_runner

#endif  // SCENARIO_RUNNER__POINTER_HPP_
