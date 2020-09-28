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

#ifndef SCENARIO_RUNNER__EXPRESSION_HPP_
#define SCENARIO_RUNNER__EXPRESSION_HPP_

#include <scenario_runner/pointer.hpp>

namespace scenario_runner
{
struct Expression
{
  virtual const std::type_info & type() const noexcept
  {
    return typeid(Expression);
  }

  virtual std::ostream & write(std::ostream & os) const
  {
    return WhenHasStreamOutputOperator<Expression>::applyIt(os, *this);
  }

  virtual Pointer<Expression> evaluate(const Pointer<Expression> &)
  {
    std::stringstream ss {};
    ss << "no viable evaluation for class Expression";
    throw ImplementationFault {ss.str()};
  }

  virtual bool accomplished()
  {
    return false;
  }

  virtual const Pointer<Expression> & state() const
  {
    std::stringstream ss {};
    ss << "class Expression is not stateful";
    throw ImplementationFault {ss.str()};
  }

  virtual void start()
  {
    std::stringstream ss {};
    ss << "class Expression is not startable";
    throw ImplementationFault {ss.str()};
  }
};
}  // namespace scenario_runner

#endif  // SCENARIO_RUNNER__EXPRESSION_HPP_
