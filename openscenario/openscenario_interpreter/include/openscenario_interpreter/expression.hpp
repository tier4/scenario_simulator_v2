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

#ifndef OPENSCENARIO_INTERPRETER__EXPRESSION_HPP_
#define OPENSCENARIO_INTERPRETER__EXPRESSION_HPP_

#include <openscenario_interpreter/pointer.hpp>

namespace openscenario_interpreter
{
struct Expression
{
  virtual const std::type_info & type() const noexcept { return typeid(Expression); }

  virtual std::ostream & write(std::ostream & os) const
  {
    return IfHasStreamOutputOperator<Expression>::applyIt(os, *this);
  }

  virtual Pointer<Expression> evaluate(const Pointer<Expression> &)
  {
    throw SemanticError("No viable evaluation for class ", type().name());
  }

  virtual bool accomplished() { return false; }

  virtual const Pointer<Expression> & state() const
  {
    throw SemanticError("Class ", type().name(), " is not a StoryboardElementType");
  }

  virtual void start()
  {
    throw SemanticError("Class ", type().name(), " is not a StoryboardElementType");
  }
};
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__EXPRESSION_HPP_
