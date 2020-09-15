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
    return IfOutputStreamable<Expression>::invoke(os, *this);
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

  virtual const Pointer<Expression> & currentState() const
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
