#ifndef SCENARIO_RUNNER__TYPE_TRAITS__IF_OUTPUT_STREAMABLE_HPP_
#define SCENARIO_RUNNER__TYPE_TRAITS__IF_OUTPUT_STREAMABLE_HPP_

#include <scenario_runner/concepts/output_streamable.hpp>
#include <scenario_runner/console/escape_sequence.hpp>
#include <scenario_runner/utility/indent.hpp>

namespace scenario_runner { inline namespace type_traits
{
  template <typename T, typename = void>
  struct IfOutputStreamable
  {
    static std::ostream& invoke(std::ostream& os, const T&)
    {
      return os << indent << blue << "<" << typeid(T).name() << "/>" << reset;
    }
  };

  template <typename T>
  struct IfOutputStreamable<T, typename std::enable_if<OutputStreamable<T>::value>::type>
  {
    static std::ostream& invoke(std::ostream& os, const T& something)
    {
      return os << something;
    }
  };
}}  // namespace scenario_runner::type_traits

#endif  // SCENARIO_RUNNER__TYPE_TRAITS__IF_OUTPUT_STREAMABLE_HPP_

