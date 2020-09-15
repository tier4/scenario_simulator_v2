#ifndef SCENARIO_RUNNER__VALIDATOR__ALL_HPP_
#define SCENARIO_RUNNER__VALIDATOR__ALL_HPP_

#include <scenario_runner/validator/element.hpp>

namespace scenario_runner
{inline namespace validator
{
struct All
  : public Elements
{
  template<typename ... Ts>
  explicit All(Ts && ... xs)
  : Elements{std::forward<decltype(xs)>(xs)...}
  {}

  Object validate(const pugi::xml_node & node, Scope & scope)
  {
    for (const auto & each : node.children()) {
      auto iter {find(each.name())};

      if (iter != std::end(*this)) {
        std::get<1>(*iter).emplace_back(each, scope);
      } else {
        std::stringstream ss {};
        ss << "unknown class \'" << each.name() << "\' specified for property of class " <<
          node.name();
        throw SyntaxError {ss.str()};
      }
    }

    for (const auto & each : *this) {
      std::stringstream ss {};
      ss << "class " << node.name() << " requires class " << each.first;
      each.second.validate(ss.str());
    }

    return unspecified;
  }
};
}}  // namespace scenario_runner::validator

#endif  // SCENARIO_RUNNER__VALIDATOR__ALL_HPP_
