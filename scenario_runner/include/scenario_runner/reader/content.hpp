#ifndef SCENARIO_RUNNER__READER__CONTENT_HPP_
#define SCENARIO_RUNNER__READER__CONTENT_HPP_

#include <boost/algorithm/string/trim.hpp>
#include <scenario_runner/syntax/parameter_type.hpp>
#include <scenario_runner/utility/pugi_extension.hpp>

namespace scenario_runner
{inline namespace reader
{
template<typename T, typename Node, typename Scope>
T readContent(const Node & node, const Scope &)
{
  const std::string text {node.text().get()};
  return boost::lexical_cast<T>(boost::algorithm::trim_copy(text));
}
}}  // namespace scenario_runner::reader

#endif  // SCENARIO_RUNNER__READER__CONTENT_HPP_
