#ifndef SCENARIO_RUNNER__READER__ELEMENT_HPP_
#define SCENARIO_RUNNER__READER__ELEMENT_HPP_

#include <scenario_runner/iterator/size.hpp>
#include <scenario_runner/object.hpp>
#include <scenario_runner/type_traits/if_not_default_constructible.hpp>
#include <scenario_runner/utility/pugi_extension.hpp>

namespace scenario_runner
{inline namespace reader
{
constexpr auto unbounded {std::numeric_limits<std::size_t>::max()};

template<typename T, typename Node, typename ... Ts>
auto readElement(const std::string & name, const Node & parent, Ts && ... xs)
{
  if (const auto child {parent.child(name.c_str())}) {
    return T {child, std::forward<decltype(xs)>(xs)...};
  } else {
    return IfNotDefaultConstructible<T>::error(parent.name(), name);   // TODO => IfNotNothrowDefaultConstructible
  }
}

template<typename Node, typename Callee>
void callWithElements(
  const Node & parent,
  const std::string & name,
  typename std::iterator_traits<typename Node::iterator>::difference_type min_occurs,
  typename std::iterator_traits<typename Node::iterator>::difference_type max_occurs,
  Callee && call_with)
{
  const auto children {parent.children(name.c_str())};

  if (const auto size {iterator::size(children)}) {
    if (min_occurs != 0 && size < min_occurs) {
      std::stringstream ss {};
      ss << parent.name() <<
        " requires class " <<
        name <<
        " at least " <<
        min_occurs <<
        " element" <<
      (1 < min_occurs ? "s" : "") <<
        ", but " <<
        size <<
        " element" <<
      (1 < size ? "s" : "") <<
        " specified";
      throw SyntaxError {ss.str()};
    } else if (max_occurs < size) {
      std::stringstream ss {};
      ss << parent.name() <<
        " requires class " <<
        name <<
        " at most " <<
        max_occurs <<
        " element" <<
      (1 < max_occurs ? "s" : "") <<
        ", but " <<
        size <<
        " element" <<
      (1 < size ? "s" : "") <<
        " specified";
      throw SyntaxError {ss.str()};
    } else {
      for (const auto & child : children) {
        call_with(child);
      }
    }
  } else if (min_occurs != 0) {
    std::stringstream ss {};
    ss << parent.name() <<
      " requires class " <<
      name << " at least " <<
      min_occurs <<
      " element" <<
    (1 < min_occurs ? "s" : "") <<
      ", but there is no specification";
    throw SyntaxError {ss.str()};
  }
}

template<typename Callee>
decltype(auto) callWithElement(const pugi::xml_node & parent, const std::string & name,
  Callee && call_with)
{
  return callWithElements(parent, name, 1, 1, std::forward<decltype(call_with)>(call_with));
}

  #define THROW_UNSUPPORTED_ERROR(PARENT) \
  [&](auto && child) \
  { \
    std::stringstream ss {}; \
    ss << "given class \'" << child.name() << "\' (element of class \'" << PARENT.name() << \
      "\') is valid OpenSCENARIO element, but is not supported"; \
    throw SyntaxError {ss.str()}; \
  }
}}  // namespace scenario_runner::reader

#endif  // SCENARIO_RUNNER__READER__ELEMENT_HPP_
