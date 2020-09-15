#ifndef SCENARIO_RUNNER__VALIDATOR__ELEMENT_HPP_
#define SCENARIO_RUNNER__VALIDATOR__ELEMENT_HPP_

#include <functional>
#include <limits>
#include <scenario_runner/scope.hpp>
#include <scenario_runner/utility/pugi_extension.hpp>

namespace scenario_runner { inline namespace validator
{
  struct Element
    : public std::vector<Object>
  {
    using Make = std::function<Object (const pugi::xml_node&, Scope&)>;

    Make make;

    const std::size_t minOccurs,
                      maxOccurs;

    explicit Element(const Make& make,
                     std::size_t minOccurs = 1,
                     std::size_t maxOccurs = 1)
      : make { make }
      , minOccurs { minOccurs }
      , maxOccurs { maxOccurs }
    {}

    template <typename... Ts>
    decltype(auto) emplace_back(Ts&&... xs)
    {
      if (size() < maxOccurs)
      {
        push_back(make(std::forward<decltype(xs)>(xs)...));
        return back();
      }
      else
      {
        throw std::runtime_error { "cardinality-error: max" };
      }
    }

    void validate(const std::string& someone_requires_this) const
    {
      if (minOccurs != 0 && size() < minOccurs)
      {
        std::stringstream ss {};
        ss << someone_requires_this << " at least " << minOccurs << " as element" << (1 < minOccurs ? "s" : "") << ", but " << size() << " specified";
        throw SyntaxError { ss.str() };
      }
    }
  };

  template <typename... Ts>
  std::basic_ostream<Ts...>& operator <<(std::basic_ostream<Ts...>& os, const Element& element)
  {
    for (const auto& each : element)
    {
      os << each << "\n";
    }

    return os;
  }

  struct Elements
    : public std::unordered_map<std::string, Element>
  {
    template <typename... Ts>
    explicit Elements(Ts&&... xs)
      : std::unordered_map<std::string, Element> { std::forward<decltype(xs)>(xs)... }
    {}

    template <typename T, typename... Ts>
    void defineGroup(T& group, Ts&&... xs) // XXX DIRTY HACK
    {
      for (const auto& each : group)
      {
        emplace(
          std::get<0>(each),
          Element(
            [&](auto&& node, auto&& scope) { return group.validate(node.parent(), scope); },
            std::forward<decltype(xs)>(xs)...));
      }
    }

    template <typename T, typename... Ts>
    void defineElement(const std::string& name, Ts&&... xs)
    {
      emplace(
        name,
        Element(
          [](auto&&... ys) { return make<T>(std::forward<decltype(ys)>(ys)...); },
          std::forward<decltype(xs)>(xs)...));
    }

    template <typename... Ts>
    void defineElementAsUnimplemented(const std::string& name, Ts&&... xs)
    {
      emplace(
        name,
        Element(
          [name](auto&&...)
          {
            std::stringstream ss {};
            ss << "given class \'" << name << "\' is valid OpenSCENARIO element, but is not yet implemented";
            throw ImplementationFault { ss.str() };
            return unspecified;
          },
          std::forward<decltype(xs)>(xs)...));
    }

    template <typename... Ts>
    void defineElementAsUnsupported(const std::string& name, Ts&&... xs)
    {
      emplace(
        name,
        Element(
          [name](auto&&...)
          {
            std::stringstream ss {};
            ss << "given class \'" << name << "\' is valid OpenSCENARIO element, but is not supported";
            throw SyntaxError { ss.str() };
            return unspecified;
          },
          std::forward<decltype(xs)>(xs)...));
    }

    template <typename... Ts>
    decltype(auto) elements(Ts&&... xs) const
    {
      return at(std::forward<decltype(xs)>(xs)...);
    }

    template <typename... Ts>
    decltype(auto) clear(Ts&&... xs)
    {
      return at(std::forward<decltype(xs)>(xs)...).clear();
    }

    decltype(auto) element(const std::string& name, std::size_t index = 0) const
    {
      return elements(name).at(index);
    }

    template <typename T>
    decltype(auto) elements() const
    {
      auto iter {
        std::find_if(std::begin(*this), std::end(*this), [](auto&& each)
        {
          return not std::get<1>(each).empty() and std::get<1>(each)[0].type() == typeid(T);
        })
      };

      if (iter != std::end(*this))
      {
        return std::get<1>(*iter);
      }
      else
      {
        std::stringstream ss {};
        ss << "Elements::elements<" << typeid(T).name() << ">() called, but such typed element not found";
        throw ImplementationFault { ss.str() };
      }
    }

    template <typename T>
    decltype(auto) element() const
    {
      return elements<T>()[0].template as<T>();
    }
  };
}}  // namespace scenario_runner::validator

#endif  // SCENARIO_RUNNER__VALIDATOR__ELEMENT_HPP_
