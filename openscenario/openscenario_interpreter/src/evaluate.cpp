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

#include <boost/spirit/include/phoenix.hpp>
#include <boost/spirit/include/qi.hpp>
#include <functional>
#include <openscenario_interpreter/reader/evaluate.hpp>
#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/parameter_type.hpp>
#include <openscenario_interpreter/utility/overload.hpp>

#if __cplusplus >= 201606
#include <variant>
#else
#include <boost/variant.hpp>
#endif

namespace openscenario_interpreter
{
inline namespace reader
{
struct Value
{
  using ExpressionType = Variant<int, unsigned int, unsigned short, double, bool>;
  ExpressionType data;

  struct Numeric;

  Value(const Value &) = default;
  Value(Value &&) = default;
  Value & operator=(const Value &) = default;
  Value & operator=(Value &&) = default;
  ~Value() = default;

  Value() : data(0) {}
  explicit Value(int v) : data(v) {}
  explicit Value(unsigned int v) : data(v) {}
  explicit Value(unsigned short v) : data(v) {}
  explicit Value(double v) : data(v) {}
  explicit Value(bool v) : data(v) {}

  auto type_name() const
  {
    return boost::core::demangle(visit([](auto v) { return typeid(v).name(); }, data));
  }

  template <typename T>
  bool same_as() const
  {
    return visit([](auto v) { return std::is_same<T, decltype(v)>::value; }, data);
  }

  template <typename T>
  Value cast() const;

  template <typename Signature>
  struct Visitor;

  template <typename R, typename... Args>
  struct Visitor<R(Args...)>
  {
    template <typename F>
    Value apply(F && f, const std::array<Value, sizeof...(Args)> & args)
    {
      return applyImpl(std::forward<F>(f), args, std::make_index_sequence<sizeof...(Args)>())
        .template cast<R>();
    }

  private:
    template <typename F, std::size_t... I>
    Value applyImpl(
      F && f, const std::array<Value, sizeof...(Args)> & args, std::index_sequence<I...>)
    {
      return visit(
        [&f](auto &&... v) { return Value{f(std::forward<decltype(v)>(v)...)}; },
        get_arg<I>(args)...);
    }

    template <std::size_t I>
    static decltype(auto) get_arg(const std::array<Value, sizeof...(Args)> & args)
    {
      using Ret = typename std::tuple_element<I, std::tuple<Args...>>::type;
      auto & arg = std::get<I>(args);
      return arg.template cast<Ret>().data;
    }
  };

#define DEFINE_UNARY(FNAME, FUNC, ARG, RET)                                    \
  Value FNAME() const                                                          \
  {                                                                            \
    return Visitor<RET(ARG)>{}.apply([](auto v) { return FUNC(v); }, {*this}); \
  }

#define DEFINE_BINARY(OP, FUNC, TYPE)                         \
  friend Value OP(const Value & lhs, const Value & rhs)       \
  {                                                           \
    return Visitor<TYPE(TYPE, TYPE)>{}.apply(                 \
      [](auto l, auto r) { return FUNC(l, r); }, {lhs, rhs}); \
  }

  // clang-format off
  DEFINE_UNARY(round, [](double v) { return static_cast<int>(std::round(v)); }, double, int)
  DEFINE_UNARY(floor, [](double v) { return static_cast<int>(std::floor(v)); }, double, int)
  DEFINE_UNARY(ceil,  [](double v) { return static_cast<int>(std::ceil(v)); },  double, int)
  DEFINE_UNARY(sqrt,  [](double v) { return static_cast<int>(std::sqrt(v)); },  double, int)
  // clang-format on
  DEFINE_UNARY(operator-, -, Numeric, Numeric)
  DEFINE_UNARY(operator!, !, bool, bool)

  DEFINE_BINARY(operator*, std::multiplies<void>{}, Numeric)
  DEFINE_BINARY(operator+, std::plus<void>{}, Numeric)
  DEFINE_BINARY(operator-, std::minus<void>{}, Numeric)
  DEFINE_BINARY(operator%, std::fmod, Numeric)
  DEFINE_BINARY(operator/, std::divides<double>{}, double)
  DEFINE_BINARY(operator&&, std::logical_and<bool>{}, bool)
  DEFINE_BINARY(operator||, std::logical_or<bool>{}, bool)
  DEFINE_BINARY(pow, std::pow, double)

  friend std::ostream & operator<<(std::ostream & os, const Value & v)
  {
    visit([&](const auto & v) mutable { os << v; }, v.data);
    return os;
  }

  friend bool operator==(const Value & lhs, const Value & rhs)
  {
    return visit(std::equal_to<void>{}, lhs.data, rhs.data);
  }
};

template <typename T>
Value Value::cast() const
{
  if (same_as<double>() and std::is_integral<T>::value)
    throw std::runtime_error{"double cannot convert to integer implicitly."};
  return Value{visit([](auto v) { return static_cast<T>(v); }, data)};
}

template <>
Value Value::cast<Value::Numeric>() const
{
  if (same_as<bool>()) throw std::runtime_error{"boolean cannot convert to numeric."};
  return *this;
}

template <>
Value Value::cast<bool>() const
{
  if (not same_as<bool>()) throw std::runtime_error{"numeric cannot convert to boolean."};
  return *this;
}

namespace qi = boost::spirit::qi;
namespace ascii = boost::spirit::ascii;
namespace ph = boost::phoenix;

template <typename Iter>
struct Grammar : qi::grammar<Iter, Value(), ascii::space_type>
{
  Grammar(const openscenario_interpreter::Scope & scope) : Grammar::base_type(lv0)
  {
    using qi::_1;
    using qi::_2;
    using qi::_val;
    const static qi::real_parser<double, qi::strict_real_policies<double>> dbl;

    // clang-format off
    lv0 = lv1[_val = _1] >> *(qi::lit("or") >> lv1[_val = _val or _1]);
    lv1 = lv2[_val = _1] >> *(qi::lit("and") >> lv2[_val = _val and _1]);
    lv2 = lv3[_val = _1] >> *(('+' >> lv3[_val = _val + _1]) | ('-' >> lv3[_val = _val - _1]));
    lv3 = lv4[_val = _1] >> *(('*' >> lv4[_val = _val * _1]) | ('/' >> lv4[_val = _val / _1]) | ('%' >> lv4[_val = _val % _1]));
    lv4 = (qi::lit('-') >> lv5)[_val = -_1]
          | (qi::lit("not") >> lv5)[_val = !_1]
          | lv5[_val = _1];
    lv5 = (qi::lit("pow") >> '(' >> lv0 >> ',' >> lv0 >> ')')[_val = ph::bind([](auto &&x, auto &&y) { return pow(x, y); }, _1, _2)]
          | (qi::lit("round") >> '(' >> lv0 >> ')')[_val = ph::bind(&Value::round, _1)]
          | (qi::lit("floor") >> '(' >> lv0 >> ')')[_val = ph::bind(&Value::floor, _1)]
          | (qi::lit("ceil") >> '(' >> lv0 >> ')')[_val = ph::bind(&Value::ceil, _1)]
          | (qi::lit("sqrt") >> '(' >> lv0 >> ')')[_val = ph::bind(&Value::sqrt, _1)]
          | lv6[_val = _1];
    lv6 = ('(' >> lv0 >> ')')[_val = _1]
          | qi::lit("true")[_val = ph::construct<Value>(true)]
          | qi::lit("false")[_val = ph::construct<Value>(false)]
          | dbl[_val = ph::construct<Value>(_1)]
          | qi::int_[_val = ph::construct<Value>(_1)]
          | qi::lexeme[qi::lit('$') >> *qi::char_("A-Za-z0-9_")][_val = ph::bind(
              [scope](auto&& chars) {
                return toValue(std::string(chars.begin(), chars.end()), scope);
              }, _1)];
    // clang-format on
  }

  static auto toValue(const std::string & key, const Scope & scope) -> Value
  {
    auto found = scope.findObject(key);
    if (not found) {
      THROW_SYNTAX_ERROR(std::quoted(key), "is not declared in this scope");
    }

    if (found.is<Integer>()) {
      return Value{static_cast<int>(found.as<Integer>())};
    } else if (found.is<UnsignedInt>()) {
      return Value{static_cast<unsigned int>(found.as<UnsignedInt>())};
    } else if (found.is<UnsignedShort>()) {
      return Value{static_cast<unsigned short>(found.as<UnsignedShort>())};
    } else if (found.is<Double>()) {
      return Value{found.as<Double>()};
    } else if (found.is<Boolean>()) {
      return Value{found.as<Boolean>()};
    } else {
      THROW_SYNTAX_ERROR(std::quoted(key), "is neither numeric nor boolean");
    }
  }

  qi::rule<Iter, Value(), ascii::space_type> lv0, lv1, lv2, lv3, lv4, lv5, lv6;
};

std::string evaluate(const std::string & expression, const Scope & scope)
{
  Grammar<decltype(expression.begin())> parser{scope};
  Value output;
  auto first = expression.begin();
  auto last = expression.end();

  qi::phrase_parse(first, last, parser, ascii::space, output);

  if (first != last) {
    THROW_SYNTAX_ERROR("Failed to parse ", std::quoted(expression));
  }

  return visit(
    overload(
      [](bool v) -> std::string { return v ? "true" : "false"; },
      [](auto v) -> std::string { return std::to_string(v); }),
    output.data);
}

}  // namespace reader
}  // namespace openscenario_interpreter
