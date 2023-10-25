// Copyright 2015 TIER IV, Inc. All rights reserved.
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

#ifndef SIMPLE_JUNIT__JUNIT5_HPP_
#define SIMPLE_JUNIT__JUNIT5_HPP_

#include <optional>
#include <pugixml.hpp>
#include <stdexcept>
#include <string>
#include <unordered_map>

namespace xs
{
using string = std::string;
}  // namespace xs

namespace common
{
inline namespace junit
{
struct Pass
{
  friend auto operator<<(std::ostream & os, const Pass &) -> std::ostream &
  {
    return os << "\x1b[32mPassed\x1b[0m";
  }
};

/*
    <xs:element name="failure">
        <xs:complexType mixed="true">
            <xs:attribute name="type" type="xs:string" use="optional"/>
            <xs:attribute name="message" type="xs:string" use="optional"/>
        </xs:complexType>
    </xs:element>
*/

struct Failure
{
  xs::string type, message;

  explicit Failure(const xs::string & type, const xs::string & message)
  : type(type), message(message)
  {
  }

  friend auto operator<<(std::ostream & os, const Failure & failure) -> std::ostream &
  {
    return os << "\x1b[1;31m" << failure.type << ": " << failure.message << "\x1b[0m";
  }

  friend auto operator<<(pugi::xml_node node, const Failure & failure) -> pugi::xml_node
  {
    if (not failure.type.empty()) {
      node.append_attribute("type") = failure.type.c_str();
    }

    if (not failure.message.empty()) {
      node.append_attribute("message") = failure.message.c_str();
    }

    return node;
  }
};

/*
    <xs:element name="error">
        <xs:complexType mixed="true">
            <xs:attribute name="type" type="xs:string" use="optional"/>
            <xs:attribute name="message" type="xs:string" use="optional"/>
        </xs:complexType>
    </xs:element>
*/

struct Error
{
  xs::string type, message;

  explicit Error(const xs::string & type, const xs::string & message) : type(type), message(message)
  {
  }

  friend auto operator<<(std::ostream & os, const Error & error) -> std::ostream &
  {
    return os << "\x1b[1;31m" << error.type << ": " << error.message << "\x1b[0m";
  }

  friend auto operator<<(pugi::xml_node node, const Error & error) -> pugi::xml_node
  {
    if (not error.type.empty()) {
      node.append_attribute("type") = error.type.c_str();
    }

    if (not error.message.empty()) {
      node.append_attribute("message") = error.message.c_str();
    }

    return node;
  }
};

/*
    <xs:element name="properties">
        <xs:complexType>
            <xs:sequence>
                <xs:element ref="property" maxOccurs="unbounded"/>
            </xs:sequence>
        </xs:complexType>
    </xs:element>
*/

struct Properties
{
  /// @todo implement `properties` in junit.
};

/*
    <xs:element name="property">
        <xs:complexType>
            <xs:attribute name="name" type="xs:string" use="required"/>
            <xs:attribute name="value" type="xs:string" use="required"/>
        </xs:complexType>
    </xs:element>
*/

struct Property
{
  /// @todo implement `property` in junit.
};

/*
    <xs:element name="skipped" type="xs:string"/>
*/

using Skipped = xs::string;

/*
    <xs:element name="system-out" type="xs:string"/>
*/

using SystemOut = xs::string;

/*
    <xs:element name="system-err" type="xs:string"/>
*/

using SystemErr = xs::string;

/*
    <xs:element name="testcase">
        <xs:complexType>
            <xs:sequence>
                <xs:element ref="skipped" minOccurs="0" maxOccurs="1"/>
                <xs:element ref="error" minOccurs="0" maxOccurs="unbounded"/>
                <xs:element ref="failure" minOccurs="0" maxOccurs="unbounded"/>
                <xs:element ref="system-out" minOccurs="0" maxOccurs="unbounded"/>
                <xs:element ref="system-err" minOccurs="0" maxOccurs="unbounded"/>
            </xs:sequence>
            <xs:attribute name="name" type="xs:string" use="required"/>
            <xs:attribute name="assertions" type="xs:string" use="optional"/>
            <xs:attribute name="time" type="xs:string" use="optional"/>
            <xs:attribute name="classname" type="xs:string" use="optional"/>
            <xs:attribute name="status" type="xs:string" use="optional"/>
        </xs:complexType>
    </xs:element>
*/

struct SimpleTestCase
{
  Skipped skipped;

  std::vector<Pass> pass;

  std::vector<Error> error;

  std::vector<Failure> failure;

  std::vector<SystemOut> system_out;

  std::vector<SystemErr> system_err;

  const xs::string name;

  xs::string assertions;

  xs::string time;

  xs::string classname;

  xs::string status;

  explicit SimpleTestCase(const xs::string & name) : name(name) {}

  friend auto operator<<(pugi::xml_node node, const SimpleTestCase & testcase) -> pugi::xml_node
  {
    auto current_node = node.append_child("testcase");

    current_node.append_attribute("name") = testcase.name.c_str();

    if (not testcase.assertions.empty()) {
      current_node.append_attribute("assertions") = testcase.assertions.c_str();
    }

    if (not testcase.time.empty()) {
      current_node.append_attribute("time") = testcase.time.c_str();
    }

    if (not testcase.classname.empty()) {
      current_node.append_attribute("classname") = testcase.classname.c_str();
    }

    if (not testcase.status.empty()) {
      current_node.append_attribute("status") = testcase.status.c_str();
    }

    /// @todo implement `skipped` element in junit.

    for (const auto & each : testcase.error) {
      current_node.append_child("error") << each;
    }

    for (const auto & each : testcase.failure) {
      current_node.append_child("failure") << each;
    }

    /// @todo implement `system-out` element in junit.

    /// @todo implement `system-err` element in junit.

    return node;
  }
};

/*
    <xs:element name="testsuite">
        <xs:complexType>
            <xs:sequence>
                <xs:element ref="properties" minOccurs="0" maxOccurs="1"/>
                <xs:element ref="testcase" minOccurs="0" maxOccurs="unbounded"/>
                <xs:element ref="system-out" minOccurs="0" maxOccurs="1"/>
                <xs:element ref="system-err" minOccurs="0" maxOccurs="1"/>
            </xs:sequence>
            <xs:attribute name="name" type="xs:string" use="required"/>
            <xs:attribute name="tests" type="xs:string" use="required"/>
            <xs:attribute name="failures" type="xs:string" use="optional"/>
            <xs:attribute name="errors" type="xs:string" use="optional"/>
            <xs:attribute name="time" type="xs:string" use="optional"/>
            <xs:attribute name="disabled" type="xs:string" use="optional"/>
            <xs:attribute name="skipped" type="xs:string" use="optional"/>
            <xs:attribute name="timestamp" type="xs:string" use="optional"/>
            <xs:attribute name="hostname" type="xs:string" use="optional"/>
            <xs:attribute name="id" type="xs:string" use="optional"/>
            <xs:attribute name="package" type="xs:string" use="optional"/>
        </xs:complexType>
    </xs:element>
*/

struct SimpleTestSuite : private std::unordered_map<xs::string, SimpleTestCase>
{
  const xs::string name;

  explicit SimpleTestSuite(const xs::string & name) : name(name) {}

  auto getTestcaseNames() const -> std::vector<std::string>
  {
    std::vector<std::string> names;
    for (auto itr = begin(); itr != end(); ++itr) {
      names.emplace_back(itr->first);
    }
    return names;
  }

  auto testcase(const xs::string & name) -> auto &
  {
    try {
      return at(name);
    } catch (const std::out_of_range &) {
      emplace(name, name);
      return at(name);
    }
  }

  friend auto operator<<(pugi::xml_node node, const SimpleTestSuite & testsuite) -> pugi::xml_node
  {
    auto current_node = node.append_child("testsuite");

    current_node.append_attribute("name") = testsuite.name.c_str();

    std::size_t tests = 0;
    std::size_t failures = 0;
    std::size_t pass = 0;
    std::size_t errors = 0;
    for (const auto & testcase : testsuite) {
      current_node << testcase.second;
      pass = pass + testcase.second.pass.size();
      failures = failures + testcase.second.failure.size();
      errors = errors + testcase.second.error.size();
    }
    tests = pass + failures + errors;
    current_node.append_attribute("failures") = failures;
    current_node.append_attribute("errors") = errors;
    current_node.append_attribute("tests") = tests;
    return node;
  }
};

/*
    <xs:element name="testsuites">
        <xs:complexType>
            <xs:sequence>
                <xs:element ref="testsuite" minOccurs="0" maxOccurs="unbounded"/>
            </xs:sequence>
            <xs:attribute name="name" type="xs:string" use="optional"/>
            <xs:attribute name="time" type="xs:string" use="optional"/>
            <xs:attribute name="tests" type="xs:string" use="optional"/>
            <xs:attribute name="failures" type="xs:string" use="optional"/>
            <xs:attribute name="disabled" type="xs:string" use="optional"/>
            <xs:attribute name="errors" type="xs:string" use="optional"/>
        </xs:complexType>
    </xs:element>
*/

struct SimpleTestSuites : private std::unordered_map<std::string, SimpleTestSuite>
{
  xs::string name;

  explicit SimpleTestSuites(const xs::string & name = "") : name(name) {}

  auto testsuite(const xs::string & name) -> auto &
  {
    try {
      return at(name);
    } catch (const std::out_of_range &) {
      emplace(name, name);
      return at(name);
    }
  }

  friend auto operator<<(pugi::xml_node node, const SimpleTestSuites & testsuites) -> pugi::xml_node
  {
    auto current_node = node.append_child("testsuites");

    if (not testsuites.name.empty()) {
      current_node.append_attribute("name") = testsuites.name.c_str();
    }
    std::size_t tests = 0;
    std::size_t failures = 0;
    std::size_t pass = 0;
    std::size_t errors = 0;
    for (const auto & testsuite : testsuites) {
      SimpleTestSuite suite = testsuite.second;
      current_node << suite;
      const auto testcase_names = suite.getTestcaseNames();
      for (const auto & testcase_name : testcase_names) {
        failures = failures + suite.testcase(testcase_name).failure.size();
        errors = errors + suite.testcase(testcase_name).error.size();
        pass = pass + suite.testcase(testcase_name).pass.size();
      }
      tests = failures + errors + pass;
    }
    current_node.append_attribute("failures") = failures;
    current_node.append_attribute("errors") = errors;
    current_node.append_attribute("tests") = tests;

    return node;
  }

  template <typename... Ts>
  auto write_to(Ts &&... xs) const
  {
    pugi::xml_document document;

    document << *this;

    document.save_file(std::forward<decltype(xs)>(xs)...);
  }
};

using JUnit5 = SimpleTestSuites;
}  // namespace junit
}  // namespace common

#endif  // SIMPLE_JUNIT__JUNIT5_HPP_
