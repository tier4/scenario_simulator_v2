// Copyright 2015-2021 Tier IV, Inc. All rights reserved.
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

#include <boost/optional.hpp>
#include <pugixml.hpp>
#include <stdexcept>
#include <unordered_map>

namespace common
{
inline namespace simple_junit
{
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
  std::string type, message;

  explicit Failure(const std::string & type, const std::string & message)
  : type(type), message(message)
  {
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
  std::string type, message;

  explicit Error(const std::string & type, const std::string & message)
  : type(type), message(message)
  {
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
  const std::string name, assertions, time, classname, status;

  std::vector<Error> error;

  std::vector<Failure> failure;

  explicit SimpleTestCase(const std::string & name) : name(name) {}

  friend auto operator<<(pugi::xml_node node, const SimpleTestCase & testcase) -> pugi::xml_node
  {
    auto current_node = node.append_child("testcase");

    current_node.append_attribute("name") = testcase.name.c_str();

    for (const auto & each : testcase.error) {
      current_node.append_child("error") << each;
    }

    for (const auto & each : testcase.failure) {
      current_node.append_child("failure") << each;
    }

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

struct SimpleTestSuite : private std::unordered_map<std::string, SimpleTestCase>
{
  const std::string name;

  explicit SimpleTestSuite(const std::string & name) : name(name) {}

  auto testcase(const std::string & name) -> auto &
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

    for (const auto & testcase : testsuite) {
      current_node << std::get<1>(testcase);
    }

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
  explicit SimpleTestSuites() = default;

  auto testsuite(const std::string & name) -> auto &
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

    for (const auto & testsuite : testsuites) {
      current_node << std::get<1>(testsuite);
    }

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
}  // namespace simple_junit
}  // namespace common

#endif  // SIMPLE_JUNIT__JUNIT5_HPP_
