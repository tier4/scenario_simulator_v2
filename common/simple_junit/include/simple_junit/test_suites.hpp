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

#ifndef JUNIT_EXPORTER__JUNIT_EXPORTER_HPP_
#define JUNIT_EXPORTER__JUNIT_EXPORTER_HPP_

#include <boost/filesystem/path.hpp>
#include <boost/optional.hpp>
#include <numeric>
#include <pugixml.hpp>
#include <simple_junit/test_suite.hpp>
#include <stdexcept>
#include <string>
#include <unordered_map>

namespace junit
{
class TestSuites
{
  const std::string timestamp_;

  std::unordered_map<std::string, TestSuite> test_suites_;

public:
  explicit TestSuites();

  void write(const boost::filesystem::path & path);

  auto contains(const std::string & suite_name, const std::string & case_name) const
  {
    const auto iter = test_suites_.find(suite_name);

    if (iter != std::end(test_suites_)) {
      return std::any_of(
        std::begin(iter->second), std::end(iter->second),
        [&](const auto & each_case) { return each_case.name == case_name; });
    } else {
      return false;
    }
  }

  auto getTotalTime() const
  {
    return std::accumulate(
      std::cbegin(test_suites_), std::cend(test_suites_), 0,
      [](const auto sum, const auto & each_suite) {
        const auto & each_cases = std::get<1>(each_suite);

        return sum + std::accumulate(
                       std::cbegin(each_cases), std::cend(each_cases), 0,
                       [](const auto sum, const auto & each_case) { return sum + each_case.time; });
      });
  }

  template <typename... Ts>
  void addTestCase(
    const std::string & suite_name,  //
    const std::string & case_name,   //
    Ts &&... xs)
  {
    if (not contains(suite_name, case_name)) {
      test_suites_[suite_name].emplace_back(
        case_name, suite_name, std::forward<decltype(xs)>(xs)...);
    }
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
  const std::string name;

  boost::optional<std::string> assertions;
  boost::optional<std::string> time;
  boost::optional<std::string> classname;
  boost::optional<std::string> status;

  explicit SimpleTestCase(const std::string & name) : name(name) {}

  friend auto operator<<(pugi::xml_node node, const SimpleTestCase & testcase) -> pugi::xml_node
  {
    node.append_child("testcase").append_attribute("name") = testcase.name.c_str();

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
    node.append_child("testsuite").append_attribute("name") = testsuite.name.c_str();

    for (const auto & testcase : testsuite) {
      node.child("testsuite") << std::get<1>(testcase);
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
    node.append_child("testsuites");

    for (const auto & testsuite : testsuites) {
      node.child("testsuites") << std::get<1>(testsuite);
    }

    return node;
  }

  template <typename... Ts>
  auto save(Ts &&... xs) const
  {
    pugi::xml_document document;

    document << *this;

    document.save_file(std::forward<decltype(xs)>(xs)...);
  }
};
}  // namespace junit

#endif  // JUNIT_EXPORTER__JUNIT_EXPORTER_HPP_
