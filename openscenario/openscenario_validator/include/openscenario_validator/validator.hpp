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

#ifndef OPENSCENARIO_VALIDATOR__VALIDATOR_HPP_
#define OPENSCENARIO_VALIDATOR__VALIDATOR_HPP_

#include <boost/filesystem.hpp>
#include <iostream>
#include <openscenario_validator/schema.hpp>
#include <sstream>
#include <stdexcept>
#include <system_error>
#include <xercesc/parsers/XercesDOMParser.hpp>
#include <xercesc/sax/HandlerBase.hpp>
#include <xercesc/util/XMLString.hpp>
#include <xercesc/validators/common/Grammar.hpp>

namespace openscenario_validator
{
class OpenSCENARIOValidator
{
  struct ErrorHandler : public xercesc::HandlerBase
  {
    using xercesc::HandlerBase::HandlerBase;

    template <typename String>
    auto makeErrorMessage(const String & category, const xercesc::SAXParseException & e)
    {
      // The following exception message is formatted to resemble GCC's
      // compilation error message.
      std::stringstream what;
      what << xercesc::XMLString::transcode(e.getSystemId()) << ":" << e.getLineNumber() << ":"
           << e.getColumnNumber() << ": " << category << ": "
           << xercesc::XMLString::transcode(e.getMessage());
      return what.str();
    }

    auto warning(const xercesc::SAXParseException & e) -> void override
    {
      std::cerr << makeErrorMessage("warning", e) << std::endl;
    }

    auto error(const xercesc::SAXParseException & e) -> void override
    {
      throw std::runtime_error(makeErrorMessage("error", e));
    }

    auto fatalError(const xercesc::SAXParseException & e) -> void override
    {
      throw std::runtime_error(makeErrorMessage("fatal", e));
    }
  };

  std::unique_ptr<xercesc::XercesDOMParser> parser;

  ErrorHandler error_handler;

  static constexpr auto schema_filepath = "/tmp/OpenSCENARIO-1.3.xsd";

public:
  OpenSCENARIOValidator()
  {
    xercesc::XMLPlatformUtils::Initialize();
    parser = std::make_unique<xercesc::XercesDOMParser>();
    if (auto file = std::ofstream(schema_filepath, std::ios::trunc)) {
      file << schema;
    } else {
      throw std::system_error(errno, std::system_category());
    }

    if (not parser->loadGrammar(schema_filepath, xercesc::Grammar::SchemaGrammarType)) {
      throw std::runtime_error(
        "Failed to load XSD schema. This is an unexpected error and an implementation issue. "
        "Please contact the developer.");
    } else {
      parser->setDoNamespaces(true);
      parser->setDoSchema(true);
      parser->setErrorHandler(&error_handler);
      parser->setExternalNoNamespaceSchemaLocation(schema_filepath);
      parser->setValidationSchemaFullChecking(true);
      parser->setValidationScheme(xercesc::XercesDOMParser::Val_Always);
    }
  }

  ~OpenSCENARIOValidator()
  {
    parser.release();
    xercesc::XMLPlatformUtils::Terminate();
  }

  auto validate(const boost::filesystem::path & xml_file) -> void
  {
    parser->parse(xml_file.string().c_str());
  }

  template <typename... Ts>
  auto operator()(Ts &&... xs) -> decltype(auto)
  {
    return validate(std::forward<decltype(xs)>(xs)...);
  }
};
}  // namespace openscenario_validator

#endif  //OPENSCENARIO_VALIDATOR__VALIDATOR_HPP_
