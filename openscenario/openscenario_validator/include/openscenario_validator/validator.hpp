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
#include <stdexcept>
#include <xercesc/framework/MemBufInputSource.hpp>
#include <xercesc/parsers/XercesDOMParser.hpp>
#include <xercesc/sax/HandlerBase.hpp>
#include <xercesc/validators/common/Grammar.hpp>

namespace openscenario_validator
{
struct XMLPlatform
{
  XMLPlatform();

  ~XMLPlatform();
};

static XMLPlatform platform{};

class OpenSCENARIOValidator
{
  xercesc::XercesDOMParser parser;

  xercesc::MemBufInputSource input_source;

  std::unique_ptr<xercesc::HandlerBase> error_handler;

public:
  explicit OpenSCENARIOValidator()
  : input_source(reinterpret_cast<const XMLByte *>(schema.data()), schema.size(), "xsd"),
    error_handler(std::make_unique<xercesc::HandlerBase>())
  {
    if (not parser.loadGrammar(input_source, xercesc::Grammar::SchemaGrammarType)) {
      throw std::runtime_error(
        "Failed to load XSD schema. This is an unexpected error and an implementation issue. "
        "Please contact the developer.");
    } else {
      parser.setErrorHandler(error_handler.get());
      parser.setValidationScheme(xercesc::XercesDOMParser::Val_Auto);
      parser.setDoNamespaces(true);
      parser.setDoSchema(true);
      parser.setValidationConstraintFatal(true);
    }
  }

  [[nodiscard]] auto validate(const boost::filesystem::path & xml_file) noexcept -> bool
  {
    try {
      parser.parse(xml_file.string().c_str());
      return parser.getErrorCount() == 0;
    } catch (const xercesc::XMLException & ex) {
      std::cerr << "Error: " << ex.getMessage() << std::endl;
      return false;
    } catch (...) {
      std::cerr << "Error: Unknown exception" << std::endl;
      return false;
    }
  }

  template <typename... Ts>
  [[nodiscard]] auto operator()(Ts &&... xs) noexcept -> decltype(auto)
  {
    return validate(std::forward<decltype(xs)>(xs)...);
  }
};
}  // namespace openscenario_validator

#endif  //OPENSCENARIO_VALIDATOR__VALIDATOR_HPP_
