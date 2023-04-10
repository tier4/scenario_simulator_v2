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

#ifndef OPENSCENARIO_UTILITY__XML_VALIDATOR_HPP_
#define OPENSCENARIO_UTILITY__XML_VALIDATOR_HPP_

#include <boost/filesystem.hpp>
#include <xercesc/framework/LocalFileInputSource.hpp>
#include <xercesc/parsers/XercesDOMParser.hpp>
#include <xercesc/sax/ErrorHandler.hpp>
#include <xercesc/sax/HandlerBase.hpp>
#include <xercesc/util/PlatformUtils.hpp>
#include <xercesc/validators/common/Grammar.hpp>

namespace openscenario_utility
{
class XMLValidator
{
public:
  explicit XMLValidator(boost::filesystem::path xsd_file) : xsd_file(xsd_file)
  {
    // Initialize Xerces library
    xercesc::XMLPlatformUtils::Initialize();
  }

  ~XMLValidator()
  {
    // Terminate Xerces library
    xercesc::XMLPlatformUtils::Terminate();
  }


  [[nodiscard]] bool validate(const boost::filesystem::path & xml_file) noexcept
  {
    try {
      // Create a DOM parser
      xercesc::XercesDOMParser parser;

      // Load the XSD file
      parser.loadGrammar(xsd_file.string().c_str(), xercesc::Grammar::SchemaGrammarType, true);

      // Set the validation scheme
      xercesc::ErrorHandler * error_handler = new xercesc::HandlerBase();
      parser.setErrorHandler(error_handler);
      parser.setValidationScheme(xercesc::XercesDOMParser::Val_Auto);
      parser.setDoNamespaces(true);
      parser.setDoSchema(true);
      parser.setValidationConstraintFatal(true);

      // Parse the XML file
      parser.parse(xml_file.string().c_str());

      int error_count = parser.getErrorCount();
      delete error_handler;
      return error_count == 0;
    } catch (const xercesc::XMLException & ex) {
      std::cerr << "Error: " << ex.getMessage() << std::endl;
      return false;
    } catch (...) {
      std::cerr << "Error: Unknown exception" << std::endl;
      return false;
    }
  }

  boost::filesystem::path xsd_file;
};
}  // namespace openscenario_utility
#endif  //OPENSCENARIO_UTILITY__XML_VALIDATOR_HPP_
