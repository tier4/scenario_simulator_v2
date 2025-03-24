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

#include <boost/filesystem.hpp>
#include <iostream>
#include <optional>
#include <regex>
#include <string>
#include <vector>
#include <xercesc/dom/DOM.hpp>
#include <xercesc/dom/impl/DOMCasts.hpp>
#include <xercesc/dom/impl/DOMDocumentImpl.hpp>
#include <xercesc/dom/impl/DOMImplementationImpl.hpp>
#include <xercesc/framework/LocalFileFormatTarget.hpp>
#include <xercesc/parsers/XercesDOMParser.hpp>
#include <xercesc/sax/HandlerBase.hpp>
#include <xercesc/util/PlatformUtils.hpp>
#include <xercesc/util/XMLString.hpp>
#include <xercesc/util/XMLUni.hpp>

namespace openscenario_preprocessor
{
using namespace xercesc;

inline std::string to_string(const XMLCh * ch)
{
  char * chars = XMLString::transcode(ch);
  std::string converted_string(chars);
  XMLString::release(&chars);
  return converted_string;
}

struct ReorderRequest
{
  std::string element_name;
  std::string content_model;
  std::vector<std::string> required_order;
};

inline void fixElementOrderRecursive(
  DOMNode * node, const std::vector<ReorderRequest> & reorder_requests)
{
  if (!node || node->getNodeType() != DOMNode::ELEMENT_NODE) {
    return;
  }

  auto current_element = dynamic_cast<DOMElement *>(node);

  std::map<std::string, std::vector<DOMElement *>> child_elements;
  DOMNodeList * child_nodes = current_element->getChildNodes();
  for (XMLSize_t i = 0; i < child_nodes->getLength(); i++) {
    DOMNode * child_node = child_nodes->item(i);
    if (child_node->getNodeType() != DOMNode::ELEMENT_NODE) continue;
    auto child_element = dynamic_cast<DOMElement *>(child_node);
    child_elements[to_string(child_element->getNodeName())].push_back(child_element);
  }

  std::optional<ReorderRequest> reorder_request_optional = std::nullopt;
  for (const auto & request : reorder_requests) {
    if (child_elements.find(request.element_name) != child_elements.end()) {
      bool all_required_order_included = true;
      for (const auto & [child_name, child_element] : child_elements) {
        if (auto it = std::find_if(
              request.required_order.begin(), request.required_order.end(),
              [child_name](const std::string & required_name) {
                return child_name == required_name;
              });
            it == request.required_order.end()) {
          all_required_order_included = false;
          break;
        }
      }

      if (all_required_order_included) {
        reorder_request_optional = request;
        break;
      }
    }
  }

  if (reorder_request_optional.has_value()) {
    ReorderRequest reorder_request = reorder_request_optional.value();
    std::cout << "re-order required: " << reorder_request.element_name << std::endl;
    std::cout << "\tcurrent order: ";
    for (const auto & child : child_elements) {
      std::cout << child.first << " ";
    }
    std::cout << std::endl;
    std::cout << "\trequired order: ";
    for (const auto & required_order : reorder_request.required_order) {
      std::cout << required_order << " ";
    }
    std::cout << std::endl;

    // delete first
    for (auto & [element_name, elements] : child_elements) {
      for (const auto & elem : elements) {
        try {
          dynamic_cast<DOMNode *>(current_element)->removeChild(elem);
        } catch (const DOMException & e) {
          std::cerr << "failed to delete: " << e.getMessage() << std::endl;
          throw;
        } catch (const std::exception & e) {
          std::cerr << "caught an exception: " << e.what() << std::endl;
          throw;
        } catch (...) {
          std::cerr << "caught unexpected exception: " << std::endl;
          throw;
        }
      }
    }

    // add by required order
    for (const auto & required_element_name : reorder_request.required_order) {
      if (auto required_element = child_elements.find(required_element_name);
          required_element != child_elements.end()) {
        for (auto & elem : required_element->second) {
          current_element->appendChild(elem);
        }
      }
    }
  }

  const DOMNodeList * updated_child_nodes = current_element->getChildNodes();
  for (XMLSize_t i = 0; i < updated_child_nodes->getLength(); i++) {
    DOMNode * child_node = updated_child_nodes->item(i);
    fixElementOrderRecursive(child_node, reorder_requests);
  }
}

class SimpleXMLFormatter
{
private:
  struct ErrorHandler : public xercesc::HandlerBase
  {
  public:
    std::vector<ReorderRequest> reorder_requests;

    ErrorHandler() {}
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
      auto message = [&]() {
        char * message_chars = XMLString::transcode(e.getMessage());
        std::string error_string(message_chars);
        XMLString::release(&message_chars);
        return error_string;
      }();

      std::regex error_regex(R"(element '([^']+)' is not allowed for content model '\(([^']+)\)')");
      std::smatch match;

      if (std::regex_search(message, match, error_regex)) {
        ReorderRequest reorder_request;
        reorder_request.element_name = match[1].str();
        reorder_request.content_model = match[2].str();

        {
          std::string elements = reorder_request.content_model;
          std::regex seq_regex(
            R"(([a-zA-Z0-9_]+)(?:,|$)?)");  // regex for each element in content model
          std::sregex_iterator it(elements.begin(), elements.end(), seq_regex);
          const std::sregex_iterator end;

          while (it != end) {
            reorder_request.required_order.push_back((*it)[1].str());
            ++it;
          }
        }

        std::cerr << "re-order request for '" << reorder_request.element_name
                  << "' is added. required content model: " << reorder_request.content_model
                  << " required order: ";
        for (const auto & required_element : reorder_request.required_order) {
          std::cerr << required_element << " ";
        }
        std::cerr << std::endl;
        reorder_requests.push_back(reorder_request);
      } else {
        std::cout << "ignore error： " << message << std::endl;
      }
    }

    auto fatalError(const xercesc::SAXParseException & e) -> void override
    {
      throw std::runtime_error(makeErrorMessage("fatal", e));
    }
  };

  std::unique_ptr<XercesDOMParser> parser = nullptr;

public:
  SimpleXMLFormatter()
  {
    try {
      XMLPlatformUtils::Initialize();
    } catch (...) {
      // do nothing
    }
  }

  ~SimpleXMLFormatter()
  {
    try {
      XMLPlatformUtils::Terminate();
    } catch (...) {
      // do nothing
    }
  }

  auto formatXML(
    const boost::filesystem::path & xml_path, const boost::filesystem::path & schema_path)
    -> std::optional<boost::filesystem::path>
  {
    boost::filesystem::path output_path = "/tmp/openscenario_preprocessor/formatted.xosc";

    if (formatXMLImpl(xml_path, schema_path, output_path)) {
      return output_path;
    } else {
      return std::nullopt;
    }
  }

  auto parse(const boost::filesystem::path & xml_path, const boost::filesystem::path & schema_path)
    -> std::optional<std::pair<DOMDocument *, std::vector<ReorderRequest>>>
  {
    try {
      parser = std::make_unique<XercesDOMParser>();
      parser->setDoNamespaces(true);
      parser->setDoSchema(true);

      ErrorHandler error_handler;
      parser->setErrorHandler(&error_handler);
      parser->setValidationSchemaFullChecking(true);
      parser->setValidationScheme(XercesDOMParser::Val_Always);
      parser->setExternalNoNamespaceSchemaLocation(schema_path.c_str());
      parser->setValidationConstraintFatal(false);
      parser->setIncludeIgnorableWhitespace(false);

      parser->parse(xml_path.c_str());
      DOMDocument * document = parser->getDocument();

      if (!document) {
        std::cerr << "failed to parse XML document" << std::endl;
        return std::nullopt;
      }

      return std::make_pair(document, error_handler.reorder_requests);
    } catch (const XMLException & e) {
      char * message = XMLString::transcode(e.getMessage());
      std::cerr << "XML error: " << message << std::endl;
      XMLString::release(&message);
      return std::nullopt;
    } catch (const DOMException & e) {
      char * message = XMLString::transcode(e.getMessage());
      std::cerr << "DOM error: " << message << std::endl;
      XMLString::release(&message);
      return std::nullopt;
    } catch (std::exception & e) {
      std::cerr << "unexpected error(std::exception): " << e.what() << std::endl;
      return std::nullopt;
    } catch (...) {
      std::cerr << "unexpected error(final guard)" << std::endl;
      return std::nullopt;
    }
  }

  auto formatXMLImpl(
    const boost::filesystem::path & xml_path, const boost::filesystem::path & schema_path,
    const boost::filesystem::path & output_path) -> std::optional<bool>
  {
    const auto ret = parse(xml_path, schema_path);
    if (not ret.has_value()) {
      return std::nullopt;
    }

    const auto reorder_requests = ret->second;
    if (reorder_requests.empty()) {
      return true;
    }

    auto document = ret->first;
    DOMElement * root = document->getDocumentElement();
    std::cout << "number of re-order requests:  " << std::dec << reorder_requests.size()
              << std::endl;

    std::set<std::string> unique_requested_name;
    std::vector<ReorderRequest> unique_reorder_requests;
    for (const auto & request : reorder_requests) {
      if (unique_requested_name.insert(request.element_name).second) {
        unique_reorder_requests.push_back(request);
      }
    }

    fixElementOrderRecursive(root, unique_reorder_requests);

    class XStr
    {
    public:
      XStr(const char * str) : fStr(nullptr) { fStr = XMLString::transcode(str); }
      ~XStr() { XMLString::release(&fStr); }
      const XMLCh * unicodeForm() const { return fStr; }
      XMLCh * fStr;
    };

    // cspell: ignore DOMLS
    DOMLSSerializer * serializer =
      ((DOMImplementationLS *)DOMImplementationRegistry::getDOMImplementation(
         XStr("LS").unicodeForm()))
        ->createLSSerializer();

    DOMConfiguration * serializer_config = serializer->getDomConfig();
    if (serializer_config->canSetParameter(XMLUni::fgDOMSchemaLocation, true)) {
      serializer_config->setParameter(XMLUni::fgDOMSchemaLocation, true);
    }

    DOMLSOutput * output = ((DOMImplementationLS *)DOMImplementationRegistry::getDOMImplementation(
                              XStr("LS").unicodeForm()))
                             ->createLSOutput();

    LocalFileFormatTarget target(output_path.c_str());
    output->setByteStream(&target);
    serializer->write(document, output);

    output->release();
    serializer->release();

    return false;
  }
};
}  // namespace openscenario_preprocessor
