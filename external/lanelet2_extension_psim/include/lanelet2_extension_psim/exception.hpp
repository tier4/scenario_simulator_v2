// Copyright 2015-2019 Tier IV, Inc. All rights reserved.
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

// Authors: Simon Thompson, Ryohsuke Mitsudome, Masaya Kataoka

#ifndef LANELET2_EXTENSION_PSIM__EXCEPTION_HPP_
#define LANELET2_EXTENSION_PSIM__EXCEPTION_HPP_

#include <exception>
#include <string>

namespace lanelet
{
class HdMapException : public std::exception
{
public:
  explicit HdMapException(const std::string & msg) : error_message_(msg) {}
  virtual ~HdMapException() throw() {}
  virtual const char * what() const throw() { return error_message_.c_str(); }

protected:
  std::string error_message_;
};

class HdMapFormatException : public std::exception
{
public:
  explicit HdMapFormatException(const std::string & msg) : error_message_(msg) {}
  virtual ~HdMapFormatException() throw() {}
  virtual const char * what() const throw() { return error_message_.c_str(); }

protected:
  std::string error_message_;
};
}  // namespace lanelet

#endif  // LANELET2_EXTENSION_PSIM__EXCEPTION_HPP_
