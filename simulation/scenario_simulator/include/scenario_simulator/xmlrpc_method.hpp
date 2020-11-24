// Copyright 2015-2020 TierIV.inc. All rights reserved.
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

#ifndef SCENARIO_SIMULATOR__XMLRPC_METHOD_HPP_
#define SCENARIO_SIMULATOR__XMLRPC_METHOD_HPP_

#include <xmlrpcpp/XmlRpcServerMethod.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include <boost/optional.hpp>

#include <functional>
#include <string>

namespace scenario_simulator
{
class XmlRpcMethod : public XmlRpc::XmlRpcServerMethod
{
public:
  explicit XmlRpcMethod(std::string const & name, XmlRpc::XmlRpcServer * server);
  void setFunction(std::function<void(XmlRpc::XmlRpcValue &, XmlRpc::XmlRpcValue &)> func);

private:
  void execute(XmlRpc::XmlRpcValue & params, XmlRpc::XmlRpcValue & result) override;
  boost::optional<std::function<void(XmlRpc::XmlRpcValue &, XmlRpc::XmlRpcValue &)>> func_;
};
}  // namespace scenario_simulator

#endif  // SCENARIO_SIMULATOR__XMLRPC_METHOD_HPP_
