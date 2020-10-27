#ifndef SCENARIO_SIMULATOR__XMLRPC_METHOD_HPP_
#define SCENARIO_SIMULATOR__XMLRPC_METHOD_HPP_

#include <xmlrpcpp/XmlRpcServerMethod.h>
#include <xmlrpcpp/XmlRpcValue.h>
#include <functional>
#include <boost/optional.hpp>

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
