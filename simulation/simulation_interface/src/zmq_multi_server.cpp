#include <simulation_interface/zmq_multi_server.hpp>

namespace zeromq
{
template<typename Proto>
zmqpp::message toZMQ(const Proto & proto)
{
  zmqpp::message msg;
  std::string serialized_str;
  proto.SerializeToString(&serialized_str);
  msg << serialized_str;
  return msg;
}

template<typename Proto>
Proto toProto(const zmqpp::message & msg)
{
  std::string serialized_str = msg.get(0);
  Proto proto;
  proto.ParseFromString(serialized_str);
  return proto;
}

MultiServer::MultiServer(
  const simulation_interface::TransportProtocol & protocol,
  const simulation_interface::HostName & hostname,
  std::function<void(const simulation_api_schema::InitializeRequest &,
  simulation_api_schema::InitializeResponse &)> initialize_func,
  std::function<void(const simulation_api_schema::UpdateEntityStatusRequest &,
  simulation_api_schema::UpdateEntityStatusResponse &)> update_entity_status_func)
: context_(zmqpp::context()),
  type_(zmqpp::socket_type::reply),
  initialize_sock_(context_, type_),
  initialize_func_(initialize_func),
  update_entity_status_sock_(context_, type_),
  update_entity_status_func_(update_entity_status_func)
{
  initialize_sock_.bind(
    simulation_interface::getEndPoint(
      protocol, hostname,
      simulation_interface::ports::initialize));
  update_entity_status_sock_.bind(
    simulation_interface::getEndPoint(
      protocol, hostname,
      simulation_interface::ports::update_entity_status));
  poller_.add(initialize_sock_);
  poller_.add(update_entity_status_sock_);
  thread_ = std::thread(&MultiServer::start_poll, this);
}

void MultiServer::poll()
{
  poller_.poll(0.01);
  if (poller_.has_input(initialize_sock_)) {
    zmqpp::message request;
    initialize_sock_.receive(request);
    simulation_api_schema::InitializeResponse response;
    initialize_func_(toProto<simulation_api_schema::InitializeRequest>(request), response);
    auto msg = toZMQ(response);
    initialize_sock_.send(msg);
  }
  if (poller_.has_input(update_entity_status_sock_)) {
    zmqpp::message request;
    update_entity_status_sock_.receive(request);
    simulation_api_schema::UpdateEntityStatusResponse response;
    update_entity_status_func_(
      toProto<simulation_api_schema::UpdateEntityStatusRequest>(
        request), response);
    auto msg = toZMQ(response);
    update_entity_status_sock_.send(msg);
  }
}
void MultiServer::start_poll()
{
  while (rclcpp::ok()) {
    poll();
  }
}
}  // namespace zeromq
