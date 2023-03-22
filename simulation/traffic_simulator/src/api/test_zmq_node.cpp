#include <rclcpp/rclcpp.hpp>
#include <traffic_simulator/api/test_zmq.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  std::shared_ptr<traffic_simulator::TestZMQ> node{nullptr};
  try {
    node = std::make_shared<traffic_simulator::TestZMQ>();
    rclcpp::WallRate loop_rate{30};
    unsigned i{0};
    while (rclcpp::ok() & i++ < 100) {
      node->tick();
      rclcpp::spin_some(node);
      loop_rate.sleep();
    }
    auto result = node->execution_time_us / node->number_of_ticks;
    node->writeToFile("Avarange execution time [ms]: " + std::to_string(result));
    RCLCPP_WARN_STREAM(node->get_logger(), "Avarange execution time [ms]: " << result);
  } catch (const std::exception & e) {
    fprintf(stderr, "%s Exiting...\n", e.what());
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}