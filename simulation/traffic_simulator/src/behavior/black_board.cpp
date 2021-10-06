#include <traffic_simulator/behavior/black_board.hpp>

namespace entity_behavior
{
boost::any BlackBoard::getValue(const std::string & key) const
{
  if (data_.find(key) == data_.end()) {
    throw std::runtime_error("key : " + key + " does not exist.");
  }
  return data_.at(key);
}
}  // namespace entity_behavior
