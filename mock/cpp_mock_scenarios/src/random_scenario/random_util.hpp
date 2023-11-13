#include <traffic_simulator/api/api.hpp>

#include <memory>
#include <chrono>
#include <vector>
#include <string>

using traffic_simulator::LaneletPose;
using traffic_simulator::helper::constructLaneletPose;
using traffic_simulator::lane_change::Direction;
using TLColor = traffic_simulator::TrafficLight::Color::Value;



enum class DIRECTION {
  CENTER,
  LEFT,
  RIGHT,
  VERY_LEFT,
  VERY_RIGHT
};

template <typename StateType>
class StateManager
{
private:
  std::vector<StateType> states_;
  std::vector<std::chrono::milliseconds> intervals_;
  std::chrono::milliseconds total_interval_{0};
  std::chrono::time_point<std::chrono::steady_clock> last_update_;
  size_t current_state_index_ = 0;

  void updateState()
  {
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_update_);

    if (elapsed > total_interval_) {
      elapsed %= total_interval_;
    }

    while (elapsed > intervals_[current_state_index_]) {
      elapsed -= intervals_[current_state_index_];
      current_state_index_ = (current_state_index_ + 1) % states_.size();
      last_update_ = now - elapsed;
    }
  }

public:
  StateManager(const std::vector<StateType> & states, const std::vector<double> & interval_secs)
  : states_(states)
  {
    for (double interval_sec : interval_secs) {
      intervals_.push_back(std::chrono::milliseconds(static_cast<int>(interval_sec * 1000)));
      total_interval_ += intervals_.back();
    }

    last_update_ = std::chrono::steady_clock::now();
  }

  StateManager(const std::vector<StateType> & states, const double interval_sec) : states_(states)
  {
    for (int i = 0; i < states.size(); ++i) {
      intervals_.push_back(std::chrono::milliseconds(static_cast<int>(interval_sec * 1000)));
      total_interval_ += intervals_.back();
    }
    last_update_ = std::chrono::steady_clock::now();
  }

  StateType getCurrentState()
  {
    updateState();
    return states_[current_state_index_];
  }
};


uint8_t get_entity_subtype(const std::string & entity_type)
{
  using traffic_simulator_msgs::msg::EntitySubtype;
  if (entity_type == "car") {
    return EntitySubtype::CAR;
  } else if (entity_type == "truck") {
    return EntitySubtype::TRUCK;
  } else if (entity_type == "bus") {
    return EntitySubtype::BUS;
  } else if (entity_type == "trailer") {
    return EntitySubtype::TRAILER;
  }
  return EntitySubtype::CAR;
}

geometry_msgs::msg::Pose createPose(const double x, const double y, const double z = 0.0)
{
  return geometry_msgs::build<geometry_msgs::msg::Pose>()
    .position(geometry_msgs::build<geometry_msgs::msg::Point>().x(x).y(y).z(z))
    .orientation(geometry_msgs::msg::Quaternion());
}

// Function to generate a random integer between min and max
int randomInt(int min, int max)
{
  if (min == max) return min;
  return min + rand() % (max - min + 1);
}

// Function to generate a random double between min and max
double randomDouble(double min, double max)
{
  double range = max - min;
  if (std::abs(range) < 0.0001) return min;
  double div = RAND_MAX / range;
  return min + (rand() / div);
}

std::string getOppositeTlColor(const std::string & color){
  if (color == "red") return "green";
  if (color == "amber") return "red";
  if (color == "green") return "red";

  throw std::invalid_argument("unexpected color.");
}

template <
  class OutputUnit = std::chrono::seconds, class InternalUnit = std::chrono::microseconds,
  class Clock = std::chrono::steady_clock>
class MyStopWatch
{
public:
  MyStopWatch() { tic(default_name); }

  void tic(const std::string & name = default_name) { t_start_[name] = Clock::now(); }

  void tic(const char * name) { tic(std::string(name)); }

  double toc(const std::string & name, const bool reset = false)
  {
    const auto t_start = t_start_.at(name);
    const auto t_end = Clock::now();
    const auto duration = std::chrono::duration_cast<InternalUnit>(t_end - t_start).count();

    if (reset) {
      t_start_[name] = Clock::now();
    }

    const auto one_sec = std::chrono::duration_cast<InternalUnit>(OutputUnit(1)).count();

    return static_cast<double>(duration) / one_sec;
  }

  double toc(const char * name, const bool reset = false) { return toc(std::string(name), reset); }

  double toc(const bool reset = false) { return toc(default_name, reset); }

private:
  using Time = std::chrono::time_point<Clock>;
  static constexpr const char * default_name{"__auto__"};

  std::unordered_map<std::string, Time> t_start_;
};
