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

#ifndef TRAFFIC_SIMULATOR__TRAFFIC_LIGHTS__TRAFFIC_LIGHT_HPP_
#define TRAFFIC_SIMULATOR__TRAFFIC_LIGHTS__TRAFFIC_LIGHT_HPP_

#include <simulation_interface/simulation_api_schema.pb.h>

#include <color_names/color_names.hpp>
#include <cstdint>
#include <geometry_msgs/msg/point.hpp>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <regex>
#include <scenario_simulator_exception/exception.hpp>
#include <set>
#include <stdexcept>
#include <traffic_simulator/hdmap_utils/hdmap_utils.hpp>
#include <traffic_simulator_msgs/msg/traffic_light_array_v1.hpp>
#include <tuple>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

namespace traffic_simulator
{
struct TrafficLight
{
  struct Color
  {
    enum Value : std::uint8_t {
      green,
      yellow,
      red,
      white,
      unknown,
    } value;

    // clang-format off
    static_assert(static_cast<std::uint8_t>(green  ) == 0b0000'0000);
    static_assert(static_cast<std::uint8_t>(yellow ) == 0b0000'0001);
    static_assert(static_cast<std::uint8_t>(red    ) == 0b0000'0010);
    static_assert(static_cast<std::uint8_t>(white  ) == 0b0000'0011);
    static_assert(static_cast<std::uint8_t>(unknown) == 0b0000'0100);
    // clang-format on

    constexpr Color(const Value value = green) : value(value) {}

    Color(const std::string & name) : value(make(name)) {}

    static inline const std::unordered_map<std::string, Value> table{
      std::make_pair("amber", yellow),
      std::make_pair("green", green),
      std::make_pair("red", red),
      std::make_pair("white", white),
      std::make_pair("yellow", yellow),
      std::make_pair("unknown", unknown),

      // BACKWARD COMPATIBILITY
      std::make_pair("Green", green),
      std::make_pair("Red", red),
      std::make_pair("Yellow", yellow),
    };

    static auto make(const std::string & name) -> Color;

    constexpr auto is(const Color given) const { return value == given; }

    constexpr operator Value() const noexcept { return value; }

    friend auto operator>>(std::istream & is, Color & color) -> std::istream &;

    friend auto operator<<(std::ostream & os, const Color & color) -> std::ostream &;
  };

  struct Status
  {
    enum Value : std::uint8_t {
      solid_on,
      solid_off,
      flashing,
      unknown,
    } value;

    // clang-format off
    static_assert(static_cast<std::uint8_t>(solid_on ) == 0b0000'0000);
    static_assert(static_cast<std::uint8_t>(solid_off) == 0b0000'0001);
    static_assert(static_cast<std::uint8_t>(flashing ) == 0b0000'0010);
    static_assert(static_cast<std::uint8_t>(unknown  ) == 0b0000'0011);
    // clang-format on

    constexpr Status(const Value value = solid_on) : value(value) {}

    Status(const std::string & name) : value(make(name)) {}

    static inline const std::unordered_map<std::string, Value> table{
      std::make_pair("solidOn", solid_on),
      std::make_pair("solidOff", solid_off),
      std::make_pair("flashing", flashing),
      std::make_pair("unknown", unknown),

      // BACKWARD COMPATIBILITY
      std::make_pair("Blank", solid_off),
      std::make_pair("none", solid_off),
    };

    static auto make(const std::string & name) -> Status;

    constexpr auto is(const Value given) const { return value == given; }

    constexpr operator bool() const { return value == solid_on or value == flashing; }

    constexpr operator Value() const noexcept { return value; }

    friend auto operator>>(std::istream & is, Status & status) -> std::istream &;

    friend auto operator<<(std::ostream & os, const Status & status) -> std::ostream &;
  };

  struct Shape
  {
    enum class Category : std::uint8_t {
      circle,
      cross,
      arrow,
      unknown,
    };

    // clang-format off
    static_assert(static_cast<std::uint8_t>(Category::circle ) == 0b0000'0000);
    static_assert(static_cast<std::uint8_t>(Category::cross  ) == 0b0000'0001);
    static_assert(static_cast<std::uint8_t>(Category::arrow  ) == 0b0000'0010);
    static_assert(static_cast<std::uint8_t>(Category::unknown) == 0b0000'0011);
    // clang-format on

    enum Value : std::uint16_t {
      // clang-format off
      circle      =                      static_cast<std::uint8_t>(Category::circle ),
      cross       =                      static_cast<std::uint8_t>(Category::cross  ),
      left        = (0b0000'1000 << 8) | static_cast<std::uint8_t>(Category::arrow  ),
      down        = (0b0000'0100 << 8) | static_cast<std::uint8_t>(Category::arrow  ),
      up          = (0b0000'0010 << 8) | static_cast<std::uint8_t>(Category::arrow  ),
      right       = (0b0000'0001 << 8) | static_cast<std::uint8_t>(Category::arrow  ),
      lower_left  = (0b0000'1100 << 8) | static_cast<std::uint8_t>(Category::arrow  ),
      upper_left  = (0b0000'1010 << 8) | static_cast<std::uint8_t>(Category::arrow  ),
      lower_right = (0b0000'0101 << 8) | static_cast<std::uint8_t>(Category::arrow  ),
      upper_right = (0b0000'0011 << 8) | static_cast<std::uint8_t>(Category::arrow  ),
      unknown     =                      static_cast<std::uint8_t>(Category::unknown),
      // clang-format on
    } value;

    // clang-format off
    static_assert(static_cast<std::uint16_t>(circle     ) == 0b0000'0000'0000'0000);
    static_assert(static_cast<std::uint16_t>(cross      ) == 0b0000'0000'0000'0001);
    static_assert(static_cast<std::uint16_t>(left       ) == 0b0000'1000'0000'0010);
    static_assert(static_cast<std::uint16_t>(down       ) == 0b0000'0100'0000'0010);
    static_assert(static_cast<std::uint16_t>(up         ) == 0b0000'0010'0000'0010);
    static_assert(static_cast<std::uint16_t>(right      ) == 0b0000'0001'0000'0010);
    static_assert(static_cast<std::uint16_t>(lower_left ) == 0b0000'1100'0000'0010);
    static_assert(static_cast<std::uint16_t>(upper_left ) == 0b0000'1010'0000'0010);
    static_assert(static_cast<std::uint16_t>(lower_right) == 0b0000'0101'0000'0010);
    static_assert(static_cast<std::uint16_t>(upper_right) == 0b0000'0011'0000'0010);
    static_assert(static_cast<std::uint16_t>(unknown    ) == 0b0000'0000'0000'0011);
    // clang-format on

    constexpr Shape(const Value value = circle) : value(value) {}

    Shape(const std::string & name) : value(make(name)) {}

    static inline const std::unordered_map<std::string, Shape::Value> table{
      std::make_pair("circle", Shape::circle),
      std::make_pair("cross", Shape::cross),
      std::make_pair("left", Shape::left),
      std::make_pair("down", Shape::down),
      std::make_pair("up", Shape::up),
      std::make_pair("right", Shape::right),
      std::make_pair("lowerLeft", Shape::lower_left),
      std::make_pair("upperLeft", Shape::upper_left),
      std::make_pair("lowerRight", Shape::lower_right),
      std::make_pair("upperRight", Shape::upper_right),
      std::make_pair("unknown", Shape::unknown),

      // BACKWARD COMPATIBILITY
      std::make_pair("straight", Shape::up),
    };

    static auto make(const std::string & name) -> Shape;

    constexpr auto category() const
    {
      return static_cast<Category>(static_cast<std::uint16_t>(value) & 0b1111'1111);
    }

    constexpr auto is(const Value given) const { return value == given; }

    constexpr auto is(const Category given) const { return category() == given; }

    constexpr operator Value() const noexcept { return value; }

    friend auto operator>>(std::istream & is, Shape & shape) -> std::istream &;

    friend auto operator<<(std::ostream & os, const Shape & shape) -> std::ostream &;
  };

  struct Bulb
  {
    using Value = std::tuple<Color, Status, Shape>;

    const Value value;

    using Hash = std::uint32_t;  // (Color::Value << 8 + 16) | (Status::Value << 16) | Shape::Value

    constexpr Bulb(const Value value) : value(value) {}

    constexpr Bulb(const Color color = {}, const Status status = {}, const Shape shape = {})
    : Bulb(std::forward_as_tuple(color, status, shape))
    {
    }

    Bulb(const std::string & name) : Bulb(make(name)) {}

    auto make(const std::string & s) -> Value;

    constexpr auto is(const Color color) const { return std::get<Color>(value).is(color); }

    constexpr auto is(const Status status) const { return std::get<Status>(value).is(status); }

    constexpr auto is(const Shape shape) const { return std::get<Shape>(value).is(shape); }

    constexpr auto is(const Shape::Category category) const
    {
      return std::get<Shape>(value).is(category);
    }

    constexpr auto hash() const -> Hash
    {
      return (static_cast<Hash>(std::get<Color>(value).value) << 24) |
             (static_cast<Hash>(std::get<Status>(value).value) << 16) |
             static_cast<Hash>(std::get<Shape>(value).value);
    }

    friend constexpr auto operator<(const Bulb & lhs, const Bulb & rhs) -> bool
    {
      return lhs.hash() < rhs.hash();
    }

    friend auto operator<<(std::ostream & os, const Bulb & bulb) -> std::ostream &;

    explicit operator simulation_api_schema::TrafficLight() const
    {
      const auto color = [this]() {
        auto color_message = simulation_api_schema::TrafficLight_Color_UNKNOWN_COLOR;
        switch (std::get<Color>(value).value) {
          case Color::green:
            color_message = simulation_api_schema::TrafficLight_Color_GREEN;
            break;
          case Color::yellow:
            color_message = simulation_api_schema::TrafficLight_Color_AMBER;
            break;
          case Color::red:
            color_message = simulation_api_schema::TrafficLight_Color_RED;
            break;
          case Color::white:
            color_message = simulation_api_schema::TrafficLight_Color_WHITE;
            break;
          case Color::unknown:
            color_message = simulation_api_schema::TrafficLight_Color_UNKNOWN_COLOR;
            break;
        }
        return color_message;
      };

      const auto status = [this]() {
        auto status_message = simulation_api_schema::TrafficLight_Status_UNKNOWN_STATUS;
        switch (std::get<Status>(value).value) {
          case Status::solid_on:
            status_message = simulation_api_schema::TrafficLight_Status_SOLID_ON;
            break;
          case Status::solid_off:
            status_message = simulation_api_schema::TrafficLight_Status_SOLID_OFF;
            break;
          case Status::flashing:
            status_message = simulation_api_schema::TrafficLight_Status_FLASHING;
            break;
          case Status::unknown:
            status_message = simulation_api_schema::TrafficLight_Status_UNKNOWN_STATUS;
            break;
        }
        return status_message;
      };

      const auto shape = [this]() {
        auto shape_message = simulation_api_schema::TrafficLight_Shape_UNKNOWN_SHAPE;
        switch (std::get<Shape>(value).value) {
          case Shape::circle:
            shape_message = simulation_api_schema::TrafficLight_Shape_CIRCLE;
            break;
          case Shape::cross:
            shape_message = simulation_api_schema::TrafficLight_Shape_CROSS;
            break;
          case Shape::left:
            shape_message = simulation_api_schema::TrafficLight_Shape_LEFT_ARROW;
            break;
          case Shape::down:
            shape_message = simulation_api_schema::TrafficLight_Shape_DOWN_ARROW;
            break;
          case Shape::up:
            shape_message = simulation_api_schema::TrafficLight_Shape_UP_ARROW;
            break;
          case Shape::right:
            shape_message = simulation_api_schema::TrafficLight_Shape_RIGHT_ARROW;
            break;
          case Shape::lower_left:
            shape_message = simulation_api_schema::TrafficLight_Shape_DOWN_LEFT_ARROW;
            break;
          case Shape::lower_right:
            shape_message = simulation_api_schema::TrafficLight_Shape_DOWN_RIGHT_ARROW;
            break;
          case Shape::upper_left:
            shape_message = simulation_api_schema::TrafficLight_Shape_UP_LEFT_ARROW;
            break;
          case Shape::upper_right:
            shape_message = simulation_api_schema::TrafficLight_Shape_UP_RIGHT_ARROW;
            break;
          case Shape::unknown:
            shape_message = simulation_api_schema::TrafficLight_Shape_UNKNOWN_SHAPE;
            break;
        }
        return shape_message;
      };

      simulation_api_schema::TrafficLight traffic_light_bulb_proto;
      traffic_light_bulb_proto.set_status(status());
      traffic_light_bulb_proto.set_shape(shape());
      traffic_light_bulb_proto.set_color(color());
      // NOTE: confidence will be overwritten in TrafficLight::operator simulation_api_schema::TrafficSignal()
      traffic_light_bulb_proto.set_confidence(1.0);

      return traffic_light_bulb_proto;
    }

    explicit operator traffic_simulator_msgs::msg::TrafficLightBulbV1() const
    {
      const auto color = [this] {
        auto color_message = traffic_simulator_msgs::msg::TrafficLightBulbV1::UNKNOWN;
        switch (std::get<Color>(value).value) {
          case Color::green:
            color_message = traffic_simulator_msgs::msg::TrafficLightBulbV1::GREEN;
            break;
          case Color::yellow:
            color_message = traffic_simulator_msgs::msg::TrafficLightBulbV1::AMBER;
            break;
          case Color::red:
            color_message = traffic_simulator_msgs::msg::TrafficLightBulbV1::RED;
            break;
          case Color::white:
            color_message = traffic_simulator_msgs::msg::TrafficLightBulbV1::WHITE;
            break;
          case Color::unknown:
            color_message = traffic_simulator_msgs::msg::TrafficLightBulbV1::UNKNOWN;
            break;
        }
        return color_message;
      };

      const auto status = [this] {
        auto status_message = traffic_simulator_msgs::msg::TrafficLightBulbV1::UNKNOWN;
        switch (std::get<Status>(value).value) {
          case Status::solid_on:
            status_message = traffic_simulator_msgs::msg::TrafficLightBulbV1::SOLID_ON;
            break;
          case Status::solid_off:
            status_message = traffic_simulator_msgs::msg::TrafficLightBulbV1::SOLID_OFF;
            break;
          case Status::flashing:
            status_message = traffic_simulator_msgs::msg::TrafficLightBulbV1::FLASHING;
            break;
          case Status::unknown:
            status_message = traffic_simulator_msgs::msg::TrafficLightBulbV1::UNKNOWN;
            break;
        }
        return status_message;
      };

      const auto shape = [this] {
        auto shape_message = traffic_simulator_msgs::msg::TrafficLightBulbV1::UNKNOWN;
        switch (std::get<Shape>(value).value) {
          case Shape::circle:
            shape_message = traffic_simulator_msgs::msg::TrafficLightBulbV1::CIRCLE;
            break;
          case Shape::cross:
            shape_message = traffic_simulator_msgs::msg::TrafficLightBulbV1::CROSS;
            break;
          case Shape::left:
            shape_message = traffic_simulator_msgs::msg::TrafficLightBulbV1::LEFT_ARROW;
            break;
          case Shape::down:
            shape_message = traffic_simulator_msgs::msg::TrafficLightBulbV1::DOWN_ARROW;
            break;
          case Shape::up:
            shape_message = traffic_simulator_msgs::msg::TrafficLightBulbV1::UP_ARROW;
            break;
          case Shape::right:
            shape_message = traffic_simulator_msgs::msg::TrafficLightBulbV1::RIGHT_ARROW;
            break;
          case Shape::lower_left:
            shape_message = traffic_simulator_msgs::msg::TrafficLightBulbV1::DOWN_LEFT_ARROW;
            break;
          case Shape::lower_right:
            shape_message = traffic_simulator_msgs::msg::TrafficLightBulbV1::DOWN_RIGHT_ARROW;
            break;
          case Shape::upper_left:
            shape_message = traffic_simulator_msgs::msg::TrafficLightBulbV1::UP_LEFT_ARROW;
            break;
          case Shape::upper_right:
            shape_message = traffic_simulator_msgs::msg::TrafficLightBulbV1::UP_RIGHT_ARROW;
            break;
          case Shape::unknown:
            shape_message = traffic_simulator_msgs::msg::TrafficLightBulbV1::UNKNOWN;
            break;
        }
        return shape_message;
      };

      traffic_simulator_msgs::msg::TrafficLightBulbV1 msg;
      msg.color = color();
      msg.status = status();
      msg.shape = shape();
      // NOTE: confidence will be overwritten
      msg.confidence = 1.0;
      // NOTE: unused data member 'enum_revision' for
      // traffic_simulator_msgs::msg::TrafficLightBulbV1
      return msg;
    }
  };

  explicit TrafficLight(const lanelet::Id, const hdmap_utils::HdMapUtils &);

  const lanelet::Id way_id;

  const lanelet::Ids regulatory_elements_ids;

  double confidence = 1.0;

  std::set<Bulb> bulbs;

  const std::map<Color, std::optional<geometry_msgs::msg::Point>> positions;

  auto clear() { bulbs.clear(); }

  auto contains(const Bulb & bulb) const { return bulbs.find(bulb) != std::end(bulbs); }

  auto contains(const Color & color, const Status & status, const Shape & shape) const
  {
    return contains(Bulb(color, status, shape));
  }

  auto contains(const std::string & name) const { return contains(Bulb(name)); }

  template <typename Markers, typename Now>
  auto draw(Markers & markers, const Now & now, const std::string & frame_id) const
  {
    auto create_bulb_marker = [&](const Color & color, const bool is_on) {
      visualization_msgs::msg::Marker marker;
      marker.header.stamp = now;
      marker.header.frame_id = frame_id;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.ns = "bulb";
      marker.id = way_id << 2 | static_cast<int>(color.value);
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.pose.position = positions.at(color).value();
      marker.pose.orientation = geometry_msgs::msg::Quaternion();
      marker.scale.x = 0.3;
      marker.scale.y = 0.3;
      marker.scale.z = 0.3;
      marker.color = color_names::makeColorMsg(boost::lexical_cast<std::string>(color));
      marker.color.a = is_on ? 1.0 : 0.3;
      return marker;
    };

    std::set<Color> added_colors;

    // Place on markers for actual bulbs
    for (const auto & bulb : bulbs) {
      // NOTE: Status is ignored intentionally
      if (bulb.is(Shape::Category::circle)) {
        const auto color = std::get<Color>(bulb.value);
        const auto position = positions.find(color);
        if (position == positions.end() or not position->second.has_value()) {
          continue;
        }

        markers.push_back(create_bulb_marker(color, true));
        added_colors.insert(color);
      }
    }

    // Place solidOff markers for other positions
    for (const auto & [color, position] : positions) {
      if (position.has_value() and added_colors.find(color) == added_colors.end()) {
        markers.push_back(create_bulb_marker(color, false));
      }
    }
  }

  template <typename... Ts>
  auto emplace(Ts &&... xs)
  {
    bulbs.emplace(std::forward<decltype(xs)>(xs)...);
  }

  auto empty() const { return bulbs.empty(); }

  auto set(const std::string & states) -> void;

  friend auto operator<<(std::ostream & os, const TrafficLight & traffic_light) -> std::ostream &;

  explicit operator simulation_api_schema::TrafficSignal() const
  {
    simulation_api_schema::TrafficSignal traffic_signal_proto;

    traffic_signal_proto.set_id(way_id);
    for (const auto & bulb : bulbs) {
      auto traffic_light_bulb_proto = static_cast<simulation_api_schema::TrafficLight>(bulb);
      traffic_light_bulb_proto.set_confidence(confidence);
      *traffic_signal_proto.add_traffic_light_status() = traffic_light_bulb_proto;
    }
    return traffic_signal_proto;
  }

  explicit operator traffic_simulator_msgs::msg::TrafficLightV1() const
  {
    traffic_simulator_msgs::msg::TrafficLightV1 traffic_signal;
    traffic_signal.lanelet_way_id = way_id;
    for (const auto & bulb : bulbs) {
      auto traffic_light_bulb = static_cast<traffic_simulator_msgs::msg::TrafficLightBulbV1>(bulb);
      traffic_light_bulb.confidence = confidence;
      traffic_signal.traffic_light_bulbs.push_back(traffic_light_bulb);
    }
    return traffic_signal;
  }
};
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__TRAFFIC_LIGHTS__TRAFFIC_LIGHT_HPP_
