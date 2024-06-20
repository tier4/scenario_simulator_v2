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

#include <autoware_auto_perception_msgs/msg/traffic_light.hpp>
#include <autoware_auto_perception_msgs/msg/traffic_signal.hpp>
#include <autoware_perception_msgs/msg/traffic_signal.hpp>
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
#include <simulation_interface/conversions.hpp>
#include <stdexcept>
#include <traffic_simulator/hdmap_utils/hdmap_utils.hpp>
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
    } value;

    // clang-format off
    static_assert(static_cast<std::uint8_t>(green ) == 0b0000'0000);
    static_assert(static_cast<std::uint8_t>(yellow) == 0b0000'0001);
    static_assert(static_cast<std::uint8_t>(red   ) == 0b0000'0010);
    static_assert(static_cast<std::uint8_t>(white ) == 0b0000'0011);
    // clang-format on

    constexpr Color(const Value value = green) : value(value) {}

    Color(const std::string & name) : value(make(name)) {}

    static inline const std::unordered_map<std::string, Value> table{
      std::make_pair("amber", yellow),
      std::make_pair("green", green),
      std::make_pair("red", red),
      std::make_pair("white", white),
      std::make_pair("yellow", yellow),

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
    };

    // clang-format off
    static_assert(static_cast<std::uint8_t>(Category::circle) == 0b0000'0000);
    static_assert(static_cast<std::uint8_t>(Category::cross ) == 0b0000'0001);
    static_assert(static_cast<std::uint8_t>(Category::arrow ) == 0b0000'0010);
    // clang-format on

    enum Value : std::uint16_t {
      // clang-format off
      circle      =                      static_cast<std::uint8_t>(Category::circle),
      cross       =                      static_cast<std::uint8_t>(Category::cross ),
      left        = (0b0000'1000 << 8) | static_cast<std::uint8_t>(Category::arrow ),
      down        = (0b0000'0100 << 8) | static_cast<std::uint8_t>(Category::arrow ),
      up          = (0b0000'0010 << 8) | static_cast<std::uint8_t>(Category::arrow ),
      right       = (0b0000'0001 << 8) | static_cast<std::uint8_t>(Category::arrow ),
      lower_left  = (0b0000'1100 << 8) | static_cast<std::uint8_t>(Category::arrow ),
      upper_left  = (0b0000'1010 << 8) | static_cast<std::uint8_t>(Category::arrow ),
      lower_right = (0b0000'0101 << 8) | static_cast<std::uint8_t>(Category::arrow ),
      upper_right = (0b0000'0011 << 8) | static_cast<std::uint8_t>(Category::arrow ),
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

    // it will be removed when autoware_perception_msgs::msg::TrafficSignal is no longer supported
    explicit operator simulation_api_schema::TrafficLight() const
    {
      auto color = [this]() {
        switch (std::get<Color>(value).value) {
          case Color::green:
            return simulation_api_schema::TrafficLight_Color_GREEN;
          case Color::yellow:
            return simulation_api_schema::TrafficLight_Color_AMBER;
          case Color::red:
            return simulation_api_schema::TrafficLight_Color_RED;
          case Color::white:
            return simulation_api_schema::TrafficLight_Color_WHITE;
          default:
            throw common::SyntaxError(std::get<Color>(value), " is not supported color.");
        }
      };

      auto status = [this]() {
        switch (std::get<Status>(value).value) {
          case Status::solid_on:
            return simulation_api_schema::TrafficLight_Status_SOLID_ON;
          case Status::solid_off:
            return simulation_api_schema::TrafficLight_Status_SOLID_OFF;
          case Status::flashing:
            return simulation_api_schema::TrafficLight_Status_FLASHING;
          case Status::unknown:
            return simulation_api_schema::TrafficLight_Status_UNKNOWN_STATUS;
          default:
            throw common::SyntaxError(std::get<Status>(value), " is not supported as a status.");
        }
      };

      auto shape = [this]() {
        switch (std::get<Shape>(value).value) {
          case Shape::circle:
            return simulation_api_schema::TrafficLight_Shape_CIRCLE;
          case Shape::cross:
            return simulation_api_schema::TrafficLight_Shape_CROSS;
          case Shape::left:
            return simulation_api_schema::TrafficLight_Shape_LEFT_ARROW;
          case Shape::down:
            return simulation_api_schema::TrafficLight_Shape_DOWN_ARROW;
          case Shape::up:
            return simulation_api_schema::TrafficLight_Shape_UP_ARROW;
          case Shape::right:
            return simulation_api_schema::TrafficLight_Shape_RIGHT_ARROW;
          case Shape::lower_left:
            return simulation_api_schema::TrafficLight_Shape_DOWN_LEFT_ARROW;
          case Shape::lower_right:
            return simulation_api_schema::TrafficLight_Shape_DOWN_RIGHT_ARROW;
          case Shape::upper_left:
            return simulation_api_schema::TrafficLight_Shape_UP_LEFT_ARROW;
          case Shape::upper_right:
            return simulation_api_schema::TrafficLight_Shape_UP_RIGHT_ARROW;
          default:
            throw common::SyntaxError(std::get<Shape>(value), " is not supported as a shape.");
        }
      };

      simulation_api_schema::TrafficLight traffic_light_bulb_proto;
      traffic_light_bulb_proto.set_status(status());
      traffic_light_bulb_proto.set_shape(shape());
      traffic_light_bulb_proto.set_color(color());
      // NOTE: confidence will be overwritten in TrafficLight::operator simulation_api_schema::TrafficSignal()
      traffic_light_bulb_proto.set_confidence(1.0);

      return traffic_light_bulb_proto;
    }

    explicit operator autoware_auto_perception_msgs::msg::TrafficLight() const
    {
      auto color = [this]() {
        switch (std::get<Color>(value).value) {
          case Color::green:
            return autoware_auto_perception_msgs::msg::TrafficLight::GREEN;
          case Color::yellow:
            return autoware_auto_perception_msgs::msg::TrafficLight::AMBER;
          case Color::red:
            return autoware_auto_perception_msgs::msg::TrafficLight::RED;
          case Color::white:
            return autoware_auto_perception_msgs::msg::TrafficLight::WHITE;
          default:
            return autoware_auto_perception_msgs::msg::TrafficLight::UNKNOWN;
        }
      };

      auto status = [this]() {
        switch (std::get<Status>(value).value) {
          case Status::solid_on:
            return autoware_auto_perception_msgs::msg::TrafficLight::SOLID_ON;
          case Status::solid_off:
            return autoware_auto_perception_msgs::msg::TrafficLight::SOLID_OFF;
          case Status::flashing:
            return autoware_auto_perception_msgs::msg::TrafficLight::FLASHING;
          default:
            return autoware_auto_perception_msgs::msg::TrafficLight::UNKNOWN;
        }
      };

      auto shape = [this]() {
        switch (std::get<Shape>(value).value) {
          case Shape::circle:
            return autoware_auto_perception_msgs::msg::TrafficLight::CIRCLE;
          case Shape::cross:
            return autoware_auto_perception_msgs::msg::TrafficLight::CROSS;
          case Shape::left:
            return autoware_auto_perception_msgs::msg::TrafficLight::LEFT_ARROW;
          case Shape::down:
            return autoware_auto_perception_msgs::msg::TrafficLight::DOWN_ARROW;
          case Shape::up:
            return autoware_auto_perception_msgs::msg::TrafficLight::UP_ARROW;
          case Shape::right:
            return autoware_auto_perception_msgs::msg::TrafficLight::RIGHT_ARROW;
          case Shape::lower_left:
            return autoware_auto_perception_msgs::msg::TrafficLight::DOWN_LEFT_ARROW;
          case Shape::lower_right:
            return autoware_auto_perception_msgs::msg::TrafficLight::DOWN_RIGHT_ARROW;
          case Shape::upper_left:
            return autoware_auto_perception_msgs::msg::TrafficLight::UP_LEFT_ARROW;
          case Shape::upper_right:
            return autoware_auto_perception_msgs::msg::TrafficLight::UP_RIGHT_ARROW;
          default:
            return autoware_auto_perception_msgs::msg::TrafficLight::UNKNOWN;
        };
      };

      autoware_auto_perception_msgs::msg::TrafficLight msg;
      msg.color = color();
      msg.status = status();
      msg.shape = shape();
      // NOTE: confidence will be overwritten
      msg.confidence = 1.0;
      return msg;
    }

    // it will be removed when autoware_perception_msgs::msg::TrafficSignal is no longer supported
    explicit operator autoware_perception_msgs::msg::TrafficSignalElement() const
    {
      auto color = [this]() {
        switch (std::get<Color>(value).value) {
          case Color::green:
            return autoware_perception_msgs::msg::TrafficSignalElement::GREEN;
          case Color::yellow:
            return autoware_perception_msgs::msg::TrafficSignalElement::AMBER;
          case Color::red:
            return autoware_perception_msgs::msg::TrafficSignalElement::RED;
          case Color::white:
            return autoware_perception_msgs::msg::TrafficSignalElement::WHITE;
          default:
            throw common::SyntaxError(std::get<Color>(value), " is not supported color.");
        }
      };

      auto status = [this]() {
        switch (std::get<Status>(value).value) {
          case Status::solid_on:
            return autoware_perception_msgs::msg::TrafficSignalElement::SOLID_ON;
          case Status::solid_off:
            return autoware_perception_msgs::msg::TrafficSignalElement::SOLID_OFF;
          case Status::flashing:
            return autoware_perception_msgs::msg::TrafficSignalElement::FLASHING;
          default:
            throw common::SyntaxError(std::get<Status>(value), " is not supported color.");
        }
      };

      auto shape = [this]() {
        switch (std::get<Shape>(value).value) {
          case Shape::circle:
            return autoware_perception_msgs::msg::TrafficSignalElement::CIRCLE;
          case Shape::cross:
            return autoware_perception_msgs::msg::TrafficSignalElement::CROSS;
          case Shape::left:
            return autoware_perception_msgs::msg::TrafficSignalElement::LEFT_ARROW;
          case Shape::down:
            return autoware_perception_msgs::msg::TrafficSignalElement::DOWN_ARROW;
          case Shape::up:
            return autoware_perception_msgs::msg::TrafficSignalElement::UP_ARROW;
          case Shape::right:
            return autoware_perception_msgs::msg::TrafficSignalElement::RIGHT_ARROW;
          case Shape::lower_left:
            return autoware_perception_msgs::msg::TrafficSignalElement::DOWN_LEFT_ARROW;
          case Shape::lower_right:
            return autoware_perception_msgs::msg::TrafficSignalElement::DOWN_RIGHT_ARROW;
          case Shape::upper_left:
            return autoware_perception_msgs::msg::TrafficSignalElement::UP_LEFT_ARROW;
          case Shape::upper_right:
            return autoware_perception_msgs::msg::TrafficSignalElement::UP_RIGHT_ARROW;
          default:
            throw common::SyntaxError(std::get<Shape>(value), " is not supported color.");
        };
      };

      autoware_perception_msgs::msg::TrafficSignalElement msg;
      msg.color = color();
      msg.status = status();
      msg.shape = shape();
      // NOTE: confidence will be overwritten
      msg.confidence = 1.0;
      return msg;
    }
  };

  explicit TrafficLight(const lanelet::Id, const hdmap_utils::HdMapUtils &);

  const lanelet::Id way_id;

  const lanelet::Ids regulatory_elements_ids;

  double confidence = 1.0;

  std::set<Bulb> bulbs;

  const std::map<Bulb::Hash, std::optional<geometry_msgs::msg::Point>> positions;

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
    auto optional_position = [this](auto && bulb) {
      try {
        return positions.at(bulb.hash() & 0b1111'0000'1111'1111);  // NOTE: Ignore status
      } catch (const std::out_of_range &) {
        return std::optional<geometry_msgs::msg::Point>(std::nullopt);
      }
    };

    for (const auto & bulb : bulbs) {
      if (optional_position(bulb).has_value() and bulb.is(Shape::Category::circle)) {
        visualization_msgs::msg::Marker marker;
        marker.header.stamp = now;
        marker.header.frame_id = frame_id;
        marker.action = marker.ADD;
        marker.ns = "bulb";
        marker.id = way_id;
        marker.type = marker.SPHERE;
        marker.pose.position = optional_position(bulb).value();
        marker.pose.orientation = geometry_msgs::msg::Quaternion();
        marker.scale.x = 0.3;
        marker.scale.y = 0.3;
        marker.scale.z = 0.3;
        marker.color =
          color_names::makeColorMsg(boost::lexical_cast<std::string>(std::get<Color>(bulb.value)));
        markers.push_back(marker);
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

  // simulation_api_schema should not occur here, but it is necessary to transfer
  // "relation_ids" in proto - which is not needed when autoware_auto_perception_msgs::msg::TrafficSignal is used
  // it will be removed when autoware_perception_msgs::msg::TrafficSignal is no longer supported
  explicit operator simulation_api_schema::TrafficSignal() const
  {
    simulation_api_schema::TrafficSignal traffic_signal_proto;

    traffic_signal_proto.set_id(way_id);
    for (const auto relation_id : regulatory_elements_ids) {
      traffic_signal_proto.add_relation_ids(relation_id);
    }
    for (const auto & bulb : bulbs) {
      auto traffic_light_bulb_proto = static_cast<simulation_api_schema::TrafficLight>(bulb);
      traffic_light_bulb_proto.set_confidence(confidence);
      *traffic_signal_proto.add_traffic_light_status() = traffic_light_bulb_proto;
    }
    return traffic_signal_proto;
  }

  explicit operator autoware_auto_perception_msgs::msg::TrafficSignal() const
  {
    autoware_auto_perception_msgs::msg::TrafficSignal traffic_signal;
    traffic_signal.map_primitive_id = way_id;
    for (const auto & bulb : bulbs) {
      auto traffic_light_bulb = static_cast<autoware_auto_perception_msgs::msg::TrafficLight>(bulb);
      traffic_light_bulb.confidence = confidence;
      traffic_signal.lights.push_back(traffic_light_bulb);
    }
    return traffic_signal;
  }

  // it will be removed when autoware_perception_msgs::msg::TrafficSignal is no longer supported
  explicit operator std::vector<autoware_perception_msgs::msg::TrafficSignal>() const
  {
    // skip if the traffic light has no bulbs
    if (bulbs.empty()) {
      return {};
    } else {
      std::vector<autoware_perception_msgs::msg::TrafficSignal> traffic_signals;
      for (const auto & regulatory_element : regulatory_elements_ids) {
        autoware_perception_msgs::msg::TrafficSignal traffic_signal;
        traffic_signal.traffic_signal_id = regulatory_element;
        for (const auto & bulb : bulbs) {
          traffic_signal.elements.push_back(
            static_cast<autoware_perception_msgs::msg::TrafficSignalElement>(bulb));
          traffic_signals.push_back(traffic_signal);
        }
      }
      return traffic_signals;
    }
  }
};
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__TRAFFIC_LIGHTS__TRAFFIC_LIGHT_HPP_
