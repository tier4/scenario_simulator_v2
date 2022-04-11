// Copyright 2015-2020 Tier IV, Inc. All rights reserved.
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

#include <autoware_auto_perception_msgs/msg/traffic_signal.hpp>
#include <color_names/color_names.hpp>
#include <cstdint>
#include <geometry_msgs/msg/point.hpp>
#include <iostream>
#include <limits>
#include <memory>
#include <regex>
#include <set>
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
    static_assert(static_cast<std::uint8_t>(green ) == 0b0000);
    static_assert(static_cast<std::uint8_t>(yellow) == 0b0001);
    static_assert(static_cast<std::uint8_t>(red   ) == 0b0010);
    static_assert(static_cast<std::uint8_t>(white ) == 0b0011);
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
    static_assert(static_cast<std::uint8_t>(solid_on ) == 0b0000);
    static_assert(static_cast<std::uint8_t>(solid_off) == 0b0001);
    static_assert(static_cast<std::uint8_t>(flashing ) == 0b0010);
    static_assert(static_cast<std::uint8_t>(unknown  ) == 0b0011);
    // clang-format on

    constexpr Status(const Value value = solid_on) : value(value) {}

    Status(const std::string & name) : value(make(name)) {}

    static inline const std::unordered_map<std::string, Value> table{
      std::make_pair("solidOn", solid_on),
      std::make_pair("solidOff", solid_off),
      std::make_pair("flashing", flashing),
      std::make_pair("unknown", unknown),

      // BACKWARD COMPATIBILITY
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
    static_assert(static_cast<std::uint8_t>(Category::circle) == 0b0000);
    static_assert(static_cast<std::uint8_t>(Category::cross ) == 0b0001);
    static_assert(static_cast<std::uint8_t>(Category::arrow ) == 0b0010);
    // clang-format on

    enum Value : std::uint16_t {
      // clang-format off
      circle      =                 static_cast<std::uint8_t>(Category::circle),
      cross       =                 static_cast<std::uint8_t>(Category::cross ),
      left        = (0b1000 << 4) | static_cast<std::uint8_t>(Category::arrow ),
      down        = (0b0100 << 4) | static_cast<std::uint8_t>(Category::arrow ),
      up          = (0b0010 << 4) | static_cast<std::uint8_t>(Category::arrow ),
      right       = (0b0001 << 4) | static_cast<std::uint8_t>(Category::arrow ),
      lower_left  = (0b1100 << 4) | static_cast<std::uint8_t>(Category::arrow ),
      upper_left  = (0b1010 << 4) | static_cast<std::uint8_t>(Category::arrow ),
      lower_right = (0b0101 << 4) | static_cast<std::uint8_t>(Category::arrow ),
      upper_right = (0b0011 << 4) | static_cast<std::uint8_t>(Category::arrow ),
      // clang-format on
    } value;

    // clang-format off
    static_assert(static_cast<std::uint16_t>(circle     ) == 0b0000'0000);
    static_assert(static_cast<std::uint16_t>(cross      ) == 0b0000'0001);
    static_assert(static_cast<std::uint16_t>(left       ) == 0b1000'0010);
    static_assert(static_cast<std::uint16_t>(down       ) == 0b0100'0010);
    static_assert(static_cast<std::uint16_t>(up         ) == 0b0010'0010);
    static_assert(static_cast<std::uint16_t>(right      ) == 0b0001'0010);
    static_assert(static_cast<std::uint16_t>(lower_left ) == 0b1100'0010);
    static_assert(static_cast<std::uint16_t>(upper_left ) == 0b1010'0010);
    static_assert(static_cast<std::uint16_t>(lower_right) == 0b0101'0010);
    static_assert(static_cast<std::uint16_t>(upper_right) == 0b0011'0010);
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
      return static_cast<Category>(static_cast<std::uint16_t>(value) & 0b1111);
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

    using Hash = std::uint32_t;  // (Color::Value << 12) | (Status::Value << 8) | Shape::Value

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
      return (static_cast<Hash>(std::get<Color>(value).value) << 12) |
             (static_cast<Hash>(std::get<Status>(value).value) << 8) |
             static_cast<Hash>(std::get<Shape>(value).value);
    }

    friend constexpr auto operator<(const Bulb & lhs, const Bulb & rhs) -> bool
    {
      return lhs.hash() < rhs.hash();
    }

    friend auto operator<<(std::ostream & os, const Bulb & bulb) -> std::ostream &;

    explicit operator autoware_auto_perception_msgs::msg::TrafficLight() const;
  };

  const std::int64_t id;

  std::set<Bulb> bulbs;

  const std::map<Bulb::Hash, boost::optional<geometry_msgs::msg::Point>> positions;

  explicit TrafficLight(const std::int64_t, hdmap_utils::HdMapUtils &);

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
    auto position = [this](auto && bulb) {
      try {
        return positions.at(bulb.hash() & 0b1111'0000'1111'1111).get();  // NOTE: Ignore status
      } catch (const std::out_of_range &) {
        throw common::scenario_simulator_exception::Error(
          "There is no position for bulb {", bulb, "}.");
      }
    };

    for (auto && bulb : bulbs) {
      if (bulb.is(Shape::Category::circle)) {
        visualization_msgs::msg::Marker marker;
        marker.header.stamp = now;
        marker.header.frame_id = frame_id;
        marker.action = marker.ADD;
        marker.ns = "bulb";
        marker.id = id;
        marker.type = marker.SPHERE;
        marker.pose.position = position(bulb);
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

  explicit operator autoware_auto_perception_msgs::msg::TrafficSignal() const;

  friend auto operator<<(std::ostream & os, const TrafficLight & traffic_light) -> std::ostream &;
};
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__TRAFFIC_LIGHTS__TRAFFIC_LIGHT_HPP_
