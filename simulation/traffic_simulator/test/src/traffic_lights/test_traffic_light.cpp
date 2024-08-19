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

#include <gtest/gtest.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <boost/lexical_cast.hpp>
#include <regex>
#include <scenario_simulator_exception/exception.hpp>
#include <traffic_simulator/traffic_lights/traffic_light.hpp>

using TrafficLight = traffic_simulator::TrafficLight;
using Color = TrafficLight::Color;
using Status = TrafficLight::Status;
using Shape = TrafficLight::Shape;
using Bulb = TrafficLight::Bulb;

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

/**
 * @note Test object creation correctness.
 */
TEST(Color, Color)
{
  {
    const auto color = Color("green");

    EXPECT_TRUE(color == Color::green);
    EXPECT_TRUE(color.is(Color::green));
    EXPECT_TRUE(boost::lexical_cast<Color>("green") == Color::green);
    EXPECT_TRUE(boost::lexical_cast<std::string>(color) == "green");
  }

  {
    const auto color = Color("yellow");

    EXPECT_TRUE(color == Color::yellow);
    EXPECT_TRUE(color.is(Color::yellow));
    EXPECT_TRUE(boost::lexical_cast<Color>("yellow") == Color::yellow);
    EXPECT_TRUE(boost::lexical_cast<std::string>(color) == "yellow");
  }

  {
    const auto color = Color("red");

    EXPECT_TRUE(color == Color::red);
    EXPECT_TRUE(color.is(Color::red));
    EXPECT_TRUE(boost::lexical_cast<Color>("red") == Color::red);
    EXPECT_TRUE(boost::lexical_cast<std::string>(color) == "red");
  }

  {
    const auto color = Color("white");

    EXPECT_TRUE(color == Color::white);
    EXPECT_TRUE(color.is(Color::white));
    EXPECT_TRUE(boost::lexical_cast<Color>("white") == Color::white);
    EXPECT_TRUE(boost::lexical_cast<std::string>(color) == "white");
  }

  {
    const auto color = Color("amber");

    EXPECT_TRUE(color == Color::yellow);
    EXPECT_TRUE(color.is(Color::yellow));
    EXPECT_TRUE(boost::lexical_cast<Color>("amber") == Color::yellow);
    EXPECT_TRUE(boost::lexical_cast<std::string>(color) == "yellow");
  }
}

/**
 * @note Test basic functionality. Test whether the function
 * creates Color object appropriate to the argument.
 */
TEST(Color, make)
{
  {
    const auto color = Color::make("green");

    EXPECT_TRUE(color == Color::green);
    EXPECT_TRUE(color.is(Color::green));
    EXPECT_TRUE(boost::lexical_cast<Color>(color) == Color::green);
    EXPECT_TRUE(boost::lexical_cast<std::string>(color) == "green");
  }

  {
    const auto color = Color::make("yellow");

    EXPECT_TRUE(color == Color::yellow);
    EXPECT_TRUE(color.is(Color::yellow));
    EXPECT_TRUE(boost::lexical_cast<Color>(color) == Color::yellow);
    EXPECT_TRUE(boost::lexical_cast<std::string>(color) == "yellow");
  }

  {
    const auto color = Color::make("red");

    EXPECT_TRUE(color == Color::red);
    EXPECT_TRUE(color.is(Color::red));
    EXPECT_TRUE(boost::lexical_cast<Color>(color) == Color::red);
    EXPECT_TRUE(boost::lexical_cast<std::string>(color) == "red");
  }

  {
    const auto color = Color::make("white");

    EXPECT_TRUE(color == Color::white);
    EXPECT_TRUE(color.is(Color::white));
    EXPECT_TRUE(boost::lexical_cast<Color>(color) == Color::white);
    EXPECT_TRUE(boost::lexical_cast<std::string>(color) == "white");
  }

  {
    const auto color = Color::make("amber");

    EXPECT_TRUE(color == Color::yellow);
    EXPECT_TRUE(color.is(Color::yellow));
    EXPECT_TRUE(boost::lexical_cast<Color>(color) == Color::yellow);
    EXPECT_TRUE(boost::lexical_cast<std::string>(color) == "yellow");
  }
}

/**
 * @note Test basic functionality. Test function behavior when called with invalid name.
 */
TEST(Color, make_wrong)
{
  EXPECT_THROW(traffic_simulator::TrafficLight::Color::make("wrong_color"), common::SyntaxError);
}

/**
 * @note Test object creation correctness.
 */
TEST(Status, Status)
{
  {
    const auto status = Status("solidOn");

    EXPECT_TRUE(status == Status::solid_on);
    EXPECT_TRUE(status.is(Status::solid_on));
    EXPECT_TRUE(boost::lexical_cast<Status>("solidOn") == Status::solid_on);
    EXPECT_TRUE(boost::lexical_cast<std::string>(status) == "solidOn");
  }

  {
    const auto status = Status("solidOff");

    EXPECT_TRUE(status == Status::solid_off);
    EXPECT_TRUE(status.is(Status::solid_off));
    EXPECT_TRUE(boost::lexical_cast<Status>("solidOff") == Status::solid_off);
    EXPECT_TRUE(boost::lexical_cast<std::string>(status) == "solidOff");
  }

  {
    const auto status = Status("flashing");

    EXPECT_TRUE(status == Status::flashing);
    EXPECT_TRUE(status.is(Status::flashing));
    EXPECT_TRUE(boost::lexical_cast<Status>("flashing") == Status::flashing);
    EXPECT_TRUE(boost::lexical_cast<std::string>(status) == "flashing");
  }

  {
    const auto status = Status("unknown");

    EXPECT_TRUE(status == Status::unknown);
    EXPECT_TRUE(status.is(Status::unknown));
    EXPECT_TRUE(boost::lexical_cast<Status>("unknown") == Status::unknown);
    EXPECT_TRUE(boost::lexical_cast<std::string>(status) == "unknown");
  }
}

/**
 * @note Test basic functionality. Test whether the function creates Status object appropriate to the argument.
 */
TEST(Status, make)
{
  {
    const auto status = Status::make("solidOn");

    EXPECT_TRUE(status == Status::solid_on);
    EXPECT_TRUE(status.is(Status::solid_on));
    EXPECT_TRUE(boost::lexical_cast<Status>(status) == Status::solid_on);
    EXPECT_TRUE(boost::lexical_cast<std::string>(status) == "solidOn");
  }

  {
    const auto status = Status::make("solidOff");

    EXPECT_TRUE(status == Status::solid_off);
    EXPECT_TRUE(status.is(Status::solid_off));
    EXPECT_TRUE(boost::lexical_cast<Status>(status) == Status::solid_off);
    EXPECT_TRUE(boost::lexical_cast<std::string>(status) == "solidOff");
  }

  {
    const auto status = Status::make("flashing");

    EXPECT_TRUE(status == Status::flashing);
    EXPECT_TRUE(status.is(Status::flashing));
    EXPECT_TRUE(boost::lexical_cast<Status>(status) == Status::flashing);
    EXPECT_TRUE(boost::lexical_cast<std::string>(status) == "flashing");
  }

  {
    const auto status = Status::make("unknown");

    EXPECT_TRUE(status == Status::unknown);
    EXPECT_TRUE(status.is(Status::unknown));
    EXPECT_TRUE(boost::lexical_cast<Status>(status) == Status::unknown);
    EXPECT_TRUE(boost::lexical_cast<std::string>(status) == "unknown");
  }
}

/**
 * @note Test basic functionality. Test function behavior when called with invalid name.
 */
TEST(Status, make_wrong)
{
  EXPECT_THROW(traffic_simulator::TrafficLight::Status::make("wrong_status"), common::SyntaxError);
}

/**
 * @note Test object creation correctness.
 */
TEST(Shape, Shape)
{
  {
    const auto shape = Shape("circle");

    EXPECT_TRUE(shape == Shape::circle);
    EXPECT_TRUE(shape.is(Shape::circle));
    EXPECT_TRUE(shape.is(Shape::Category::circle));
    EXPECT_TRUE(shape.category() == Shape::Category::circle);
    EXPECT_TRUE(boost::lexical_cast<Shape>("circle") == Shape::circle);
    EXPECT_TRUE(boost::lexical_cast<std::string>(shape) == "circle");
  }

  {
    const auto shape = Shape("cross");

    EXPECT_TRUE(shape == Shape::cross);
    EXPECT_TRUE(shape.is(Shape::cross));
    EXPECT_TRUE(shape.is(Shape::Category::cross));
    EXPECT_TRUE(shape.category() == Shape::Category::cross);
    EXPECT_TRUE(boost::lexical_cast<Shape>("cross") == Shape::cross);
    EXPECT_TRUE(boost::lexical_cast<std::string>(shape) == "cross");
  }

  {
    const auto shape = Shape("left");

    EXPECT_TRUE(shape == Shape::left);
    EXPECT_TRUE(shape.is(Shape::left));
    EXPECT_TRUE(shape.is(Shape::Category::arrow));
    EXPECT_TRUE(shape.category() == Shape::Category::arrow);
    EXPECT_TRUE(boost::lexical_cast<Shape>("left") == Shape::left);
    EXPECT_TRUE(boost::lexical_cast<std::string>(shape) == "left");
  }

  {
    const auto shape = Shape("down");

    EXPECT_TRUE(shape == Shape::down);
    EXPECT_TRUE(shape.is(Shape::down));
    EXPECT_TRUE(shape.is(Shape::Category::arrow));
    EXPECT_TRUE(shape.category() == Shape::Category::arrow);
    EXPECT_TRUE(boost::lexical_cast<Shape>("down") == Shape::down);
    EXPECT_TRUE(boost::lexical_cast<std::string>(shape) == "down");
  }

  {
    const auto shape = Shape("up");

    EXPECT_TRUE(shape == Shape::up);
    EXPECT_TRUE(shape.is(Shape::up));
    EXPECT_TRUE(shape.is(Shape::Category::arrow));
    EXPECT_TRUE(shape.category() == Shape::Category::arrow);
    EXPECT_TRUE(boost::lexical_cast<Shape>("up") == Shape::up);
    EXPECT_TRUE(boost::lexical_cast<std::string>(shape) == "up");
  }

  {
    const auto shape = Shape("right");

    EXPECT_TRUE(shape == Shape::right);
    EXPECT_TRUE(shape.is(Shape::right));
    EXPECT_TRUE(shape.is(Shape::Category::arrow));
    EXPECT_TRUE(shape.category() == Shape::Category::arrow);
    EXPECT_TRUE(boost::lexical_cast<Shape>("right") == Shape::right);
    EXPECT_TRUE(boost::lexical_cast<std::string>(shape) == "right");
  }

  {
    const auto shape = Shape("lowerLeft");

    EXPECT_TRUE(shape == Shape::lower_left);
    EXPECT_TRUE(shape.is(Shape::lower_left));
    EXPECT_TRUE(shape.is(Shape::Category::arrow));
    EXPECT_TRUE(shape.category() == Shape::Category::arrow);
    EXPECT_TRUE(boost::lexical_cast<Shape>("lowerLeft") == Shape::lower_left);
    EXPECT_TRUE(boost::lexical_cast<std::string>(shape) == "lowerLeft");
  }

  {
    const auto shape = Shape("upperLeft");

    EXPECT_TRUE(shape == Shape::upper_left);
    EXPECT_TRUE(shape.is(Shape::upper_left));
    EXPECT_TRUE(shape.is(Shape::Category::arrow));
    EXPECT_TRUE(shape.category() == Shape::Category::arrow);
    EXPECT_TRUE(boost::lexical_cast<Shape>("upperLeft") == Shape::upper_left);
    EXPECT_TRUE(boost::lexical_cast<std::string>(shape) == "upperLeft");
  }

  {
    const auto shape = Shape("lowerRight");

    EXPECT_TRUE(shape == Shape::lower_right);
    EXPECT_TRUE(shape.is(Shape::lower_right));
    EXPECT_TRUE(shape.is(Shape::Category::arrow));
    EXPECT_TRUE(shape.category() == Shape::Category::arrow);
    EXPECT_TRUE(boost::lexical_cast<Shape>("lowerRight") == Shape::lower_right);
    EXPECT_TRUE(boost::lexical_cast<std::string>(shape) == "lowerRight");
  }

  {
    const auto shape = Shape("upperRight");

    EXPECT_TRUE(shape == Shape::upper_right);
    EXPECT_TRUE(shape.is(Shape::upper_right));
    EXPECT_TRUE(shape.is(Shape::Category::arrow));
    EXPECT_TRUE(shape.category() == Shape::Category::arrow);
    EXPECT_TRUE(boost::lexical_cast<Shape>("upperRight") == Shape::upper_right);
    EXPECT_TRUE(boost::lexical_cast<std::string>(shape) == "upperRight");
  }
}

/**
 * @note Test basic functionality. Test whether the function creates Shape object appropriate to the argument.
 */
TEST(Shape, make)
{
  {
    const auto shape = Shape::make("circle");

    EXPECT_TRUE(shape == Shape::circle);
    EXPECT_TRUE(shape.is(Shape::circle));
    EXPECT_TRUE(shape.is(Shape::Category::circle));
    EXPECT_TRUE(shape.category() == Shape::Category::circle);
    EXPECT_TRUE(boost::lexical_cast<Shape>("circle") == Shape::circle);
    EXPECT_TRUE(boost::lexical_cast<std::string>(shape) == "circle");
  }

  {
    const auto shape = Shape::make("cross");

    EXPECT_TRUE(shape == Shape::cross);
    EXPECT_TRUE(shape.is(Shape::cross));
    EXPECT_TRUE(shape.is(Shape::Category::cross));
    EXPECT_TRUE(shape.category() == Shape::Category::cross);
    EXPECT_TRUE(boost::lexical_cast<Shape>("cross") == Shape::cross);
    EXPECT_TRUE(boost::lexical_cast<std::string>(shape) == "cross");
  }

  {
    const auto shape = Shape::make("left");

    EXPECT_TRUE(shape == Shape::left);
    EXPECT_TRUE(shape.is(Shape::left));
    EXPECT_TRUE(shape.is(Shape::Category::arrow));
    EXPECT_TRUE(shape.category() == Shape::Category::arrow);
    EXPECT_TRUE(boost::lexical_cast<Shape>("left") == Shape::left);
    EXPECT_TRUE(boost::lexical_cast<std::string>(shape) == "left");
  }

  {
    const auto shape = Shape::make("down");

    EXPECT_TRUE(shape == Shape::down);
    EXPECT_TRUE(shape.is(Shape::down));
    EXPECT_TRUE(shape.is(Shape::Category::arrow));
    EXPECT_TRUE(shape.category() == Shape::Category::arrow);
    EXPECT_TRUE(boost::lexical_cast<Shape>("down") == Shape::down);
    EXPECT_TRUE(boost::lexical_cast<std::string>(shape) == "down");
  }

  {
    const auto shape = Shape::make("up");

    EXPECT_TRUE(shape == Shape::up);
    EXPECT_TRUE(shape.is(Shape::up));
    EXPECT_TRUE(shape.is(Shape::Category::arrow));
    EXPECT_TRUE(shape.category() == Shape::Category::arrow);
    EXPECT_TRUE(boost::lexical_cast<Shape>("up") == Shape::up);
    EXPECT_TRUE(boost::lexical_cast<std::string>(shape) == "up");
  }

  {
    const auto shape = Shape::make("right");

    EXPECT_TRUE(shape == Shape::right);
    EXPECT_TRUE(shape.is(Shape::right));
    EXPECT_TRUE(shape.is(Shape::Category::arrow));
    EXPECT_TRUE(shape.category() == Shape::Category::arrow);
    EXPECT_TRUE(boost::lexical_cast<Shape>("right") == Shape::right);
    EXPECT_TRUE(boost::lexical_cast<std::string>(shape) == "right");
  }

  {
    const auto shape = Shape::make("lowerLeft");

    EXPECT_TRUE(shape == Shape::lower_left);
    EXPECT_TRUE(shape.is(Shape::lower_left));
    EXPECT_TRUE(shape.is(Shape::Category::arrow));
    EXPECT_TRUE(shape.category() == Shape::Category::arrow);
    EXPECT_TRUE(boost::lexical_cast<Shape>("lowerLeft") == Shape::lower_left);
    EXPECT_TRUE(boost::lexical_cast<std::string>(shape) == "lowerLeft");
  }

  {
    const auto shape = Shape::make("upperLeft");

    EXPECT_TRUE(shape == Shape::upper_left);
    EXPECT_TRUE(shape.is(Shape::upper_left));
    EXPECT_TRUE(shape.is(Shape::Category::arrow));
    EXPECT_TRUE(shape.category() == Shape::Category::arrow);
    EXPECT_TRUE(boost::lexical_cast<Shape>("upperLeft") == Shape::upper_left);
    EXPECT_TRUE(boost::lexical_cast<std::string>(shape) == "upperLeft");
  }

  {
    const auto shape = Shape::make("lowerRight");

    EXPECT_TRUE(shape == Shape::lower_right);
    EXPECT_TRUE(shape.is(Shape::lower_right));
    EXPECT_TRUE(shape.is(Shape::Category::arrow));
    EXPECT_TRUE(shape.category() == Shape::Category::arrow);
    EXPECT_TRUE(boost::lexical_cast<Shape>("lowerRight") == Shape::lower_right);
    EXPECT_TRUE(boost::lexical_cast<std::string>(shape) == "lowerRight");
  }

  {
    const auto shape = Shape::make("upperRight");

    EXPECT_TRUE(shape == Shape::upper_right);
    EXPECT_TRUE(shape.is(Shape::upper_right));
    EXPECT_TRUE(shape.is(Shape::Category::arrow));
    EXPECT_TRUE(shape.category() == Shape::Category::arrow);
    EXPECT_TRUE(boost::lexical_cast<Shape>("upperRight") == Shape::upper_right);
    EXPECT_TRUE(boost::lexical_cast<std::string>(shape) == "upperRight");
  }
}

/**
 * @note Test basic functionality. Test function behavior when called with invalid name. 
 */
TEST(Shape, make_wrong)
{
  EXPECT_THROW(traffic_simulator::TrafficLight::Shape::make("wrong_shape"), common::SyntaxError);
}

/**
 * @note Test hashing function. An object must be assigned the same hash every time.
 */
TEST(Bulb, hash)
{
  // clang-format off
  static_assert(Bulb(Color::green,  Status::solid_on,  Shape::circle     ).hash() == 0b0000'0000'0000'0000'0000'0000'0000'0000);
  static_assert(Bulb(Color::yellow, Status::solid_on,  Shape::circle     ).hash() == 0b0000'0001'0000'0000'0000'0000'0000'0000);
  static_assert(Bulb(Color::red,    Status::solid_on,  Shape::circle     ).hash() == 0b0000'0010'0000'0000'0000'0000'0000'0000);
  static_assert(Bulb(Color::white,  Status::solid_on,  Shape::circle     ).hash() == 0b0000'0011'0000'0000'0000'0000'0000'0000);

  static_assert(Bulb(Color::green,  Status::solid_on,  Shape::circle     ).hash() == 0b0000'0000'0000'0000'0000'0000'0000'0000);
  static_assert(Bulb(Color::green,  Status::solid_off, Shape::circle     ).hash() == 0b0000'0000'0000'0001'0000'0000'0000'0000);
  static_assert(Bulb(Color::green,  Status::flashing,  Shape::circle     ).hash() == 0b0000'0000'0000'0010'0000'0000'0000'0000);
  static_assert(Bulb(Color::green,  Status::unknown,   Shape::circle     ).hash() == 0b0000'0000'0000'0011'0000'0000'0000'0000);

  static_assert(Bulb(Color::green,  Status::solid_on,  Shape::circle     ).hash() == 0b0000'0000'0000'0000'0000'0000'0000'0000);
  static_assert(Bulb(Color::green,  Status::solid_on,  Shape::cross      ).hash() == 0b0000'0000'0000'0000'0000'0000'0000'0001);
  static_assert(Bulb(Color::green,  Status::solid_on,  Shape::left       ).hash() == 0b0000'0000'0000'0000'0000'1000'0000'0010);
  static_assert(Bulb(Color::green,  Status::solid_on,  Shape::down       ).hash() == 0b0000'0000'0000'0000'0000'0100'0000'0010);
  static_assert(Bulb(Color::green,  Status::solid_on,  Shape::up         ).hash() == 0b0000'0000'0000'0000'0000'0010'0000'0010);
  static_assert(Bulb(Color::green,  Status::solid_on,  Shape::right      ).hash() == 0b0000'0000'0000'0000'0000'0001'0000'0010);
  static_assert(Bulb(Color::green,  Status::solid_on,  Shape::lower_left ).hash() == 0b0000'0000'0000'0000'0000'1100'0000'0010);
  static_assert(Bulb(Color::green,  Status::solid_on,  Shape::upper_left ).hash() == 0b0000'0000'0000'0000'0000'1010'0000'0010);
  static_assert(Bulb(Color::green,  Status::solid_on,  Shape::lower_right).hash() == 0b0000'0000'0000'0000'0000'0101'0000'0010);
  static_assert(Bulb(Color::green,  Status::solid_on,  Shape::upper_right).hash() == 0b0000'0000'0000'0000'0000'0011'0000'0010);
  // clang-format on
}

/**
 * @note Test basic functionality. Test Bulb comparisons with different configurations.
 */
TEST(Bulb, is)
{
  {
    constexpr auto bulb = Bulb(Color::red, Status::flashing, Shape::circle);

    EXPECT_TRUE(bulb.is(Color::red));
    EXPECT_TRUE(bulb.is(Status::flashing));
    EXPECT_TRUE(bulb.is(Shape::circle));
    EXPECT_TRUE(bulb.is(Shape::Category::circle));
  }

  {
    constexpr auto bulb = Bulb(Color::green, Status::solid_on, Shape::right);

    EXPECT_TRUE(bulb.is(Color::green));
    EXPECT_TRUE(bulb.is(Status::solid_on));
    EXPECT_TRUE(bulb.is(Shape::right));
    EXPECT_TRUE(bulb.is(Shape::Category::arrow));
  }

  {
    const auto bulb = Bulb("red flashing circle");

    EXPECT_TRUE(bulb.is(Color::red));
    EXPECT_TRUE(bulb.is(Status::flashing));
    EXPECT_TRUE(bulb.is(Shape::circle));
    EXPECT_TRUE(bulb.is(Shape::Category::circle));
  }

  {
    const auto bulb = Bulb("red flashing");

    EXPECT_TRUE(bulb.is(Color::red));
    EXPECT_TRUE(bulb.is(Status::flashing));
    EXPECT_TRUE(bulb.is(Shape::circle));
    EXPECT_TRUE(bulb.is(Shape::Category::circle));
  }

  {
    const auto bulb = Bulb("red");

    EXPECT_TRUE(bulb.is(Color::red));
    EXPECT_TRUE(bulb.is(Status::solid_on));
    EXPECT_TRUE(bulb.is(Shape::circle));
    EXPECT_TRUE(bulb.is(Shape::Category::circle));
  }

  {
    const auto bulb = Bulb("green solidOn right");

    EXPECT_TRUE(bulb.is(Color::green));
    EXPECT_TRUE(bulb.is(Status::solid_on));
    EXPECT_TRUE(bulb.is(Shape::right));
    EXPECT_TRUE(bulb.is(Shape::Category::arrow));
  }

  {
    const auto bulb = Bulb("green right");

    EXPECT_TRUE(bulb.is(Color::green));
    EXPECT_TRUE(bulb.is(Status::solid_on));
    EXPECT_TRUE(bulb.is(Shape::right));
    EXPECT_TRUE(bulb.is(Shape::Category::arrow));
  }
}

/**
 * @note Test basic functionality. Test whether the function creates Bulb object appropriate to the argument.
 */
TEST(Bulb, make)
{
  {
    constexpr auto bulb = Bulb(Color::red, Status::flashing, Shape::circle);

    EXPECT_TRUE(bulb.is(Color::red));
    EXPECT_TRUE(bulb.is(Status::flashing));
    EXPECT_TRUE(bulb.is(Shape::circle));
    EXPECT_TRUE(bulb.is(Shape::Category::circle));
  }

  {
    constexpr auto bulb = Bulb(Color::green, Status::solid_on, Shape::right);

    EXPECT_TRUE(bulb.is(Color::green));
    EXPECT_TRUE(bulb.is(Status::solid_on));
    EXPECT_TRUE(bulb.is(Shape::right));
    EXPECT_TRUE(bulb.is(Shape::Category::arrow));
  }

  {
    const auto bulb = Bulb("red flashing circle");

    EXPECT_TRUE(bulb.is(Color::red));
    EXPECT_TRUE(bulb.is(Status::flashing));
    EXPECT_TRUE(bulb.is(Shape::circle));
    EXPECT_TRUE(bulb.is(Shape::Category::circle));
  }

  {
    const auto bulb = Bulb("red flashing");

    EXPECT_TRUE(bulb.is(Color::red));
    EXPECT_TRUE(bulb.is(Status::flashing));
    EXPECT_TRUE(bulb.is(Shape::circle));
    EXPECT_TRUE(bulb.is(Shape::Category::circle));
  }

  {
    const auto bulb = Bulb("red");

    EXPECT_TRUE(bulb.is(Color::red));
    EXPECT_TRUE(bulb.is(Status::solid_on));
    EXPECT_TRUE(bulb.is(Shape::circle));
    EXPECT_TRUE(bulb.is(Shape::Category::circle));
  }

  {
    const auto bulb = Bulb("green solidOn right");

    EXPECT_TRUE(bulb.is(Color::green));
    EXPECT_TRUE(bulb.is(Status::solid_on));
    EXPECT_TRUE(bulb.is(Shape::right));
    EXPECT_TRUE(bulb.is(Shape::Category::arrow));
  }

  {
    const auto bulb = Bulb("green right");

    EXPECT_TRUE(bulb.is(Color::green));
    EXPECT_TRUE(bulb.is(Status::solid_on));
    EXPECT_TRUE(bulb.is(Shape::right));
    EXPECT_TRUE(bulb.is(Shape::Category::arrow));
  }
}

/**
 * @note Test basic functionality. Test function behavior when called with invalid name.
 */
TEST(Bulb, make_wrong)
{
  EXPECT_THROW(Bulb("red flashing wrong_shape"), common::SyntaxError);
  EXPECT_THROW(Bulb("red wrong_status circle"), common::SyntaxError);
  EXPECT_THROW(Bulb("wrong_color flashing circle"), common::SyntaxError);
  EXPECT_THROW(Bulb("wrong_color wrong_status wrong_shape"), common::SyntaxError);
}

/**
 * @note Test basic functionality. Test whether the TrafficLight message
 * is constructed configured according to the Bulb object.
 */
TEST(Bulb, operator_TrafficLight)
{
  {
    constexpr auto bulb = Bulb(Color::red, Status::flashing, Shape::circle);
    std::ostringstream oss;
    oss << bulb;
    EXPECT_TRUE(oss.str() == "red flashing circle");
  }

  {
    constexpr auto bulb = Bulb(Color::green, Status::solid_on, Shape::right);
    std::ostringstream oss;
    oss << bulb;
    EXPECT_TRUE(oss.str() == "green solidOn right");
  }

  {
    const auto bulb = Bulb("red flashing circle");
    std::ostringstream oss;
    oss << bulb;
    EXPECT_TRUE(oss.str() == "red flashing circle");
  }

  {
    const auto bulb = Bulb("red flashing");
    std::ostringstream oss;
    oss << bulb;
    EXPECT_TRUE(oss.str() == "red flashing circle");
  }

  {
    const auto bulb = Bulb("red");
    std::ostringstream oss;
    oss << bulb;
    EXPECT_TRUE(oss.str() == "red solidOn circle");
  }

  {
    const auto bulb = Bulb("green solidOn right");
    std::ostringstream oss;
    oss << bulb;
    EXPECT_TRUE(oss.str() == "green solidOn right");
  }

  {
    const auto bulb = Bulb("green right");
    std::ostringstream oss;
    oss << bulb;
    EXPECT_TRUE(oss.str() == "green solidOn right");
  }
}

class TrafficLightTest : public testing::Test
{
protected:
  TrafficLightTest()
  : map_manager(
      ament_index_cpp::get_package_share_directory("traffic_simulator") + "/map/lanelet2_map.osm",
      geographic_msgs::build<geographic_msgs::msg::GeoPoint>()
        .latitude(35.61836750154)
        .longitude(139.78066608243)
        .altitude(0.0))
  {
  }
  hdmap_utils::HdMapUtils map_manager;
};

/**
 * @note test if function correctly determines if a given bulb
 * is in the bulbs vector given a Color, Status, Shape triple.
 */
TEST_F(TrafficLightTest, contains_colorStatusShape)
{
  {
    auto traffic_light = TrafficLight(34802, map_manager);

    traffic_light.bulbs.emplace(Color::red, Status::flashing, Shape::circle);
    traffic_light.bulbs.emplace(Color::green, Status::solid_on, Shape::right);

    ASSERT_TRUE(traffic_light.bulbs.size() == static_cast<std::size_t>(2));
    EXPECT_TRUE(traffic_light.contains(Color::red, Status::flashing, Shape::circle));
    EXPECT_TRUE(traffic_light.contains(Color::green, Status::solid_on, Shape::right));
  }

  {
    auto traffic_light = TrafficLight(34802, map_manager);

    traffic_light.bulbs.emplace("red flashing circle");
    traffic_light.bulbs.emplace("green solidOn right");

    EXPECT_TRUE(traffic_light.contains(Color::red, Status::flashing, Shape::circle));
    EXPECT_TRUE(traffic_light.contains(Color::green, Status::solid_on, Shape::right));
  }
}

/**
 * @note Test function behavior with a valid string.
 */
TEST_F(TrafficLightTest, set_valid)
{
  auto traffic_light = TrafficLight(34802, map_manager);

  traffic_light.set("red flashing circle, green solidOn right");

  EXPECT_TRUE(traffic_light.contains(Color::red, Status::flashing, Shape::circle));
  EXPECT_TRUE(traffic_light.contains(Color::green, Status::solid_on, Shape::right));
}
