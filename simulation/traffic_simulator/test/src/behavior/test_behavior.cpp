#include <gtest/gtest.h>

#include <traffic_simulator/data_type/behavior.hpp>

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

TEST(Behavior, getRequestString_none)
{
  const auto req = traffic_simulator::behavior::Request::NONE;
  std::string str;
  EXPECT_NO_THROW(str = traffic_simulator::behavior::getRequestString(req));
  EXPECT_TRUE("none" == str);
}

TEST(Behavior, getRequestString_lane_change)
{
  const auto req = traffic_simulator::behavior::Request::LANE_CHANGE;
  std::string str;
  EXPECT_NO_THROW(str = traffic_simulator::behavior::getRequestString(req));
  EXPECT_TRUE("lane_change" == str);
}

TEST(Behavior, getRequestString_follow_lane)
{
  const auto req = traffic_simulator::behavior::Request::FOLLOW_LANE;
  std::string str;
  EXPECT_NO_THROW(str = traffic_simulator::behavior::getRequestString(req));
  EXPECT_TRUE("follow_lane" == str);
}

TEST(Behavior, getRequestString_follow_polyline_trajectory)
{
  const auto req = traffic_simulator::behavior::Request::FOLLOW_POLYLINE_TRAJECTORY;
  std::string str;
  EXPECT_NO_THROW(str = traffic_simulator::behavior::getRequestString(req));
  EXPECT_TRUE("follow_polyline_trajectory" == str);
}

TEST(Behavior, getRequestString_walk_straight)
{
  const auto req = traffic_simulator::behavior::Request::WALK_STRAIGHT;
  std::string str;
  EXPECT_NO_THROW(str = traffic_simulator::behavior::getRequestString(req));
  EXPECT_TRUE("walk_straight" == str);
}