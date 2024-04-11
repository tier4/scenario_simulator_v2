#include <gtest/gtest.h>

#include <traffic_simulator/job/job.hpp>

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

TEST(Job, onUpdate)
{
  bool was_cleanup_func_called = false;
  auto update_func = [](const double) { return true; };
  auto cleanup_func = [&was_cleanup_func_called]() { was_cleanup_func_called = true; };
  auto type = traffic_simulator::job::Type::UNKNOWN;
  auto event = traffic_simulator::job::Event::POST_UPDATE;
  auto is_exclusive = true;

  auto job = traffic_simulator::job::Job(update_func, cleanup_func, type, is_exclusive, event);
  const double step_time = 0.0;
  job.onUpdate(step_time);

  EXPECT_TRUE(was_cleanup_func_called);
}