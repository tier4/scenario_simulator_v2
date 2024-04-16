#include <gtest/gtest.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <traffic_simulator/behavior/route_planner.hpp>
#include <traffic_simulator/hdmap_utils/hdmap_utils.hpp>
#include <traffic_simulator/helper/helper.hpp>

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
