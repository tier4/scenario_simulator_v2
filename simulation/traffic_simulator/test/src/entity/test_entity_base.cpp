#include <gtest/gtest.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <scenario_simulator_exception/exception.hpp>
#include <traffic_simulator/entity/entity_base.hpp>
#include <traffic_simulator/helper/helper.hpp>

#include "../catalogs.hpp"
#include "../expect_eq_macros.hpp"

class DummyEntity : public traffic_simulator::entity::EntityBase
{
public:
  explicit DummyEntity(
    const std::string & name, const traffic_simulator::CanonicalizedEntityStatus & entity_status,
    const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr)
  : EntityBase(name, entity_status, hdmap_utils_ptr)
  {
  }

  void appendToJobList(const std::function<bool(const double)> & func_on_update,
    const std::function<void()> & func_on_cleanup, traffic_simulator::job::Type type, bool exclusive,
    const traffic_simulator::job::Event event)
  {
    job_list_.append(func_on_update, func_on_cleanup, type, exclusive, event);
  }

  auto getCurrentAction() const -> std::string override { return {}; }

  auto getDefaultDynamicConstraints() const
    -> const traffic_simulator_msgs::msg::DynamicConstraints & override
  {
    static const auto default_dynamic_constraints = []() {
      auto dynamic_constraints = traffic_simulator_msgs::msg::DynamicConstraints();
      dynamic_constraints.max_speed = 0.0;
      dynamic_constraints.max_acceleration = 0.0;
      dynamic_constraints.max_acceleration_rate = 0.0;
      dynamic_constraints.max_deceleration = 0.0;
      dynamic_constraints.max_deceleration_rate = 0.0;
      return dynamic_constraints;
    }();
  
    return default_dynamic_constraints;
  }

  auto getEntityType() const -> const traffic_simulator_msgs::msg::EntityType & override
  {
    static traffic_simulator_msgs::msg::EntityType type;
    type.type = traffic_simulator_msgs::msg::EntityType::MISC_OBJECT;
    return type;
  }

  auto getEntityTypename() const -> const std::string & override
  {
    static const auto str = std::string("dummy");
    return str;
  }

  ~DummyEntity() override = default;

  auto getGoalPoses() -> std::vector<traffic_simulator::CanonicalizedLaneletPose> override { return {}; }

  std::optional<traffic_simulator_msgs::msg::Obstacle> getObstacle() override
  {
    return std::nullopt;
  }

  auto getRouteLanelets(double) -> lanelet::Ids override
  {
    THROW_SEMANTIC_ERROR("getRouteLanelets function cannot not use in MiscObjectEntity");
  }

  auto fillLaneletPose(traffic_simulator::CanonicalizedEntityStatus &) -> void override {}

  auto getWaypoints() -> const traffic_simulator_msgs::msg::WaypointsArray override
  {
    return traffic_simulator_msgs::msg::WaypointsArray();
  }

  void requestSpeedChange(double, bool) override {}

  void requestSpeedChange(const traffic_simulator::speed_change::RelativeTargetSpeed &, bool) override {}

  void requestAssignRoute(const std::vector<traffic_simulator::CanonicalizedLaneletPose> &) override {}

  void requestAssignRoute(const std::vector<geometry_msgs::msg::Pose> &) override {}

  void requestAcquirePosition(const traffic_simulator::CanonicalizedLaneletPose &) override {}

  void requestAcquirePosition(const geometry_msgs::msg::Pose &) override {}

  void requestSpeedChange(
    const double, const traffic_simulator::speed_change::Transition, const traffic_simulator::speed_change::Constraint,
    const bool) override {}

  auto getBehaviorParameter() const -> traffic_simulator_msgs::msg::BehaviorParameter override
  {
    THROW_SEMANTIC_ERROR("getBehaviorParameter function does not support in MiscObjectEntity.");
  }

  void setBehaviorParameter(const traffic_simulator_msgs::msg::BehaviorParameter &) override {}

  void setVelocityLimit(double) override {}

  void setAccelerationLimit(double) override {}

  void setAccelerationRateLimit(double) override {}

  void setDecelerationLimit(double) override {}

  void setDecelerationRateLimit(double) override {}
};

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
/*
  std::string path =
    ament_index_cpp::get_package_share_directory("traffic_simulator") + "/map/lanelet2_map.osm";
  geographic_msgs::msg::GeoPoint origin;
  origin.latitude = 35.61836750154;
  origin.longitude = 139.78066608243;
  auto map = std::make_shared<hdmap_utils::HdMapUtils>(path, origin);
*/

TEST(EntityBase, asFieldOperatorApplication)
{
  const std::string name("test");
  auto entity_status = traffic_simulator::EntityStatus();
  entity_status.name = name;
  entity_status.lanelet_pose_valid = false;

  auto canonicalized_entity_status = traffic_simulator::entity_status::CanonicalizedEntityStatus(entity_status, nullptr);

  auto dummy = DummyEntity(name, canonicalized_entity_status, nullptr);
  
  EXPECT_THROW(dummy.asFieldOperatorApplication(), std::runtime_error);
}

TEST(EntityBase, startNpcLogic)
{
  const std::string name("test");
  auto entity_status = traffic_simulator::EntityStatus();
  entity_status.name = name;
  entity_status.lanelet_pose_valid = false;

  auto canonicalized_entity_status = traffic_simulator::entity_status::CanonicalizedEntityStatus(entity_status, nullptr);

  auto dummy = DummyEntity(name, canonicalized_entity_status, nullptr);
  
  EXPECT_FALSE(dummy.isNpcLogicStarted());
  dummy.startNpcLogic();
  EXPECT_TRUE(dummy.isNpcLogicStarted());
}

TEST(EntityBase, activateOutOfRangeJob_speed)
{
  const std::string name("test");
  auto entity_status = traffic_simulator::EntityStatus();
  entity_status.name = name;
  entity_status.lanelet_pose_valid = false;

  auto canonicalized_entity_status = traffic_simulator::entity_status::CanonicalizedEntityStatus(entity_status, nullptr);

  auto dummy = DummyEntity(name, canonicalized_entity_status, nullptr);

  double min_velocity = 0.0;
  double max_velocity = 0.0;
  double min_acceleration = -100.0;
  double max_acceleration = 100.0;
  double min_jerk = -100.0;
  double max_jerk = 100.0;
  double velocity = 1.0;
  dummy.setLinearVelocity(velocity);
  dummy.activateOutOfRangeJob(min_velocity, max_velocity, min_acceleration, max_acceleration, min_jerk, max_jerk);
  double current_time = 0.0;
  double step_time = 0.0;
  EXPECT_NO_THROW(dummy.onUpdate(current_time, step_time));
  EXPECT_THROW(dummy.onPostUpdate(current_time, step_time), std::runtime_error);
}

TEST(EntityBase, activateOutOfRangeJob_acceleration)
{
  const std::string name("test");
  auto entity_status = traffic_simulator::EntityStatus();
  entity_status.name = name;
  entity_status.lanelet_pose_valid = false;

  auto canonicalized_entity_status = traffic_simulator::entity_status::CanonicalizedEntityStatus(entity_status, nullptr);

  auto dummy = DummyEntity(name, canonicalized_entity_status, nullptr);

  double min_velocity = -100.0;
  double max_velocity = 100.0;
  double min_acceleration = 0.0;
  double max_acceleration = 0.0;
  double min_jerk = -100.0;
  double max_jerk = 100.0;
  double acceleration = 1.0;
  dummy.setLinearAcceleration(acceleration);
  dummy.activateOutOfRangeJob(min_velocity, max_velocity, min_acceleration, max_acceleration, min_jerk, max_jerk);
  double current_time = 0.0;
  double step_time = 0.0;
  EXPECT_NO_THROW(dummy.onUpdate(current_time, step_time));
  EXPECT_THROW(dummy.onPostUpdate(current_time, step_time), std::runtime_error);
}

TEST(EntityBase, activateOutOfRangeJob_jerk)
{
  const std::string name("test");
  auto entity_status = traffic_simulator::EntityStatus();
  entity_status.name = name;
  entity_status.lanelet_pose_valid = false;

  auto canonicalized_entity_status = traffic_simulator::entity_status::CanonicalizedEntityStatus(entity_status, nullptr);

  auto dummy = DummyEntity(name, canonicalized_entity_status, nullptr);

  double min_velocity = -100.0;
  double max_velocity = 100.0;
  double min_acceleration = -100.0;
  double max_acceleration = 100.0;
  double min_jerk = 0.0;
  double max_jerk = 0.0;
  double jerk = 1.0;
  dummy.setLinearJerk(jerk);
  dummy.activateOutOfRangeJob(min_velocity, max_velocity, min_acceleration, max_acceleration, min_jerk, max_jerk);
  double current_time = 0.0;
  double step_time = 0.0;
  EXPECT_NO_THROW(dummy.onUpdate(current_time, step_time));
  EXPECT_THROW(dummy.onPostUpdate(current_time, step_time), std::runtime_error);
}

TEST(EntityBase, onUpdate)
{
  const std::string name("test");
  auto entity_status = traffic_simulator::EntityStatus();
  entity_status.name = name;
  entity_status.lanelet_pose_valid = false;

  auto canonicalized_entity_status = traffic_simulator::entity_status::CanonicalizedEntityStatus(entity_status, nullptr);

  auto dummy = DummyEntity(name, canonicalized_entity_status, nullptr);

  bool first_cleanup = false;
  bool first_update = false;
  bool second_cleanup = false;
  bool second_update = false;

  auto first_update_func = [&first_update](const double) { return first_update = true; };
  auto first_cleanup_func = [&first_cleanup]() { first_cleanup = true; };
  auto second_update_func = [&second_update](const double) { return second_update = true; };
  auto second_cleanup_func = [&second_cleanup]() { second_cleanup = true; };

  auto type_first = traffic_simulator::job::Type::LINEAR_VELOCITY;
  auto type_second = traffic_simulator::job::Type::LINEAR_ACCELERATION;
  auto first_event = traffic_simulator::job::Event::PRE_UPDATE;
  auto second_event = traffic_simulator::job::Event::POST_UPDATE;
  auto is_exclusive = true;

  dummy.appendToJobList(first_update_func, first_cleanup_func, type_first, is_exclusive, first_event);
  dummy.appendToJobList(second_update_func, second_cleanup_func, type_second, is_exclusive, second_event);

  double current_time = 0.0;
  double step_time = 0.0;
  dummy.onUpdate(current_time, step_time);

  EXPECT_TRUE(first_cleanup);
  EXPECT_TRUE(first_update);
  EXPECT_FALSE(second_cleanup);
  EXPECT_FALSE(second_update);
}

TEST(EntityBase, onPostUpdate)
{
  const std::string name("test");
  auto entity_status = traffic_simulator::EntityStatus();
  entity_status.name = name;
  entity_status.lanelet_pose_valid = false;

  auto canonicalized_entity_status = traffic_simulator::entity_status::CanonicalizedEntityStatus(entity_status, nullptr);

  auto dummy = DummyEntity(name, canonicalized_entity_status, nullptr);

  bool first_cleanup = false;
  bool first_update = false;
  bool second_cleanup = false;
  bool second_update = false;

  auto first_update_func = [&first_update](const double) { return first_update = true; };
  auto first_cleanup_func = [&first_cleanup]() { first_cleanup = true; };
  auto second_update_func = [&second_update](const double) { return second_update = true; };
  auto second_cleanup_func = [&second_cleanup]() { second_cleanup = true; };

  auto type_first = traffic_simulator::job::Type::LINEAR_VELOCITY;
  auto type_second = traffic_simulator::job::Type::LINEAR_ACCELERATION;
  auto first_event = traffic_simulator::job::Event::PRE_UPDATE;
  auto second_event = traffic_simulator::job::Event::POST_UPDATE;
  auto is_exclusive = true;

  dummy.appendToJobList(first_update_func, first_cleanup_func, type_first, is_exclusive, first_event);
  dummy.appendToJobList(second_update_func, second_cleanup_func, type_second, is_exclusive, second_event);

  double current_time = 0.0;
  double step_time = 0.0;
  dummy.onPostUpdate(current_time, step_time);

  EXPECT_FALSE(first_cleanup);
  EXPECT_FALSE(first_update);
  EXPECT_TRUE(second_cleanup);
  EXPECT_TRUE(second_update);
}