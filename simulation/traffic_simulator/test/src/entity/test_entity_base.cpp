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

  traffic_simulator_msgs::msg::BehaviorParameter behavior_parameter;
  auto getBehaviorParameter() const -> traffic_simulator_msgs::msg::BehaviorParameter override
  {
    return behavior_parameter;
  }

  void setBehaviorParameter(const traffic_simulator_msgs::msg::BehaviorParameter & params) override
  {
    behavior_parameter = params;
  }

  void appendToJobList(
    const std::function<bool(const double)> & func_on_update,
    const std::function<void()> & func_on_cleanup, traffic_simulator::job::Type type,
    bool exclusive, const traffic_simulator::job::Event event)
  {
    job_list_.append(func_on_update, func_on_cleanup, type, exclusive, event);
  }

  auto getCurrentAction() const -> std::string override { return {}; }

  auto getDefaultDynamicConstraints() const
    -> const traffic_simulator_msgs::msg::DynamicConstraints & override
  {
    static const auto default_dynamic_constraints = []() {
      auto dynamic_constraints = traffic_simulator_msgs::msg::DynamicConstraints();
      dynamic_constraints.max_speed = 3.0;
      dynamic_constraints.max_acceleration = 5.0;
      dynamic_constraints.max_acceleration_rate = 7.0;
      dynamic_constraints.max_deceleration = 11.0;
      dynamic_constraints.max_deceleration_rate = 13.0;
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

  auto getGoalPoses() -> std::vector<traffic_simulator::CanonicalizedLaneletPose> override
  {
    return {};
  }

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

  void setVelocityLimit(double) override {}

  void setAccelerationLimit(double) override {}

  void setAccelerationRateLimit(double) override {}

  void setDecelerationLimit(double) override {}

  void setDecelerationRateLimit(double) override {}

  void requestSpeedChange(double, bool) override {}

  void requestSpeedChange(
    const traffic_simulator::speed_change::RelativeTargetSpeed &, bool) override
  {
  }

  void requestAssignRoute(const std::vector<traffic_simulator::CanonicalizedLaneletPose> &) override
  {
  }

  void requestAssignRoute(const std::vector<geometry_msgs::msg::Pose> &) override {}

  void requestAcquirePosition(const traffic_simulator::CanonicalizedLaneletPose &) override {}

  void requestAcquirePosition(const geometry_msgs::msg::Pose &) override {}

  void requestSpeedChange(
    const double, const traffic_simulator::speed_change::Transition,
    const traffic_simulator::speed_change::Constraint, const bool) override
  {
  }
};

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

TEST(EntityBase, asFieldOperatorApplication)
{
  const std::string name("test");
  auto entity_status = traffic_simulator::EntityStatus();
  entity_status.name = name;
  entity_status.lanelet_pose_valid = false;

  auto canonicalized_entity_status =
    traffic_simulator::entity_status::CanonicalizedEntityStatus(entity_status, nullptr);

  auto dummy = DummyEntity(name, canonicalized_entity_status, nullptr);

  EXPECT_THROW(dummy.asFieldOperatorApplication(), std::runtime_error);
}

TEST(EntityBase, startNpcLogic)
{
  const std::string name("test");
  auto entity_status = traffic_simulator::EntityStatus();
  entity_status.name = name;
  entity_status.lanelet_pose_valid = false;

  auto canonicalized_entity_status =
    traffic_simulator::entity_status::CanonicalizedEntityStatus(entity_status, nullptr);

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

  auto canonicalized_entity_status =
    traffic_simulator::entity_status::CanonicalizedEntityStatus(entity_status, nullptr);

  auto dummy = DummyEntity(name, canonicalized_entity_status, nullptr);

  double min_velocity = 0.0;
  double max_velocity = 0.0;
  double min_acceleration = -100.0;
  double max_acceleration = 100.0;
  double min_jerk = -100.0;
  double max_jerk = 100.0;
  double velocity = 1.0;
  dummy.setLinearVelocity(velocity);
  dummy.activateOutOfRangeJob(
    min_velocity, max_velocity, min_acceleration, max_acceleration, min_jerk, max_jerk);
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

  auto canonicalized_entity_status =
    traffic_simulator::entity_status::CanonicalizedEntityStatus(entity_status, nullptr);

  auto dummy = DummyEntity(name, canonicalized_entity_status, nullptr);

  double min_velocity = -100.0;
  double max_velocity = 100.0;
  double min_acceleration = 0.0;
  double max_acceleration = 0.0;
  double min_jerk = -100.0;
  double max_jerk = 100.0;
  double acceleration = 1.0;
  dummy.setLinearAcceleration(acceleration);
  dummy.activateOutOfRangeJob(
    min_velocity, max_velocity, min_acceleration, max_acceleration, min_jerk, max_jerk);
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

  auto canonicalized_entity_status =
    traffic_simulator::entity_status::CanonicalizedEntityStatus(entity_status, nullptr);

  auto dummy = DummyEntity(name, canonicalized_entity_status, nullptr);

  double min_velocity = -100.0;
  double max_velocity = 100.0;
  double min_acceleration = -100.0;
  double max_acceleration = 100.0;
  double min_jerk = 0.0;
  double max_jerk = 0.0;
  double jerk = 1.0;
  dummy.setLinearJerk(jerk);
  dummy.activateOutOfRangeJob(
    min_velocity, max_velocity, min_acceleration, max_acceleration, min_jerk, max_jerk);
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

  auto canonicalized_entity_status =
    traffic_simulator::entity_status::CanonicalizedEntityStatus(entity_status, nullptr);

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

  dummy.appendToJobList(
    first_update_func, first_cleanup_func, type_first, is_exclusive, first_event);
  dummy.appendToJobList(
    second_update_func, second_cleanup_func, type_second, is_exclusive, second_event);

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

  auto canonicalized_entity_status =
    traffic_simulator::entity_status::CanonicalizedEntityStatus(entity_status, nullptr);

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

  dummy.appendToJobList(
    first_update_func, first_cleanup_func, type_first, is_exclusive, first_event);
  dummy.appendToJobList(
    second_update_func, second_cleanup_func, type_second, is_exclusive, second_event);

  double current_time = 0.0;
  double step_time = 0.0;
  dummy.onPostUpdate(current_time, step_time);

  EXPECT_FALSE(first_cleanup);
  EXPECT_FALSE(first_update);
  EXPECT_TRUE(second_cleanup);
  EXPECT_TRUE(second_update);
}

TEST(EntityBase, resetDynamicConstraints)
{
  const std::string name("test");
  auto entity_status = traffic_simulator::EntityStatus();
  entity_status.name = name;
  entity_status.lanelet_pose_valid = false;

  auto canonicalized_entity_status =
    traffic_simulator::entity_status::CanonicalizedEntityStatus(entity_status, nullptr);

  auto dummy = DummyEntity(name, canonicalized_entity_status, nullptr);

  auto default_constraints = dummy.getDefaultDynamicConstraints();
  dummy.resetDynamicConstraints();
  auto current_constraints = dummy.getDynamicConstraints();

  EXPECT_EQ(default_constraints.max_speed, current_constraints.max_speed);
  EXPECT_EQ(default_constraints.max_acceleration, current_constraints.max_acceleration);
  EXPECT_EQ(default_constraints.max_deceleration, current_constraints.max_deceleration);
  EXPECT_EQ(default_constraints.max_acceleration_rate, current_constraints.max_acceleration_rate);
  EXPECT_EQ(default_constraints.max_deceleration_rate, current_constraints.max_deceleration_rate);
}

TEST(EntityBase, setDynamicConstraints)
{
  const std::string name("test");
  auto entity_status = traffic_simulator::EntityStatus();
  entity_status.name = name;
  entity_status.lanelet_pose_valid = false;

  auto canonicalized_entity_status =
    traffic_simulator::entity_status::CanonicalizedEntityStatus(entity_status, nullptr);

  auto dummy = DummyEntity(name, canonicalized_entity_status, nullptr);
  traffic_simulator_msgs::msg::DynamicConstraints default_constraints{};

  default_constraints.max_speed = 5.0;
  default_constraints.max_acceleration = 7.0;
  default_constraints.max_deceleration = 11.0;
  default_constraints.max_acceleration_rate = 13.0;
  default_constraints.max_deceleration_rate = 17.0;
  dummy.setDynamicConstraints(default_constraints);
  auto current_constraints = dummy.getDynamicConstraints();

  EXPECT_EQ(default_constraints.max_speed, current_constraints.max_speed);
  EXPECT_EQ(default_constraints.max_acceleration, current_constraints.max_acceleration);
  EXPECT_EQ(default_constraints.max_deceleration, current_constraints.max_deceleration);
  EXPECT_EQ(default_constraints.max_acceleration_rate, current_constraints.max_acceleration_rate);
  EXPECT_EQ(default_constraints.max_deceleration_rate, current_constraints.max_deceleration_rate);
}

TEST(EntityBase, requestFollowTrajectory)
{
  const std::string name("test");
  auto entity_status = traffic_simulator::EntityStatus();
  entity_status.name = name;
  entity_status.lanelet_pose_valid = false;

  auto canonicalized_entity_status =
    traffic_simulator::entity_status::CanonicalizedEntityStatus(entity_status, nullptr);
  auto dummy = DummyEntity(name, canonicalized_entity_status, nullptr);

  std::shared_ptr<traffic_simulator_msgs::msg::PolylineTrajectory> ptr = nullptr;
  EXPECT_THROW(dummy.requestFollowTrajectory(ptr), std::runtime_error);
}

TEST(EntityBase, requestWalkStraight)
{
  const std::string name("test");
  auto entity_status = traffic_simulator::EntityStatus();
  entity_status.name = name;
  entity_status.lanelet_pose_valid = false;

  auto canonicalized_entity_status =
    traffic_simulator::entity_status::CanonicalizedEntityStatus(entity_status, nullptr);
  auto dummy = DummyEntity(name, canonicalized_entity_status, nullptr);

  EXPECT_THROW(dummy.requestWalkStraight(), std::runtime_error);
}

TEST(EntityBase, updateStandStillDuration_startedMoving)
{
  const std::string name("test");
  auto entity_status = traffic_simulator::EntityStatus();
  entity_status.name = name;
  entity_status.lanelet_pose_valid = false;

  auto canonicalized_entity_status =
    traffic_simulator::entity_status::CanonicalizedEntityStatus(entity_status, nullptr);
  auto dummy = DummyEntity(name, canonicalized_entity_status, nullptr);
  dummy.startNpcLogic();
  dummy.setLinearVelocity(3.0);

  EXPECT_EQ(0.0, dummy.updateStandStillDuration(0.1));
}

TEST(EntityBase, updateStandStillDuration_notStarted)
{
  const std::string name("test");
  auto entity_status = traffic_simulator::EntityStatus();
  entity_status.name = name;
  entity_status.lanelet_pose_valid = false;

  auto canonicalized_entity_status =
    traffic_simulator::entity_status::CanonicalizedEntityStatus(entity_status, nullptr);
  auto dummy = DummyEntity(name, canonicalized_entity_status, nullptr);

  dummy.setLinearVelocity(3.0);
  EXPECT_EQ(0.0, dummy.updateStandStillDuration(0.1));

  dummy.setLinearVelocity(0.0);
  EXPECT_EQ(0.0, dummy.updateStandStillDuration(0.1));
}

TEST(EntityBase, updateTraveledDistance_startedMoving)
{
  const std::string name("test");
  auto entity_status = traffic_simulator::EntityStatus();
  entity_status.name = name;
  entity_status.lanelet_pose_valid = false;

  auto canonicalized_entity_status =
    traffic_simulator::entity_status::CanonicalizedEntityStatus(entity_status, nullptr);
  auto dummy = DummyEntity(name, canonicalized_entity_status, nullptr);

  double velocity = 3.0;
  double step_time = 0.1;
  dummy.startNpcLogic();
  dummy.setLinearVelocity(velocity);

  EXPECT_EQ(step_time * velocity, dummy.updateTraveledDistance(step_time));
}

TEST(EntityBase, updateTraveledDistance_notStarted)
{
  const std::string name("test");
  auto entity_status = traffic_simulator::EntityStatus();
  entity_status.name = name;
  entity_status.lanelet_pose_valid = false;

  auto canonicalized_entity_status =
    traffic_simulator::entity_status::CanonicalizedEntityStatus(entity_status, nullptr);
  auto dummy = DummyEntity(name, canonicalized_entity_status, nullptr);

  double velocity = 3.0;
  double step_time = 0.1;
  dummy.setLinearVelocity(velocity);
  EXPECT_EQ(0.0, dummy.updateTraveledDistance(step_time));

  dummy.setLinearVelocity(0.0);
  EXPECT_EQ(0.0, dummy.updateTraveledDistance(step_time));
}

TEST(EntityBase, stopAtCurrentPosition)
{
  const std::string name("test");
  auto entity_status = traffic_simulator::EntityStatus();
  entity_status.name = name;
  entity_status.lanelet_pose_valid = false;

  auto canonicalized_entity_status =
    traffic_simulator::entity_status::CanonicalizedEntityStatus(entity_status, nullptr);
  auto dummy = DummyEntity(name, canonicalized_entity_status, nullptr);

  double velocity = 3.0;
  dummy.setLinearVelocity(velocity);
  auto curr_twist = dummy.getCurrentTwist();
  EXPECT_TRUE(curr_twist.linear.x == velocity);

  dummy.stopAtCurrentPosition();
  curr_twist = dummy.getCurrentTwist();
  EXPECT_TRUE(curr_twist.linear.x == 0.0);
}
