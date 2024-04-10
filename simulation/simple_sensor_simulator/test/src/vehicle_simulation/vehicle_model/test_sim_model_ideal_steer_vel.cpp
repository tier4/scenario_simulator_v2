#include <gtest/gtest.h>

#include <simple_sensor_simulator/vehicle_simulation/vehicle_model/sim_model_ideal_steer_vel.hpp>

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

/*
0. dt: division by 0
1. wheelbase: division by 0
2. unintuitive choice of storing the parameters
3. unsure of how calcModel should work. initial position does not influence the result.
*/

TEST(SimModelIdealSteerVel, update_nonZeroDT)
{
  const double wheelbase = 1.0;
  auto model = SimModelIdealSteerVel(wheelbase);
  SimModelInterface * p_model = &model;
  const double dt = 0.01;
  Eigen::VectorXd x_state(p_model->getDimX());
  Eigen::VectorXd u_input(p_model->getDimU());
  const double start_vx = 3.0;
  const double start_x = 5.0;
  x_state << start_x, 0.0, 0.0;
  u_input << start_vx, 0.0;
  p_model->setInput(u_input);
  p_model->setState(x_state);

  for (int i = 0; i < 10; i++) {
    EXPECT_TRUE(std::abs(start_x + i * dt * start_vx - p_model->getX()) < 1e-5);
    EXPECT_TRUE(0.0 == p_model->getY());
    EXPECT_TRUE(0.0 == p_model->getYaw());
    EXPECT_TRUE(start_vx == p_model->getVx());
    p_model->update(dt);
  }
  EXPECT_TRUE(p_model->getAx() < std::numeric_limits<double>::infinity());
}

TEST(SimModelIdealSteerVel, update_zeroDT)
{
  const double wheelbase = 1.0;
  auto model = SimModelIdealSteerVel(wheelbase);
  SimModelInterface * p_model = &model;
  const double dt = 0.0;
  Eigen::VectorXd x_state(p_model->getDimX());
  Eigen::VectorXd u_input(p_model->getDimU());
  const double start_vx = 3.0;
  const double start_x = 5.0;
  x_state << start_x, 0.0, 0.0;
  u_input << start_vx, 0.0;
  p_model->setInput(u_input);
  p_model->setState(x_state);

  for (int i = 0; i < 10; i++) {
    EXPECT_TRUE(std::abs(start_x + i * dt * start_vx - p_model->getX()) < 1e-5);
    EXPECT_TRUE(0.0 == p_model->getY());
    EXPECT_TRUE(0.0 == p_model->getYaw());
    EXPECT_TRUE(start_vx == p_model->getVx());
    p_model->update(dt);
  }
  EXPECT_TRUE(p_model->getAx() < std::numeric_limits<double>::infinity());
}

TEST(SimModelIdealSteerVel, calcModel_zeroWheelbase)
{
  const double wheelbase = 0.0;
  auto model = SimModelIdealSteerVel(wheelbase);
  SimModelInterface * p_model = &model;
  Eigen::VectorXd x_state(p_model->getDimX());
  Eigen::VectorXd u_input(p_model->getDimU());
  const double start_x = 3.0;
  const double start_y = 5.0;
  const double start_yaw = 7.0;
  const double start_vx = 11.0;
  const double start_steer = 13.0;
  x_state << start_x, start_y, start_yaw;
  u_input << start_vx, start_steer;

  auto out_x_state = p_model->calcModel(x_state, u_input);
  p_model->setState(out_x_state);

  EXPECT_TRUE(p_model->getYaw() < std::numeric_limits<double>::infinity());
}