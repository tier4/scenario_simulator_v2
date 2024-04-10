#include <gtest/gtest.h>

#include <simple_sensor_simulator/vehicle_simulation/vehicle_model/sim_model_ideal_steer_acc.hpp>

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

TEST(SimModelIdealSteerAcc, update_nonZeroDT)
{
  const double wheelbase = 1.0;
  auto model = SimModelIdealSteerAcc(wheelbase);
  SimModelInterface * p_model = &model;
  Eigen::VectorXd x_state(p_model->getDimX());
  Eigen::VectorXd u_input(p_model->getDimU());
  const double start_x = 3.0;
  const double start_y = 5.0;
  const double start_yaw = 0.0;
  const double start_vx = 11.0;
  const double start_ax_des = 13.0;
  const double start_steer_des = 0.0;
  x_state << start_x, start_y, start_yaw, start_vx;
  u_input << start_ax_des, start_steer_des;

  p_model->setInput(u_input);
  p_model->setState(x_state);

  const double dt = 1.0;
  for (int i = 0; i < 10; i++) {
    double current_vx = start_vx + i * dt * start_ax_des;
    double current_x = start_x + i * dt * start_vx + 0.5 * i * i * dt * start_ax_des;
    EXPECT_TRUE(std::abs(current_x - p_model->getX()) < 1e-5);
    EXPECT_TRUE(std::abs(current_vx - p_model->getVx()) < 1e-5);
    EXPECT_TRUE(start_y == p_model->getY());
    EXPECT_TRUE(start_yaw == p_model->getYaw());
    p_model->update(dt);
  }
}

TEST(SimModelIdealSteerAcc, update_zeroDT)
{
  const double wheelbase = 1.0;
  auto model = SimModelIdealSteerAcc(wheelbase);
  SimModelInterface * p_model = &model;
  Eigen::VectorXd x_state(p_model->getDimX());
  Eigen::VectorXd u_input(p_model->getDimU());
  const double start_x = 3.0;
  const double start_y = 5.0;
  const double start_yaw = 7.0;
  const double start_vx = 11.0;
  const double start_ax_des = 13.0;
  const double start_steer_des = 17.0;
  x_state << start_x, start_y, start_yaw, start_vx;
  u_input << start_ax_des, start_steer_des;

  p_model->setInput(u_input);
  p_model->setState(x_state);

  const double dt = 0.0;
  for (int i = 0; i < 10; i++) {
    EXPECT_TRUE(start_x == p_model->getX());
    EXPECT_TRUE(start_y == p_model->getY());
    EXPECT_TRUE(start_yaw == p_model->getYaw());
    EXPECT_TRUE(start_vx == p_model->getVx());
    p_model->update(dt);
  }
}

TEST(SimModelIdealSteerAcc, calcModel_zeroWheelbase)
{
  const double wheelbase = 0.0;
  auto model = SimModelIdealSteerAcc(wheelbase);
  SimModelInterface * p_model = &model;
  Eigen::VectorXd x_state(p_model->getDimX());
  Eigen::VectorXd u_input(p_model->getDimU());
  const double start_x = 3.0;
  const double start_y = 5.0;
  const double start_yaw = 7.0;
  const double start_vx = 11.0;
  const double start_ax_des = 13.0;
  const double start_steer_des = 17.0;
  x_state << start_x, start_y, start_yaw, start_vx;
  u_input << start_ax_des, start_steer_des;

  auto out_x_state = p_model->calcModel(x_state, u_input);
  p_model->setState(out_x_state);

  EXPECT_TRUE(p_model->getYaw() < std::numeric_limits<double>::infinity());
}