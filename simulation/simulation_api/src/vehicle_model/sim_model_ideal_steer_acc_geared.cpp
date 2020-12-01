// Copyright 2020 The Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "simulation_api/vehicle_model/sim_model_ideal_steer_acc_geared.hpp"
#include <algorithm>


SimModelIdealSteerAccGeard::SimModelIdealSteerAccGeard(double wheelbase)
: SimModelInterface(4 /* dim x */, 3 /* dim u */), wheelbase_(wheelbase), current_acc_(0.0) {}

double SimModelIdealSteerAccGeard::getX() {return state_(IDX::X);}
double SimModelIdealSteerAccGeard::getY() {return state_(IDX::Y);}
double SimModelIdealSteerAccGeard::getYaw() {return state_(IDX::YAW);}
double SimModelIdealSteerAccGeard::getVx() {return state_(IDX::VX);}
double SimModelIdealSteerAccGeard::getVy() {return 0.0;}
double SimModelIdealSteerAccGeard::getAx() {return current_acc_;}
double SimModelIdealSteerAccGeard::getWz()
{
  return state_(IDX::VX) * std::tan(input_(IDX_U::STEER_DES)) / wheelbase_;
}
double SimModelIdealSteerAccGeard::getSteer() {return input_(IDX_U::STEER_DES);}
void SimModelIdealSteerAccGeard::update(const double & dt)
{
  const auto prev_vx = state_(IDX::VX);

  updateRungeKutta(dt, input_);

  if (gear_ == SimModelInterface::GEAR::DRIVE) {
    if (state_(IDX::VX) < 0.0) {state_(IDX::VX) = 0.0;}
  } else if (gear_ == SimModelInterface::GEAR::REVERSE) {
    if (state_(IDX::VX) > 0.0) {state_(IDX::VX) = 0.0;}
  } else if (gear_ == SimModelInterface::GEAR::PARKING) {
    state_(IDX::VX) = 0.0;
  }

  current_acc_ = (state_(IDX::VX) - prev_vx) / std::max(dt, 1.0e-5);
}

Eigen::VectorXd SimModelIdealSteerAccGeard::calcModel(
  const Eigen::VectorXd & state, const Eigen::VectorXd & input)
{
  const double vx = state(IDX::VX);
  const double yaw = state(IDX::YAW);
  const double ax = input(IDX_U::AX_DES);
  const double steer = input(IDX_U::STEER_DES);

  Eigen::VectorXd d_state = Eigen::VectorXd::Zero(dim_x_);
  d_state(IDX::X) = vx * std::cos(yaw);
  d_state(IDX::Y) = vx * std::sin(yaw);
  d_state(IDX::VX) = ax;
  d_state(IDX::YAW) = vx * std::tan(steer) / wheelbase_;

  return d_state;
}
