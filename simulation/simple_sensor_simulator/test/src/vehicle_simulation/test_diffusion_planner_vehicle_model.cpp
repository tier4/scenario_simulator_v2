// Copyright 2026 TIER IV, Inc.
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

#include <utility>
#include <vector>

struct VmModel;

extern "C" {
VmModel * vm_create_delay_steer_acc_geared_for_diffusion_planner(
  double, double, double, double, double, double, double, double, double, double, double, double,
  double, double, double);
VmModel * vm_create_delay_steer_acc_geared_wo_fall_guard(
  double, double, double, double, double, double, double, double, double, double, double, double,
  double, double, double);
void vm_destroy(VmModel *);
void vm_reset_state(VmModel *, double, double, double, double, double, double, double, double);
void vm_set_input(VmModel *, double, double);
void vm_step(VmModel *);
double vm_get_x(VmModel *);
double vm_get_y(VmModel *);
double vm_get_yaw(VmModel *);
double vm_get_vx(VmModel *);
double vm_get_vy(VmModel *);
double vm_get_steer(VmModel *);
double vm_get_ax(VmModel *);
double vm_get_wz(VmModel *);
}

namespace
{

class ModelHandle
{
public:
  explicit ModelHandle(VmModel * model) : model_(model) {}
  ~ModelHandle() { vm_destroy(model_); }
  ModelHandle(const ModelHandle &) = delete;
  ModelHandle & operator=(const ModelHandle &) = delete;
  VmModel * get() const { return model_; }

private:
  VmModel * model_;
};

}  // namespace

TEST(DiffusionPlannerVehicleModel, LegacyFactoryAbiAndGoldenOutput)
{
  using LegacyFactory =
    VmModel * (*)(double, double, double, double, double, double, double, double, double, double, double, double, double, double, double);
  LegacyFactory factory = &vm_create_delay_steer_acc_geared_for_diffusion_planner;
  ModelHandle model(
    factory(50.0, 1.0, 7.0, 5.0, 4.76012, 0.05, 0.0, 0.2, 0.0, 0.3, 0.001, 0.002, 1.1, 0.9, 0.01));
  vm_reset_state(model.get(), 1.0, -2.0, 0.3, 5.0, 0.04, 0.1, 0.0, 0.0);

  const std::vector<std::pair<double, double>> commands = {
    {0.8, 0.1},    {0.8, 0.1}, {0.8, 0.1}, {0.8, 0.1}, {-0.4, -0.05}, {-0.4, -0.05},
    {-0.4, -0.05}, {0.2, 0.0}, {0.2, 0.0}, {0.2, 0.0}, {0.2, 0.0},    {0.2, 0.0}};
  for (const auto & [accel, steer] : commands) {
    vm_set_input(model.get(), accel, steer);
    vm_step(model.get());
  }

  EXPECT_NEAR(vm_get_x(model.get()), 3.9030562666740551, 1.0e-12);
  EXPECT_NEAR(vm_get_y(model.get()), -1.0612881230318352, 1.0e-12);
  EXPECT_NEAR(vm_get_yaw(model.get()), 0.32183446062469057, 1.0e-12);
  EXPECT_NEAR(vm_get_vx(model.get()), 5.1508359200954432, 1.0e-12);
  EXPECT_NEAR(vm_get_ax(model.get()), 0.15442719936370253, 1.0e-12);
  EXPECT_NEAR(vm_get_steer(model.get()), 0.0081270691003071382, 1.0e-12);
  EXPECT_NEAR(vm_get_wz(model.get()), 0.010380135252851159, 1.0e-12);
  EXPECT_DOUBLE_EQ(vm_get_vy(model.get()), 0.0);
}

TEST(DiffusionPlannerVehicleModel, LegacyZeroDelayStillDegeneratesToWoFallGuard)
{
  const auto make_args = [](auto factory) {
    return factory(
      50.0, 1.0, 7.0, 5.0, 4.76012, 0.01, 0.0, 0.2, 0.0, 0.3, 0.001, 0.002, 1.1, 0.9, 0.01);
  };
  ModelHandle legacy(make_args(vm_create_delay_steer_acc_geared_for_diffusion_planner));
  ModelHandle reference(make_args(vm_create_delay_steer_acc_geared_wo_fall_guard));
  vm_reset_state(legacy.get(), 1.0, -2.0, 0.3, 5.0, 0.04, 0.1, 0.0, 0.0);
  vm_reset_state(reference.get(), 1.0, -2.0, 0.3, 5.0, 0.04, 0.1, 0.0, 0.0);

  for (int index = 0; index < 100; ++index) {
    const double accel = index < 40 ? 0.8 : -0.3;
    const double steer = index < 50 ? 0.1 : -0.04;
    vm_set_input(legacy.get(), accel, steer);
    vm_set_input(reference.get(), accel, steer);
    vm_step(legacy.get());
    vm_step(reference.get());
    EXPECT_DOUBLE_EQ(vm_get_x(legacy.get()), vm_get_x(reference.get()));
    EXPECT_DOUBLE_EQ(vm_get_y(legacy.get()), vm_get_y(reference.get()));
    EXPECT_DOUBLE_EQ(vm_get_yaw(legacy.get()), vm_get_yaw(reference.get()));
    EXPECT_DOUBLE_EQ(vm_get_vx(legacy.get()), vm_get_vx(reference.get()));
    EXPECT_DOUBLE_EQ(vm_get_ax(legacy.get()), vm_get_ax(reference.get()));
    EXPECT_DOUBLE_EQ(vm_get_steer(legacy.get()), vm_get_steer(reference.get()));
    EXPECT_DOUBLE_EQ(vm_get_wz(legacy.get()), vm_get_wz(reference.get()));
  }
}
