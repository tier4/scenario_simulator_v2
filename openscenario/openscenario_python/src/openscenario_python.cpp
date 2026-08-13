// Copyright 2025 TIER IV, Inc. All rights reserved.
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

// openscenario_python (SSV2_HEADLESS_EGO only): a thin pybind11 facade that lifts the headless
// OpenSCENARIO Interpreter driving model (proven by headless_interpreter_drive_check.cpp) to the
// Python boundary, so the Diffusion-Planner training loop can drive one tick at a time in-process:
//
//   runner = openscenario_python.HeadlessRunner(osc_path)
//   runner.configure(); runner.activate()
//   while runner.step() == "running":
//       runner.set_ego_trajectory(planned_points)      # DP output, map frame [N, >=3]
//       states = runner.get_entity_states()             # NPC truth for scoring
//   runner.deactivate()
//
// The runner drives the sim through the exported openscenario_interpreter::headless bridge (inject
// + read); it must NOT touch SimulatorCore's header-inline statics directly, because this module is
// a separate .so and would bind to its own null `core` (see headless_bridge.hpp).
// The DP-output marshalling / replan cadence / SceneContext construction live in Python (Phase 3);
// this module only exposes the in-process seam.

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <cmath>
#include <cstdint>
#include <memory>
#include <openscenario_interpreter/headless_bridge.hpp>
#include <openscenario_interpreter/openscenario_interpreter.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace py = pybind11;

namespace openscenario_python
{
using openscenario_interpreter::Interpreter;
namespace bridge = openscenario_interpreter::headless;

namespace
{
auto poseToDict(const geometry_msgs::msg::Pose & pose) -> py::dict
{
  py::dict d;
  d["x"] = pose.position.x;
  d["y"] = pose.position.y;
  d["z"] = pose.position.z;
  d["qx"] = pose.orientation.x;
  d["qy"] = pose.orientation.y;
  d["qz"] = pose.orientation.z;
  d["qw"] = pose.orientation.w;
  // Convenience yaw (map-frame heading) so Python callers do not each re-derive it.
  const auto & q = pose.orientation;
  d["yaw"] = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  return d;
}

auto twistToDict(const geometry_msgs::msg::Twist & twist) -> py::dict
{
  py::dict d;
  d["linear_x"] = twist.linear.x;
  d["linear_y"] = twist.linear.y;
  d["linear_z"] = twist.linear.z;
  d["angular_x"] = twist.angular.x;
  d["angular_y"] = twist.angular.y;
  d["angular_z"] = twist.angular.z;
  return d;
}

auto accelToDict(const geometry_msgs::msg::Accel & accel) -> py::dict
{
  py::dict d;
  d["linear_x"] = accel.linear.x;
  d["linear_y"] = accel.linear.y;
  d["linear_z"] = accel.linear.z;
  d["angular_x"] = accel.angular.x;
  d["angular_y"] = accel.angular.y;
  d["angular_z"] = accel.angular.z;
  return d;
}

auto boundingBoxToDict(const traffic_simulator_msgs::msg::BoundingBox & bbox) -> py::dict
{
  py::dict center;
  center["x"] = bbox.center.x;
  center["y"] = bbox.center.y;
  center["z"] = bbox.center.z;
  py::dict dims;
  dims["x"] = bbox.dimensions.x;
  dims["y"] = bbox.dimensions.y;
  dims["z"] = bbox.dimensions.z;
  py::dict d;
  d["center"] = center;
  d["dimensions"] = dims;
  return d;
}
}  // namespace

// Facade: owns the Interpreter and reaches the simulator core through the exported headless bridge
// functions (openscenario_interpreter::headless), NOT the header-inline SimulatorCore statics —
// see headless_bridge.hpp for why crossing the .so boundary requires this.
class HeadlessRunner
{
  std::shared_ptr<Interpreter> interpreter_;
  // Only tear down rclcpp if THIS runner started it. In Phase 3 the module is imported inside the
  // ROS-enabled training process, which owns the context — we must not shut it down underneath it.
  bool owns_rclcpp_ = false;

  // Marshal one in-process-composed EntityState into a Python dict. No cross-boundary composition
  // happens here — the bridge already assembled the truth in a single call.
  static auto entityStateDict(const bridge::EntityState & s) -> py::dict
  {
    py::dict d;
    d["name"] = s.name;
    d["type"] = s.type;  // 0=EGO 1=VEHICLE 2=PEDESTRIAN 3=MISC_OBJECT
    d["action"] = s.action;
    d["turn_indicator"] = s.turn_indicator;  // ego only; "" for other entities
    d["pose"] = poseToDict(s.pose);
    d["twist"] = twistToDict(s.twist);
    d["accel"] = accelToDict(s.accel);
    d["bounding_box"] = boundingBoxToDict(s.bounding_box);
    return d;
  }

public:
  explicit HeadlessRunner(
    const std::string & osc_path, const std::string & output_directory,
    double local_frame_rate, double local_real_time_factor, bool consider_pose_by_road_slope)
  {
    // The Interpreter is an rclcpp lifecycle node; one process drives one scenario because
    // SimulatorCore is a static singleton. rclcpp is initialized once per process (guarded), and
    // we remember whether we were the initializer so teardown only shuts down what we started.
    if (not rclcpp::ok()) {
      rclcpp::init(0, nullptr);
      owns_rclcpp_ = true;
    }
    rclcpp::NodeOptions options;
    options.append_parameter_override("osc_path", osc_path);
    options.append_parameter_override("local_frame_rate", local_frame_rate);
    options.append_parameter_override("local_real_time_factor", local_real_time_factor);
    options.append_parameter_override("output_directory", output_directory);
    options.append_parameter_override("headless", true);
    options.append_parameter_override("consider_pose_by_road_slope", consider_pose_by_road_slope);
    interpreter_ = std::make_shared<Interpreter>(options);
  }

  ~HeadlessRunner() { close(); }

  // Tear down in dependency order WHILE the rclcpp context is still alive: deactivate the
  // lifecycle node (despawns entities and unloads the behavior_tree_plugin ClassLoader via
  // on_deactivate -> SimulatorCore::deactivate), then drop the node, then shut rclcpp down only if
  // we started it. Doing this before rclcpp::shutdown avoids the class_loader unload abort at exit.
  auto close() -> void
  {
    if (interpreter_) {
      try {
        if (
          interpreter_->get_current_state().id() ==
          lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
          interpreter_->deactivate();
        }
      } catch (...) {
        // best-effort teardown
      }
      interpreter_.reset();
    }
    if (owns_rclcpp_ && rclcpp::ok()) {
      rclcpp::shutdown();
      owns_rclcpp_ = false;
    }
  }

  auto configure() -> std::string
  {
    interpreter_->configure();
    return interpreter_->get_current_state().label();
  }

  auto activate() -> std::string
  {
    interpreter_->activate();
    return interpreter_->get_current_state().label();
  }

  auto deactivate() -> std::string
  {
    if (interpreter_->get_current_state().id() ==
        lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      interpreter_->deactivate();
    }
    return interpreter_->get_current_state().label();
  }

  auto state() const -> std::string { return interpreter_->get_current_state().label(); }

  auto step() -> std::string
  {
    Interpreter::StepOutcome outcome;
    {
      py::gil_scoped_release release;  // no executor is spun, but keep the GIL free during the frame
      outcome = interpreter_->step();
    }
    return outcome == Interpreter::StepOutcome::terminated ? "terminated" : "running";
  }

  auto resultKind() const -> std::string { return interpreter_->resultKind(); }

  auto simulationTime() const -> double { return bridge::simulationTime(); }

  // Inject a Diffusion-Planner trajectory. `points` is a map-frame numpy array [N, >=3] of
  // (x, y, yaw[, longitudinal_velocity_mps]). The stamp is taken from the sim clock, not Python.
  auto setEgoTrajectory(py::array_t<double> points, const std::string & ego_ref) -> void
  {
    const auto buffer = points.unchecked<2>();
    const auto rows = static_cast<std::size_t>(buffer.shape(0));
    const auto cols = static_cast<std::size_t>(buffer.shape(1));
    if (cols < 3) {
      throw std::runtime_error("trajectory array must have shape [N, >=3] (x, y, yaw[, v])");
    }

    autoware_planning_msgs::msg::Trajectory trajectory;
    trajectory.points.reserve(rows);
    for (std::size_t i = 0; i < rows; ++i) {
      autoware_planning_msgs::msg::TrajectoryPoint tp;
      tp.pose.position.x = buffer(i, 0);
      tp.pose.position.y = buffer(i, 1);
      tp.pose.position.z = 0.0;  // z is snapped to the lanelet surface by the ego on injection
      const double yaw = buffer(i, 2);
      tp.pose.orientation.x = 0.0;
      tp.pose.orientation.y = 0.0;
      tp.pose.orientation.z = std::sin(yaw * 0.5);
      tp.pose.orientation.w = std::cos(yaw * 0.5);
      tp.longitudinal_velocity_mps = cols >= 4 ? static_cast<float>(buffer(i, 3)) : 0.0f;
      trajectory.points.push_back(tp);
    }

    const double t = bridge::simulationTime();
    const rclcpp::Time stamp(static_cast<int64_t>(t * 1e9));
    bridge::setEgoTrajectory(ego_ref, stamp, trajectory);
  }

  // command: 0=NO_COMMAND 1=DISABLE 2=ENABLE_LEFT 3=ENABLE_RIGHT (autoware_vehicle_msgs).
  auto setEgoTurnIndicator(std::uint8_t command, const std::string & ego_ref) -> void
  {
    autoware_vehicle_msgs::msg::TurnIndicatorsCommand cmd;
    cmd.command = command;
    bridge::setEgoTurnIndicator(ego_ref, cmd);
  }

  auto getEgoState(const std::string & ego_ref) const -> py::dict
  {
    return entityStateDict(bridge::entityState(ego_ref));
  }

  auto getEntityStates() const -> py::dict
  {
    py::dict states;
    for (const auto & s : bridge::entityStates()) {
      states[py::str(s.name)] = entityStateDict(s);
    }
    return states;
  }

  // Composed conventional traffic-light state for a lanelet id (e.g. "green" / "red circle").
  // Bulk enumeration of all lights is deferred to Phase 3 (TL sync).
  auto getTrafficLightState(std::int64_t lanelet_id) const -> std::string
  {
    return bridge::conventionalTrafficLightComposedState(lanelet_id);
  }

  // The map the interpreter resolved and loaded. Valid after configure().
  auto getLanelet2MapPath() const -> std::string { return bridge::lanelet2MapPath(); }
};

PYBIND11_MODULE(openscenario_python, m)
{
  m.doc() = "Headless in-process OpenSCENARIO driving for the Diffusion-Planner closed-loop "
            "validator (SSV2_HEADLESS_EGO).";

  py::class_<HeadlessRunner>(m, "HeadlessRunner")
    .def(
      py::init<const std::string &, const std::string &, double, double, bool>(),
      py::arg("osc_path"), py::arg("output_directory") = "/tmp/openscenario_python",
      py::arg("local_frame_rate") = 10.0, py::arg("local_real_time_factor") = 1.0,
      py::arg("consider_pose_by_road_slope") = false)
    .def("configure", &HeadlessRunner::configure)
    .def("activate", &HeadlessRunner::activate)
    .def("deactivate", &HeadlessRunner::deactivate)
    .def("state", &HeadlessRunner::state)
    .def("step", &HeadlessRunner::step)
    .def("result_kind", &HeadlessRunner::resultKind)
    .def("simulation_time", &HeadlessRunner::simulationTime)
    .def(
      "set_ego_trajectory", &HeadlessRunner::setEgoTrajectory, py::arg("points"),
      py::arg("ego_ref") = "ego")
    .def(
      "set_ego_turn_indicator", &HeadlessRunner::setEgoTurnIndicator, py::arg("command"),
      py::arg("ego_ref") = "ego")
    .def("get_ego_state", &HeadlessRunner::getEgoState, py::arg("ego_ref") = "ego")
    .def("get_entity_states", &HeadlessRunner::getEntityStates)
    .def("get_traffic_light_state", &HeadlessRunner::getTrafficLightState, py::arg("lanelet_id"))
    .def("lanelet2_map_path", &HeadlessRunner::getLanelet2MapPath)
    .def("close", &HeadlessRunner::close)
    .def("__enter__", [](HeadlessRunner & self) -> HeadlessRunner & { return self; })
    .def("__exit__", [](HeadlessRunner & self, const py::object &, const py::object &,
                        const py::object &) { self.close(); });
}
}  // namespace openscenario_python
