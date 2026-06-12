# Lockstep run profile.
#
# Encapsulates everything specific to running a scenario in the fast lockstep
# configuration (minimal Autoware + PlanTrajectory service + best-effort loop):
# which launch file to use, which parameters enable lockstep, and how worker
# slots map to ZMQ ports. The parallel runner stays agnostic of all of this.

from pathlib import Path


def add_profile_arguments(parser):
    parser.add_argument("--global-timeout", type=float, default=600.0,
                        help="wall seconds allowed for the scenario itself")
    parser.add_argument("--startup-margin", type=float, default=240.0,
                        help="extra wall seconds on top of global_timeout "
                             "(Autoware launch, TensorRT engine load, ...)")
    parser.add_argument("--base-port", type=int, default=6000,
                        help="worker slot i uses ZMQ port = base + i")
    parser.add_argument("--architecture-type", default="awf/universe/20250130")
    parser.add_argument("--sensor-model", default="sample_sensor_kit")
    parser.add_argument("--vehicle-model", default="sample_vehicle_perfect_tracker",
                        help="must end with _perfect_tracker")
    parser.add_argument("--autoware-launch-package",
                        default="parallel_test_runner")
    parser.add_argument("--autoware-launch-file",
                        default="minimal_lockstep_autoware.launch.xml")
    parser.add_argument("--launch-arg", action="append", default=[],
                        help="extra key:=value passed to "
                             "scenario_test_runner.launch.py; repeatable")


def write_interpreter_params(out_dir: Path) -> Path:
    """Parameter file enabling the best-effort (non real-time) interpreter loop."""
    path = out_dir / "lockstep_params.yaml"
    path.write_text("/**:\n  ros__parameters:\n    best_effort_frame_rate: true\n")
    return path


def make_command_builder(args, params_yaml: Path):
    def build(xosc: Path, workdir: Path, slot: int):
        return [
            "ros2", "launch", "scenario_test_runner", "scenario_test_runner.launch.py",
            f"scenario:={xosc}",
            f"output_directory:={workdir}",
            f"port:={args.base_port + slot}",
            f"architecture_type:={args.architecture_type}",
            f"sensor_model:={args.sensor_model}",
            f"vehicle_model:={args.vehicle_model}",
            f"autoware_launch_package:={args.autoware_launch_package}",
            f"autoware_launch_file:={args.autoware_launch_file}",
            "simple_sensor_simulator.diffusion_planner_lockstep:=true",
            f"parameter_file_path:={params_yaml}",
            f"global_timeout:={args.global_timeout}",
            "record:=false",
            "launch_rviz:=false",
        ] + args.launch_arg

    return build
