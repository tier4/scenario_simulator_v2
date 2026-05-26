#!/usr/bin/env python3
"""Minimal CARLA <-> Autoware bridge for the Odaiba use case.

Spawns the ego at lanelet 1960 on startup, initializes Autoware's
localization, publishes /vehicle/status/control_mode (= MANUAL) for the
engage flow, and teleports the ego on /initialpose. CARLA must be
launched separately with `--ros2`; its in-engine ROS 2 bridge takes care
of /tf, /localization/kinematic_state, /vehicle/status/* and IMU once a
`sensor.other.vehicle_status` and `sensor.other.imu` are attached to the
ego.
"""

import math
import threading
import time
from contextlib import contextmanager

import carla
import rclpy
from rclpy.node import Node

from autoware_adapi_v1_msgs.srv import InitializeLocalization
from autoware_vehicle_msgs.msg import ControlModeReport
from autoware_vehicle_msgs.srv import ControlModeCommand
from geometry_msgs.msg import Point, PoseWithCovarianceStamped, Quaternion


# --- Tier4 Odaiba localization anchor (matches CARLA build constants) ------
REFERENCE_MAP_X = 89626.180
REFERENCE_MAP_Y = 42257.898
REFERENCE_MAP_Z = 6.4475
REFERENCE_MAP_YAW_RAD = math.radians(121.71)
REFERENCE_CARLA_BASE_X = -2382.261356801974
REFERENCE_CARLA_BASE_Y = 3077.154881891110
REFERENCE_CARLA_BASE_Z = 10.9
REFERENCE_CARLA_BASE_YAW_RAD = math.radians(-121.564627)
MAP_TO_CARLA_SCALE = 1.0001113293488773
MAP_TO_CARLA_XY_YAW_RAD = 0.00020967531865156985
MAP_TO_CARLA_YAW_RAD = 0.0011632706731004028

EGO_BLUEPRINT = "vehicle.byd.j6gen2"
J6_PIVOT_TO_BASE_LINK_X = -2.23353124

# lanelet 1960 bootstrap pose (autoware map frame).
DEFAULT_POSE_MAP_X = 89405.36
DEFAULT_POSE_MAP_Y = 43256.08
DEFAULT_POSE_MAP_YAW_RAD = math.radians(-2.93)

# Attributes the vehicle_status sensor needs so its in-engine publishers
# emit Autoware-localization ground-truth in the map frame.
VEHICLE_STATUS_ATTRS = (
    ("publish_autoware_localization_ground_truth", "true"),
    ("reference_map_x", str(REFERENCE_MAP_X)),
    ("reference_map_y", str(REFERENCE_MAP_Y)),
    ("reference_map_z", str(REFERENCE_MAP_Z)),
    ("reference_map_yaw_rad", str(REFERENCE_MAP_YAW_RAD)),
    ("reference_carla_base_x", str(REFERENCE_CARLA_BASE_X)),
    ("reference_carla_base_y", str(REFERENCE_CARLA_BASE_Y)),
    ("reference_carla_base_z", str(REFERENCE_CARLA_BASE_Z)),
    ("reference_carla_base_yaw_rad", str(REFERENCE_CARLA_BASE_YAW_RAD)),
    ("map_to_carla_scale", str(MAP_TO_CARLA_SCALE)),
    ("map_to_carla_xy_yaw_rad", str(MAP_TO_CARLA_XY_YAW_RAD)),
    ("map_to_carla_yaw_rad", str(MAP_TO_CARLA_YAW_RAD)),
)

TICK_FAIL_LOG_PERIOD_S = 5.0  # throttle world.tick() failure spam
SHUTDOWN_LOCK_TIMEOUT_S = 2.0  # never block shutdown waiting for a stuck tick


# --- Coordinate helpers ----------------------------------------------------
def normalize_angle(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def rotate_xy(x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return x * c - y * s, x * s + y * c


def map_pose_to_carla_spawn(map_x, map_y, map_yaw_rad):
    """Autoware map pose -> CARLA spawn transform (matches what the
    in-engine ground-truth publisher will reconstruct)."""
    d_x, d_y = rotate_xy(
        MAP_TO_CARLA_SCALE * (map_x - REFERENCE_MAP_X),
        MAP_TO_CARLA_SCALE * -(map_y - REFERENCE_MAP_Y),
        MAP_TO_CARLA_XY_YAW_RAD,
    )
    carla_yaw = normalize_angle(MAP_TO_CARLA_YAW_RAD - map_yaw_rad)
    pivot_x, pivot_y = rotate_xy(-J6_PIVOT_TO_BASE_LINK_X, 0.0, carla_yaw)
    return carla.Transform(
        carla.Location(
            REFERENCE_CARLA_BASE_X + d_x + pivot_x,
            REFERENCE_CARLA_BASE_Y + d_y + pivot_y,
            REFERENCE_CARLA_BASE_Z,
        ),
        carla.Rotation(yaw=math.degrees(carla_yaw)),
    )


# --- Bridge node -----------------------------------------------------------
class CarlaBridge(Node):
    def __init__(self):
        super().__init__("carla_bridge")
        self.host = self.declare_parameter("host", "127.0.0.1").value
        self.port = self.declare_parameter("port", 2000).value
        self.hz_rate = self.declare_parameter("hz_rate", 30).value

        self.client = carla.Client(self.host, self.port)
        self.client.set_timeout(60.0)
        self.world = self._get_world()
        self._configure_sync_mode()

        self.ego = None
        self._vehicle_status_sensor = None
        self._imu_sensor = None
        self._mode = ControlModeReport.MANUAL
        self._world_lock = threading.Lock()
        self._shutdown = threading.Event()
        self._pending_init = None  # (map_x, map_y, map_yaw) waiting for AdAPI
        self._world_thread = None  # set after spawn so shutdown is safe
        self._warn_throttle = {}   # what -> last-log monotonic seconds

        self.control_mode_pub = self.create_publisher(
            ControlModeReport, "/vehicle/status/control_mode", 1
        )
        self.create_timer(0.02, lambda: self.control_mode_pub.publish(  # 50 Hz
            ControlModeReport(stamp=self.get_clock().now().to_msg(), mode=self._mode)
        ))

        self.create_service(
            ControlModeCommand,
            "/control/control_mode_request",
            self._on_control_mode_request,
        )
        self.create_subscription(
            PoseWithCovarianceStamped, "/initialpose", self._on_initialpose, 1
        )
        self.init_loc_client = self.create_client(
            InitializeLocalization, "/api/localization/initialize"
        )
        self.create_timer(2.0, self._try_localization_init)

        self._spawn_ego_at_map_pose(
            DEFAULT_POSE_MAP_X,
            DEFAULT_POSE_MAP_Y,
            DEFAULT_POSE_MAP_YAW_RAD,
            label="bootstrap",
        )

        # Tick CARLA off the rclpy executor so the UE5 main loop is not
        # gated by ROS callback latency (sync mode freezes otherwise).
        self._world_thread = threading.Thread(
            target=self._world_loop, daemon=True
        )
        self._world_thread.start()

    # --- Logging helper ----------------------------------------------------
    @contextmanager
    def _warn_on_error(self, what, throttle_s=0, exc_type=Exception):
        """Run a block; on `exc_type` log "<what> failed: <exc>" and swallow.
        throttle_s>0 throttles repeated logs of the same `what`."""
        try:
            yield
        except exc_type as exc:
            if throttle_s > 0:
                now = time.monotonic()
                if now - self._warn_throttle.get(what, 0.0) < throttle_s:
                    return
                self._warn_throttle[what] = now
            self.get_logger().warn(f"{what} failed: {exc!r}")

    # --- CARLA setup -------------------------------------------------------
    def _get_world(self):
        # OpenDRIVE-less maps make the first get_world() flaky.
        for attempt in range(3):
            try:
                return self.client.get_world()
            except RuntimeError as exc:
                self.get_logger().warn(
                    f"client.get_world() attempt {attempt}: {exc!r}"
                )
                time.sleep(0.5)
        raise RuntimeError("client.get_world() failed 3 times")

    def _configure_sync_mode(self):
        # NOTE: CARLA's in-engine ROS 2 bridge is enabled by the `--ros2`
        # CLI flag, not by a WorldSettings attribute.
        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 1.0 / float(self.hz_rate)
        self.world.apply_settings(settings)
        self.world.set_publish_tf(False)
        self.get_logger().info(
            f"World sync mode at {self.hz_rate} Hz "
            f"(dt={settings.fixed_delta_seconds:.3f}s)"
        )

    # --- Ego spawn / teleport ---------------------------------------------
    def _destroy_attached_actors(self):
        for actor in (self._imu_sensor, self._vehicle_status_sensor, self.ego):
            if actor is None:
                continue
            with self._warn_on_error("actor.destroy()", exc_type=RuntimeError):
                actor.destroy()
        self._imu_sensor = None
        self._vehicle_status_sensor = None
        self.ego = None

    def _spawn_ego_at_map_pose(self, map_x, map_y, map_yaw_rad, label):
        spawn_tf = map_pose_to_carla_spawn(map_x, map_y, map_yaw_rad)
        bp_library = self.world.get_blueprint_library()

        ego_bp = bp_library.find(EGO_BLUEPRINT)
        # role_name="ego" lets CARLA's built-in ROS 2 bridge route
        # vehicle_status publishers to this actor.
        ego_bp.set_attribute("role_name", "ego")

        vs_bp = bp_library.find("sensor.other.vehicle_status")
        for name, value in VEHICLE_STATUS_ATTRS:
            vs_bp.set_attribute(name, value)

        # Tie IMU rate to the world step so the sensor doesn't lag the sim.
        imu_bp = bp_library.find("sensor.other.imu")
        imu_bp.set_attribute("sensor_tick", f"{1.0 / float(self.hz_rate)}")
        imu_bp.set_attribute("ros_name", "tamagawa/imu_link")
        imu_bp.set_attribute("ros_topic_name", "/sensing/imu/imu_data")

        with self._world_lock:
            self._destroy_attached_actors()

            self.ego = self.world.try_spawn_actor(ego_bp, spawn_tf)
            if self.ego is None:
                self.get_logger().error(
                    f"{label} ego spawn failed at {spawn_tf.location}"
                )
                return False

            self._vehicle_status_sensor = self.world.try_spawn_actor(
                vs_bp, carla.Transform(), attach_to=self.ego
            )
            if self._vehicle_status_sensor is None:
                self.get_logger().error(
                    f"{label} vehicle_status sensor spawn failed"
                )

            self._imu_sensor = self.world.try_spawn_actor(
                imu_bp, carla.Transform(), attach_to=self.ego
            )
            if self._imu_sensor is None:
                self.get_logger().error(f"{label} IMU sensor spawn failed")
            else:
                self._imu_sensor.enable_for_ros()

            self.world.tick()

        self.get_logger().info(
            f"{label} spawn OK: map=({map_x:.2f}, {map_y:.2f}, "
            f"{math.degrees(map_yaw_rad):.2f} deg) -> CARLA=("
            f"{spawn_tf.location.x:.2f}, {spawn_tf.location.y:.2f})"
        )
        self._pending_init = (map_x, map_y, map_yaw_rad)
        return True

    def _on_initialpose(self, msg):
        if msg.header.frame_id not in ("", "map"):
            self.get_logger().warn(
                f"/initialpose must be in map frame; got {msg.header.frame_id!r}"
            )
            return
        ok = self._spawn_ego_at_map_pose(
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            yaw_from_quaternion(msg.pose.pose.orientation),
            label="/initialpose",
        )
        if not ok:
            self.get_logger().warn(
                "/initialpose respawn failed; ego unchanged. "
                "Try another location with no obstacle."
            )

    def _try_localization_init(self):
        if self._pending_init is None or not self.init_loc_client.service_is_ready():
            return
        map_x, map_y, map_yaw_rad = self._pending_init

        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.pose.position = Point(x=map_x, y=map_y, z=REFERENCE_MAP_Z)
        pose.pose.pose.orientation = Quaternion(
            z=math.sin(map_yaw_rad * 0.5), w=math.cos(map_yaw_rad * 0.5)
        )
        req = InitializeLocalization.Request()
        req.pose.append(pose)

        def done(future):
            try:
                future.result()
                self.get_logger().info("Autoware localization initialized.")
            except Exception as exc:
                self.get_logger().warn(
                    f"InitializeLocalization call failed: {exc!r}; "
                    "will retry on the next spawn"
                )

        self._pending_init = None
        self.init_loc_client.call_async(req).add_done_callback(done)

    # --- World tick + spectator (background thread) -----------------------
    def _world_loop(self):
        period = 1.0 / float(self.hz_rate)
        next_tick = time.monotonic()
        while not self._shutdown.is_set():
            next_tick += period
            with self._warn_on_error("world.tick()", throttle_s=TICK_FAIL_LOG_PERIOD_S):
                with self._world_lock:
                    self.world.tick()
            with self._warn_on_error("move_spectator"):
                self._move_spectator()
            sleep_dt = next_tick - time.monotonic()
            if sleep_dt > 0:
                if self._shutdown.wait(sleep_dt):
                    return
            else:
                next_tick = time.monotonic()

    def _move_spectator(self):
        # Snapshot the ego transform under the lock so it can't be destroyed
        # between the None-check and the get_transform() RPC.
        with self._world_lock:
            ego = self.ego
            if ego is None:
                return
            ego_tf = ego.get_transform()
        yaw_rad = math.radians(ego_tf.rotation.yaw)
        loc = ego_tf.location
        self.world.get_spectator().set_transform(carla.Transform(
            carla.Location(
                loc.x - 15.0 * math.cos(yaw_rad),
                loc.y - 15.0 * math.sin(yaw_rad),
                loc.z + 7.0,
            ),
            carla.Rotation(pitch=-15.0, yaw=ego_tf.rotation.yaw),
        ))

    # --- ControlMode publish / service ------------------------------------
    def _on_control_mode_request(self, request, response):
        self._mode = int(request.mode)
        response.success = True
        return response

    # --- Shutdown ---------------------------------------------------------
    def shutdown(self):
        self._shutdown.set()
        if self._world_thread is not None and self._world_thread.is_alive():
            self._world_thread.join(timeout=2.0)
        # Best-effort cleanup; never block on a stuck world tick. If we
        # can't get the lock, skip touching the world and just let the
        # process exit so a kill -9 isn't needed.
        acquired = self._world_lock.acquire(timeout=SHUTDOWN_LOCK_TIMEOUT_S)
        try:
            self._destroy_attached_actors()
            world = getattr(self, "world", None)
            if world is not None:
                with self._warn_on_error("restore async mode"):
                    settings = world.get_settings()
                    settings.synchronous_mode = False
                    world.apply_settings(settings)
        finally:
            if acquired:
                self._world_lock.release()


def main():
    rclpy.init()
    node = None
    try:
        node = CarlaBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
