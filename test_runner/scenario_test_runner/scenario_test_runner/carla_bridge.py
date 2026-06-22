#!/usr/bin/env python3
"""Minimal CARLA <-> Autoware bridge for scenario_simulator_v2 integration.

Waits for CARLA to accept connections (retrying for up to 120 s), then
publishes /carla_bridge/ready (TRANSIENT_LOCAL) so that
EgoEntitySimulation can publish /initialpose3d.  On receipt of
/initialpose3d (also TRANSIENT_LOCAL), spawns the ego vehicle in CARLA
at the requested map-frame pose.

CARLA must be launched separately (or via the launch file) with
``--ros2``; its in-engine ROS 2 bridge takes care of /tf,
/localization/kinematic_state, /vehicle/status/* and IMU once a
``sensor.other.vehicle_status`` and ``sensor.other.imu`` are attached
to the ego.
"""

import math
import threading
import time
from contextlib import contextmanager

import carla
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Empty


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

CONNECT_TIMEOUT_S = 120.0
CONNECT_RETRY_INTERVAL_S = 2.0
TICK_FAIL_LOG_PERIOD_S = 5.0
SHUTDOWN_LOCK_TIMEOUT_S = 2.0

TRANSIENT_LOCAL_QOS = QoSProfile(
    depth=1,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    reliability=QoSReliabilityPolicy.RELIABLE,
)


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

        self.client = None
        self.world = self._get_world()
        self._configure_sync_mode()

        self.ego = None
        self._vehicle_status_sensor = None
        self._imu_sensor = None
        self._world_lock = threading.Lock()
        self._shutdown = threading.Event()
        self._world_thread = None
        self._warn_throttle = {}

        self.create_subscription(
            PoseWithCovarianceStamped,
            "/initialpose3d",
            self._on_initialpose3d,
            TRANSIENT_LOCAL_QOS,
        )

        self._ready_pub = self.create_publisher(
            Empty, "/carla_bridge/ready", TRANSIENT_LOCAL_QOS
        )
        self._ready_pub.publish(Empty())
        self.get_logger().info("Published /carla_bridge/ready.")

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
        deadline = time.monotonic() + CONNECT_TIMEOUT_S
        attempt = 0
        while True:
            attempt += 1
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise RuntimeError(
                    f"client.get_world() failed after {CONNECT_TIMEOUT_S:.0f}s"
                )
            try:
                self.client = carla.Client(self.host, self.port)
                self.client.set_timeout(60.0)
                return self.client.get_world()
            except RuntimeError as exc:
                self.get_logger().warn(
                    f"client.get_world() attempt {attempt} failed: {exc!r} "
                    f"({remaining:.0f}s remaining, retrying in "
                    f"{CONNECT_RETRY_INTERVAL_S:.0f}s)"
                )
                time.sleep(CONNECT_RETRY_INTERVAL_S)

    def _configure_sync_mode(self):
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
        ego_bp.set_attribute("role_name", "ego")

        vs_bp = bp_library.find("sensor.other.vehicle_status")
        for name, value in VEHICLE_STATUS_ATTRS:
            vs_bp.set_attribute(name, value)

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
        return True

    def _on_initialpose3d(self, msg):
        if msg.header.frame_id not in ("", "map"):
            self.get_logger().warn(
                f"/initialpose3d must be in map frame; got {msg.header.frame_id!r}"
            )
            return
        ok = self._spawn_ego_at_map_pose(
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            yaw_from_quaternion(msg.pose.pose.orientation),
            label="/initialpose3d",
        )
        if not ok:
            self.get_logger().warn(
                "/initialpose3d respawn failed; ego unchanged. "
                "Try another location with no obstacle."
            )

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

    # --- Shutdown ---------------------------------------------------------
    def shutdown(self):
        self._shutdown.set()
        if self._world_thread is not None and self._world_thread.is_alive():
            self._world_thread.join(timeout=2.0)
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
