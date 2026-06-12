#!/usr/bin/env python3

# Minimal AD API stub for the diffusion_planner lockstep simulation.
#
# Provides just enough of the AD API surface required by scenario_simulator_v2's
# concealer (FieldOperatorApplication) so that a scenario can initialize, set a
# route and engage WITHOUT launching localization / perception / control stacks
# or autoware_default_adapi (whose operation_mode_transition_manager refuses to
# report is_autonomous_mode_available without control topics).
#
# Responsibilities:
#   - operation mode / localization / engage state machine (trivial)
#   - forwarding of routing AD API calls to mission_planner (route_selector)
#   - periodic NORMAL mrm_state and DRIVE gear_cmd (the perfect trajectory
#     tracker clamps velocity to zero while the gear is NONE)

import threading
import uuid

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from autoware_adapi_v1_msgs.msg import (
    LocalizationInitializationState,
    MrmState,
    OperationModeState,
    RouteState,
)
from autoware_adapi_v1_msgs.srv import (
    ChangeOperationMode,
    ClearRoute,
    InitializeLocalization,
    SetRoute,
    SetRoutePoints,
)
from autoware_planning_msgs.msg import LaneletPrimitive, LaneletSegment
from autoware_planning_msgs.msg import RouteState as InternalRouteState
from autoware_planning_msgs.srv import ClearRoute as InternalClearRoute
from autoware_planning_msgs.srv import SetLaneletRoute, SetWaypointRoute
from autoware_vehicle_msgs.msg import GearCommand
from tier4_external_api_msgs.msg import ResponseStatus
from tier4_external_api_msgs.srv import Engage, SetVelocityLimit
from tier4_rtc_msgs.srv import AutoModeWithModule, CooperateCommands
from unique_identifier_msgs.msg import UUID


def transient_local(depth=1):
    return QoSProfile(depth=depth, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class MinimalAdApiStub(Node):
    def __init__(self):
        super().__init__("minimal_adapi_stub")

        self.group = ReentrantCallbackGroup()
        self.lock = threading.Lock()

        self.localization_state = LocalizationInitializationState.UNINITIALIZED
        self.operation_mode = OperationModeState.STOP
        self.autoware_control_enabled = False

        # --- state publishers (transient_local, latched) ---
        self.pub_localization_state = self.create_publisher(
            LocalizationInitializationState,
            "/api/localization/initialization_state",
            transient_local(),
        )
        self.pub_operation_mode_state = self.create_publisher(
            OperationModeState, "/api/operation_mode/state", transient_local()
        )
        # mission_planner subscribes to the system-internal operation mode topic
        # (used only for reroute safety checks); mirror the AD API state there.
        self.pub_system_operation_mode_state = self.create_publisher(
            OperationModeState, "/system/operation_mode/state", transient_local()
        )
        self.pub_route_state = self.create_publisher(
            RouteState, "/api/routing/state", transient_local()
        )
        self.pub_mrm_state = self.create_publisher(MrmState, "/api/fail_safe/mrm_state", 1)
        self.pub_gear_cmd = self.create_publisher(GearCommand, "/control/command/gear_cmd", 1)

        # --- internal route state from mission_planner (route_selector) ---
        self.create_subscription(
            InternalRouteState,
            "/planning/route_state",
            self.on_internal_route_state,
            transient_local(depth=3),
            callback_group=self.group,
        )

        # --- clients to mission_planner main route interface ---
        self.cli_set_waypoint_route = self.create_client(
            SetWaypointRoute, "/planning/set_waypoint_route", callback_group=self.group
        )
        self.cli_set_lanelet_route = self.create_client(
            SetLaneletRoute, "/planning/set_lanelet_route", callback_group=self.group
        )
        self.cli_clear_route = self.create_client(
            InternalClearRoute, "/planning/clear_route", callback_group=self.group
        )

        # --- AD API services required by concealer ---
        self.create_service(
            InitializeLocalization,
            "/api/localization/initialize",
            self.on_initialize_localization,
            callback_group=self.group,
        )
        self.create_service(
            ChangeOperationMode,
            "/api/operation_mode/change_to_stop",
            self.on_change_to_stop,
            callback_group=self.group,
        )
        self.create_service(
            ChangeOperationMode,
            "/api/operation_mode/enable_autoware_control",
            self.on_enable_autoware_control,
            callback_group=self.group,
        )
        self.create_service(
            Engage, "/api/external/set/engage", self.on_engage, callback_group=self.group
        )
        self.create_service(
            SetRoutePoints,
            "/api/routing/set_route_points",
            self.on_set_route_points,
            callback_group=self.group,
        )
        self.create_service(
            SetRoute, "/api/routing/set_route", self.on_set_route, callback_group=self.group
        )
        self.create_service(
            ClearRoute, "/api/routing/clear_route", self.on_clear_route, callback_group=self.group
        )
        self.create_service(
            SetVelocityLimit,
            "/api/autoware/set/velocity_limit",
            self.on_set_velocity_limit,
            callback_group=self.group,
        )
        self.create_service(
            AutoModeWithModule,
            "/api/external/set/rtc_auto_mode",
            self.on_rtc_auto_mode,
            callback_group=self.group,
        )
        self.create_service(
            CooperateCommands,
            "/api/external/set/rtc_commands",
            self.on_rtc_commands,
            callback_group=self.group,
        )

        self.publish_states()
        self.create_timer(0.1, self.on_timer, callback_group=self.group)

    # ------------------------------------------------------------------
    def publish_states(self):
        with self.lock:
            localization_state = self.localization_state
            operation_mode = self.operation_mode
            control_enabled = self.autoware_control_enabled
        stamp = self.get_clock().now().to_msg()

        localization = LocalizationInitializationState()
        localization.stamp = stamp
        localization.state = localization_state
        self.pub_localization_state.publish(localization)

        operation = OperationModeState()
        operation.stamp = stamp
        operation.mode = operation_mode
        operation.is_autoware_control_enabled = control_enabled
        operation.is_in_transition = False
        operation.is_stop_mode_available = True
        operation.is_autonomous_mode_available = True
        operation.is_local_mode_available = True
        operation.is_remote_mode_available = True
        self.pub_operation_mode_state.publish(operation)
        self.pub_system_operation_mode_state.publish(operation)

    def on_timer(self):
        mrm = MrmState()
        mrm.stamp = self.get_clock().now().to_msg()
        mrm.state = MrmState.NORMAL
        mrm.behavior = MrmState.NONE
        self.pub_mrm_state.publish(mrm)

        gear = GearCommand()
        gear.stamp = self.get_clock().now().to_msg()
        gear.command = GearCommand.DRIVE
        self.pub_gear_cmd.publish(gear)

    def on_internal_route_state(self, internal):
        # Same conversion table as autoware_default_adapi route_conversion.cpp
        table = {
            InternalRouteState.INITIALIZING: RouteState.UNSET,
            InternalRouteState.UNSET: RouteState.UNSET,
            InternalRouteState.ROUTING: RouteState.UNSET,
            InternalRouteState.SET: RouteState.SET,
            InternalRouteState.REROUTING: RouteState.CHANGING,
            InternalRouteState.ARRIVED: RouteState.ARRIVED,
            InternalRouteState.ABORTED: RouteState.SET,
            InternalRouteState.INTERRUPTED: RouteState.SET,
        }
        external = RouteState()
        external.stamp = internal.stamp
        external.state = table.get(internal.state, RouteState.UNKNOWN)
        self.pub_route_state.publish(external)

    # ------------------------------------------------------------------
    def on_initialize_localization(self, request, response):
        with self.lock:
            self.localization_state = LocalizationInitializationState.INITIALIZED
        self.publish_states()
        self.get_logger().info("localization initialized (stub)")
        response.status.success = True
        return response

    def on_change_to_stop(self, request, response):
        with self.lock:
            self.operation_mode = OperationModeState.STOP
        self.publish_states()
        response.status.success = True
        return response

    def on_enable_autoware_control(self, request, response):
        with self.lock:
            self.autoware_control_enabled = True
        self.publish_states()
        response.status.success = True
        return response

    def on_engage(self, request, response):
        with self.lock:
            if request.engage:
                self.operation_mode = OperationModeState.AUTONOMOUS
                self.autoware_control_enabled = True
            else:
                self.operation_mode = OperationModeState.STOP
                self.autoware_control_enabled = False
        self.publish_states()
        self.get_logger().info(f"engage: {request.engage} (stub)")
        response.status.code = ResponseStatus.SUCCESS
        return response

    # ------------------------------------------------------------------
    def call_internal(self, client, request, timeout_sec=30.0):
        if not client.wait_for_service(timeout_sec=timeout_sec):
            return None
        future = client.call_async(request)
        event = threading.Event()
        future.add_done_callback(lambda _: event.set())
        if not event.wait(timeout_sec):
            return None
        return future.result()

    def forward_route_request(self, client, internal, response, unavailable_message):
        """Call an internal mission_planner service and mirror its status."""
        result = self.call_internal(client, internal)
        response.status.success = result is not None and result.status.success
        if not response.status.success:
            response.status.message = (
                result.status.message if result else unavailable_message
            )
        return response

    def on_set_route_points(self, request, response):
        internal = SetWaypointRoute.Request()
        internal.header = request.header
        internal.goal_pose = request.goal
        internal.waypoints = list(request.waypoints)
        internal.uuid = UUID(uuid=list(uuid.uuid4().bytes))
        internal.allow_modification = request.option.allow_goal_modification
        return self.forward_route_request(
            self.cli_set_waypoint_route, internal, response, "set_waypoint_route unavailable")

    def on_set_route(self, request, response):
        internal = SetLaneletRoute.Request()
        internal.header = request.header
        internal.goal_pose = request.goal
        for segment in request.segments:
            out = LaneletSegment()
            out.preferred_primitive = LaneletPrimitive(
                id=segment.preferred.id, primitive_type=segment.preferred.type
            )
            out.primitives.append(out.preferred_primitive)
            for primitive in segment.alternatives:
                out.primitives.append(
                    LaneletPrimitive(id=primitive.id, primitive_type=primitive.type)
                )
            internal.segments.append(out)
        internal.uuid = UUID(uuid=list(uuid.uuid4().bytes))
        internal.allow_modification = request.option.allow_goal_modification
        return self.forward_route_request(
            self.cli_set_lanelet_route, internal, response, "set_lanelet_route unavailable")

    def on_clear_route(self, request, response):
        return self.forward_route_request(
            self.cli_clear_route, InternalClearRoute.Request(), response,
            "clear_route unavailable")

    # ------------------------------------------------------------------
    def on_set_velocity_limit(self, request, response):
        response.status.code = ResponseStatus.SUCCESS
        return response

    def on_rtc_auto_mode(self, request, response):
        response.success = True
        return response

    def on_rtc_commands(self, request, response):
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MinimalAdApiStub()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
