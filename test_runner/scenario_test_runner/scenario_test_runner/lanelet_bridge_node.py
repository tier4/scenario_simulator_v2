#!/usr/bin/env python3
"""ROS2 bridge node: subscribes to /map/vector_map (LaneletMapBin),
deserializes the lanelet2 map, and serves geometry batches on demand
via /godot/lanelet_batch_count and /godot/lanelet_batch services.

Usage:
    source /path/to/autoware/install/setup.bash
    python3 lanelet_bridge_node.py
"""

import ctypes
import json
import os
import tempfile
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from autoware_map_msgs.msg import LaneletMapBin
from tf2_msgs.msg import TFMessage

import lanelet2
from lanelet2.io import Origin, loadRobust
from lanelet2.projection import UtmProjector

# Pre-load Autoware lanelet2 extension to register custom regulatory elements.
# Searches AMENT_PREFIX_PATH and common install paths.
_search_paths = []
for prefix in os.environ.get("AMENT_PREFIX_PATH", "").split(":"):
    candidate = os.path.join(prefix, "lib", "libautoware_lanelet2_extension_lib.so")
    if os.path.isfile(candidate):
        _search_paths.append(candidate)
for candidate in _search_paths:
    if os.path.isfile(candidate):
        try:
            ctypes.CDLL(candidate, mode=ctypes.RTLD_GLOBAL)
            print(f"[lanelet_bridge] Loaded extension: {candidate}")
            break
        except OSError as e:
            print(f"[lanelet_bridge] WARN: {e}")


class LaneletBridgeNode(Node):
    BATCH_SIZE = 2000

    def __init__(self):
        super().__init__("lanelet_bridge_node")

        # Subscribe to vector map
        sub_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.sub_ = self.create_subscription(
            LaneletMapBin, "/map/vector_map", self._on_map, sub_qos
        )

        # Services for on-demand batch delivery
        from std_msgs.msg import String
        from std_srvs.srv import Trigger
        from example_interfaces.srv import SetBool

        self.srv_count_ = self.create_service(
            Trigger, "/godot/lanelet_batch_count", self._on_batch_count
        )
        self.srv_batch_ = self.create_service(
            SetBool, "/godot/lanelet_batch", self._on_batch_request
        )

        self._batches: list = []
        self._batches_json: list = []
        self._last_map_data: bytes = None
        self._ready = threading.Event()
        self._processing_lock = threading.Lock()
        self.get_logger().info("Waiting for /map/vector_map ...")

        # Relay /tf_static -> /tf so rosbridge can forward static transforms to Godot.
        # rosbridge subscribes to /tf_static with volatile QoS when no publishers exist yet,
        # causing Godot to miss the cached map->viewer transform.
        # By republishing on /tf (volatile) at 1 Hz, Godot reliably receives all static TFs.
        self._static_transforms: list = []
        self._static_tf_lock = threading.Lock()

        static_qos = QoSProfile(
            depth=100,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._tf_static_sub = self.create_subscription(
            TFMessage, "/tf_static", self._on_tf_static, static_qos
        )
        self._tf_pub = self.create_publisher(TFMessage, "/tf", 100)
        self._tf_relay_timer = self.create_timer(1.0, self._relay_static_tfs)

    def _on_tf_static(self, msg: TFMessage):
        with self._static_tf_lock:
            for tf in msg.transforms:
                key = (tf.header.frame_id, tf.child_frame_id)
                is_new = not any(
                    (t.header.frame_id, t.child_frame_id) == key
                    for t in self._static_transforms
                )
                self._static_transforms = [
                    t for t in self._static_transforms
                    if (t.header.frame_id, t.child_frame_id) != key
                ]
                self._static_transforms.append(tf)
                if is_new:
                    self.get_logger().info(
                        f"[tf_relay] NEW /tf_static received: "
                        f"{tf.header.frame_id} -> {tf.child_frame_id}  "
                        f"t=({tf.transform.translation.x:.3f}, "
                        f"{tf.transform.translation.y:.3f}, "
                        f"{tf.transform.translation.z:.3f})"
                    )
                else:
                    self.get_logger().debug(
                        f"[tf_relay] Updated /tf_static: "
                        f"{tf.header.frame_id} -> {tf.child_frame_id}"
                    )

    def _relay_static_tfs(self):
        stamp = self.get_clock().now().to_msg()
        with self._static_tf_lock:
            if not self._static_transforms:
                self.get_logger().debug("[tf_relay] No static TFs cached yet, skip relay.")
                return
            msg = TFMessage()
            for tf in self._static_transforms:
                tf.header.stamp = stamp
            msg.transforms = list(self._static_transforms)
            n = len(msg.transforms)
        self._tf_pub.publish(msg)
        self.get_logger().debug(f"[tf_relay] Relayed {n} static TF(s) to /tf")

    def _on_map(self, msg: LaneletMapBin):
        self.get_logger().info(f"Received /map/vector_map ({len(msg.data)} bytes)")
        self._last_map_data = bytes(msg.data)
        self._ready.clear()
        thread = threading.Thread(target=self._prepare_batches_threaded, daemon=True)
        thread.start()

    def _prepare_batches_threaded(self):
        with self._processing_lock:
            self._prepare_batches()
            self._ready.set()

    def _prepare_batches(self):
        lanelet_map = self._deserialize(self._last_map_data)
        if lanelet_map is None:
            return

        all_lanelets = self._extract_lanelets(lanelet_map)
        intersection_areas = self._extract_intersection_areas(lanelet_map)
        hatched_road_markings = self._extract_hatched_road_markings(lanelet_map)
        parking_lots = self._extract_parking_lots(lanelet_map)
        road_borders = self._extract_road_borders(lanelet_map)
        shoulders = self._extract_shoulders(lanelet_map)
        road_markings = self._extract_road_markings(lanelet_map)
        traffic_light_groups = self._extract_traffic_light_groups(lanelet_map)

        self.get_logger().info(
            f"Prepared: {len(all_lanelets)} ll, {len(intersection_areas)} ia, "
            f"{len(hatched_road_markings)} hm, {len(parking_lots)} pk, "
            f"{len(road_borders)} rb, {len(shoulders)} sh, {len(road_markings)} rm, "
            f"{len(traffic_light_groups)} tl"
        )

        # Build all batch payloads
        BS = self.BATCH_SIZE
        self._batches = []

        for i in range(0, max(len(all_lanelets), 1), BS):
            self._batches.append({"lanelets": all_lanelets[i:i+BS]})
        for i in range(0, max(len(intersection_areas), 1), BS):
            chunk = intersection_areas[i:i+BS]
            if chunk:
                self._batches.append({"intersection_areas": chunk})
        for i in range(0, max(len(hatched_road_markings), 1), BS):
            chunk = hatched_road_markings[i:i+BS]
            if chunk:
                self._batches.append({"hatched_road_markings": chunk})
        for i in range(0, max(len(parking_lots), 1), BS):
            chunk = parking_lots[i:i+BS]
            if chunk:
                self._batches.append({"parking_lots": chunk})
        for i in range(0, max(len(road_borders), 1), BS):
            chunk = road_borders[i:i+BS]
            if chunk:
                self._batches.append({"road_borders": chunk})
        for i in range(0, max(len(shoulders), 1), BS):
            chunk = shoulders[i:i+BS]
            if chunk:
                self._batches.append({"shoulders": chunk})
        for i in range(0, max(len(road_markings), 1), BS):
            chunk = road_markings[i:i+BS]
            if chunk:
                self._batches.append({"road_markings": chunk})
        if traffic_light_groups:
            self._batches.append({"traffic_light_groups": traffic_light_groups})

        self._batches_json = [json.dumps(batch) for batch in self._batches]
        self.get_logger().info(f"Ready: {len(self._batches)} batches for serving")

    def _on_batch_count(self, request, response):
        """Returns the number of available batches."""
        if not self._ready.is_set():
            if self._last_map_data:
                self.get_logger().info("Waiting for map processing to complete...")
                self._ready.wait(timeout=30.0)
        response.success = self._ready.is_set() and len(self._batches) > 0
        response.message = str(len(self._batches))
        self.get_logger().info(f"Batch count requested → {len(self._batches)}")
        return response

    def _on_batch_request(self, request, response):
        """Returns batch at index (passed via request.data as bool trick:
        the actual index is sent in a separate mechanism).
        We use a simple pop approach: each call returns the next batch."""
        # Use request.data as a reset flag: True = start from beginning
        if request.data:
            self._serve_index = 0

        if not hasattr(self, '_serve_index'):
            self._serve_index = 0

        if self._serve_index < len(self._batches_json):
            response.success = True
            response.message = self._batches_json[self._serve_index]
            self._serve_index += 1
        else:
            response.success = False
            response.message = ""

        return response

    def _deserialize(self, data: bytes):
        fd, tmp_path = tempfile.mkstemp(suffix=".bin")
        try:
            with os.fdopen(fd, "wb") as f:
                f.write(bytes(data))
            projector = UtmProjector(Origin(0, 0))
            lanelet_map, errors = loadRobust(tmp_path, projector)
            if errors:
                self.get_logger().warn(f"Map loaded with {len(errors)} warnings")
            return lanelet_map
        except Exception as e:
            self.get_logger().error(f"Failed to deserialize: {e}")
            return None
        finally:
            os.unlink(tmp_path)

    def _extract_lanelets(self, lanelet_map) -> list:
        lanelets = []
        for ll in lanelet_map.laneletLayer:
            left_pts = [[round(p.x, 3), round(p.y, 3), round(p.z, 3)] for p in ll.leftBound]
            right_pts = [[round(p.x, 3), round(p.y, 3), round(p.z, 3)] for p in ll.rightBound]
            if len(left_pts) < 2 or len(right_pts) < 2:
                continue
            lanelets.append({"id": int(ll.id), "left": left_pts, "right": right_pts})
        return lanelets

    def _extract_polygons_by_type(self, lanelet_map, type_name: str) -> list:
        areas = []
        for poly in lanelet_map.polygonLayer:
            if "type" not in poly.attributes:
                continue
            if str(poly.attributes["type"]) != type_name:
                continue
            pts = [[round(p.x, 3), round(p.y, 3), round(p.z, 3)] for p in poly]
            if len(pts) < 3:
                continue
            areas.append({"id": int(poly.id), "points": pts})
        return areas

    def _extract_intersection_areas(self, lanelet_map) -> list:
        return self._extract_polygons_by_type(lanelet_map, "intersection_area")

    def _extract_hatched_road_markings(self, lanelet_map) -> list:
        return self._extract_polygons_by_type(lanelet_map, "hatched_road_markings")

    def _extract_parking_lots(self, lanelet_map) -> list:
        return self._extract_polygons_by_type(lanelet_map, "parking_lot")

    def _extract_road_borders(self, lanelet_map) -> list:
        borders = []
        for ls in lanelet_map.lineStringLayer:
            try:
                if str(ls.attributes["type"]) != "road_border":
                    continue
            except Exception:
                continue
            pts = [[round(p.x, 3), round(p.y, 3), round(p.z, 3)] for p in ls]
            if len(pts) < 2:
                continue
            borders.append({"id": int(ls.id), "points": pts})
        return borders

    def _extract_shoulders(self, lanelet_map) -> list:
        import numpy as np
        from scipy.spatial import KDTree

        boundary_usage = {}
        for ll in lanelet_map.laneletLayer:
            boundary_usage[ll.leftBound.id] = boundary_usage.get(ll.leftBound.id, 0) + 1
            boundary_usage[ll.rightBound.id] = boundary_usage.get(ll.rightBound.id, 0) + 1

        rb_pts_xy, rb_pts_z = [], []
        for ls in lanelet_map.lineStringLayer:
            try:
                if str(ls.attributes["type"]) != "road_border":
                    continue
            except Exception:
                continue
            for pt in ls:
                rb_pts_xy.append([pt.x, pt.y])
                rb_pts_z.append(pt.z)
        if not rb_pts_xy:
            return []

        rb_xy = np.array(rb_pts_xy)
        rb_z = np.array(rb_pts_z)
        tree = KDTree(rb_xy)

        MAX_AVG_DIST = 10.0
        shoulders = []
        for ls in lanelet_map.lineStringLayer:
            if ls.id not in boundary_usage or boundary_usage[ls.id] != 1:
                continue
            try:
                if str(ls.attributes["type"]) == "road_border":
                    continue
            except Exception:
                pass
            pts = list(ls)
            if len(pts) < 2:
                continue
            query = np.array([[p.x, p.y] for p in pts])
            dists, indices = tree.query(query)
            if np.mean(dists) > MAX_AVG_DIST or np.mean(dists) < 0.3:
                continue
            left_pts = [[round(p.x, 3), round(p.y, 3), round(p.z, 3)] for p in pts]
            right_pts = [
                [round(float(rb_xy[idx][0]), 3), round(float(rb_xy[idx][1]), 3), round(float(rb_z[idx]), 3)]
                for idx in indices
            ]
            shoulders.append({"left": left_pts, "right": right_pts})
        return shoulders

    def _extract_road_markings(self, lanelet_map) -> list:
        MARKING_TYPES = {"line_thin", "line_thick", "stop_line"}
        markings = []
        for ls in lanelet_map.lineStringLayer:
            try:
                t = str(ls.attributes["type"])
            except Exception:
                continue
            if t not in MARKING_TYPES:
                continue
            pts = [[round(p.x, 3), round(p.y, 3), round(p.z, 3)] for p in ls]
            if len(pts) < 2:
                continue
            markings.append({"points": pts, "type": t})
        return markings

    def _extract_traffic_light_groups(self, lanelet_map) -> list:
        """Extract traffic light groups from regulatory elements."""
        groups = []
        seen_ids: set = set()

        for ll in lanelet_map.laneletLayer:
            for reg in ll.regulatoryElements:
                if reg.id in seen_ids:
                    continue
                seen_ids.add(reg.id)

                try:
                    subtype = str(reg.attributes["subtype"])
                except Exception:
                    continue
                if subtype != "traffic_light":
                    continue

                # Get referred linestrings (traffic light fixtures)
                refers = []
                try:
                    refers = list(reg.parameters["refers"])
                except Exception:
                    pass
                if not refers:
                    continue

                # Get light_bulbs linestrings if available (Autoware extension)
                light_bulb_linestrings = []
                try:
                    light_bulb_linestrings = list(reg.parameters["light_bulbs"])
                except Exception:
                    pass

                # Build boards_map: linestring_id -> board dict
                boards_map = {}
                for fixture_ls in refers:
                    try:
                        pts = list(fixture_ls)
                    except Exception:
                        continue
                    if len(pts) < 2:
                        continue

                    # Read height attribute (same as C++ vector_map.cpp)
                    height = 0.7
                    try:
                        if "height" in fixture_ls.attributes:
                            height = float(fixture_ls.attributes["height"])
                    except Exception:
                        pass

                    front = pts[0]
                    back = pts[-1]

                    board = self._compute_tl_board(front, back, height)
                    boards_map[fixture_ls.id] = board

                # Extract bulbs, keyed by board id
                light_bulbs_map = {}
                for bulb_ls in light_bulb_linestrings:
                    try:
                        board_id = int(bulb_ls.attributes["traffic_light_id"])
                    except Exception:
                        continue
                    bulb_list = []
                    for pt in bulb_ls:
                        color = "none"
                        arrow = "none"
                        radius = 0.1
                        try:
                            color = str(pt.attributes["color"])
                        except Exception:
                            pass
                        try:
                            arrow = str(pt.attributes["arrow"])
                        except Exception:
                            pass
                        try:
                            radius = float(pt.attributes["radius"])
                        except Exception:
                            pass
                        # Bulb normal inherited from board (same as C++)
                        normal = boards_map[board_id]["normal"] \
                            if board_id in boards_map else [1.0, 0.0, 0.0]
                        bulb_list.append({
                            "position": [round(pt.x, 3), round(pt.y, 3),
                                         round(pt.z, 3)],
                            "radius": radius,
                            "color": color,
                            "arrow": arrow,
                            "normal": normal,
                        })
                    if bulb_list:
                        light_bulbs_map[board_id] = bulb_list

                # Assemble traffic_lights per board
                traffic_lights = []
                for ls_id, board in boards_map.items():
                    bulbs = light_bulbs_map.get(ls_id, [])
                    if not bulbs:
                        # Fallback: generate default bulbs
                        front = list(refers[0])[0]
                        back = list(refers[0])[-1]
                        bulbs = self._default_tl_bulbs(front, back, height)
                        for b in bulbs:
                            b["normal"] = board["normal"]
                    traffic_lights.append({
                        "board": board,
                        "light_bulbs": bulbs,
                    })

                if traffic_lights:
                    groups.append({
                        "group_id": int(reg.id),
                        "traffic_lights": traffic_lights,
                    })

        self.get_logger().info(
            f"Extracted {len(groups)} traffic light groups"
        )
        return groups

    @staticmethod
    def _compute_tl_board(front, back, height: float) -> dict:
        """Compute board from linestring front/back points + height attribute.

        Matches C++ vector_map.cpp logic exactly:
        - Bottom corners at linestring z
        - Top corners at linestring z + height
        - Normal = cross(right_top - left_top, left_bottom - left_top)
        """
        # 4 corners (same as C++)
        # front = left, back = right
        lt = [front.x, front.y, front.z + height]  # left_top
        rt = [back.x,  back.y,  back.z + height]   # right_top
        lb = [front.x, front.y, front.z]            # left_bottom
        rb = [back.x,  back.y,  back.z]             # right_bottom

        # Center
        cx = (lt[0] + rt[0] + lb[0] + rb[0]) / 4.0
        cy = (lt[1] + rt[1] + lb[1] + rb[1]) / 4.0
        cz = (lt[2] + rt[2] + lb[2] + rb[2]) / 4.0

        # Width = horizontal distance between front and back
        dx = back.x - front.x
        dy = back.y - front.y
        board_w = (dx ** 2 + dy ** 2) ** 0.5

        # Height = the height attribute
        board_h = height

        # Normal = cross(right_top - left_top, left_bottom - left_top)
        # u = rt - lt = (dx, dy, dz_top)
        ux = rt[0] - lt[0]
        uy = rt[1] - lt[1]
        uz = rt[2] - lt[2]
        # v = lb - lt = (0, 0, -height)
        vx = lb[0] - lt[0]
        vy = lb[1] - lt[1]
        vz = lb[2] - lt[2]
        # cross product
        nx = uy * vz - uz * vy
        ny = uz * vx - ux * vz
        nz = ux * vy - uy * vx
        nl = max((nx ** 2 + ny ** 2 + nz ** 2) ** 0.5, 0.001)
        nx /= nl
        ny /= nl
        nz /= nl

        return {
            "center": [round(cx, 3), round(cy, 3), round(cz, 3)],
            "width": round(board_w, 3),
            "height": round(board_h, 3),
            "normal": [round(nx, 6), round(ny, 6), round(nz, 6)],
        }

    @staticmethod
    def _default_tl_bulbs(front, back, height: float) -> list:
        """Create default red/yellow/green bulbs between front/back at mid-height."""
        cx = (front.x + back.x) / 2.0
        cy = (front.y + back.y) / 2.0
        cz = (front.z + back.z) / 2.0 + height / 2.0  # mid-height of board

        dx = back.x - front.x
        dy = back.y - front.y
        horiz_ext = max((dx ** 2 + dy ** 2) ** 0.5, 0.01)
        hdx = dx / horiz_ext
        hdy = dy / horiz_ext

        spacing = horiz_ext / 3.0 if horiz_ext > 0.3 else 0.12
        bulbs = []
        for i, color in enumerate(["red", "yellow", "green"]):
            offset = (i - 1) * spacing
            bulbs.append({
                "position": [
                    round(cx + hdx * offset, 3),
                    round(cy + hdy * offset, 3),
                    round(cz, 3),
                ],
                "radius": min(spacing * 0.4, 0.1),
                "color": color,
                "arrow": "none",
            })
        return bulbs


def main():
    rclpy.init()
    node = LaneletBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()