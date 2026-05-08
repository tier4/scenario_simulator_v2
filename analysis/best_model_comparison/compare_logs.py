#!/usr/bin/env python3
"""実機ログ vs Godotシム vs 通常シム の三方比較プロット生成スクリプト.

Usage:
    python3 compare_logs.py

Outputs: comparison/figures/*.{png,pdf}, comparison/report.md
"""

import math
import warnings
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
matplotlib.rcParams["font.family"] = "Noto Sans CJK JP"
import matplotlib.patches as mpatches
import numpy as np
import pandas as pd
from lxml import etree
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory

# ---------------------------------------------------------------------------
# 設定
# ---------------------------------------------------------------------------
BASE = Path(__file__).parent
LITE_DIR = BASE / "lite"
OUT_DIR = BASE / "comparison"
FIGS_DIR = OUT_DIR / "figures"
MAP_DIR = Path.home() / ".webauto/simulation/data/map/x2_dev/2231"
SCENARIO_NAME = "x2_dev/2231 テレポート駅→日本科学未来館"

LOGS = {
    "実機": {
        "path": LITE_DIR / "real.lite.mcap",
        "kinematic": "/sub/localization/kinematic_state",
        "accel": "/sub/localization/acceleration",
        "cmd": "/sub/control/command/control_cmd",
        "color": "black",
        "lw": 2.5,
        "ls": "-",
        "marker": "o",
        "ms": 5,
    },
    "Godot シム": {
        "path": LITE_DIR / "sim_godot.lite.mcap",
        "kinematic": "/localization/kinematic_state",
        "accel": "/localization/acceleration",
        "cmd": "/control/trajectory_follower/control_cmd",
        "color": "#1f77b4",
        "lw": 2.0,
        "ls": "--",
        "marker": "^",
        "ms": 5,
    },
    "通常シム": {
        "path": LITE_DIR / "sim_normal.lite.mcap",
        "kinematic": "/localization/kinematic_state",
        "accel": "/localization/acceleration",
        "cmd": "/control/trajectory_follower/control_cmd",
        "color": "#ff7f0e",
        "lw": 2.0,
        "ls": (0, (4, 2)),
        "marker": "s",
        "ms": 5,
    },
}

AUTONOMOUS_MODE = 2  # autoware_adapi_v1_msgs OperationModeState.AUTONOMOUS
WHEELBASE = 5.15     # m — kinematic_state × steering から実データで推定

# 地図座標系でのカーブ中心（右折・左折・右折の順）
CURVE_CENTERS = [
    {"label": "カーブ①（右折）", "cx": 89440, "cy": 43200, "margin": 20},
    {"label": "カーブ②（左折）", "cx": 89301, "cy": 43085, "margin": 20},
    {"label": "カーブ③（右折）", "cx": 89372, "cy": 42830, "margin": 40},
]


# ---------------------------------------------------------------------------
# MCAPローダー
# ---------------------------------------------------------------------------

def _iter_msgs(mcap_path: Path, topics: list[str]):
    with open(mcap_path, "rb") as f:
        reader = make_reader(f, decoder_factories=[DecoderFactory()])
        yield from reader.iter_decoded_messages(topics=topics)


def load_operation_mode(mcap_path: Path) -> pd.DataFrame:
    rows = []
    for _, channel, message, ros_msg in _iter_msgs(
        mcap_path, ["/system/operation_mode/state"]
    ):
        rows.append({"t_ns": message.log_time, "mode": ros_msg.mode})
    if not rows:
        return pd.DataFrame(columns=["t_ns", "mode"])
    return pd.DataFrame(rows)


def load_velocity(mcap_path: Path) -> pd.DataFrame:
    rows = []
    for _, channel, message, ros_msg in _iter_msgs(
        mcap_path, ["/vehicle/status/velocity_status"]
    ):
        rows.append({
            "t_ns": message.log_time,
            "lon_vel": ros_msg.longitudinal_velocity,
        })
    return pd.DataFrame(rows)


def load_steering(mcap_path: Path) -> pd.DataFrame:
    rows = []
    for _, channel, message, ros_msg in _iter_msgs(
        mcap_path, ["/vehicle/status/steering_status"]
    ):
        rows.append({
            "t_ns": message.log_time,
            "steer": ros_msg.steering_tire_angle,
        })
    return pd.DataFrame(rows)


def load_kinematic(mcap_path: Path, topic: str) -> pd.DataFrame:
    rows = []
    for _, channel, message, ros_msg in _iter_msgs(mcap_path, [topic]):
        p = ros_msg.pose.pose.position
        o = ros_msg.pose.pose.orientation
        yaw = math.atan2(2*(o.w*o.z + o.x*o.y), 1 - 2*(o.y*o.y + o.z*o.z))
        rows.append({
            "t_ns": message.log_time,
            "x": p.x,
            "y": p.y,
            "yaw": yaw,
        })
    return pd.DataFrame(rows)


def load_accel(mcap_path: Path, topic: str) -> pd.DataFrame:
    rows = []
    for _, channel, message, ros_msg in _iter_msgs(mcap_path, [topic]):
        a = ros_msg.accel.accel.linear
        rows.append({"t_ns": message.log_time, "accel": a.x})
    return pd.DataFrame(rows)


def load_cmd(mcap_path: Path, topic: str) -> pd.DataFrame:
    rows = []
    for _, channel, message, ros_msg in _iter_msgs(mcap_path, [topic]):
        rows.append({
            "t_ns": message.log_time,
            "cmd_vel": ros_msg.longitudinal.velocity,
            "cmd_accel": ros_msg.longitudinal.acceleration,
            "cmd_steer": ros_msg.lateral.steering_tire_angle,
        })
    return pd.DataFrame(rows)


# ---------------------------------------------------------------------------
# 時刻整列
# ---------------------------------------------------------------------------

def find_autonomous_start(df_mode: pd.DataFrame, df_vel: pd.DataFrame) -> int:
    """オートノマス開始時刻 (ns) を返す。取れない場合は速度ベースのフォールバック。"""
    auto_rows = df_mode[df_mode["mode"] == AUTONOMOUS_MODE]
    if not auto_rows.empty:
        return int(auto_rows["t_ns"].iloc[0])
    warnings.warn("AUTONOMOUS モード遷移が見つからないため速度ベースにフォールバック", stacklevel=2)
    moving = df_vel[df_vel["lon_vel"] > 0.1]
    if not moving.empty:
        return int(moving["t_ns"].iloc[0])
    return int(df_vel["t_ns"].iloc[0])


def align_time(df: pd.DataFrame, t0_ns: int) -> pd.DataFrame:
    df = df.copy()
    df["t"] = (df["t_ns"] - t0_ns) / 1e9
    return df[df["t"] >= 0].reset_index(drop=True)


# ---------------------------------------------------------------------------
# 地図パーサー（OSM local_x / local_y）
# ---------------------------------------------------------------------------

def _resolve_map_osm() -> Path | None:
    """MAP_DIR 下の最新バージョンディレクトリから lanelet2_map.osm を返す。"""
    if not MAP_DIR.exists():
        return None
    candidates = sorted(
        MAP_DIR.glob("2231-*/lanelet2_map.osm"),
        key=lambda p: p.parent.name,
        reverse=True,
    )
    return candidates[0] if candidates else None


def load_map_ways(osm_path: Path) -> tuple[list[np.ndarray], list[np.ndarray]]:
    """OSMをパースし、way の (x,y) 座標列リストを返す。
    戻り値: (road_ways, all_ways) — road_ways は lanelet boundary/centerline。
    """
    tree = etree.parse(str(osm_path))
    root = tree.getroot()

    # ノードマップ: node_id → (local_x, local_y)
    node_xy = {}
    for node in root.findall("node"):
        nid = int(node.get("id"))
        tags = {t.get("k"): t.get("v") for t in node.findall("tag")}
        if "local_x" in tags and "local_y" in tags:
            node_xy[nid] = (float(tags["local_x"]), float(tags["local_y"]))

    # way_id → 座標列
    way_coords = {}
    for way in root.findall("way"):
        wid = int(way.get("id"))
        refs = [int(nd.get("ref")) for nd in way.findall("nd")]
        pts = [node_xy[r] for r in refs if r in node_xy]
        if len(pts) >= 2:
            way_coords[wid] = np.array(pts)

    # relation(lanelet) から使われている way を収集
    used_way_ids = set()
    for rel in root.findall("relation"):
        tags = {t.get("k"): t.get("v") for t in rel.findall("tag")}
        if tags.get("type") == "lanelet":
            for member in rel.findall("member"):
                if member.get("type") == "way":
                    used_way_ids.add(int(member.get("ref")))

    road_ways = [way_coords[wid] for wid in used_way_ids if wid in way_coords]
    return road_ways


# ---------------------------------------------------------------------------
# プロット
# ---------------------------------------------------------------------------

def _setup_fig(title: str, nrows=1, ncols=1, figsize=(12, 5)):
    fig, axes = plt.subplots(nrows, ncols, figsize=figsize, squeeze=False)
    fig.suptitle(f"{SCENARIO_NAME}\n{title}", fontsize=11)
    return fig, axes


def _save(fig, name: str):
    FIGS_DIR.mkdir(parents=True, exist_ok=True)
    for ext in ("png", "pdf"):
        path = FIGS_DIR / f"{name}.{ext}"
        fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  保存: {name}.{{png,pdf}}")


def _traj_plot(ax, df_xy, d, label, markevery=20, zorder=3):
    """軌跡を線スタイル＋等間隔マーカーで描画するヘルパー。"""
    n = len(df_xy)
    me = max(1, n // markevery) if n > 0 else 1
    ax.plot(np.asarray(df_xy["x"]), np.asarray(df_xy["y"]),
            color=d["color"], lw=d["lw"], ls=d["ls"],
            marker=d["marker"], markersize=d["ms"], markevery=me,
            markerfacecolor=d["color"], markeredgecolor="white", markeredgewidth=0.5,
            label=label, zorder=zorder)


def plot_trajectory(data: dict, map_ways: list | None):
    """地図背景あり軌跡プロット。表示範囲は3軌跡の bbox + マージン に限定。"""
    fig, axes = _setup_fig("軌跡比較", figsize=(10, 10))
    ax = axes[0, 0]

    # 3軌跡の bounding box を算出
    all_xy = np.concatenate(
        [d["kinematic"][["x", "y"]].values for d in data.values() if not d["kinematic"].empty]
    )
    margin = 30  # m
    x_min, y_min = all_xy.min(axis=0) - margin
    x_max, y_max = all_xy.max(axis=0) + margin

    if map_ways:
        for pts in map_ways:
            wx, wy = pts[:, 0], pts[:, 1]
            if wx.max() < x_min or wx.min() > x_max:
                continue
            if wy.max() < y_min or wy.min() > y_max:
                continue
            ax.plot(wx, wy, color="#cccccc", lw=0.5, zorder=1)

    for label, d in data.items():
        _traj_plot(ax, d["kinematic"], d, label, markevery=15)

    ax.set_xlim(x_min, x_max)
    ax.set_ylim(y_min, y_max)
    ax.set_aspect("equal")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.grid(True, lw=0.5, alpha=0.5)
    ax.legend(fontsize=10)
    fig.tight_layout()
    _save(fig, "trajectory_with_map" if map_ways else "trajectory_xy")


def _ts_plot(ax, t, y, d, label, cmd=False):
    """時系列プロット用ヘルパー。応答=各ログのスタイル、指令=薄い細線。"""
    t = np.asarray(t)
    y = np.asarray(y)
    if cmd:
        ax.plot(t, y, color=d["color"], lw=1.2, ls=":", alpha=0.65, label=label)
    else:
        ax.plot(t, y, color=d["color"], lw=d["lw"], ls=d["ls"], label=label)


def plot_velocity(data: dict):
    fig, axes = _setup_fig("速度比較（指令 vs 応答）", nrows=2, figsize=(14, 8))

    for label, d in data.items():
        vel = d["velocity"]
        cmd = d["cmd"]
        _ts_plot(axes[0, 0], vel["t"], vel["lon_vel"], d, label)
        if not cmd.empty:
            _ts_plot(axes[1, 0], cmd["t"], cmd["cmd_vel"], d, label, cmd=True)

    axes[0, 0].set_title("実応答 (VelocityReport.longitudinal_velocity)  ─ 実線/破線/点鎖線は各ログ")
    axes[0, 0].set_ylabel("速度 [m/s]")
    axes[0, 0].legend(fontsize=9)
    axes[0, 0].grid(True, lw=0.5)

    axes[1, 0].set_title("指令 (control_cmd.longitudinal.velocity)  ─ 点線・薄色")
    axes[1, 0].set_ylabel("速度指令 [m/s]")
    axes[1, 0].set_xlabel("経過時間 [s]")
    axes[1, 0].legend(fontsize=9)
    axes[1, 0].grid(True, lw=0.5)

    fig.tight_layout()
    _save(fig, "velocity")


def plot_acceleration(data: dict):
    fig, axes = _setup_fig("加速度比較（指令 vs 応答）", nrows=2, figsize=(14, 8))

    for label, d in data.items():
        acc = d["accel"]
        cmd = d["cmd"]
        _ts_plot(axes[0, 0], acc["t"], acc["accel"], d, label)
        if not cmd.empty:
            _ts_plot(axes[1, 0], cmd["t"], cmd["cmd_accel"], d, label, cmd=True)

    axes[0, 0].set_title("実応答 (localization/acceleration.accel.accel.linear.x)")
    axes[0, 0].set_ylabel("加速度 [m/s²]")
    axes[0, 0].legend(fontsize=9)
    axes[0, 0].grid(True, lw=0.5)

    axes[1, 0].set_title("指令 (control_cmd.longitudinal.acceleration)  ─ 点線・薄色")
    axes[1, 0].set_ylabel("加速度指令 [m/s²]")
    axes[1, 0].set_xlabel("経過時間 [s]")
    axes[1, 0].legend(fontsize=9)
    axes[1, 0].grid(True, lw=0.5)

    fig.tight_layout()
    _save(fig, "acceleration")


def plot_steering(data: dict):
    fig, axes = _setup_fig("ステアリング比較（指令 vs 応答）", nrows=2, figsize=(14, 8))

    for label, d in data.items():
        steer = d["steering"]
        cmd = d["cmd"]
        _ts_plot(axes[0, 0], steer["t"], np.degrees(steer["steer"]), d, label)
        if not cmd.empty:
            _ts_plot(axes[1, 0], cmd["t"], np.degrees(cmd["cmd_steer"]), d, label, cmd=True)

    axes[0, 0].set_title("実応答 (SteeringReport.steering_tire_angle)")
    axes[0, 0].set_ylabel("ステア角 [deg]")
    axes[0, 0].legend(fontsize=9)
    axes[0, 0].grid(True, lw=0.5)

    axes[1, 0].set_title("指令 (control_cmd.lateral.steering_tire_angle)  ─ 点線・薄色")
    axes[1, 0].set_ylabel("ステア指令 [deg]")
    axes[1, 0].set_xlabel("経過時間 [s]")
    axes[1, 0].legend(fontsize=9)
    axes[1, 0].grid(True, lw=0.5)

    fig.tight_layout()
    _save(fig, "steering")


def plot_curves(data: dict, map_ways: list | None):
    """3つのカーブにフォーカスした軌跡比較（横3列サブプロット）。"""
    n = len(CURVE_CENTERS)
    fig, axes = plt.subplots(1, n, figsize=(6 * n, 7))
    fig.suptitle(f"{SCENARIO_NAME}\nカーブ別軌跡比較", fontsize=11)

    for col, curve in enumerate(CURVE_CENTERS):
        ax = axes[col]
        cx, cy, mg = curve["cx"], curve["cy"], curve["margin"]
        x_min, x_max = cx - mg, cx + mg
        y_min, y_max = cy - mg, cy + mg

        # 地図背景
        if map_ways:
            for pts in map_ways:
                wx, wy = pts[:, 0], pts[:, 1]
                if wx.max() < x_min or wx.min() > x_max:
                    continue
                if wy.max() < y_min or wy.min() > y_max:
                    continue
                ax.plot(wx, wy, color="#cccccc", lw=0.6, zorder=1)

        # 各ログの軌跡（bbox内の点のみ）
        for log_label, d in data.items():
            df = d["kinematic"]
            mask = (
                (df["x"] >= x_min) & (df["x"] <= x_max) &
                (df["y"] >= y_min) & (df["y"] <= y_max)
            )
            seg = df[mask]
            if seg.empty:
                continue
            _traj_plot(ax, seg, d, log_label, markevery=8)

        ax.set_xlim(x_min, x_max)
        ax.set_ylim(y_min, y_max)
        ax.set_aspect("equal")
        ax.set_title(curve["label"], fontsize=10)
        ax.set_xlabel("x [m]")
        if col == 0:
            ax.set_ylabel("y [m]")
        ax.grid(True, lw=0.5, alpha=0.5)

        # 凡例は最初のサブプロットのみ
        if col == 0:
            ax.legend(fontsize=9, loc="best")

    fig.tight_layout()
    _save(fig, "curves_closeup")


# ---------------------------------------------------------------------------
# カーブ②発進分析
# ---------------------------------------------------------------------------

def _find_curve2_launch(df_vel: pd.DataFrame) -> float | None:
    """カーブ②直前の一時停止から発進する時刻 [s] を返す。
    停止区間を列挙し、カーブ②が含まれる t=20〜120s の範囲で
    最後に終わる停止の終端時刻を採用。
    """
    stopped = df_vel[df_vel["lon_vel"] < 0.05]["t"].values
    if len(stopped) == 0:
        return None
    gaps = np.where(np.diff(stopped) > 2.0)[0]
    starts_idx = np.concatenate([[0], gaps + 1])
    ends_idx   = np.concatenate([gaps, [len(stopped) - 1]])
    candidates = []
    for s, e in zip(starts_idx, ends_idx):
        dur = stopped[e] - stopped[s]
        t_end = stopped[e]
        if dur >= 0.5 and 20.0 <= t_end <= 120.0:
            candidates.append(t_end)
    if not candidates:
        return None
    # カーブ②に最も近い（最初の）発進を採用
    return float(min(candidates))


def plot_curve2_analysis(data: dict, map_ways: list | None):
    """カーブ②（左折）の一時停止発進からの挙動を軌跡＋時系列で比較。

    レイアウト:
        上段(高め): カーブ②付近の軌跡図
        下段3列: 速度 | 加速度 | ステアリング
    """
    # ---- 各ログの発進t=0を検出 ----
    launch_t: dict[str, float] = {}
    for label, d in data.items():
        t_launch = _find_curve2_launch(d["velocity"])
        if t_launch is None:
            warnings.warn(f"{label}: カーブ②前の停止が見つからないためスキップ")
            continue
        launch_t[label] = t_launch
        print(f"  [{label}] カーブ②発進 t={t_launch:.1f}s")

    if not launch_t:
        warnings.warn("発進時刻を検出できるログがないため plot_curve2_analysis をスキップ")
        return

    # ---- 図のセットアップ（2行: 上=軌跡, 下=時系列3列）----
    fig = plt.figure(figsize=(16, 13))
    fig.suptitle(f"{SCENARIO_NAME}\nカーブ②（左折）一時停止発進からの挙動比較", fontsize=12)
    gs = fig.add_gridspec(2, 3, height_ratios=[1.6, 1.0], hspace=0.38, wspace=0.32)
    ax_map  = fig.add_subplot(gs[0, :])   # 上段全幅: 軌跡
    ax_vel  = fig.add_subplot(gs[1, 0])   # 下段左: 速度
    ax_acc  = fig.add_subplot(gs[1, 1])   # 下段中: 加速度
    ax_str  = fig.add_subplot(gs[1, 2])   # 下段右: ステアリング

    # 表示時間範囲（発進前2s ～ 発進後25s）
    T_PRE, T_POST = -2.0, 25.0

    # カーブ②表示範囲
    c2 = CURVE_CENTERS[1]
    cx, cy, mg = c2["cx"], c2["cy"], 80

    # ---- 軌跡図 ----
    if map_ways:
        for pts in map_ways:
            wx, wy = pts[:, 0], pts[:, 1]
            if wx.max() < cx - mg or wx.min() > cx + mg: continue
            if wy.max() < cy - mg or wy.min() > cy + mg: continue
            ax_map.plot(wx, wy, color="#cccccc", lw=0.5, zorder=1)

    for label, d in data.items():
        if label not in launch_t:
            continue
        t_l = launch_t[label]
        df_k = d["kinematic"]
        # 発進前2s〜発進後25s の軌跡
        seg = df_k[(df_k["t"] >= t_l + T_PRE) & (df_k["t"] <= t_l + T_POST)]
        if seg.empty:
            continue
        _traj_plot(ax_map, seg, d, label, markevery=8)
        # 発進点マーカー（★）
        launch_row = df_k.iloc[(df_k["t"] - t_l).abs().argsort().iloc[0]]
        ax_map.plot(launch_row["x"], launch_row["y"], "*",
                    color=d["color"], ms=14, zorder=6,
                    markeredgecolor="white", markeredgewidth=0.5)

    ax_map.set_xlim(cx - mg, cx + mg)
    ax_map.set_ylim(cy - mg, cy + mg)
    ax_map.set_aspect("equal")
    ax_map.set_xlabel("x [m]")
    ax_map.set_ylabel("y [m]")
    ax_map.grid(True, lw=0.5, alpha=0.5)
    ax_map.set_title("軌跡（★=発進点、表示範囲: 発進前2s〜発進後25s）", fontsize=10)
    ax_map.legend(fontsize=10, loc="best")

    # ---- 時系列プロット（発進をt=0に揃える）----
    for label, d in data.items():
        if label not in launch_t:
            continue
        t_l = launch_t[label]

        def clip(df, col):
            mask = (df["t"] >= t_l + T_PRE) & (df["t"] <= t_l + T_POST)
            sub = df[mask].copy()
            sub["tr"] = sub["t"] - t_l   # 相対時刻
            return sub

        vel = clip(d["velocity"], "lon_vel")
        acc = clip(d["accel"],    "accel")
        steer = clip(d["steering"], "steer")
        cmd = clip(d["cmd"],      "cmd_vel")

        # 速度
        _ts_plot(ax_vel, vel["tr"], vel["lon_vel"], d, label)
        if not cmd.empty:
            _ts_plot(ax_vel, cmd["tr"], cmd["cmd_vel"], d, label + "（指令）", cmd=True)

        # 加速度
        _ts_plot(ax_acc, acc["tr"], acc["accel"], d, label)
        if not cmd.empty:
            _ts_plot(ax_acc, cmd["tr"], cmd["cmd_accel"], d, label + "（指令）", cmd=True)

        # ステアリング
        _ts_plot(ax_str, steer["tr"], np.degrees(steer["steer"]), d, label)
        if not cmd.empty:
            _ts_plot(ax_str, cmd["tr"], np.degrees(cmd["cmd_steer"]), d, label + "（指令）", cmd=True)

    # t=0 縦線
    for ax in (ax_vel, ax_acc, ax_str):
        ax.axvline(0, color="gray", lw=1.0, ls="--", alpha=0.7)

    ax_vel.set_title("速度", fontsize=10)
    ax_vel.set_ylabel("m/s")
    ax_vel.set_xlabel("発進からの時刻 [s]")
    ax_vel.grid(True, lw=0.5)
    ax_vel.legend(fontsize=7, loc="best")

    ax_acc.set_title("加速度", fontsize=10)
    ax_acc.set_ylabel("m/s²")
    ax_acc.set_xlabel("発進からの時刻 [s]")
    ax_acc.grid(True, lw=0.5)
    ax_acc.legend(fontsize=7, loc="best")

    ax_str.set_title("ステアリング角", fontsize=10)
    ax_str.set_ylabel("deg")
    ax_str.set_xlabel("発進からの時刻 [s]")
    ax_str.grid(True, lw=0.5)
    ax_str.legend(fontsize=7, loc="best")

    _save(fig, "curve2_analysis")


def plot_curve2_steering_detail(data: dict, map_ways: list | None):
    """カーブ②の一時停止発進からのステアリング詳細分析。

    レイアウト（2列構成）:
        左列 上段: 軌跡（カーブ②付近）
        左列 下段: 指令 vs 応答 の重ね描き（同一軸）
        右列 上段: ステアリング角速度 [deg/s]
        右列 中段: 指令追従誤差（応答 − 指令）[deg]
        右列 下段: ステアリング角 累積絶対値（操舵量の積分）
    """
    # 発進時刻を検出
    launch_t: dict[str, float] = {}
    for label, d in data.items():
        t_l = _find_curve2_launch(d["velocity"])
        if t_l is not None:
            launch_t[label] = t_l

    if not launch_t:
        warnings.warn("発進時刻を検出できないため plot_curve2_steering_detail をスキップ")
        return

    T_PRE, T_POST = -2.0, 25.0

    fig = plt.figure(figsize=(16, 14))
    fig.suptitle(f"{SCENARIO_NAME}\nカーブ②（左折）ステアリング詳細分析　─　一時停止発進 t=0", fontsize=12)
    gs = fig.add_gridspec(3, 2, hspace=0.45, wspace=0.30,
                          height_ratios=[1.4, 1.0, 1.0])
    ax_map   = fig.add_subplot(gs[0, 0])   # 左列上: 軌跡
    ax_ovl   = fig.add_subplot(gs[1:, 0])  # 左列下: 指令 vs 応答（大きく）
    ax_rate  = fig.add_subplot(gs[0, 1])   # 右列上: 角速度
    ax_err   = fig.add_subplot(gs[1, 1])   # 右列中: 追従誤差
    ax_integ = fig.add_subplot(gs[2, 1])   # 右列下: 累積操舵量

    # カーブ②付近の軌跡
    c2 = CURVE_CENTERS[1]
    cx, cy, mg = c2["cx"], c2["cy"], 80
    if map_ways:
        for pts in map_ways:
            wx, wy = pts[:, 0], pts[:, 1]
            if wx.max() < cx - mg or wx.min() > cx + mg: continue
            if wy.max() < cy - mg or wy.min() > cy + mg: continue
            ax_map.plot(wx, wy, color="#cccccc", lw=0.5, zorder=1)

    for label, d in data.items():
        if label not in launch_t:
            continue
        t_l = launch_t[label]
        df_k = d["kinematic"]
        seg = df_k[(df_k["t"] >= t_l + T_PRE) & (df_k["t"] <= t_l + T_POST)]
        if not seg.empty:
            _traj_plot(ax_map, seg, d, label, markevery=8)
        lr = df_k.iloc[(df_k["t"] - t_l).abs().argsort().iloc[0]]
        ax_map.plot(lr["x"], lr["y"], "*", color=d["color"], ms=13,
                    zorder=6, markeredgecolor="white", markeredgewidth=0.5)

    ax_map.set_xlim(cx - mg, cx + mg)
    ax_map.set_ylim(cy - mg, cy + mg)
    ax_map.set_aspect("equal")
    ax_map.set_xlabel("x [m]"); ax_map.set_ylabel("y [m]")
    ax_map.grid(True, lw=0.5, alpha=0.5)
    ax_map.legend(fontsize=9, loc="best")
    ax_map.set_title("軌跡（★=発進点）", fontsize=10)

    # 各ログの時系列ステアリング処理
    for label, d in data.items():
        if label not in launch_t:
            continue
        t_l = launch_t[label]
        steer_df = d["steering"]
        cmd_df   = d["cmd"]

        mask_s = (steer_df["t"] >= t_l + T_PRE) & (steer_df["t"] <= t_l + T_POST)
        mask_c = (cmd_df["t"]   >= t_l + T_PRE) & (cmd_df["t"]   <= t_l + T_POST)
        s = steer_df[mask_s].copy(); s["tr"] = s["t"] - t_l
        c = cmd_df[mask_c].copy();   c["tr"] = c["t"] - t_l

        s_deg = np.degrees(s["steer"].values)
        t_s   = s["tr"].values

        # --- 左列下: 指令 vs 応答の重ね描き ---
        ax_ovl.plot(t_s, s_deg, color=d["color"], lw=d["lw"], ls=d["ls"],
                    label=f"{label} 応答")
        if not c.empty:
            ax_ovl.plot(np.asarray(c["tr"]), np.degrees(np.asarray(c["cmd_steer"])),
                        color=d["color"], lw=1.2, ls=":", alpha=0.65,
                        label=f"{label} 指令")

        # --- 右列上: 角速度 (deg/s) ---
        if len(t_s) > 1:
            dt = np.diff(t_s)
            rate = np.diff(s_deg) / np.where(dt > 1e-6, dt, np.nan)
            # 外れ値を除去
            rate = np.where(np.abs(rate) < 200, rate, np.nan)
            ax_rate.plot(t_s[1:], rate, color=d["color"], lw=d["lw"] * 0.9,
                         ls=d["ls"], label=label)

        # --- 右列中: 追従誤差（応答 − 指令）---
        if not c.empty and len(t_s) > 0:
            cmd_interp = np.interp(t_s, c["tr"].values, np.degrees(c["cmd_steer"].values))
            err = s_deg - cmd_interp
            ax_err.plot(t_s, err, color=d["color"], lw=d["lw"] * 0.9,
                        ls=d["ls"], label=label)

        # --- 右列下: 累積絶対操舵量 ---
        if len(t_s) > 1:
            dt_full = np.diff(t_s)
            incr = np.abs(np.diff(s_deg))
            cumsum = np.concatenate([[0], np.cumsum(incr * dt_full)])
            ax_integ.plot(t_s, cumsum, color=d["color"], lw=d["lw"],
                          ls=d["ls"], label=label)

    # t=0 縦線
    for ax in (ax_ovl, ax_rate, ax_err, ax_integ):
        ax.axvline(0, color="gray", lw=1.0, ls="--", alpha=0.7)

    ax_ovl.set_title("ステアリング角　指令（点線）vs 応答（実線/破線）", fontsize=10)
    ax_ovl.set_ylabel("ステア角 [deg]"); ax_ovl.set_xlabel("発進からの時刻 [s]")
    ax_ovl.grid(True, lw=0.5); ax_ovl.legend(fontsize=8, ncol=2)

    ax_rate.set_title("ステアリング角速度", fontsize=10)
    ax_rate.set_ylabel("deg/s"); ax_rate.set_xlabel("発進からの時刻 [s]")
    ax_rate.axhline(0, color="gray", lw=0.6, ls="-")
    ax_rate.grid(True, lw=0.5); ax_rate.legend(fontsize=9)

    ax_err.set_title("指令追従誤差（応答 − 指令）", fontsize=10)
    ax_err.set_ylabel("誤差 [deg]"); ax_err.set_xlabel("発進からの時刻 [s]")
    ax_err.axhline(0, color="gray", lw=0.6, ls="-")
    ax_err.grid(True, lw=0.5); ax_err.legend(fontsize=9)

    ax_integ.set_title("累積絶対操舵量（∫|dθ/dt|dt）", fontsize=10)
    ax_integ.set_ylabel("deg"); ax_integ.set_xlabel("発進からの時刻 [s]")
    ax_integ.grid(True, lw=0.5); ax_integ.legend(fontsize=9)

    _save(fig, "curve2_steering_detail")


def plot_curve2_yaw_steer(data: dict):
    """ステア角と実際に進んだ方向（ヨーレート換算）の差分を可視化。

    自転車モデル: yaw_rate_pred = v * tan(steer) / L
    等価ステア角:  steer_equiv  = atan(L * yaw_rate_actual / v)

    レイアウト（縦4段、共通t軸）:
        段1: ステア角（実測）vs 等価ステア角（実ヨーレートから逆算） [deg]
        段2: ステア角 − 等価ステア角 = 「進行方向とのズレ」[deg]
        段3: 実ヨーレート vs 自転車モデル予測ヨーレート [deg/s]
        段4: yaw角の累積変化量（カーブの深さ比較）[deg]
    """
    launch_t: dict[str, float] = {}
    for label, d in data.items():
        t_l = _find_curve2_launch(d["velocity"])
        if t_l is not None:
            launch_t[label] = t_l

    if not launch_t:
        return

    T_PRE, T_POST = -2.0, 25.0

    fig, axes = plt.subplots(4, 1, figsize=(13, 16), sharex=True)
    fig.suptitle(
        f"{SCENARIO_NAME}\n"
        f"カーブ②　ステア角 vs 実際の進行方向（自転車モデル L={WHEELBASE:.2f}m）",
        fontsize=12,
    )

    for label, d in data.items():
        if label not in launch_t:
            continue
        t_l = launch_t[label]

        # ---- 時系列データを発進t=0で切り出し ----
        def clip(df):
            m = (df["t"] >= t_l + T_PRE) & (df["t"] <= t_l + T_POST)
            sub = df[m].copy()
            sub["tr"] = sub["t"] - t_l
            return sub

        kin   = clip(d["kinematic"])
        vel   = clip(d["velocity"])
        steer = clip(d["steering"])

        if kin.empty or vel.empty or steer.empty:
            continue

        t_k = kin["tr"].values

        # yaw unwrap → yaw_rate (rad/s)
        yaw_u    = np.unwrap(kin["yaw"].values)
        yaw_rate = np.gradient(yaw_u, t_k)          # rad/s

        # 速度・ステアを kinematic の時刻に補間
        v_i = np.interp(t_k, vel["tr"].values, vel["lon_vel"].values)
        s_i = np.interp(t_k, steer["tr"].values, steer["steer"].values)  # rad

        # 等価ステア角: atan(L * yaw_rate / v)  [rad]
        # v が小さいと不安定 → 0.3m/s 以下はマスク
        v_safe = np.where(v_i > 0.3, v_i, np.nan)
        steer_equiv = np.arctan2(WHEELBASE * yaw_rate, v_safe)  # rad

        # 予測ヨーレート: v * tan(steer) / L  [rad/s]
        yaw_rate_pred = v_i * np.tan(s_i) / WHEELBASE

        # 差分: ステア角 − 等価ステア角  [deg]
        diff_deg = np.degrees(s_i - steer_equiv)
        # 外れ値（v≈0 由来）をクリップ
        diff_deg = np.clip(diff_deg, -30, 30)

        # yaw累積変化量（t=0の値を基準）
        t0_idx = int(np.argmin(np.abs(t_k)))
        yaw_cum = np.degrees(yaw_u - yaw_u[t0_idx])

        # ---- 描画 ----
        kw = dict(color=d["color"], lw=d["lw"], ls=d["ls"])

        # 段1: ステア角 vs 等価ステア角
        axes[0].plot(t_k, np.degrees(s_i), label=f"{label} ステア実測", **kw)
        axes[0].plot(t_k, np.degrees(steer_equiv),
                     color=d["color"], lw=1.2, ls=":", alpha=0.7,
                     label=f"{label} 等価（実ヨーレートから逆算）")

        # 段2: 差分
        axes[1].plot(t_k, diff_deg, label=label, **kw)

        # 段3: 実ヨーレート vs 予測ヨーレート
        axes[2].plot(t_k, np.degrees(yaw_rate),
                     label=f"{label} 実測", **kw)
        axes[2].plot(t_k, np.degrees(yaw_rate_pred),
                     color=d["color"], lw=1.2, ls=":", alpha=0.7,
                     label=f"{label} 予測（自転車モデル）")

        # 段4: yaw累積変化
        axes[3].plot(t_k, yaw_cum, label=label, **kw)

    # 共通装飾
    for ax in axes:
        ax.axvline(0, color="gray", lw=1.0, ls="--", alpha=0.7)
        ax.axhline(0, color="gray", lw=0.5)
        ax.grid(True, lw=0.5)
        ax.legend(fontsize=8, ncol=2, loc="best")

    axes[0].set_title("ステア角（実線）vs 等価ステア角（点線）── 一致するほど自転車モデルが成立", fontsize=10)
    axes[0].set_ylabel("deg")

    axes[1].set_title(f"ステア角 − 等価ステア角（進行方向とのズレ）  ─  L={WHEELBASE:.2f}m", fontsize=10)
    axes[1].set_ylabel("deg")

    axes[2].set_title("ヨーレート（実線=実測、点線=自転車モデル予測）", fontsize=10)
    axes[2].set_ylabel("deg/s")

    axes[3].set_title("yaw 累積変化量（t=0 基準）── カーブの総旋回量", fontsize=10)
    axes[3].set_ylabel("deg")
    axes[3].set_xlabel("発進からの時刻 [s]")

    fig.tight_layout()
    _save(fig, "curve2_yaw_steer")


def plot_steer_response(data: dict):
    """ステアリング応答性能の比較図。

    ステア指令が 1deg を超えた瞬間を各ログの t=0 に揃え、
    指令 vs 応答の「追従の遅れ・形の違い」を正規化して一目でわかるようにする。

    レイアウト:
        上段(大): 正規化ステア角（指令 vs 応答）— ピーク指令値で正規化して
                  指令波形の形が揃った状態で応答のズレを比較
        中段左: 追従誤差（応答 − 指令）の時系列
        中段右: 立ち上がり時間（指令ピーク90%到達）の棒グラフ
        下段左: RMSE [deg]の棒グラフ
        下段右: ピーク追従率（応答ピーク / 指令ピーク）の棒グラフ
    """
    from scipy.signal import correlate

    T_ONSET_PRE  = -0.5   # onset 前の余白 [s]
    T_ONSET_POST = 12.0   # onset 後の表示幅 [s]
    ONSET_THRESH_DEG = 1.0

    fig = plt.figure(figsize=(15, 14))
    fig.suptitle(
        f"{SCENARIO_NAME}\n"
        "ステアリング応答性能比較　──　ステア入力開始 t=0 に揃えた比較",
        fontsize=12,
    )
    gs = fig.add_gridspec(3, 2, height_ratios=[1.6, 1.0, 1.0],
                          hspace=0.50, wspace=0.30)
    ax_main = fig.add_subplot(gs[0, :])
    ax_err  = fig.add_subplot(gs[1, 0])
    ax_rise = fig.add_subplot(gs[1, 1])
    ax_rmse = fig.add_subplot(gs[2, 0])
    ax_peak = fig.add_subplot(gs[2, 1])

    # 定量値を収集
    labels_list   = []
    onset_delays  = []   # 指令onset → 応答onset の遅延 [s]
    delays        = []   # xcorr ラグ [s]
    rmse_list     = []
    rise_list     = []
    peak_ratio    = []

    for label, d in data.items():
        t_l = _find_curve2_launch(d["velocity"])
        if t_l is None:
            continue

        vel_df   = d["velocity"]
        steer_df = d["steering"]
        cmd_df   = d["cmd"]

        def clip_rel(df, t_l=t_l):
            m = (df["t"] >= t_l - 3) & (df["t"] <= t_l + 20)
            sub = df[m].copy()
            sub["tr"] = sub["t"] - t_l   # 発進基準の相対時刻
            return sub

        steer_r = clip_rel(steer_df)
        cmd_r   = clip_rel(cmd_df)
        if steer_r.empty or cmd_r.empty:
            continue

        cmd_deg   = np.degrees(cmd_r["cmd_steer"].values)
        steer_deg = np.degrees(steer_r["steer"].values)

        # ステア指令の onset（発進後で閾値を超えた最初の時刻）
        onset_mask = (cmd_r["tr"] >= 0) & (np.abs(cmd_deg) > ONSET_THRESH_DEG)
        if not onset_mask.any():
            continue
        t_onset_cmd = float(cmd_r["tr"].values[onset_mask][0])

        onset_mask_act = (steer_r["tr"] >= 0) & (np.abs(steer_deg) > ONSET_THRESH_DEG)
        t_onset_act = float(steer_r["tr"].values[onset_mask_act][0]) if onset_mask_act.any() else t_onset_cmd

        onset_delay = t_onset_act - t_onset_cmd

        # onset t=0 で揃えた窓を切り出す
        t_c = cmd_r["tr"].values   - t_onset_cmd
        t_a = steer_r["tr"].values - t_onset_cmd

        win_mask_c = (t_c >= T_ONSET_PRE) & (t_c <= T_ONSET_POST)
        win_mask_a = (t_a >= T_ONSET_PRE) & (t_a <= T_ONSET_POST)
        t_c_w, c_w = t_c[win_mask_c], cmd_deg[win_mask_c]
        t_a_w, a_w = t_a[win_mask_a], steer_deg[win_mask_a]

        if len(c_w) == 0 or len(a_w) == 0:
            continue

        # 正規化（ピーク指令値）
        peak_cmd = np.abs(c_w).max()
        peak_act = np.abs(a_w).max()
        c_norm = c_w / peak_cmd if peak_cmd > 0 else c_w
        a_norm = a_w / peak_cmd if peak_cmd > 0 else a_w

        # RMSE（共通グリッドで計算）
        dt = 0.02
        t_grid = np.arange(T_ONSET_PRE, T_ONSET_POST, dt)
        c_i = np.interp(t_grid, t_c_w, c_w, left=np.nan, right=np.nan)
        a_i = np.interp(t_grid, t_a_w, a_w, left=np.nan, right=np.nan)
        valid = ~(np.isnan(c_i) | np.isnan(a_i))
        rmse = float(np.sqrt(np.mean((a_i[valid] - c_i[valid])**2))) if valid.any() else np.nan

        # xcorr による遅延推定
        if valid.sum() > 10:
            c_v, a_v = c_i[valid] - np.nanmean(c_i), a_i[valid] - np.nanmean(a_i)
            corr = correlate(a_v, c_v, mode="full")
            lag = (corr.argmax() - (len(c_v) - 1)) * dt
        else:
            lag = onset_delay

        # 立ち上がり時間（指令ピーク90% への到達）
        thresh90 = 0.90 * peak_cmd
        rise_mask = (t_a_w >= 0) & (np.abs(a_w) >= thresh90)
        rise_time = float(t_a_w[rise_mask][0]) if rise_mask.any() else np.nan

        # ---- メインプロット（正規化）----
        kw = dict(color=d["color"], lw=d["lw"], ls=d["ls"])
        ax_main.plot(t_c_w, c_norm, label=f"{label} 指令",
                     **kw, alpha=0.55, marker=None)
        ax_main.plot(t_a_w, a_norm, label=f"{label} 応答",
                     color=d["color"], lw=d["lw"]+0.5, ls=d["ls"],
                     marker=d["marker"], markersize=d["ms"], markevery=12,
                     markerfacecolor=d["color"], markeredgecolor="white", markeredgewidth=0.5)

        # 応答ピークに達した時刻をマーカー（点線）
        if rise_mask.any():
            tr_rise = t_a_w[rise_mask][0]
            ax_main.axvline(tr_rise, color=d["color"], lw=1.2, ls=":", alpha=0.6)

        # 応答 onset の縦線（指令 onset t=0 からの遅延を鎖線で表示）
        ax_main.axvline(onset_delay, color=d["color"], lw=2.0, ls="-.",
                        alpha=0.9)

        # ---- 追従誤差 ----
        err_i = np.interp(t_grid, t_a_w, a_w, left=np.nan, right=np.nan) - \
                np.interp(t_grid, t_c_w, c_w, left=np.nan, right=np.nan)
        ax_err.plot(t_grid, err_i, color=d["color"], lw=d["lw"], ls=d["ls"], label=label)

        # 収集
        labels_list.append(label)
        onset_delays.append(onset_delay)
        delays.append(lag)
        rmse_list.append(rmse)
        rise_list.append(rise_time)
        peak_ratio.append(peak_act / peak_cmd if peak_cmd > 0 else np.nan)

    # 遅延の ←→ アノテーション（ax_main の上端付近に配置）
    y_base = 1.10
    for i, (lbl, delay_s) in enumerate(zip(labels_list, onset_delays)):
        d_cfg = data[lbl]
        yp = y_base + i * 0.09
        x_end = max(delay_s, 0.015)   # 非常に小さい場合も矢印が見えるよう最低幅を確保
        ax_main.annotate(
            "",
            xy=(x_end, yp),
            xytext=(0.0, yp),
            xycoords="data",
            textcoords="data",
            arrowprops=dict(arrowstyle="<->", color=d_cfg["color"],
                            lw=1.8, mutation_scale=10),
            annotation_clip=False,
        )
        ax_main.text(x_end / 2, yp + 0.035,
                     f"{lbl}: {delay_s * 1000:.0f} ms",
                     ha="center", va="bottom", fontsize=8,
                     color=d_cfg["color"], fontweight="bold",
                     clip_on=False)

    # 装飾
    ax_main.axvline(0, color="gray", lw=1.2, ls="--", alpha=0.7, label="入力onset t=0")
    ax_main.axhline(0, color="gray", lw=0.5)
    ax_main.axhline(1.0, color="gray", lw=0.5, ls=":")
    ax_main.axhline(0.9, color="gray", lw=0.5, ls=":", alpha=0.5)
    ax_main.set_ylim(-0.15, 1.45)  # 上部に矢印アノテーション用の余白を確保
    ax_main.set_title(
        "正規化ステア角（ピーク指令=1.0）　実線=応答、薄線=指令\n"
        "縦点線=応答90%到達、縦鎖線=応答onset　↔矢印=応答遅延",
        fontsize=10,
    )
    ax_main.set_ylabel("正規化ステア角 [−]")
    ax_main.set_xlabel("入力開始からの時刻 [s]")
    ax_main.grid(True, lw=0.5)
    ax_main.legend(fontsize=8, ncol=3, loc="lower right")

    ax_err.axvline(0, color="gray", lw=1.0, ls="--", alpha=0.7)
    ax_err.axhline(0, color="gray", lw=0.5)
    ax_err.set_title("追従誤差（応答 − 指令）", fontsize=10)
    ax_err.set_ylabel("deg"); ax_err.set_xlabel("入力開始からの時刻 [s]")
    ax_err.grid(True, lw=0.5); ax_err.legend(fontsize=9)

    # 棒グラフ
    colors_bar = [data[l]["color"] for l in labels_list if l in data]
    x = np.arange(len(labels_list))
    bw = 0.5

    delay_ms = [d * 1000 if d is not None and not np.isnan(d) else 0 for d in onset_delays]
    ax_rise.bar(x, delay_ms, width=bw, color=colors_bar, alpha=0.85, edgecolor="white")
    ax_rise.set_xticks(x); ax_rise.set_xticklabels(labels_list, fontsize=9)
    ax_rise.set_title("応答遅延（指令onset → 応答onset）", fontsize=10)
    ax_rise.set_ylabel("ms"); ax_rise.grid(True, axis="y", lw=0.5)
    for xi, v_ms in zip(x, delay_ms):
        ax_rise.text(xi, v_ms + 1, f"{v_ms:.0f} ms", ha="center", fontsize=9, fontweight="bold")

    ax_rmse.bar(x, [r if r is not None and not np.isnan(r) else 0 for r in rmse_list],
                width=bw, color=colors_bar, alpha=0.85, edgecolor="white")
    ax_rmse.set_xticks(x); ax_rmse.set_xticklabels(labels_list, fontsize=9)
    ax_rmse.set_title("RMSE（指令追従誤差）", fontsize=10)
    ax_rmse.set_ylabel("deg"); ax_rmse.grid(True, axis="y", lw=0.5)
    for xi, v in zip(x, rmse_list):
        if v is not None and not np.isnan(v):
            ax_rmse.text(xi, v + 0.02, f"{v:.3f}°", ha="center", fontsize=9)

    ax_peak.bar(x, [r if r is not None and not np.isnan(r) else 0 for r in peak_ratio],
                width=bw, color=colors_bar, alpha=0.85, edgecolor="white")
    ax_peak.axhline(1.0, color="gray", lw=1.0, ls="--", alpha=0.6)
    ax_peak.set_xticks(x); ax_peak.set_xticklabels(labels_list, fontsize=9)
    ax_peak.set_title("ピーク追従率（応答ピーク / 指令ピーク）", fontsize=10)
    ax_peak.set_ylabel("−"); ax_peak.set_ylim(0, 1.3); ax_peak.grid(True, axis="y", lw=0.5)
    for xi, v in zip(x, peak_ratio):
        if v is not None and not np.isnan(v):
            ax_peak.text(xi, v + 0.02, f"{v:.3f}", ha="center", fontsize=9)

    fig.tight_layout()
    _save(fig, "steer_response")


# ---------------------------------------------------------------------------
# 数値レポート
# ---------------------------------------------------------------------------

def nearest_point_distance(ref_xy: np.ndarray, query_xy: np.ndarray) -> np.ndarray:
    """各query点に対するref_xyの最近傍距離を返す。"""
    from scipy.spatial import cKDTree
    tree = cKDTree(ref_xy)
    dists, _ = tree.query(query_xy)
    return dists


def build_report(data: dict) -> str:
    lines = [f"# 比較レポート\n\nシナリオ: {SCENARIO_NAME}\n"]

    # 完走時間
    lines.append("## 完走時間（AUTONOMOUS 開始〜停止）\n")
    for label, d in data.items():
        vel = d["velocity"]
        duration = vel["t"].max() - vel["t"].min()
        lines.append(f"- **{label}**: {duration:.1f} s")
    lines.append("")

    # 速度統計
    lines.append("## 速度統計（VelocityReport.longitudinal_velocity）\n")
    lines.append("| ログ | 平均 [m/s] | 最大 [m/s] | 標準偏差 |")
    lines.append("|---|---|---|---|")
    for label, d in data.items():
        v = d["velocity"]["lon_vel"]
        lines.append(f"| {label} | {v.mean():.3f} | {v.max():.3f} | {v.std():.3f} |")
    lines.append("")

    # 速度指令 RMSE（指令 vs 応答）
    lines.append("## 速度 RMSE（指令 vs 応答）\n")
    lines.append("| ログ | RMSE [m/s] |")
    lines.append("|---|---|")
    for label, d in data.items():
        cmd = d["cmd"]
        vel = d["velocity"]
        if cmd.empty or vel.empty:
            lines.append(f"| {label} | N/A |")
            continue
        cmd_i = np.interp(vel["t"], cmd["t"], cmd["cmd_vel"])
        rmse = np.sqrt(np.mean((vel["lon_vel"].values - cmd_i) ** 2))
        lines.append(f"| {label} | {rmse:.4f} |")
    lines.append("")

    # ステア RMSE（指令 vs 応答）
    lines.append("## ステアリング RMSE（指令 vs 応答）\n")
    lines.append("| ログ | RMSE [deg] |")
    lines.append("|---|---|")
    for label, d in data.items():
        cmd = d["cmd"]
        steer = d["steering"]
        if cmd.empty or steer.empty:
            lines.append(f"| {label} | N/A |")
            continue
        cmd_i = np.interp(steer["t"], cmd["t"], cmd["cmd_steer"])
        rmse = np.degrees(np.sqrt(np.mean((steer["steer"].values - cmd_i) ** 2)))
        lines.append(f"| {label} | {rmse:.4f} |")
    lines.append("")

    # 軌跡乖離（実機を基準、実機の空間的 bounding box 内のシム点のみで計算）
    lines.append("## 軌跡乖離（実機との最近傍距離）\n")
    lines.append("> 実機ログは経路の後半区間のみ記録されているため、  \n")
    lines.append("> 実機の bounding box 内に含まれるシム軌跡点のみを対象に計算しています。\n")
    lines.append("")
    lines.append("| ログ | 平均 [m] | 最大 [m] | 対象点数 |")
    lines.append("|---|---|---|---|")
    ref_kin = data["実機"]["kinematic"]
    ref_xy = ref_kin[["x", "y"]].values
    x_min, x_max = ref_xy[:, 0].min(), ref_xy[:, 0].max()
    y_min, y_max = ref_xy[:, 1].min(), ref_xy[:, 1].max()
    # bbox に若干のマージンを追加
    margin = 50
    for label, d in data.items():
        if label == "実機":
            lines.append(f"| {label} | — (基準) | — | {len(ref_xy)} |")
            continue
        try:
            from scipy.spatial import cKDTree
            q_df = d["kinematic"]
            # 実機 bbox 内の点のみ
            mask = (
                (q_df["x"] >= x_min - margin) & (q_df["x"] <= x_max + margin) &
                (q_df["y"] >= y_min - margin) & (q_df["y"] <= y_max + margin)
            )
            q_xy = q_df.loc[mask, ["x", "y"]].values
            if len(q_xy) == 0:
                lines.append(f"| {label} | N/A (bbox 外) | — | 0 |")
                continue
            dists = nearest_point_distance(ref_xy, q_xy)
            lines.append(f"| {label} | {dists.mean():.3f} | {dists.max():.3f} | {len(q_xy)} |")
        except Exception as e:
            lines.append(f"| {label} | ERR: {e} | — | — |")
    lines.append("")

    return "\n".join(lines)


# ---------------------------------------------------------------------------
# メイン
# ---------------------------------------------------------------------------

def main():
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    FIGS_DIR.mkdir(parents=True, exist_ok=True)

    print("=== データ読み込み中 ===")
    loaded = {}
    for label, cfg in LOGS.items():
        mcap_path: Path = cfg["path"]
        print(f"  [{label}] {mcap_path.name}")

        df_mode = load_operation_mode(mcap_path)
        df_vel = load_velocity(mcap_path)
        t0 = find_autonomous_start(df_mode, df_vel)
        print(f"    → t0 = {t0} ns (AUTONOMOUS 開始)")

        loaded[label] = {
            "velocity": align_time(df_vel, t0),
            "steering": align_time(load_steering(mcap_path), t0),
            "kinematic": align_time(load_kinematic(mcap_path, cfg["kinematic"]), t0),
            "accel": align_time(load_accel(mcap_path, cfg["accel"]), t0),
            "cmd": align_time(load_cmd(mcap_path, cfg["cmd"]), t0),
            "color": cfg["color"],
            "lw": cfg["lw"],
            "ls": cfg["ls"],
            "marker": cfg["marker"],
            "ms": cfg["ms"],
        }

    # Lanelet2 地図読み込み
    print("\n=== 地図読み込み中 ===")
    map_ways = None
    map_osm = _resolve_map_osm()
    if map_osm is not None:
        try:
            map_ways = load_map_ways(map_osm)
            print(f"  {len(map_ways)} ways をロード ({map_osm})")
        except Exception as e:
            warnings.warn(f"地図ロード失敗: {e}")
    else:
        warnings.warn(f"地図ファイルが見つかりません: {MAP_DIR}/2231-*/lanelet2_map.osm")

    print("\n=== プロット生成中 ===")
    plot_trajectory(loaded, map_ways)
    if not map_ways:
        plot_trajectory(loaded, None)  # フォールバック: 地図なし
    plot_curves(loaded, map_ways)
    plot_curve2_analysis(loaded, map_ways)
    plot_curve2_steering_detail(loaded, map_ways)
    plot_curve2_yaw_steer(loaded)
    plot_steer_response(loaded)
    plot_velocity(loaded)
    plot_acceleration(loaded)
    plot_steering(loaded)

    print("\n=== レポート生成中 ===")
    try:
        report = build_report(loaded)
        report_path = OUT_DIR / "report.md"
        report_path.write_text(report, encoding="utf-8")
        print(f"  保存: {report_path}")
    except Exception as e:
        warnings.warn(f"レポート生成失敗: {e}")

    print("\n完了。出力先:", FIGS_DIR)


if __name__ == "__main__":
    main()
