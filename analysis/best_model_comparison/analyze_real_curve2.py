#!/usr/bin/env python3
"""実機ログのカーブ②詳細解析 — アンダーステア検証。

Usage:
    python3 analyze_real_curve2.py

Output: comparison/figures/real_curve2_detail.{png,pdf}
"""

import math
import warnings
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
matplotlib.rcParams["font.family"] = "Noto Sans CJK JP"
import numpy as np
import pandas as pd
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory
from _params_utils import add_params_annotation

BASE = Path(__file__).parent
LITE_DIR = BASE / "lite"
OUT_DIR = BASE / "comparison"
FIGS_DIR = OUT_DIR / "figures"
MAP_DIR = Path.home() / ".webauto/simulation/data/map/x2_dev/2231"
REAL_MCAP = LITE_DIR / "real.lite.mcap"

CURVE2_CX, CURVE2_CY, CURVE2_MARGIN = 89301, 43085, 60
WHEELBASE = 5.15  # m

# ---- MCAP ローダー ----

def _iter_msgs(mcap_path, topics):
    with open(mcap_path, "rb") as f:
        reader = make_reader(f, decoder_factories=[DecoderFactory()])
        yield from reader.iter_decoded_messages(topics=topics)


def _load_velocity(p):
    rows = []
    for _, _, msg, ros in _iter_msgs(p, ["/vehicle/status/velocity_status"]):
        rows.append({"t_ns": msg.log_time, "lon_vel": ros.longitudinal_velocity})
    return pd.DataFrame(rows)


def _load_steering(p):
    rows = []
    for _, _, msg, ros in _iter_msgs(p, ["/vehicle/status/steering_status"]):
        rows.append({"t_ns": msg.log_time, "steer": ros.steering_tire_angle})
    return pd.DataFrame(rows)


def _load_kinematic(p, topic):
    rows = []
    for _, _, msg, ros in _iter_msgs(p, [topic]):
        o = ros.pose.pose.orientation
        yaw = math.atan2(2*(o.w*o.z + o.x*o.y), 1 - 2*(o.y*o.y + o.z*o.z))
        pos = ros.pose.pose.position
        rows.append({"t_ns": msg.log_time, "x": pos.x, "y": pos.y, "yaw": yaw})
    return pd.DataFrame(rows)


def _load_accel(p, topic):
    rows = []
    for _, _, msg, ros in _iter_msgs(p, [topic]):
        rows.append({"t_ns": msg.log_time, "accel": ros.accel.accel.linear.x})
    return pd.DataFrame(rows)


def _load_cmd(p, topic):
    rows = []
    for _, _, msg, ros in _iter_msgs(p, [topic]):
        rows.append({
            "t_ns": msg.log_time,
            "cmd_vel":   ros.longitudinal.velocity,
            "cmd_accel": ros.longitudinal.acceleration,
            "cmd_steer": ros.lateral.steering_tire_angle,
        })
    return pd.DataFrame(rows)


def _load_opmode(p):
    rows = []
    for _, _, msg, ros in _iter_msgs(p, ["/system/operation_mode/state"]):
        rows.append({"t_ns": msg.log_time, "mode": ros.mode})
    return pd.DataFrame(rows)


def _align(df, t0_ns):
    df = df.copy()
    df["t"] = (df["t_ns"] - t0_ns) / 1e9
    return df[df["t"] >= 0].reset_index(drop=True)


def _find_curve2_launch(df_vel):
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
    return float(min(candidates)) if candidates else None


def _load_map_ways():
    if not MAP_DIR.exists():
        return []
    from lxml import etree
    candidates = sorted(MAP_DIR.glob("2231-*/lanelet2_map.osm"),
                        key=lambda p: p.parent.name, reverse=True)
    if not candidates:
        return []
    tree = etree.parse(str(candidates[0]))
    root = tree.getroot()
    node_xy = {}
    for node in root.findall("node"):
        tags = {t.get("k"): t.get("v") for t in node.findall("tag")}
        if "local_x" in tags and "local_y" in tags:
            node_xy[int(node.get("id"))] = (float(tags["local_x"]), float(tags["local_y"]))
    used = set()
    for rel in root.findall("relation"):
        tags = {t.get("k"): t.get("v") for t in rel.findall("tag")}
        if tags.get("type") == "lanelet":
            for m in rel.findall("member"):
                if m.get("type") == "way":
                    used.add(int(m.get("ref")))
    ways = []
    for way in root.findall("way"):
        wid = int(way.get("id"))
        if wid not in used:
            continue
        refs = [int(nd.get("ref")) for nd in way.findall("nd")]
        pts = [node_xy[r] for r in refs if r in node_xy]
        if len(pts) >= 2:
            ways.append(np.array(pts))
    return ways


# ---- メイン ----

print(f"Loading: {REAL_MCAP}")
df_opmode  = _load_opmode(REAL_MCAP)
df_vel     = _load_velocity(REAL_MCAP)
df_steer   = _load_steering(REAL_MCAP)
df_kin     = _load_kinematic(REAL_MCAP, "/sub/localization/kinematic_state")
df_accel   = _load_accel(REAL_MCAP, "/sub/localization/acceleration")
df_cmd     = _load_cmd(REAL_MCAP, "/sub/control/command/control_cmd")

# 自律走行開始時刻
if not df_opmode.empty:
    t0_ns = int(df_opmode[df_opmode["mode"] == 2]["t_ns"].iloc[0])
else:
    t0_ns = int(df_vel[df_vel["lon_vel"] > 0.1]["t_ns"].iloc[0])

df_vel   = _align(df_vel,   t0_ns)
df_steer = _align(df_steer, t0_ns)
df_kin   = _align(df_kin,   t0_ns)
df_accel = _align(df_accel, t0_ns)
df_cmd   = _align(df_cmd,   t0_ns)

# カーブ② 発進時刻
t_launch = _find_curve2_launch(df_vel)
if t_launch is None:
    print("ERROR: カーブ②前の停止を検出できませんでした", flush=True)
    raise SystemExit(1)
print(f"カーブ②発進 t={t_launch:.1f}s")

T_PRE, T_POST = -3.0, 30.0


def _clip(df, t_launch):
    mask = (df["t"] >= t_launch + T_PRE) & (df["t"] <= t_launch + T_POST)
    sub = df[mask].copy()
    sub["tr"] = sub["t"] - t_launch
    return sub


vel   = _clip(df_vel,   t_launch)
steer = _clip(df_steer, t_launch)
kin   = _clip(df_kin,   t_launch)
accel = _clip(df_accel, t_launch)
cmd   = _clip(df_cmd,   t_launch)

# ---- ステア追従誤差（補間） ----
steer_cmd_interp = np.interp(steer["tr"].values,
                              cmd["tr"].values,
                              np.degrees(cmd["cmd_steer"].values))
steer_resp_deg = np.degrees(steer["steer"].values)
steer_error = steer_resp_deg - steer_cmd_interp  # 応答 − 指令（正=オーバーステア）

# ---- ステア変化率 ----
dt_steer = np.diff(steer["t"].values)
dt_steer = np.where(dt_steer > 0, dt_steer, np.nan)
dsteer_dt = np.diff(steer_resp_deg) / dt_steer      # 応答 steer rate
dcmd_dt_raw = np.diff(np.degrees(cmd["cmd_steer"].values))
dt_cmd = np.diff(cmd["t"].values)
dt_cmd = np.where(dt_cmd > 0, dt_cmd, np.nan)
dcmd_dt = dcmd_dt_raw / dt_cmd                       # 指令 steer rate

# 地図
map_ways = _load_map_ways()
print(f"地図 way 数: {len(map_ways)}")

# ---- プロット ----
fig = plt.figure(figsize=(18, 16))
fig.suptitle("実機 カーブ②詳細解析\n（アンダーステア / オーバーステア検証）", fontsize=12)

gs = fig.add_gridspec(3, 3,
                      height_ratios=[1.5, 1.0, 1.0],
                      hspace=0.45, wspace=0.35)

ax_map    = fig.add_subplot(gs[0, :])    # 上段全幅: 軌跡
ax_steer  = fig.add_subplot(gs[1, 0])   # 中段左: ステア指令 vs 応答
ax_err    = fig.add_subplot(gs[1, 1])   # 中段中: 追従誤差
ax_rate   = fig.add_subplot(gs[1, 2])   # 中段右: ステア変化率
ax_vel    = fig.add_subplot(gs[2, 0])   # 下段左: 速度
ax_acc    = fig.add_subplot(gs[2, 1])   # 下段中: 加速度
ax_cumul  = fig.add_subplot(gs[2, 2])   # 下段右: 累積誤差

# --- 軌跡 ---
if map_ways:
    for pts in map_ways:
        wx, wy = pts[:, 0], pts[:, 1]
        cx, cy, mg = CURVE2_CX, CURVE2_CY, CURVE2_MARGIN
        if wx.max() < cx - mg or wx.min() > cx + mg: continue
        if wy.max() < cy - mg or wy.min() > cy + mg: continue
        ax_map.plot(wx, wy, color="#cccccc", lw=0.5, zorder=1)

ax_map.plot(kin["x"].values, kin["y"].values, color="black", lw=2.5, label="実機 軌跡", zorder=3)
# 時刻に応じた色点（発進後0,5,10,15,20s）
for t_mark in [0, 5, 10, 15, 20, 25]:
    row = kin.iloc[(kin["tr"] - t_mark).abs().argsort().iloc[:1]]
    if row.empty:
        continue
    ax_map.scatter(row["x"], row["y"], c="red" if t_mark == 0 else "gray",
                   s=50, zorder=5)
    ax_map.annotate(f"t={t_mark}s", (row["x"].values[0], row["y"].values[0]),
                    textcoords="offset points", xytext=(4, 4), fontsize=7, color="gray")

ax_map.set_xlim(CURVE2_CX - CURVE2_MARGIN, CURVE2_CX + CURVE2_MARGIN)
ax_map.set_ylim(CURVE2_CY - CURVE2_MARGIN, CURVE2_CY + CURVE2_MARGIN)
ax_map.set_aspect("equal")
ax_map.set_xlabel("x [m]")
ax_map.set_ylabel("y [m]")
ax_map.set_title("実機 カーブ②付近の軌跡（赤★=発進点、t=0〜25s）", fontsize=10)
ax_map.legend(fontsize=9)
ax_map.grid(True, lw=0.4, alpha=0.5)

# --- ステア指令 vs 応答 ---
ax_steer.plot(cmd["tr"].values, np.degrees(cmd["cmd_steer"].values),
              color="tab:blue", lw=1.5, ls="--", label="指令 cmd_steer")
ax_steer.plot(steer["tr"].values, steer_resp_deg,
              color="black", lw=2.0, label="応答 steering_tire_angle")
ax_steer.axvline(0, color="gray", lw=0.8, ls="--")
ax_steer.set_title("ステアリング角: 指令 vs 応答", fontsize=10)
ax_steer.set_xlabel("発進からの時刻 [s]")
ax_steer.set_ylabel("deg")
ax_steer.legend(fontsize=8)
ax_steer.grid(True, lw=0.4, alpha=0.5)

# --- 追従誤差（応答 − 指令） ---
tr_s = steer["tr"].values
ax_err.fill_between(tr_s, steer_error, 0,
                    where=(steer_error >= 0), color="tab:red",  alpha=0.3, label="オーバーステア（+）")
ax_err.fill_between(tr_s, steer_error, 0,
                    where=(steer_error < 0),  color="tab:blue", alpha=0.3, label="アンダーステア（−）")
ax_err.plot(tr_s, steer_error, color="black", lw=1.0)
ax_err.axhline(0, color="gray", lw=0.8)
ax_err.axvline(0, color="gray", lw=0.8, ls="--")
ax_err.set_title("ステア追従誤差 (応答 − 指令)", fontsize=10)
ax_err.set_xlabel("発進からの時刻 [s]")
ax_err.set_ylabel("deg")
ax_err.legend(fontsize=8)
ax_err.grid(True, lw=0.4, alpha=0.5)

rmse_err = float(np.sqrt(np.nanmean(steer_error**2)))
mean_err = float(np.nanmean(steer_error))
ax_err.text(0.02, 0.95, f"RMSE={rmse_err:.3f}°\n平均={mean_err:.3f}°",
            transform=ax_err.transAxes, fontsize=8, va="top",
            bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.7))

# --- ステア変化率 ---
ax_rate.plot(steer["tr"].values[1:], dsteer_dt,
             color="black", lw=1.5, label="応答 steer rate")
ax_rate.plot(cmd["tr"].values[1:], dcmd_dt,
             color="tab:blue", lw=1.2, ls="--", label="指令 steer rate")
ax_rate.axhline(0, color="gray", lw=0.5)
ax_rate.axvline(0, color="gray", lw=0.8, ls="--")
ax_rate.set_title("ステア変化率 [deg/s]", fontsize=10)
ax_rate.set_xlabel("発進からの時刻 [s]")
ax_rate.set_ylabel("deg/s")
ax_rate.legend(fontsize=8)
ax_rate.grid(True, lw=0.4, alpha=0.5)
ax_rate.set_ylim(-30, 30)

# --- 速度 ---
ax_vel.plot(vel["tr"].values, vel["lon_vel"].values,
            color="black", lw=2.0, label="実機 速度")
if not cmd.empty:
    ax_vel.plot(cmd["tr"].values, cmd["cmd_vel"].values,
                color="tab:blue", lw=1.5, ls="--", label="速度指令")
ax_vel.axvline(0, color="gray", lw=0.8, ls="--")
ax_vel.set_title("速度", fontsize=10)
ax_vel.set_xlabel("発進からの時刻 [s]")
ax_vel.set_ylabel("m/s")
ax_vel.legend(fontsize=8)
ax_vel.grid(True, lw=0.4, alpha=0.5)

# --- 加速度 ---
ax_acc.plot(accel["tr"].values, accel["accel"].values,
            color="black", lw=2.0, label="実機 加速度 (IMU)")
if not cmd.empty:
    ax_acc.plot(cmd["tr"].values, cmd["cmd_accel"].values,
                color="tab:blue", lw=1.5, ls="--", label="加速度指令")
ax_acc.axhline(0, color="gray", lw=0.5)
ax_acc.axvline(0, color="gray", lw=0.8, ls="--")
ax_acc.set_title("加速度", fontsize=10)
ax_acc.set_xlabel("発進からの時刻 [s]")
ax_acc.set_ylabel("m/s²")
ax_acc.legend(fontsize=8)
ax_acc.grid(True, lw=0.4, alpha=0.5)

# --- 累積絶対誤差 ---
dt_arr = np.diff(steer["t"].values, prepend=steer["t"].values[0])
dt_arr[0] = dt_arr[1] if len(dt_arr) > 1 else 0.02
steer_err_abs_cumul_t = np.nancumsum(np.abs(steer_error) * dt_arr)
ax_cumul.plot(steer["tr"].values, steer_err_abs_cumul_t,
              color="black", lw=2.0)
ax_cumul.axvline(0, color="gray", lw=0.8, ls="--")
ax_cumul.set_title("ステア追従誤差 累積絶対値 [deg·s]", fontsize=10)
ax_cumul.set_xlabel("発進からの時刻 [s]")
ax_cumul.set_ylabel("deg·s")
ax_cumul.grid(True, lw=0.4, alpha=0.5)

# 追加情報テキスト
total_cumul = steer_err_abs_cumul_t[-1] if len(steer_err_abs_cumul_t) else 0
ax_cumul.text(0.02, 0.95, f"合計={total_cumul:.2f}°·s",
              transform=ax_cumul.transAxes, fontsize=8, va="top",
              bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.7))

FIGS_DIR.mkdir(parents=True, exist_ok=True)
add_params_annotation(fig)
for ext in ("png", "pdf"):
    p = FIGS_DIR / f"real_curve2_detail.{ext}"
    fig.savefig(str(p), dpi=150, bbox_inches="tight")
    print(f"Saved: {p}")
plt.close(fig)

# ---- 統計サマリ ----
print("\n=== カーブ② ステア追従誤差サマリ ===")
print(f"  RMSE:   {rmse_err:.4f} deg")
print(f"  平均誤差: {mean_err:.4f} deg (正=オーバーステア / 負=アンダーステア)")
print(f"  最大誤差: {float(np.nanmax(steer_error)):.4f} deg")
print(f"  最小誤差: {float(np.nanmin(steer_error)):.4f} deg")
print(f"  累積誤差: {total_cumul:.2f} deg·s")

# ステア各フェーズの誤差
if not steer.empty:
    phase1 = steer[(steer["tr"] >= 0) & (steer["tr"] <= 5)]
    phase2 = steer[(steer["tr"] > 5) & (steer["tr"] <= 15)]
    phase3 = steer[(steer["tr"] > 15)]
    for label, ph in [("t=0〜5s", phase1), ("t=5〜15s", phase2), ("t=15〜25s", phase3)]:
        if ph.empty:
            continue
        ph_err = steer_error[phase1.index[0] - steer.index[0]: phase1.index[-1] - steer.index[0] + 1] \
                 if ph is phase1 else \
                 steer_error[phase1.index[-1] - steer.index[0] + 1: (phase2.index[-1] - steer.index[0] + 1 if ph is phase2 else None)]
        if len(ph_err) == 0:
            continue
        print(f"  [{label}] 平均誤差={float(np.nanmean(ph_err)):.3f}°")

print("\nDone.")
