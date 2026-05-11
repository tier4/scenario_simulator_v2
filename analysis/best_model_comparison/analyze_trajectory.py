#!/usr/bin/env python3
"""DiffusionPlannerの計画軌跡を実機・シム間で直接比較する."""

import math
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
OUT_DIR  = BASE / "comparison" / "figures"

SIM_MCAP  = LITE_DIR / "sim_curve2.lite.mcap"
REAL_MCAP = LITE_DIR / "real.lite.mcap"

TRAJ_TOPIC = "/planning/trajectory_generator/neural_network_based_planner/diffusion_planner_node/output/trajectory"
SIG_TOPIC  = "/perception/traffic_light_recognition/traffic_signals"
TRACKED_OBJECTS_TOPIC = "/perception/object_recognition/tracking/objects"


def _iter_msgs(mcap_path, topics):
    with open(mcap_path, "rb") as f:
        reader = make_reader(f, decoder_factories=[DecoderFactory()])
        yield from reader.iter_decoded_messages(topics=topics)


def load_velocity(p):
    rows = []
    for _, _, msg, ros in _iter_msgs(p, ["/vehicle/status/velocity_status",
                                          "/sub/vehicle/status/velocity_status"]):
        rows.append({"t_ns": msg.log_time, "lon_vel": ros.longitudinal_velocity})
    return pd.DataFrame(rows)


def load_opmode(p):
    rows = []
    for _, _, msg, ros in _iter_msgs(p, ["/system/operation_mode/state"]):
        rows.append({"t_ns": msg.log_time, "mode": ros.mode})
    return pd.DataFrame(rows)


def find_t0_and_launch(p, real=False):
    df_vel = load_velocity(p)
    if real:
        df_op = load_opmode(p)
        if not df_op.empty:
            t0_ns = int(df_op[df_op["mode"] == 2]["t_ns"].iloc[0])
        else:
            t0_ns = int(df_vel[df_vel["lon_vel"] > 0.1]["t_ns"].iloc[0])
    else:
        t0_ns = int(df_vel["t_ns"].iloc[0])

    df_vel["t"] = (df_vel["t_ns"] - t0_ns) / 1e9
    df_vel = df_vel[df_vel["t"] >= 0].reset_index(drop=True)

    # 停止後発進検出
    stopped = df_vel[df_vel["lon_vel"] < 0.05]["t"].values
    t_launch = None
    if len(stopped) > 0:
        gaps = np.where(np.diff(stopped) > 2.0)[0]
        starts_idx = np.concatenate([[0], gaps + 1])
        ends_idx   = np.concatenate([gaps, [len(stopped) - 1]])
        candidates = []
        for s, e in zip(starts_idx, ends_idx):
            dur = stopped[e] - stopped[s]
            if dur >= 0.5 and 20.0 <= stopped[e] <= 120.0:
                candidates.append(stopped[e])
        if candidates:
            t_launch = float(min(candidates))

    if t_launch is None:
        moving = df_vel[(df_vel["lon_vel"] > 0.5) & (df_vel["t"] >= 5.0)]
        t_launch = float(moving["t"].iloc[0]) if not moving.empty else 0.0

    return t0_ns, t_launch


def load_trajectory_at_launch(mcap_path, t0_ns, t_launch, window_s=15.0):
    """t_launch周辺の軌跡フレームをロードする（発進前1s〜発進後window_s）."""
    t_launch_ns = t0_ns + int(t_launch * 1e9)
    t_start_ns  = t_launch_ns - int(1.0 * 1e9)
    t_end_ns    = t_launch_ns + int(window_s * 1e9)

    frames = []
    for _, _, msg, ros in _iter_msgs(mcap_path, [TRAJ_TOPIC]):
        t_msg = msg.log_time
        if t_msg < t_start_ns or t_msg > t_end_ns:
            continue
        t_rel = (t_msg - t0_ns) / 1e9 - t_launch

        pts = ros.points
        if not pts:
            continue

        x0, y0 = pts[0].pose.position.x, pts[0].pose.position.y
        dists, vels = [], []
        for pt in pts:
            dx = pt.pose.position.x - x0
            dy = pt.pose.position.y - y0
            d = math.sqrt(dx**2 + dy**2)
            if d > 100.0:
                break
            dists.append(d)
            vels.append(pt.longitudinal_velocity_mps)

        frames.append({
            "t_rel": t_rel,
            "t_ns": t_msg,
            "dists": np.array(dists),
            "vels":  np.array(vels),
        })

    return frames


def count_tracked_objects(mcap_path, t0_ns, t_launch, window_s=20.0):
    """発進前後の追跡物体数を時系列で取得する."""
    t_start_ns = t0_ns + int((t_launch - 5.0) * 1e9)
    t_end_ns   = t0_ns + int((t_launch + window_s) * 1e9)

    rows = []
    for _, _, msg, ros in _iter_msgs(mcap_path, [TRACKED_OBJECTS_TOPIC]):
        t_msg = msg.log_time
        if t_msg < t_start_ns or t_msg > t_end_ns:
            continue
        t_rel = (t_msg - t0_ns) / 1e9 - t_launch
        n_obj = len(ros.objects)
        rows.append({"t_rel": t_rel, "n_objects": n_obj})

    return pd.DataFrame(rows)


def load_traffic_signals(mcap_path, t0_ns, t_launch, window_s=50.0):
    """交通信号の状態履歴を抽出する."""
    t_start_ns = t0_ns
    t_end_ns   = t0_ns + int((t_launch + window_s) * 1e9)

    rows = []
    for _, _, msg, ros in _iter_msgs(mcap_path, [SIG_TOPIC]):
        t_msg = msg.log_time
        if t_msg < t_start_ns or t_msg > t_end_ns:
            continue
        t_rel = (t_msg - t0_ns) / 1e9

        for group in ros.traffic_light_groups:
            for elem in group.elements:
                rows.append({
                    "t": t_rel,
                    "group_id": group.traffic_light_group_id,
                    "color": elem.color,
                    "shape": elem.shape,
                    "status": elem.status,
                })

    return pd.DataFrame(rows)


def _frames_to_interp(frames, t_targets, d_target):
    """各t_targetにおけるd_target地点の速度を補間して返す."""
    result = []
    sorted_frames = sorted(frames, key=lambda x: x["t_rel"])
    for t in t_targets:
        closest = min(sorted_frames, key=lambda f: abs(f["t_rel"] - t), default=None)
        if closest is None or abs(closest["t_rel"] - t) > 0.15:
            result.append(np.nan)
            continue
        fr = closest
        if len(fr["dists"]) > 0 and d_target <= fr["dists"][-1]:
            v = float(np.interp(d_target, fr["dists"], fr["vels"]))
        else:
            v = np.nan
        result.append(v)
    return np.array(result)


def main():
    print("=== DiffusionPlanner 計画軌跡 実機 vs シム 直接比較 ===\n")

    t0_sim,  tl_sim  = find_t0_and_launch(SIM_MCAP,  real=False)
    t0_real, tl_real = find_t0_and_launch(REAL_MCAP, real=True)
    print(f"シム  : t_launch={tl_sim:.1f}s")
    print(f"実機  : t_launch={tl_real:.1f}s")

    # --- シム交通信号 ---
    print("\n--- シム 交通信号10583 状態推移 ---")
    df_sig = load_traffic_signals(SIM_MCAP, t0_sim, tl_sim, window_s=50.0)
    if df_sig.empty:
        print("  [警告] 交通信号データなし")
    else:
        sig_10583 = df_sig[df_sig["group_id"] == 10583]
        if not sig_10583.empty:
            prev = None
            for _, row in sig_10583.iterrows():
                state = (row["color"], row["status"])
                if state != prev:
                    color_name = {1: "RED", 2: "AMBER", 3: "GREEN", 0: "UNKNOWN"}.get(row["color"], "?")
                    print(f"  t={row['t']:6.1f}s: 色={color_name} status={row['status']}")
                    prev = state
        else:
            print(f"  信号10583なし。存在グループ: {sorted(df_sig['group_id'].unique())[:10]}")

    # --- 追跡物体数 ---
    print("\n--- 追跡物体数（発進前後） ---")
    df_obj_real = count_tracked_objects(REAL_MCAP, t0_real, tl_real)
    df_obj_sim  = count_tracked_objects(SIM_MCAP,  t0_sim,  tl_sim)

    if not df_obj_real.empty:
        print(f"  実機: 平均{df_obj_real['n_objects'].mean():.1f}物体, "
              f"最大{df_obj_real['n_objects'].max()}, 最小{df_obj_real['n_objects'].min()}")
    else:
        print("  実機: 追跡物体データなし")

    if not df_obj_sim.empty:
        print(f"  シム: 平均{df_obj_sim['n_objects'].mean():.1f}物体, "
              f"最大{df_obj_sim['n_objects'].max()}, 最小{df_obj_sim['n_objects'].min()}")
    else:
        print("  シム: 追跡物体データなし（0物体が正常）")

    # --- DP軌跡ロード ---
    print("\n--- DiffusionPlanner計画軌跡ロード ---")
    frames_sim  = load_trajectory_at_launch(SIM_MCAP,  t0_sim,  tl_sim,  window_s=15.0)
    frames_real = load_trajectory_at_launch(REAL_MCAP, t0_real, tl_real, window_s=15.0)
    print(f"  シム: {len(frames_sim)} フレーム")
    print(f"  実機: {len(frames_real)} フレーム")

    # --- テーブル表示: t_rel vs 各距離の計画速度 ---
    print("\n--- DP計画速度 比較テーブル（t_rel=-1〜+10s） ---")
    print(f"{'':>4} | {'------シム------':^42} | {'------実機------':^42}")
    print(f"{'t[s]':>4} | {'d=0':>6} {'d=5':>6} {'d=10':>6} {'d=20':>6} {'d=30':>6} {'d=50':>6} | "
          f"{'d=0':>6} {'d=5':>6} {'d=10':>6} {'d=20':>6} {'d=30':>6} {'d=50':>6}")
    print("  " + "-"*95)

    for t_val in [-1.0, 0.0, 0.5, 1.0, 2.0, 3.0, 5.0, 7.0, 10.0]:
        def get_vel_at(frames, t_target, d_target):
            sorted_f = sorted(frames, key=lambda x: x["t_rel"])
            closest = min(sorted_f, key=lambda f: abs(f["t_rel"] - t_target), default=None)
            if closest is None or abs(closest["t_rel"] - t_target) > 0.2:
                return float("nan")
            fr = closest
            if len(fr["dists"]) > 0 and d_target <= fr["dists"][-1]:
                return float(np.interp(d_target, fr["dists"], fr["vels"]))
            return float("nan")

        sim_row  = [get_vel_at(frames_sim,  t_val, d) for d in [0, 5, 10, 20, 30, 50]]
        real_row = [get_vel_at(frames_real, t_val, d) for d in [0, 5, 10, 20, 30, 50]]

        sim_str  = " ".join(f"{v:>6.2f}" if not np.isnan(v) else f"{'nan':>6}" for v in sim_row)
        real_str = " ".join(f"{v:>6.2f}" if not np.isnan(v) else f"{'nan':>6}" for v in real_row)
        print(f"{t_val:>4.1f} | {sim_str} | {real_str}")

    # --- プロット ---
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    fig, axes = plt.subplots(2, 3, figsize=(18, 10))
    fig.suptitle("DiffusionPlanner計画軌跡 直接比較（実機 vs シム）", fontsize=13)

    t_vec = np.linspace(-1, 12, 200)
    d_targets = [0, 5, 10, 20]
    labels_d = [f"d={d}m" for d in d_targets]
    colors_d = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728"]

    # 上段: 各距離での計画速度の時系列
    ax_sim  = axes[0, 0]
    ax_real = axes[0, 1]
    ax_diff = axes[0, 2]

    for d, lbl, col in zip(d_targets, labels_d, colors_d):
        v_sim  = _frames_to_interp(frames_sim,  t_vec, d)
        v_real = _frames_to_interp(frames_real, t_vec, d)
        ax_sim.plot(t_vec,  v_sim,  color=col, lw=1.5, label=lbl)
        ax_real.plot(t_vec, v_real, color=col, lw=1.5, label=lbl)
        ax_diff.plot(t_vec, v_sim - v_real, color=col, lw=1.5, ls="--", label=f"Δ {lbl}")

    for ax in [ax_sim, ax_real, ax_diff]:
        ax.axvline(0, color="gray", lw=0.8, ls="--", alpha=0.6)
        ax.axhline(0, color="gray", lw=0.5)
        ax.grid(True, lw=0.4)
        ax.set_xlabel("発進後 t [s]")
        ax.legend(fontsize=8)

    ax_sim.set_title("シム DP計画速度 [m/s]")
    ax_sim.set_ylabel("計画速度 [m/s]")
    ax_real.set_title("実機 DP計画速度 [m/s]")
    ax_diff.set_title("速度差 (シム − 実機) [m/s]")

    # 下段: 発進時 速度プロファイル vs 距離
    ax_prof_sim  = axes[1, 0]
    ax_prof_real = axes[1, 1]
    ax_obj       = axes[1, 2]

    cmap = plt.cm.viridis
    for frames, ax, title in [(frames_sim, ax_prof_sim, "シム"), (frames_real, ax_prof_real, "実機")]:
        sorted_f = sorted(frames, key=lambda x: x["t_rel"])
        for fr in sorted_f:
            if not (-1.0 <= fr["t_rel"] <= 10.0):
                continue
            c = cmap((fr["t_rel"] + 1) / 11.0)
            ax.plot(fr["dists"], fr["vels"], color=c, lw=1.2, alpha=0.6)
        # 発進時フレームを強調
        launch_f = [f for f in sorted_f if abs(f["t_rel"]) < 0.15]
        if launch_f:
            fr0 = launch_f[0]
            ax.plot(fr0["dists"], fr0["vels"], "r-", lw=3, label=f"t≈0 (launch)", zorder=5)
        sm = plt.cm.ScalarMappable(cmap=cmap,
             norm=matplotlib.colors.Normalize(vmin=-1, vmax=10))
        sm.set_array([])
        plt.colorbar(sm, ax=ax, label="t [s]")
        ax.set_xlabel("経路点距離 [m]")
        ax.set_ylabel("計画速度 [m/s]")
        ax.set_title(f"{title} DP速度プロファイル（-1s〜+10s）")
        ax.legend(fontsize=8)
        ax.grid(True, lw=0.4)

    # 追跡物体数
    if not df_obj_real.empty:
        ax_obj.plot(df_obj_real["t_rel"].values, df_obj_real["n_objects"].values,
                    "k-", lw=1.5, label="実機 追跡物体数", alpha=0.8)
    if not df_obj_sim.empty:
        ax_obj.plot(df_obj_sim["t_rel"].values, df_obj_sim["n_objects"].values,
                    color="#e05c00", lw=1.5, ls="--", label="シム 追跡物体数", alpha=0.8)
    else:
        ax_obj.axhline(0, color="#e05c00", lw=1.5, ls="--", label="シム (0物体)", alpha=0.8)
    ax_obj.axvline(0, color="gray", lw=0.8, ls="--", alpha=0.6)
    ax_obj.set_xlabel("発進後 t [s]")
    ax_obj.set_ylabel("追跡物体数")
    ax_obj.set_title("追跡物体数（DiffusionPlannerへの社会的コンテキスト）")
    ax_obj.legend(fontsize=9)
    ax_obj.grid(True, lw=0.4)

    fig.tight_layout()
    add_params_annotation(fig)
    out = OUT_DIR / "c2_dp_trajectory_comparison.png"
    fig.savefig(str(out), dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"\n  保存: {out}")

    # --- 実際の速度との比較（DP出力 vs actual velocity） ---
    print("\n--- DP計画速度(d=0) vs actual速度 ---")
    df_vel_sim  = load_velocity(SIM_MCAP)
    df_vel_real = load_velocity(REAL_MCAP)
    df_vel_sim["t_rel"]  = (df_vel_sim["t_ns"]  - t0_sim)  / 1e9 - tl_sim
    df_vel_real["t_rel"] = (df_vel_real["t_ns"] - t0_real) / 1e9 - tl_real
    df_vel_sim  = df_vel_sim[df_vel_sim["t_rel"]  >= -1].reset_index(drop=True)
    df_vel_real = df_vel_real[df_vel_real["t_rel"] >= -1].reset_index(drop=True)

    fig2, axes2 = plt.subplots(1, 2, figsize=(14, 5))
    fig2.suptitle("DPが計画した速度(d=0) vs 実際の速度", fontsize=12)

    for (frames, df_v, label, ax) in [
        (frames_sim,  df_vel_sim,  "シム",  axes2[0]),
        (frames_real, df_vel_real, "実機", axes2[1]),
    ]:
        dp_v = _frames_to_interp(frames, t_vec, 0)
        ax.plot(t_vec, dp_v, "b-", lw=2, label="DP計画速度 (d=0m)")
        ax.plot(df_v["t_rel"].values, df_v["lon_vel"].values, "r-", lw=1.5, alpha=0.8, label="actual速度")
        ax.axvline(0, color="gray", lw=0.8, ls="--", alpha=0.6)
        ax.set_xlabel("発進後 t [s]")
        ax.set_ylabel("速度 [m/s]")
        ax.set_title(f"{label}: DP計画 vs actual")
        ax.legend(fontsize=9)
        ax.grid(True, lw=0.4)

    fig2.tight_layout()
    add_params_annotation(fig2)
    out2 = OUT_DIR / "c2_dp_vs_actual.png"
    fig2.savefig(str(out2), dpi=150, bbox_inches="tight")
    plt.close(fig2)
    print(f"  保存: {out2}")


if __name__ == "__main__":
    main()
