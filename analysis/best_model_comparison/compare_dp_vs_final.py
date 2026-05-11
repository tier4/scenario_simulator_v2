#!/usr/bin/env python3
"""DP出力軌跡 vs 最終planning/trajectory の速度プロファイル比較.

実機mcapで両トピックを比較して、trajectory_velocity_optimizerが
発進後の速度にどう影響するか確認する。
"""

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

REAL_MCAP = LITE_DIR / "real.lite.mcap"
SIM_MCAP  = LITE_DIR / "sim_curve2.lite.mcap"

DP_TOPIC    = "/planning/trajectory_generator/neural_network_based_planner/diffusion_planner_node/output/trajectory"
FINAL_TOPIC = "/planning/trajectory"


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


def load_traj_frames(mcap_path, topic, t0_ns, t_launch, window_s=15.0):
    t_launch_ns = t0_ns + int(t_launch * 1e9)
    t_start_ns  = t_launch_ns - int(1.5 * 1e9)
    t_end_ns    = t_launch_ns + int(window_s * 1e9)

    frames = []
    for _, _, msg, ros in _iter_msgs(mcap_path, [topic]):
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
        frames.append({"t_rel": t_rel, "dists": np.array(dists), "vels": np.array(vels)})
    return frames


def frames_to_series(frames, t_vec, d_target):
    result = []
    sorted_f = sorted(frames, key=lambda x: x["t_rel"])
    for t in t_vec:
        closest = min(sorted_f, key=lambda f: abs(f["t_rel"] - t), default=None)
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
    print("=== DP出力 vs 最終trajectory 速度比較 ===\n")

    t0_real, tl_real = find_t0_and_launch(REAL_MCAP, real=True)
    print(f"実機 t_launch={tl_real:.1f}s")

    # 実機: DP出力 vs 最終軌跡
    frames_dp    = load_traj_frames(REAL_MCAP, DP_TOPIC,    t0_real, tl_real)
    frames_final = load_traj_frames(REAL_MCAP, FINAL_TOPIC, t0_real, tl_real)
    print(f"  DP出力フレーム: {len(frames_dp)}")
    print(f"  最終軌跡フレーム: {len(frames_final)}")

    t_vals = [-1.0, 0.0, 0.5, 1.0, 2.0, 3.0, 5.0, 7.0, 10.0, 12.0]
    d_list = [0, 5, 10, 20, 30]

    print("\n--- 実機: DP出力 vs 最終trajectory（各距離の速度）---")
    print(f"{'t[s]':>5} | {'------ DP出力 ------':^35} | {'--- 最終traj ---':^35}")
    print(f"{'':>5} | " + " ".join(f"d={d}m" for d in d_list) + " | " + " ".join(f"d={d}m" for d in d_list))
    print("-" * 90)

    for t_val in t_vals:
        def gv(frames, t, d):
            sorted_f = sorted(frames, key=lambda x: x["t_rel"])
            c = min(sorted_f, key=lambda f: abs(f["t_rel"] - t), default=None)
            if c is None or abs(c["t_rel"] - t) > 0.2:
                return float("nan")
            fr = c
            if len(fr["dists"]) > 0 and d <= fr["dists"][-1]:
                return float(np.interp(d, fr["dists"], fr["vels"]))
            return float("nan")

        dp_row  = [gv(frames_dp,    t_val, d) for d in d_list]
        fin_row = [gv(frames_final, t_val, d) for d in d_list]

        def fmt(v):
            return f"{v:>6.2f}" if not np.isnan(v) else f"{'nan':>6}"

        print(f"{t_val:>5.1f} | " + " ".join(fmt(v) for v in dp_row) +
              " | " + " ".join(fmt(v) for v in fin_row))

    # シム: DP出力のみ（最終trajectoryなし）
    t0_sim, tl_sim = find_t0_and_launch(SIM_MCAP, real=False)
    frames_sim_dp = load_traj_frames(SIM_MCAP, DP_TOPIC, t0_sim, tl_sim)
    print(f"\nシム t_launch={tl_sim:.1f}s, DP出力フレーム: {len(frames_sim_dp)}")

    # --- プロット ---
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    t_vec = np.linspace(-1, 13, 250)

    fig, axes = plt.subplots(2, 3, figsize=(18, 10))
    fig.suptitle("DP出力 vs 最終planning/trajectory — 実機（行上段）/ 速度差（行下段）", fontsize=12)

    d_targets = [0, 5, 10, 20, 30]
    colors = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "#9467bd"]

    # 上段: DP出力 vs 最終trajectory の各距離速度
    for i, d in enumerate(d_targets[:3]):
        ax = axes[0, i]
        v_dp    = frames_to_series(frames_dp,    t_vec, d)
        v_final = frames_to_series(frames_final, t_vec, d)
        v_sim   = frames_to_series(frames_sim_dp, t_vec, d)
        ax.plot(t_vec, v_dp,    "k-",  lw=2,   label="実機 DP出力")
        ax.plot(t_vec, v_final, "b--", lw=2,   label="実機 最終traj")
        ax.plot(t_vec, v_sim,   "r:",  lw=1.5, label="シム DP出力")
        ax.fill_between(t_vec, v_dp, v_final, alpha=0.2, color="blue",
                        label=f"optimizer補正")
        ax.axvline(0, color="gray", lw=0.8, ls="--", alpha=0.6)
        ax.set_title(f"d={d}m 地点の計画速度")
        ax.set_xlabel("発進後 t [s]")
        ax.set_ylabel("計画速度 [m/s]")
        ax.legend(fontsize=8)
        ax.grid(True, lw=0.4)

    # 下段: DP vs 最終の差と、実機 vs シムの差
    ax_diff_optimizer = axes[1, 0]
    ax_diff_realvsim  = axes[1, 1]
    ax_summary        = axes[1, 2]

    for d, col in zip(d_targets, colors):
        v_dp    = frames_to_series(frames_dp,    t_vec, d)
        v_final = frames_to_series(frames_final, t_vec, d)
        v_sim   = frames_to_series(frames_sim_dp, t_vec, d)
        diff_opt = v_final - v_dp
        diff_rs  = v_sim   - v_dp
        ax_diff_optimizer.plot(t_vec, diff_opt, color=col, lw=1.5, label=f"d={d}m")
        ax_diff_realvsim.plot( t_vec, diff_rs,  color=col, lw=1.5, label=f"d={d}m")

    for ax, title in [
        (ax_diff_optimizer, "optimizer補正量\n(最終traj - DP出力) [実機]"),
        (ax_diff_realvsim,  "シム - 実機 DP計画速度差"),
    ]:
        ax.axhline(0, color="gray", lw=0.5)
        ax.axvline(0, color="gray", lw=0.8, ls="--", alpha=0.6)
        ax.set_xlabel("発進後 t [s]")
        ax.set_ylabel("Δv [m/s]")
        ax.set_title(title)
        ax.legend(fontsize=8)
        ax.grid(True, lw=0.4)

    # 要約: 発進後の実速度
    df_vel_real = load_velocity(REAL_MCAP)
    df_vel_sim  = load_velocity(SIM_MCAP)
    t0_r, tl_r = t0_real, tl_real
    t0_s, tl_s = t0_sim,  tl_sim
    df_vel_real["tr"] = (df_vel_real["t_ns"] - t0_r) / 1e9 - tl_r
    df_vel_sim["tr"]  = (df_vel_sim["t_ns"]  - t0_s) / 1e9 - tl_s
    df_vel_real = df_vel_real[df_vel_real["tr"] >= -1]
    df_vel_sim  = df_vel_sim[df_vel_sim["tr"]   >= -1]

    dp0_real = frames_to_series(frames_dp,     t_vec, 0)
    dp0_sim  = frames_to_series(frames_sim_dp, t_vec, 0)
    fin0_real = frames_to_series(frames_final, t_vec, 0)

    ax_summary.plot(df_vel_real["tr"].values, df_vel_real["lon_vel"].values,
                    "k-", lw=2, label="実機 actual速度")
    ax_summary.plot(df_vel_sim["tr"].values, df_vel_sim["lon_vel"].values,
                    "r-", lw=2, ls="--", label="シム actual速度")
    ax_summary.plot(t_vec, dp0_real,  "k:", lw=1.5, label="実機 DP d=0")
    ax_summary.plot(t_vec, dp0_sim,   "r:", lw=1.5, alpha=0.7, label="シム DP d=0")
    ax_summary.plot(t_vec, fin0_real, "b--", lw=1.5, label="実機 最終traj d=0")
    ax_summary.axvline(0, color="gray", lw=0.8, ls="--", alpha=0.6)
    ax_summary.set_xlabel("発進後 t [s]")
    ax_summary.set_ylabel("速度 [m/s]")
    ax_summary.set_title("DP出力 d=0 / 最終traj / actual速度 比較")
    ax_summary.legend(fontsize=8)
    ax_summary.grid(True, lw=0.4)

    fig.tight_layout()
    add_params_annotation(fig)
    out = OUT_DIR / "c2_dp_vs_final_traj.png"
    fig.savefig(str(out), dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"\n  保存: {out}")

    # --- optimizer補正の定量評価 ---
    print("\n--- optimizer補正量の要約（実機）---")
    print(f"{'t[s]':>5} | " + " ".join(f"{'d='+str(d)+'m':>7}" for d in d_list))
    print("-" * 50)
    for t_val in [0.0, 1.0, 3.0, 5.0, 7.0, 10.0]:
        def gv(frames, t, d):
            sorted_f = sorted(frames, key=lambda x: x["t_rel"])
            c = min(sorted_f, key=lambda f: abs(f["t_rel"] - t), default=None)
            if c is None or abs(c["t_rel"] - t) > 0.2:
                return float("nan")
            fr = c
            if len(fr["dists"]) > 0 and d <= fr["dists"][-1]:
                return float(np.interp(d, fr["dists"], fr["vels"]))
            return float("nan")
        corr = [gv(frames_final, t_val, d) - gv(frames_dp, t_val, d) for d in d_list]
        def fmt(v):
            return f"{v:>+7.2f}" if not np.isnan(v) else f"{'nan':>7}"
        print(f"{t_val:>5.1f} | " + " ".join(fmt(v) for v in corr))


if __name__ == "__main__":
    main()
