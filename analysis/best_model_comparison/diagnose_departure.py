#!/usr/bin/env python3
"""発進時の車両モデル応答診断スクリプト.

実機 real.lite.mcap の /sub/control/command/control_cmd をオープンループ入力として
DELAY_STEER_ACC_BRAKE_GEARED_WO_FALL_GUARD モデルを様々な brake_time_constant で
シミュレーションし、実機 actual velocity との比較から適切なパラメータを診断する。
"""

import math
from pathlib import Path
from collections import deque

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
matplotlib.rcParams["font.family"] = "Noto Sans CJK JP"
import numpy as np
import pandas as pd
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory
from _params_utils import add_params_annotation, load_sim_params

BASE = Path(__file__).parent
LITE_DIR = BASE / "lite"
OUT_DIR  = BASE / "comparison" / "figures"
REAL_MCAP = LITE_DIR / "real.lite.mcap"


def _iter_msgs(mcap_path, topics):
    with open(mcap_path, "rb") as f:
        reader = make_reader(f, decoder_factories=[DecoderFactory()])
        yield from reader.iter_decoded_messages(topics=topics)


def load_control_cmd(p):
    rows = []
    for _, _, msg, ros in _iter_msgs(p, ["/sub/control/command/control_cmd"]):
        rows.append({
            "t_ns": msg.log_time,
            "acc": ros.longitudinal.acceleration,
        })
    return pd.DataFrame(rows)


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


def find_t0_and_launch(p):
    df_vel = load_velocity(p)
    df_op = load_opmode(p)
    if not df_op.empty:
        t0_ns = int(df_op[df_op["mode"] == 2]["t_ns"].iloc[0])
    else:
        t0_ns = int(df_vel[df_vel["lon_vel"] > 0.1]["t_ns"].iloc[0])

    df_vel["t"] = (df_vel["t_ns"] - t0_ns) / 1e9
    df_vel = df_vel[df_vel["t"] >= 0].reset_index(drop=True)

    stopped = df_vel[df_vel["lon_vel"] < 0.05]["t"].values
    t_launch = None
    if len(stopped) > 0:
        gaps = np.where(np.diff(stopped) > 2.0)[0]
        starts_idx = np.concatenate([[0], gaps + 1])
        ends_idx   = np.concatenate([gaps, [len(stopped) - 1]])
        for s, e in zip(starts_idx, ends_idx):
            dur = stopped[e] - stopped[s]
            if dur >= 0.5 and 20.0 <= stopped[e] <= 120.0:
                t_launch = float(stopped[e])
                break
    if t_launch is None:
        moving = df_vel[(df_vel["lon_vel"] > 0.5) & (df_vel["t"] >= 5.0)]
        t_launch = float(moving["t"].iloc[0]) if not moving.empty else 0.0

    return t0_ns, t_launch


class BrakeAccVehicleModel:
    """DELAY_STEER_ACC_BRAKE_GEARED_WO_FALL_GUARD の縦方向モデル簡易実装."""

    def __init__(self, acc_time_delay, acc_time_constant, brake_delay, brake_time_constant,
                 departure_vx_threshold, vel_rate_lim, dt):
        self.acc_tc    = acc_time_constant
        self.brake_tc  = brake_time_constant
        self.departure_threshold = departure_vx_threshold
        self.vel_rate_lim = vel_rate_lim
        self.dt = dt

        acc_queue_len   = max(1, round(acc_time_delay / dt))
        brake_queue_len = max(1, round(brake_delay / dt))
        self.acc_queue   = deque([0.0] * acc_queue_len)
        self.brake_queue = deque([0.0] * brake_queue_len)

        self.vx            = 0.0
        self.pedal_acc     = 0.0
        self.pedal_brake   = 0.0
        self.in_departure  = True
        self.delayed_accel = 0.0
        self.delayed_brake = 0.0

    def step(self, cmd_acc):
        accel_des = max(0.0, cmd_acc)
        brake_des = min(0.0, cmd_acc)

        self.acc_queue.append(accel_des)
        self.delayed_accel = self.acc_queue.popleft()
        self.brake_queue.append(brake_des)
        self.delayed_brake = self.brake_queue.popleft()

        if self.vx == 0.0:
            self.in_departure = True
        if self.vx > self.departure_threshold:
            self.in_departure = False

        eff_brake_tc = self.brake_tc if self.in_departure else self.acc_tc

        net = self.pedal_acc + self.pedal_brake
        d_vx = net
        d_pacc   = -(self.pedal_acc - min(self.delayed_accel, self.vel_rate_lim)) / self.acc_tc
        d_pbrake = -(self.pedal_brake - max(self.delayed_brake, -self.vel_rate_lim)) / eff_brake_tc

        prev_vx = self.vx
        self.vx          += d_vx    * self.dt
        self.pedal_acc   += d_pacc  * self.dt
        self.pedal_brake += d_pbrake * self.dt

        self.pedal_acc   = max(0.0, self.pedal_acc)
        self.pedal_brake = min(0.0, self.pedal_brake)
        self.vx          = max(-50.0, min(self.vx, 50.0))

        # stop condition
        if prev_vx * self.vx <= 0.0 and self.delayed_brake < 0.0:
            if -(self.pedal_acc + self.pedal_brake) >= 0.0:
                self.vx = 0.0

        # backward guard
        if self.vx < 0.0 and self.delayed_brake >= 0.0:
            net_ss = self.delayed_accel
            if net_ss >= 0.0:
                self.vx = 0.0

        return self.vx


def simulate_departure(cmd_series, brake_tc_variants, params_base, dt=1.0/40):
    """同一 cmd_series に対して複数の brake_time_constant でシミュレーション."""
    results = {}
    for btc in brake_tc_variants:
        model = BrakeAccVehicleModel(
            acc_time_delay=params_base["acc_time_delay"],
            acc_time_constant=params_base["acc_time_constant"],
            brake_delay=params_base["brake_delay"],
            brake_time_constant=btc,
            departure_vx_threshold=params_base["departure_vx_threshold"],
            vel_rate_lim=params_base["vel_rate_lim"],
            dt=dt,
        )
        vx_list = []
        for cmd in cmd_series:
            vx_list.append(model.step(cmd))
        results[btc] = np.array(vx_list)
    return results


def main():
    print("=== 発進時車両モデル応答診断 ===\n")

    t0_ns, t_launch = find_t0_and_launch(REAL_MCAP)
    print(f"実機 t_launch = {t_launch:.2f} s")

    df_cmd = load_control_cmd(REAL_MCAP)
    df_vel = load_velocity(REAL_MCAP)

    df_cmd["t"] = (df_cmd["t_ns"] - t0_ns) / 1e9 - t_launch
    df_vel["t"] = (df_vel["t_ns"] - t0_ns) / 1e9 - t_launch

    window = (-2.0, 15.0)
    df_cmd = df_cmd[(df_cmd["t"] >= window[0]) & (df_cmd["t"] <= window[1])].reset_index(drop=True)
    df_vel = df_vel[(df_vel["t"] >= window[0]) & (df_vel["t"] <= window[1])].reset_index(drop=True)

    print(f"control_cmd サンプル数: {len(df_cmd)}")
    print(f"velocity サンプル数: {len(df_vel)}")

    if df_cmd.empty:
        print("ERROR: control_cmd が空です。real.lite.mcap を確認してください。")
        return

    dt = 1.0 / 40.0
    t_sim = np.arange(window[0], window[1], dt)
    cmd_resampled = np.interp(t_sim, df_cmd["t"].values, df_cmd["acc"].values)

    params_base = {
        "acc_time_delay":        0.101,
        "acc_time_constant":     0.2589,
        "brake_delay":           0.0685,
        "departure_vx_threshold": 1.0,
        "vel_rate_lim":          7.0,
    }

    brake_tc_variants = [0.0301, 0.10, 0.20, 0.40]
    colors = ["#d62728", "#ff7f0e", "#2ca02c", "#9467bd"]
    labels = [f"brake_tc={v:.4f}s" for v in brake_tc_variants]
    labels[0] += " (現在)"

    print("\nbake_time_constant バリアントでシミュレーション中...")
    sim_results = simulate_departure(cmd_resampled, brake_tc_variants, params_base, dt=dt)

    # 実機 actual velocity の t=0以降を抽出
    df_vel_dep = df_vel[df_vel["t"] >= -0.5]

    # --- テーブル出力 ---
    print(f"\n{'t[s]':>6} | {'実機actual':>10} | " +
          " | ".join(f"btc={v:.4f}" for v in brake_tc_variants))
    print("-" * (6 + 10 + 16 * len(brake_tc_variants) + 10))
    for t_chk in [-0.5, 0.0, 0.5, 1.0, 1.5, 2.0, 3.0, 5.0, 7.0, 10.0]:
        idx = np.argmin(np.abs(t_sim - t_chk))
        real_v = float(np.interp(t_chk, df_vel["t"].values, df_vel["lon_vel"].values)) if not df_vel.empty else float("nan")
        row = f"{t_chk:>6.1f} | {real_v:>10.3f} | "
        row += " | ".join(f"{sim_results[v][idx]:>10.3f}   " for v in brake_tc_variants)
        print(row)

    # --- プロット ---
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    fig.suptitle("発進動作：brake_time_constant 感度分析\n"
                 "(実機 post-gate cmd をオープンループ入力→シミュレーション速度 vs 実機 actual速度)", fontsize=11)

    # 左: 速度プロファイル比較
    ax1 = axes[0]
    ax1.plot(df_vel["t"].values, df_vel["lon_vel"].values,
             "k-", lw=3, label="実機 actual速度", zorder=10)
    for btc, col, lbl in zip(brake_tc_variants, colors, labels):
        ax1.plot(t_sim, sim_results[btc], color=col, lw=1.8, ls="--", label=f"FMU {lbl}")
    ax1.axvline(0, color="gray", lw=0.8, ls="--", alpha=0.6, label="発進 t=0")
    ax1.set_xlim(-1, 10)
    ax1.set_ylim(-0.5, 8)
    ax1.set_xlabel("発進後 t [s]")
    ax1.set_ylabel("速度 [m/s]")
    ax1.set_title("速度プロファイル比較（発進前後）")
    ax1.legend(fontsize=8)
    ax1.grid(True, lw=0.4)

    # 右: 発進直後のズーム
    ax2 = axes[1]
    ax2.plot(df_vel["t"].values, df_vel["lon_vel"].values,
             "k-", lw=3, label="実機 actual速度", zorder=10)
    for btc, col, lbl in zip(brake_tc_variants, colors, labels):
        ax2.plot(t_sim, sim_results[btc], color=col, lw=1.8, ls="--", label=f"FMU {lbl}")
    ax2.axvline(0, color="gray", lw=0.8, ls="--", alpha=0.6)
    ax2.set_xlim(-0.5, 3.0)
    ax2.set_ylim(-0.2, 2.5)
    ax2.set_xlabel("発進後 t [s]")
    ax2.set_ylabel("速度 [m/s]")
    ax2.set_title("発進直後ズーム (t=-0.5~3s)")
    ax2.legend(fontsize=8)
    ax2.grid(True, lw=0.4)

    # cmd_acc をサブプロットとして下段に追加
    fig2, axes2 = plt.subplots(1, 1, figsize=(10, 4))
    fig2.suptitle("実機 post-gate control_cmd (acc) — 発進前後", fontsize=11)
    axes2.plot(df_cmd["t"].values, df_cmd["acc"].values, "b-", lw=1.5, label="実機 cmd_acc (post-gate)")
    axes2.axvline(0, color="gray", lw=0.8, ls="--", alpha=0.6)
    axes2.set_xlim(-2, 10)
    axes2.set_xlabel("発進後 t [s]")
    axes2.set_ylabel("加速度指令 [m/s²]")
    axes2.legend(fontsize=9)
    axes2.grid(True, lw=0.4)

    fig.tight_layout()
    fig2.tight_layout()

    # params_base に YAML の共通パラメータをマージしてアノテーションを付与する
    _anno_params = {**load_sim_params(), **params_base}
    add_params_annotation(fig,  _anno_params)
    add_params_annotation(fig2, _anno_params)

    out1 = OUT_DIR / "departure_brake_tc_sensitivity.png"
    out2 = OUT_DIR / "real_cmd_acc_departure.png"
    fig.savefig(str(out1), dpi=150, bbox_inches="tight")
    fig2.savefig(str(out2), dpi=150, bbox_inches="tight")
    plt.close("all")
    print(f"\n保存: {out1}")
    print(f"保存: {out2}")

    # --- 発進誤差の定量評価 ---
    print("\n--- 実機 actual速度 vs FMU シム 発進誤差 RMSE (t=0~5s) ---")
    t_mask = (t_sim >= 0.0) & (t_sim <= 5.0)
    real_interp = np.interp(t_sim[t_mask], df_vel["t"].values, df_vel["lon_vel"].values)
    for btc in brake_tc_variants:
        sim_v = sim_results[btc][t_mask]
        rmse = np.sqrt(np.mean((sim_v - real_interp) ** 2))
        mean_err = np.mean(sim_v - real_interp)
        print(f"  brake_tc={btc:.4f}:  RMSE={rmse:.4f} m/s, mean_err={mean_err:+.4f} m/s")


if __name__ == "__main__":
    main()
