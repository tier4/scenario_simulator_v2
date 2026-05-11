#!/usr/bin/env python3
"""カーブ②乖離詳細診断スクリプト.

軌跡乖離の原因を特定するため、1秒分解能で位置・速度・ステア・yaw の比較を行う。
乖離の縦横成分分解と、プランナー速度差の寄与を定量化する。
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

BASE     = Path(__file__).parent
LITE_DIR = BASE / "lite"
OUT_DIR  = BASE / "comparison" / "figures"

LOGS = {
    "実機": {
        "path": LITE_DIR / "real.lite.mcap",
        "kinematic": "/sub/localization/kinematic_state",
        "cmd": "/sub/control/command/control_cmd",
        "color": "black",
        "real": True,
    },
    "シム (Curve2)": {
        "path": LITE_DIR / "sim_curve2.lite.mcap",
        "kinematic": "/localization/kinematic_state",
        "cmd": "/control/trajectory_follower/control_cmd",
        "color": "#e05c00",
        "real": False,
    },
}


def _iter_msgs(mcap_path, topics):
    with open(mcap_path, "rb") as f:
        reader = make_reader(f, decoder_factories=[DecoderFactory()])
        yield from reader.iter_decoded_messages(topics=topics)


def _load_velocity(p):
    rows = []
    topic = "/sub/vehicle/status/velocity_status" if "real" in str(p) else "/vehicle/status/velocity_status"
    for _, _, msg, ros in _iter_msgs(p, ["/vehicle/status/velocity_status",
                                          "/sub/vehicle/status/velocity_status"]):
        rows.append({"t_ns": msg.log_time, "lon_vel": ros.longitudinal_velocity})
    return pd.DataFrame(rows)


def _load_steering(p):
    rows = []
    for _, _, msg, ros in _iter_msgs(p, ["/vehicle/status/steering_status",
                                          "/sub/vehicle/status/steering_status"]):
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


def _find_sim_launch(df_vel, threshold=0.5, min_t=5.0):
    moving = df_vel[(df_vel["lon_vel"] > threshold) & (df_vel["t"] >= min_t)]
    return float(moving["t"].iloc[0]) if not moving.empty else None


def load_data():
    loaded = {}
    for label, cfg in LOGS.items():
        p = cfg["path"]
        df_vel = _load_velocity(p)

        if cfg["real"]:
            t0_ns = int(df_vel[df_vel["lon_vel"] > 0.1]["t_ns"].iloc[0])
            df_vel = _align(df_vel, t0_ns)
            t_launch = _find_curve2_launch(df_vel)
            print(f"  [{label}] カーブ②発進 t={t_launch:.1f}s")
        else:
            t0_ns = int(df_vel["t_ns"].iloc[0])
            df_vel = _align(df_vel, t0_ns)
            t_launch = _find_curve2_launch(df_vel)
            if t_launch is not None:
                print(f"  [{label}] 信号停止後発進 t={t_launch:.1f}s")
            else:
                t_launch = _find_sim_launch(df_vel)
                print(f"  [{label}] 発進検出 t={t_launch:.1f}s")

        df_kin = _align(_load_kinematic(p, cfg["kinematic"]), t0_ns)
        df_cmd = _align(_load_cmd(p, cfg["cmd"]), t0_ns)
        df_steer = _align(_load_steering(p), t0_ns)

        # tr: 発進後の時刻
        for df in [df_vel, df_kin, df_cmd, df_steer]:
            df["tr"] = df["t"] - t_launch

        loaded[label] = {
            "cfg": cfg, "t_launch": t_launch,
            "vel": df_vel, "kin": df_kin, "cmd": df_cmd, "steer": df_steer,
        }
    return loaded


def _at(df, tr_target, col):
    """指定 tr に最も近い行の col 値を返す。"""
    if df.empty:
        return np.nan
    idx = (df["tr"] - tr_target).abs().idxmin()
    return df.loc[idx, col]


def decompose_deviation(real_kin, sim_kin, tr_target):
    """実機のyaw方向を基準に横方向・縦方向の乖離を分解する。"""
    # 実機の位置・yaw
    idx_r = (real_kin["tr"] - tr_target).abs().idxmin()
    rx, ry, ryaw = real_kin.loc[idx_r, ["x", "y", "yaw"]]

    # シムの位置
    idx_s = (sim_kin["tr"] - tr_target).abs().idxmin()
    sx, sy = sim_kin.loc[idx_s, ["x", "y"]]

    dx, dy = sx - rx, sy - ry
    dist = math.sqrt(dx**2 + dy**2)

    # 実機進行方向を基準にした横方向・縦方向
    lon =  dx * math.cos(ryaw) + dy * math.sin(ryaw)   # 前方が正
    lat = -dx * math.sin(ryaw) + dy * math.cos(ryaw)   # 左が正

    return dist, lon, lat, rx, ry, ryaw, sx, sy


def print_diagnosis(loaded):
    real = loaded["実機"]
    sim  = loaded["シム (Curve2)"]

    real_kin = real["kin"]
    sim_kin  = sim["kin"]
    real_vel = real["vel"]
    sim_vel  = sim["vel"]
    real_cmd = real["cmd"]
    sim_cmd  = sim["cmd"]
    real_steer = real["steer"]
    sim_steer  = sim["steer"]

    print("\n" + "="*80)
    print("カーブ②乖離詳細診断（1s分解能）")
    print("="*80)
    print(f"{'t[s]':>5} | {'実機速度':>8} {'シム速度':>8} {'速度差':>7} | "
          f"{'実機cmd':>8} {'シムcmd':>8} {'cmd差':>7} | "
          f"{'実機steer':>9} {'シムsteer':>9} | "
          f"{'乖離[m]':>7} {'縦[m]':>7} {'横[m]':>7}")
    print("-"*110)

    for tr in range(-2, 28):
        r_v   = _at(real_vel,   tr, "lon_vel")
        s_v   = _at(sim_vel,    tr, "lon_vel")
        r_cmd = _at(real_cmd,   tr, "cmd_vel")
        s_cmd = _at(sim_cmd,    tr, "cmd_vel")
        r_str = math.degrees(_at(real_steer, tr, "steer"))
        s_str = math.degrees(_at(sim_steer,  tr, "steer"))

        dist, lon, lat, *_ = decompose_deviation(real_kin, sim_kin, tr)

        dv  = s_v   - r_v
        dc  = s_cmd - r_cmd

        print(f"{tr:>5} | {r_v:>8.3f} {s_v:>8.3f} {dv:>+7.3f} | "
              f"{r_cmd:>8.3f} {s_cmd:>8.3f} {dc:>+7.3f} | "
              f"{r_str:>9.2f} {s_str:>9.2f} | "
              f"{dist:>7.3f} {lon:>+7.3f} {lat:>+7.3f}")


def plot_detailed(loaded):
    real = loaded["実機"]
    sim  = loaded["シム (Curve2)"]

    t_vec = np.linspace(-2, 27, 300)

    def interp(d, col):
        df = d[col if col in d else "vel"]
        return np.interp(t_vec, df["tr"].values, df["lon_vel"].values,
                         left=np.nan, right=np.nan)

    # 位置差の縦横分解
    dists, lons, lats = [], [], []
    for tr in t_vec:
        d, lon, lat, *_ = decompose_deviation(real["kin"], sim["kin"], tr)
        dists.append(d)
        lons.append(lon)
        lats.append(lat)
    dists = np.array(dists)
    lons  = np.array(lons)
    lats  = np.array(lats)

    # yaw差
    real_kin = real["kin"]
    sim_kin  = sim["kin"]
    yaw_diff = []
    for tr in t_vec:
        ry = _at(real_kin, tr, "yaw")
        sy = _at(sim_kin,  tr, "yaw")
        diff = math.degrees(sy - ry)
        while diff >  180: diff -= 360
        while diff < -180: diff += 360
        yaw_diff.append(diff)
    yaw_diff = np.array(yaw_diff)

    # 速度・cmd_vel
    rv = np.interp(t_vec, real["vel"]["tr"].values, real["vel"]["lon_vel"].values, left=np.nan, right=np.nan)
    sv = np.interp(t_vec, sim["vel"]["tr"].values,  sim["vel"]["lon_vel"].values,  left=np.nan, right=np.nan)
    rc = np.interp(t_vec, real["cmd"]["tr"].values, real["cmd"]["cmd_vel"].values, left=np.nan, right=np.nan)
    sc = np.interp(t_vec, sim["cmd"]["tr"].values,  sim["cmd"]["cmd_vel"].values,  left=np.nan, right=np.nan)

    # ステア
    rs = np.degrees(np.interp(t_vec, real["steer"]["tr"].values, real["steer"]["steer"].values, left=np.nan, right=np.nan))
    ss = np.degrees(np.interp(t_vec, sim["steer"]["tr"].values,  sim["steer"]["steer"].values,  left=np.nan, right=np.nan))

    fig, axes = plt.subplots(5, 1, figsize=(14, 22), sharex=True)
    fig.suptitle("カーブ②乖離詳細診断", fontsize=13)

    # 速度
    ax = axes[0]
    ax.plot(t_vec, rv, "k-", lw=2, label="実機 actual")
    ax.plot(t_vec, sv, color="#e05c00", lw=2, ls="--", label="シム actual")
    ax.plot(t_vec, rc, "k:", lw=1.2, alpha=0.6, label="実機 cmd")
    ax.plot(t_vec, sc, color="#e05c00", lw=1.2, ls=":", alpha=0.6, label="シム cmd")
    ax.fill_between(t_vec, rv, sv, alpha=0.15, color="red", label="速度差")
    ax.set_ylabel("速度 [m/s]")
    ax.set_title("速度（actual/cmd）")
    ax.legend(fontsize=8, ncol=4)
    ax.grid(True, lw=0.4)
    ax.axvline(0, color="gray", lw=0.8, ls="--", alpha=0.6)

    # 速度差
    ax = axes[1]
    ax.plot(t_vec, sv - rv, color="#e05c00", lw=2, ls="--", label="actual差 (sim-real)")
    ax.plot(t_vec, sc - rc, color="#e05c00", lw=1.5, ls=":", alpha=0.7, label="cmd差 (sim-real)")
    ax.axhline(0, color="gray", lw=0.5)
    ax.set_ylabel("m/s")
    ax.set_title("速度差（シム − 実機）")
    ax.legend(fontsize=8)
    ax.grid(True, lw=0.4)
    ax.axvline(0, color="gray", lw=0.8, ls="--", alpha=0.6)

    # ステア
    ax = axes[2]
    ax.plot(t_vec, rs, "k-", lw=2, label="実機 actual")
    ax.plot(t_vec, ss, color="#e05c00", lw=2, ls="--", label="シム actual")
    ax.set_ylabel("deg")
    ax.set_title("ステアリング角応答")
    ax.legend(fontsize=8)
    ax.grid(True, lw=0.4)
    ax.axvline(0, color="gray", lw=0.8, ls="--", alpha=0.6)

    # yaw差
    ax = axes[3]
    ax.plot(t_vec, yaw_diff, color="purple", lw=2, label="yaw差 (sim-real) [deg]")
    ax.axhline(0, color="gray", lw=0.5)
    ax.set_ylabel("deg")
    ax.set_title("ヨー角差（シム − 実機）")
    ax.legend(fontsize=8)
    ax.grid(True, lw=0.4)
    ax.axvline(0, color="gray", lw=0.8, ls="--", alpha=0.6)

    # 乖離 縦横分解
    ax = axes[4]
    ax.plot(t_vec, dists, "r-", lw=2, label="総乖離距離 [m]")
    ax.plot(t_vec, lons, color="blue", lw=1.5, ls="--", label="縦方向 (実機前方正) [m]")
    ax.plot(t_vec, lats, color="green", lw=1.5, ls="-.", label="横方向 (左正) [m]")
    ax.axhline(0, color="gray", lw=0.5)
    ax.set_ylabel("m")
    ax.set_xlabel("発進からの時刻 [s]")
    ax.set_title("軌跡乖離の縦横分解（実機進行方向基準）")
    ax.legend(fontsize=8)
    ax.grid(True, lw=0.4)
    ax.axvline(0, color="gray", lw=0.8, ls="--", alpha=0.6)

    fig.tight_layout()
    add_params_annotation(fig)
    out = OUT_DIR / "c2_diagnosis.png"
    fig.savefig(str(out), dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"\n  保存: {out}")


def main():
    print("=== データ読み込み ===")
    loaded = load_data()

    print_diagnosis(loaded)
    plot_detailed(loaded)


if __name__ == "__main__":
    main()
