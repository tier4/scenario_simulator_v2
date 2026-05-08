#!/usr/bin/env python3
"""実機 vs カーブ②集中シム の比較プロット生成スクリプト.

Usage:
    python3 compare_curve2.py

Inputs:
    lite/real.lite.mcap      — 実機ログ（カーブ②前の一時停止を自動検出して発進t=0に揃える）
    lite/sim_curve2.lite.mcap — x2_dev_curve2_start.yaml のシム出力（t=0が発進点）

Outputs:
    comparison/figures/c2_{trajectory,timeseries,steering_detail,velocity_response,deviation}.{png,pdf}
    comparison/figures/c2_{timeseries,velocity_response,deviation}_vs_dist.{png,pdf}
    comparison/curve2_report.md
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

BASE = Path(__file__).parent
LITE_DIR = BASE / "lite"
OUT_DIR = BASE / "comparison"
FIGS_DIR = OUT_DIR / "figures"
MAP_DIR = Path.home() / ".webauto/simulation/data/map/x2_dev/2231"

CURVE2_CX, CURVE2_CY, CURVE2_MARGIN = 89301, 43085, 80
WHEELBASE = 5.15

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
        "ms": 4,
        "real": True,
    },
    "シム (Curve2)": {
        "path": LITE_DIR / "sim_curve2.lite.mcap",
        "kinematic": "/localization/kinematic_state",
        "accel": "/localization/acceleration",
        "cmd": "/control/trajectory_follower/control_cmd",
        "color": "#e05c00",
        "lw": 2.0,
        "ls": "--",
        "marker": "s",
        "ms": 4,
        "real": False,
    },
}

T_PRE, T_POST = -2.0, 30.0


# ---------------------------------------------------------------------------
# MCAP ローダー
# ---------------------------------------------------------------------------

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
    if df.empty:
        return df
    df = df.copy()
    df["t"] = (df["t_ns"] - t0_ns) / 1e9
    return df[df["t"] >= 0].reset_index(drop=True)


def _find_curve2_launch(df_vel):
    """カーブ②前の一時停止終了時刻 [s] を返す。"""
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
    """シムの発進時刻（velocity が初めて threshold m/s を超えた時刻）を返す。

    min_t 以降を対象にし、Autoware 起動直後の瞬間スパイクを除外する。
    """
    moving = df_vel[(df_vel["lon_vel"] > threshold) & (df_vel["t"] >= min_t)]
    if moving.empty:
        return None
    return float(moving["t"].iloc[0])


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


def _clip(df, t_launch):
    if df.empty or "t" not in df.columns:
        df2 = df.copy()
        df2["tr"] = pd.Series(dtype=float)
        return df2
    mask = (df["t"] >= t_launch + T_PRE) & (df["t"] <= t_launch + T_POST)
    sub = df[mask].copy()
    sub["tr"] = sub["t"] - t_launch
    return sub.reset_index(drop=True)


def _save(fig, name):
    FIGS_DIR.mkdir(parents=True, exist_ok=True)
    for ext in ("png", "pdf"):
        p = FIGS_DIR / f"{name}.{ext}"
        fig.savefig(str(p), dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  保存: {name}.{{png,pdf}}")


# ---------------------------------------------------------------------------
# 累積距離計算ユーティリティ
# ---------------------------------------------------------------------------

def _add_cumulative_dist(df_kin):
    """clip済み kinematic データに累積距離 [m] 列を追加（tr=0 を原点に正規化）."""
    if df_kin.empty:
        return df_kin.assign(dist=pd.Series(dtype=float))
    df = df_kin.copy()
    x, y = df["x"].values, df["y"].values
    step = np.sqrt(np.diff(x, prepend=x[0]) ** 2 + np.diff(y, prepend=y[0]) ** 2)
    raw = np.cumsum(step)
    idx0 = int(np.argmin(np.abs(df["tr"].values)))
    df["dist"] = raw - raw[idx0]
    return df


def _add_dist_from_kin(df, df_kin):
    """df_kin の (tr, dist) を線形補間して df に dist 列を追加する."""
    if df.empty or df_kin.empty or "dist" not in df_kin.columns:
        return df.assign(dist=np.nan)
    df = df.copy()
    df["dist"] = np.interp(
        df["tr"].values,
        df_kin["tr"].values,
        df_kin["dist"].values,
        left=np.nan,
        right=np.nan,
    )
    return df


# ---------------------------------------------------------------------------
# データ読み込み
# ---------------------------------------------------------------------------

def load_data():
    loaded = {}
    for label, cfg in LOGS.items():
        mcap_path = cfg["path"]
        print(f"  [{label}] {mcap_path.name}")

        df_vel = _load_velocity(mcap_path)

        if cfg["real"]:
            df_opmode = _load_opmode(mcap_path)
            if not df_opmode.empty:
                t0_ns = int(df_opmode[df_opmode["mode"] == 2]["t_ns"].iloc[0])
            else:
                t0_ns = int(df_vel[df_vel["lon_vel"] > 0.1]["t_ns"].iloc[0])
            df_vel = _align(df_vel, t0_ns)
            t_launch = _find_curve2_launch(df_vel)
            if t_launch is None:
                raise RuntimeError(f"[{label}] カーブ②前の停止を検出できませんでした")
            print(f"    カーブ②発進 t={t_launch:.1f}s")
        else:
            # シムは t0_ns を最初のタイムスタンプとし、発進時刻を t_launch とする
            t0_ns = int(df_vel["t_ns"].iloc[0])
            df_vel = _align(df_vel, t0_ns)
            # 信号停止がある場合は _find_curve2_launch（停止後発進）を優先使用
            t_launch = _find_curve2_launch(df_vel)
            if t_launch is not None:
                print(f"    シム: 信号停止後発進 t={t_launch:.1f}s")
            else:
                t_launch = _find_sim_launch(df_vel)
                if t_launch is None:
                    t_launch = 0.0
                    print(f"    シム: 発進検出失敗 → t=0 を使用（診断用）")
                else:
                    print(f"    シム: 発進検出 t={t_launch:.1f}s")

        df_steer   = _align(_load_steering(mcap_path),                   t0_ns)
        df_kin     = _align(_load_kinematic(mcap_path, cfg["kinematic"]), t0_ns)
        df_accel   = _align(_load_accel(mcap_path, cfg["accel"]),         t0_ns)
        df_cmd     = _align(_load_cmd(mcap_path, cfg["cmd"]),             t0_ns)

        # kin に累積距離を追加し（tr=0 を原点）、他のDFへも補間で追加
        kin_clipped   = _add_cumulative_dist(_clip(df_kin,   t_launch))
        vel_clipped   = _add_dist_from_kin(_clip(df_vel,   t_launch), kin_clipped)
        steer_clipped = _add_dist_from_kin(_clip(df_steer, t_launch), kin_clipped)
        accel_clipped = _add_dist_from_kin(_clip(df_accel, t_launch), kin_clipped)
        cmd_clipped   = _add_dist_from_kin(_clip(df_cmd,   t_launch), kin_clipped)

        loaded[label] = {
            "cfg":      cfg,
            "t_launch": t_launch,
            "vel":      vel_clipped,
            "steer":    steer_clipped,
            "kin":      kin_clipped,
            "accel":    accel_clipped,
            "cmd":      cmd_clipped,
        }
    return loaded


# ---------------------------------------------------------------------------
# 軌跡比較プロット（地図 + 発進点マーカー）
# ---------------------------------------------------------------------------

def _draw_map(ax):
    """地図wayをaxに描画する（共通処理）。"""
    pass  # wayは呼び出し元から渡す


def plot_trajectory(loaded, map_ways):
    from matplotlib.collections import LineCollection
    from matplotlib.colors import Normalize
    import matplotlib.cm as cm

    # ---- メインプロット（左）とズームパネル（右）で構成 ----
    fig = plt.figure(figsize=(18, 10))
    fig.suptitle("カーブ②集中比較: 軌跡（★=発進点、発進後30s）", fontsize=12)
    ax    = fig.add_axes([0.04, 0.06, 0.50, 0.88])   # 全体
    ax_z  = fig.add_axes([0.56, 0.06, 0.42, 0.88])   # ズーム（カーブ本体）

    # カーブ②中心周辺
    ZOOM_CX, ZOOM_CY, ZOOM_M = 89303, 43075, 50

    for ax_cur, xlim, ylim in [
        (ax,   (CURVE2_CX - CURVE2_MARGIN, CURVE2_CX + CURVE2_MARGIN),
                (CURVE2_CY - CURVE2_MARGIN, CURVE2_CY + CURVE2_MARGIN)),
        (ax_z, (ZOOM_CX - ZOOM_M, ZOOM_CX + ZOOM_M),
                (ZOOM_CY - ZOOM_M, ZOOM_CY + ZOOM_M)),
    ]:
        if map_ways:
            for pts in map_ways:
                wx, wy = pts[:, 0], pts[:, 1]
                if wx.max() < xlim[0] or wx.min() > xlim[1]:
                    continue
                if wy.max() < ylim[0] or wy.min() > ylim[1]:
                    continue
                ax_cur.plot(wx, wy, color="#cccccc", lw=0.6, zorder=1)

        for label, d in loaded.items():
            cfg = d["cfg"]
            kin = d["kin"]
            if kin.empty:
                continue

            xs  = kin["x"].values
            ys  = kin["y"].values
            trs = kin["tr"].values

            # 時間で色付けした折れ線（LineCollection）
            pts_arr   = np.array([xs, ys]).T.reshape(-1, 1, 2)
            segs      = np.concatenate([pts_arr[:-1], pts_arr[1:]], axis=1)
            t_mid     = (trs[:-1] + trs[1:]) / 2.0
            norm      = Normalize(vmin=-2, vmax=25)
            cmap_name = "Blues" if d["cfg"]["real"] else "Oranges"
            cmap      = cm.get_cmap(cmap_name)
            colors    = cmap(norm(t_mid))
            # real は濃い青系、sim はオレンジ系
            lc = LineCollection(segs, colors=colors, lw=2.5 if d["cfg"]["real"] else 2.0,
                                 linestyle=cfg["ls"], zorder=3, label=f"_{label}_lc")
            ax_cur.add_collection(lc)

            # 単色の凡例用ライン（dummy）
            ax_cur.plot([], [], color=cfg["color"], lw=cfg["lw"], ls=cfg["ls"], label=label)

            # 発進点★
            row0 = kin.iloc[(kin["tr"]).abs().argsort().iloc[:1]]
            ax_cur.plot(row0["x"].values[0], row0["y"].values[0], "*",
                        color=cfg["color"], ms=14, zorder=6,
                        markeredgecolor="white", markeredgewidth=0.5)

            # 時刻ラベル（0, 5, 10, 15, 20s）
            for t_mark in [0, 5, 10, 15, 20]:
                row = kin.iloc[(kin["tr"] - t_mark).abs().argsort().iloc[:1]]
                if row.empty:
                    continue
                rx, ry = row["x"].values[0], row["y"].values[0]
                ax_cur.plot(rx, ry, "o", color=cfg["color"], ms=5, zorder=5,
                            markeredgecolor="white", markeredgewidth=0.5)
                ax_cur.annotate(f"{t_mark}s",
                                (rx, ry),
                                textcoords="offset points",
                                xytext=(5, 3) if d["cfg"]["real"] else (-18, -12),
                                fontsize=8, color=cfg["color"], fontweight="bold")

        # ---- 同時刻点間の乖離矢印（ズームパネルのみ）----
        if ax_cur is ax_z:
            real_kin = loaded.get("実機", {}).get("kin")
            sim_kin  = loaded.get("シム (Curve2)", {}).get("kin")
            if real_kin is not None and sim_kin is not None:
                for t_mark in [5, 8, 10, 12, 14, 16]:
                    idx_r = (real_kin["tr"] - t_mark).abs().idxmin()
                    idx_s = (sim_kin["tr"]  - t_mark).abs().idxmin()
                    rx, ry = real_kin.loc[idx_r, ["x", "y"]]
                    sx, sy = sim_kin.loc[idx_s,  ["x", "y"]]
                    dist = math.sqrt((sx-rx)**2 + (sy-ry)**2)
                    ax_z.annotate("",
                                  xy=(sx, sy), xytext=(rx, ry),
                                  arrowprops=dict(arrowstyle="->", color="red",
                                                  lw=1.5, shrinkA=0, shrinkB=0),
                                  zorder=8)
                    mx, my = (rx+sx)/2, (ry+sy)/2
                    ax_z.text(mx+1, my+1, f"{dist:.1f}m",
                              fontsize=7, color="red", fontweight="bold",
                              bbox=dict(boxstyle="round,pad=0.1", fc="white", alpha=0.7))

        ax_cur.set_xlim(xlim)
        ax_cur.set_ylim(ylim)
        ax_cur.set_aspect("equal")
        ax_cur.set_xlabel("x [m]")
        ax_cur.set_ylabel("y [m]")
        ax_cur.legend(fontsize=9)
        ax_cur.grid(True, lw=0.4, alpha=0.5)

    ax.set_title("全体（80m視野）")
    ax_z.set_title("カーブ本体ズーム（50m視野）- 赤矢印＝同時刻位置差")

    # カラーバー（時刻）
    sm = plt.cm.ScalarMappable(cmap=cm.get_cmap("Greys"), norm=Normalize(vmin=-2, vmax=25))
    sm.set_array([])
    cbar = fig.colorbar(sm, ax=ax_z, fraction=0.03, pad=0.01)
    cbar.set_label("発進からの時刻 [s]")

    _save(fig, "c2_trajectory")


# ---------------------------------------------------------------------------
# 時系列比較（速度・加速度・ステアリング）— 時刻版・距離版共通実装
# ---------------------------------------------------------------------------

def _plot_timeseries_impl(loaded, x_col, xlabel, name):
    x_desc = "発進 t=0 に揃え" if x_col == "tr" else "発進点 dist=0 に揃え"
    fig, axes = plt.subplots(3, 2, figsize=(15, 12))
    fig.suptitle(f"カーブ②集中比較: 時系列（{x_desc}）", fontsize=12)

    ax_vel_r, ax_vel_c = axes[0, 0], axes[0, 1]
    ax_acc_r, ax_acc_c = axes[1, 0], axes[1, 1]
    ax_str_r, ax_str_c = axes[2, 0], axes[2, 1]

    for label, d in loaded.items():
        cfg    = d["cfg"]
        kw     = dict(color=cfg["color"], lw=cfg["lw"], ls=cfg["ls"])
        kw_cmd = dict(color=cfg["color"], lw=1.2, ls=":", alpha=0.65)

        vel   = d["vel"]
        accel = d["accel"]
        steer = d["steer"]
        cmd   = d["cmd"]

        if x_col in vel.columns:
            ax_vel_r.plot(vel[x_col].values, vel["lon_vel"].values, label=label, **kw)
        if x_col in accel.columns:
            ax_acc_r.plot(accel[x_col].values, accel["accel"].values, label=label, **kw)
        if x_col in steer.columns:
            ax_str_r.plot(steer[x_col].values, np.degrees(steer["steer"].values), label=label, **kw)

        if not cmd.empty and x_col in cmd.columns:
            ax_vel_c.plot(cmd[x_col].values, cmd["cmd_vel"].values,
                          label=f"{label} 指令", **kw_cmd)
            ax_acc_c.plot(cmd[x_col].values, cmd["cmd_accel"].values,
                          label=f"{label} 指令", **kw_cmd)
            ax_str_c.plot(cmd[x_col].values, np.degrees(cmd["cmd_steer"].values),
                          label=f"{label} 指令", **kw_cmd)

    for ax in axes.flat:
        if x_col == "tr":
            ax.axvline(0, color="gray", lw=0.8, ls="--", alpha=0.6)
        ax.grid(True, lw=0.4)
        ax.legend(fontsize=8)
        ax.set_xlabel(xlabel)

    ax_vel_r.set_title("速度 — 応答");                   ax_vel_r.set_ylabel("m/s")
    ax_vel_c.set_title("速度 — 指令");                   ax_vel_c.set_ylabel("m/s")
    ax_acc_r.set_title("加速度 — 応答 (IMU/オドメトリ)"); ax_acc_r.set_ylabel("m/s²")
    ax_acc_c.set_title("加速度 — 指令");                 ax_acc_c.set_ylabel("m/s²")
    ax_str_r.set_title("ステアリング角 — 応答");          ax_str_r.set_ylabel("deg")
    ax_str_c.set_title("ステアリング角 — 指令");          ax_str_c.set_ylabel("deg")

    fig.tight_layout()
    _save(fig, name)


def plot_timeseries(loaded):
    _plot_timeseries_impl(loaded, "tr", "発進からの時刻 [s]", "c2_timeseries")


def plot_timeseries_vs_dist(loaded):
    _plot_timeseries_impl(loaded, "dist", "発進からの累積距離 [m]", "c2_timeseries_vs_dist")


# ---------------------------------------------------------------------------
# ステア詳細比較（指令 vs 応答 + 追従誤差）
# ---------------------------------------------------------------------------

def plot_steering_detail(loaded):
    fig = plt.figure(figsize=(16, 12))
    fig.suptitle("カーブ②集中比較: ステアリング詳細", fontsize=12)
    gs = fig.add_gridspec(3, 2, hspace=0.45, wspace=0.30)
    ax_ovl  = fig.add_subplot(gs[0, :])
    ax_err  = fig.add_subplot(gs[1, 0])
    ax_rate = fig.add_subplot(gs[1, 1])
    ax_yaw  = fig.add_subplot(gs[2, 0])
    ax_vel  = fig.add_subplot(gs[2, 1])

    for label, d in loaded.items():
        cfg   = d["cfg"]
        steer = d["steer"]
        cmd   = d["cmd"]
        kin   = d["kin"]
        vel   = d["vel"]
        kw    = dict(color=cfg["color"], lw=cfg["lw"], ls=cfg["ls"])

        t_s     = steer["tr"].values
        s_deg   = np.degrees(steer["steer"].values)

        ax_ovl.plot(t_s, s_deg, label=f"{label} 応答", **kw)
        if not cmd.empty:
            ax_ovl.plot(cmd["tr"].values, np.degrees(cmd["cmd_steer"].values),
                        color=cfg["color"], lw=1.2, ls=":", alpha=0.65,
                        label=f"{label} 指令")

        if not cmd.empty and len(t_s) > 0:
            cmd_interp = np.interp(t_s, cmd["tr"].values,
                                   np.degrees(cmd["cmd_steer"].values))
            err = s_deg - cmd_interp
            ax_err.plot(t_s, err, label=label, **kw)

        if len(t_s) > 1:
            dt = np.diff(t_s)
            rate = np.diff(s_deg) / np.where(dt > 1e-6, dt, np.nan)
            rate = np.where(np.abs(rate) < 200, rate, np.nan)
            ax_rate.plot(t_s[1:], rate, label=label, **kw)

        if len(kin) > 1:
            t_k  = kin["tr"].values
            yaw_u = np.unwrap(kin["yaw"].values)
            t0_i  = int(np.argmin(np.abs(t_k)))
            yaw_cum = np.degrees(yaw_u - yaw_u[t0_i])
            ax_yaw.plot(t_k, yaw_cum, label=label, **kw)

        ax_vel.plot(vel["tr"].values, vel["lon_vel"].values, label=label, **kw)
        if not cmd.empty:
            ax_vel.plot(cmd["tr"].values, cmd["cmd_vel"].values,
                        color=cfg["color"], lw=1.2, ls=":", alpha=0.65,
                        label=f"{label} 指令")

    for ax in (ax_ovl, ax_err, ax_rate, ax_yaw, ax_vel):
        ax.axvline(0, color="gray", lw=0.8, ls="--", alpha=0.6)
        ax.axhline(0, color="gray", lw=0.5)
        ax.grid(True, lw=0.4)
        ax.legend(fontsize=8, ncol=2)
        ax.set_xlabel("発進からの時刻 [s]")

    ax_ovl.set_title("ステアリング角 指令（点線）vs 応答（実線/破線）")
    ax_ovl.set_ylabel("deg")

    ax_err.set_title("追従誤差（応答 − 指令）[deg]")
    ax_err.set_ylabel("deg")

    ax_rate.set_title("ステア変化率 [deg/s]")
    ax_rate.set_ylabel("deg/s")
    ax_rate.set_ylim(-30, 30)

    ax_yaw.set_title("yaw 累積変化量（t=0 基準）[deg]")
    ax_yaw.set_ylabel("deg")

    ax_vel.set_title("速度（応答 vs 指令）")
    ax_vel.set_ylabel("m/s")

    fig.tight_layout()
    _save(fig, "c2_steering_detail")


# ---------------------------------------------------------------------------
# 速度応答（cmd vs actual）— 時刻版・距離版共通実装
# ---------------------------------------------------------------------------

def _plot_velocity_response_impl(loaded, x_col, xlabel, name):
    x_desc = "時刻基準" if x_col == "tr" else "距離基準"
    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    fig.suptitle(f"カーブ②集中比較: 速度応答（cmd vs actual）— {x_desc}", fontsize=12)

    ax_vel = axes[0]
    ax_lag = axes[1]

    for label, d in loaded.items():
        cfg = d["cfg"]
        vel = d["vel"]
        cmd = d["cmd"]
        kw  = dict(color=cfg["color"], lw=cfg["lw"], ls=cfg["ls"])

        if vel.empty or cmd.empty:
            continue
        if x_col not in vel.columns or x_col not in cmd.columns:
            continue

        x_v   = vel[x_col].values
        v_act = vel["lon_vel"].values
        # cmd_vel は時刻軸で補間（tr を使うことで x_col 非依存）
        v_cmd = np.interp(vel["tr"].values, cmd["tr"].values, cmd["cmd_vel"].values)

        ax_vel.plot(x_v, v_act, label=f"{label} 応答", **kw)
        ax_vel.plot(cmd[x_col].values, cmd["cmd_vel"].values,
                    color=cfg["color"], lw=1.2, ls=":", alpha=0.65,
                    label=f"{label} 指令")

        lag = v_act - v_cmd
        ax_lag.plot(x_v, lag, label=label, **kw)

    if x_col == "tr":
        ax_vel.axvline(0, color="gray", lw=0.8, ls="--", alpha=0.6)
        ax_lag.axvline(0, color="gray", lw=0.8, ls="--", alpha=0.6)
    ax_vel.set_ylabel("m/s")
    ax_vel.set_title("速度 応答（実線）vs 指令（点線）")
    ax_vel.legend(fontsize=8, ncol=2)
    ax_vel.grid(True, lw=0.4)

    ax_lag.axhline(0, color="gray", lw=0.5)
    ax_lag.set_ylabel("actual − cmd [m/s]")
    ax_lag.set_xlabel(xlabel)
    ax_lag.set_title("速度追従誤差（応答 − 指令）")
    ax_lag.legend(fontsize=8)
    ax_lag.grid(True, lw=0.4)

    fig.tight_layout()
    _save(fig, name)


def plot_velocity_response(loaded):
    _plot_velocity_response_impl(loaded, "tr", "発進からの時刻 [s]", "c2_velocity_response")


def plot_velocity_response_vs_dist(loaded):
    _plot_velocity_response_impl(loaded, "dist", "発進からの累積距離 [m]", "c2_velocity_response_vs_dist")


# ---------------------------------------------------------------------------
# 軌跡乖離（実機を基準にしたシムの最近傍距離）— 時刻版・距離版共通実装
# ---------------------------------------------------------------------------

def _plot_deviation_impl(loaded, x_col, xlabel, name):
    real_kin = loaded.get("実機", {}).get("kin")
    if real_kin is None or real_kin.empty:
        warnings.warn("実機データなし: deviation プロットをスキップ")
        return

    from scipy.spatial import cKDTree
    ref_xy = real_kin[["x", "y"]].values
    tree   = cKDTree(ref_xy)

    x_desc = "時刻基準" if x_col == "tr" else "距離基準"
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    fig.suptitle(f"カーブ②集中比較: 実機からの軌跡乖離 — {x_desc}", fontsize=12)

    ax_map = axes[0]
    ax_dev = axes[1]

    for label, d in loaded.items():
        cfg = d["cfg"]
        kin = d["kin"]
        if kin.empty:
            continue

        q_xy = kin[["x", "y"]].values
        dists, _ = tree.query(q_xy)

        ax_map.plot(kin["x"].values, kin["y"].values,
                    color=cfg["color"], lw=cfg["lw"], ls=cfg["ls"],
                    label=label, zorder=3)

        if x_col in kin.columns:
            ax_dev.plot(kin[x_col].values, dists,
                        color=cfg["color"], lw=cfg["lw"], ls=cfg["ls"],
                        label=f"{label} (平均={dists.mean():.2f}m, 最大={dists.max():.2f}m)")

    ax_map.set_xlim(CURVE2_CX - CURVE2_MARGIN, CURVE2_CX + CURVE2_MARGIN)
    ax_map.set_ylim(CURVE2_CY - CURVE2_MARGIN, CURVE2_CY + CURVE2_MARGIN)
    ax_map.set_aspect("equal")
    ax_map.set_xlabel("x [m]")
    ax_map.set_ylabel("y [m]")
    ax_map.legend(fontsize=9)
    ax_map.grid(True, lw=0.4, alpha=0.5)
    ax_map.set_title("軌跡比較")

    if x_col == "tr":
        ax_dev.axvline(0, color="gray", lw=0.8, ls="--", alpha=0.6)
    ax_dev.set_title("実機軌跡からの最近傍距離 [m]")
    ax_dev.set_xlabel(xlabel)
    ax_dev.set_ylabel("距離 [m]")
    ax_dev.legend(fontsize=9)
    ax_dev.grid(True, lw=0.4)

    fig.tight_layout()
    _save(fig, name)


def plot_deviation(loaded):
    _plot_deviation_impl(loaded, "tr", "発進からの時刻 [s]", "c2_deviation")


def plot_deviation_vs_dist(loaded):
    _plot_deviation_impl(loaded, "dist", "発進からの累積距離 [m]", "c2_deviation_vs_dist")


# ---------------------------------------------------------------------------
# 数値レポート
# ---------------------------------------------------------------------------

def build_report(loaded):
    lines = ["# カーブ②集中比較レポート\n"]

    # 速度統計
    lines.append("## 速度統計（発進後30s）\n")
    lines.append("| ログ | 平均 [m/s] | 最大 [m/s] |")
    lines.append("|---|---|---|")
    for label, d in loaded.items():
        vel = d["vel"]
        if vel.empty:
            lines.append(f"| {label} | N/A | N/A |")
            continue
        lines.append(f"| {label} | {vel['lon_vel'].mean():.3f} | {vel['lon_vel'].max():.3f} |")
    lines.append("")

    # ステア追従 RMSE
    lines.append("## ステア追従 RMSE（発進後30s）\n")
    lines.append("| ログ | RMSE [deg] | 平均誤差 [deg] |")
    lines.append("|---|---|---|")
    for label, d in loaded.items():
        steer = d["steer"]
        cmd   = d["cmd"]
        if steer.empty or cmd.empty:
            lines.append(f"| {label} | N/A | N/A |")
            continue
        cmd_i = np.interp(steer["tr"].values, cmd["tr"].values,
                          np.degrees(cmd["cmd_steer"].values))
        err  = np.degrees(steer["steer"].values) - cmd_i
        rmse = float(np.sqrt(np.nanmean(err**2)))
        mean_err = float(np.nanmean(err))
        lines.append(f"| {label} | {rmse:.4f} | {mean_err:.4f} |")
    lines.append("")

    # 軌跡乖離（実機基準）
    lines.append("## 軌跡乖離（実機軌跡からの最近傍距離）\n")
    lines.append("| ログ | 平均 [m] | 最大 [m] |")
    lines.append("|---|---|---|")
    real_kin = loaded.get("実機", {}).get("kin")
    if real_kin is not None and not real_kin.empty:
        from scipy.spatial import cKDTree
        ref_xy = real_kin[["x", "y"]].values
        tree = cKDTree(ref_xy)
        for label, d in loaded.items():
            if label == "実機":
                lines.append(f"| {label} | — (基準) | — |")
                continue
            kin = d["kin"]
            if kin.empty:
                lines.append(f"| {label} | N/A | N/A |")
                continue
            q_xy = kin[["x", "y"]].values
            dists, _ = tree.query(q_xy)
            lines.append(f"| {label} | {dists.mean():.3f} | {dists.max():.3f} |")
    lines.append("")

    # 速度応答誤差（cmd vs actual）
    lines.append("## 速度追従誤差（actual − cmd、発進後30s）\n")
    lines.append("| ログ | 平均誤差 [m/s] | RMSE [m/s] | 遅相比率 (actual<cmd の割合) |")
    lines.append("|---|---|---|---|")
    for label, d in loaded.items():
        vel = d["vel"]
        cmd = d["cmd"]
        if vel.empty or cmd.empty:
            lines.append(f"| {label} | N/A | N/A | N/A |")
            continue
        t_v   = vel["tr"].values
        v_act = vel["lon_vel"].values
        v_cmd = np.interp(t_v, cmd["tr"].values, cmd["cmd_vel"].values)
        lag   = v_act - v_cmd
        rmse  = float(np.sqrt(np.nanmean(lag**2)))
        mean_l = float(np.nanmean(lag))
        lag_ratio = float(np.mean(lag < 0))
        lines.append(f"| {label} | {mean_l:.3f} | {rmse:.3f} | {lag_ratio:.2%} |")
    lines.append("")

    # カーブ②進入速度比較
    lines.append("## カーブ②進入速度（t=5〜15s 平均）\n")
    lines.append("| ログ | 平均速度 [m/s] |")
    lines.append("|---|---|")
    for label, d in loaded.items():
        vel = d["vel"]
        seg = vel[(vel["tr"] >= 5) & (vel["tr"] <= 15)]
        if seg.empty:
            lines.append(f"| {label} | N/A |")
        else:
            lines.append(f"| {label} | {seg['lon_vel'].mean():.3f} |")
    lines.append("")

    # 計画速度（cmd_vel）統計
    lines.append("## 計画速度（cmd_vel）統計（発進後30s）\n")
    lines.append("| ログ | 平均 cmd [m/s] | 最大 cmd [m/s] | t=5〜15s 平均 cmd [m/s] |")
    lines.append("|---|---|---|---|")
    for label, d in loaded.items():
        cmd = d["cmd"]
        if cmd.empty:
            lines.append(f"| {label} | N/A | N/A | N/A |")
            continue
        seg = cmd[(cmd["tr"] >= 5) & (cmd["tr"] <= 15)]
        seg_mean = seg["cmd_vel"].mean() if not seg.empty else float("nan")
        lines.append(
            f"| {label} | {cmd['cmd_vel'].mean():.3f} | {cmd['cmd_vel'].max():.3f} | {seg_mean:.3f} |"
        )
    lines.append("")

    return "\n".join(lines)


# ---------------------------------------------------------------------------
# メイン
# ---------------------------------------------------------------------------

def main():
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    FIGS_DIR.mkdir(parents=True, exist_ok=True)

    print("=== データ読み込み ===")
    loaded = load_data()

    print("\n=== 地図読み込み ===")
    map_ways = _load_map_ways()
    print(f"  {len(map_ways)} ways ロード完了")

    print("\n=== プロット生成 ===")
    plot_trajectory(loaded, map_ways)
    plot_timeseries(loaded)
    plot_timeseries_vs_dist(loaded)
    plot_steering_detail(loaded)
    plot_velocity_response(loaded)
    plot_velocity_response_vs_dist(loaded)
    plot_deviation(loaded)
    plot_deviation_vs_dist(loaded)

    print("\n=== レポート生成 ===")
    report = build_report(loaded)
    report_path = OUT_DIR / "curve2_report.md"
    report_path.write_text(report, encoding="utf-8")
    print(f"  保存: {report_path}")

    print("\n完了。出力先:", FIGS_DIR)


if __name__ == "__main__":
    main()
