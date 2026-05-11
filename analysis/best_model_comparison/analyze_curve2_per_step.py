#!/usr/bin/env python3
"""カーブ② 区間の per-step delta 分析（C++ 車両モデル使用）.

手法:
  各制御コマンド区間 (t_k, t_{k+1}) について:
    1. C++ 車両モデルを t_k の実機状態にリセット（累積誤差を排除）
    2. 実機制御コマンドを ZOH で適用
    3. 区間終端の予測状態を実機状態と比較（車両ローカル座標系）

使用モデル: DELAY_STEER_ACC_GEARED_WO_FALL_GUARD (C++ ctypes 経由)
  ライブラリ: libvehicle_model_wrapper.so (vehicle_model_c_wrapper.cpp)
  パラメータ: analysis/best_model_description/config/simulator_model.param.yaml

出力:
  comparison/curve2_per_step/
    overview.png, error_timeseries.png, error_vs_speed.png,
    map_distribution.png, summary.txt, per_step_delta.csv
"""

import ctypes
import math
import subprocess
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

BASE      = Path(__file__).parent
LITE_DIR  = BASE / "lite"
OUT_DIR   = BASE / "comparison" / "curve2_per_step"
REAL_MCAP = LITE_DIR / "real.lite.mcap"
MAP_DIR   = Path.home() / ".webauto/simulation/data/map/x2_dev/2231"

# カーブ②発進前後の解析窓
T_PRE  = 3.0   # [s]
T_POST = 35.0  # [s]

# 車両モデルパラメータ (simulator_model.param.yaml + vehicle_info.param.yaml)
PARAMS = {
    "wheelbase":          4.76012,   # wheel_base [m]
    "vel_lim":            50.0,
    "vel_rate_lim":        7.0,
    "steer_lim":           1.0,
    "steer_rate_lim":      5.0,
    "acc_time_delay":      0.101,
    "acc_time_constant":   0.2589,
    "steer_time_delay":    0.0315,
    "steer_time_constant": 0.4983,
    "steer_dead_band":     0.0,
    "steer_bias":          0.01,
    "sub_dt":              1.0 / 30.0,
}

SUB_DT = PARAMS["sub_dt"]   # 積分ステップ幅 [s]（FMU_DT 相当）


# ---------------------------------------------------------------------------
# DELAY_STEER_ACC_GEARED_WO_FALL_GUARD の C++ ctypes ラッパー
# ---------------------------------------------------------------------------

def _build_lib(so: Path) -> None:
    cpp = BASE / "vehicle_model_c_wrapper.cpp"
    if not cpp.exists():
        raise FileNotFoundError(f"{cpp} が見つかりません")
    print(f"  [build] {cpp.name} → {so.name} ...")
    result = subprocess.run(
        [
            "g++", "-shared", "-fPIC", "-O2", "-std=c++17",
            "-I/usr/include/eigen3",
            "-o", str(so), str(cpp),
        ],
        capture_output=True, text=True,
    )
    if result.returncode != 0:
        raise RuntimeError(
            f"コンパイル失敗:\n{result.stderr}"
        )
    print(f"  [build] 完了")


def _load_lib() -> ctypes.CDLL:
    so = BASE / "libvehicle_model_wrapper.so"
    cpp = BASE / "vehicle_model_c_wrapper.cpp"
    # .so が存在しない、または .cpp より古い場合は自動再コンパイル
    if not so.exists() or (cpp.exists() and cpp.stat().st_mtime > so.stat().st_mtime):
        _build_lib(so)
    lib = ctypes.CDLL(str(so))

    c_double = ctypes.c_double
    c_void_p = ctypes.c_void_p

    lib.vm_create.restype  = c_void_p
    lib.vm_create.argtypes = [c_double] * 12

    lib.vm_reset_full.restype  = None
    lib.vm_reset_full.argtypes = [c_void_p] + [c_double] * 6

    # State-only reset (queues untouched) + explicit queue setter
    lib.vm_reset_state.restype  = None
    lib.vm_reset_state.argtypes = [c_void_p] + [c_double] * 6

    lib.vm_set_queues.restype  = None
    lib.vm_set_queues.argtypes = [
        c_void_p,
        ctypes.POINTER(c_double), ctypes.c_int,
        ctypes.POINTER(c_double), ctypes.c_int,
    ]

    lib.vm_get_acc_q_size.restype  = ctypes.c_int
    lib.vm_get_acc_q_size.argtypes = [c_void_p]
    lib.vm_get_steer_q_size.restype  = ctypes.c_int
    lib.vm_get_steer_q_size.argtypes = [c_void_p]

    lib.vm_set_input.restype  = None
    lib.vm_set_input.argtypes = [c_void_p, c_double, c_double]

    lib.vm_step.restype  = None
    lib.vm_step.argtypes = [c_void_p]

    for fn in ("vm_get_x", "vm_get_y", "vm_get_yaw",
               "vm_get_vx", "vm_get_steer", "vm_get_ax"):
        getattr(lib, fn).restype  = c_double
        getattr(lib, fn).argtypes = [c_void_p]

    lib.vm_destroy.restype  = None
    lib.vm_destroy.argtypes = [c_void_p]

    return lib


class VehicleModel:
    """DELAY_STEER_ACC_GEARED_WO_FALL_GUARD の C++ ctypes ラッパー."""

    _lib: ctypes.CDLL | None = None

    @classmethod
    def _get_lib(cls) -> ctypes.CDLL:
        if cls._lib is None:
            cls._lib = _load_lib()
        return cls._lib

    def __init__(self, params: dict, sub_dt: float):
        p = params
        lib = self._get_lib()
        self._lib = lib
        self._ptr = lib.vm_create(
            p["vel_lim"],
            p["steer_lim"],
            p["vel_rate_lim"],
            p["steer_rate_lim"],
            p["wheelbase"],
            sub_dt,
            p["acc_time_delay"],
            p["acc_time_constant"],
            p["steer_time_delay"],
            p["steer_time_constant"],
            p["steer_dead_band"],
            p["steer_bias"],
        )
        self._sub_dt    = sub_dt
        self._steer_bias = p["steer_bias"]

    def __del__(self):
        if hasattr(self, "_ptr") and self._ptr and hasattr(self, "_lib") and self._lib:
            self._lib.vm_destroy(self._ptr)
            self._ptr = None

    @property
    def acc_q_size(self) -> int:
        return self._lib.vm_get_acc_q_size(self._ptr)

    @property
    def steer_q_size(self) -> int:
        return self._lib.vm_get_steer_q_size(self._ptr)

    def reset_with_history(self,
                           x: float, y: float, yaw: float, vx: float,
                           steer_actual: float, ax: float,
                           acc_history: list[float],
                           steer_history: list[float]) -> None:
        """状態と delay queue を実際の過去コマンド履歴でリセット。

        acc_history   : accel_des [oldest→newest], len == acc_q_size
        steer_history : steer_des [oldest→newest], len == steer_q_size
        """
        self._lib.vm_reset_state(self._ptr, x, y, yaw, vx, steer_actual, ax)

        n_acc   = len(acc_history)
        n_steer = len(steer_history)
        ArrAcc   = (ctypes.c_double * n_acc)(*acc_history)
        ArrSteer = (ctypes.c_double * n_steer)(*steer_history)
        self._lib.vm_set_queues(
            self._ptr,
            ArrAcc,   ctypes.c_int(n_acc),
            ArrSteer, ctypes.c_int(n_steer),
        )

    def step(self, accel_des: float, steer_des: float) -> None:
        """Euler 1 ステップ積分（sub_dt 秒）。"""
        self._lib.vm_set_input(self._ptr, accel_des, steer_des)
        self._lib.vm_step(self._ptr)

    @property
    def x(self) -> float:     return self._lib.vm_get_x(self._ptr)
    @property
    def y(self) -> float:     return self._lib.vm_get_y(self._ptr)
    @property
    def yaw(self) -> float:   return self._lib.vm_get_yaw(self._ptr)
    @property
    def vx(self) -> float:    return self._lib.vm_get_vx(self._ptr)
    @property
    def steer_state(self) -> float:
        return self._lib.vm_get_steer(self._ptr) - self._steer_bias


# ---------------------------------------------------------------------------
# データ読み込み
# ---------------------------------------------------------------------------

def _iter_msgs(path: Path, topics: list[str]):
    with open(path, "rb") as f:
        reader = make_reader(f, decoder_factories=[DecoderFactory()])
        yield from reader.iter_decoded_messages(topics=topics)


def load_real_bag(path: Path) -> dict[str, pd.DataFrame]:
    """real.lite.mcap から必要トピックを読み込む。"""
    rows_mode, rows_vel, rows_steer, rows_kin, rows_acc, rows_cmd = [], [], [], [], [], []

    topics = [
        "/system/operation_mode/state",
        "/vehicle/status/velocity_status",
        "/vehicle/status/steering_status",
        "/sub/localization/kinematic_state",
        "/sub/localization/acceleration",
        "/sub/control/command/control_cmd",
    ]

    for _, channel, msg, ros in _iter_msgs(path, topics):
        t_ns = msg.log_time
        topic = channel.topic
        if topic == "/system/operation_mode/state":
            rows_mode.append({"t_ns": t_ns, "mode": ros.mode})
        elif topic == "/vehicle/status/velocity_status":
            rows_vel.append({"t_ns": t_ns, "vx": ros.longitudinal_velocity})
        elif topic == "/vehicle/status/steering_status":
            rows_steer.append({"t_ns": t_ns, "steer": ros.steering_tire_angle})
        elif topic == "/sub/localization/kinematic_state":
            p = ros.pose.pose.position
            o = ros.pose.pose.orientation
            # quaternion → yaw
            yaw = math.atan2(
                2.0 * (o.w * o.z + o.x * o.y),
                1.0 - 2.0 * (o.y * o.y + o.z * o.z),
            )
            rows_kin.append({
                "t_ns": t_ns, "x": p.x, "y": p.y, "yaw": yaw,
                "vx": ros.twist.twist.linear.x,
                "wz": ros.twist.twist.angular.z,
            })
        elif topic == "/sub/localization/acceleration":
            rows_acc.append({"t_ns": t_ns, "ax": ros.accel.accel.linear.x})
        elif topic == "/sub/control/command/control_cmd":
            rows_cmd.append({
                "t_ns": t_ns,
                "accel_des": ros.longitudinal.acceleration,
                "steer_des": ros.lateral.steering_tire_angle,
            })

    return {
        "mode":  pd.DataFrame(rows_mode),
        "vel":   pd.DataFrame(rows_vel),
        "steer": pd.DataFrame(rows_steer),
        "kin":   pd.DataFrame(rows_kin),
        "acc":   pd.DataFrame(rows_acc),
        "cmd":   pd.DataFrame(rows_cmd),
    }


def find_autonomous_start(data: dict) -> int:
    """AUTONOMOUS モード（mode==2）が最初に現れる t_ns を返す。"""
    df = data["mode"]
    if df.empty:
        raise RuntimeError("operation_mode トピックが空です")
    auto = df[df["mode"] == 2]
    if auto.empty:
        raise RuntimeError("AUTONOMOUS モードが見つかりません")
    return int(auto["t_ns"].iloc[0])


def find_curve2_launch(df_vel: pd.DataFrame) -> float:
    """カーブ②信号前の停止から発進する時刻 [s]（t0 起点）を検出する。"""
    stopped = df_vel[df_vel["vx"] < 0.05]["t"].values
    if len(stopped) == 0:
        return None
    gaps = np.where(np.diff(stopped) > 2.0)[0]
    starts_idx = np.concatenate([[0], gaps + 1])
    ends_idx   = np.concatenate([gaps, [len(stopped) - 1]])
    candidates = []
    for s, e in zip(starts_idx, ends_idx):
        dur   = stopped[e] - stopped[s]
        t_end = stopped[e]
        if dur >= 0.5 and 20.0 <= t_end <= 120.0:
            candidates.append(t_end)
    return float(min(candidates)) if candidates else None


# ---------------------------------------------------------------------------
# per-step delta 分析
# ---------------------------------------------------------------------------

def run_per_step(data: dict, t0_ns: int, t_launch: float, params: dict) -> pd.DataFrame:
    """per-step delta を実行し結果 DataFrame を返す。"""

    # -- タイムスタンプを秒に変換 --
    def to_sec(df: pd.DataFrame) -> pd.DataFrame:
        df = df.copy()
        df["t"] = (df["t_ns"] - t0_ns) / 1e9
        return df.drop(columns=["t_ns"])

    df_kin   = to_sec(data["kin"]).sort_values("t").reset_index(drop=True)
    df_acc   = to_sec(data["acc"]).sort_values("t").reset_index(drop=True)
    df_steer = to_sec(data["steer"]).sort_values("t").reset_index(drop=True)
    df_cmd   = to_sec(data["cmd"]).sort_values("t").reset_index(drop=True)

    t_lo = t_launch - T_PRE
    t_hi = t_launch + T_POST

    # delay queue 分だけ過去コマンドも取得できるよう cmd 窓を広げる
    max_delay_sec = max(params["acc_time_delay"], params["steer_time_delay"]) + SUB_DT
    df_cmd_full = df_cmd[(df_cmd["t"] >= t_lo - max_delay_sec) & (df_cmd["t"] <= t_hi)].reset_index(drop=True)
    df_cmd      = df_cmd[(df_cmd["t"] >= t_lo) & (df_cmd["t"] <= t_hi)].reset_index(drop=True)
    df_kin   = df_kin[(df_kin["t"] >= t_lo - 1) & (df_kin["t"] <= t_hi + 1)].reset_index(drop=True)
    df_acc   = df_acc[(df_acc["t"] >= t_lo - 1) & (df_acc["t"] <= t_hi + 1)].reset_index(drop=True)
    df_steer = df_steer[(df_steer["t"] >= t_lo - 1) & (df_steer["t"] <= t_hi + 1)].reset_index(drop=True)

    if df_cmd.empty:
        raise RuntimeError("制御コマンドが空です")
    if df_kin.empty:
        raise RuntimeError("運動学状態が空です")

    # -- GT を cmd タイムスタンプに線形補間 --
    t_cmd   = df_cmd["t"].values
    t_kin   = df_kin["t"].values
    t_acc   = df_acc["t"].values
    t_steer = df_steer["t"].values

    gt_x   = np.interp(t_cmd, t_kin,   df_kin["x"].values)
    gt_y   = np.interp(t_cmd, t_kin,   df_kin["y"].values)
    gt_yaw = np.interp(t_cmd, t_kin,   np.unwrap(df_kin["yaw"].values))
    gt_vx  = np.interp(t_cmd, t_kin,   df_kin["vx"].values)
    gt_wz  = np.interp(t_cmd, t_kin,   df_kin["wz"].values)
    gt_ax  = np.interp(t_cmd, t_acc,   df_acc["ax"].values)   if not df_acc.empty   else np.zeros_like(t_cmd)
    gt_steer = np.interp(t_cmd, t_steer, df_steer["steer"].values) if not df_steer.empty else np.zeros_like(t_cmd)

    # 運動学ステア: ego_entity_simulation.cpp と同じ初期化方式
    # state(4) = atan(wz * wb / vx)。低速では sensor 値にフォールバック。
    _wb = params["wheelbase"]
    _vx_thresh = 0.5  # [m/s]
    gt_steer_kinematic = np.where(
        gt_vx > _vx_thresh,
        np.arctan(gt_wz * _wb / np.where(gt_vx > _vx_thresh, gt_vx, 1.0)),
        gt_steer,
    )

    # 過去コマンド補間用（queue 分の過去を含む全区間）
    t_cmd_full       = df_cmd_full["t"].values
    accel_des_full   = df_cmd_full["accel_des"].values
    steer_des_full   = df_cmd_full["steer_des"].values

    model = VehicleModel(params, SUB_DT)
    acc_q_size   = model.acc_q_size    # = round(acc_time_delay / SUB_DT)
    steer_q_size = model.steer_q_size  # = round(steer_time_delay / SUB_DT)

    records = []
    n = len(t_cmd)
    for k in range(n - 1):
        interval = t_cmd[k + 1] - t_cmd[k]
        if interval <= 0.001 or interval > 1.0:
            continue

        accel_des = float(df_cmd["accel_des"].iloc[k])
        steer_des = float(df_cmd["steer_des"].iloc[k])

        # -- delay queue に実際の過去コマンド履歴を設定 --
        # acc_q[i] = accel_des at t_k - (acc_q_size - i) * SUB_DT  (oldest→newest)
        # update() はこの oldest を delayed として消費する
        acc_history = [
            float(np.interp(
                t_cmd[k] - (acc_q_size - i) * SUB_DT,
                t_cmd_full, accel_des_full,
                left=accel_des_full[0], right=accel_des_full[-1],
            ))
            for i in range(acc_q_size)
        ]
        steer_history = [
            float(np.interp(
                t_cmd[k] - (steer_q_size - i) * SUB_DT,
                t_cmd_full, steer_des_full,
                left=steer_des_full[0], right=steer_des_full[-1],
            ))
            for i in range(steer_q_size)
        ]

        # -- モデルを t_k の実機状態にリセット（過去履歴を delay queue にセット） --
        # ego_entity_simulation.cpp と同じ: state(4) = atan(wz*wb/vx) をキネマティック初期値として使用。
        # vm_reset_state 内: state(4) = steer_actual - steer_bias → steer_actual = steer_kinematic + steer_bias
        model.reset_with_history(
            x=gt_x[k], y=gt_y[k], yaw=gt_yaw[k], vx=gt_vx[k],
            steer_actual=float(gt_steer_kinematic[k]) + params["steer_bias"], ax=gt_ax[k],
            acc_history=acc_history, steer_history=steer_history,
        )

        # -- interval 分だけ積分 --
        n_sub = max(1, round(interval / SUB_DT))
        for i in range(n_sub):
            model.step(accel_des, steer_des)

        # -- delta 計算 --
        real_dx = gt_x[k + 1] - gt_x[k]
        real_dy = gt_y[k + 1] - gt_y[k]
        sim_dx  = model.x - gt_x[k]
        sim_dy  = model.y - gt_y[k]

        cos_y = math.cos(gt_yaw[k])
        sin_y = math.sin(gt_yaw[k])
        real_ds_long =  real_dx * cos_y + real_dy * sin_y
        real_ds_lat  = -real_dx * sin_y + real_dy * cos_y
        sim_ds_long  =  sim_dx  * cos_y + sim_dy  * sin_y
        sim_ds_lat   = -sim_dx  * sin_y + sim_dy  * cos_y

        sim_steer_kp1 = model.steer_state + params["steer_bias"]
        records.append({
            "timestamp":  t_cmd[k],
            "tr":         t_cmd[k] - t_launch,
            "accel_des":  accel_des,
            "steer_des":  steer_des,
            "interval_sec": interval,
            "pos_x":       gt_x[k],
            "pos_y":       gt_y[k],
            "real_vx":     gt_vx[k],
            "real_steer_k":   gt_steer[k],       # t_k の実機ステア（リセット時の初期値）
            "real_steer_kp1": gt_steer[k + 1],   # t_{k+1} の実機ステア（予測の比較対象）
            "sim_steer_kp1":  sim_steer_kp1,     # モデルが予測した t_{k+1} のステア
            "err_steer":  gt_steer[k + 1] - sim_steer_kp1,  # 予測誤差 [rad]
            "real_ax":     gt_ax[k],
            "sim_vx":      model.vx,
            "real_ds_long": real_ds_long,
            "real_ds_lat":  real_ds_lat,
            "sim_ds_long":  sim_ds_long,
            "sim_ds_lat":   sim_ds_lat,
            "err_ds_long":  real_ds_long - sim_ds_long,
            "err_ds_lat":   real_ds_lat  - sim_ds_lat,
        })

        if (k + 1) % 200 == 0:
            print(f"  {k + 1}/{n - 1} ...")

    return pd.DataFrame(records)


# ---------------------------------------------------------------------------
# プロット
# ---------------------------------------------------------------------------

def _save(fig: plt.Figure, name: str) -> None:
    path = OUT_DIR / f"{name}.png"
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  Saved: {path}")



def plot_overview(df: pd.DataFrame, params: dict) -> None:
    fig, axes = plt.subplots(2, 2, figsize=(14, 9))
    tr = df["tr"].values
    window = max(1, len(df) // 30)

    # 速度
    ax = axes[0, 0]
    ax.plot(tr, df["real_vx"].values, color="blue",  lw=1.2, label="実機 vx")
    ax.plot(tr, df["sim_vx"].values,  color="red",   lw=1.0, ls="--", label="モデル vx")
    ax.axvline(0, color="green", lw=1.0, ls=":", alpha=0.8, label="発進")
    ax.set_xlabel("発進からの時刻 [s]"); ax.set_ylabel("速度 [m/s]")
    ax.set_title("速度: 実機 vs モデル"); ax.legend(fontsize=8); ax.grid(True, alpha=0.3)

    # 加速度指令 vs 実機 ax
    ax = axes[0, 1]
    ax.plot(tr, df["accel_des"].values, color="gray", lw=0.8, label="指令 accel_des")
    ax.plot(tr, df["real_ax"].values,   color="blue", lw=1.0, label="実機 ax")
    ax.axvline(0, color="green", lw=1.0, ls=":", alpha=0.8)
    ax.set_xlabel("発進からの時刻 [s]"); ax.set_ylabel("加速度 [m/s²]")
    ax.set_title("加速度: 指令 vs 実機"); ax.legend(fontsize=8); ax.grid(True, alpha=0.3)

    # 縦方向誤差
    ax = axes[1, 0]
    err_s = (df["err_ds_long"] * 100)
    ma    = err_s.rolling(window, center=True, min_periods=1).mean().values
    ax.plot(tr, err_s.values, color="gray", lw=0.4, alpha=0.4, label="raw")
    ax.plot(tr, ma,           color="red",  lw=1.5,             label=f"移動平均(w={window})")
    ax.axhline(0, color="black", lw=0.8)
    ax.axvline(0, color="green", lw=1.0, ls=":", alpha=0.8)
    ax.set_xlabel("発進からの時刻 [s]"); ax.set_ylabel("縦方向誤差 [cm]")
    ax.set_title("1ステップ縦方向誤差"); ax.legend(fontsize=8); ax.grid(True, alpha=0.3)

    # 横方向誤差
    ax = axes[1, 1]
    err_s = (df["err_ds_lat"] * 100)
    ma    = err_s.rolling(window, center=True, min_periods=1).mean().values
    ax.plot(tr, err_s.values, color="gray", lw=0.4, alpha=0.4, label="raw")
    ax.plot(tr, ma,           color="red",  lw=1.5,             label=f"移動平均(w={window})")
    ax.axhline(0, color="black", lw=0.8)
    ax.axvline(0, color="green", lw=1.0, ls=":", alpha=0.8)
    ax.set_xlabel("発進からの時刻 [s]"); ax.set_ylabel("横方向誤差 [cm]")
    ax.set_title("1ステップ横方向誤差"); ax.legend(fontsize=8); ax.grid(True, alpha=0.3)

    fig.suptitle(
        "カーブ② per-step delta 分析\n"
        "(各ステップで実機状態にリセット — 計画挙動の差を除外)",
        fontsize=11,
    )
    fig.tight_layout()
    add_params_annotation(fig, params)
    _save(fig, "overview")


def plot_error_timeseries(df: pd.DataFrame, params: dict) -> None:
    rad2deg = 180.0 / math.pi
    fig, axes = plt.subplots(3, 1, figsize=(12, 11), sharex=True)
    tr = df["tr"].values
    window = max(1, len(df) // 30)

    for ax, vals, ylabel, title, color in [
        (axes[0], df["err_ds_long"].values * 100,   "縦方向誤差 [cm]",   "1ステップ縦方向位置誤差 (実機 − モデル)", "red"),
        (axes[1], df["err_ds_lat"].values  * 100,   "横方向誤差 [cm]",   "1ステップ横方向位置誤差 (実機 − モデル)", "red"),
        (axes[2], df["err_steer"].values   * rad2deg, "ステア予測誤差 [deg]", "1ステップステア予測誤差 (actual[k+1] − pred[k+1])", "purple"),
    ]:
        ma = pd.Series(vals).rolling(window, center=True, min_periods=1).mean().values
        ax.plot(tr, vals, color="gray", lw=0.4, alpha=0.4, label="raw")
        ax.plot(tr, ma,   color=color,  lw=1.5,             label=f"移動平均(w={window})")
        ax.axhline(0, color="black", lw=0.8)
        ax.axvline(0, color="green", lw=1.0, ls=":", alpha=0.8, label="発進")
        ax.set_ylabel(ylabel); ax.set_title(title)
        ax.legend(fontsize=8); ax.grid(True, alpha=0.3)

    axes[2].set_xlabel("発進からの時刻 [s]")
    fig.suptitle("カーブ② per-step delta 誤差時系列", fontsize=11)
    fig.tight_layout()
    add_params_annotation(fig, params)
    _save(fig, "error_timeseries")


def plot_error_vs_speed(df: pd.DataFrame, params: dict) -> None:
    rad2deg = 180.0 / math.pi
    fig, axes = plt.subplots(1, 3, figsize=(17, 5))
    vx = df["real_vx"].values
    speed_bins = [0.0, 2.0, 5.0, 8.0, 50.0]
    colors = ["#4472C4", "#ED7D31", "#A9D18E", "#FF0000"]

    for ax, vals, ylabel in [
        (axes[0], df["err_ds_long"].values * 100,    "縦方向誤差 [cm]"),
        (axes[1], df["err_ds_lat"].values  * 100,    "横方向誤差 [cm]"),
        (axes[2], df["err_steer"].values   * rad2deg, "ステア予測誤差 [deg]"),
    ]:
        for i, (lo, hi) in enumerate(zip(speed_bins[:-1], speed_bins[1:])):
            mask = (vx >= lo) & (vx < hi)
            if mask.sum() == 0:
                continue
            lbl = f"v={lo:.0f}–{hi:.0f} m/s" if hi < 50 else f"v≥{lo:.0f} m/s"
            ax.scatter(vx[mask], vals[mask], s=4, alpha=0.5,
                       color=colors[i % len(colors)], label=lbl)
        ax.axhline(0, color="black", lw=0.8)
        ax.set_xlabel("速度 [m/s]"); ax.set_ylabel(ylabel)
        ax.set_title(f"{ylabel} vs 速度")
        ax.legend(fontsize=8); ax.grid(True, alpha=0.3)

    fig.suptitle("カーブ② per-step delta: 速度依存性", fontsize=11)
    fig.tight_layout()
    add_params_annotation(fig, params)
    _save(fig, "error_vs_speed")


def plot_steering_analysis(df: pd.DataFrame, params: dict) -> None:
    """ステア 1ステップ予測の詳細分析（4パネル）."""
    rad2deg = 180.0 / math.pi
    tr      = df["tr"].values
    window  = max(1, len(df) // 30)

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))

    # --- (0,0) ステア角の時系列: 実機 t_{k+1} vs モデル予測 t_{k+1} vs 指令 ---
    ax = axes[0, 0]
    ax.plot(tr, df["real_steer_kp1"].values * rad2deg, color="blue",   lw=1.2, label="実機 steer[k+1]")
    ax.plot(tr, df["sim_steer_kp1"].values  * rad2deg, color="red",    lw=1.0, ls="--", label="予測 steer[k+1]")
    ax.plot(tr, df["steer_des"].values       * rad2deg, color="gray",   lw=0.7, ls=":",  label="指令 steer_des[k]")
    ax.axvline(0, color="green", lw=1.0, ls=":", alpha=0.8, label="発進")
    ax.set_xlabel("発進からの時刻 [s]"); ax.set_ylabel("ステア角 [deg]")
    ax.set_title("ステア角: 実機[k+1] vs モデル予測[k+1] vs 指令[k]")
    ax.legend(fontsize=8); ax.grid(True, alpha=0.3)

    # --- (0,1) ステア追従誤差（指令 vs 実機）: 実機のステア制御性能 ---
    ax = axes[0, 1]
    steer_follow_err = (df["real_steer_kp1"].values - df["steer_des"].values) * rad2deg
    ma_f = pd.Series(steer_follow_err).rolling(window, center=True, min_periods=1).mean().values
    ax.plot(tr, steer_follow_err, color="gray", lw=0.4, alpha=0.4, label="raw")
    ax.plot(tr, ma_f,             color="blue", lw=1.5, label=f"移動平均(w={window})")
    ax.axhline(0, color="black", lw=0.8)
    ax.axvline(0, color="green", lw=1.0, ls=":", alpha=0.8)
    ax.set_xlabel("発進からの時刻 [s]"); ax.set_ylabel("追従誤差 [deg]")
    ax.set_title("実機ステア追従誤差 (actual[k+1] − cmd[k])")
    ax.legend(fontsize=8); ax.grid(True, alpha=0.3)

    # --- (1,0) ステア予測誤差の時系列 ---
    ax = axes[1, 0]
    err_deg = df["err_steer"].values * rad2deg
    ma_e    = pd.Series(err_deg).rolling(window, center=True, min_periods=1).mean().values
    ax.plot(tr, err_deg, color="gray", lw=0.4, alpha=0.4, label="raw")
    ax.plot(tr, ma_e,    color="red",  lw=1.5, label=f"移動平均(w={window})")
    ax.axhline(0, color="black", lw=0.8)
    ax.axvline(0, color="green", lw=1.0, ls=":", alpha=0.8, label="発進")
    rmse_deg = float(np.sqrt(np.mean(err_deg ** 2)))
    ax.set_xlabel("発進からの時刻 [s]"); ax.set_ylabel("予測誤差 [deg]")
    ax.set_title(f"1ステップ ステア予測誤差 (actual[k+1] − pred[k+1])  RMSE={rmse_deg:.4f}°")
    ax.legend(fontsize=8); ax.grid(True, alpha=0.3)

    # --- (1,1) ステア予測誤差 vs 指令ステア角（大入力時の精度確認）---
    ax = axes[1, 1]
    cmd_deg = df["steer_des"].values * rad2deg
    speed_bins = [0.0, 2.0, 5.0, 8.0, 50.0]
    colors     = ["#4472C4", "#ED7D31", "#A9D18E", "#FF0000"]
    vx = df["real_vx"].values
    for i, (lo, hi) in enumerate(zip(speed_bins[:-1], speed_bins[1:])):
        mask = (vx >= lo) & (vx < hi)
        if mask.sum() == 0:
            continue
        lbl = f"v={lo:.0f}–{hi:.0f} m/s" if hi < 50 else f"v≥{lo:.0f} m/s"
        ax.scatter(cmd_deg[mask], err_deg[mask], s=4, alpha=0.5,
                   color=colors[i % len(colors)], label=lbl)
    ax.axhline(0, color="black", lw=0.8)
    ax.set_xlabel("指令ステア角 [deg]"); ax.set_ylabel("予測誤差 [deg]")
    ax.set_title("ステア予測誤差 vs 指令ステア角（色=速度域）")
    ax.legend(fontsize=8); ax.grid(True, alpha=0.3)

    fig.suptitle(
        "カーブ② per-step ステアリング分析\n"
        "(1ステップ予測誤差: actual[k+1] − model_pred[k+1])",
        fontsize=11,
    )
    fig.tight_layout()
    add_params_annotation(fig, params)
    _save(fig, "steering_analysis")


def _load_map_ways(map_dir: Path) -> list[np.ndarray] | None:
    from lxml import etree
    candidates = sorted(map_dir.glob("*/lanelet2_map.osm"))
    if not candidates:
        return None
    tree = etree.parse(str(candidates[0]))
    nodes = {}
    for nd in tree.findall("node"):
        nodes[nd.get("id")] = (float(nd.get("lon")), float(nd.get("lat")))
    # local → just collect node positions in map frame
    # use x/y from "local_x" / "local_y" tags if present
    node_xy = {}
    for nd in tree.findall("node"):
        lx = nd.find("tag[@k='local_x']")
        ly = nd.find("tag[@k='local_y']")
        if lx is not None and ly is not None:
            node_xy[nd.get("id")] = (float(lx.get("v")), float(ly.get("v")))
    ways = []
    for way in tree.findall("way"):
        pts = []
        for nd in way.findall("nd"):
            ref = nd.get("ref")
            if ref in node_xy:
                pts.append(node_xy[ref])
        if len(pts) >= 2:
            ways.append(np.array(pts))
    return ways if ways else None


def plot_map_distribution(df: pd.DataFrame, params: dict) -> None:
    rad2deg = 180.0 / math.pi
    map_ways = _load_map_ways(MAP_DIR)

    fig, axes = plt.subplots(1, 3, figsize=(20, 7))
    cx, cy = 89301, 43085

    for ax, vals, label, unit in [
        (axes[0], df["err_ds_long"].values * 100,    "縦方向誤差",    "cm"),
        (axes[1], df["err_ds_lat"].values  * 100,    "横方向誤差",    "cm"),
        (axes[2], df["err_steer"].values   * rad2deg, "ステア予測誤差", "deg"),
    ]:
        if map_ways:
            for pts in map_ways:
                wx, wy = pts[:, 0], pts[:, 1]
                if wx.max() < cx - 80 or wx.min() > cx + 80: continue
                if wy.max() < cy - 80 or wy.min() > cy + 80: continue
                ax.plot(wx, wy, color="#cccccc", lw=0.5, zorder=1)

        vmax = max(abs(vals).max(), 1.0)
        sc = ax.scatter(df["pos_x"], df["pos_y"], c=vals, cmap="RdBu_r",
                        vmin=-vmax, vmax=vmax, s=8, zorder=3)
        plt.colorbar(sc, ax=ax, label=unit)
        ax.set_xlim(cx - 80, cx + 80)
        ax.set_ylim(cy - 80, cy + 80)
        ax.set_aspect("equal")
        ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
        ax.set_title(f"地図上の誤差分布: {label} [{unit}]")
        ax.grid(True, lw=0.5, alpha=0.5)

    fig.suptitle("カーブ② per-step delta: 地図上の誤差分布", fontsize=11)
    fig.tight_layout()
    add_params_annotation(fig, params)
    _save(fig, "map_distribution")


def save_summary(df: pd.DataFrame) -> None:
    rad2deg = 180.0 / math.pi
    tr = df["tr"].values

    def rmse_cm(col, mask=None):
        v = df[col].values if mask is None else df[col].values[mask]
        return float(np.sqrt(np.mean(v ** 2))) * 100

    def rmse_deg(col, mask=None):
        v = df[col].values if mask is None else df[col].values[mask]
        return float(np.sqrt(np.mean(v ** 2))) * rad2deg

    def mean_deg(col, mask=None):
        v = df[col].values if mask is None else df[col].values[mask]
        return float(np.mean(v)) * rad2deg

    lines = [
        "=== カーブ② per-step delta 分析 サマリ ===",
        f"有効ステップ数: {len(df)}",
        f"解析窓: tr={tr[0]:.1f}〜{tr[-1]:.1f}s",
        "",
        "--- 全区間: 位置 ---",
        f"縦方向 RMSE: {rmse_cm('err_ds_long'):.3f} cm",
        f"横方向 RMSE: {rmse_cm('err_ds_lat'):.3f} cm",
        "",
        "--- 全区間: ステア予測 (actual[k+1] − pred[k+1]) ---",
        f"ステア予測 RMSE: {rmse_deg('err_steer'):.4f} deg",
        f"ステア予測 平均誤差: {mean_deg('err_steer'):.4f} deg  (正=実機が指令より大/負=小)",
        "",
        "--- 発進後時間帯別 (縦方向 RMSE) ---",
    ]
    for lbl, lo, hi in [
        ("t=0〜5s  (発進直後)",  0.0,  5.0),
        ("t=5〜15s (カーブ進入)", 5.0, 15.0),
        ("t=15〜35s (カーブ奥)", 15.0, 35.0),
    ]:
        mask = (tr >= lo) & (tr < hi)
        if mask.sum() > 0:
            lines.append(f"  {lbl}: {rmse_cm('err_ds_long', mask):.3f} cm  (n={mask.sum()})")

    lines += ["", "--- 発進後時間帯別 (ステア予測 RMSE) ---"]
    for lbl, lo, hi in [
        ("t=0〜5s  (発進直後)",  0.0,  5.0),
        ("t=5〜15s (カーブ進入)", 5.0, 15.0),
        ("t=15〜35s (カーブ奥)", 15.0, 35.0),
    ]:
        mask = (tr >= lo) & (tr < hi)
        if mask.sum() > 0:
            lines.append(f"  {lbl}: {rmse_deg('err_steer', mask):.4f} deg  (n={mask.sum()})")

    lines += ["", "--- 速度域別 ---"]
    for lo, hi in [(0, 2), (2, 5), (5, 8), (8, 100)]:
        lbl = f"v={lo}〜{hi} m/s" if hi < 100 else f"v≥{lo} m/s"
        mask = (df["real_vx"].values >= lo) & (df["real_vx"].values < hi)
        if mask.sum() > 0:
            lines.append(
                f"  {lbl:22s}: 縦={rmse_cm('err_ds_long', mask):.3f} cm, "
                f"横={rmse_cm('err_ds_lat', mask):.3f} cm, "
                f"ステア={rmse_deg('err_steer', mask):.4f} deg  (n={mask.sum()})"
            )

    text = "\n".join(lines)
    print(text)
    (OUT_DIR / "summary.txt").write_text(text + "\n", encoding="utf-8")
    print(f"  Saved: {OUT_DIR / 'summary.txt'}")


# ---------------------------------------------------------------------------
# メイン
# ---------------------------------------------------------------------------

def main() -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)

    print(f"Loading: {REAL_MCAP}")
    data = load_real_bag(REAL_MCAP)

    # AUTONOMOUS 開始時刻
    t0_ns = find_autonomous_start(data)
    print(f"AUTONOMOUS 開始: t0_ns={t0_ns}")

    # velocity を秒に変換して発進時刻検出
    df_vel = data["vel"].copy()
    df_vel["t"] = (df_vel["t_ns"] - t0_ns) / 1e9
    df_vel = df_vel[df_vel["t"] >= 0].sort_values("t").reset_index(drop=True)

    t_launch = find_curve2_launch(df_vel)
    if t_launch is None:
        raise RuntimeError("カーブ②発進時刻を検出できませんでした")
    print(f"カーブ②発進: t={t_launch:.1f}s")

    print("\n=== per-step delta 分析開始 ===")
    df = run_per_step(data, t0_ns, t_launch, PARAMS)
    print(f"有効ステップ: {len(df)}")

    if df.empty:
        raise RuntimeError("有効なステップが 0 件でした")

    print("\n=== 出力生成 ===")
    df.to_csv(OUT_DIR / "per_step_delta.csv", index=False)
    print(f"  Saved: {OUT_DIR / 'per_step_delta.csv'}")

    save_summary(df)
    plot_overview(df, PARAMS)
    plot_error_timeseries(df, PARAMS)
    plot_error_vs_speed(df, PARAMS)
    plot_steering_analysis(df, PARAMS)
    plot_map_distribution(df, PARAMS)

    print(f"\n完了。出力先: {OUT_DIR}")


if __name__ == "__main__":
    main()
