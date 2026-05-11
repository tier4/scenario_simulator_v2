"""共通ユーティリティ: シミュレータパラメータの読み込みと図への注釈追加."""

from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt

BASE = Path(__file__).parent
_DESC_DIR = BASE.parent / "best_model_description" / "config"
_SIM_YAML  = _DESC_DIR / "simulator_model.param.yaml"
_INFO_YAML = _DESC_DIR / "vehicle_info.param.yaml"


def load_sim_params() -> dict:
    """simulator_model.param.yaml + vehicle_info.param.yaml から主要パラメータを返す。

    YAML が読めない場合はフォールバック値を使用。
    """
    params: dict = {}
    for yaml_path in (_SIM_YAML, _INFO_YAML):
        try:
            import yaml  # PyYAML or ruamel.yaml
            with open(yaml_path, encoding="utf-8") as f:
                doc = yaml.safe_load(f)
            # ROS 2 YAML: /**:/ros__parameters/ 以下がフラット
            ros_p = (
                doc.get("/**", {}).get("ros__parameters", {})
                or doc.get("/*", {}).get("ros__parameters", {})
                or {}
            )
            params.update(ros_p)
        except Exception:
            pass

    # フォールバック（YAML 読み取り失敗時）
    defaults = {
        "vehicle_model_type":  "DELAY_STEER_ACC_GEARED_WO_FALL_GUARD",
        "acc_time_delay":      0.101,
        "acc_time_constant":   0.2589,
        "brake_delay":         0.0685,
        "brake_time_constant": 0.15,
        "steer_time_delay":    0.0315,
        "steer_time_constant": 0.4983,
        "steer_dead_band":     0.0,
        "steer_bias":          0.0005,
        "vel_lim":             50.0,
        "vel_rate_lim":        7.0,
        "steer_lim":           1.0,
        "steer_rate_lim":      5.0,
        "wheel_base":          4.76012,
        "max_steer_angle":     0.640,
    }
    for k, v in defaults.items():
        params.setdefault(k, v)

    return params


def make_annotation_text(params: dict) -> str:
    """パラメータ dict からアノテーション文字列を生成する。"""
    wb  = params.get("wheel_base",          params.get("wheelbase", 4.76012))
    msa = params.get("max_steer_angle",     0.640)
    vmt = params.get("vehicle_model_type",  "DELAY_STEER_ACC_GEARED_WO_FALL_GUARD")

    lines = [
        f"Model: {vmt}",
        f"acc_delay={params.get('acc_time_delay', '?'):.4f}s"
        f"  acc_tc={params.get('acc_time_constant', '?'):.4f}s",
    ]
    if "brake_delay" in params:
        lines.append(
            f"brake_delay={params['brake_delay']:.4f}s"
            f"  brake_tc={params.get('brake_time_constant', '?'):.4f}s"
        )
    lines += [
        f"steer_delay={params.get('steer_time_delay', '?'):.4f}s"
        f"  steer_tc={params.get('steer_time_constant', '?'):.4f}s",
        f"steer_bias={params.get('steer_bias', '?'):.4f}rad"
        f"  steer_db={params.get('steer_dead_band', '?'):.4f}rad",
        f"vel_lim={params.get('vel_lim', '?'):.1f}m/s"
        f"  vx_rate_lim={params.get('vel_rate_lim', '?'):.1f}m/s²",
        f"steer_lim={params.get('steer_lim', '?'):.2f}rad"
        f"  steer_rate_lim={params.get('steer_rate_lim', '?'):.1f}rad/s",
        f"wheelbase={wb:.5f}m  max_steer={msa:.3f}rad",
    ]
    if "sub_dt" in params:
        lines.append(f"sub_dt={params['sub_dt']:.5f}s")
    return "\n".join(lines)


def add_params_annotation(fig: plt.Figure, params: dict | None = None) -> None:
    """図の右下にモデルパラメータのテキストボックスを追加する。

    params が None の場合は YAML から自動読み込みする。
    """
    if params is None:
        params = load_sim_params()
    text = make_annotation_text(params)
    fig.text(
        0.99, 0.01,
        text,
        ha="right", va="bottom",
        fontsize=6.5,
        fontfamily="monospace",
        color="#555555",
        bbox=dict(
            boxstyle="round,pad=0.3",
            facecolor="white",
            alpha=0.7,
            edgecolor="#aaaaaa",
        ),
        transform=fig.transFigure,
    )
