"""可复现的 VMC 响应曲线绘制工具。"""

from __future__ import annotations

from pathlib import Path

import numpy as np
from numpy.typing import NDArray


def save_attitude_response(
    time_s: NDArray[np.float64],
    roll_rad: NDArray[np.float64],
    pitch_rad: NDArray[np.float64],
    yaw_rad: NDArray[np.float64],
    disturbance_active: NDArray[np.bool_],
    *,
    output_prefix: Path,
    show: bool,
) -> tuple[Path, Path, Path]:
    """保存 yaw、pitch、roll 时域曲线为 CSV、PDF 和 300-DPI PNG。"""

    arrays = tuple(
        np.asarray(values)
        for values in (time_s, roll_rad, pitch_rad, yaw_rad, disturbance_active)
    )
    if not arrays[0].size or any(values.shape != arrays[0].shape for values in arrays):
        raise ValueError("姿态记录必须是形状相同的非空一维数组")

    output_prefix = Path(output_prefix)
    output_prefix.parent.mkdir(parents=True, exist_ok=True)
    time_s, roll_rad, pitch_rad, yaw_rad, disturbance_active = arrays
    angles_deg = np.rad2deg(
        np.column_stack(
            (np.unwrap(roll_rad), np.unwrap(pitch_rad), np.unwrap(yaw_rad))
        )
    )

    csv_path = output_prefix.with_suffix(".csv")
    pdf_path = output_prefix.with_suffix(".pdf")
    png_path = output_prefix.with_suffix(".png")
    np.savetxt(
        csv_path,
        np.column_stack((time_s, angles_deg, disturbance_active.astype(int))),
        delimiter=",",
        header="time_s,roll_deg,pitch_deg,yaw_deg,disturbance_active",
        comments="",
    )

    import matplotlib

    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    with plt.rc_context(
        {
            "font.family": "serif",
            "font.serif": ["Times New Roman", "DejaVu Serif"],
            "font.size": 10,
            "axes.titlesize": 11,
            "axes.titleweight": "bold",
            "axes.labelsize": 10,
            "legend.fontsize": 9,
            "legend.frameon": False,
            "figure.dpi": 150,
            "savefig.dpi": 300,
            "savefig.bbox": "tight",
            "axes.spines.top": False,
            "axes.spines.right": False,
            "axes.grid": True,
            "grid.alpha": 0.18,
            "grid.linestyle": "-",
            "lines.linewidth": 1.8,
        }
    ):
        figure, axis = plt.subplots(figsize=(6.75, 3.2))
        axis.plot(time_s, angles_deg[:, 2], color="#0072B2", label="Yaw")
        axis.plot(
            time_s,
            angles_deg[:, 1],
            color="#D55E00",
            linestyle="--",
            label="Pitch",
        )
        axis.plot(
            time_s,
            angles_deg[:, 0],
            color="#009E73",
            linestyle="-.",
            label="Roll",
        )
        _shade_disturbances(axis, time_s, disturbance_active)
        axis.axhline(0.0, color="#555555", linewidth=0.8, alpha=0.7)
        axis.set_title("VMC Body Attitude Response")
        axis.set_xlabel("Time (s)")
        axis.set_ylabel("Attitude (deg)")
        axis.legend(loc="upper left", ncol=3)
        axis.margins(x=0.0)
        figure.savefig(pdf_path)
        figure.savefig(png_path, dpi=300)
        if show:
            plt.show()
        else:
            plt.close(figure)
    return csv_path, pdf_path, png_path


def _shade_disturbances(axis, time_s: NDArray, active: NDArray) -> None:
    transitions = np.diff(np.pad(active.astype(int), (1, 1)))
    starts = np.flatnonzero(transitions == 1)
    stops = np.flatnonzero(transitions == -1)
    for index, (start, stop) in enumerate(zip(starts, stops, strict=True)):
        right = time_s[min(stop, len(time_s) - 1)]
        axis.axvspan(
            time_s[start],
            right,
            color="#E69F00",
            alpha=0.14,
            linewidth=0.0,
            label="Disturbance" if index == 0 else None,
        )
