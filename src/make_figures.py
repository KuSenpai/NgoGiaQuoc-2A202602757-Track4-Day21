"""Vẽ biểu đồ + ảnh demo/failure từ kết quả của src.drift_sweep.

    python -m src.drift_sweep --data-root data/kitti_mini --tag kitti
    python -m src.drift_sweep --data-root data/nuscenes_mini_subset --tag nusc
    python -m src.make_figures
"""
from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import pandas as pd

from src.drift_sweep import make_calib, object_points
from starter.datasets import load_frame
from starter.projection import draw_box2d, overlay_points, project_velo_to_image

KITTI, NUSC = "data/kitti_mini", "data/nuscenes_mini_subset"


def plot_curves(res: Path, fig_dir: Path) -> None:
    fig, ax = plt.subplots(1, 3, figsize=(16, 4.5))
    for tag, ls in [("kitti", "-"), ("nusc", "--")]:
        s = pd.read_csv(res / f"calib_drift_sweep_{tag}.csv")
        for kind in ["yaw", "pitch", "roll"]:
            d = s[(s.group == "all") & (s.dist == "all") & (s.kind == kind)]
            ax[0].plot(d.level, d.pct_box, ls, marker="o", label=f"{kind} ({tag})")
        if tag == "kitti":
            for (g, dist), c in [(("car", "15-30m"), "C0"), (("vru", "15-30m"), "C3"),
                                 (("car", "30-50m"), "C2"), (("vru", "0-15m"), "C1")]:
                d = s[(s.group == g) & (s.dist == dist) & (s.kind == "yaw")]
                ax[1].plot(d.level, d.pct_box, "-o", color=c, label=f"{g} {dist} (n={d.n_obj.iloc[0]})")
        d = s[(s.group == "all") & (s.dist == "all") & (s.kind == "ty")]
        ax[2].plot(d.level * 100, d.pct_box, ls, marker="o", label=f"ty ({tag})")
    ax[0].set(title="Mọi object: % điểm còn trong 2D box", xlabel="độ lệch (độ)", ylabel="% điểm trong 2D box")
    ax[1].set(title="KITTI, yaw: theo loại object và khoảng cách", xlabel="yaw (độ)")
    ax[2].set(title="Dịch ngang ty (cm)", xlabel="ty (cm)")
    for a in ax:
        a.set_ylim(0, 105)
        a.grid(alpha=.3)
        a.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(fig_dir / "calib_drift_curves.png", dpi=130)
    plt.close(fig)


def crop_panel(fr: dict, obj, idx, calibs: list, titles: list[str], pad: int = 40, scale: int = 4):
    x1, y1, x2, y2 = (int(v) for v in obj.bbox)
    H, W = fr["image"].shape[:2]
    cx1, cy1, cx2, cy2 = max(x1 - pad, 0), max(y1 - pad, 0), min(x2 + pad, W), min(y2 + pad, H)
    tiles = []
    for calib, title in zip(calibs, titles):
        uv, depth, _ = project_velo_to_image(fr["points"][idx], calib, fr["image"].shape)
        vis = overlay_points(fr["image"], uv, depth, radius=1)
        vis = draw_box2d(vis, obj.bbox)[cy1:cy2, cx1:cx2]
        vis = cv2.resize(vis, None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)
        cv2.putText(vis, title, (6, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 3)
        cv2.putText(vis, title, (6, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 1)
        tiles.append(vis)
    return cv2.hconcat(tiles)


def pick(res: Path, group: str, lo: float, hi: float) -> pd.Series:
    """Object nhiều điểm nhất trong [lo, hi) m, ban đầu khớp tốt (>= 85% trong box)."""
    d = pd.read_csv(res / "calib_drift_per_object_kitti.csv")
    d = d[(d.group == group) & (d.depth_m >= lo) & (d.depth_m < hi) & (d.kind == "yaw") & (d.level == 0) & (d.pct_box >= 85)]
    return d.sort_values("n_pts", ascending=False).iloc[0]


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--results", default="results")
    a = ap.parse_args()
    res = Path(a.results)
    fig_dir = res / "figures"
    fig_dir.mkdir(parents=True, exist_ok=True)
    plot_curves(res, fig_dir)

    def get(row):
        fr = load_frame(KITTI, f"{int(row.frame):06d}")
        for k, (obj, _, idx) in enumerate(object_points(fr, 0.3, 1, 5)):
            if k == int(row.obj):
                return fr, obj, idx

    def panel(row, kind, levels, name):
        fr, obj, idx = get(row)
        calibs = [make_calib(fr["calib"], kind, lv) for lv in levels]
        unit = {"ty": "m"}.get(kind, "deg")
        titles = [f"{kind} {lv}{unit}" for lv in levels]
        cv2.imwrite(str(fig_dir / name), crop_panel(fr, obj, idx, calibs, titles))
        print(f"{name}: frame {row.frame} {obj.type} z={obj.location[2]:.1f}m n_pts={len(idx)}")

    # Basic: 3 overlay ở 3 khoảng cách (calib gốc)
    for tag, lo, hi in [("near", 0, 15), ("mid", 15, 30), ("far", 40, 200)]:
        panel(pick(res, "car", lo, hi), "yaw", [0], f"demo_overlay_{tag}.png")
    # Failure 01: người đi bộ ở 15-30 m, yaw 0 / 1 / 2 độ -> điểm bị kéo ra khỏi box
    row = pick(res, "vru", 15, 30)
    panel(row, "yaw", [0, 1, 2], "fail_01_yaw_pedestrian.png")
    # Failure 02: dịch 10 cm ngang -> không thấy khác biệt, metric không phát hiện được
    panel(row, "ty", [0, 0.05, 0.10], "fail_02_ty10cm_undetected.png")


if __name__ == "__main__":
    main()
