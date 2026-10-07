"""Topic A: đo độ nhạy của projection LiDAR->camera với calibration drift.

Với mỗi object (label 3D) không bị cắt/che nhiều:
  1. Lấy các điểm LiDAR nằm trong 3D box GT (calib gốc) -> "điểm của object".
  2. Làm lệch extrinsic (yaw / pitch / roll / translation y) bằng starter.projection.perturb_extrinsic.
  3. Chiếu lại đúng những điểm đó, đếm % còn trong ảnh và % còn trong 2D box của label.
Hoàn toàn tất định (không có phép ngẫu nhiên), nên chạy lại ra đúng cùng số.

    python -m src.drift_sweep --data-root data/kitti_mini --tag kitti
    python -m src.drift_sweep --data-root data/nuscenes_mini_subset --tag nusc
"""
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import pandas as pd

from starter.datasets import list_frames, load_frame
from starter.projection import perturb_extrinsic, project_velo_to_image, velo_to_cam

CAR_LIKE = {"Car", "Van", "Truck", "Bus"}
VRU = {"Pedestrian", "Cyclist", "Bicycle", "Motorcycle", "Person_sitting"}
DIST_BINS = [(0, 15), (15, 30), (30, 50), (50, 200)]
# (loại perturb, các mức). yaw/pitch/roll tính bằng độ, ty bằng mét (dịch dọc trục y của LiDAR).
SWEEPS = {"yaw": [0, 0.5, 1, 2, 3], "pitch": [0, 0.5, 1, 2, 3], "roll": [0, 0.5, 1, 2, 3],
          "ty": [0, 0.02, 0.05, 0.10]}


def make_calib(calib, kind: str, level: float):
    if kind == "ty":
        return perturb_extrinsic(calib, t_xyz_m=(0, level, 0))
    return perturb_extrinsic(calib, **{f"{kind}_deg": level})


def points_in_box3d(pts_cam: np.ndarray, obj, margin: float = 0.1) -> np.ndarray:
    """Mask điểm (camera frame) nằm trong box KITTI (location = tâm đáy, y xuống)."""
    h, w, l = obj.dimensions
    c, s = np.cos(obj.rotation_y), np.sin(obj.rotation_y)
    d = pts_cam - obj.location
    x = c * d[:, 0] - s * d[:, 2]          # R_y^T @ d
    z = s * d[:, 0] + c * d[:, 2]
    y = d[:, 1]
    return (np.abs(x) <= l / 2 + margin) & (np.abs(z) <= w / 2 + margin) & (y <= margin) & (y >= -h - margin)


def object_points(fr: dict, max_trunc: float, max_occ: int, min_pts: int):
    """Yield (obj, chỉ số điểm LiDAR gốc nằm trong box) cho các object đủ điều kiện."""
    pts = fr["points"]
    finite = np.where(np.isfinite(pts[:, :3]).all(1))[0]
    pc = velo_to_cam(pts[finite, :3], fr["calib"])
    for obj in fr["labels"]:
        group = "car" if obj.type in CAR_LIKE else "vru" if obj.type in VRU else None
        if group is None or obj.truncated > max_trunc or obj.occluded > max_occ:
            continue
        idx = finite[points_in_box3d(pc, obj)]
        if len(idx) >= min_pts:
            yield obj, group, idx


def measure(fr: dict, obj, idx: np.ndarray, calib) -> tuple[float, float]:
    """(% điểm còn trong ảnh, % điểm rơi vào 2D box của label); mẫu số là mọi điểm của object."""
    uv, _, mask = project_velo_to_image(fr["points"][idx], calib, fr["image"].shape)
    x1, y1, x2, y2 = obj.bbox
    inside = (uv[:, 0] >= x1) & (uv[:, 0] <= x2) & (uv[:, 1] >= y1) & (uv[:, 1] <= y2)
    return 100 * mask.mean(), 100 * inside.sum() / len(idx)


def run(data_root: str, frames: list[str], max_trunc: float, max_occ: int, min_pts: int) -> pd.DataFrame:
    rows = []
    for fid in frames:
        fr = load_frame(data_root, fid)
        for k, (obj, group, idx) in enumerate(object_points(fr, max_trunc, max_occ, min_pts)):
            for kind, levels in SWEEPS.items():
                for lv in levels:
                    fov, box = measure(fr, obj, idx, make_calib(fr["calib"], kind, lv))
                    rows.append(dict(frame=fid, obj=k, type=obj.type, group=group, depth_m=obj.location[2],
                                     n_pts=len(idx), kind=kind, level=lv, pct_fov=fov, pct_box=box))
    return pd.DataFrame(rows)


def summarize(df: pd.DataFrame) -> pd.DataFrame:
    """Trung bình theo object (mỗi object 1 phiếu), theo kind x level x group x khoảng cách."""
    df = df.copy()
    df["dist"] = pd.cut(df.depth_m, [b[0] for b in DIST_BINS] + [DIST_BINS[-1][1]], right=False,
                        labels=[f"{a}-{b}m" for a, b in DIST_BINS])
    parts = []
    for gname, g in [("all", df)] + list(df.groupby("group")):
        for dname, d in [("all", g)] + [(str(k), v) for k, v in g.groupby("dist", observed=True)]:
            s = d.groupby(["kind", "level"]).agg(n_obj=("obj", "size"), n_pts=("n_pts", "sum"),
                                                 pct_fov=("pct_fov", "mean"), pct_box=("pct_box", "mean")).reset_index()
            s.insert(0, "dist", dname)
            s.insert(0, "group", gname)
            parts.append(s)
    return pd.concat(parts, ignore_index=True)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--data-root", default="data/kitti_mini")
    ap.add_argument("--tag", default="kitti", help="hậu tố tên file kết quả")
    ap.add_argument("--max-trunc", type=float, default=0.3, help="bỏ object bị cắt ở rìa ảnh quá mức này")
    ap.add_argument("--max-occ", type=int, default=1, help="bỏ object bị che hơn mức này (0 thấy rõ)")
    ap.add_argument("--min-pts", type=int, default=5, help="bỏ object có ít điểm LiDAR hơn mức này")
    ap.add_argument("--out-dir", default="results")
    a = ap.parse_args()

    df = run(a.data_root, list_frames(a.data_root), a.max_trunc, a.max_occ, a.min_pts)
    out = Path(a.out_dir)
    out.mkdir(parents=True, exist_ok=True)
    df.to_csv(out / f"calib_drift_per_object_{a.tag}.csv", index=False, float_format="%.3f")
    summ = summarize(df)
    summ.to_csv(out / f"calib_drift_sweep_{a.tag}.csv", index=False, float_format="%.2f")
    print(f"{df.groupby(['frame', 'obj']).ngroups} objects")
    print(summ[(summ.group == "all") & (summ.dist == "all")].to_string(index=False))


if __name__ == "__main__":
    main()
