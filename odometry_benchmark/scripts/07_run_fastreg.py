"""FastReg (Arnold et al., RA-L 2021) as LiDAR odometry over a common scan cache.

Runs in the fastreg:cu128 image (GPU):
  docker run --rm --gpus all -v D:/Odometry/odometry_benchmark:/work -v <cache>:/data:ro fastreg:cu128 \
      python scripts/07_run_fastreg.py --cache /data --out outputs_public/<ds>/fastreg

Each consecutive scan pair (k, k+1) is registered exactly as in FastReg's KITTI evaluation
(eval.py + KITTIOdometryDataset): [x, y, z, intensity] padded with zeros to 129,300 points,
VoxelSampling(0.3 m), the shipped KITTI checkpoint, T = 1e-2, then FastReg's own RANSAC on
the predicted correspondences. The estimate maps scan k into scan k+1 (p1 = R p0 + t), so
poses chain as T_w,k+1 = T_w,k @ inv([R|t]). Intensity (8-bit on NCLT and Boreas) is scaled to
[0, 1] like KITTI's. Open3D's RANSAC is seeded so runs are repeatable.
"""
import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np
import pandas as pd
import torch

sys.path.insert(0, "/opt/fastreg")
import config  # noqa: E402  (FastReg's config: T, VOXEL_SAMPLING_SIZE)
import open3d as o3d  # noqa: E402
from lib.dataset.datatransforms import VoxelSampling  # noqa: E402
from lib.models.fastreg import FastReg  # noqa: E402

KITTI_PAD = 129300  # pts array size used by FastReg's KITTI loader


def load(cache, idx, scale):
    xyz = np.load(cache / "scans" / f"{idx:06d}.npy")[:, :3]
    inten = np.load(cache / "scans_intensity" / f"{idx:06d}.npy").astype(np.float32) / scale
    return np.c_[xyz, inten].astype(np.float32)


def rot_to_quat(R):
    from scipy.spatial.transform import Rotation
    return Rotation.from_matrix(R).as_quat()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cache", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--checkpoint", default="/opt/fastreg/assets/fastreg-kitti-checkpoint.pth")
    ap.add_argument("--intensity-scale", type=float, default=255.0)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--max-scans", type=int)
    args = ap.parse_args()
    cache, out = Path(args.cache), Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    torch.manual_seed(args.seed)
    o3d.utility.random.seed(args.seed)
    model = FastReg(T=config.T)
    model.load_state_dict(torch.load(args.checkpoint, map_location="cpu"))
    model.eval().cuda()
    voxel = VoxelSampling(config.VOXEL_SAMPLING_SIZE)

    stamps = pd.read_csv(cache / "scan_stamps.csv")
    if args.max_scans:
        stamps = stamps.iloc[: args.max_scans]
    T_w = np.eye(4)
    rows = [(stamps.sensor_stamp[0], 0, 0, 0, 0, 0, 0, 1)]
    prev = load(cache, int(stamps.idx[0]), args.intensity_scale)
    dts = []
    for k in range(1, len(stamps)):
        cur = load(cache, int(stamps.idx[k]), args.intensity_scale)
        n = max(KITTI_PAD, len(prev), len(cur))
        pb = np.zeros((2, n, 4), np.float32)
        pb[0, :len(prev)], pb[1, :len(cur)] = prev, cur
        pb, _ = voxel(pb, np.eye(4, dtype=np.float32))
        t0 = time.perf_counter()
        with torch.no_grad():
            R, t = model(torch.from_numpy(pb)[None])[:2]
        torch.cuda.synchronize()
        dts.append(time.perf_counter() - t0)
        T_10 = np.eye(4)
        T_10[:3, :3], T_10[:3, 3] = R[0].cpu().numpy(), t[0, 0].cpu().numpy()
        T_w = T_w @ np.linalg.inv(T_10)
        rows.append((stamps.sensor_stamp[k], *T_w[:3, 3], *rot_to_quat(T_w[:3, :3])))
        prev = cur
        if k % 250 == 0:
            print(f"[fastreg] scan {k}/{len(stamps)}  pos {T_w[:3, 3].round(1)}  "
                  f"{1000 * np.mean(dts[-250:]):.0f} ms/pair", flush=True)

    np.savetxt(out / "fastreg_tum.txt", np.array(rows), fmt="%.9f")
    dts = np.array(dts)
    timing = {"method": "fastreg", "n_scans": len(rows), "ms_per_scan_mean": round(1000 * dts.mean(), 2),
              "ms_per_scan_median": round(1000 * np.median(dts), 2), "fps": round(1 / dts.mean(), 1),
              "gpu": torch.cuda.get_device_name(0)}
    (out / "timing.json").write_text(json.dumps(timing, indent=2))
    print(timing)


if __name__ == "__main__":
    main()
