"""BUFFER-X (Seo et al., ICCV 2025) as LiDAR odometry over consecutive SnowPole scans.

Runs in the bufferx:cu128 image (cwd /opt/bufferx):
  docker run --rm --gpus all -v D:/Odometry/odometry_benchmark:/work bufferx:cu128 \
      python /work/scripts/10_run_bufferx.py --bins /work/cache/kitti_bins --out /work/outputs/bufferx

Each pair (k, k+1) goes through exactly the repository's KITTI test path: the test-split
branch of dataset/kitti.py (sphericity-based adaptive voxel size, two-level voxel downsampling,
max_numPts cap), collate_fn_descriptor, the KITTI config and the KITTI Desc+Pose checkpoints
(snapshot/kitti), as in test.py. Only the data source differs (our scans instead of KITTI
pairs; no ground truth, so relt_pose = I, which the model only uses for training losses).
The estimate maps scan k into scan k+1, so T_w,k+1 = T_w,k @ inv(T). Seeds fixed.
"""
import argparse
import json
import random
import sys
import time
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn

sys.path.insert(0, "/opt/bufferx")
from config import make_cfg  # noqa: E402
from dataset.dataloader import collate_fn_descriptor  # noqa: E402
from dataset.kitti import make_open3d_point_cloud, sphericity_based_voxel_analysis  # noqa: E402
from models.BUFFERX import BufferX  # noqa: E402
import open3d as o3d  # noqa: E402


def item(cfg, xyz0, xyz1):
    """Test-split branch of dataset/kitti.py KITTIDataset.__getitem__ (relt_pose = I)."""
    src_pcd = make_open3d_point_cloud(xyz0, [1, 0.706, 0])
    tgt_pcd = make_open3d_point_cloud(xyz1, [0, 0.651, 0.929])
    cfg.data.downsample, sphericity, _ = sphericity_based_voxel_analysis(src_pcd, tgt_pcd)
    is_aligned_to_global_z = cfg.patch.is_aligned_to_global_z
    src_pcd = o3d.geometry.PointCloud.voxel_down_sample(src_pcd, voxel_size=cfg.data.downsample)
    src_pts = np.array(src_pcd.points)
    np.random.shuffle(src_pts)
    tgt_pcd = o3d.geometry.PointCloud.voxel_down_sample(tgt_pcd, voxel_size=cfg.data.downsample)
    tgt_pts = np.array(tgt_pcd.points)
    np.random.shuffle(tgt_pts)
    ds = cfg.data.voxel_size_0
    src_kpt = np.array(o3d.geometry.PointCloud.voxel_down_sample(src_pcd, voxel_size=ds).points)
    np.random.shuffle(src_kpt)
    tgt_kpt = np.array(o3d.geometry.PointCloud.voxel_down_sample(tgt_pcd, voxel_size=ds).points)
    np.random.shuffle(tgt_kpt)
    if src_kpt.shape[0] > cfg.data.max_numPts:
        src_kpt = src_kpt[np.random.choice(range(src_kpt.shape[0]), cfg.data.max_numPts, replace=False)]
    if tgt_kpt.shape[0] > cfg.data.max_numPts:
        tgt_kpt = tgt_kpt[np.random.choice(range(tgt_kpt.shape[0]), cfg.data.max_numPts, replace=False)]
    return {"src_fds_pts": src_pts, "tgt_fds_pts": tgt_pts, "relt_pose": np.eye(4),
            "src_sds_pts": src_kpt, "tgt_sds_pts": tgt_kpt, "voxel_size": ds,
            "src_id": "0_0", "tgt_id": "0_1", "dataset_name": cfg.data.dataset,
            "sphericity": sphericity, "is_aligned_to_global_z": is_aligned_to_global_z}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bins", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--experiment", default="kitti")
    ap.add_argument("--max-scans", type=int)
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()
    random.seed(args.seed); np.random.seed(args.seed); torch.manual_seed(args.seed)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    cfg = make_cfg("KITTI", "/opt/bufferx/datasets")
    cfg[cfg.data.dataset] = cfg.copy()
    cfg.stage = "test"
    model = BufferX(cfg)
    for stage in cfg.train.all_stage:
        sd = torch.load(f"/opt/bufferx/snapshot/{args.experiment}/{stage}/best.pth", map_location="cuda:0")
        md = model.state_dict()
        md.update({k: v for k, v in sd.items() if stage in k})
        model.load_state_dict(md)
    model = nn.DataParallel(model.to("cuda:0"), device_ids=[0]).eval()

    files = sorted(Path(args.bins).glob("*.bin"))[: args.max_scans]
    load = lambda f: np.fromfile(f, dtype=np.float32).reshape(-1, 4)[:, :3].astype(np.float64)
    T_w = np.eye(4)
    poses = [T_w[:3, :].reshape(-1)]
    prev, dts, fails = load(files[0]), [], 0
    for k in range(1, len(files)):
        cur = load(files[k])
        batch = collate_fn_descriptor([item(cfg, prev, cur)], cfg)
        t0 = time.perf_counter()
        with torch.no_grad():
            T = model(batch)[0]
        torch.cuda.synchronize()
        dts.append(time.perf_counter() - t0)
        if T is None:
            T, fails = np.eye(4), fails + 1
        T = T.cpu().numpy() if torch.is_tensor(T) else np.asarray(T)
        T_w = T_w @ np.linalg.inv(T)
        poses.append(T_w[:3, :].reshape(-1))
        prev = cur
        if k % 250 == 0:
            print(f"[bufferx] scan {k}/{len(files)}  pos {T_w[:3, 3].round(1)}  "
                  f"{1000 * np.mean(dts[-250:]):.0f} ms/pair  failures {fails}", flush=True)
    np.savetxt(out / "bufferx_poses_kitti.txt", np.array(poses), fmt="%.9f")
    dts = np.array(dts)
    timing = {"method": "bufferx", "n_scans": len(poses), "ms_per_scan_mean": round(1000 * dts.mean(), 2),
              "ms_per_scan_median": round(1000 * np.median(dts), 2), "fps": round(1 / dts.mean(), 1),
              "registration_failures": fails, "checkpoint": f"snapshot/{args.experiment}"}
    (out / "timing.json").write_text(json.dumps(timing, indent=2))
    print(timing)


if __name__ == "__main__":
    main()
