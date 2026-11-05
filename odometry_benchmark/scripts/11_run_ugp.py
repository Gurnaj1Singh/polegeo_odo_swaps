"""UGP (Pang et al., CVPR 2025 highlight) as LiDAR odometry over consecutive SnowPole scans.

Runs in the ugp:cu128 image:
  docker run --rm --gpus all -v D:/Odometry/odometry_benchmark:/work ugp:cu128 \
      python /work/scripts/11_run_ugp.py --bins /work/cache/kitti_bins --out /work/outputs/ugp

Follows the repository's KITTI test path (experiments/UGP.kitti): points voxel-downsampled
at 0.3 m (data/Kitti/downsample_pcd.py), ref = scan k, src = scan k+1, all-ones features,
registration_collate_fn_stack_mode_full via build_dataloader_stack_mode, create_model(cfg) with
the shipped KITTI checkpoint, test.point_limit = None. Neighbour limits are calibrated with
the repository's calibrate_neighbors_stack_mode on these scans (the original calibrates on
KITTI training pairs, which are not part of this benchmark). estimated_transform maps src
into ref, so T_w,k+1 = T_w,k @ T. Seeds fixed.

Input range is capped at 80 m (--max-range), the reach of the HDL-64 KITTI scans the checkpoint was
trained on. At the shared 150 m the superpoint positional embedding (its memory grows with the scan's
extent, not its density) exceeds the 8 GB GPU on the open stretch around scans 3700-4500. A pair that
still does not fit falls back to the levels in FALLBACKS; such pairs are listed in timing.json.
"""
import argparse
import json
import random
import sys
import time
from pathlib import Path

import numpy as np
import open3d as o3d
import torch

sys.path.insert(0, "/opt/ugp/experiments/UGP.kitti")
sys.path.insert(0, "/opt/ugp")
from config import make_cfg  # noqa: E402
from model import create_model  # noqa: E402
from ugp.utils.data import build_dataloader_stack_mode, calibrate_neighbors_stack_mode  # noqa: E402
from ugp.utils.preprocess_utils import registration_collate_fn_stack_mode_full  # noqa: E402
from ugp.utils.torch import release_cuda, to_cuda  # noqa: E402


class ConsecutivePairs(torch.utils.data.Dataset):
    def __init__(self, files, voxel=0.3, max_range=None):
        self.files = files
        self.voxel = voxel
        self.max_range = max_range
        self.cache = {}

    def points(self, k):
        if k not in self.cache:
            xyz = np.fromfile(self.files[k], dtype=np.float32).reshape(-1, 4)[:, :3]
            if self.max_range:
                xyz = xyz[np.linalg.norm(xyz, axis=1) <= self.max_range]
            pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(xyz.astype(np.float64)))
            self.cache = {k: np.asarray(pcd.voxel_down_sample(self.voxel).points).astype(np.float32)}
        return self.cache[k]

    def __len__(self):
        return len(self.files) - 1

    def __getitem__(self, i):
        ref, src = self.points(i).copy(), self.points(i + 1).copy()
        return {"seq_id": 0, "ref_frame": i, "src_frame": i + 1,
                "ref_points": ref, "src_points": src,
                "ref_feats": np.ones((len(ref), 1), np.float32), "src_feats": np.ones((len(src), 1), np.float32),
                "transform": np.eye(4, dtype=np.float32)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bins", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--checkpoint", default="/opt/ugp/ckpts/kitti.pth.tar")
    ap.add_argument("--max-scans", type=int)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--max-range", type=float, default=80.0, help="input range cap in m (KITTI HDL-64 reach)")
    ap.add_argument("--resume", action="store_true", help="continue from <out>/ugp_partial.npz after a crash")
    args = ap.parse_args()
    random.seed(args.seed); np.random.seed(args.seed); torch.manual_seed(args.seed)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    cfg = make_cfg()
    files = sorted(Path(args.bins).glob("*.bin"))[: args.max_scans]
    ds = ConsecutivePairs(files, max_range=args.max_range)
    partial = out / "ugp_partial.npz"
    if args.resume and partial.exists():
        z = np.load(partial)
        poses, dts, limits = list(z["poses"]), list(z["dts"]), z["limits"]
        coarse = [list(c) + [0.0] * (3 - len(c)) for c in z["coarse"]]  # [pair, voxel m, max range m (0 = none)]
        print(f"[ugp] resuming after pair {len(poses) - 1} (neighbour limits {limits})", flush=True)
    else:
        poses, dts, coarse = [np.eye(4)[:3, :].reshape(-1)], [], []
        limits = calibrate_neighbors_stack_mode(ds, registration_collate_fn_stack_mode_full, cfg.backbone.num_stages,
                                                cfg.backbone.init_voxel_size, cfg.backbone.init_radius)
        print(f"[ugp] neighbour limits calibrated on SnowPole pairs: {limits}", flush=True)
    start = len(poses) - 1  # pairs already registered
    loader = build_dataloader_stack_mode(torch.utils.data.Subset(ds, range(start, len(ds))),
                                         registration_collate_fn_stack_mode_full, cfg.backbone.num_stages,
                                         cfg.backbone.init_voxel_size, cfg.backbone.init_radius, limits,
                                         batch_size=1, num_workers=0, shuffle=False, distributed=False,
                                         phase="test", max_points=cfg.max_points)
    model = create_model(cfg).cuda()
    state = torch.load(args.checkpoint, map_location="cpu")
    model.load_state_dict(state["model"] if "model" in state else state)
    model.eval()

    def register(data):
        t0 = time.perf_counter()
        with torch.no_grad():
            T = release_cuda(model(to_cuda(data))["estimated_transform"])
        torch.cuda.synchronize()
        return T, time.perf_counter() - t0

    # (voxel m, max range m) tried in order for a pair the 8 GB GPU cannot take at 0.3 m / --max-range
    FALLBACKS = ((0.45, args.max_range), (0.3, 60.0), (0.45, 40.0))

    def save(bad_pair=-1, bad_level=0):
        np.savez(partial, poses=np.array(poses), dts=np.array(dts), coarse=np.array(coarse, dtype=float).reshape(-1, 3),
                 limits=np.asarray(limits), bad_pair=bad_pair, bad_level=bad_level)

    def driver_fault(k, e, level):
        # after "CUDA driver error: device not ready" the caching allocator is unusable: save and let a
        # fresh process (new CUDA context) continue with this pair at the next voxel size
        save(bad_pair=k - 1, bad_level=level)
        print(f"[ugp] pair {k - 1}->{k}: {e}; checkpoint saved, exiting for a fresh CUDA context", flush=True)
        sys.exit(3)

    def register_coarse(k, data, reason, level=0):
        # a few dense pairs exceed the 8 GB GPU at 0.3 m; that pair is registered at a coarser voxel size
        for i in range(level, len(FALLBACKS)):
            voxel, max_range = FALLBACKS[i]
            del data
            torch.cuda.empty_cache()
            data = registration_collate_fn_stack_mode_full(
                [ConsecutivePairs(files, voxel, max_range)[k - 1]], cfg.backbone.num_stages, cfg.backbone.init_voxel_size,
                cfg.backbone.init_radius, limits, max_points=cfg.max_points, phase="test")
            try:
                T, dt = register(data)
                coarse.append([k - 1, voxel, max_range or 0.0])
                print(f"[ugp] pair {k - 1}->{k}: {reason} at 0.3 m, registered at {voxel} m voxel"
                      + (f", range <= {max_range:.0f} m" if max_range else ""), flush=True)
                return T, dt
            except torch.OutOfMemoryError:
                continue
            except RuntimeError as e:
                if "CUDA" not in str(e) or i + 1 == len(FALLBACKS):
                    raise
                driver_fault(k, e, i + 1)
        raise RuntimeError(f"pair {k - 1}->{k} fails at every voxel size")

    # a pair that broke the CUDA context in the previous process is registered coarse in this fresh process,
    # starting at the voxel size recorded with it
    resumed = args.resume and partial.exists()
    bad_pair = int(z["bad_pair"]) if resumed and "bad_pair" in z.files else -1
    bad_level = int(z["bad_level"]) if resumed and "bad_level" in z.files else 0
    T_w = np.eye(4)
    T_w[:3, :] = np.asarray(poses[-1]).reshape(3, 4)
    for k, data in enumerate(loader, start + 1):
        if k - 1 == bad_pair:
            T, dt = register_coarse(k, data, "CUDA driver error", bad_level)
        else:
            try:
                T, dt = register(data)
            except torch.OutOfMemoryError:
                T, dt = register_coarse(k, data, "GPU out of memory")
            except RuntimeError as e:
                if "CUDA" not in str(e):
                    raise
                driver_fault(k, e, 0)
        dts.append(dt)
        T_w = T_w @ np.asarray(T, dtype=np.float64)
        poses.append(T_w[:3, :].reshape(-1))
        if k % 250 == 0:
            print(f"[ugp] scan {k}/{len(files)}  pos {T_w[:3, 3].round(1)}  {1000 * np.mean(dts[-250:]):.0f} ms/pair",
                  flush=True)
            save()
    np.savetxt(out / "ugp_poses_kitti.txt", np.array(poses), fmt="%.9f")
    dts = np.array(dts)
    timing = {"method": "ugp", "n_scans": len(poses), "ms_per_scan_mean": round(1000 * dts.mean(), 2),
              "ms_per_scan_median": round(1000 * np.median(dts), 2), "fps": round(1 / dts.mean(), 1),
              "neighbor_limits": [int(x) for x in limits], "checkpoint": args.checkpoint,
              "fallback_pairs [pair, voxel_m, max_range_m (0 = full)]": coarse}
    (out / "timing.json").write_text(json.dumps(timing, indent=2))
    print(timing)


if __name__ == "__main__":
    main()
