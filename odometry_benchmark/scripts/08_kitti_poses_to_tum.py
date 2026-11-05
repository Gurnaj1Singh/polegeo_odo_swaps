"""Convert a KITTI-format trajectory (one 3x4 row-major pose per line, LiDAR frame of the
first scan) to TUM with the scan cache's sensor-clock stamps, so methods that do not stamp
their own poses (PIN-SLAM, DeepPointMap, Hi-LOAM) go through the same bridge as the rest.

  python scripts/08_kitti_poses_to_tum.py --poses odom_poses_kitti.txt --out pin_slam_tum.txt
  python scripts/08_kitti_poses_to_tum.py --poses trajectory.allframes.txt \
      --frame-ids trajectory.allsteps.txt --out deeppointmap_tum.txt

Pose k belongs to scan k of cache/scan_stamps.csv unless --frame-ids gives the frame index of
each pose (DeepPointMap writes one).
"""
import argparse
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--poses", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--frame-ids")
    ap.add_argument("--cache", default="cache")
    ap.add_argument("--project-rotations", action="store_true",
                    help="project invalid rotation matrices (e.g. reflections) to the nearest rotation; positions kept")
    args = ap.parse_args()

    P = np.loadtxt(args.poses)
    P = P.reshape(-1, 12) if P.ndim == 1 else P
    stamps = pd.read_csv(Path(args.cache) / "scan_stamps.csv").sensor_stamp.to_numpy()
    ids = np.loadtxt(args.frame_ids, dtype=int).reshape(-1) if args.frame_ids else np.arange(len(P))
    if len(ids) != len(P):
        raise SystemExit(f"{len(P)} poses but {len(ids)} frame ids")
    if ids.max() >= len(stamps):
        raise SystemExit(f"frame id {ids.max()} beyond {len(stamps)} scans")
    T = P.reshape(-1, 3, 4)
    R = T[:, :, :3].copy()
    if args.project_rotations:
        # Keep the method's positions exactly; only make each stored orientation a valid rotation
        # (nearest rotation, sign-corrected) so it can be written as a quaternion. Scoring uses
        # positions only (02_bridge_to_csv.py reads x, y, z), so this does not change any score.
        U, _, Vt = np.linalg.svd(R)
        d = np.sign(np.linalg.det(U @ Vt))
        n_bad = int((np.linalg.det(R) < 0).sum())
        U[:, :, 2] *= d[:, None]  # flip the last axis where needed so det = +1
        R = U @ Vt
        print(f"[project] {n_bad} poses had a reflection (det < 0) in the method's output; "
              f"orientations projected to the nearest rotation, positions unchanged")
    q = Rotation.from_matrix(R).as_quat()
    out = np.c_[stamps[ids], T[:, :, 3], q]
    np.savetxt(args.out, out, fmt="%.9f")
    print(f"{len(P)} poses -> {args.out}  (frames {ids.min()}..{ids.max()} of {len(stamps)})")


if __name__ == "__main__":
    main()
