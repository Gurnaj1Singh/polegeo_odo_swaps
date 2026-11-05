"""Export a scan cache as a KITTI-odometry style sequence for the learned methods
(DeepPointMap, Hi-LOAM, UGP, BUFFER-X): <out>/NNNNNN.bin, float32 (x, y, z, remission).

Points are exactly those of cache/scans (same 2-150 m filter, same order); remission is the
8-bit calibrated reflectivity in cache/scans_intensity scaled to [0, 1] like KITTI's.
Frame index k corresponds to row k of cache/scan_stamps.csv.

  python scripts/09_export_kitti_bins.py --cache cache --out cache/kitti_bins
"""
import argparse
from pathlib import Path

import numpy as np
import pandas as pd


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cache", default="cache")
    ap.add_argument("--out", default="cache/kitti_bins")
    args = ap.parse_args()
    cache, out = Path(args.cache), Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    stamps = pd.read_csv(cache / "scan_stamps.csv")
    for k, idx in enumerate(stamps.idx.astype(int)):
        xyz = np.load(cache / "scans" / f"{idx:06d}.npy")[:, :3]
        rem = np.load(cache / "scans_intensity" / f"{idx:06d}.npy").astype(np.float32) / 255.0
        np.c_[xyz, rem].astype(np.float32).tofile(out / f"{k:06d}.bin")
    print(f"wrote {len(stamps)} KITTI .bin frames -> {out}")


if __name__ == "__main__":
    main()
