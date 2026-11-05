"""Compare two cached pole-detection runs over the SnowPole bag (03_cache_detections.py outputs).

  python scripts/19_compare_detections.py cache/detections.pkl cache_comb4/detections.pkl > results/...md

Per run: detections per frame, overall and split into the frames that are (844-2999) / are not in the
SnowPole Detection dataset the Combination-4 detector was trained and tested on.
"""
import pickle
import sys
from pathlib import Path

import numpy as np

DATASET_FRAMES = (844, 2999)  # results/detector_dataset_frame_map.csv


def stats(frames, sel):
    n = np.array([len(frames[i]) for i in sel])
    d = np.array([b[1] for i in sel for b in frames[i]] or [np.nan])
    return (f"{len(n)} | {int((n > 0).sum())} ({100 * (n > 0).mean():.1f} %) | {int(n.sum())} | {n.mean():.2f} | "
            f"{np.nanmedian(d):.1f} | {int((d <= 5).sum())}")


def main():
    runs = [(Path(p), pickle.load(open(p, "rb"))["frames"]) for p in sys.argv[1:]]
    n = len(runs[0][1])
    lo, hi = DATASET_FRAMES
    parts = {"all frames": range(1, n), f"in detector dataset ({lo}-{hi})": range(lo, hi + 1),
             "outside detector dataset": [i for i in range(1, n) if not lo <= i <= hi]}
    print("# Pole detections over the SnowPole drive\n")
    print("Frame 0 is skipped by the pipeline. Distance = LiDAR distance to the box's nearest point "
          "(boxes <= 5 m are later dropped by the sweep's gate).\n")
    print("| Run | Frames | Frames with >= 1 box | Boxes | Boxes/frame | Median distance m | Boxes <= 5 m |")
    print("|---|---|---|---|---|---|---|")
    for name, sel in parts.items():
        for p, fr in runs:
            print(f"| {p.parent.name}: {name} | {stats(fr, sel)} |")


if __name__ == "__main__":
    main()
