"""Collect a run's scores into one table and compare it with the frozen reference.

  python scripts/05_check_results.py --run outputs_rerun            # check vs results/expected_scores.csv
  python scripts/05_check_results.py --run outputs --freeze         # (re)write the expected file
  python scripts/05_check_results.py --run outputs_rerun --traj-ref outputs   # also diff trajectories

Exit code 0 only if every (method, seed, GNSS %) median matches to the reported
precision (0.01 m).
"""
import argparse
import sys
from pathlib import Path

import numpy as np
import pandas as pd

HERE = Path(__file__).resolve().parent.parent
EXPECTED = HERE / "results" / "expected_scores.csv"
COLS = ["method", "seed", "gnss_pct", "err_pred_median", "err_pred_mean", "err_odom_median"]


def collect(run):
    rows = []
    for f in sorted((Path(run) / "scores").glob("*_seed*.csv")):
        d = pd.read_csv(f)
        d["seed"] = int(f.stem.rsplit("_seed", 1)[1])
        rows.append(d)
    if not rows:
        sys.exit(f"no *_seed*.csv score files under {run}/scores")
    return pd.concat(rows)[COLS].sort_values(["method", "seed", "gnss_pct"]).reset_index(drop=True)


def traj_diff(run, ref):
    for name in ["rko_lio/rko_lio_0/rko_lio_0_tum.txt", "genz_icp/genz_icp_tum.txt",
                 "mad_icp/mad_icp_tum.txt", "kiss_icp/kiss_icp_tum.txt"]:
        a, b = Path(run) / name, Path(ref) / name
        if not (a.exists() and b.exists()):
            print(f"[traj] {name}: missing in one run, skipped")
            continue
        A, B = np.loadtxt(a), np.loadtxt(b)
        n = min(len(A), len(B))
        d = np.linalg.norm(A[:n, 1:4] - B[:n, 1:4], axis=1)
        print(f"[traj] {name}: {len(A)} vs {len(B)} poses, position diff max {d.max():.6f} m, "
              f"final {d[-1]:.6f} m")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", required=True)
    ap.add_argument("--freeze", action="store_true")
    ap.add_argument("--traj-ref")
    args = ap.parse_args()

    got = collect(args.run)
    table = got[got.seed == 0].pivot(index="gnss_pct", columns="method", values="err_pred_median")
    print(table.to_string(float_format=lambda v: f"{v:.2f}"))
    if args.traj_ref:
        traj_diff(args.run, args.traj_ref)
    if args.freeze:
        EXPECTED.parent.mkdir(exist_ok=True)
        got.to_csv(EXPECTED, index=False)
        print(f"froze {len(got)} rows -> {EXPECTED}")
        return

    exp = pd.read_csv(EXPECTED)
    m = exp.merge(got, on=["method", "seed", "gnss_pct"], how="left", suffixes=("_exp", "_got"))
    missing = m.err_pred_median_got.isna()
    diff = (m.err_pred_median_got - m.err_pred_median_exp).abs()
    bad = m[missing | (diff > 0.005)]
    print(f"\nchecked {len(m)} expected medians: {len(m) - len(bad)} match, {len(bad)} differ/missing")
    if len(bad):
        print(bad[["method", "seed", "gnss_pct", "err_pred_median_exp", "err_pred_median_got"]].to_string())
        sys.exit(1)
    print("REPRODUCED: all medians match the published table.")


if __name__ == "__main__":
    main()
